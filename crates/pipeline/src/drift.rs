//! Clock drift estimation and compensation.
//!
//! Provides:
//! - Primary estimator: calculates source sample rate ppm offset from (device position,
//!   QPC/monotonic timestamp) pairs supplied by `AudioChunk`, low-pass filtered.
//! - Fallback estimator: Proportional-Integral (PI) controller operating on ring-buffer
//!   fill level to prevent buffer underrun/overrun.
//! - Hard clamping: drift correction strictly clamped to `[-500.0, 500.0]` ppm.

use std::time::{Duration, Instant};

/// Maximum allowable drift correction in parts per million (±500 ppm).
pub const MAX_DRIFT_CORRECTION_PPM: f64 = 500.0;

/// A chunk starting further than this from where the previous one ended is a gap in the
/// timeline (audio lost, or none delivered), not drift. Derived arrival timestamps jitter by
/// about one 10 ms capture period; drift moves them by microseconds per chunk.
const TIMELINE_GAP: Duration = Duration::from_millis(50);

/// Configuration parameters for drift compensation.
#[derive(Debug, Clone)]
pub struct DriftConfig {
    /// Nominal input sample rate in Hz (e.g. 48000 or 44100).
    pub nominal_sample_rate: u32,
    /// Target buffer fill level (in frames or items).
    pub target_fill_level: usize,
    /// Low-pass filter smoothing factor (0.0 < alpha <= 1.0) for primary estimator.
    /// Alpha = 0.05 corresponds to ~20 chunk time constant.
    pub filter_alpha: f64,
    /// Proportional gain Kp for fallback PI controller.
    pub kp: f64,
    /// Integral gain Ki for fallback PI controller.
    pub ki: f64,
}

impl Default for DriftConfig {
    fn default() -> Self {
        Self {
            nominal_sample_rate: 48000,
            target_fill_level: 16, // 16 packets of 352 frames ≈ 5632 frames ≈ 128 ms
            filter_alpha: 0.05,
            kp: 200.0,
            ki: 50.0,
        }
    }
}

/// Estimates and compensates for clock drift between capture device and network timeline.
#[derive(Debug)]
pub struct DriftCompensator {
    config: DriftConfig,
    /// Frames delivered by the source between `anchor` and the start of the latest chunk.
    total_source_frames: u64,
    /// First timestamp recorded.
    start_time: Option<Instant>,
    /// Where the primary estimate's timeline starts, moved by every gap in it so that time
    /// without audio is not counted as a slow clock.
    anchor: Option<Instant>,
    /// Where the latest chunk ends on its timeline.
    next_expected: Option<Instant>,
    /// Frames in the latest chunk, added to the total when the next one arrives.
    pending_frames: u64,
    /// Last timestamp recorded.
    last_time: Option<Instant>,
    /// Low-pass filtered estimated drift in ppm.
    filtered_drift_ppm: f64,
    /// Fallback PI controller integrator state.
    integral_error: f64,
    /// Last applied correction in ppm.
    applied_correction_ppm: f64,
}

impl DriftCompensator {
    /// Create a new DriftCompensator with given configuration.
    pub fn new(config: DriftConfig) -> Self {
        Self {
            config,
            total_source_frames: 0,
            start_time: None,
            anchor: None,
            next_expected: None,
            pending_frames: 0,
            last_time: None,
            filtered_drift_ppm: 0.0,
            integral_error: 0.0,
            applied_correction_ppm: 0.0,
        }
    }

    /// Reset internal state.
    pub fn reset(&mut self) {
        self.total_source_frames = 0;
        self.start_time = None;
        self.anchor = None;
        self.next_expected = None;
        self.pending_frames = 0;
        self.last_time = None;
        self.filtered_drift_ppm = 0.0;
        self.integral_error = 0.0;
        self.applied_correction_ppm = 0.0;
    }

    /// Return the current filtered drift estimate in ppm.
    pub fn estimated_drift_ppm(&self) -> f64 {
        self.filtered_drift_ppm
    }

    /// Return the last applied correction in ppm.
    pub fn applied_correction_ppm(&self) -> f64 {
        self.applied_correction_ppm
    }

    /// Update drift estimate using incoming `AudioChunk` timestamp and frame count.
    ///
    /// # Primary Method: Device Position & QPC Timestamp
    /// Compares cumulative audio frames against elapsed monotonic time:
    /// `elapsed_nominal_secs = total_frames / nominal_rate`
    /// `actual_rate = total_frames / elapsed_real_secs`
    /// `drift_ppm = ((actual_rate - nominal_rate) / nominal_rate) * 1e6`
    ///
    /// Low-pass filtered via exponential moving average:
    /// `filtered_ppm = (1 - alpha) * filtered_ppm + alpha * raw_ppm`
    ///
    /// A gap in the timeline (a chunk starting more than [`TIMELINE_GAP`] away from where the
    /// previous one ended: audio dropped while the capture buffer was full, or a pause with
    /// nothing delivered) moves the anchor by the gap instead of counting it as drift. Counted,
    /// a 250 ms loss at startup read as -500 ppm for minutes.
    pub fn update_primary(&mut self, chunk_frames: usize, timestamp: Instant) -> f64 {
        let chunk_duration =
            Duration::from_secs_f64(chunk_frames as f64 / self.config.nominal_sample_rate as f64);
        let (Some(anchor), Some(expected)) = (self.anchor, self.next_expected) else {
            self.start_time = Some(timestamp);
            self.anchor = Some(timestamp);
            self.last_time = Some(timestamp);
            self.next_expected = Some(timestamp + chunk_duration);
            self.total_source_frames = 0;
            self.pending_frames = chunk_frames as u64;
            return self.filtered_drift_ppm;
        };

        let start = if timestamp > expected + TIMELINE_GAP {
            anchor + (timestamp - expected)
        } else if timestamp + TIMELINE_GAP < expected {
            anchor.checked_sub(expected - timestamp).unwrap_or(anchor)
        } else {
            anchor
        };
        self.anchor = Some(start);
        self.last_time = Some(timestamp);
        self.next_expected = Some(timestamp + chunk_duration);
        self.total_source_frames += self.pending_frames;
        self.pending_frames = chunk_frames as u64;

        if timestamp <= start {
            return self.filtered_drift_ppm;
        }

        let elapsed = timestamp.duration_since(start).as_secs_f64();
        // Wait at least 200 ms for initial settling to avoid small denominator noise
        if elapsed < 0.2 {
            return self.filtered_drift_ppm;
        }

        let nominal_rate = self.config.nominal_sample_rate as f64;
        let measured_rate = (self.total_source_frames as f64) / elapsed;
        let raw_drift_ppm = ((measured_rate - nominal_rate) / nominal_rate) * 1_000_000.0;

        // Apply low-pass filter
        let alpha = self.config.filter_alpha;
        self.filtered_drift_ppm = (1.0 - alpha) * self.filtered_drift_ppm + alpha * raw_drift_ppm;

        self.filtered_drift_ppm
    }

    /// The fill level (packets) the fallback PI controller steers towards.
    pub fn target_fill_level(&self) -> usize {
        self.config.target_fill_level
    }

    /// Changes the fill level the fallback PI controller steers towards (e.g. 2 packets for
    /// live sources whose backlog is trimmed at 3 packets by the sender).
    pub fn set_target_fill_level(&mut self, packets: usize) {
        self.config.target_fill_level = packets;
        self.integral_error = 0.0;
    }

    /// Update fallback PI controller based on current ring-buffer fill level.
    ///
    /// Returns the PI controller's suggested drift compensation in ppm.
    pub fn update_fallback_pi(&mut self, current_fill: usize, dt: Duration) -> f64 {
        let dt_secs = dt.as_secs_f64().max(0.001);
        let target = self.config.target_fill_level as f64;
        if target <= 0.0 {
            return 0.0;
        }

        // Positive error means buffer is filling up (source delivering faster than consumer)
        let error = (current_fill as f64 - target) / target;

        // Integral update with anti-windup clamping to [-1.0, 1.0]
        self.integral_error = (self.integral_error + error * dt_secs).clamp(-1.0, 1.0);

        let p_term = self.config.kp * error;
        let i_term = self.config.ki * self.integral_error;

        p_term + i_term
    }

    /// Compute the combined drift correction in ppm, clamped to `[-500.0, 500.0]`.
    ///
    /// If primary estimation is active and has settled, it takes precedence;
    /// otherwise, the fallback PI controller provides the correction.
    pub fn compute_correction(
        &mut self,
        chunk_frames: usize,
        timestamp: Instant,
        current_fill: usize,
        dt: Duration,
    ) -> f64 {
        let primary_ppm = self.update_primary(chunk_frames, timestamp);
        let pi_ppm = self.update_fallback_pi(current_fill, dt);

        // If primary has run for at least 1.0 second, use primary + small PI trimming;
        // otherwise rely primarily on PI.
        let elapsed = self
            .start_time
            .map(|t| timestamp.saturating_duration_since(t).as_secs_f64())
            .unwrap_or(0.0);

        let combined_ppm = if elapsed >= 1.0 {
            // Primary feedforward with small PI feedback to maintain target buffer level
            primary_ppm + 0.2 * pi_ppm
        } else {
            pi_ppm
        };

        // Strictly clamp to [-500.0, 500.0] ppm
        let clamped = combined_ppm.clamp(-MAX_DRIFT_CORRECTION_PPM, MAX_DRIFT_CORRECTION_PPM);
        self.applied_correction_ppm = clamped;
        clamped
    }

    /// Convert drift correction ppm to relative resampler ratio multiplier.
    ///
    /// When source runs faster (+ppm), ratio is decreased (`< 1.0`) so fewer output
    /// samples are generated per input sample.
    /// When source runs slower (-ppm), ratio is increased (`> 1.0`).
    #[inline]
    pub fn ppm_to_relative_ratio(ppm: f64) -> f64 {
        1.0 / (1.0 + ppm * 1e-6)
    }
}

/// Proportional gain of [`QueueController`]: ppm per second of queue error. With the
/// critically damped integral below, the loop's time constant is `2 / (KP * 1e-6)` ≈ 33 s,
/// slow enough to ignore arrival bursts.
const QUEUE_KP: f64 = 60_000.0;
/// Integral gain of [`QueueController`] (ppm per second² of queue error): `KP² / 4` in ppm
/// units, which damps the loop critically.
const QUEUE_KI: f64 = QUEUE_KP * QUEUE_KP * 1e-6 / 4.0;
/// Smoothing time of the measured queue level.
const QUEUE_FILTER_SECS: f64 = 1.0;
/// After playback (re)starts, the queue level is averaged this long to find the level to hold.
const QUEUE_SETTLE_SECS: f64 = 2.0;

/// Steers the resampler from the fill of a source's own jitter queue, for audio that arrives
/// on another device's clock but is read on demand (live stdin).
///
/// The queue grows when the producer runs faster than the receiver consumes it, so a PI loop on
/// the queue error (in seconds) settles on the clock difference. Its integral term is that
/// difference, reported as the estimated drift.
///
/// The level held is the average over the first seconds after playback (re)starts, not the
/// source's buffering target: bursty delivery leaves the average below the level playback
/// started at, and steering that offset away is not drift correction (near the ppm limit it
/// takes minutes).
#[derive(Debug, Default)]
pub struct QueueController {
    /// Low-pass filtered queue level in frames; `None` until the first level after a reset.
    filtered_frames: Option<f64>,
    /// Level held, in frames; `None` while settling.
    setpoint: Option<f64>,
    /// Sum of level × seconds and seconds seen while settling.
    settle: (f64, f64),
    /// Integral of the queue error, in seconds².
    integral: f64,
    applied_correction_ppm: f64,
}

impl QueueController {
    pub fn new() -> Self {
        Self::default()
    }

    /// Update with the queue level in frames after `dt` of audio at `sample_rate` and return
    /// the correction in ppm (positive when the producer is faster). Without a level (the
    /// source is buffering) and while settling after it, the estimated drift alone is applied.
    pub fn update(&mut self, level: Option<usize>, sample_rate: u32, dt: Duration) -> f64 {
        let dt_secs = dt.as_secs_f64();
        let frames = match (level, self.setpoint) {
            (None, _) => {
                self.filtered_frames = None;
                self.setpoint = None;
                self.settle = (0.0, 0.0);
                return self.hold_estimate();
            }
            (Some(frames), None) => {
                self.settle.0 += frames as f64 * dt_secs;
                self.settle.1 += dt_secs;
                if self.settle.1 >= QUEUE_SETTLE_SECS {
                    let average = self.settle.0 / self.settle.1;
                    self.setpoint = Some(average);
                    self.filtered_frames = Some(average);
                }
                return self.hold_estimate();
            }
            (Some(frames), Some(_)) => frames as f64,
        };
        let filtered = match self.filtered_frames {
            Some(f) => f + (frames - f) * (dt_secs / QUEUE_FILTER_SECS).min(1.0),
            None => frames,
        };
        self.filtered_frames = Some(filtered);

        let setpoint = self.setpoint.unwrap_or(filtered);
        let error_secs = (filtered - setpoint) / sample_rate.max(1) as f64;
        let integral = self.integral + error_secs * dt_secs;
        let unclamped = QUEUE_KP * error_secs + QUEUE_KI * integral;
        // Anti-windup: while the output is pinned at the limit, an error pushing further that
        // way is not integrated. Otherwise a long recovery at the limit (e.g. refilling after
        // playback starts) winds the integral far past the actual drift.
        if unclamped.abs() <= MAX_DRIFT_CORRECTION_PPM || unclamped.signum() != error_secs.signum()
        {
            let integral_max = MAX_DRIFT_CORRECTION_PPM / QUEUE_KI;
            self.integral = integral.clamp(-integral_max, integral_max);
        }
        self.applied_correction_ppm = (QUEUE_KP * error_secs + QUEUE_KI * self.integral)
            .clamp(-MAX_DRIFT_CORRECTION_PPM, MAX_DRIFT_CORRECTION_PPM);
        self.applied_correction_ppm
    }

    /// The clock difference settled on so far, in ppm.
    pub fn estimated_drift_ppm(&self) -> f64 {
        QUEUE_KI * self.integral
    }

    fn hold_estimate(&mut self) -> f64 {
        self.applied_correction_ppm = self
            .estimated_drift_ppm()
            .clamp(-MAX_DRIFT_CORRECTION_PPM, MAX_DRIFT_CORRECTION_PPM);
        self.applied_correction_ppm
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_drift_primary_positive_ppm() {
        let mut comp = DriftCompensator::new(DriftConfig {
            nominal_sample_rate: 48000,
            filter_alpha: 0.2,
            ..Default::default()
        });

        let start = Instant::now();
        // Simulate +200 ppm clock: source produces 48000 * 1.0002 = 48009.6 samples per second
        let nominal_chunk = 480; // 10 ms at 48000
        let real_interval = Duration::from_nanos((10_000_000.0 / 1.0002) as u64);

        let mut t = start;
        for _ in 0..200 {
            t += real_interval;
            comp.update_primary(nominal_chunk, t);
        }

        let est = comp.estimated_drift_ppm();
        // Should converge close to +200 ppm (within 10 ppm)
        assert!(
            (est - 200.0).abs() < 10.0,
            "Expected ~200 ppm, got {:.2} ppm",
            est
        );
    }

    #[test]
    fn test_drift_primary_negative_ppm() {
        let mut comp = DriftCompensator::new(DriftConfig {
            nominal_sample_rate: 48000,
            filter_alpha: 0.2,
            ..Default::default()
        });

        let start = Instant::now();
        // Simulate -200 ppm clock: source produces 48000 * 0.9998 = 47990.4 samples per second
        let nominal_chunk = 480;
        let real_interval = Duration::from_nanos((10_000_000.0 / 0.9998) as u64);

        let mut t = start;
        for _ in 0..200 {
            t += real_interval;
            comp.update_primary(nominal_chunk, t);
        }

        let est = comp.estimated_drift_ppm();
        assert!(
            (est - (-200.0)).abs() < 10.0,
            "Expected ~-200 ppm, got {:.2} ppm",
            est
        );
    }

    #[test]
    fn test_hard_clamp_pm500_ppm() {
        let mut comp = DriftCompensator::new(DriftConfig::default());
        let now = Instant::now();

        // High fill forces large PI correction
        let corr = comp.compute_correction(480, now, 1000, Duration::from_millis(50));
        assert!(
            corr <= MAX_DRIFT_CORRECTION_PPM,
            "Exceeded +500 ppm: {}",
            corr
        );

        // Low fill forces negative PI correction
        let corr_neg = comp.compute_correction(480, now, 0, Duration::from_millis(50));
        assert!(
            corr_neg >= -MAX_DRIFT_CORRECTION_PPM,
            "Exceeded -500 ppm: {}",
            corr_neg
        );
    }

    #[test]
    fn test_ppm_to_relative_ratio() {
        let r_zero = DriftCompensator::ppm_to_relative_ratio(0.0);
        assert!((r_zero - 1.0).abs() < 1e-12);

        let r_pos = DriftCompensator::ppm_to_relative_ratio(500.0);
        assert!(r_pos < 1.0);
        assert!((r_pos - (1.0 / 1.0005)).abs() < 1e-9);

        let r_neg = DriftCompensator::ppm_to_relative_ratio(-500.0);
        assert!(r_neg > 1.0);
        assert!((r_neg - (1.0 / 0.9995)).abs() < 1e-9);
    }

    #[test]
    fn queue_controller_speeds_up_when_the_queue_grows() {
        let dt = Duration::from_millis(10);
        let settled = || {
            let mut q = QueueController::new();
            for _ in 0..200 {
                assert_eq!(q.update(Some(4_410), 44_100, dt), 0.0);
            }
            q
        };
        let mut q = settled();
        assert!(q.update(Some(5_000), 44_100, dt) > 0.0);
        let mut q = settled();
        assert!(q.update(Some(4_000), 44_100, dt) < 0.0);
        // Buffering: only the estimated drift is applied.
        assert_eq!(q.update(None, 44_100, dt), q.estimated_drift_ppm());
    }

    /// Closed loop as live stdin runs it: a producer whose clock is off by `drift_ppm`
    /// delivers bursts with up to 30 ms of jitter, the pipeline reads 10 ms chunks and reports
    /// the level before each read, playback starts with the queue at its target, and a 2 s
    /// pause empties the queue once. The controller must settle on the drift without
    /// overshooting it, and hold the queue without running dry or overflowing.
    fn simulate_queue_loop(drift_ppm: f64) {
        let rate = 44_100.0;
        let target = 6_615.0; // 150 ms
        let chunk = 441.0;
        let dt = 0.01;
        let mut rng = crate::convert::Xorshift64::new(42);
        let mut q = QueueController::new();
        let mut level = target;
        let mut playing = true;
        let mut owed = 0.0; // produced but not yet delivered (held back by jitter)
        let mut next_burst = 0.0;
        let mut corr_ppm = 0.0;
        let (mut min_level, mut max_level) = (f64::MAX, 0.0f64);
        let (mut min_est, mut max_est) = (0.0f64, 0.0f64);
        for step in 0..60_000 {
            let t = step as f64 * dt;
            if !(300.0..302.0).contains(&t) {
                owed += rate * (1.0 + drift_ppm * 1e-6) * dt;
            }
            if t >= next_burst {
                level += owed;
                owed = 0.0;
                next_burst = t + 0.005 + (rng.next_uniform_sym() as f64 + 0.5) * 0.03;
            }
            if !playing && level >= target {
                level = target;
                playing = true;
            }
            if !playing || level < chunk {
                playing = false;
                corr_ppm = q.update(None, 44_100, Duration::from_secs_f64(dt));
                continue;
            }
            let measured = level;
            level -= chunk * (1.0 + corr_ppm * 1e-6);
            if !(300.0..310.0).contains(&t) {
                min_level = min_level.min(level);
                max_level = max_level.max(level);
            }
            corr_ppm = q.update(Some(measured as usize), 44_100, Duration::from_secs_f64(dt));
            min_est = min_est.min(q.estimated_drift_ppm());
            max_est = max_est.max(q.estimated_drift_ppm());
        }
        let (low, high) = (drift_ppm.min(0.0), drift_ppm.max(0.0));
        assert!(
            min_est > low - 30.0 && max_est < high + 30.0,
            "estimate swung over {min_est:.0}..{max_est:.0} ppm for {drift_ppm} ppm"
        );
        assert!(
            (q.estimated_drift_ppm() - drift_ppm).abs() < 5.0,
            "estimated {} ppm, actual {drift_ppm}",
            q.estimated_drift_ppm()
        );
        assert!(min_level > 0.0, "queue ran dry");
        assert!(max_level < target * 3.0, "queue overflowed: {max_level}");
    }

    #[test]
    fn queue_controller_follows_a_fast_producer() {
        simulate_queue_loop(100.0);
    }

    #[test]
    fn queue_controller_follows_a_slow_producer() {
        simulate_queue_loop(-300.0);
    }

    /// Feeds 10 ms chunks of a 48 kHz source running `drift_ppm` off for `secs`, skipping the
    /// chunks whose start falls in one of `lost` (start, end) windows, with timestamps off by
    /// up to ±`jitter_ms`.
    fn primary_run(
        drift_ppm: f64,
        secs: f64,
        lost: &[(f64, f64)],
        jitter_ms: f64,
    ) -> DriftCompensator {
        let mut comp = DriftCompensator::new(DriftConfig {
            nominal_sample_rate: 48000,
            ..Default::default()
        });
        let start = Instant::now();
        let chunk_secs = 0.01 / (1.0 + drift_ppm * 1e-6);
        let mut seed = 1u32;
        for k in 0..(secs / chunk_secs) as u64 {
            let t = k as f64 * chunk_secs;
            if lost.iter().any(|&(a, b)| t >= a && t < b) {
                continue;
            }
            seed = seed.wrapping_mul(1_664_525).wrapping_add(1_013_904_223);
            let jitter = (seed >> 8) as f64 / (1u32 << 24) as f64 * 2.0 - 1.0;
            let at = t + 0.5 + jitter * jitter_ms / 1000.0; // +0.5 s: keep the jitter positive
            comp.update_primary(480, start + Duration::from_secs_f64(at));
        }
        comp
    }

    fn primary_estimate(drift_ppm: f64, secs: f64, lost: &[(f64, f64)]) -> f64 {
        primary_run(drift_ppm, secs, lost, 0.0).estimated_drift_ppm()
    }

    #[test]
    fn audio_lost_at_startup_is_not_drift() {
        // Measured on Windows: the capture buffer overflowed while the AirPlay session was
        // still being set up and 250 ms were lost; the estimate then read -488 ppm after
        // 8 minutes for a source about 9 ppm fast.
        let est = primary_estimate(9.0, 30.0, &[(1.3, 1.55)]);
        assert!(
            (est - 9.0).abs() < 3.0,
            "estimate {est:.1} ppm, expected ~9"
        );
    }

    #[test]
    fn a_pause_without_audio_is_not_drift() {
        let est = primary_estimate(-670.0, 100.0, &[(20.0, 80.0)]);
        assert!(
            (est + 670.0).abs() < 3.0,
            "estimate {est:.1} ppm, expected ~-670"
        );
    }

    #[test]
    fn timestamp_jitter_is_not_a_gap() {
        // Arrival timestamps derived from the capture queue jitter by up to one period.
        let comp = primary_run(-670.0, 60.0, &[], 10.0);
        assert_eq!(comp.anchor, comp.start_time, "jitter moved the anchor");
    }
}
