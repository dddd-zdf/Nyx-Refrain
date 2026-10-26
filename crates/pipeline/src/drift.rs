//! Clock drift compensation by steering the sender queue's fill level.
//!
//! The capture clock and the send timer never tick at exactly the same rate, so the queue
//! between them slowly fills or drains. A PI controller on the (smoothed) fill level nudges
//! the resampler ratio to hold it at a target: the proportional term reacts to the current
//! offset, the integral term learns the steady clock difference.
//!
//! An earlier version fed forward a whole-session average of the capture rate. A few minutes
//! of uneven delivery (a paused player) skewed that average by ~300 ppm, and the fill term
//! was too weak to override it, so the queue overflowed every ~30 s for many minutes.
//! Steering on the fill alone cannot get stuck that way.
//!
//! Corrections are clamped to `[-1000.0, 1000.0]` ppm (under 2 cents of pitch). 500 ppm was
//! too tight: an LG monitor's HDMI audio clock runs ~670 ppm slow, so the queue drained past
//! the limit and ran dry every ~140 s.

use std::time::Duration;

/// Maximum allowable drift correction in parts per million (±1000 ppm).
pub const MAX_DRIFT_CORRECTION_PPM: f64 = 1000.0;

/// Packets of fill change per second per ppm of correction: 44.1 kHz output in 352-frame
/// packets, so 1 ppm moves `44100e-6 / 352` packets a second.
const PACKETS_PER_SEC_PER_PPM: f64 = 44_100e-6 / 352.0;

/// Configuration parameters for drift compensation.
#[derive(Debug, Clone)]
pub struct DriftConfig {
    /// Nominal input sample rate in Hz (e.g. 48000 or 44100); converts chunk sizes to time.
    pub nominal_sample_rate: u32,
    /// Target queue fill level in packets.
    pub target_fill_level: usize,
    /// Time constant (s) of the fill smoothing; removes the per-chunk sawtooth.
    pub fill_smoothing_secs: f64,
    /// Proportional gain, ppm per packet of fill error.
    pub kp: f64,
    /// Integral gain, ppm per packet-second of fill error.
    pub ki: f64,
}

impl Default for DriftConfig {
    fn default() -> Self {
        // Critically damped loop with ωn = 0.1 rad/s. The fill is an integrator of the
        // correction with gain PACKETS_PER_SEC_PER_PPM (g), so kp = 2ωn / g and ki = ωn² / g.
        let wn = 0.1;
        Self {
            nominal_sample_rate: 48000,
            target_fill_level: 16,
            fill_smoothing_secs: 1.0,
            kp: 2.0 * wn / PACKETS_PER_SEC_PER_PPM,
            ki: wn * wn / PACKETS_PER_SEC_PER_PPM,
        }
    }
}

/// Holds the sender queue at its target fill by nudging the resampling ratio.
#[derive(Debug)]
pub struct DriftCompensator {
    config: DriftConfig,
    /// Smoothed fill level in packets; `None` until the first update.
    smoothed_fill: Option<f64>,
    /// Integral of the fill error (packet-seconds).
    integral: f64,
    /// Last applied correction in ppm.
    applied_correction_ppm: f64,
}

impl DriftCompensator {
    /// Create a new DriftCompensator with given configuration.
    pub fn new(config: DriftConfig) -> Self {
        Self {
            config,
            smoothed_fill: None,
            integral: 0.0,
            applied_correction_ppm: 0.0,
        }
    }

    /// Reset internal state.
    pub fn reset(&mut self) {
        self.smoothed_fill = None;
        self.integral = 0.0;
        self.applied_correction_ppm = 0.0;
    }

    /// The learned steady clock difference in ppm (the integral term).
    pub fn estimated_drift_ppm(&self) -> f64 {
        self.config.ki * self.integral
    }

    /// Return the last applied correction in ppm.
    pub fn applied_correction_ppm(&self) -> f64 {
        self.applied_correction_ppm
    }

    /// The fill level (packets) the controller steers towards.
    pub fn target_fill_level(&self) -> usize {
        self.config.target_fill_level
    }

    /// Changes the fill level (packets) the controller steers towards.
    pub fn set_target_fill_level(&mut self, packets: usize) {
        self.config.target_fill_level = packets;
        self.integral = 0.0;
    }

    /// Restarts the fill smoothing at `fill`, after the queue was refilled in one step.
    pub fn refilled(&mut self, fill: usize) {
        self.smoothed_fill = Some(fill as f64);
    }

    /// Updates the controller after a chunk of `chunk_frames` source frames, with the queue
    /// holding `current_fill` packets, and returns the correction in ppm (positive = the
    /// queue is too full, produce slightly less output).
    pub fn compute_correction(&mut self, chunk_frames: usize, current_fill: usize) -> f64 {
        let dt = chunk_frames as f64 / self.config.nominal_sample_rate.max(1) as f64;
        let fill = current_fill as f64;
        let smoothed = match self.smoothed_fill {
            Some(prev) => {
                let a = (dt / self.config.fill_smoothing_secs.max(1e-3)).min(1.0);
                prev + a * (fill - prev)
            }
            None => fill,
        };
        self.smoothed_fill = Some(smoothed);

        let error = smoothed - self.config.target_fill_level as f64;
        let max_integral = MAX_DRIFT_CORRECTION_PPM / self.config.ki.max(1e-9);
        let candidate = (self.integral + error * dt).clamp(-max_integral, max_integral);
        let unclamped = self.config.kp * error + self.config.ki * candidate;
        // Anti-windup: while the output is pinned at the limit in the error's direction
        // (a stall or burst), stop learning so the integral doesn't overshoot afterwards.
        let pinned =
            unclamped.abs() > MAX_DRIFT_CORRECTION_PPM && unclamped.signum() == error.signum();
        if !pinned {
            self.integral = candidate;
        }

        let correction = (self.config.kp * error + self.config.ki * self.integral)
            .clamp(-MAX_DRIFT_CORRECTION_PPM, MAX_DRIFT_CORRECTION_PPM);
        self.applied_correction_ppm = correction;
        correction
    }

    /// Converts a correction in ppm to the resampler's relative ratio.
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

    /// Simulated live queue: 10 ms chunks of 480 frames at 48 kHz, the source running
    /// `drift_ppm` fast (negative = slow) against the send timer, capped at 8 packets like
    /// the sender. Returns the fill level (packets) at the end of each simulated second.
    struct Queue {
        comp: DriftCompensator,
        fill: f64,
    }

    impl Queue {
        fn new(drift_target: usize) -> Self {
            let mut comp = DriftCompensator::new(DriftConfig::default());
            comp.set_target_fill_level(drift_target);
            Self { comp, fill: 3.0 }
        }

        fn play(&mut self, drift_ppm: f64, secs: usize) -> Vec<f64> {
            let mut out = Vec::new();
            for step in 1..=secs * 100 {
                let corr = self
                    .comp
                    .compute_correction(480, self.fill.round() as usize);
                self.fill = (self.fill + (drift_ppm - corr) * PACKETS_PER_SEC_PER_PPM * 0.01)
                    .clamp(0.0, 8.0);
                if step % 100 == 0 {
                    out.push(self.fill);
                }
            }
            out
        }

        /// The source delivers nothing for `secs`: the sender drains the queue and starves,
        /// then the feeder tops it back up to the target when audio returns.
        fn stall(&mut self, secs: f64) {
            self.fill = (self.fill - 44_100.0 / 352.0 * secs).max(0.0);
            if self.fill < 0.5 {
                self.fill = self.comp.target_fill_level() as f64;
                self.comp.refilled(self.comp.target_fill_level());
            }
        }
    }

    #[test]
    fn holds_the_target_and_learns_a_steady_clock_difference() {
        for drift in [-700.0, -300.0, -50.0, 0.0, 80.0, 300.0, 800.0] {
            let mut q = Queue::new(3);
            let fill = q.play(drift, 600);
            let last = *fill.last().unwrap();
            // The fill is measured in whole packets, so it may rest up to half a packet off.
            assert!((last - 3.0).abs() <= 0.6, "drift {drift}: fill {last:.2}");
            let learned = q.comp.estimated_drift_ppm();
            assert!(
                (learned - drift).abs() < 10.0,
                "drift {drift}: learned {learned:.1}"
            );
            // Never near either end of the queue.
            assert!(
                fill.iter().all(|&f| (1.0..=5.0).contains(&f)),
                "drift {drift}: {fill:?}"
            );
        }
    }

    #[test]
    fn stalls_do_not_leave_a_lasting_wrong_correction() {
        // The session in which a paused player made the old estimator overflow the queue
        // every ~30 s: settled at -10 ppm, then 8 minutes of uneven delivery (a 30 ms hole
        // every second), then normal playback.
        let mut q = Queue::new(3);
        q.play(-10.0, 120);
        for _ in 0..480 {
            q.stall(0.03);
            q.play(-10.0, 1);
        }
        let after = q.play(-10.0, 600);
        assert!(after.iter().all(|&f| f < 6.0), "near overflow: {after:?}");
        assert!(
            after[60..].iter().all(|&f| (2.0..=4.0).contains(&f)),
            "{after:?}"
        );
        let learned = q.comp.estimated_drift_ppm();
        assert!(
            (learned - -10.0).abs() < 20.0,
            "learned {learned:.1} ppm after the stalls"
        );
    }

    #[test]
    fn corrections_stay_within_the_limit() {
        let mut q = Queue::new(3);
        for fill in [0, 8, 1000, 0] {
            for _ in 0..1000 {
                let c = q.comp.compute_correction(480, fill);
                assert!(c.abs() <= MAX_DRIFT_CORRECTION_PPM, "{c}");
            }
        }
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
}
