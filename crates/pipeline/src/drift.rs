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
//! Corrections are clamped to `[-500.0, 500.0]` ppm (under 1 cent of pitch).

/// Maximum allowable drift correction in parts per million (±500 ppm).
pub const MAX_DRIFT_CORRECTION_PPM: f64 = 500.0;

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
        for drift in [-300.0, -50.0, 0.0, 80.0, 300.0] {
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
}
