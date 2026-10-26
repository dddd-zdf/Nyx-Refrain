//! Clocks the stream from the capture device instead of a send timer.
//!
//! A delay-locked loop (F. Adriaensen, "Using a DLL to filter time", 2005) filters the
//! capture timestamps of each chunk into a smooth frame-index -> time mapping that runs at
//! the device's true rate. The sender then sends each packet as soon as it is ready and
//! tells the receiver, in its sync packets, which RTP time plays when; the receiver follows
//! that rate. With a single clock there is nothing to steer: no fill target, no trimming,
//! no refilling.

use std::sync::Mutex;
use std::sync::atomic::{AtomicBool, Ordering};
use std::time::{Duration, Instant};

use capture::DataSignal;

use crate::resample::TARGET_SAMPLE_RATE;

/// Loop bandwidth (Hz). Capture timestamps jitter by about a millisecond; at 0.1 Hz that
/// averages out to microseconds while still following any real rate change in seconds.
const DLL_BANDWIDTH_HZ: f64 = 0.1;

/// A chunk this far off the filtered clock is a break in the stream, not jitter.
pub const MAX_CLOCK_ERROR: Duration = Duration::from_millis(20);

/// Largest accepted deviation of the device rate from nominal (2000 ppm); beyond it the
/// estimate is wrong, not the device.
const MAX_RATE_DEVIATION: f64 = 2e-3;

/// Second-order DLL mapping input frame indices to capture times.
#[derive(Debug, Clone)]
pub struct Dll {
    nominal_spf: f64,
    /// Filtered capture time (seconds since `epoch`) of frame `n_ref`.
    t_ref: f64,
    n_ref: f64,
    /// Filtered seconds per frame.
    spf: f64,
}

/// What a [`Dll::update`] made of a chunk.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum ClockUpdate {
    /// On time within jitter; the loop took it in.
    Tracked,
    /// Arrived late by this many seconds: audio is missing before it.
    Late(f64),
    /// Arrived early by this many seconds (after silence was made up for a gap that turned
    /// out shorter); the clock was moved to it.
    Early(f64),
}

impl Dll {
    /// Starts the loop at frame `n` captured at `t` (seconds), at the nominal `rate`.
    pub fn new(rate: u32, n: f64, t: f64) -> Self {
        let nominal_spf = 1.0 / rate.max(1) as f64;
        Self {
            nominal_spf,
            t_ref: t,
            n_ref: n,
            spf: nominal_spf,
        }
    }

    /// Filtered capture time of frame `n`.
    pub fn time_of(&self, n: f64) -> f64 {
        self.t_ref + (n - self.n_ref) * self.spf
    }

    /// Device rate relative to nominal, in ppm (positive = the device runs fast).
    pub fn rate_ppm(&self) -> f64 {
        (self.nominal_spf / self.spf - 1.0) * 1e6
    }

    /// Takes in a chunk starting at frame `n` captured at `t`. A late chunk is not taken in:
    /// the caller makes up the missing frames and calls again.
    pub fn update(&mut self, n: f64, t: f64) -> ClockUpdate {
        let dn = n - self.n_ref;
        if dn <= 0.0 {
            return ClockUpdate::Tracked;
        }
        let predicted = self.time_of(n);
        let e = t - predicted;
        let max = MAX_CLOCK_ERROR.as_secs_f64();
        if e > max {
            return ClockUpdate::Late(e);
        }
        if e < -max {
            self.t_ref = t;
            self.n_ref = n;
            return ClockUpdate::Early(-e);
        }
        let w = 2.0 * std::f64::consts::PI * DLL_BANDWIDTH_HZ * dn * self.spf;
        self.t_ref = predicted + std::f64::consts::SQRT_2 * w * e;
        self.n_ref = n;
        self.spf = (self.spf + w * w * e / dn).clamp(
            self.nominal_spf * (1.0 - MAX_RATE_DEVIATION),
            self.nominal_spf * (1.0 + MAX_RATE_DEVIATION),
        );
        ClockUpdate::Tracked
    }
}

/// Output-frame clock of the stream, published by the feeder for the sender.
#[derive(Debug, Clone, Copy)]
struct Mapping {
    /// Output frames (since the stream started) at `t_ref`.
    out_ref: f64,
    t_ref: Instant,
    /// Output frames per second.
    out_rate: f64,
}

/// Shared between the pipeline feeder, which drives it from capture timestamps, and the
/// sender, which reads it to time sync packets.
#[derive(Debug, Default)]
pub struct CaptureClock {
    started: AtomicBool,
    mapping: Mutex<Option<Mapping>>,
    /// Raised by the feeder whenever it queues packets.
    pub packets_ready: DataSignal,
}

impl CaptureClock {
    pub fn new() -> Self {
        Self::default()
    }

    /// Called by the sender when it is ready for audio: the feeder discards capture until
    /// then, and the stream (output frame 0) starts at the next chunk.
    pub fn start(&self) {
        self.started.store(true, Ordering::Release);
    }

    pub fn is_started(&self) -> bool {
        self.started.load(Ordering::Acquire)
    }

    /// Output frames (since the stream started, at 44.1 kHz) captured by `t`; `None` until
    /// the first chunk of the stream.
    pub fn output_frames_at(&self, t: Instant) -> Option<f64> {
        let m = (*self.mapping.lock().ok()?)?;
        let dt = if t >= m.t_ref {
            t.duration_since(m.t_ref).as_secs_f64()
        } else {
            -m.t_ref.duration_since(t).as_secs_f64()
        };
        Some(m.out_ref + dt * m.out_rate)
    }

    /// Publishes the feeder's DLL: input frame `n_start` is output frame 0, and the input
    /// runs at `in_rate` nominal, resampled to 44.1 kHz. `epoch` is the DLL's time zero.
    pub fn publish(&self, dll: &Dll, epoch: Instant, n_start: f64, in_rate: u32) {
        let ratio = TARGET_SAMPLE_RATE as f64 / in_rate.max(1) as f64;
        let t_ref = epoch + Duration::from_secs_f64(dll.t_ref.max(0.0));
        let mapping = Mapping {
            out_ref: (dll.n_ref - n_start) * ratio,
            t_ref,
            out_rate: ratio / dll.spf,
        };
        if let Ok(mut m) = self.mapping.lock() {
            *m = Some(mapping);
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// 10 ms chunks of 480 frames from a device `ppm` off nominal, each timestamp off by up
    /// to ±`jitter` seconds.
    fn run(ppm: f64, jitter: f64, chunks: usize) -> Dll {
        let true_spf = 1.0 / (48_000.0 * (1.0 + ppm * 1e-6));
        let mut dll = Dll::new(48_000, 0.0, 0.0);
        let mut seed = 0x1234_5678_u64;
        for k in 1..=chunks {
            seed ^= seed << 13;
            seed ^= seed >> 7;
            seed ^= seed << 17;
            let noise = ((seed % 2001) as f64 / 1000.0 - 1.0) * jitter;
            let n = (k * 480) as f64;
            assert_eq!(dll.update(n, n * true_spf + noise), ClockUpdate::Tracked);
        }
        dll
    }

    #[test]
    fn learns_the_device_rate_through_jitter() {
        for ppm in [-300.0, -20.0, 0.0, 45.0, 400.0] {
            let clean = run(ppm, 0.0, 30_000);
            assert!(
                (clean.rate_ppm() - ppm).abs() < 1.0,
                "{ppm}: {}",
                clean.rate_ppm()
            );
            // ±1 ms timestamp jitter leaves a rate wobble of some tens of ppm (inaudible, the
            // receiver smooths it further) but the timing itself within a fraction of it.
            let dll = run(ppm, 1e-3, 30_000);
            assert!(
                (dll.rate_ppm() - ppm).abs() < 50.0,
                "{ppm}: {}",
                dll.rate_ppm()
            );
            let n = 30_000.0 * 480.0;
            let truth = n / (48_000.0 * (1.0 + ppm * 1e-6));
            let err = (dll.time_of(n) - truth).abs();
            assert!(err < 2e-4, "{ppm}: off by {:.0} µs", err * 1e6);
        }
    }

    #[test]
    fn reports_breaks_instead_of_bending_the_clock() {
        let mut dll = run(0.0, 0.0, 1000);
        let n = 1001.0 * 480.0;
        let on_time = n / 48_000.0;
        match dll.update(n, on_time + 0.05) {
            ClockUpdate::Late(e) => assert!((e - 0.05).abs() < 1e-6),
            other => panic!("{other:?}"),
        }
        // Not taken in: the same chunk on time is still tracked.
        assert_eq!(dll.update(n, on_time), ClockUpdate::Tracked);
        let n2 = n + 480.0;
        assert!(matches!(
            dll.update(n2, n2 / 48_000.0 - 0.05),
            ClockUpdate::Early(_)
        ));
        assert!((dll.time_of(n2) - (n2 / 48_000.0 - 0.05)).abs() < 1e-9);
    }

    #[test]
    fn maps_output_frames_at_the_learned_rate() {
        let clock = CaptureClock::new();
        let epoch = Instant::now();
        assert!(clock.output_frames_at(epoch).is_none());
        let dll = run(100.0, 0.0, 10_000);
        clock.publish(&dll, epoch, 0.0, 48_000);
        let t = epoch + Duration::from_secs(100);
        let out = clock.output_frames_at(t).unwrap();
        // 100 s of a +100 ppm device, resampled to 44.1 kHz.
        let expected = 100.0 * 44_100.0 * (1.0 + 100e-6);
        assert!((out - expected).abs() < 1.0, "{out} vs {expected}");
    }
}
