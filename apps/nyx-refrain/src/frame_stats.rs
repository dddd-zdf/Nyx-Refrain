//! Optional flyout frame timing, for finding where UI CPU time goes. Set `NYX_FRAME_STATS=1`
//! to write one line per second to `%TEMP%\nyx-refrain-frames.log`.

use std::fs::File;
use std::io::Write;
use std::time::{Duration, Instant};

#[derive(Clone, Copy)]
pub enum Stage {
    /// egui pass: app logic, layout and text.
    Ui,
    Tessellate,
    Raster,
    Present,
}

const STAGE_NAMES: [&str; 4] = ["ui", "tessellate", "raster", "present"];

#[derive(Clone, Copy, Default)]
struct Timing {
    total: Duration,
    max: Duration,
}

pub struct FrameStats {
    file: Option<File>,
    started: Instant,
    window_start: Instant,
    passes: u32,
    painted: u32,
    redrawn_pixels: u64,
    window_pixels: u64,
    stages: [Timing; 4],
}

impl FrameStats {
    pub fn from_env() -> Self {
        let file = std::env::var_os("NYX_FRAME_STATS")
            .filter(|v| !v.is_empty() && v != "0")
            .and_then(|_| File::create(std::env::temp_dir().join("nyx-refrain-frames.log")).ok());
        Self::new(file, Instant::now())
    }

    fn new(file: Option<File>, started: Instant) -> Self {
        Self {
            file,
            started,
            window_start: Instant::now(),
            passes: 0,
            painted: 0,
            redrawn_pixels: 0,
            window_pixels: 0,
            stages: Default::default(),
        }
    }

    /// Counts one egui pass, painted or not.
    pub fn pass(&mut self) {
        self.passes += 1;
    }

    /// Records the time `stage` took, ending now.
    pub fn stage(&mut self, stage: Stage, since: Instant) {
        if self.file.is_some() {
            let t = &mut self.stages[stage as usize];
            let d = since.elapsed();
            t.total += d;
            t.max = t.max.max(d);
        }
    }

    /// Counts one presented frame that redrew `redrawn` of the window's `total` pixels.
    pub fn painted(&mut self, redrawn: u64, total: u64) {
        self.painted += 1;
        self.redrawn_pixels += redrawn;
        self.window_pixels += total;
    }

    /// Writes and resets the counters once a second.
    pub fn flush_if_due(&mut self) {
        let Some(file) = &mut self.file else { return };
        if self.window_start.elapsed() < Duration::from_secs(1) {
            return;
        }
        let mut line = format!(
            "t={:.1}s passes={} painted={} skipped={} redrawn={}%",
            self.started.elapsed().as_secs_f32(),
            self.passes,
            self.painted,
            self.passes - self.painted,
            (self.redrawn_pixels * 100)
                .checked_div(self.window_pixels)
                .unwrap_or(0),
        );
        for (i, t) in self.stages.iter().enumerate() {
            // The egui pass runs every pass; the rest only for painted frames.
            let n = if i == Stage::Ui as usize {
                self.passes
            } else {
                self.painted
            };
            if n > 0 {
                line += &format!(
                    " {}={:.2}/{:.2}ms",
                    STAGE_NAMES[i],
                    t.total.as_secs_f64() * 1000.0 / n as f64,
                    t.max.as_secs_f64() * 1000.0
                );
            }
        }
        let _ = writeln!(file, "{line}");
        *self = Self::new(self.file.take(), self.started);
    }
}
