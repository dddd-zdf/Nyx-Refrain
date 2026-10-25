//! Optional audio health log, for tracing crackles to a cause. Set `NYX_AUDIO_STATS=1` to
//! write `%TEMP%\nyx-refrain-audio.log` (`/tmp` on Linux): a line for every second in which
//! something could be heard (marked `!`), and a heartbeat every 10 s otherwise.

use std::fs::File;
use std::io::Write;
use std::sync::atomic::Ordering;
use std::time::{Duration, Instant};

use pipeline::{GlitchCounts, PipelineMetricsTracker};
use raop::ap2::session::Ap2Stats;

/// A send this late (ms) past its schedule is worth flagging.
const LATE_FLAG_MS: f64 = 10.0;
const HEARTBEAT: Duration = Duration::from_secs(10);

const LEGEND: &str = "\
# starved  sender had no audio ready and sent silence (feeder too slow)
# trimmed  packets dropped because too much audio piled up (feeder bursts)
# gap      capture chunks arrived >= 50 ms apart; silence bridged the gap
# silent   the source delivered nothing for 50 ms; a silent packet was queued
# wasapi   Windows flagged a break in the captured audio
# overflow captured frames dropped because the capture buffer was full
# resend   packets the speaker asked for again (network loss)
# late     largest delay past a packet's scheduled send time
# Where a clump of audio comes from (largest per line):
# win_burst  frames Windows handed over in one capture wake-up (480 = one 10 ms packet)
# win_gap    time between capture wake-ups (about 10 ms is normal)
# feed_gap   time between feeder reads that got audio (about 10 ms is normal)
# ring_peak  packets queued when the sender took one (above 8 means a trim)";

#[derive(Clone, Copy, Default)]
struct Counts {
    sent: u64,
    trimmed: u64,
    resend: u64,
    glitches: GlitchCounts,
    wasapi: u64,
    overflow: u64,
}

impl Counts {
    fn read(metrics: &PipelineMetricsTracker, stats: &Ap2Stats) -> Self {
        let snap = metrics.snapshot();
        #[cfg(windows)]
        let (wasapi, overflow) = (
            capture::wasapi::DIAG_WASAPI_GLITCHES.load(Ordering::Relaxed),
            capture::wasapi::DIAG_CAPTURE_OVERFLOW_FRAMES.load(Ordering::Relaxed),
        );
        #[cfg(not(windows))]
        let (wasapi, overflow) = (0, 0);
        Self {
            sent: snap.packets_sent,
            trimmed: snap.packets_dropped,
            resend: stats.retransmit_requests.load(Ordering::Relaxed),
            glitches: metrics.glitch_counts(),
            wasapi,
            overflow,
        }
    }
}

pub struct AudioWatch {
    file: Option<File>,
    last: Option<Counts>,
    window_start: Instant,
    last_line: Instant,
}

impl AudioWatch {
    pub fn from_env() -> Self {
        let file = std::env::var_os("NYX_AUDIO_STATS")
            .filter(|v| !v.is_empty() && v != "0")
            .and_then(|_| File::create(std::env::temp_dir().join("nyx-refrain-audio.log")).ok())
            .map(|mut f| {
                let _ = writeln!(f, "{LEGEND}");
                f
            });
        Self {
            file,
            last: None,
            window_start: Instant::now(),
            last_line: Instant::now(),
        }
    }

    /// Called on the engine's refresh tick while streaming; writes at most once a second.
    pub fn sample(&mut self, metrics: &PipelineMetricsTracker, stats: &Ap2Stats, fill: usize) {
        let Some(file) = &mut self.file else { return };
        if self.window_start.elapsed() < Duration::from_secs(1) {
            return;
        }
        self.window_start = Instant::now();
        let now = Counts::read(metrics, stats);
        // Peaks cover this second only, whether or not it gets a line.
        let late_ms = stats.send_late_max_us.swap(0, Ordering::Relaxed) as f64 / 1000.0;
        let (feed_gap, ring_peak) = metrics.take_burst_peaks();
        #[cfg(windows)]
        let (win_burst, win_gap_us) = (
            capture::wasapi::DIAG_CAPTURE_BURST_MAX_FRAMES.swap(0, Ordering::Relaxed),
            capture::wasapi::DIAG_CAPTURE_WAKE_GAP_MAX_US.swap(0, Ordering::Relaxed),
        );
        #[cfg(not(windows))]
        let (win_burst, win_gap_us) = (0, 0);
        // A new session starts its counters from zero.
        let Some(last) = self.last.replace(now).filter(|l| l.sent <= now.sent) else {
            let _ = writeln!(file, "{} session started", clock());
            return;
        };
        let d = |a: u64, b: u64| a.saturating_sub(b);
        let (g, lg) = (now.glitches, last.glitches);
        let fields = [
            ("starved", d(g.sender_starved, lg.sender_starved)),
            ("trimmed", d(now.trimmed, last.trimmed)),
            ("gap", d(g.capture_gap, lg.capture_gap)),
            ("silent", d(g.source_silent, lg.source_silent)),
            ("wasapi", d(now.wasapi, last.wasapi)),
            ("overflow", d(now.overflow, last.overflow)),
            ("resend", d(now.resend, last.resend)),
        ];
        let event = fields.iter().any(|&(_, n)| n > 0) || late_ms >= LATE_FLAG_MS;
        if !event && self.last_line.elapsed() < HEARTBEAT {
            return;
        }
        self.last_line = Instant::now();
        let mut line = format!("{} {}", clock(), if event { '!' } else { ' ' });
        for (name, n) in fields {
            line += &format!(" {name}={n}");
        }
        line += &format!(
            " win_burst={win_burst} win_gap={:.1}ms feed_gap={:.1}ms ring_peak={ring_peak}",
            win_gap_us as f64 / 1000.0,
            feed_gap.as_secs_f64() * 1000.0,
        );
        line += &format!(
            " late={late_ms:.1}ms fill={fill} drift={:+.1}ppm sent={}",
            metrics.snapshot().estimated_drift_ppm,
            d(now.sent, last.sent)
        );
        let _ = writeln!(file, "{line}");
    }
}

/// Local wall-clock time, to match a log line with when a crackle was heard.
fn clock() -> String {
    #[cfg(windows)]
    {
        let t = unsafe { windows::Win32::System::SystemInformation::GetLocalTime() };
        format!(
            "{:02}:{:02}:{:02}.{:03}",
            t.wHour, t.wMinute, t.wSecond, t.wMilliseconds
        )
    }
    #[cfg(not(windows))]
    {
        let secs = std::time::SystemTime::now()
            .duration_since(std::time::UNIX_EPOCH)
            .map_or(0, |d| d.as_secs());
        format!(
            "{:02}:{:02}:{:02}Z",
            secs / 3600 % 24,
            secs / 60 % 60,
            secs % 60
        )
    }
}
