//! Optional audio health log, for tracing crackles to a cause. Turn on "Audio log" in the
//! flyout settings (or set `NYX_AUDIO_STATS=1`) to write `%TEMP%\nyx-refrain-audio.log` (`/tmp` on Linux): one line per second, marked `!`
//! when something could be heard. Every second is written so that a crackle the counters
//! don't explain can still be matched against that second's numbers.

use std::fs::File;
use std::io::Write;
use std::sync::atomic::Ordering;
use std::time::{Duration, Instant};

use pipeline::{GlitchCounts, PipelineMetricsTracker};
use raop::ap2::session::Ap2Stats;

/// A send this late (ms) past its schedule is worth flagging.
/// On the Low profile a delay well under 10 ms may already be audible.
const LATE_FLAG_MS: f64 = 5.0;

const LEGEND: &str = "\
# starved  sender had no audio ready and sent silence (feeder too slow)
# trimmed  packets dropped because too much audio piled up (feeder bursts)
# gap      capture chunks arrived >= 50 ms apart; silence bridged the gap
# silent   the source delivered nothing for 50 ms; a silent packet was queued
# wasapi   Windows flagged a break in the captured audio
# skipped  frames Windows never handed over (lost before capture, or its buffer overflowed)
# overflow captured frames dropped because the capture buffer was full
# resend   resend requests from the speaker (network loss); one request can name several packets
# resent   packets sent again in answer
# unsent   packets asked for that were too old to send again (heard as a crackle)
# clipped  samples louder than full scale, cut off at the limit (crackle on loud parts)
# timer    send-timer waits that ran out at the 50 ms fallback instead of firing
# late     largest delay past a packet's scheduled send time (flagged from 5 ms)
# send_block longest time one network send blocked (a stall here is the Wi-Fi/network driver)
# Where a clump of audio comes from (largest per line):
# win_silent frames Windows flagged as silent (480 = 10 ms)
# win_burst  frames Windows handed over in one capture wake-up (480 = one 10 ms packet)
# win_gap    time between capture wake-ups (about 10 ms is normal)
# feed_gap   time between feeder reads that got audio (about 10 ms is normal)
# ring_peak  packets queued when the sender took one (above 8 means a trim)
# peak       loudest sample sent (1.00 = full scale; above it clips)
# Drift steering:
# nudge    speed correction applied right now (ppm; 500 = 0.05%)
# learned  steady clock difference it has learned (ppm)";

#[derive(Clone, Copy, Default)]
struct Counts {
    sent: u64,
    trimmed: u64,
    resend: u64,
    resent: u64,
    unsent: u64,
    glitches: GlitchCounts,
    wasapi: u64,
    overflow: u64,
    skipped: u64,
    clipped: u64,
    timer: u64,
}

impl Counts {
    fn read(metrics: &PipelineMetricsTracker, stats: &Ap2Stats) -> Self {
        let snap = metrics.snapshot();
        #[cfg(windows)]
        let (wasapi, overflow, skipped) = (
            capture::wasapi::DIAG_WASAPI_GLITCHES.load(Ordering::Relaxed),
            capture::wasapi::DIAG_CAPTURE_OVERFLOW_FRAMES.load(Ordering::Relaxed),
            capture::wasapi::DIAG_CAPTURE_SKIPPED_FRAMES.load(Ordering::Relaxed),
        );
        #[cfg(not(windows))]
        let (wasapi, overflow, skipped) = (0, 0, 0);
        Self {
            sent: snap.packets_sent,
            trimmed: snap.packets_dropped,
            resend: stats.retransmit_requests.load(Ordering::Relaxed),
            resent: stats.retransmit_served.load(Ordering::Relaxed),
            unsent: stats.retransmit_missing.load(Ordering::Relaxed),
            glitches: metrics.glitch_counts(),
            wasapi,
            overflow,
            skipped,
            clipped: pipeline::convert::DIAG_CLIPPED_SAMPLES.load(Ordering::Relaxed),
            timer: pipeline::timer::DIAG_TIMER_TIMEOUTS.load(Ordering::Relaxed),
        }
    }
}

pub struct AudioWatch {
    file: Option<File>,
    /// `NYX_AUDIO_STATS` is set: log whatever the setting says.
    forced: bool,
    last: Option<Counts>,
    window_start: Instant,
}

impl AudioWatch {
    pub fn from_env() -> Self {
        let forced = std::env::var_os("NYX_AUDIO_STATS").is_some_and(|v| !v.is_empty() && v != "0");
        let mut watch = Self {
            file: None,
            forced,
            last: None,
            window_start: Instant::now(),
        };
        watch.set_enabled(false);
        watch
    }

    /// Follows the "Audio log" setting; takes effect at once, mid-stream too.
    pub fn set_enabled(&mut self, enabled: bool) {
        let enabled = enabled || self.forced;
        if enabled == self.file.is_some() {
            return;
        }
        self.last = None;
        self.file = enabled
            .then(|| {
                // Appended to, so a restart keeps the earlier lines.
                std::fs::OpenOptions::new()
                    .create(true)
                    .append(true)
                    .open(std::env::temp_dir().join("nyx-refrain-audio.log"))
                    .ok()
            })
            .flatten()
            .map(|mut f| {
                let _ = writeln!(f, "{LEGEND}");
                f
            });
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
        let block_ms = stats.send_block_max_us.swap(0, Ordering::Relaxed) as f64 / 1000.0;
        let (feed_gap, ring_peak) = metrics.take_burst_peaks();
        let peak = f32::from_bits(pipeline::convert::DIAG_PEAK_F32_BITS.swap(0, Ordering::Relaxed));
        #[cfg(windows)]
        let (win_silent, win_burst, win_gap_us) = (
            capture::wasapi::DIAG_SILENT_FLAG_FRAMES.swap(0, Ordering::Relaxed),
            capture::wasapi::DIAG_CAPTURE_BURST_MAX_FRAMES.swap(0, Ordering::Relaxed),
            capture::wasapi::DIAG_CAPTURE_WAKE_GAP_MAX_US.swap(0, Ordering::Relaxed),
        );
        #[cfg(not(windows))]
        let (win_silent, win_burst, win_gap_us) = (0, 0, 0);
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
            ("skipped", d(now.skipped, last.skipped)),
            ("overflow", d(now.overflow, last.overflow)),
            ("resend", d(now.resend, last.resend)),
            ("resent", d(now.resent, last.resent)),
            ("unsent", d(now.unsent, last.unsent)),
            ("clipped", d(now.clipped, last.clipped)),
            ("timer", d(now.timer, last.timer)),
        ];
        let event = fields.iter().any(|&(_, n)| n > 0) || late_ms >= LATE_FLAG_MS;
        let mut line = format!("{} {}", clock(), if event { '!' } else { ' ' });
        for (name, n) in fields {
            line += &format!(" {name}={n}");
        }
        line += &format!(
            " win_silent={win_silent} win_burst={win_burst} win_gap={:.1}ms feed_gap={:.1}ms ring_peak={ring_peak} peak={peak:.2}",
            win_gap_us as f64 / 1000.0,
            feed_gap.as_secs_f64() * 1000.0,
        );
        let snap = metrics.snapshot();
        line += &format!(
            " late={late_ms:.1}ms send_block={block_ms:.1}ms fill={fill} nudge={:+.0}ppm learned={:+.1}ppm sent={}",
            snap.applied_correction_ppm,
            snap.estimated_drift_ppm,
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
