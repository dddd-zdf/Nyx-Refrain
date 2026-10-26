//! WASAPI audio capture backend for Windows.

pub mod com;
pub mod endpoint;
pub mod loopback;
pub mod notification;
pub mod process_loopback;
pub mod silence_stream;

pub use com::{ComMtaGuard, MmcssGuard};
pub use endpoint::{
    DefaultOutputGuard, DeviceInfo, DeviceSpec, EndpointMuteGuard, create_device_enumerator,
    get_default_render_device, get_device_friendly_name, get_device_id, list_render_endpoints,
    open_render_device,
};
pub use loopback::{WasapiLoopbackConfig, WasapiLoopbackSource};
pub use notification::{DeviceNotificationClient, NotificationRegistration};
pub use process_loopback::{ProcessLoopbackConfig, ProcessLoopbackMode, ProcessLoopbackSource};
pub use silence_stream::SilenceKeepAliveStream;

use windows::Win32::System::Com::{COINIT_MULTITHREADED, CoInitializeEx, CoUninitialize};

/// Packets Windows flagged as a break in the captured audio (`AUDCLNT_BUFFERFLAGS_DATA_DISCONTINUITY`),
/// across all WASAPI sources; read by the audio watcher.
pub static DIAG_WASAPI_GLITCHES: std::sync::atomic::AtomicU64 =
    std::sync::atomic::AtomicU64::new(0);
/// Frames Windows handed over flagged silent (`AUDCLNT_BUFFERFLAGS_SILENT`): nothing was
/// playing, or the engine had nothing for that period.
pub static DIAG_SILENT_FLAG_FRAMES: std::sync::atomic::AtomicU64 =
    std::sync::atomic::AtomicU64::new(0);
/// Captured frames dropped because the capture ring was full.
pub static DIAG_CAPTURE_OVERFLOW_FRAMES: std::sync::atomic::AtomicU64 =
    std::sync::atomic::AtomicU64::new(0);

/// Largest number of frames one capture wake-up drained since last reset: a big value means
/// Windows delivered a clump of audio at once.
pub static DIAG_CAPTURE_BURST_MAX_FRAMES: std::sync::atomic::AtomicU64 =
    std::sync::atomic::AtomicU64::new(0);
/// Longest time (µs) between two capture wake-ups that had audio, since last reset.
pub static DIAG_CAPTURE_WAKE_GAP_MAX_US: std::sync::atomic::AtomicU64 =
    std::sync::atomic::AtomicU64::new(0);

/// Frames Windows skipped: a packet's device position started past the end of the previous
/// one, so that audio never reached us (it was lost before capture, or the capture buffer
/// overflowed while this thread was late).
pub static DIAG_CAPTURE_SKIPPED_FRAMES: std::sync::atomic::AtomicU64 =
    std::sync::atomic::AtomicU64::new(0);

/// Feeds [`DIAG_CAPTURE_BURST_MAX_FRAMES`] and [`DIAG_CAPTURE_WAKE_GAP_MAX_US`] from a capture
/// thread.
#[derive(Default)]
pub(crate) struct WakeDiag {
    last: Option<std::time::Instant>,
    frames: u64,
    /// Device position just past the previous packet.
    next_pos: Option<u64>,
}

impl WakeDiag {
    pub(crate) fn woke(&mut self) {
        use std::sync::atomic::Ordering::Relaxed;
        let now = std::time::Instant::now();
        if let Some(last) = self.last {
            DIAG_CAPTURE_WAKE_GAP_MAX_US.fetch_max((now - last).as_micros() as u64, Relaxed);
        }
        self.last = Some(now);
        self.frames = 0;
    }

    pub(crate) fn frames(&mut self, n: u32) {
        self.frames += n as u64;
    }

    /// Checks that a packet starting at device position `pos` follows on from the previous
    /// one; a position that went backwards (a new stream) just restarts the check.
    pub(crate) fn position(&mut self, pos: u64, frames: u32) {
        if let Some(next) = self.next_pos
            && pos > next
        {
            DIAG_CAPTURE_SKIPPED_FRAMES.fetch_add(pos - next, std::sync::atomic::Ordering::Relaxed);
        }
        self.next_pos = Some(pos + frames as u64);
    }

    pub(crate) fn done(&self) {
        DIAG_CAPTURE_BURST_MAX_FRAMES.fetch_max(self.frames, std::sync::atomic::Ordering::Relaxed);
    }
}

/// Writes as many whole frames of `samples` as fit into the capture ring in one block and
/// returns the number of samples dropped. Per-sample pushes could split a frame when the ring
/// filled, shifting every later frame by one channel.
pub(crate) fn push_frames(
    producer: &mut rtrb::Producer<f32>,
    samples: &[f32],
    channels: usize,
) -> usize {
    let channels = channels.max(1);
    let n = samples.len().min(producer.slots()) / channels * channels;
    if n > 0
        && let Ok(chunk) = producer.write_chunk_uninit(n)
    {
        chunk.fill_from_iter(samples[..n].iter().copied());
    }
    samples.len() - n
}

/// Appends up to `max_samples` queued samples, whole frames only, to `out` in one block.
pub(crate) fn pop_frames(
    consumer: &mut rtrb::Consumer<f32>,
    max_samples: usize,
    channels: usize,
    out: &mut Vec<f32>,
) {
    let channels = channels.max(1);
    let n = consumer.slots().min(max_samples) / channels * channels;
    if n > 0
        && let Ok(chunk) = consumer.read_chunk(n)
    {
        let (a, b) = chunk.as_slices();
        out.extend_from_slice(a);
        out.extend_from_slice(b);
        chunk.commit_all();
    }
}

/// Test and verify WASAPI / COM initialization and linking.
pub fn init_wasapi() -> Result<(), String> {
    unsafe {
        CoInitializeEx(None, COINIT_MULTITHREADED)
            .ok()
            .map_err(|e| format!("CoInitializeEx failed: {e}"))?;
        CoUninitialize();
    }
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn ring_moves_whole_frames_only() {
        let (mut producer, mut consumer) = rtrb::RingBuffer::<f32>::new(5);
        // Room for 5 samples but only 2 stereo frames: the half frame is dropped.
        assert_eq!(
            push_frames(&mut producer, &[1.0, 2.0, 3.0, 4.0, 5.0, 6.0], 2),
            2
        );
        let mut out = Vec::new();
        // Asking for 3 samples returns one whole frame.
        pop_frames(&mut consumer, 3, 2, &mut out);
        assert_eq!(out, [1.0, 2.0]);
        pop_frames(&mut consumer, 8, 2, &mut out);
        assert_eq!(out, [1.0, 2.0, 3.0, 4.0]);
    }
}
