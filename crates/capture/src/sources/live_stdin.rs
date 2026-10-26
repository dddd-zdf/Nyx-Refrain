//! Live stdin source: raw PCM produced on another device's clock, e.g. a Bluetooth A2DP sink
//! piping what a phone plays.
//!
//! Plain [`StdinSource`](super::StdinSource) is read on demand, so the sender's clock sets the
//! pace. A live producer instead writes at its own rate and in bursts, and stops when playback
//! pauses. This source reads it on a background thread into a jitter queue and is read on
//! demand from that queue:
//! - Playback starts once the queue holds the target amount, and starts over from buffering
//!   after the queue runs dry (the producer paused or stalled). Silence fills the gap, so the
//!   stream to the receiver carries on.
//! - The pipeline steers its resampler to keep the queue level steady (see
//!   [`AudioSource::queue_frames`]), which follows the producer's clock without needing
//!   timestamps for bursty arrivals.
//! - A backlog far above the target (audio queued while the receiver connection was set up)
//!   is dropped down to the target, so it does not become permanent extra latency.

use crate::{AudioChunk, AudioSource};
use std::collections::VecDeque;
use std::io::{Read, stdin};
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::{Arc, Mutex};
use std::time::{Duration, Instant};
use tracing::{info, warn};

use super::StdinFormat;

/// A backlog this many times the target is cut back to the target.
const OVERFLOW_FACTOR: usize = 3;

/// Bytes read from the producer per read call.
const READ_BYTES: usize = 4096;

#[derive(Debug, Default)]
struct Shared {
    /// Interleaved samples waiting to be read.
    queue: Mutex<VecDeque<f32>>,
    /// The producer closed its end.
    eof: AtomicBool,
}

/// Audio source reading raw PCM from a live producer through a jitter queue.
pub struct LiveStdinSource {
    shared: Arc<Shared>,
    sample_rate: u32,
    channels: u16,
    target_frames: usize,
    playing: bool,
    /// Queue level before the latest read while playing, see [`AudioSource::queue_frames`].
    level: Option<usize>,
    frames_produced: u64,
    start_time: Instant,
}

impl LiveStdinSource {
    /// Create a source reading standard input, keeping about `buffer` of audio queued.
    pub fn from_stdin(
        format: StdinFormat,
        sample_rate: u32,
        channels: u16,
        buffer: Duration,
    ) -> std::io::Result<Self> {
        Self::new(stdin(), format, sample_rate, channels, buffer)
    }

    /// Create a source reading any stream on a background thread, keeping about `buffer` of
    /// audio queued.
    pub fn new<R: Read + Send + 'static>(
        reader: R,
        format: StdinFormat,
        sample_rate: u32,
        channels: u16,
        buffer: Duration,
    ) -> std::io::Result<Self> {
        let shared = Arc::new(Shared::default());
        let frame_size = format.bytes_per_sample() * channels.max(1) as usize;
        let thread_shared = shared.clone();
        // Detached: a blocking read on stdin cannot be interrupted, and the thread ends at EOF.
        std::thread::Builder::new()
            .name("nyx-refrain-stdin".into())
            .spawn(move || read_loop(reader, format, frame_size, &thread_shared))?;
        let target_frames = ((buffer.as_secs_f64() * sample_rate as f64).round() as usize).max(1);
        Ok(Self {
            shared,
            sample_rate,
            channels,
            target_frames,
            playing: false,
            level: None,
            frames_produced: 0,
            start_time: Instant::now(),
        })
    }

    fn queued_frames(queue: &VecDeque<f32>, channels: u16) -> usize {
        queue.len() / channels.max(1) as usize
    }

    /// Next chunk on the source's own continuous timeline (frames delivered so far, silence
    /// included), so the pipeline never sees a gap.
    fn chunk(&mut self, data: Vec<f32>) -> AudioChunk {
        let elapsed = self.frames_produced;
        let rate = self.sample_rate as u64;
        let at = self.start_time
            + Duration::new(
                elapsed / rate,
                ((elapsed % rate) * 1_000_000_000 / rate) as u32,
            );
        let chunk = AudioChunk::new(data, self.sample_rate, self.channels, at);
        self.frames_produced += chunk.frame_count() as u64;
        chunk
    }

    fn silence(&mut self, frames: usize) -> AudioChunk {
        self.chunk(vec![0.0; frames * self.channels as usize])
    }

    fn ms(&self, frames: usize) -> u64 {
        frames as u64 * 1000 / self.sample_rate.max(1) as u64
    }
}

fn read_loop<R: Read>(mut reader: R, format: StdinFormat, frame_size: usize, shared: &Shared) {
    let mut buf = vec![0u8; READ_BYTES.max(frame_size)];
    // Bytes of an incomplete frame carried over to the next read.
    let mut carry = 0;
    let mut samples = Vec::with_capacity(READ_BYTES / format.bytes_per_sample());
    loop {
        match reader.read(&mut buf[carry..]) {
            Ok(0) => break,
            Ok(n) => {
                let len = carry + n;
                let whole = len - len % frame_size;
                samples.clear();
                format.decode(&buf[..whole], &mut samples);
                if let Ok(mut queue) = shared.queue.lock() {
                    queue.extend(&samples);
                }
                buf.copy_within(whole..len, 0);
                carry = len - whole;
            }
            Err(e) if e.kind() == std::io::ErrorKind::Interrupted => {}
            Err(e) => {
                warn!("live stdin read failed: {e}");
                break;
            }
        }
    }
    shared.eof.store(true, Ordering::Relaxed);
}

impl AudioSource for LiveStdinSource {
    fn sample_rate(&self) -> u32 {
        self.sample_rate
    }

    fn channels(&self) -> u16 {
        self.channels
    }

    fn read_chunk(
        &mut self,
        max_frames: usize,
    ) -> Result<Option<AudioChunk>, Box<dyn std::error::Error + Send + Sync>> {
        if max_frames == 0 || self.channels == 0 {
            return Ok(Some(AudioChunk::new(
                Vec::new(),
                self.sample_rate,
                self.channels,
                Instant::now(),
            )));
        }
        let ch = self.channels as usize;
        let eof = self.shared.eof.load(Ordering::Relaxed);
        let mut queue = self
            .shared
            .queue
            .lock()
            .map_err(|_| "live stdin queue poisoned")?;
        let mut queued = Self::queued_frames(&queue, self.channels);
        self.level = None;
        if eof && queued == 0 {
            return Ok(None);
        }

        if !self.playing {
            // Whatever is left after the producer closed is played out without the target.
            if queued < self.target_frames && !eof {
                drop(queue);
                return Ok(Some(self.silence(max_frames)));
            }
            self.playing = true;
            if queued > self.target_frames {
                queue.drain(..(queued - self.target_frames) * ch);
                queued = self.target_frames;
            }
            info!("live stdin: {} ms buffered, playing", self.ms(queued));
        } else if queued > self.target_frames * OVERFLOW_FACTOR {
            warn!(
                "live stdin: {} ms queued, dropping down to {} ms",
                self.ms(queued),
                self.ms(self.target_frames)
            );
            queue.drain(..(queued - self.target_frames) * ch);
            queued = self.target_frames;
        }

        if queued == 0 {
            self.playing = false;
            drop(queue);
            warn!(
                "live stdin ran dry; buffering {} ms again",
                self.ms(self.target_frames)
            );
            return Ok(Some(self.silence(max_frames)));
        }

        self.level = Some(queued);
        let n = queued.min(max_frames);
        let data: Vec<f32> = queue.drain(..n * ch).collect();
        drop(queue);
        Ok(Some(self.chunk(data)))
    }

    fn queue_frames(&self) -> Option<usize> {
        self.level
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::sync::mpsc::{Receiver, Sender, channel};

    /// Blocking reader fed by the test, like a pipe from a live producer.
    struct ChannelReader {
        rx: Receiver<Vec<u8>>,
        pending: Vec<u8>,
    }

    impl Read for ChannelReader {
        fn read(&mut self, buf: &mut [u8]) -> std::io::Result<usize> {
            if self.pending.is_empty() {
                match self.rx.recv() {
                    Ok(bytes) => self.pending = bytes,
                    Err(_) => return Ok(0),
                }
            }
            let n = self.pending.len().min(buf.len());
            buf[..n].copy_from_slice(&self.pending[..n]);
            self.pending.drain(..n);
            Ok(n)
        }
    }

    /// 1 kHz stereo s16 source with a 10-frame target.
    fn source() -> (LiveStdinSource, Sender<Vec<u8>>) {
        let (tx, rx) = channel();
        let reader = ChannelReader {
            rx,
            pending: Vec::new(),
        };
        let source = LiveStdinSource::new(
            reader,
            StdinFormat::S16Le,
            1000,
            2,
            Duration::from_millis(10),
        )
        .unwrap();
        (source, tx)
    }

    /// `frames` stereo frames whose left and right samples are both `value`.
    fn frames(frames: usize, value: i16) -> Vec<u8> {
        (0..frames * 2).flat_map(|_| value.to_le_bytes()).collect()
    }

    fn wait_queued(source: &LiveStdinSource, frames: usize) {
        let deadline = Instant::now() + Duration::from_secs(5);
        while LiveStdinSource::queued_frames(&source.shared.queue.lock().unwrap(), 2) < frames {
            assert!(
                Instant::now() < deadline,
                "reader thread did not queue {frames} frames"
            );
            std::thread::sleep(Duration::from_millis(1));
        }
    }

    fn is_silent(chunk: &AudioChunk) -> bool {
        chunk.data.iter().all(|&s| s == 0.0)
    }

    #[test]
    fn buffers_to_the_target_before_playing() {
        let (mut source, tx) = source();
        tx.send(frames(6, 100)).unwrap();
        wait_queued(&source, 6);
        let chunk = source.read_chunk(4).unwrap().unwrap();
        assert_eq!(chunk.frame_count(), 4);
        assert!(is_silent(&chunk));
        assert_eq!(source.queue_frames(), None);

        tx.send(frames(4, 100)).unwrap();
        wait_queued(&source, 10);
        let chunk = source.read_chunk(4).unwrap().unwrap();
        assert!(chunk.data.iter().all(|&s| s == 100.0 / 32768.0));
        assert_eq!(source.queue_frames(), Some(10));
    }

    #[test]
    fn starts_from_the_newest_audio_and_trims_a_large_backlog() {
        let (mut source, tx) = source();
        // Queued while the connection was set up: only the newest 10 frames are kept.
        tx.send(frames(15, 1)).unwrap();
        tx.send(frames(10, 2)).unwrap();
        wait_queued(&source, 25);
        let chunk = source.read_chunk(10).unwrap().unwrap();
        assert!(chunk.data.iter().all(|&s| s == 2.0 / 32768.0));

        // More than 3x the target while playing: cut back to the target.
        tx.send(frames(25, 3)).unwrap();
        tx.send(frames(10, 4)).unwrap();
        wait_queued(&source, 35);
        let chunk = source.read_chunk(100).unwrap().unwrap();
        assert_eq!(chunk.frame_count(), 10);
        assert!(chunk.data.iter().all(|&s| s == 4.0 / 32768.0));
    }

    #[test]
    fn running_dry_returns_to_buffering_with_a_continuous_timeline() {
        let (mut source, tx) = source();
        tx.send(frames(10, 5)).unwrap();
        wait_queued(&source, 10);
        let first = source.read_chunk(10).unwrap().unwrap();
        assert!(!is_silent(&first));

        let dry = source.read_chunk(4).unwrap().unwrap();
        assert!(is_silent(&dry));
        assert_eq!(dry.timestamp, first.timestamp + first.duration());
        assert_eq!(source.queue_frames(), None);

        // Not playing again until the target is back.
        tx.send(frames(5, 6)).unwrap();
        wait_queued(&source, 5);
        assert!(is_silent(&source.read_chunk(4).unwrap().unwrap()));
    }

    #[test]
    fn plays_out_the_rest_after_eof_then_ends() {
        let (mut source, tx) = source();
        // A partial frame at the end is dropped.
        let mut bytes = frames(3, 7);
        bytes.push(0);
        tx.send(bytes).unwrap();
        drop(tx);
        let deadline = Instant::now() + Duration::from_secs(5);
        while !source.shared.eof.load(Ordering::Relaxed) {
            assert!(Instant::now() < deadline);
            std::thread::sleep(Duration::from_millis(1));
        }
        let chunk = source.read_chunk(10).unwrap().unwrap();
        assert_eq!(chunk.frame_count(), 3);
        assert!(!is_silent(&chunk));
        assert!(source.read_chunk(10).unwrap().is_none());
    }
}
