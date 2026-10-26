//! Audio processing pipeline: downmix, resampling, drift compensation, TPDF dither,
//! strict 352-frame packetizer, and underrun handling.

use std::sync::Arc;
use std::sync::atomic::{AtomicBool, Ordering};
use std::time::{Duration, Instant};

use capture::{AudioChunk, AudioSource};

use crate::convert::{TpdfDither, downmix_to_stereo, quantize_exact};
use crate::drift::{DriftCompensator, DriftConfig, QueueController};
use crate::metrics::{Glitch, SharedPipelineMetrics};
use crate::resample::{DEFAULT_RESAMPLER_CHUNK_FRAMES, ResamplePipeline};

/// Number of frames per AirPlay audio packet.
pub const ALAC_FRAME_SAMPLES: usize = 352;

/// Number of interleaved 16-bit PCM samples per AirPlay packet (352 frames * 2 channels).
pub const ALAC_PCM_SAMPLES: usize = ALAC_FRAME_SAMPLES * 2; // 704

/// Default timeout in milliseconds for underrun detection (default 50 ms).
pub const DEFAULT_UNDERRUN_TIMEOUT_MS: u64 = 50;

/// Default capacity in packets for the lock-free ring buffer (32 packets ≈ 255 ms).
pub const DEFAULT_RING_BUFFER_PACKETS: usize = 32;

/// Target ring fill (packets) for live sources; must stay well below the sender's backlog cap
/// (`raop::ap2::session::LIVE_MAX_BUFFERED_PACKETS` = 8) so chunk-granularity peaks never trim.
pub const LIVE_TARGET_FILL_PACKETS: usize = 3;

/// Core processing engine that converts, resamples, compensates drift, dithers,
/// and slices audio into strict 352-frame packets.
pub struct PipelineProcessor {
    source_sample_rate: u32,
    resampler: ResamplePipeline,
    drift: DriftCompensator,
    /// Steer the queue fill (live sources); paced sources keep the nominal ratio.
    steer: bool,
    /// Steers the resampler from the source's jitter queue; see [`Self::set_source_queue`].
    queue: Option<QueueController>,
    source_queue: Option<usize>,
    dither: TpdfDither,
    underrun_timeout: Duration,
    last_chunk_time: Option<Instant>,
    downmix_buf: Vec<f32>,
    resampled_f32: Vec<f32>,
    staging_s16: Vec<i16>,
    metrics: SharedPipelineMetrics,
}

impl PipelineProcessor {
    /// Steers the queue towards `packets` (live sources). Without this the processor keeps
    /// the nominal ratio: back-pressured sources hold the queue full, so there is no clock
    /// difference to correct. See [`DriftCompensator::set_target_fill_level`].
    pub fn set_target_fill_level(&mut self, packets: usize) {
        self.drift.set_target_fill_level(packets);
        self.steer = true;
    }

    /// Fill level (packets) to top the queue up to after the sender ran dry, when steering.
    pub fn refill_target(&self) -> Option<usize> {
        self.steer.then(|| self.drift.target_fill_level())
    }

    /// Tells the drift controller the queue was just topped up to `fill` packets.
    pub fn refilled(&mut self, fill: usize) {
        self.drift.refilled(fill);
    }

    /// Passes the source's [`AudioSource::queue_frames`] after each read. Once a source reports
    /// a level, the resampler follows that queue instead of the drift estimator: the source is
    /// read on demand, but its audio arrives on another device's clock.
    pub fn set_source_queue(&mut self, level: Option<usize>) {
        if level.is_some() && self.queue.is_none() {
            self.queue = Some(QueueController::new());
        }
        self.source_queue = level;
    }

    /// For sources paced by back-pressure (files, stdin): the sender's clock already sets
    /// their rate, so there is no drift to correct. The resampler then stays at ratio 1.0,
    /// which at 44.1 kHz is a passthrough, and 16-bit input reaches the receiver bit-perfect.
    pub fn disable_drift_compensation(&mut self) {
        self.steer = false;
        self.drift.reset();
        self.metrics.set_drift_ppm(0.0, 0.0);
    }

    pub fn new(
        source_sample_rate: u32,
        metrics: SharedPipelineMetrics,
        underrun_timeout: Option<Duration>,
    ) -> Result<Self, String> {
        let resampler = ResamplePipeline::new(source_sample_rate, DEFAULT_RESAMPLER_CHUNK_FRAMES)?;
        let drift = DriftCompensator::new(DriftConfig {
            nominal_sample_rate: source_sample_rate,
            target_fill_level: DEFAULT_RING_BUFFER_PACKETS / 2,
            ..Default::default()
        });

        Ok(Self {
            source_sample_rate,
            resampler,
            drift,
            steer: false,
            queue: None,
            source_queue: None,
            dither: TpdfDither::default(),
            underrun_timeout: underrun_timeout
                .unwrap_or(Duration::from_millis(DEFAULT_UNDERRUN_TIMEOUT_MS)),
            last_chunk_time: None,
            downmix_buf: Vec::with_capacity(1024),
            resampled_f32: Vec::with_capacity(1024),
            staging_s16: Vec::with_capacity(ALAC_PCM_SAMPLES * 4),
            metrics,
        })
    }

    /// Reset internal state.
    pub fn reset(&mut self) {
        self.drift.reset();
        self.last_chunk_time = None;
        self.downmix_buf.clear();
        self.resampled_f32.clear();
        self.staging_s16.clear();
    }

    /// Process an incoming `AudioChunk` and produce ready 352-frame packets (`[i16; 704]`).
    ///
    /// Checks for underruns: if more than `underrun_timeout` has elapsed since the last chunk,
    /// synthesized silence frames are inserted to bridge the gap.
    pub fn process_chunk(
        &mut self,
        chunk: &AudioChunk,
        current_fill_packets: usize,
        packets_out: &mut Vec<[i16; ALAC_PCM_SAMPLES]>,
    ) -> Result<(), String> {
        let now = chunk.timestamp;
        if chunk.sample_rate != self.source_sample_rate && chunk.sample_rate > 0 {
            self.change_sample_rate(chunk.sample_rate)?;
        }

        // Check for source underrun
        if let Some(last_time) = self.last_chunk_time
            && now > last_time
        {
            let gap = now.duration_since(last_time);
            if gap >= self.underrun_timeout {
                self.metrics.record_glitch(Glitch::CaptureGap);
                // Synthesize silence for the missing duration
                let gap_frames =
                    ((gap.as_secs_f64() * self.source_sample_rate as f64) as usize).min(48000);
                if gap_frames > 0 {
                    let silence_f32 = vec![0.0f32; gap_frames * 2];
                    self.process_stereo_f32(&silence_f32, packets_out)?;
                }
            }
        }
        self.last_chunk_time = Some(now);

        // 1. Channel downmix to stereo f32
        let mut downmix = std::mem::take(&mut self.downmix_buf);
        downmix_to_stereo(&chunk.data, chunk.channels, &mut downmix);

        // 2. Clock drift compensation
        if let Some(queue) = &mut self.queue {
            let corr_ppm =
                queue.update(self.source_queue, self.source_sample_rate, chunk.duration());
            let rel_ratio = DriftCompensator::ppm_to_relative_ratio(corr_ppm);
            self.resampler.set_relative_ratio(rel_ratio)?;
            self.metrics
                .set_drift_ppm(queue.estimated_drift_ppm(), corr_ppm);
        } else {
            let corr_ppm = if self.steer {
                self.drift
                    .compute_correction(chunk.frame_count(), current_fill_packets)
            } else {
                0.0
            };
            let rel_ratio = DriftCompensator::ppm_to_relative_ratio(corr_ppm);
            self.resampler.set_relative_ratio(rel_ratio)?;
            self.metrics
                .set_drift_ppm(self.drift.estimated_drift_ppm(), corr_ppm);
        }

        // 3. Resample and packetize
        let res = self.process_stereo_f32(&downmix, packets_out);
        self.downmix_buf = downmix;
        res?;

        Ok(())
    }

    /// Follows a source whose rate changed mid-stream (device loopback after the default
    /// output switched, e.g. to a Bluetooth headset): resamples from the new rate and drops
    /// the drift estimates, which belonged to the old device's clock.
    fn change_sample_rate(&mut self, rate: u32) -> Result<(), String> {
        tracing::info!(
            from = self.source_sample_rate,
            to = rate,
            "Source sample rate changed"
        );
        self.resampler = ResamplePipeline::new(rate, DEFAULT_RESAMPLER_CHUNK_FRAMES)?;
        self.source_sample_rate = rate;
        self.drift = DriftCompensator::new(DriftConfig {
            nominal_sample_rate: rate,
            target_fill_level: self.drift.target_fill_level(),
            ..Default::default()
        });
        if self.queue.is_some() {
            self.queue = Some(QueueController::new());
        }
        Ok(())
    }

    /// Process stereo f32 samples through resampler, quantizer, and packetizer.
    fn process_stereo_f32(
        &mut self,
        stereo_f32: &[f32],
        packets_out: &mut Vec<[i16; ALAC_PCM_SAMPLES]>,
    ) -> Result<(), String> {
        self.resampled_f32.clear();
        self.resampler
            .process(stereo_f32, &mut self.resampled_f32)?;

        self.quantize_resampled();

        // Extract complete 352-frame packets (704 s16 samples)
        while self.staging_s16.len() >= ALAC_PCM_SAMPLES {
            let mut packet = [0i16; ALAC_PCM_SAMPLES];
            packet.copy_from_slice(&self.staging_s16[..ALAC_PCM_SAMPLES]);
            packets_out.push(packet);
            self.staging_s16.drain(..ALAC_PCM_SAMPLES);
        }

        Ok(())
    }

    /// Append `resampled_f32` to `staging_s16`: exactly if it is untouched 16-bit audio,
    /// otherwise with TPDF dither.
    fn quantize_resampled(&mut self) {
        let start_len = self.staging_s16.len();
        self.staging_s16
            .resize(start_len + self.resampled_f32.len(), 0);
        let out = &mut self.staging_s16[start_len..];
        if !quantize_exact(&self.resampled_f32, out) {
            self.dither.quantize_slice(&self.resampled_f32, out);
        }
    }

    /// Flush remaining staged samples into packets, padding the final packet with silence.
    pub fn flush(&mut self, packets_out: &mut Vec<[i16; ALAC_PCM_SAMPLES]>) -> Result<(), String> {
        self.resampled_f32.clear();
        self.resampler.flush(&mut self.resampled_f32)?;

        if !self.resampled_f32.is_empty() {
            self.quantize_resampled();
        }

        if !self.staging_s16.is_empty() {
            let mut packet = [0i16; ALAC_PCM_SAMPLES];
            let copy_len = self.staging_s16.len().min(ALAC_PCM_SAMPLES);
            packet[..copy_len].copy_from_slice(&self.staging_s16[..copy_len]);
            packets_out.push(packet);
            self.staging_s16.clear();
        }

        Ok(())
    }

    /// Access live metrics tracker.
    pub fn metrics(&self) -> SharedPipelineMetrics {
        self.metrics.clone()
    }
}

/// Spawns the background audio pipeline feeder thread that reads from `AudioSource`,
/// processes through `PipelineProcessor`, and pushes 352-frame packets into `rtrb::Producer`.
pub fn start_pipeline_feeder(
    mut source: Box<dyn AudioSource + Send>,
    mut producer: rtrb::Producer<[i16; ALAC_PCM_SAMPLES]>,
    metrics: SharedPipelineMetrics,
    stop_signal: Arc<AtomicBool>,
    underrun_timeout: Duration,
) -> Result<std::thread::JoinHandle<()>, std::io::Error> {
    let sample_rate = source.sample_rate();
    let _channels = source.channels();

    std::thread::Builder::new()
        .name("nyx-refrain-feeder".into())
        .spawn(move || {
            let _priority = capture::enter_audio_thread_priority();
            let mut processor = match PipelineProcessor::new(
                sample_rate,
                metrics.clone(),
                Some(underrun_timeout),
            ) {
                Ok(p) => p,
                Err(e) => {
                    tracing::error!("Failed to initialize PipelineProcessor: {e}");
                    return;
                }
            };

            if source.is_live() {
                // Live capture: the sender trims above 8 packets, so steer to 3.
                processor.set_target_fill_level(LIVE_TARGET_FILL_PACKETS);
            } else {
                processor.disable_drift_compensation();
            }
            let _ = source.start();
            // Sources that signal new audio let this thread sleep until a packet arrives
            // instead of waking every few ms to poll.
            let data_signal = source.data_signal();
            let mut packets_batch = Vec::with_capacity(8);
            let mut last_activity = Instant::now();
            let mut starved_seen = metrics.glitch_counts().sender_starved;

            // Calculate chunk size for ~10 ms reads
            let read_frames = (sample_rate / 100).max(128) as usize;

            while !stop_signal.load(Ordering::Relaxed) {
                let mut idle = false;
                match source.read_chunk(read_frames) {
                    Ok(Some(chunk)) => {
                        metrics.record_feeder_gap(last_activity.elapsed());
                        last_activity = Instant::now();
                        processor.set_source_queue(source.queue_frames());
                        packets_batch.clear();
                        let current_fill = producer.slots(); // capacity - slots = fill
                        let mut ring_fill =
                            DEFAULT_RING_BUFFER_PACKETS.saturating_sub(current_fill);

                        let starved = metrics.glitch_counts().sender_starved;
                        if starved != starved_seen {
                            starved_seen = starved;
                            // The sender just ran dry and sent silence. Top the queue back up
                            // to the target in one step (stretching that silence by a few ms)
                            // rather than steering up over ~50 s, during which every timing
                            // bump would leave another gap.
                            if let Some(target) = processor.refill_target() {
                                while ring_fill < target
                                    && producer.push([0i16; ALAC_PCM_SAMPLES]).is_ok()
                                {
                                    ring_fill += 1;
                                }
                                processor.refilled(ring_fill);
                            }
                        }
                        metrics.set_buffer_fill(ring_fill);

                        if let Ok(()) =
                            processor.process_chunk(&chunk, ring_fill, &mut packets_batch)
                        {
                            for mut pkt in packets_batch.drain(..) {
                                while !stop_signal.load(Ordering::Relaxed) {
                                    match producer.push(pkt) {
                                        Ok(()) => break,
                                        Err(rtrb::PushError::Full(returned)) => {
                                            pkt = returned;
                                            std::thread::sleep(Duration::from_millis(1));
                                        }
                                    }
                                }
                            }
                        }
                    }
                    Ok(None) => {
                        // Nothing yet / end of stream: wait a little longer below
                        idle = true;
                    }
                    Err(e) => {
                        tracing::warn!("Error reading from AudioSource: {e}");
                        metrics.record_discontinuity();
                        std::thread::sleep(Duration::from_millis(10));
                    }
                }

                // If source delivered nothing for underrun_timeout, synthesize silence packet
                if last_activity.elapsed() >= underrun_timeout {
                    let current_fill = producer.slots();
                    let ring_fill = DEFAULT_RING_BUFFER_PACKETS.saturating_sub(current_fill);
                    metrics.set_buffer_fill(ring_fill);
                    metrics.record_glitch(Glitch::SourceSilent);

                    let silence_pkt = [0i16; ALAC_PCM_SAMPLES];
                    let _ = producer.push(silence_pkt);
                    last_activity = Instant::now();
                }

                match &data_signal {
                    // Got audio: read again at once so a burst of packets never leaves a backlog.
                    Some(_) if !idle => {}
                    // Capped at one WASAPI period so the underrun check above keeps pace.
                    Some(signal) => signal.wait(
                        underrun_timeout
                            .saturating_sub(last_activity.elapsed())
                            .clamp(Duration::from_millis(1), Duration::from_millis(10)),
                    ),
                    None => std::thread::sleep(Duration::from_millis(if idle { 7 } else { 2 })),
                }
            }

            let _ = source.stop();
        })
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::metrics::PipelineMetricsTracker;
    use capture::sources::sine::SineSource;

    #[test]
    fn test_pipeline_processor_48k_to_44k_packetization() {
        let metrics = Arc::new(PipelineMetricsTracker::new(DEFAULT_RING_BUFFER_PACKETS));
        let mut proc = PipelineProcessor::new(48000, metrics.clone(), None).unwrap();

        let mut sine = SineSource::new(440.0, -20.0, 48000, 2);
        let mut packets = Vec::new();

        // Feed 100 ms of 48 kHz audio (4800 frames)
        for _ in 0..10 {
            let chunk = sine.read_chunk(480).unwrap().unwrap();
            proc.process_chunk(&chunk, 16, &mut packets).unwrap();
        }

        proc.flush(&mut packets).unwrap();

        // 100 ms at 44.1 kHz is ~4410 frames = ~12.5 packets of 352 frames
        assert!(
            packets.len() >= 12 && packets.len() <= 14,
            "Expected 12-14 packets, got {}",
            packets.len()
        );

        // Every packet must have exactly ALAC_PCM_SAMPLES (704)
        for pkt in &packets {
            assert_eq!(pkt.len(), ALAC_PCM_SAMPLES);
        }
    }

    #[test]
    fn source_rate_change_keeps_duration() {
        let metrics = Arc::new(PipelineMetricsTracker::new(DEFAULT_RING_BUFFER_PACKETS));
        let mut proc = PipelineProcessor::new(48000, metrics, None).unwrap();
        let mut packets = Vec::new();

        // 100 ms at 48 kHz stereo, then the device switches: 100 ms at 16 kHz mono.
        let mut speakers = SineSource::new(440.0, -20.0, 48000, 2);
        for _ in 0..10 {
            let chunk = speakers.read_chunk(480).unwrap().unwrap();
            proc.process_chunk(&chunk, 16, &mut packets).unwrap();
        }
        let mut headset = SineSource::new(440.0, -20.0, 16000, 1);
        for _ in 0..10 {
            let chunk = headset.read_chunk(160).unwrap().unwrap();
            proc.process_chunk(&chunk, 16, &mut packets).unwrap();
        }
        proc.flush(&mut packets).unwrap();

        // 200 ms at 44.1 kHz is ~25 packets; reading the 16 kHz audio as 48 kHz would give ~17.
        assert!(
            (23..=27).contains(&packets.len()),
            "Expected ~25 packets, got {}",
            packets.len()
        );
    }

    #[test]
    fn test_pipeline_underrun_detection_and_silence_insertion() {
        let metrics = Arc::new(PipelineMetricsTracker::new(DEFAULT_RING_BUFFER_PACKETS));
        let mut proc =
            PipelineProcessor::new(44100, metrics.clone(), Some(Duration::from_millis(50)))
                .unwrap();

        let now = Instant::now();
        let chunk1 = AudioChunk::new(vec![0.5f32; 882], 44100, 2, now);
        let mut packets = Vec::new();
        proc.process_chunk(&chunk1, 16, &mut packets).unwrap();

        // Simulate 100 ms gap (> 50 ms underrun timeout)
        let gap_time = now + Duration::from_millis(100);
        let chunk2 = AudioChunk::new(vec![0.5f32; 882], 44100, 2, gap_time);
        proc.process_chunk(&chunk2, 16, &mut packets).unwrap();

        let snap = metrics.snapshot();
        assert!(
            snap.capture_discontinuities > 0,
            "Underrun should have been detected"
        );
    }

    #[test]
    fn test_pipeline_processor_reset_and_flush_padding() {
        let metrics = Arc::new(PipelineMetricsTracker::new(DEFAULT_RING_BUFFER_PACKETS));
        let mut proc = PipelineProcessor::new(44100, metrics.clone(), None).unwrap();
        assert_eq!(proc.metrics().snapshot().estimated_drift_ppm, 0.0);

        // Feed half a packet (352 samples = 176 frames of stereo)
        let chunk = AudioChunk::new(vec![0.1f32; 352], 44100, 2, Instant::now());
        let mut packets = Vec::new();
        proc.process_chunk(&chunk, 16, &mut packets).unwrap();
        assert_eq!(packets.len(), 0); // Not enough for a full packet

        // Flush must pad to 704 samples
        proc.flush(&mut packets).unwrap();
        assert_eq!(packets.len(), 1);
        assert_eq!(packets[0].len(), ALAC_PCM_SAMPLES);
        assert_ne!(packets[0][0], 0); // Audio content present
        assert_eq!(packets[0][703], 0); // Silence padded at the end

        // Calling reset clears buffers
        proc.reset();
        let mut empty_packets = Vec::new();
        proc.flush(&mut empty_packets).unwrap();
        assert_eq!(empty_packets.len(), 0);
    }

    /// 16-bit 44.1 kHz stereo from a back-pressured source (a CD on stdin) must reach the
    /// packets unchanged: no resampling, no dither.
    #[test]
    fn test_pipeline_16bit_stdin_is_bit_perfect() {
        use capture::sources::stdin::{StdinFormat, StdinSource};

        let mut rng = crate::convert::Xorshift64::new(7);
        let samples: Vec<i16> = (0..ALAC_PCM_SAMPLES * 40)
            .map(|_| rng.next_u64() as i16)
            .chain([i16::MIN, i16::MAX, 0, -1, 1, i16::MIN + 1])
            .collect();
        let bytes: Vec<u8> = samples.iter().flat_map(|s| s.to_le_bytes()).collect();
        let mut source =
            StdinSource::new(std::io::Cursor::new(bytes), StdinFormat::S16Le, 44100, 2);

        let metrics = Arc::new(PipelineMetricsTracker::new(DEFAULT_RING_BUFFER_PACKETS));
        let mut proc = PipelineProcessor::new(44100, metrics.clone(), None).unwrap();
        proc.disable_drift_compensation();
        let mut packets = Vec::new();
        while let Some(chunk) = source.read_chunk(441).unwrap() {
            // A back-pressured ring sits full; this must not steer the rate.
            proc.process_chunk(&chunk, DEFAULT_RING_BUFFER_PACKETS, &mut packets)
                .unwrap();
        }
        proc.flush(&mut packets).unwrap();

        let out: Vec<i16> = packets.concat();
        assert_eq!(&out[..samples.len()], &samples[..]);
        assert!(out[samples.len()..].iter().all(|&s| s == 0));
        assert_eq!(metrics.snapshot().applied_correction_ppm, 0.0);
    }

    /// A source whose jitter queue grows makes the resampler consume faster; until it reports
    /// a level, the passthrough is kept.
    #[test]
    fn test_pipeline_steers_by_the_source_queue() {
        let metrics = Arc::new(PipelineMetricsTracker::new(DEFAULT_RING_BUFFER_PACKETS));
        let mut proc = PipelineProcessor::new(44100, metrics.clone(), None).unwrap();
        proc.disable_drift_compensation();
        let mut packets = Vec::new();
        let mut at = Instant::now();
        for i in 0..400 {
            // Steady through the settling time, then growing.
            let level = (i >= 10).then_some(if i < 250 {
                6_615
            } else {
                6_615 + (i - 250) * 50
            });
            proc.set_source_queue(level);
            let chunk = AudioChunk::new(vec![0.25f32; 882], 44100, 2, at);
            at += chunk.duration();
            proc.process_chunk(&chunk, DEFAULT_RING_BUFFER_PACKETS, &mut packets)
                .unwrap();
            if i < 10 {
                assert_eq!(metrics.snapshot().applied_correction_ppm, 0.0);
            }
        }
        assert!(metrics.snapshot().applied_correction_ppm > 0.0);
    }
}
