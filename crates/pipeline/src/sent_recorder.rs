//! Optional recording of the exact audio sent to the receiver, for telling crackles made
//! before the network from ones made after it. Set `NYX_RECORD_SENT=1` (or [`ENABLED`],
//! the app's setting) to write one
//! `%TEMP%\nyx-refrain-sent-<ms>.wav` per stream (`/tmp` on Linux), 44.1 kHz 16-bit stereo,
//! plus a `.txt` of the same name holding the Unix time of its first sample (a restarted
//! stream gets new files). Packets are sent in real time, so a position in the file maps
//! straight to the time it was sent. Each file stops after an hour (about 635 MB).

use std::fs::File;
use std::io::{BufWriter, Seek, SeekFrom, Write};
use std::sync::Arc;
use std::sync::atomic::{AtomicBool, Ordering};
use std::time::{Duration, SystemTime, UNIX_EPOCH};

use crate::pipeline::ALAC_PCM_SAMPLES;
use crate::resample::TARGET_SAMPLE_RATE;

const MAX_SECONDS: u64 = 3600;

/// Record streams started from now on, as `NYX_RECORD_SENT` does.
pub static ENABLED: AtomicBool = AtomicBool::new(false);

/// Hands packets from the sender thread to a writer thread; never blocks the sender.
pub struct SentRecorder {
    producer: rtrb::Producer<[i16; ALAC_PCM_SAMPLES]>,
    stop: Arc<AtomicBool>,
}

impl SentRecorder {
    /// A recorder if `NYX_RECORD_SENT` or [`ENABLED`] is set and the file could be created.
    pub fn from_env() -> Option<Self> {
        let env = std::env::var_os("NYX_RECORD_SENT").is_some_and(|v| !v.is_empty() && v != "0");
        if !env && !ENABLED.load(Ordering::Relaxed) {
            return None;
        }
        let created = SystemTime::now()
            .duration_since(UNIX_EPOCH)
            .unwrap_or_default()
            .as_millis();
        let path = std::env::temp_dir().join(format!("nyx-refrain-sent-{created}.wav"));
        let mut file = BufWriter::new(File::create(&path).ok()?);
        write_header(&mut file, 0).ok()?;
        let (producer, mut consumer) = rtrb::RingBuffer::<[i16; ALAC_PCM_SAMPLES]>::new(1024);
        let stop = Arc::new(AtomicBool::new(false));
        let thread_stop = stop.clone();
        std::thread::Builder::new()
            .name("nyx-refrain-recorder".into())
            .spawn(move || {
                let mut frames: u64 = 0;
                let mut since_header = 0u64;
                let mut bytes = [0u8; ALAC_PCM_SAMPLES * 2];
                loop {
                    let mut wrote = false;
                    while let Ok(pkt) = consumer.pop() {
                        if frames == 0 {
                            let start = SystemTime::now()
                                .checked_sub(Duration::from_millis(8))
                                .unwrap_or_else(SystemTime::now)
                                .duration_since(UNIX_EPOCH)
                                .unwrap_or_default();
                            let _ = std::fs::write(
                                path.with_extension("txt"),
                                format!("{:.3}\n", start.as_secs_f64()),
                            );
                        }
                        if frames >= MAX_SECONDS * TARGET_SAMPLE_RATE as u64 {
                            continue;
                        }
                        for (i, s) in pkt.iter().enumerate() {
                            bytes[2 * i..2 * i + 2].copy_from_slice(&s.to_le_bytes());
                        }
                        let _ = file.write_all(&bytes);
                        frames += (ALAC_PCM_SAMPLES / 2) as u64;
                        since_header += (ALAC_PCM_SAMPLES / 2) as u64;
                        wrote = true;
                    }
                    // Keep the header current so the file opens even if the app is killed.
                    if wrote && since_header >= TARGET_SAMPLE_RATE as u64 {
                        since_header = 0;
                        let _ = update_header(&mut file, frames);
                    }
                    if thread_stop.load(Ordering::Relaxed) {
                        let _ = update_header(&mut file, frames);
                        return;
                    }
                    std::thread::sleep(Duration::from_millis(50));
                }
            })
            .ok()?;
        Some(Self { producer, stop })
    }

    /// Queues one sent packet; dropped (never waited for) if the writer fell behind.
    #[inline]
    pub fn record(&mut self, pcm: &[i16; ALAC_PCM_SAMPLES]) {
        let _ = self.producer.push(*pcm);
    }
}

impl Drop for SentRecorder {
    fn drop(&mut self) {
        self.stop.store(true, Ordering::Relaxed);
    }
}

fn write_header(w: &mut impl Write, frames: u64) -> std::io::Result<()> {
    let data = (frames * 4).min(u32::MAX as u64 - 36) as u32;
    let rate = TARGET_SAMPLE_RATE;
    w.write_all(b"RIFF")?;
    w.write_all(&(36 + data).to_le_bytes())?;
    w.write_all(b"WAVEfmt ")?;
    w.write_all(&16u32.to_le_bytes())?;
    w.write_all(&1u16.to_le_bytes())?; // PCM
    w.write_all(&2u16.to_le_bytes())?; // stereo
    w.write_all(&rate.to_le_bytes())?;
    w.write_all(&(rate * 4).to_le_bytes())?;
    w.write_all(&4u16.to_le_bytes())?;
    w.write_all(&16u16.to_le_bytes())?;
    w.write_all(b"data")?;
    w.write_all(&data.to_le_bytes())
}

fn update_header(file: &mut BufWriter<File>, frames: u64) -> std::io::Result<()> {
    file.flush()?;
    let f = file.get_mut();
    let end = f.stream_position()?;
    f.seek(SeekFrom::Start(0))?;
    write_header(f, frames)?;
    f.seek(SeekFrom::Start(end))?;
    Ok(())
}
