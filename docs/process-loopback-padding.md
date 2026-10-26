# Faint ticks every ~45 s: Windows process loopback padding

Fork-only notes (not for upstream), written 2026-10-26 after a long debugging session. The problem is still open.

## Symptom

On Difei's Windows 11 PC, streaming to a HomePod mini ("Difei's Bedroom") in the default **per-app capture** mode, there is a faint tick or crackle about every **44.6 s**. It's easy to miss in quiet passages and audible in louder ones. It happens with any player (Spotify and Chrome were both tested) and with any latency profile.

## Cause (measured, not guessed)

- Windows **process loopback** (`ActivateAudioInterfaceAsync` with `PROCESS_LOOPBACK`, which `ProcessLoopbackSource` uses) runs on the **system timer** (QPC). The capture-clock DLL measured it at ±10 ppm against `Instant`.
- The apps render to the real output device, which on this PC runs **670 ppm slow** against QPC. That was measured through **endpoint loopback** (learned −670.7 ppm, steady) and through the output's `IAudioClock` (−671 ppm).
- So process loopback consumes app audio 0.067% faster than the apps produce it. Every 30 ms ÷ 0.00067 ≈ **44.8 s** it runs dry and inserts **30 ms of exact digital zeros** (1440 frames at 48 kHz, about 3 periods).
- The padding is **not flagged**: no `AUDCLNT_BUFFERFLAGS_SILENT`, no `DATA_DISCONTINUITY`, and device positions are continuous (`skipped=0`). No app audio is lost; it continues right after the padding.

### Evidence

- A recording of the exact audio sent to the speaker (see Tools) showed 28–31 ms near-silent holes (dither level, |sample| ≤ 4) about every 44.6 s, mid-music, for over an hour, across sessions and restarts.
- The phase isn't locked to our stream start or to any of our timers: it stays the same with now-playing and remote control turned off.
- **Endpoint loopback** (`capture_mode = "endpoint"`) had **zero** gaps in 4 minutes with music playing 94% of the time; we'd expect about 5.
- With per-app capture, the audio log's `win_silent=0` and `skipped=0` at every gap: Windows hands the zeros over as normal audio.

## Constraints from Difei (hard)

- **Must stay on per-app capture with "Mute this PC while streaming".** Endpoint loopback was rejected:
  - its level follows the Windows volume (25% Windows volume means a quiet stream);
  - pressing a volume key unmutes the speakers;
  - Difei wants 25% Windows volume when not streaming.
- No workarounds that touch the Windows volume.
- Keep fixes small and separate. Upstream PRs (`pStrikeZ/Nyx-Refrain`) must be one independent feature each, with no diagnostics. Diagnostics stay on this fork.

## What was tried and reverted (learn from these)

1. **Capture-clocked sender** (9f0b5dc, reverted in e6e0584): a DLL on capture timestamps, packets sent as soon as ready, sync packets carrying the capture clock. The receiver *does* follow a sender clock that runs off nominal; that was verified with the RTP clock 408 ppm slow for 5 minutes, clean. But the rewrite didn't touch the padding, so it fixed nothing the user heard. It was too large for the gain.
2. **Padding cutter, version 1** (unpushed, discarded): a silent render stream on the default output read `IAudioClock` (device position / frequency) into a DLL. The feeder clocked the output at the device rate and cut runs of exact zeros (≥ 1 ms, ≤ 60 ms) inside sound.
   - **What worked:** four paddings in 3 minutes were cut, exactly 1440 frames each, and none were left in the sent audio.
   - **What broke the sound on the Low profile:**
     - it cut every zero run regardless of how far ahead the stream was, so a padding that came early (some came right after 20 ms late capture wake-ups, not on the 45 s schedule) left the stream up to 55–62 ms behind schedule, and the receiver dropped those packets;
     - it nibbled 1–15 ms zero runs inside quiet fades, which damaged the music.
3. **Padding cutter, version 2** (unpushed, discarded): it capped cuts at "ahead + 5 ms" and set the minimum run to 8 ms. It **never cut anything** (`pad_cut=0` throughout), and a padding at 19:58:48 went out whole. The cause wasn't found. The suspect: the device-clock DLL kept resetting (`learned=+0.0` flickered in the log), which made "ahead" wrong.

## If you try a fix

- **The cleanest idea:** keep per-app capture and cut the padding.
  - The zeros are exact (`0.0f32` on every channel).
  - Each run is about 30 ms and sits inside non-silent audio.
  - Cutting one loses nothing.
- **The rate problem:** after cutting, the content runs at the **device rate (−670 ppm)**, while the current timer-paced sender runs at QPC. The existing fill-level controller (`pipeline::drift`) is clamped at `MAX_DRIFT_CORRECTION_PPM = 500`, which is less than 670, so on its own it would drain the queue and starve. Either raise the clamp to about 1000 ppm or clock the sender from the device.
- **The step problem:** one cut removes about 3.75 packets at once, while the live queue target is only 3 packets (`LIVE_TARGET_FILL_PACKETS`). Cutting padding on the current path would starve the sender at each cut unless the target is raised or the cut is spread out. Version 1 avoided this because it sent packets as they were ready and kept time by the device clock.
- **Stay ahead of schedule:**
  - never let a cut put the stream behind its schedule;
  - on Low (sync −140 ms) the slack is under 100 ms, and packets 50 ms late get dropped;
  - test on **Low**, not Stable.
- **Leave music alone:** don't touch zero runs shorter than one 10 ms period. Real silence (track gaps, pauses) is longer than 60 ms and should pass through.
- **Device clock:** if you use `IAudioClock` on a silent render stream, check that its DLL is stable before trusting it. Version 2's resets are the main suspect.
- **Something else observed:** the app didn't always write `local_muted = true` to `gui.toml` after muting. If the app is killed while streaming, the speakers may stay muted and the next run treats that as the user's own mute.

## Tools (this fork)

- `NYX_AUDIO_STATS=1` writes `%TEMP%\nyx-refrain-audio.log`: one line per second, appended across restarts.
  - `win_silent`: frames Windows flagged silent.
  - `skipped`: device-position gaps.
  - `clipped` and `peak`: clipping and level.
  - `learned` / `nudge`: drift steering.
  - `late`: send lateness.
  - `starved` / `trimmed`: queue events.
- `NYX_RECORD_SENT=1` writes `%TEMP%\nyx-refrain-sent-<ms>.wav` (one per stream; 44.1 kHz s16 stereo, exactly what is sent) plus a `.txt` with the Unix time of its first sample. It stops after an hour (about 635 MB per file).
- `scripts/find_clicks.py [HH:MM[:SS] ...]` scans the latest recording for dropouts (≥ 1 ms at dither level inside sound), sharp jumps and clipping, and prints local times. With no arguments it scans the whole file; with times, it scans ±40 s around each. It's pure Python, with no numpy. It has been checked against planted clicks.
- Run the app from `target\release\nyx-refrain.exe` with those variables set. To tell "made before the network" apart from "made after", match the times the user reports against the recording first, before changing any code.
