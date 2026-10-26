# Which output causes the padding (2026-10-26)

Follow-up to [process-loopback-padding.md](process-loopback-padding.md).

- **The default output is the LG HDR 4K monitor**, which gets its audio from the AMD GPU over HDMI/DisplayPort ("AMD High Definition Audio Device"). Monitor audio doesn't have its own clock: it's derived from the video timing, so being 670 ppm off is plausible there (inferred, not proven).
- **Test:** clean master (cdf8fea, without Astra's work-in-progress), per-app capture, the HyperX Cloud III S Wireless set as the default output, Chrome playing.
  - Recorded with `NYX_RECORD_SENT=1` and scanned with `scripts/find_clicks.py`.
  - **0 padding gaps in 317 s**, with music audible 98% of the time.
  - With the LG monitor as the output, the same 317 s would have about 7 gaps (one every ~44.8 s).
- **Conclusion:** the ticks come from the LG/HDMI output's clock running about −670 ppm against the system timer. Windows' per-app capture pads any mismatch, but with a normal clock the padding is rare.
- **Workaround with no code:** use an output with a better clock while streaming (the PC speakers are muted then anyway). A code fix still has to work when the output is the monitor.
