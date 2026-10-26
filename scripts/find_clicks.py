"""Find clicks, dropouts and clipping in nyx-refrain-sent.wav.

usage: python find_clicks.py [HH:MM[:SS] ...]   (default: whole file)
Each time given is scanned +-40 s.
"""
import array, datetime, glob, os, sys, tempfile

tmp = tempfile.gettempdir()
RATE = 44100
# One recording per stream: (start time, wav path), oldest first.
recs = []
for txt in glob.glob(os.path.join(tmp, "nyx-refrain-sent*.txt")):
    w = txt[:-4] + ".wav"
    if os.path.exists(w):
        recs.append((float(open(txt).read()), w))
recs.sort()
if not recs:
    sys.exit(f"no recordings in {tmp}: run the app with NYX_RECORD_SENT=1 first")


def pick(when=None):
    """The recording that was running at local time `when` (latest if None)."""
    chosen = recs[-1]
    if when is not None:
        for r in recs:
            if datetime.datetime.fromtimestamp(r[0]) <= when:
                chosen = r
    return chosen


start, wav = recs[-1]
t0 = datetime.datetime.fromtimestamp(start)
frames_total = (os.path.getsize(wav) - 44) // 4


def use(rec):
    global start, wav, t0, frames_total
    start, wav = rec
    t0 = datetime.datetime.fromtimestamp(start)
    frames_total = (os.path.getsize(wav) - 44) // 4


def at(frame):
    return (t0 + datetime.timedelta(seconds=frame / RATE)).strftime("%H:%M:%S.%f")[:-3]


def read(f0, f1):
    with open(wav, "rb") as f:
        f.seek(44 + f0 * 4)
        a = array.array("h")
        a.frombytes(f.read((f1 - f0) * 4))
    return a


def scan(f0, f1):
    a = read(f0, f1)
    n = len(a) // 2
    events = []
    for ch in (0, 1):
        x = a[ch::2]
        win = 441
        acc = 0.0
        hist = [0] * win
        p1 = p2 = 0
        for i in range(n):
            s = x[i]
            if s >= 32767 or s <= -32768:
                events.append((f0 + i, "clip", ch, s))
            d2 = abs(s - 2 * p1 + p2)
            mean = acc / win
            if i > win and d2 > 300 and d2 > 12 * (mean + 20):
                events.append((f0 + i, "jump", ch, round(d2 / (mean + 1), 1)))
            acc += d2 - hist[i % win]
            hist[i % win] = d2
            p2, p1 = p1, s
    # dropouts: >= 1 ms of digital silence (dither level) on both channels inside non-silent audio
    run = 0
    for i in range(n):
        if abs(a[2 * i]) <= 4 and abs(a[2 * i + 1]) <= 4:
            run += 1
        else:
            if run >= 44:
                j0 = i - run
                before = max(abs(v) for v in a[max(0, 2 * (j0 - 441)):2 * j0] or [0])
                after = max(abs(v) for v in a[2 * i:2 * (i + 441)] or [0])
                if before > 200 and after > 200:
                    events.append((f0 + j0, "dropout", -1, round(run / RATE * 1000, 1)))
            run = 0
    events.sort()
    # merge events closer than 5 ms
    merged = []
    for e in events:
        if merged and e[0] - merged[-1][0] < 220 and e[1] == merged[-1][1]:
            continue
        merged.append(e)
    return merged


def when_of(hms):
    parts = [int(p) for p in hms.split(":")]
    while len(parts) < 3:
        parts.append(0)
    return t0.replace(hour=parts[0], minute=parts[1], second=parts[2], microsecond=0)


jobs = []
if len(sys.argv) > 1:
    for hms in sys.argv[1:]:
        jobs.append(when_of(hms))
else:
    jobs.append(None)
for when in jobs:
    use(pick(when))
    print(f"recording {os.path.basename(wav)}: {at(0)} to {at(frames_total)}")
    if when is None:
        f0, f1 = 0, frames_total
    else:
        c = int((when - t0).total_seconds() * RATE)
        f0, f1 = max(0, c - 40 * RATE), min(frames_total, c + 40 * RATE)
    if f1 <= f0:
        print("  not in this recording")
        continue
    print(f"-- {at(f0)} to {at(f1)}")
    ev = scan(f0, f1)
    for fr, kind, ch, v in ev:
        detail = {"clip": f"value {v}", "jump": f"{v}x the local level",
                  "dropout": f"{v} ms of silence"}[kind]
        side = "" if ch < 0 else (" L" if ch == 0 else " R")
        print(f"{at(fr)}  {kind}{side}  {detail}")
    if not ev:
        print("nothing found")
