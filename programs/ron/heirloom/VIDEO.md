# Heirloom from the video clips (phase 3, started 2026-11-04)

Ron's 2023–2024 Camera Roll holds 481 mp4 clips of the old modular video synth (1920×1080, ~30 fps, 8–30 s
each). Each clip is a patch being played: knobs turned, a shape drifting, a key breathing. This phase turns
the distinct looks in those clips into Heirloom programs, the same way the 55 jpg photos were turned into
programs in phase 1 ([REBUILD.md](REBUILD.md)) and opened up in phase 2 ([PHASE2.md](PHASE2.md)).

## What is different from a photo

A clip gives two things a still never did:

1. **The motion is known.** The strip and the m-frames around the still show exactly how the patch moved:
   which element drifts, in which direction, how fast, what breathes, what tears. Measure it (per-element
   displacement per 0.5 s between the m-frames; period of a breathing key; tear burst rate) and make the
   program's default motion match. "Logical moving parts" are no longer a guess.
2. **The control ranges are known.** Where the clip shows the same patch evolving (Ron turning a knob), the
   other candidate stills from the same clip (see `candidates.tsv`) are the far ends of that knob. A control
   that reproduces that evolution is exactly what the patch had.

Because Ron's phase-2 brief already exists, these programs are built to the phase-2 standard from the start:
element-level parameters rather than global zoom, broad ranges with the clip's look at the default, bold
switches, every control alive at any setting of the others, P12 the headline, S11 Source = Video / Osc. No
separate phase-2 round should be needed. Versions start at **v0.1.0**.

## Inputs

- Stills: `programs/ron/heirloom/photos/vid/<stem>_t<T>.png` (1920×1080 PNG), where `<stem>` is the clip
  (`WIN_YYYYMMDD_HH_MM_SS`) and `<T>` the time in seconds with `p` for the decimal point (`t15p0` = 15.0 s).
  - `<stem>_t<T>_strip.png`: eight 480×270 frames from t−1.5 s to t+2.0 s, 0.5 s apart, left to right.
  - `<stem>_t<T>_m0..m3.png`: 960×540 frames at t−0.5, t+0.5, t+1.0, t+1.5 s, for measuring motion.
- `.heirloom_work/vid/candidates.tsv`: every distinct look found in every clip (id, clip, time, clip length,
  a motion score, duplicate-of, jpg-photo match). `.heirloom_work/vid/triage.txt` holds the grades and the
  queue order. The clips themselves stay on Windows (`Pictures\Camera Roll`); re-extract any other moment
  with `.heirloom_work/vid/extract.py` or `ffmpeg` (`.heirloom_work/tools/ffm/.../ffmpeg-linux-x86_64-*`).
- Work dir per program: `.heirloom_work/rb_<name>/`, as in phase 1.
- Tools: everything in [REBUILD.md](REBUILD.md) "Inputs and tools" (`tools/py`, `palfit.py`, `hwcolor.py`,
  `cmp.py`, `sim.sh`, `check.sh`).

## Workflow

Every step of [REBUILD.md](REBUILD.md) applies, with these additions:

1. **Look** at the still, then at the strip and the m-frames. Write in `notes.md` what moves and how, and
   what Ron's knob was doing across the clip (compare the clip's other candidate stills).
2. **Measure** the still as in REBUILD.md, and measure the motion: per element, the displacement between
   m-frames (cross-correlation of a crop), rates of any breathing or cycling, tear statistics per frame.
3. **Proto**: the float prototype renders the still; a second render at +0.5 s and +1.0 s must move the way
   the m-frames do.
4. **Controls** to the phase-2 standard (PHASE2.md "What that means for each program"): per element and
   per group of repeated shapes, broad ranges, the clip's look at the defaults, a designed P12 0 % and
   100 %, switches as look-flips, Source on S11 defaulting to Video. Name every control after the element it
   drives. Render the control sheet.
5. **Integer model, RTL, GHDL render against the still, build.** Build through the shared queue:
   `MAKE_TIMEOUT=900 .heirloom_work/vid/build_queue.sh <name>` (router1: prefix
   `NEXTPNR_ROUTER="--router router1"`). It builds at most four programs at once, so a run may wait.
6. **Name** it after what the picture shows (one word), and check it is free first:
   `.heirloom_work/vid/namecheck.sh <name>`.
7. **Deliver** as in REBUILD.md step 9: `programs/ron/heirloom/<name>/` with `<name>.vhd`, `<name>.toml`,
   optional `<name>.py` + `<name>_pkg.vhd`; the SUMMARY.md text in the final message (the orchestrator writes
   it); do not edit README.md or VIDEO.md; do not commit.

SUMMARY.md gets one extra section after "Reading of the photo": **Motion in the clip**, with the measured
rates and what the program does with them.

## Budget and contracts

As in REBUILD.md: ≤ ~7,100 LC (92 %), ≤ 24 EBR, HD timing at the full 74.25 MHz, the escalator shell with
its generated avid, one latency in every mode, the blanking gate, the HD HDMI output conditioning, virtual
1080-line coordinates, hardware colour convention (U = Cr), and the running-sum resync (README "Family-wide
fix"). Start from a current phase-2 program's shell (fern, kiln, lozenge, jukebox) rather than the v0.2
shell: those already carry the resync and the block-RAM delay rings.

## Index

The photo → program rows for this phase live in [README.md](README.md) under "From the video clips".
