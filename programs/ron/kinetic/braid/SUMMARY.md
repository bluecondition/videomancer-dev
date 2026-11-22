# Braid (Kinetic, wave 2)

**v0.1.0 built 2026-11-22, not flashed.** Flash `out/rev_b/ron/kinetic/braid.vmprog`.

## What it draws

Plaited cord: three-strand braids, rope sennit, cable-knit and Celtic plaitwork. Vertical plaits stand side
by side across the screen. Each strand is a round lit tube (bright along its centre line, dark at its edges,
a contact shadow on the strand beneath where another crosses over it), the three strands in the palette's
three colours, with a true over-and-under: the strand on top always cuts the one under it cleanly, with a
dark rim at the crossing. The default frame (Terra palette) shows about six and a half classic braids on a
dark ground. The braids flow steadily along their length and a travelling wave makes neighbouring plaits
slip against each other.

## Element and state

Element: a plait (column i, pitch P = 2H; plait 0 on the screen centre, or half a pitch off it with Radial
or Mirrored so a plait boundary sits on the centre line). Strand s of plait i on line y:

    t_s   = psi_i(y) + s / 3                     (two strands: s / 2)
    x_s   = W sin 2 pi t_s                       the centre, from the plait centre
    z_s   = sin 4 pi t_s                         the height (the figure-eight; two strands: cos 2 pi t_s)
    rh_s  = R g(|cos 2 pi t_s|) + c_i            g = sqrt(1 + (A cos)^2): a tilted tube is wider across
    psi_i = K y / H + phase + Depth sin(phase + Spread index_i)

State: the plait's phase psi_i (Wave = Slip) or its radius offset (Wave = Swell). The depth order is exact:
the order of the three heights only changes where 2 psi crosses k/12, so it is a 6-entry table of that
sector. The visible strand is the highest of those within its half-width; the shade is the tube profile at
e / rh (e = the distance in from the edge), less the contact shadow of the strand directly above (a ramp over
R / 4 from that strand's edge); the edge is anti-aliased over max(1 px, the edge's slope) against black
(another strand beneath) or the ground. The thickness is capped so two strands never overlap where they swap
order (a fat strand would otherwise pop over its neighbour in one line).

## Controls

| Control | Name | What it does |
|---|---|---|
| K1 | Scale | plait pitch 160 .. 640 virtual px (a dozen cords .. three fat ropes), exponential, glided |
| K2 | Shape | strand thickness: thin cords with ground between them .. fat strands that fill the plait (capped against overlap), glided |
| K3 | Depth | wave depth, square law: Slip 0 .. 1/2 braid cycle, Swell 0 .. +-2 R, glided |
| K4 | Rate | wave travel and braid flow, bipolar, centre = still; exponential, 60 s per cycle .. 2 cycles per second |
| K5 | Colour | the 16 shared Kinetic palettes (strands A, B, accent; the ground), hysteresis; default Terra |
| K6 | Camera | camera (and Osc) strength, default 100 %: luma swells the strands of each plait; 0 = the black-input picture |
| S7 | Wave | Slip (the wave slides the plaits along their length) / Swell (the wave fattens and thins them instead) |
| S8 | Field | Linear (index i) / Radial (index about the centre line, mirror-symmetric, the wave runs out both ways) |
| S9 | Strands | Three (plait) / Two (twisted rope, fatter strands) |
| S10 | Lay | Parallel / Mirrored (alternate plaits mirrored, pairs in phase: the chevrons of a cable-knit) |
| S11 | Source | Video / Osc |
| P12 | PLAIT | amplitude and pitch together: 0 % nearly straight parallel cords with a slight weave, 50 % the classic braid, 100 % wide tight plaits whose strands touch the neighbours: one woven fabric |

Standard layout: Colour on K5, so the spec's Spread has no knob (frozen at 0.09 cycle per plait). S7 Wave
replaces Snap: a phase dwell-and-flip is invisible in a still (the braid repeats every 1/3 cycle) while Swell
is an unmistakable flip.

## Video

`program_type = "processing"`. The luma (>> 2, less 16) runs through a horizontal 1/32 IIR, is sampled as
the beam crosses each plait's centre (+22 px for the IIR's lag) and goes through a vertical 1/16 IIR per
plait (a ping-pong state RAM, so the value is held per plait over about 32 lines). That level swells all three
strands of the plait: c = span x min(1, 4 K6 l^2) through a per-field table, span = the head-room up to the
overlap cap and the room inside the plait. The subject is drawn in thick and thin rope; every edge stays a
clean curve because the swell is one smooth value per plait. Black input: c = 0, exactly the K6 = 0 picture.
Source = Osc substitutes the shell's drifting 640 x 540 triangle lattice at 3/4 of full swing (about twice
the camera's typical level).

## Engine

- **Per field** (vblank zone, "no avid for 1024 clocks"): five knob glides; about 16 products on ONE
  pipelined 15 x 12 multiplier (op codes, registered once) for the braid constants as polynomials of P12 and
  the knobs (no divisions; the pitch through an exp2 ROM), the overlap cap from a ROM, the beam's start, the
  rate step; the ground colour; the colour tables (three strands x Y 128 steps, U / V 64) from the palette's
  precomputed ramp words. Then, on the same multiplier: RT (R g and the AA ramp width over |cos|, 256
  entries), WT (the camera swell by level, 256) and D (the per-plait wave, 16).
- **Per line**: slots 0 and 1 are fetched in the blanking after the line before (2 x 16 clocks, well inside
  the SD blanking); every further plait slot is fetched just in time as the beam enters the one before it:
  three phases, a SIN read and W |sin| on the multiplier for each strand centre, an RT read for its radius,
  the sector table for the order, the camera swell from the state RAM.
- **Per pixel** (stages 9 .. 29): three distances, edge distances and inside tests, the visible strand / the
  one beneath / the one above, a 7-bit restoring divider (q = e / rh), the profile ROM, the shadow ramp, the
  colour tables, the coverage blend for luma (9 levels) and a coverage threshold for chroma, then riley's
  output conditioning (chroma [1 2 2 2 2 2 2 2 1]/16, luma [1 2 1]/4 at the same group delay, luma 13/16,
  blanking gate). Block-RAM delay lines carry the syncs, the chroma 8-sums' trailing taps and the pixel state
  13 -> 21.
- Generated avid (the shell's parity-locked start), one latency (40) in every mode, virtual 1080-line
  coordinates from the timing ID, colours in the hardware convention (U register = Cr).

## Status

- v0.1.0 built 2026-11-22 through the shared queue with **router1** (`NEXTPNR_ROUTER="--router router1"`):
  router2 missed (seeds 1, 2) and stalled (seed 3, and again on the final netlist), router1 closed every config
  on **seed 1**. Not flashed.
- Post-route Fmax: HD Analog 85.97 MHz, HD HDMI 84.83, HD Dual 89.30, SD Analog 85.24, SD HDMI 86.53, SD Dual
  90.46 (HD needs 74.25, SD 27). All seed 1.
- 6,772 .. 6,798 / 7,680 LC (88 %), 20 / 32 EBR. Six bitstreams pass iceunpack. No `emulate_transparency`, no
  carry self-adds (dupin: 0), no integer-minus-signed sites (scan.py: 0).
- Rebuild: `NEXTPNR_ROUTER="--router router1" ./build_set.sh kinetic braid` (router1 is needed).
- Model = RTL (GHDL, 720p60, decimation 1): pixel diff 0 of 918,720 on the defaults (black input, K5 at the
  Terra position 672), Source = Osc, selfie.jpg as video, K5 = 0 and K5 = 1023, and a later field for motion
  (field 7: 0 pixels differ; 75 % of its pixels changed since field 3); and on both
  fields of an NTSC render (720 x 243 each) and both fields of a 1080i render (1920 x 540 each). The GHDL frame
  is the model's shifted 4 columns right with 4 blank columns at the left (the generated avid starts 4 clocks
  after the source's), as in riley.
- Renders: `.kinetic_work/rb_braid/cmp_final.png`, `sheet.png` (control sheet), `strip.png` (motion),
  `camera.png` (K6 25 / 50 / 100 % and Osc against selfie.jpg), `rtl_default.png`.
- Fit history: the first full build (7,219 LC) failed placement; the engine was cut twice more (shared adders
  in the slot fetch, a 16-bit colour pipeline replaced by an 8-bit address, delay-line tricks) and the timing
  fixed from nextpnr's critical paths (a registered op code for the multiplier's operand mux, no zone flag in
  the sequencer's decodes). Details in the work dir's notes.md.

## Cut / deviations

- Depth dimming dropped: the order is exact (sector table) but the lower strands are not darkened by depth;
  the contact shadow and the black rim at each crossing carry the over-and-under.
- Chroma at a strand's edge switches at half coverage (the output's 9-tap chroma filter softens it); luma is
  blended by coverage. Over another strand an edge blends toward black, not toward that strand's colour.
- Spread has no knob (frozen at 0.09 cycle per plait); S7 is Slip / Swell instead of Snap.
- At most 16 plait slots per line (the 17th and later would repeat slot 15; the smallest pitch shows 13).
- The camera swells all three strands of a plait together (one level per plait per line), as the spec asks
  ("held per plait"); it does not modulate along a strand within a line.
- The camera's per-plait state RAM is never reset, so the top ~30 lines of a field start from the bottom of the
  field before (a soft lag at the top edge, never a ragged one); the model mirrors it.
- The camera swell is modest at the default Shape, where the strands already sit near their overlap cap
  (little head-room); thinner Shape settings leave more room for it.

## Unsure on hardware

- Crisp lit edges in motion: riley's output conditioning ([1 2 1] luma, 9-tap chroma, luma 13/16) is built in;
  if HD HDMI still bars, widen the chroma first (wrinkle's lesson).
- The camera sample column depends on the source's hsync -> avid parity by one pixel (invisible).
