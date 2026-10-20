# Dicefall

Many small dice tumbling down the screen. Each is a random colour and carries its numbers, with every outline and crease exactly anti-aliased. Each falls and tumbles about its own random spin, and comes back at the top as a new die.

Built on NAT20's renderer (`programs/ron/nat20`), with many dice instead of one.

**P12 (Fall):** how fast the dice fall, from a slow drift to a quick rain. They never stop.

## Controls

| | Control | Range | What it does |
|---|---|---|---|
| K1 | Count | 0–100% | How many dice: up to 20 at HD (an even number), up to 4 at SD (6 with the mix) |
| K2 | Tumble | 0–100% | How fast the dice spin; 0 freezes their spin, not their fall |
| K3 | Light | 0–100% | Key light position, raking-left to raking-right |
| K4 | Size | 4–11% | Die diameter as a share of the frame height; smaller dice fit more of them |
| S7 | Dice | d20 / Mix | All d20s, or a random mix of d4, d6, d8, d10 and d20 |
| S8 | Backdrop | Gray / Black | Black, or a soft gray gradient behind the dice |
| P12 | Fall | 0–100% | Fall speed |

## Technical notes

- Program type: processing; core `yuv444_30b`; input-to-output latency 12 clocks.
- Sizes follow the frame height, so the picture is the same shape at SD, 720p and 1080p. SD shows fewer dice: it has about a fifth of HD's clocks a field to compute them.
- No more than two dice share a scanline: the dice fall at one speed with evenly staggered starts, alternating between the left and right halves.
- Built and timed at the full 74.25 MHz HD pixel clock.

## License

GPL-3.0-only. Copyright (C) 2026 czartroniks.
