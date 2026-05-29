LZX Videomancer Program Prompt — "Glitch Cascade on Teal"

Overall Concept

Create a Videomancer program that generates a central, ragged, irregularly-shaped mass of vertical datamosh/pixel-sort glitch streaks floating on a flat, calm, solid teal/cyan background. The composition is a high-contrast juxtaposition: a chaotic, luminous, multicolored "wound" or "explosion" of vertical data against a still, almost oceanic backdrop. Think "corrupted VHS or JPEG compression artifact suspended in mid-air." The effect should feel like a broken data stream bleeding downward while white-hot lightning veins crawl across it.

Canvas \& Layout



Aspect ratio: roughly square, slightly wider than tall (approximately 1:0.9).

Background: flat, untextured teal-cyan fill. No gradient, no noise, no scanlines on the background itself — it must remain pristine and still, emphasizing the glitch mass by contrast. Target color: roughly RGB (60, 175, 185) — a muted, slightly desaturated tropical/ocean cyan, leaning green. It should read as calm and slightly dim, NOT neon or vibrant.

Central mass placement: the glitch region is roughly centered but irregular. It occupies roughly 65–75% of the canvas width, horizontally centered, with the background showing as a clean margin on the left (\~8–12%), right (\~8–12%), and top (\~3–8%). The glitch region extends nearly to the bottom edge, with long vertical streaks reaching the very bottom.

Silhouette of the mass: NOT a rectangle. The top edge is jagged and mountainous/city-skyline-like — short spikes and tall peaks that look torn or pixel-shifted upward. The left and right edges are ragged with vertical column fragments of varying heights sticking out like stalactites/stalagmites. The bottom dissolves into long vertical streaks/drips that extend downward into the background like melting or bleeding data.



Structural Pattern — Vertical Pixel Sort / Datamosh

The defining structural element is dense vertical striping:



Every pixel column inside the mass is stretched vertically, as if each column was sampled from a single horizontal row and smeared down the Y-axis.

Stripe width varies from 1–4 pixels; they are fine, not chunky.

Some columns extend the full height of the mass; others are shorter, creating the uneven top/bottom silhouette.

The pattern reads as a pixel-sort effect (think Kim Asendorf-style sorting) combined with horizontal slice displacement — data dragged vertically with occasional horizontal glitch seams.

Horizontal banding is subtle but present: faint horizontal discontinuities suggest rows where the sorting "resets" or where data shifts sideways.



Color Palette Inside the Glitch Mass

The interior is a rich, jewel-toned, high-saturation explosion of competing colors. Colors are NOT blended — they sit in discrete vertical bands. Dominant hues, roughly in order of coverage:



Deep magenta / hot pink — strongest on the mid-left region, large blocks of saturated pink-magenta stripes.

Crimson / blood red — concentrated in a roughly ovular hot zone on the lower-left-of-center, glowing brightly.

Cobalt / royal blue and indigo — scattered throughout, especially upper-center and right.

Deep violet / purple — transitional columns between red and blue.

Emerald / forest green — speckled columns, mainly right side and scattered.

Amber / orange / yellow — smaller accents, particularly around the lower-left red zone and as thin column highlights.

Near-black / deep maroon — columns interspersed throughout as darker "shadow" stripes providing contrast.

White / pale cyan highlights — blown-out hot spots (see luminance section).



Saturation is high (\~80–95%) everywhere in the mass except the bright hot spots which wash out to white. The color distribution is chaotic but has loose regional tendencies: reds/pinks dominate the left and lower-left, blues/purples dominate the upper and center, greens scatter on the right.

Luminance \& Hot Spots



There are 2–3 major blown-out white/cyan-white hot spots inside the mass where the glitch is brightest:



A large irregular bright region in the center-right, bleeding into almost pure white.

A smaller bright zone in the lower-center.

A secondary bright patch in the middle-left, near the red/pink transition.





These hot spots look like overexposed CRT blooming — they desaturate to white at their core and fade through pale cyan/pink halos back into saturated color.

Outside these hot spots, overall luminance of the mass is mid-to-high but the colors hold their saturation.



Overlay — The Lightning / Circuit Veins (Critical Element)

Layered ON TOP of the vertical glitch mass is a web of fine, wispy, bright-white branching lines that resemble lightning, river deltas, neural networks, or circuit traces:



Color: pure white with occasional pale yellow, pale cyan, or pale pink tinting where they cross colored regions.

Line weight: very thin, 1–2 pixels, with slight glow/bloom around them.

Pattern: fractal, dendritic branching — main trunks split into smaller branches, which split again. Organic, not geometric. They flow more horizontally/diagonally than the vertical stripes, providing cross-directional contrast.

Density: densest in the upper-left and middle regions; sparser toward the edges and bottom.

They do NOT extend outside the glitch mass into the teal background — they are contained within or just barely at the ragged edges of the glitch.

Think: static-electricity arcs, Lichtenberg figures, topographic river maps overlaid on the chaos.



Texture \& Detail Notes



Slight compression artifacts / JPEG mosquito noise around high-contrast edges, especially where the mass meets the teal background.

A few scattered black dots/specks in the upper-left teal background area (tiny, 2–5 px) — like dust or dead pixels.

The bottom edge of the mass features long thin vertical "drips" — isolated single-column streaks of red, pink, blue, and white that extend well below the main mass into the teal.

No visible text, logos, or recognizable shapes. Pure abstract.



Mood \& Aesthetic References



Corrupted video memory, databending, ANSI pixel-sort art

Kim Asendorf "ASDF Pixel Sort" aesthetic

Glitch art scene (Rosa Menkman, Phillip Stearns)

LZX modular video synthesis corruption patches

Lightning over a neon city at night, compressed into a data stream

Frozen moment of a CRT tearing itself apart



LZX Videomancer Implementation Guidance

Work within the Videomancer's limitations. Prioritize in this order if constraints force compromises:



Flat teal background — must stay clean and still.

Vertical striping / pixel-sort appearance of the central mass — the defining texture.

Ragged, non-rectangular silhouette of the mass with vertical drips extending downward.

High-saturation jewel-tone color distribution (magenta-left, blue-center, scattered greens, red hot zone lower-left).

White blown-out hot spots at 2–3 interior locations.

White lightning/dendritic overlay — if the synth cannot do this directly, approximate with a high-frequency noise or edge-detect pass modulated into a luma key.

Compression/edge artifacts and drip columns — nice-to-have polish.



Suggest appropriate Videomancer modules, shaders, oscillators, keyers, and feedback paths to achieve each layer. If any element exceeds the synth's capability, propose the closest analog and note what's been simplified.

