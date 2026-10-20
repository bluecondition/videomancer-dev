# STARWIPE patterns (v0.3.0)

A wipes to B on the T-bar (P12). Pattern (K1) selects within the Bank (S7); Modifier (K2) is per pattern;
Origin (K3) places the anchor of every centred pattern (centre, then clockwise from the top-left corner through
the edge midpoints, with travel between stops); Edge (K4) is soft to the left of centre, a border to the right;
Border Color (K5) white / hue wheel / black / gold; B Source (K6) black / hue wheel / white; Reverse (S8)
runs B out instead of in; Spin (S9) turns the pattern 1.25 turns over the travel; Auto (S11) sweeps the
T-bar there and back on its own with the T-bar as the rate (~22 s a cycle at 0 %, ~0.6 s at 100 %, each end
held bit-exact for ~6 % of the cycle). Repeat (S10) is not yet implemented.

Edge-launched patterns ignore Origin (they enter from the screen edges). All ends are bit-exact A below 2 %
and bit-exact B above 98 % of the travel.

## CLASSIC (S7 off), 21 patterns

| # | Pattern (K1 %) | sweep | Modifier (K2) |
|---|---|---|---|
| 1 | MIX (0-5) | dissolve | none (dissolve) |
| 2 | BAR > (5-10) | edge-launched bar | tilt +-45 deg |
| 3 | BAR v (10-14) | edge-launched bar | tilt +-45 deg |
| 4 | DIAGONAL (14-19) | edge-launched bar | tilt +-45 deg |
| 5 | BARN DOOR <> (19-24) | edge-launched bar | tilt +-45 deg |
| 6 | BARN DOOR ^v (24-29) | edge-launched bar | tilt +-45 deg |
| 7 | BARN DOOR diag (29-33) | edge-launched bar | tilt +-45 deg |
| 8 | X DOORS (33-38) | edge-launched, union of two bars | crossing angle 22-68 deg (hourglass to flat X) |
| 9 | BOX (38-43) | size sweep from the origin | aspect, 1/2 to 2x the screen |
| 10 | DIAMOND (43-48) | size sweep from the origin | aspect |
| 11 | CIRCLE (48-52) | size sweep from the origin (q-form) | ellipse squash (tall to wide) |
| 12 | EYE (52-57) | size sweep from the origin (q-form) | lens width |
| 13 | CROSS (57-62) | size sweep from the origin | arm thickness |
| 14 | VEE (62-67) | edge-launched chevron | arm angle 17-50 deg |
| 15 | BARN VEE (67-71) | edge-launched chevron | arm angle 17-50 deg |
| 16 | TRIANGLE (71-76) | size sweep from the origin | base notch (arrowhead) on the right half |
| 17 | POLYGON (76-81) | size sweep from the origin | sides 3-12 (noon 6) |
| 18 | CLOCK (81-86) | angular sweep about the origin | start angle |
| 19 | PINWHEEL (86-90) | angular sweep about the origin | blades 2-8 (noon 4) |
| 20 | DOUBLE FAN (90-95) | angular sweep about the origin | none |
| 21 | DOUBLE SWEEP (95-100) | angular sweep about the origin | start angle |

## CHEESE (S7 on), 4 patterns

| # | Pattern (K1 %) | sweep | Modifier (K2) |
|---|---|---|---|
| 1 | STAR WIPE (0-25) | size sweep from the origin | points 3-12 (noon 5) |
| 2 | SUNBURST (25-50) | size sweep from the origin | rays 4-12 (noon 8) |
| 3 | APERTURE (50-75) | size sweep from the origin | blades 5-9 (noon 7) |
| 4 | SPOTLIGHT (75-100) | size sweep from the origin (q-form) | none |

## Not built (brief)

CLASSIC: SLIDE, FOUR BOX, DOUBLE DIAMOND, ROUND RECT, BOW TIE, ZIGZAG, BARN ZIGZAG, SALOON DOOR, WINDSHIELD,
BLINDS, CHECKERBOARD, SNAKE, SPIRAL BOXES, WATERFALL, MOSAIC, ROULETTE. CHEESE: STARBURST, HEART, KEYHOLE,
FLOWER, GEAR, CLOUD, SPLAT, SPIRAL, VORTEX, SHOCKWAVE, RADAR, TV OFF, THEATER CURTAIN, PAGE PEEL, WAVY
CURTAIN, LIGHTNING, MELT, PLASMA, HALFTONE, SWISS CHEESE, STARS, HONEYCOMB, TRIANGLES, HARLEQUIN, PIXEL
DISSOLVE. The ray engine draws star-shaped regions from one anchor (or a union / intersection of two): the
periodic and textured ones (blinds, checkerboard, mosaic, halftone, cheese, honeycomb...) need a different
engine; ROUND RECT, HEART, KEYHOLE, FLOWER fit the two-accumulator union and are the next candidates.
