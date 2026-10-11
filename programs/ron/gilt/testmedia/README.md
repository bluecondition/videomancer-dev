# GILT test media

The Phase 2 model renders every clip here. Each look change is judged on all of them, as
contact sheets: input | GILT | nearest Klimt painting.

**Drop files here from Windows:**
`\\wsl.localhost\Ubuntu\home\ron\videomancer-dev\programs\ron\gilt\testmedia`
(paste that into the Explorer address bar; older Windows: `\\wsl$\Ubuntu\...`).

Clips are git-ignored. They stay on this machine.

## The ten clips

| file name | what | why it is in the set |
|---|---|---|
| `01_portrait` | one face, frontal, head and shoulders | flesh key, cameo, hard flesh edge |
| `02_two_people` | two people close together (hugging, side by side) | two flesh areas, one robe mass between them: The Kiss |
| `03_full_body` | one person standing, head to feet | robe silhouette following a whole figure |
| `04_group` | three or more people | many small flesh areas, ornament density |
| `05_greenery` | plants, garden, landscape | MEADOW switch, green handling |
| `06_plain_wall` | a plain interior wall or room, nobody in it | the calm FIELD on its own |
| `07_low_key` | a dark scene (dim room, night) | tone auto-levelling at the bottom |
| `08_high_key` | a bright scene (white room, overcast window) | tone auto-levelling at the top |
| `09_fast_motion` | fast movement: dancing, waving, a quick pan | stability, no boiling or flicker |
| `10_no_people` | an ordinary scene with no people (street, kitchen, objects) | composition from detail alone |

## Format

- **Any common video format** (mp4, mov, mkv, avi), or a still image (png, jpg) where
  motion doesn't matter (01, 06, 10 can be stills).
- **1080p** preferred (1920×1080). 720p is fine. Phone footage is fine.
- **5–15 seconds** each. Longer clips only slow the model down.
- **Locked-off camera** for at least 01 and 06 (tripod, or phone propped up). The
  stability test needs a still camera on a still subject.
- **Best of all:** shoot with the same camera or source you normally feed into the
  Videomancer. Its noise, white balance and skin tones are what GILT will actually see.
- Name the files as above, with any extension: `01_portrait.mp4`, `06_plain_wall.png`.
  More than one per category is welcome: `01_portrait_b.mov`.

Any category you don't supply gets filled with public-domain footage from Wikimedia
Commons, recorded in `SOURCES.md` here.
