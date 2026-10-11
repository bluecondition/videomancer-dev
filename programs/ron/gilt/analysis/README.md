# GILT style lab (Phase 1 analysis; Phase 2+ conformance tests)

Python 3 venv with numpy, pillow, scipy, scikit-image:

    python3 -m venv ~/.venvs/giltlab && ~/.venvs/giltlab/bin/pip install numpy pillow scipy scikit-image

Run from this directory, in order:

| step | script | output |
|---|---|---|
| fetch references | `python3 -I fetch_refs.py ../refs` | `../refs/*.jpg`, `manifest.json` |
| source list | `~/.venvs/giltlab/bin/python sources.py` | `../refs/SOURCES.md` |
| zone overlays (for approval) | `~/.venvs/giltlab/bin/python overlay_zones.py` | `figs/zones_*.png` |
| palettes, coverage, luma, band energy, field texture | `~/.venvs/giltlab/bin/python measure.py` | `results/measure.json` (~2 min) |
| cell geometry | `~/.venvs/giltlab/bin/python cells.py` | `results/cells.json`, `figs/cells_*.png` |
| constants + tables | `~/.venvs/giltlab/bin/python build_spec.py` | `../style_constants.json`, `results/tables.md` |

Helpers:
- `lab.py`: loading, canvas crop, zone rasteriser, colour conversions, pixel-family
  classifier, k-means, band energy.
- `grid_view.py`: render a painting with a labelled grid, for tracing zone polygons and
  measuring motifs by hand.
- `commons_search.py`: find candidate images on Wikimedia Commons.

Inputs written by hand:
- `zones.json`: zone polygons and canvas crops.
- `manual_measurements.json`: Adele motif sizes, rims, motif tallies.
