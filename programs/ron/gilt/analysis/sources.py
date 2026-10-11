"""Write refs/SOURCES.md from refs/manifest.json (written by fetch_refs.py)."""
import json
import os

import lab

USE = {
    "kiss": "palette, coverage, luma, zones, field texture, cells (man's robe, woman's dress)",
    "adele1": "palette, coverage, luma, zones, field texture, motif sizes (manual)",
    "stoclet_fulfillment": "zones, cell geometry (ovals, checker block), palette of the cartoon",
    "stoclet_expectation": "coverage, luma, motif tallies (stacked triangles)",
    "stoclet_treeoflife": "coverage, luma, motif tallies (spirals)",
    "danae": "coverage, luma, enamel pool (violets)",
    "judith1": "coverage, luma (canvas cropped inside the frame)",
    "fritza_riedler": "coverage, luma",
    "hope2": "coverage, luma, enamel pool",
    "water_serpents1": "coverage, luma, enamel pool",
    "beethoven_kiss": "visual reference only (in-situ photo: perspective and room light)",
}

m = json.load(open(os.path.join(lab.REFS, "manifest.json")))
L = ["# Reference images: sources", "",
     "Downloaded by `analysis/fetch_refs.py` from Wikimedia Commons (scaled copies of the",
     "originals; re-run the script to re-fetch). The image files are git-ignored; this file and",
     "`manifest.json` are the record.", "",
     "| file | Commons page | original px | licence | used for |", "|---|---|---|---|---|"]
for k, v in m.items():
    L.append(f"| {k}.jpg | [{v['title'][5:]}]({v['page']}) | {v['original_size'][0]}x{v['original_size'][1]} "
             f"| {v['licence']} | {USE.get(k, '')} |")
L += ["", "Notes",
      "- The three Stoclet images are Klimt's full-size working cartoons (MAK Vienna): gouache,",
      "  gold and silver paint on paper, not the executed mosaic. Their ground is tan paper.",
      "- Colour grading differs between reproductions; palette numbers are reproduction-dependent",
      "  to a few dE. The Kiss copy is the Google Art Project scan (Belvedere).",
      "- beethoven_kiss.jpg is a 2017 photograph (CC BY-SA 4.0, credit on its Commons page);",
      "  the painting itself is public domain."]
open(os.path.join(lab.REFS, "SOURCES.md"), "w").write("\n".join(L) + "\n")
print("\n".join(L))
