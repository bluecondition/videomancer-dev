"""Download the Phase 1 reference images from Wikimedia Commons (scaled copies)
and record provenance (source page, original size, licence, credit) in a JSON
manifest that SOURCES.md is generated from.

usage: python3 -I fetch_refs.py <refs_dir>
"""
import json, os, sys, time, urllib.parse, urllib.request

UA = "GiltStyleLab/0.1 (personal research; bluecondition)"
API = "https://commons.wikimedia.org/w/api.php"

# key, Commons title, requested width (scaled copy; long side ~3-5k px)
REFS = [
    ("kiss",          "File:The Kiss - Gustav Klimt - Google Cultural Institute.jpg", 3840),
    ("adele1",        "File:Gustav Klimt - Porträt der Adele Bloch-Bauer I (1907).jpg", 2801),
    ("stoclet_fulfillment", "File:KlimtDieUmarmung.jpg", 2560),
    ("stoclet_expectation", "File:Klimt - Expectation.jpg", 2560),
    ("stoclet_treeoflife",  "File:Klimt - Nine Cartoons for the Stoclet Frieze- Part 4, Part of the tree of life (1910-11).jpg", 2560),
    ("danae",         "File:Klimt - Danae - 1907-08.jpeg", 2694),
    ("judith1",       "File:Judith 1.jpg", 1769),
    ("fritza_riedler","File:Gustav Klimt - Fritza Riedler - 3379 - Österreichische Galerie Belvedere.jpg", 3067),
    ("hope2",         "File:Gustav Klimt - Hope, II - Google Art Project.jpg", 3648),
    ("water_serpents1","File:Klimt - Wasserschlangen I (Freundinnen).jpeg", 1662),
    # in-situ photo (angle + room light): motif tallies only, not palette
    ("beethoven_kiss", "File:Wiener Secessionsgebäude. Beethovenfries. Diesen Kuss der ganzen Welt. 2017-10-01 11-29-28.jpg", 3840),
]

def get(url, tries=6):
    for i in range(tries):
        try:
            req = urllib.request.Request(url, headers={"User-Agent": UA})
            with urllib.request.urlopen(req, timeout=120) as r:
                return r.read()
        except urllib.error.HTTPError as e:
            if e.code == 429 and i < tries - 1:
                time.sleep(10 * (i + 1)); continue
            raise

def info(title, width):
    q = dict(action="query", format="json", titles=title, prop="imageinfo",
             iiprop="url|size|extmetadata", iiurlwidth=str(width))
    d = json.loads(get(API + "?" + urllib.parse.urlencode(q)))
    page = next(iter(d["query"]["pages"].values()))
    return page["imageinfo"][0]

def strip(s):
    import re
    return re.sub(r"<[^>]+>", "", s or "").strip()

def main(refs_dir):
    os.makedirs(refs_dir, exist_ok=True)
    man_path = os.path.join(refs_dir, "manifest.json")
    manifest = json.load(open(man_path)) if os.path.exists(man_path) else {}
    for key, title, width in REFS:
        out = os.path.join(refs_dir, key + ".jpg")
        if key in manifest and os.path.exists(out):
            continue
        ii = info(title, width)
        md = ii.get("extmetadata", {})
        url = ii.get("thumburl") or ii["url"]
        data = get(url)
        open(out, "wb").write(data)
        manifest[key] = dict(
            title=title, page=ii.get("descriptionurl"), file_url=url,
            original_size=[ii.get("width"), ii.get("height")],
            artist=strip(md.get("Artist", {}).get("value")),
            date=strip(md.get("DateTimeOriginal", {}).get("value")),
            credit=strip(md.get("Credit", {}).get("value"))[:200],
            licence=strip(md.get("LicenseShortName", {}).get("value")),
            bytes=len(data))
        json.dump(manifest, open(man_path, "w"), indent=1, ensure_ascii=False)
        print(f"{key}: {len(data)/1e6:.1f} MB  licence={manifest[key]['licence']}")
        time.sleep(4)

if __name__ == "__main__":
    main(sys.argv[1])
