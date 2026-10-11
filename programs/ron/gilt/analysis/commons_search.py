"""Search Wikimedia Commons File: namespace and list candidate images with pixel size."""
import json, sys, urllib.parse, urllib.request

UA = "GiltStyleLab/0.1 (personal research; bluecondition)"
API = "https://commons.wikimedia.org/w/api.php"

def api(**params):
    params["format"] = "json"
    url = API + "?" + urllib.parse.urlencode(params)
    req = urllib.request.Request(url, headers={"User-Agent": UA})
    with urllib.request.urlopen(req, timeout=30) as r:
        return json.load(r)

def search(q, n=8):
    d = api(action="query", generator="search", gsrsearch=q, gsrnamespace=6,
            gsrlimit=n, prop="imageinfo", iiprop="size|mime")
    pages = d.get("query", {}).get("pages", {}).values()
    out = []
    for p in pages:
        ii = (p.get("imageinfo") or [{}])[0]
        out.append((p.get("index", 0), p["title"], ii.get("width"), ii.get("height"),
                    round((ii.get("size") or 0) / 1e6, 1), ii.get("mime")))
    return sorted(out)

if __name__ == "__main__":
    for q in sys.argv[1:]:
        print(f"## {q}")
        for row in search(q):
            print("  ", row[1:])
