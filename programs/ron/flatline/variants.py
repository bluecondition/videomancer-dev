#!/usr/bin/env python3
"""Stamp the FLATLINE build variants (VHDL + TOML) from the master in this directory.

    flatline          A: bilateral + edge lines, temporal vote           (K6 shadow not built)
    flatline_ink      B: median + XDoG + edge lines, temporal vote      (K6 shadow not built)
    flatline_canny    C: median + Canny-lite + edge lines, temporal     (K6 shadow not built)
    flatline_kuwa     D: Kuwahara + edge lines, temporal vote           (K6 shadow not built)
    flatline_finish   E: median + edge lines + soft + shadow + temporal

Only the C_HAS_* / C_VARIANT constants differ between the VHDL copies; the TOMLs differ in
labels, ids and descriptions and are all generated from _template.toml.  Edit the master
(flatline.vhd / _template.toml here), never the stamped copies, then run this script.
The master directory itself is build A (its flatline.toml is generated too).
"""
import pathlib, re, sys

HERE = pathlib.Path(__file__).resolve().parent
MASTER_VHD = HERE / "flatline.vhd"
MASTER_TOML = HERE / "_template.toml"      # generic template: every TOML is generated from it

BASE = dict(C_HAS_MEDIAN="false", C_HAS_BILAT="false", C_HAS_KUWA="false", C_HAS_XDOG="false",
            C_HAS_CANNY="false", C_HAS_TEMPORAL="true", C_HAS_SHADOW="false", C_HAS_SOFT="false")

VARIANTS = {
    "flatline": dict(BASE, C_VARIANT='"A"', C_HAS_BILAT="true"),
    "flatline_ink": dict(BASE, C_VARIANT='"B"', C_HAS_MEDIAN="true", C_HAS_XDOG="true"),
    "flatline_canny": dict(BASE, C_VARIANT='"C"', C_HAS_MEDIAN="true", C_HAS_CANNY="true"),
    "flatline_kuwa": dict(BASE, C_VARIANT='"D"', C_HAS_KUWA="true"),
    "flatline_finish": dict(BASE, C_VARIANT='"E"', C_HAS_MEDIAN="true", C_HAS_SHADOW="true", C_HAS_SOFT="true"),
}

TOML = {
    "flatline": dict(
        name="Flatline", desc="Flatline A: separable bilateral smoothing, flat mattes with region-boundary ink, temporal vote. Raw video on the wipe.",
        k1=["Off", "Bilateral"], k4=["None", "Edge", "Edge White", "Edge Bold"], s10=("Temporal", ["Off", "On"], "On"),
        k6="Shadow n/a", k1def="Bilateral", s7=False,
        note="# FLATLINE (build A) -- bilateral + edge lines + temporal vote.  The drop shadow,\n# soft quantization, XDoG and Canny live in the sibling builds (see DESIGN.md)."),
    "flatline_ink": dict(
        name="Flatline Ink", desc="Flatline B: median smoothing, XDoG or region-boundary ink over flat mattes, temporal vote. Raw video on the wipe.",
        k1=["Off", "Median"], k4=["None", "Edge", "Edge White", "XDoG"], s10=("Temporal", ["Off", "On"], "On"),
        k6="Shadow n/a", k1def="Median", s7=False,
        note="# FLATLINE_INK (build B) -- stamped sibling of programs/ron/flatline: median + XDoG."),
    "flatline_canny": dict(
        name="Flatline Canny", desc="Flatline C: median smoothing, Canny-lite (NMS + hysteresis) or region-boundary ink over flat mattes. Raw video on the wipe.",
        k1=["Off", "Median"], k4=["None", "Edge", "Edge White", "Canny"], s10=("Chroma Grad", ["Off", "On"], "On"),
        k6="Shadow n/a", k1def="Median", s7=False,
        note="# FLATLINE_CANNY (build C) -- stamped sibling of programs/ron/flatline: median + Canny-lite.\n# S10 is the Canny chroma gradient here (no temporal store)."),
    "flatline_kuwa": dict(
        name="Flatline Kuwa", desc="Flatline D: Kuwahara (range criterion) smoothing, flat mattes with region-boundary ink, temporal vote. Raw video on the wipe.",
        k1=["Off", "Kuwahara"], k4=["None", "Edge", "Edge White", "Edge Bold"], s10=("Temporal", ["Off", "On"], "On"),
        k6="Shadow n/a", k1def="Kuwahara", s7=False,
        note="# FLATLINE_KUWA (build D) -- stamped sibling of programs/ron/flatline: Kuwahara 5x5."),
    "flatline_finish": dict(
        name="Flatline Finish", desc="Flatline E: median smoothing, soft quantization, drop shadow, temporal vote, region-boundary ink over flat mattes.",
        k1=["Off", "Median"], k4=["None", "Edge", "Edge White", "Edge Bold"], s10=("Temporal", ["Off", "On"], "On"),
        k6="Shadow", k1def="Median", s7=True,
        note="# FLATLINE_FINISH (build E) -- stamped sibling of programs/ron/flatline: the finishing\n# features (soft quantization, drop shadow) on the median smoother."),
}


def stamp_vhd(src: str, name: str, cfg: dict) -> str:
    out = src
    for k, v in cfg.items():
        pat = re.compile(r"^(\s*constant\s+%s\s*:\s*\w+\s*:=\s*)[^;]+;" % k, re.M)
        out, n = pat.subn(lambda m: m.group(1) + v + ";", out)
        if n != 1:
            sys.exit(f"{name}: constant {k} matched {n} times")
    if name != "flatline":
        out = out.replace("-- FLATLINE -- image flattening lab",
                          f"-- {name.upper()} -- STAMPED COPY of programs/ron/flatline/flatline.vhd (variants.py);\n-- edit the master, not this file.\n-- FLATLINE -- image flattening lab", 1)
    return out


def labels(lst):
    return "[" + ", ".join('"%s"' % x for x in lst) + "]"


def stamp_toml(src: str, name: str, t: dict) -> str:
    out = re.sub(r"^#.*?\n\n", t["note"] + "\n\n", src, count=1, flags=re.S)
    out = out.replace('program_id = "com.czartroniks.flatline"', f'program_id = "com.czartroniks.{name}"')
    out = out.replace('program_name = "Flatline"', f'program_name = "{t["name"]}"')
    out = re.sub(r'description = ".*?"', 'description = "%s"' % t["desc"], out)
    out = re.sub(r'(name_label = "Smoother"\nvalue_labels = )\[.*?\]\ninitial_value_label = ".*?"',
                 r'\1' + labels(t["k1"]) + '\ninitial_value_label = "%s"' % t["k1def"], out)
    out = re.sub(r'(name_label = "Lines"\nvalue_labels = )\[.*?\]', r'\1' + labels(t["k4"]), out)
    out = out.replace('name_label = "Shadow"\n', 'name_label = "%s"\n' % t["k6"])
    s10n, s10l, s10d = t["s10"]
    out = re.sub(r'(parameter_id = "toggle_switch_10"\nname_label = ).*?\nvalue_labels = \[.*?\]\ninitial_value_label = ".*?"',
                 r'\1"%s"\nvalue_labels = %s\ninitial_value_label = "%s"' % (s10n, labels(s10l), s10d), out)
    if not t["s7"]:
        out = out.replace('name_label = "Quant"\nvalue_labels = ["Hard", "Soft"]\ninitial_value_label = "Hard"', 'name_label = "Quant n/a"\nvalue_labels = ["Hard", "(n/a)"]\ninitial_value_label = "Hard"')
    # presets: K1 zone centres depend on the label count (2 labels: 256 / 768)
    n1 = len(t["k1"])
    out = re.sub(r"rotary_potentiometer_1 = \d+", "rotary_potentiometer_1 = %d" % int((n1 - 0.5) * 1024 / n1), out)
    # K4 presets (4 zones: centres 128/384/640/896): the XDoG/Canny preset falls back to Edge
    out = re.sub(r"rotary_potentiometer_4 = 307", "rotary_potentiometer_4 = 384", out)
    out = re.sub(r"rotary_potentiometer_4 = 102", "rotary_potentiometer_4 = 128", out)
    if t["k4"][3] in ("XDoG", "Canny"):
        out = re.sub(r"rotary_potentiometer_4 = 716", "rotary_potentiometer_4 = 896", out)
    else:
        out = re.sub(r"rotary_potentiometer_4 = 716", "rotary_potentiometer_4 = 384", out)
    return out


def main():
    vhd = MASTER_VHD.read_text()
    toml = MASTER_TOML.read_text()
    for name, cfg in VARIANTS.items():
        d = HERE.parent / name
        d.mkdir(exist_ok=True)
        (d / f"{name}.vhd").write_text(stamp_vhd(vhd, name, cfg))
        (d / f"{name}.toml").write_text(stamp_toml(toml, name, TOML[name]))
        print(f"stamped {name}")


if __name__ == "__main__":
    main()
