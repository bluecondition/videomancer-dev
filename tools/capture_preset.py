#!/usr/bin/env python3
# Copyright (C) 2026 LZX Industries LLC
# SPDX-License-Identifier: GPL-3.0-only
"""Capture Videomancer knob state into a program TOML.

Reads the loaded program over the USB CDC shell and writes the current
manual parameter values into that program's source file. Factory presets
store manual values only (modulation time/space/slope/source stay on the
device as user presets).

Shell commands used (responses are ``@program:<json>``; errors are ``!<code>:<text>``):

    program info     -> {"name","id","parameters":[...]}
    program state    -> {"m":[12], "t":[12], "sp":[12], "sl":[12], "sr":[12]}

``m`` is the raw manual value for each hardware slot (0 = rotary 1 ... 11 = fader).
Toggle switches are stored in TOML presets as label indices 0 or 1.

Do not open the port at 1200 baud. That line coding reboots the RP2040 into BOOTSEL.

Usage:
    python3 scripts/capture_preset.py --preset 0
    python3 scripts/capture_preset.py --preset 2 --name "Tight"
    python3 scripts/capture_preset.py --defaults
    python3 scripts/capture_preset.py --preset 0 --toml programs/sabattier/sabattier.toml --dry-run
"""

from __future__ import annotations

import argparse
import json
import os
import re
import sys
import time
from pathlib import Path

try:
    import tomli as tomllib  # Python 3.10 and below
except ImportError:
    import tomllib  # Python 3.11+


# Local copy of videomancer-sdk/scripts/capture_preset.py (LZX, upstream
# 606b668) living at <repo>/tools/ so the SDK stays stock; see the two
# HW-measured fixes in raw_to_toggle / raw_to_label_index.
SDK_ROOT = Path(__file__).resolve().parents[1] / "videomancer-sdk"

# VID/PID from lzx::usb_ids (usb_ids.hpp).
VIDEOMANCER_VID = 0x16D0
VIDEOMANCER_PID = 0x14DB

# USB CDC. 1200 baud is the BOOTSEL reboot trigger — never use it.
SHELL_BAUDRATE = 115200

MAX_PRESETS = 8
PRESET_NAME_MAX_BYTES = 15
NUM_CHANNELS = 12

PARAMETER_IDS = (
    "rotary_potentiometer_1",
    "rotary_potentiometer_2",
    "rotary_potentiometer_3",
    "rotary_potentiometer_4",
    "rotary_potentiometer_5",
    "rotary_potentiometer_6",
    "toggle_switch_7",
    "toggle_switch_8",
    "toggle_switch_9",
    "toggle_switch_10",
    "toggle_switch_11",
    "linear_potentiometer_12",
)

TOGGLE_IDS = frozenset(PARAMETER_IDS[6:11])

ANSI_RE = re.compile(r"\x1b\[[0-9;]*[A-Za-z]")
PROGRAM_ID_RE = re.compile(
    r"""(?m)^program_id\s*=\s*(?:"([^"\n]*)"|'([^'\n]*)')"""
)
PROGRAM_NAME_RE = re.compile(
    r"""(?m)^program_name\s*=\s*(?:"([^"\n]*)"|'([^'\n]*)')"""
)
PRESET_HEADER_RE = re.compile(r"^\[\[preset\]\]\s*$")
PARAMETER_HEADER_RE = re.compile(r"^\[\[parameter\]\]\s*$")
TABLE_RE = re.compile(r"^\[")
INITIAL_VALUE_RE = re.compile(
    r"""^(\s*)initial_value\s*=\s*-?\d+(\s*(?:#.*)?)?\s*$"""
)
INITIAL_LABEL_RE = re.compile(
    r"""^(\s*)initial_value_label\s*=\s*"(?:[^"\\]|\\.)*"(.*)$"""
)
SKIP_DIRS = {
    ".git",
    "build",
    "out",
    "releases",
    "third_party",
    ".work",
    "node_modules",
    "__pycache__",
}


class CaptureError(Exception):
    """User-facing failure. The message is the whole report."""


class ShellError(CaptureError):
    def __init__(self, code: str, message: str) -> None:
        super().__init__(f"device error {code}: {message}")
        self.code = code
        self.message = message


def strip_ansi(text: str) -> str:
    return ANSI_RE.sub("", text)


def parse_shell_line(line: str) -> tuple[str, str, str] | None:
    """Return ('ok', command, payload) or ('err', code, message)."""
    line = strip_ansi(line).strip()
    if line.startswith("@"):
        command, sep, payload = line[1:].partition(":")
        if not sep:
            return None
        return ("ok", command, payload)
    if line.startswith("!"):
        code, sep, message = line[1:].partition(":")
        if not sep:
            return None
        return ("err", code, message)
    return None


def transact(port, command: str, timeout: float = 2.0) -> str:
    """Send one shell line and return the ``@program`` JSON payload."""
    port.reset_input_buffer()
    port.write((command + "\n").encode("ascii"))
    port.flush()

    deadline = time.monotonic() + timeout
    pending = ""
    while time.monotonic() < deadline:
        chunk = port.read(4096)
        if chunk:
            pending += chunk.decode("utf-8", errors="replace")
            while "\n" in pending:
                line, pending = pending.split("\n", 1)
                parsed = parse_shell_line(line)
                if parsed is None:
                    continue
                kind, key, payload = parsed
                if kind == "err":
                    raise ShellError(key, payload)
                if kind == "ok" and key == "program":
                    return payload
        else:
            time.sleep(0.02)
    raise CaptureError(
        f"timed out waiting for a reply to {command!r}. "
        "Is the Videomancer shell on this port?"
    )


def decode_program_info(payload: str) -> tuple[str, str]:
    try:
        data = json.loads(payload)
    except json.JSONDecodeError as exc:
        raise CaptureError(f"program info was not JSON: {payload!r}") from exc
    program_id = str(data.get("id") or "")
    program_name = str(data.get("name") or "")
    if not program_id and not program_name:
        raise CaptureError(f"program info has no id or name: {payload!r}")
    return program_id, program_name


def decode_manual_values(payload: str) -> list[int]:
    try:
        data = json.loads(payload)
    except json.JSONDecodeError as exc:
        raise CaptureError(f"program state was not JSON: {payload!r}") from exc
    manual = data.get("m")
    if not isinstance(manual, list) or len(manual) != NUM_CHANNELS:
        raise CaptureError(
            f"program state manual array must have {NUM_CHANNELS} values, got {manual!r}"
        )
    values: list[int] = []
    for item in manual:
        if isinstance(item, bool) or not isinstance(item, int):
            raise CaptureError(f"manual value {item!r} is not an integer")
        if not 0 <= item <= 1023:
            raise CaptureError(f"manual value {item} is outside 0..1023")
        values.append(item)
    return values


def toml_basic_string(value: str) -> str:
    escaped = value.replace("\\", "\\\\").replace('"', '\\"')
    return f'"{escaped}"'


def validate_preset_name(name: str) -> str:
    if not name or name.strip() == "":
        raise CaptureError("preset name is empty")
    if "\n" in name or "\r" in name:
        raise CaptureError("preset name cannot contain a newline")
    if len(name.encode("utf-8")) > PRESET_NAME_MAX_BYTES:
        raise CaptureError(
            f"preset name {name!r} exceeds {PRESET_NAME_MAX_BYTES} bytes"
        )
    return name


def slot_for(parameter_id: str) -> int | None:
    try:
        return PARAMETER_IDS.index(parameter_id)
    except ValueError:
        return None


def raw_to_toggle(raw: int) -> int:
    # The firmware reports toggle switches as 0/1 in "m", not 0/1023
    # (HW-measured 2026-10-02, edgelab): a >= 512 test read every ON
    # switch as OFF.  Any nonzero value is ON.
    return 1 if raw >= 1 else 0


def raw_to_label_index(raw: int, label_count: int) -> int:
    if label_count <= 1:
        return 0
    if label_count == 2:
        return raw_to_toggle(raw)
    # Labelled defaults load as the raw label INDEX (0..N-1), so a value
    # below the label count is the index itself; otherwise use the even
    # 1024/N zones the programs decode (not round-to-nearest-label).
    if raw < label_count:
        return raw
    return max(0, min(label_count - 1, raw * label_count // 1024))


def load_config(text: str) -> dict:
    try:
        return tomllib.loads(text)
    except tomllib.TOMLDecodeError as exc:
        raise CaptureError(f"TOML parse error: {exc}") from exc


def defined_parameters(config: dict) -> list[dict]:
    parameters = []
    seen: set[str] = set()
    for param in config.get("parameter", []):
        parameter_id = param.get("parameter_id", "none")
        if not isinstance(parameter_id, str) or slot_for(parameter_id) is None:
            continue
        if parameter_id in seen:
            raise CaptureError(f"duplicate parameter_id {parameter_id}")
        seen.add(parameter_id)
        parameters.append(param)
    if not parameters:
        raise CaptureError("TOML has no hardware parameters")
    return parameters


def preset_assignments(parameters: list[dict], manual: list[int]) -> dict[str, int]:
    """Map defined parameter ids to TOML preset values (toggles as 0/1)."""
    values: dict[str, int] = {}
    for param in parameters:
        parameter_id = param["parameter_id"]
        raw = manual[slot_for(parameter_id)]
        if parameter_id in TOGGLE_IDS:
            values[parameter_id] = raw_to_toggle(raw)
        else:
            values[parameter_id] = raw
    return values


def render_preset(name: str, parameters: list[dict], values: dict[str, int]) -> str:
    lines = ["[[preset]]", f"name = {toml_basic_string(name)}"]
    for param in parameters:
        parameter_id = param["parameter_id"]
        lines.append(f"{parameter_id} = {values[parameter_id]}")
    return "\n".join(lines)


def newline_of(text: str) -> str:
    return "\r\n" if "\r\n" in text else "\n"


def table_spans(lines: list[str], header_re: re.Pattern[str]) -> list[tuple[int, int]]:
    """Return [start, end) line spans for tables whose header matches."""
    headers = [i for i, line in enumerate(lines) if header_re.match(line)]
    spans = []
    for start in headers:
        end = len(lines)
        for j in range(start + 1, len(lines)):
            if TABLE_RE.match(lines[j]):
                end = j
                break
        spans.append((start, end))
    return spans


def join_lines(lines: list[str], newline: str, trailing: bool) -> str:
    body = newline.join(lines)
    if trailing:
        body += newline
    return body


def replace_span(text: str, start: int, end: int, block: str) -> str:
    newline = newline_of(text)
    lines = text.splitlines()
    trailing = text.endswith("\n")
    new_lines = block.splitlines()
    if end < len(lines) and (not new_lines or new_lines[-1] != ""):
        new_lines.append("")
    out = lines[:start] + new_lines + lines[end:]
    return join_lines(out, newline, trailing or True)


def append_preset_block(text: str, block: str) -> str:
    newline = newline_of(text)
    if text and not text.endswith(("\n", "\r\n")):
        text += newline
    if text and not text.endswith(newline + newline):
        text += newline
    text += block.replace("\n", newline)
    if not text.endswith(newline):
        text += newline
    return text


def rewrite_preset(
    text: str,
    index: int,
    manual: list[int],
    name: str | None,
) -> str:
    if not 0 <= index < MAX_PRESETS:
        raise CaptureError(f"preset index must be 0..{MAX_PRESETS - 1}")
    config = load_config(text)
    parameters = defined_parameters(config)
    presets = list(config.get("preset", []))
    if index > len(presets):
        raise CaptureError(
            f"preset {index} does not exist ({len(presets)} in file). "
            f"Use {len(presets)} to append the next preset."
        )
    values = preset_assignments(parameters, manual)
    if index == len(presets):
        preset_name = validate_preset_name(name or "Captured")
        return append_preset_block(text, render_preset(preset_name, parameters, values))

    preset_name = validate_preset_name(name if name is not None else str(presets[index].get("name", "")))
    lines = text.splitlines()
    spans = table_spans(lines, PRESET_HEADER_RE)
    if index >= len(spans):
        raise CaptureError(
            f"parsed {len(presets)} presets but found {len(spans)} [[preset]] blocks"
        )
    start, end = spans[index]
    return replace_span(text, start, end, render_preset(preset_name, parameters, values))


def _replace_or_insert(lines: list[str], start: int, end: int, pattern: re.Pattern[str], body: str) -> None:
    for i in range(start, end):
        match = pattern.match(lines[i])
        if match:
            suffix = match.group(2) or ""
            lines[i] = f"{match.group(1)}{body}{suffix}"
            return
    indent = ""
    for i in range(start, end):
        if lines[i].strip().startswith("parameter_id"):
            indent = re.match(r"\s*", lines[i]).group(0)
            break
    insert_at = end
    while insert_at > start + 1 and lines[insert_at - 1].strip() == "":
        insert_at -= 1
    lines.insert(insert_at, f"{indent}{body}")


def rewrite_defaults(text: str, manual: list[int]) -> str:
    config = load_config(text)
    parameters = defined_parameters(config)
    newline = newline_of(text)
    lines = text.splitlines()
    spans = table_spans(lines, PARAMETER_HEADER_RE)
    if len(spans) < len(parameters):
        raise CaptureError(
            f"parsed {len(parameters)} parameters but found {len(spans)} [[parameter]] blocks"
        )

    # Walk bottom-up so an inserted line does not shift later spans.
    defined_spans = []
    span_i = 0
    for param in config.get("parameter", []):
        parameter_id = param.get("parameter_id", "none")
        if slot_for(parameter_id if isinstance(parameter_id, str) else "") is None:
            span_i += 1
            continue
        if span_i >= len(spans):
            raise CaptureError("parameter block walk ran past the TOML tables")
        defined_spans.append((spans[span_i], param))
        span_i += 1

    for (start, end), param in reversed(defined_spans):
        parameter_id = param["parameter_id"]
        raw = manual[slot_for(parameter_id)]
        labels = param.get("value_labels") or []
        if labels:
            label_index = raw_to_label_index(raw, len(labels))
            label = str(labels[label_index])
            _replace_or_insert(
                lines,
                start,
                end,
                INITIAL_LABEL_RE,
                f"initial_value_label = {toml_basic_string(label)}",
            )
        else:
            _replace_or_insert(
                lines,
                start,
                end,
                INITIAL_VALUE_RE,
                f"initial_value = {raw}",
            )
        # Inserts shift subsequent indices; spans above this block are untouched
        # because we walk from the bottom.

    return join_lines(lines, newline, text.endswith("\n") or True)


def iter_toml_files(root: Path):
    if not root.is_dir():
        return
    for dirpath, dirnames, filenames in os.walk(root, followlinks=False):
        dirnames[:] = [
            name for name in dirnames if name not in SKIP_DIRS and not name.startswith(".")
        ]
        for filename in filenames:
            if not filename.endswith(".toml"):
                continue
            if filename.startswith(".") or ".lzx-status" in filename:
                continue
            yield Path(dirpath) / filename


def default_search_roots(sdk_root: Path = SDK_ROOT) -> list[Path]:
    roots: list[Path] = []
    programs = sdk_root / "programs"
    if programs.is_dir():
        roots.append(programs)
    parent = sdk_root.parent
    try:
        children = list(parent.iterdir())
    except OSError:
        children = []
    for child in children:
        name = child.name
        if name.startswith("videomancer_") and name.endswith("_vmprog"):
            try:
                if child.is_dir():
                    roots.append(child)
            except OSError:
                continue
    return roots


def read_toml(path: Path) -> str | None:
    try:
        return path.read_text(encoding="utf-8")
    except OSError:
        return None


def quoted_field(match: re.Match[str] | None) -> str:
    if match is None:
        return ""
    return match.group(1) if match.group(1) is not None else match.group(2)


def find_program_sources(
    program_id: str,
    program_name: str,
    roots: list[Path],
) -> list[Path]:
    matches: list[Path] = []
    name_matches: list[Path] = []
    for root in roots:
        for path in iter_toml_files(root):
            text = read_toml(path)
            if text is None:
                continue
            found_id = quoted_field(PROGRAM_ID_RE.search(text))
            found_name = quoted_field(PROGRAM_NAME_RE.search(text))
            if program_id and found_id == program_id:
                matches.append(path)
            elif not program_id and program_name and found_name == program_name:
                name_matches.append(path)
    return matches or name_matches


def inherited_preset_value(param: dict) -> int:
    """Value a missing preset key inherits, in TOML preset units."""
    parameter_id = param["parameter_id"]
    labels = list(param.get("value_labels") or [])
    initial = param.get("initial_value_label")
    if parameter_id in TOGGLE_IDS:
        if labels and initial in labels and labels.index(initial) >= 1:
            return 1
        return 0
    if labels:
        index = labels.index(initial) if initial in labels else 0
        if len(labels) == 2:
            return 1023 if index >= 1 else 0
        return round(index * 1023 / max(len(labels) - 1, 1))
    return int(param.get("initial_value", 512))


def effective_preset_value(param: dict, preset: dict) -> int:
    parameter_id = param["parameter_id"]
    if parameter_id in preset:
        return int(preset[parameter_id])
    return inherited_preset_value(param)


def change_lines(
    parameters: list[dict],
    before: dict[str, int],
    after: dict[str, int],
) -> list[str]:
    lines = []
    for param in parameters:
        parameter_id = param["parameter_id"]
        old = before.get(parameter_id)
        new = after[parameter_id]
        if old != new:
            lines.append(f"  {parameter_id}: {old} -> {new}")
    return lines


def capture_into_source(
    text: str,
    manual: list[int],
    *,
    preset_index: int | None,
    defaults: bool,
    name: str | None,
) -> tuple[str, list[str]]:
    config = load_config(text)
    parameters = defined_parameters(config)
    if defaults:
        updated = rewrite_defaults(text, manual)
        before = {}
        for param in parameters:
            parameter_id = param["parameter_id"]
            if param.get("value_labels"):
                labels = list(param["value_labels"])
                initial = param.get("initial_value_label")
                before[parameter_id] = labels.index(initial) if initial in labels else None
            else:
                before[parameter_id] = int(param.get("initial_value", 512))
        after_config = load_config(updated)
        after = {}
        for param in defined_parameters(after_config):
            parameter_id = param["parameter_id"]
            if param.get("value_labels"):
                labels = list(param["value_labels"])
                initial = param.get("initial_value_label")
                after[parameter_id] = labels.index(initial) if initial in labels else None
            else:
                after[parameter_id] = int(param.get("initial_value", 512))
        return updated, change_lines(parameters, before, after)

    if preset_index is None:
        raise CaptureError("pass --preset N or --defaults")
    presets = list(config.get("preset", []))
    before: dict[str, int] = {}
    if preset_index < len(presets):
        for param in parameters:
            before[param["parameter_id"]] = effective_preset_value(param, presets[preset_index])
    updated = rewrite_preset(text, preset_index, manual, name)
    after = preset_assignments(parameters, manual)
    # Compare toggle/raw preset numbers, not label indexes.
    if preset_index < len(presets):
        notes = change_lines(parameters, before, after)
    else:
        notes = [f"  {pid} = {after[pid]}" for pid in after]
    return updated, notes


def find_videomancer_port() -> str:
    try:
        from serial.tools import list_ports
    except ImportError as exc:
        raise CaptureError(
            "pyserial is required (pip install pyserial)"
        ) from exc
    matches = [
        port for port in list_ports.comports()
        if port.vid == VIDEOMANCER_VID and port.pid == VIDEOMANCER_PID
    ]
    if len(matches) == 1:
        return matches[0].device
    if len(matches) > 1:
        names = ", ".join(port.device for port in matches)
        raise CaptureError(f"multiple Videomancer ports: {names}. Pass --port.")
    raise CaptureError(
        "no Videomancer found (VID 16D0 PID 14DB). Pass --port COMx or /dev/ttyACM0."
    )


def open_port(port_name: str):
    try:
        import serial
    except ImportError as exc:
        raise CaptureError(
            "pyserial is required (pip install pyserial)"
        ) from exc
    port = serial.Serial()
    port.port = port_name
    port.baudrate = SHELL_BAUDRATE
    port.timeout = 0.05
    port.write_timeout = 1.0
    # Avoid a baud-rate touch. 1200 baud reboots into BOOTSEL.
    # TEMPORARY (2026-10-15, Ron approved): DTR asserted. Original was
    # `port.dtr = False`, which worked for months; on 2026-10-15 the shell
    # stopped replying with DTR low (after an interrupted capture, survived a
    # WSL reboot; Videomancer not yet power-cycled). Revert to False once a
    # power-cycled device answers with DTR low again. See tools/LOCAL_CHANGES.md.
    port.dtr = True
    port.rts = False
    try:
        port.open()
    except serial.SerialException as exc:
        raise CaptureError(f"could not open {port_name}: {exc}") from exc
    return port


def read_device(port, timeout: float) -> tuple[str, str, list[int]]:
    info_payload = transact(port, "program info", timeout)
    program_id, program_name = decode_program_info(info_payload)
    state_payload = transact(port, "program state", timeout)
    manual = decode_manual_values(state_payload)
    return program_id, program_name, manual


def narrow_by_cwd(matches: list[Path], cwd: Path | None = None) -> list[Path]:
    """Keep sources under the working directory when that selects one tree."""
    cwd = (cwd or Path.cwd()).resolve()
    narrowed = []
    for path in matches:
        resolved = path.resolve()
        if cwd == resolved.parent or cwd in resolved.parents:
            narrowed.append(path)
    return narrowed or matches


def resolve_source(
    program_id: str,
    program_name: str,
    toml_path: Path | None,
    programs_dirs: list[Path],
    cwd: Path | None = None,
) -> tuple[Path, str]:
    if toml_path is not None:
        if not toml_path.is_file():
            raise CaptureError(f"TOML not found: {toml_path}")
        text = read_toml(toml_path)
        if text is None:
            raise CaptureError(f"could not read {toml_path}")
        found_id = quoted_field(PROGRAM_ID_RE.search(text))
        if program_id and found_id and found_id != program_id:
            raise CaptureError(
                f"{toml_path} is {found_id}, but the device is running {program_id}"
            )
        return toml_path, text

    roots = list(programs_dirs) if programs_dirs else default_search_roots()
    matches = narrow_by_cwd(find_program_sources(program_id, program_name, roots), cwd)
    if not matches:
        ident = program_id or program_name
        raise CaptureError(
            f"no TOML with program identity {ident!r}. "
            "Pass --toml or --programs-dir."
        )
    if len(matches) > 1:
        listed = "\n".join(f"  {path}" for path in matches)
        raise CaptureError(
            "several TOML files match the loaded program:\n"
            f"{listed}\n"
            "Pass --toml, or run from the program directory."
        )
    text = read_toml(matches[0])
    if text is None:
        raise CaptureError(f"could not read {matches[0]}")
    return matches[0], text


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description=(
            "Read the connected Videomancer's current parameter values and "
            "write them into the loaded program's TOML as factory preset N "
            "or as parameter defaults."
        )
    )
    target = parser.add_mutually_exclusive_group(required=True)
    target.add_argument(
        "--preset",
        type=int,
        metavar="N",
        help=f"factory preset index to replace, or the next free index to append (0..{MAX_PRESETS - 1})",
    )
    target.add_argument(
        "--defaults",
        action="store_true",
        help="write the live values into each parameter's initial_value / initial_value_label",
    )
    parser.add_argument("--name", help="preset name (max 15 bytes). Kept when replacing if omitted.")
    parser.add_argument("--port", help="serial port. Default: the Videomancer USB CDC port.")
    parser.add_argument(
        "--toml",
        type=Path,
        help="program TOML to update. Default: search by the device program_id.",
    )
    parser.add_argument(
        "--programs-dir",
        type=Path,
        action="append",
        default=[],
        help="extra directory to search for program TOML files (repeatable)",
    )
    parser.add_argument(
        "--timeout",
        type=float,
        default=2.0,
        help="seconds to wait for each shell reply (default 2)",
    )
    parser.add_argument(
        "--dry-run",
        action="store_true",
        help="print the update and do not write the file",
    )
    return parser


def main(argv: list[str] | None = None) -> int:
    args = build_parser().parse_args(argv)
    if args.defaults and args.name:
        raise CaptureError("--name applies to --preset, not --defaults")
    port_name = args.port or find_videomancer_port()
    port = open_port(port_name)
    try:
        program_id, program_name, manual = read_device(port, args.timeout)
    finally:
        port.close()

    path, text = resolve_source(
        program_id,
        program_name,
        args.toml,
        args.programs_dir,
    )
    updated, notes = capture_into_source(
        text,
        manual,
        preset_index=args.preset,
        defaults=bool(args.defaults),
        name=args.name,
    )
    ident = program_id or program_name
    target = "defaults" if args.defaults else f"preset {args.preset}"
    print(f"{ident}  {program_name}")
    print(f"{path}")
    print(f"{target}")
    if updated == text:
        print("already matches")
        return 0
    if notes:
        print("\n".join(notes))
    else:
        print("values already match; source text refreshed")
    if args.dry_run:
        print("dry-run, not written")
        return 0
    path.write_text(updated, encoding="utf-8", newline="\n")
    print("wrote", path)
    return 0


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except CaptureError as exc:
        print(f"error: {exc}", file=sys.stderr)
        raise SystemExit(1)
