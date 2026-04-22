"""
电机驱动「状态异常时 SDO 诊断」条目：由 JSON 配置，便于同川 / 其他厂商切换索引与显示格式。

配置文件目录：joystick/config/motor_diagnostics/*.json
格式说明见同目录下 tongchuan_mdx.json。
"""
from __future__ import annotations

import json
import os
from dataclasses import dataclass
from typing import Any, Optional

from loguru import logger

from Ds402_ctl import safe_sdo_read

KNOWN_FORMATS = frozenset({"u16_hex", "u32_hex", "i16_0p1pct", "i16_dec", "i8_dec"})

_JOYSTICK_DIR = os.path.dirname(os.path.abspath(__file__))
_DEFAULT_DIAG_DIR = os.path.join(_JOYSTICK_DIR, "config", "motor_diagnostics")


@dataclass(frozen=True)
class MotorDiagnosticSdoEntry:
    index: int
    sub: int
    format: str
    unit_note: str = ""


@dataclass(frozen=True)
class MotorDiagnosticProfile:
    id: str
    entries: tuple[MotorDiagnosticSdoEntry, ...]


def resolve_motor_diagnostics_json_path(
    *,
    explicit_path: Optional[str],
    profile_id: str,
) -> str:
    if explicit_path:
        return os.path.normpath(explicit_path)
    name = profile_id if profile_id.lower().endswith(".json") else f"{profile_id}.json"
    return os.path.join(_DEFAULT_DIAG_DIR, name)


def _parse_obj_index(obj: Any) -> int:
    if isinstance(obj, int):
        return int(obj)
    s = str(obj).strip()
    if s.lower().startswith("0x"):
        return int(s, 16)
    return int(s)


def _sdo_i16_value(v: int) -> int:
    x = int(v) & 0xFFFF
    return x - 0x10000 if x >= 0x8000 else x


def _sdo_i8_value(v: int) -> int:
    x = int(v) & 0xFF
    return x - 0x100 if x >= 0x80 else x


def _format_entry_value(v: Optional[int], idx: int, fmt: str, unit_note: str) -> str:
    label = f"0x{idx:04X}"
    if v is None:
        return f"{label}=?"
    if fmt == "u16_hex":
        return f"{label}=0x{int(v) & 0xFFFF:04X}"
    if fmt == "u32_hex":
        return f"{label}=0x{int(v) & 0xFFFFFFFF:08X}"
    if fmt == "i16_0p1pct":
        n = _sdo_i16_value(int(v))
        suffix = unit_note.strip() if unit_note.strip() else "rated"
        return f"{label}={n} ({n * 0.1:g}% {suffix})"
    if fmt == "i16_dec":
        return f"{label}={_sdo_i16_value(int(v))}"
    if fmt == "i8_dec":
        return f"{label}={_sdo_i8_value(int(v))}"
    return f"{label}=?"


def load_motor_diagnostic_profile(path: str) -> MotorDiagnosticProfile:
    with open(path, "r", encoding="utf-8") as f:
        data = json.load(f)
    pid = str(data.get("id", os.path.splitext(os.path.basename(path))[0]))
    entries: list[MotorDiagnosticSdoEntry] = []
    rows = data.get("sdo_reads")
    if not isinstance(rows, list):
        raise ValueError(f"{path}: missing or invalid 'sdo_reads' array")
    for i, row in enumerate(rows):
        if not isinstance(row, dict):
            raise ValueError(f"{path}: sdo_reads[{i}] must be an object")
        if "index" not in row or "format" not in row:
            raise ValueError(f"{path}: sdo_reads[{i}] needs 'index' and 'format'")
        idx = _parse_obj_index(row["index"])
        sub = int(row.get("sub", 0))
        fmt = str(row["format"])
        if fmt not in KNOWN_FORMATS:
            raise ValueError(
                f"{path}: sdo_reads[{i}] unknown format {fmt!r} "
                f"(known: {sorted(KNOWN_FORMATS)})"
            )
        note = str(row.get("unit_note", ""))
        entries.append(MotorDiagnosticSdoEntry(index=idx, sub=sub, format=fmt, unit_note=note))
    return MotorDiagnosticProfile(id=pid, entries=tuple(entries))


def load_motor_diagnostic_profile_safe(path: str) -> MotorDiagnosticProfile:
    try:
        return load_motor_diagnostic_profile(path)
    except FileNotFoundError:
        logger.error("Motor diagnostics config not found: {}", path)
    except Exception as e:
        logger.error("Motor diagnostics config invalid ({}): {}", path, e)
    return MotorDiagnosticProfile(id="empty", entries=())


def format_motor_diagnostic_sdo_line(node, profile: MotorDiagnosticProfile) -> str:
    if not profile.entries:
        return ""
    parts: list[str] = []
    for e in profile.entries:
        sub = None if e.sub == 0 else e.sub
        v = safe_sdo_read(node, e.index, sub, default=None)
        parts.append(_format_entry_value(v, e.index, e.format, e.unit_note))
    return "  ".join(parts)
