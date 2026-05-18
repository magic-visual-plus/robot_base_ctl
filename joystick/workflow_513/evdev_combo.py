"""evdev 组合键解析（513 工位）：与具体 ROS 逻辑解耦。"""

from __future__ import annotations

import copy
import time
from dataclasses import dataclass
from typing import Any, Callable, Optional

from evdev import ecodes


def ev_abs_label(by_type_dict: dict, code: int, fallback_prefix: str) -> str:
    raw = by_type_dict.get(code, f"{fallback_prefix}_{code}")
    if isinstance(raw, (list, tuple)):
        return "|".join(raw)
    return str(raw)


def resolve_hat_axis_code(name: str) -> int:
    """name e.g. HAT0X -> ABS_HAT0X."""
    key = name.strip().upper()
    if key.startswith("ABS_"):
        attr = key
    else:
        attr = f"ABS_{key}"
    if hasattr(ecodes, attr):
        return int(getattr(ecodes, attr))
    raise ValueError(f"Unknown hat axis name {name!r} (tried ecodes.{attr})")


@dataclass
class ComboRule:
    zone_id: str
    type: str
    spec: dict[str, Any]


class ComboDetector:
    """
    输入 evdev Event，在满足组合条件边沿时回调 zone_id。
    - pickup: TR 按住 + 指定按键首次按下
    - hang: TR 按住 + 十字键某一轴向取值切入目标值（边沿）
    """

    def __init__(
        self,
        zones_cfg: dict[str, Any],
        *,
        modifier_tr_codes: Optional[list[int]] = None,
        cooldown_sec: float = 1.5,
        logger: Optional[Callable[[str], None]] = None,
    ):
        self._log = logger or (lambda _s: None)
        self._cooldown_sec = float(cooldown_sec)
        self._last_fire_mono = 0.0

        self._tr_codes = modifier_tr_codes or [int(ecodes.BTN_TR)]
        self._rules = self._build_rules(zones_cfg)

        self._keys_down: set[int] = set()
        self._keys_prev: set[int] = set()
        self._hat_x = 0
        self._hat_y = 0
        self._hat_prev_x = 0
        self._hat_prev_y = 0

    def _build_rules(self, zones_cfg: dict[str, Any]) -> list[ComboRule]:
        rules: list[ComboRule] = []
        for zone_id, zcfg in zones_cfg.items():
            combo = zcfg.get("combo") or {}
            rules.append(ComboRule(zone_id, combo.get("type", ""), combo))
        return rules

    def _tr_held(self) -> bool:
        return bool(self._keys_down.intersection(self._tr_codes))

    def feed(self, event) -> Optional[str]:
        """返回本次满足的 zone_id，否则 None（冷却期内丢弃触发，但仍更新内部按键/帽子状态）。"""
        self._keys_prev = copy.copy(self._keys_down)

        triggered: Optional[str] = None
        if event.type == ecodes.EV_KEY:
            code = int(event.code)
            val = int(event.value)
            if val == 1:
                self._keys_down.add(code)
            elif val == 0:
                self._keys_down.discard(code)

            triggered = self._eval_after_key(code) or self._eval_modifier_edge_pickup(code)

        elif event.type == ecodes.EV_ABS:
            code = int(event.code)
            val = int(event.value)
            label = ev_abs_label(ecodes.bytype[ecodes.EV_ABS], code, "UNKNOWN_ABS")
            if label.startswith("ABS_HAT0X") or code == ecodes.ABS_HAT0X:
                self._hat_prev_x = self._hat_x
                self._hat_x = val
            elif label.startswith("ABS_HAT0Y") or code == ecodes.ABS_HAT0Y:
                self._hat_prev_y = self._hat_y
                self._hat_y = val

            triggered = self._eval_after_hat(code)

        if triggered is None:
            return None

        now = time.monotonic()
        if now - self._last_fire_mono < self._cooldown_sec:
            self._log("[combo] cooldown — ignored duplicate trigger")
            return None
        self._last_fire_mono = now
        return triggered

    def _eval_after_key(self, code: int) -> Optional[str]:
        if not self._tr_held():
            return None
        for rule in self._rules:
            if rule.type != "tr_and_key_edge":
                continue
            key_codes = [int(x) for x in rule.spec.get("key_codes") or []]
            if not key_codes:
                continue
            if code not in key_codes:
                continue
            if code in self._keys_down and code not in self._keys_prev:
                self._log(f"[combo] zone={rule.zone_id} (TR + key {code} edge)")
                return rule.zone_id
        return None

    def _eval_modifier_edge_pickup(self, code: int) -> Optional[str]:
        """TR 刚按下且副键已按住（先按 Y 再按 TR 也能触发取料）。"""
        if code not in self._tr_codes:
            return None
        if code not in self._keys_down or code in self._keys_prev:
            return None
        for rule in self._rules:
            if rule.type != "tr_and_key_edge":
                continue
            key_codes = {int(x) for x in rule.spec.get("key_codes") or []}
            if key_codes and key_codes.issubset(self._keys_down):
                self._log(f"[combo] zone={rule.zone_id} (key held + TR edge)")
                return rule.zone_id
        return None

    def _eval_after_hat(self, axis_code: int) -> Optional[str]:
        if not self._tr_held():
            return None
        for rule in self._rules:
            if rule.type != "tr_and_hat_edge":
                continue
            try:
                want_axis = resolve_hat_axis_code(rule.spec["axis"])
            except (KeyError, ValueError):
                continue
            if want_axis != axis_code:
                continue
            target = int(rule.spec["value"])
            if want_axis == ecodes.ABS_HAT0X:
                prev, cur = self._hat_prev_x, self._hat_x
            elif want_axis == ecodes.ABS_HAT0Y:
                prev, cur = self._hat_prev_y, self._hat_y
            else:
                continue
            if cur == target and prev != target:
                self._log(f"[combo] zone={rule.zone_id} (TR + hat edge -> {target})")
                return rule.zone_id
        return None
