#!/usr/bin/env python3
"""513 工位流程入口：手柄组合键 → 底盘 / 升降 / IK（配置见 workflow_513/config/*.yaml）。"""

from __future__ import annotations

import argparse
import sys
import threading
from pathlib import Path
from typing import Any

_SCRIPT_DIR = Path(__file__).resolve().parent
if str(_SCRIPT_DIR) not in sys.path:
    sys.path.insert(0, str(_SCRIPT_DIR))

try:
    import yaml
except ImportError:
    yaml = None  # type: ignore

from workflow_513.evdev_combo import ComboDetector
from workflow_513.evdev_device import find_joystick


def _default_config_path() -> Path:
    return _SCRIPT_DIR / "workflow_513" / "config" / "default.yaml"


def _infer_zone_group(zone_id: str, zone_cfg: dict[str, Any]) -> str:
    group = str(zone_cfg.get("group", "")).strip().lower()
    if group:
        return group
    zid = zone_id.lower()
    if zid.startswith("pickup"):
        return "pickup"
    if zid.startswith("hang"):
        return "hang"
    return zid


def _select_transition_steps(
    transitions_cfg: dict[str, Any],
    *,
    from_zone_id: str,
    from_group: str,
    to_zone_id: str,
    to_group: str,
) -> tuple[list[dict[str, Any]], bool, str]:
    """
    选择过渡步骤。
    返回: (steps, reverse, name)
    """
    # 新结构：transitions.pickup_to_hang.by_hang_zone.<hang_zone>.steps
    trans_new = transitions_cfg.get("pickup_to_hang") or {}
    enabled_new = bool(trans_new.get("enabled", False))
    reverse_back_new = bool(trans_new.get("reverse_for_hang_to_pickup", True))
    if enabled_new and {from_group, to_group} == {"pickup", "hang"}:
        by_hang_zone = trans_new.get("by_hang_zone") or {}
        hang_zone_id = to_zone_id if to_group == "hang" else from_zone_id
        hang_cfg = by_hang_zone.get(hang_zone_id) or {}
        steps = hang_cfg.get("steps") or []
        if steps:
            reverse = from_group == "hang" and to_group == "pickup" and reverse_back_new
            return steps, reverse, f"{from_zone_id}_to_{to_zone_id}[via:{hang_zone_id}]"

    # 兼容旧结构：transitions.pickup_hang.steps
    trans_old = transitions_cfg.get("pickup_hang") or {}
    enabled_old = bool(trans_old.get("enabled", False))
    if enabled_old and {from_group, to_group} == {"pickup", "hang"}:
        steps_old = trans_old.get("steps") or []
        reverse_back_old = bool(trans_old.get("reverse_for_hang_to_pickup", True))
        reverse_old = from_group == "hang" and to_group == "pickup" and reverse_back_old
        return steps_old, reverse_old, f"{from_group}_to_{to_group}"

    return [], False, ""


def load_config(path: Path) -> dict[str, Any]:
    if yaml is None:
        raise RuntimeError("Install PyYAML: pip install pyyaml")
    text = path.read_text(encoding="utf-8")
    data = yaml.safe_load(text)
    if not isinstance(data, dict):
        raise ValueError(f"Invalid YAML root in {path}")
    return data


def main(argv: list[str] | None = None) -> int:
    argv = argv if argv is not None else sys.argv[1:]
    ap = argparse.ArgumentParser(description="513 workflow: joystick combos → ROS actions")
    ap.add_argument(
        "--config",
        type=Path,
        default=_default_config_path(),
        help="YAML config (base poses, torso heights, IK presets)",
    )
    ap.add_argument("--dry-run", action="store_true", help="Print combos only; do not call ROS")
    args = ap.parse_args(argv)

    cfg = load_config(args.config)
    zones = cfg.get("zones") or {}
    if not zones:
        print("No zones in config", file=sys.stderr)
        return 1

    ros_cfg = cfg.get("ros") or {}
    transitions_cfg = cfg.get("transitions") or {}
    cooldown = float(cfg.get("cooldown_sec", 1.5))
    joy_cfg = cfg.get("joystick") or {}
    tr_override = joy_cfg.get("modifier_tr_codes") or []
    tr_codes = [int(x) for x in tr_override] if tr_override else None

    def log(msg: str) -> None:
        print(msg, flush=True)

    detector = ComboDetector(
        zones,
        modifier_tr_codes=tr_codes,
        cooldown_sec=cooldown,
        logger=log,
    )

    dev = find_joystick()
    log(f"[init] device={dev.path!s} name={dev.name!r}")
    log(f"[init] config={args.config}")
    log("[init] combos: 取料 TR+308边沿 / 挂料 TR+十字键边沿（见 513_work_flow.md）")

    if args.dry_run:

        def loop():
            for ev in dev.read_loop():
                z = detector.feed(ev)
                if z:
                    log(f"[dry-run] would run zone={z!r}")

        try:
            loop()
        except KeyboardInterrupt:
            log("exit")
        return 0

    import rclpy
    from workflow_513.ros_bridge import Workflow513RosBridge

    rclpy.init()
    bridge = Workflow513RosBridge(ros_cfg, logger=log)
    exec_lock = threading.Lock()
    last_zone_id: str | None = None

    def run_zone(zone_id: str) -> None:
        def job():
            nonlocal last_zone_id
            if not exec_lock.acquire(blocking=False):
                log("[warn] sequence already running — ignored")
                return
            try:
                zone_cfg = zones[zone_id]
                seq = zone_cfg.get("sequence")
                if not seq:
                    log(f"[err] zone {zone_id!r} has no sequence")
                    return
                label = zone_cfg.get("label", zone_id)
                target_group = _infer_zone_group(zone_id, zone_cfg)
                sequence_ik_first = False

                # 插入取料<->挂料的过渡步骤（可配置顺序；回程可自动逆序）
                if last_zone_id and last_zone_id in zones:
                    from_cfg = zones[last_zone_id]
                    from_group = _infer_zone_group(last_zone_id, from_cfg)
                    to_group = target_group
                    if from_group == "pickup" and to_group == "hang":
                        # pickup -> hang: 先切 IK(SRV) 再执行动作
                        sequence_ik_first = True
                    if (
                        from_group == "hang"
                        and to_group == "hang"
                        and last_zone_id != zone_id
                    ):
                        log(
                            f"[route] blocked: hang zones are not directly connected "
                            f"({last_zone_id} -> {zone_id}). Go to pickup first."
                        )
                        return
                    if from_group != to_group:
                        steps, reverse, trans_name = _select_transition_steps(
                            transitions_cfg,
                            from_zone_id=last_zone_id,
                            from_group=from_group,
                            to_zone_id=zone_id,
                            to_group=to_group,
                        )
                    else:
                        steps, reverse, trans_name = [], False, ""
                    if steps:
                        log(
                            f"[transition] matched={trans_name}, "
                            f"from={last_zone_id}({from_group}) to={zone_id}({to_group}), "
                            f"steps={len(steps)}, reverse={str(reverse).lower()}"
                        )
                        ok_trans = bridge.run_steps(
                            steps,
                            reverse=reverse,
                            name=trans_name,
                        )
                        if not ok_trans:
                            log("[transition] FAILED, zone sequence skipped")
                            return
                    elif {from_group, to_group} == {"pickup", "hang"}:
                        log(
                            f"[transition] no transition steps for "
                            f"from={last_zone_id}({from_group}) to={zone_id}({to_group}); "
                            "run zone sequence directly"
                        )

                log(f"=== start zone {zone_id} ({label}) ===")
                ok = bridge.run_sequence(seq, ik_first=sequence_ik_first)
                log(f"=== done zone {zone_id} ok={ok} ===")
                if ok:
                    last_zone_id = zone_id
            finally:
                exec_lock.release()

        threading.Thread(target=job, daemon=True).start()

    try:
        import select

        log("[init] ROS bridge ready")
        while rclpy.ok():
            r, _, _ = select.select([dev.fd], [], [], 0.05)
            if r:
                for ev in dev.read():
                    zid = detector.feed(ev)
                    if zid:
                        run_zone(zid)
    except KeyboardInterrupt:
        log("KeyboardInterrupt — shutdown")
    finally:
        bridge.destroy_node()
        rclpy.shutdown()

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
