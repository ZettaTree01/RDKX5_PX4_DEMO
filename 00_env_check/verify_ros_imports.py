#!/usr/bin/env python3
"""导入校验 ROS 例程模块（不 spin）。需已 source TogetheROS。"""
import importlib.util
import sys

MODS = [
    "/app/zettatree_demo/_common/indoor.py",
    "/app/zettatree_demo/_common/offboard_manager.py",
    "/app/zettatree_demo/_common/emergency_disarm.py",
    "/app/zettatree_demo/_common/camera_node.py",
    "/app/zettatree_demo/_common/frame_output.py",
    "/app/zettatree_demo/_common/yolo_detector.py",
    "/app/zettatree_demo/_common/helipad_h.py",
    "/app/zettatree_demo/04_object_detection/object_detection.py",
    "/app/zettatree_demo/05_obstacle_avoidance/obstacle_avoidance.py",
    "/app/zettatree_demo/06_autonomous_cruise/autonomous_cruise.py",
    "/app/zettatree_demo/07_target_tracking/target_tracking.py",
    "/app/zettatree_demo/08_formation_flight/formation_flight.py",
]


def load(path):
    name = path.rsplit("/", 1)[-1][:-3]
    folder = path.rsplit("/", 1)[0]
    if folder not in sys.path:
        sys.path.insert(0, folder)
    spec = importlib.util.spec_from_file_location(name, path)
    mod = importlib.util.module_from_spec(spec)
    sys.modules[name] = mod
    spec.loader.exec_module(mod)
    return name


def main():
    failed = 0
    for path in MODS:
        try:
            name = load(path)
            print("IMPORT OK", name)
        except Exception as e:
            failed += 1
            print("IMPORT FAIL", path, e)
    return 1 if failed else 0


if __name__ == "__main__":
    sys.exit(main())
