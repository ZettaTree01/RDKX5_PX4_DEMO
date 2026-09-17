#!/usr/bin/env python3
"""导入校验 ROS 例程模块（不 spin）。

在已 source TogetheROS/ROS2 的环境下，按路径逐个 load 共享组件与例程脚本，
确认语法与依赖可导入。失败会打印异常并累计计数；全部成功则退出码 0。
"""
import importlib.util
import sys

# 待校验的模块绝对路径（板端容器内布局）
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
    "/app/zettatree_demo/10_target_follow/target_follow.py",
    "/app/zettatree_demo/11_formation_flight/formation_flight.py",
]


def load(path):
    """按文件路径动态导入模块；目录加入 sys.path 以解析相对依赖。

    返回模块名（文件名去掉 .py）。导入失败则向上抛出异常。
    """
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
    """依次导入 MODS；有失败则返回 1，否则 0。"""
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
