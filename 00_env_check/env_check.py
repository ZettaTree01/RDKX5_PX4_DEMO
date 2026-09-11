#!/usr/bin/env python3
"""
检查 02 例程包运行所需的板端环境。文档例程前置检查。

不接飞控、不接线也能跑完。失败项表示对应例程会缺库或缺设备，
不代表整板损坏。ROS 相关检查会 source /opt/tros/humble/setup.bash。
"""
import importlib
import glob
import os
import subprocess
import sys

CHECKS = []


def ok(name, detail=""):
    CHECKS.append((True, name, detail))
    print(f"[OK]   {name}" + (f"  ({detail})" if detail else ""))


def fail(name, detail=""):
    CHECKS.append((False, name, detail))
    print(f"[FAIL] {name}" + (f"  ({detail})" if detail else ""))


def try_import(mod, label=None):
    label = label or mod
    try:
        m = importlib.import_module(mod)
        ver = getattr(m, "__version__", "")
        ok(f"python:{label}", ver)
        return True
    except Exception as e:
        fail(f"python:{label}", str(e))
        return False


def which_file(path, label=None):
    label = label or path
    if os.path.exists(path):
        ok(label, path)
        return True
    fail(label, f"不存在: {path}")
    return False


def main():
    print("RDK X5 / 02 例程包环境自检")
    print("=" * 50)
    print("Python", sys.version.replace("\n", " "))

    try_import("Hobot.GPIO", "Hobot.GPIO")
    try_import("serial", "pyserial")
    smbus_ok = try_import("smbus") or try_import("smbus2")
    if not smbus_ok:
        fail("python:smbus/smbus2", "I2C 例程需要 python3-smbus 或 python3-smbus2")
    try_import("i2cdev")
    try_import("numpy")
    try_import("cv2")
    try_import("hbm_runtime")

    which_file("/opt/tros/humble/setup.bash", "TogetheROS humble")
    which_file("/opt/ros/humble/setup.bash", "ROS2 humble")
    which_file("/dev/ttyS2", "飞控 40PIN UART2 串口 /dev/ttyS2")
    which_file("/dev/i2c-5", "I2C5 /dev/i2c-5")

    videos = sorted(glob.glob("/dev/video*"))
    if videos:
        ok("摄像头设备", ", ".join(videos))
    else:
        fail("摄像头设备", "未找到 /dev/video*")

    model = "/opt/hobot/model/x5/basic/yolov8_640x640_nv12.bin"
    which_file(model, "YOLOv8 量化模型")

    env = os.environ.copy()
    setup = "/opt/tros/humble/setup.bash"
    if os.path.exists(setup):
        cmd = (
            f"bash -lc 'source {setup}; python3 -c \""
            "import rclpy; from cv_bridge import CvBridge; "
            "from mavros_msgs.msg import State; from mavros_msgs.srv import CommandBool, SetMode; "
            "print(\\\"rclpy+mavros_msgs+cv_bridge OK\\\")\"'"
        )
        p = subprocess.run(cmd, shell=True, capture_output=True, text=True, env=env)
        if p.returncode == 0:
            ok("ROS2:rclpy+mavros_msgs+cv_bridge", p.stdout.strip())
        else:
            fail("ROS2:rclpy+mavros_msgs+cv_bridge", (p.stderr or p.stdout).strip()[:300])

        p2 = subprocess.run(
            f"bash -lc 'source {setup}; ros2 pkg prefix mavros'",
            shell=True, capture_output=True, text=True, env=env,
        )
        if p2.returncode == 0:
            ok("ROS2:mavros 包", p2.stdout.strip())
            prefix = p2.stdout.strip()
            binary = os.path.join(prefix, "lib", "mavros", "mavros_node")
            p3 = subprocess.run(
                f"bash -lc 'source {setup}; ldd {binary}'",
                shell=True, capture_output=True, text=True, env=env)
            missing = [
                line.strip() for line in p3.stdout.splitlines()
                if "not found" in line]
            if p3.returncode == 0 and not missing:
                ok("ROS2:mavros 运行库")
            else:
                fail(
                    "ROS2:mavros 运行库",
                    "; ".join(missing) or (p3.stderr.strip()[:200]))
        else:
            fail("ROS2:mavros 包", (p2.stderr or p2.stdout).strip()[:200])

    print("=" * 50)
    n_ok = sum(1 for s, _, _ in CHECKS if s)
    n_fail = sum(1 for s, _, _ in CHECKS if not s)
    print(f"通过 {n_ok}，失败 {n_fail}")
    return 0 if n_fail == 0 else 1


if __name__ == "__main__":
    sys.exit(main())
