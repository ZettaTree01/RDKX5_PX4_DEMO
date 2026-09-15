# -*- coding: utf-8 -*-
"""例程10：自动部署与远程调试（从开发机运行，Windows/Linux 均可）。

用法：
    python deploy_and_debug.py              # 同步例程10 → 远程自检
    python deploy_and_debug.py --bench      # 同步 + 台架实跑 25 秒并抓日志
    python deploy_and_debug.py --direct     # 台架实跑用 planner:=direct
    python deploy_and_debug.py --check-only # 不上传，仅远程自检

依赖：pip install paramiko
依赖链：EGO 工作空间复用例程 09（setup_full_ego.sh 已构建+打补丁），
本脚本只做同步、自检与台架冒烟，不在开发机上编译任何东西。
"""
import argparse
import os
import posixpath
import stat
import sys
import time

import paramiko

HOST = os.environ.get("ONBOARD_HOST", "192.168.101.168")
USER = os.environ.get("ONBOARD_USER", "sunrise")
PASSWORD = os.environ.get("ONBOARD_PASS", "sunrise")

LOCAL_DIR = os.path.dirname(os.path.abspath(__file__))
REMOTE_DIR = "/app/zettatree_demo/10_target_follow"
EGO_WS = "/app/zettatree_demo/09_depth_nav/ego_ws"
EGO_FSM = posixpath.join(
    EGO_WS, "src", "ego-planner-swarm", "src", "planner",
    "plan_manage", "src", "ego_replan_fsm.cpp")
YOLO_MODELS = [
    "/opt/hobot/model/x5/basic/yolov8_640x640_nv12.bin",
    "/app/pydev_demo/02_detection_sample/03_ultralytics_yolov8/"
    "yolov8x_detect_bayese_640x640_nv12.bin",
]
SKIP_NAMES = {"__pycache__", "deploy_and_debug.py", ".git"}
EXEC_EXT = {".py", ".sh", ".rviz"}

CHECKS = []          # (title, ok, detail)


def connect():
    c = paramiko.SSHClient()
    c.set_missing_host_key_policy(paramiko.AutoAddPolicy())
    c.connect(HOST, username=USER, password=PASSWORD, timeout=15)
    return c


def run(c, cmd, timeout=60):
    """执行远程命令，返回 (rc, out, err)。"""
    stdin, stdout, stderr = c.exec_command(cmd, timeout=timeout)
    rc = stdout.channel.recv_exit_status()
    return rc, stdout.read().decode("utf-8", "replace"), \
        stderr.read().decode("utf-8", "replace")


def record(title, ok, detail=""):
    CHECKS.append((title, ok, detail.strip()))
    mark = "PASS" if ok else "FAIL"
    print(f"[{mark}] {title}" + (f"\n       {detail.strip()}" if detail and not ok else ""))


def deploy(c):
    """上传例程10（LF 转换 + 可执行权限）。"""
    sftp = c.open_sftp()
    uploaded = 0
    for dirpath, dirnames, filenames in os.walk(LOCAL_DIR):
        dirnames[:] = [d for d in dirnames if d not in SKIP_NAMES]
        for name in filenames:
            if name in SKIP_NAMES:
                continue
            local = os.path.join(dirpath, name)
            rel = os.path.relpath(local, LOCAL_DIR)
            remote = posixpath.join(
                REMOTE_DIR, rel.replace("\\", "/"))
            # 递归建目录
            rdir = posixpath.dirname(remote)
            parts, cur = rdir.split("/"), ""
            for p in parts:
                if not p:
                    continue
                cur += "/" + p
                try:
                    sftp.stat(cur)
                except OSError:
                    sftp.mkdir(cur)
            # Windows CRLF 会让 bash/py 出错
            with open(local, "rb") as f:
                data = f.read().replace(b"\r\n", b"\n")
            with sftp.file(remote, "wb") as rf:
                rf.write(data)
            if os.path.splitext(name)[1] in EXEC_EXT:
                sftp.chmod(remote, stat.S_IRWXU | stat.S_IRGRP
                           | stat.S_IXGRP | stat.S_IROTH | stat.S_IXOTH)
            uploaded += 1
            print("put", rel)
    sftp.close()
    print(f"uploaded {uploaded} files -> {REMOTE_DIR}")
    return uploaded


def remote_checks(c):
    """远程自检：语法 / EGO / 模型 / 依赖。"""
    rc, out, err = run(c, (
        "cd /app/zettatree_demo/10_target_follow && "
        "python3 -m py_compile target_follow.py target_follow.launch.py "
        "&& echo OK"))
    record("Python 语法（target_follow.py / launch）", rc == 0, err or out)

    rc, out, err = run(c, f"test -f '{EGO_WS}/install/setup.bash' && echo OK")
    record("EGO 工作空间已编译（复用例程09）", rc == 0,
           "未找到 install/setup.bash，请先跑 setup.sh")

    rc, out, err = run(c, f"grep -q 'ZETTATREE: dynamic goal replan' "
                          f"'{EGO_FSM}' && echo OK")
    record("EGO 动态目标补丁已应用", rc == 0,
           "ego_replan_fsm.cpp 缺补丁标记，动态跟随会 abort")

    rc, out, err = run(c,
                       "ls " + " ".join(f"'{m}'" for m in YOLO_MODELS)
                       + " 2>/dev/null | head -1")
    record("YOLO 模型存在", rc == 0 and out.strip(), "未找到 yolov8 模型 bin")

    rc, out, err = run(c, "ls /app/zettatree_demo/08_depth_camera/"
                          "start_stereonet.sh >/dev/null && echo OK")
    record("例程08 Stereonet 脚本存在", rc == 0)

    rc, out, err = run(c, (
        "source /opt/tros/humble/setup.bash 2>/dev/null "
        "|| source /opt/ros/humble/setup.bash; "
        "python3 -c 'import rclpy, cv2, cv_bridge; print(\"OK\")'"))
    record("ROS2 Python 依赖（rclpy/cv2/cv_bridge）", "OK" in out, err)

    rc, out, err = run(c, (
        "source /opt/tros/humble/setup.bash 2>/dev/null "
        "|| source /opt/ros/humble/setup.bash; "
        f"source '{EGO_WS}/install/setup.bash' 2>/dev/null; "
        "ros2 pkg prefix ego_planner >/dev/null 2>&1 && echo OK"))
    record("ego_planner 包可用", "OK" in out, err)

    rc, out, err = run(c, (
        "bash /app/zettatree_demo/10_target_follow/run.sh "
        "--help 2>&1 | head -1; true"))
    # run.sh 无 --help；只验证它存在且可执行即可
    rc, out, err = run(c, "test -x /app/zettatree_demo/10_target_follow/"
                          "run.sh && echo OK")
    record("run.sh 可执行", "OK" in out)


def bench_run(c, planner="ego", duration=25):
    """台架实跑：bench:=true + arm:=false，抓取日志并检查错误。"""
    log = f"/tmp/tf10_bench_{int(time.time())}.log"
    cmd = (
        "cd /app/zettatree_demo/10_target_follow && "
        f"(timeout {duration + 15} bash run.sh "
        f"bench:=true arm:=false rviz:=false show:=false "
        f"snapshot:=/tmp/tf10_snapshot.jpg "
        f"planner:={planner} >{log} 2>&1; true) && "
        # 全量日志取证：标记行 + 致命错误 + 尾部输出
        f"echo '--- markers ---' && "
        f"grep -aE '跟随源=|目标跟随 planner=|快照已更新' {log} | head -6 && "
        f"echo '--- tail ---' && tail -c 8000 {log}"
    )
    print(f"[bench] 台架实跑 planner:={planner}，{duration}s …")
    rc, out, err = run(c, cmd, timeout=duration + 90)
    print(out[-6000:])
    # markers 段由远程 grep 输出；tail 段用于致命错误判定
    tail_part = out.split("--- tail ---", 1)[-1] if "--- tail ---" in out else out
    fatal = [ln for ln in tail_part.splitlines() if (
        "Traceback (most recent call last)" in ln
        or "process has died" in ln
        or "already been added to an executor" in ln
        or "Segmentation fault" in ln)]
    started = ("目标跟随 planner=" in out) and ("跟随源=" in out)
    record(f"台架实跑 planner:={planner}（无致命错误）",
           started and not fatal,
           "\n".join(fatal[:10]) if fatal else "未见 target_follow 启动标记")
    rc, _, _ = run(c, f"rm -f {log}")
    return not fatal and started


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--bench", action="store_true", help="部署后台架实跑 ego")
    ap.add_argument("--direct", action="store_true",
                    help="台架实跑改用 planner:=direct")
    ap.add_argument("--check-only", action="store_true", help="只自检不上传")
    ap.add_argument("--duration", type=int, default=25, help="台架时长(秒)")
    args = ap.parse_args()

    c = connect()
    try:
        if not args.check_only:
            deploy(c)
        remote_checks(c)
        if args.bench or args.direct:
            bench_run(c, "direct" if args.direct else "ego", args.duration)
    finally:
        c.close()

    fails = [t for t, ok, _ in CHECKS if not ok]
    print("\n==== 自检结果 ====")
    for t, ok, _ in CHECKS:
        print(f"  {'PASS' if ok else 'FAIL'}  {t}")
    if fails:
        print(f"\n{len(fails)} 项失败")
        sys.exit(1)
    print("\n全部通过")


if __name__ == "__main__":
    main()
