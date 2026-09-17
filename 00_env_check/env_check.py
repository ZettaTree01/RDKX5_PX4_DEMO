#!/usr/bin/env python3
"""00 环境一键体检/修复器：完成后直接进入例程验证与开发阶段。

目标不是“列一堆缺失项”，而是尽可能把软件环境修到可运行状态：
1. 自动选择并 source RDK X5 TogetheROS/ROS2 Humble；
2. 自动安装 Ubuntu/ROS 构建、GUI、通信、视觉依赖；
3. 自动发现并安装 apt 源中可用的 RDK hobot/mipi/stereonet 包；
4. 自动准备 colcon/rosdep/C++ EGO-Planner（09/10 共用）；
5. 修复当前用户常见 dialout/i2c 权限与 ~/.bashrc ROS 环境；
6. 检查 GS130W、Stereonet、YOLO 模型、RViz、MAVROS；
7. 对 02~11 的 launch 做“解析级”启动验证，不真正解锁/启动飞控；
8. 最终输出 READY / BLOCKED，并明确剩余项是否只是硬件未接入。

注意：RDK 厂商组件必须与板端 BSP/TogetheROS 匹配。脚本只会安装 apt 源里真实存在的
厂商包，不会从 PyPI 猜测 hbm_runtime/Hobot.GPIO 等包，避免破坏 BSP。
"""
from __future__ import annotations

import argparse
import importlib
import os
import re
import shutil
import subprocess
import sys
from pathlib import Path

PIP_INDEX_URL_CN = "https://pypi.tuna.tsinghua.edu.cn/simple"

# 国内 PyPI 镜像，供本脚本及子进程 pip 使用
os.environ.setdefault("PIP_INDEX_URL", PIP_INDEX_URL_CN)
os.environ.setdefault("PIP_TRUSTED_HOST", "pypi.tuna.tsinghua.edu.cn")
os.environ.setdefault("PIP_DEFAULT_TIMEOUT", "60")

ROOT = Path(__file__).resolve().parents[1]  # zettatree_demo 根目录
SETUPS = [Path('/opt/tros/humble/setup.bash'), Path('/opt/ros/humble/setup.bash')]
CHECKS: list[tuple[bool, str, str, str]] = []  # (ok, 名称, 详情, 类别)

# 标准软件包：不存在就安装。
APT_PACKAGES = [
    'git', 'curl', 'wget', 'ca-certificates', 'build-essential', 'cmake', 'pkg-config',
    'python3-pip', 'python3-venv', 'python3-dev', 'python3-setuptools', 'python3-wheel',
    'python3-serial', 'python3-numpy', 'python3-opencv', 'python3-pil', 'python3-paramiko',
    'python3-smbus', 'python3-smbus2', 'python3-yaml', 'python3-tk', 'python3-colcon-common-extensions', 'python3-rosdep',
    'libeigen3-dev', 'libpcl-dev', 'libarmadillo-dev', 'libopencv-dev', 'libgl1', 'libegl1',
    'libx11-6', 'libxext6', 'libxrender1', 'libxrandr2', 'libxcb-xinerama0',
]
ROS_PACKAGES = [
    'ros-humble-ros-base', 'ros-humble-rclpy', 'ros-humble-cv-bridge',
    'ros-humble-rviz2',
    'ros-humble-tf2-ros', 'ros-humble-tf2-eigen', 'ros-humble-image-transport',
    'ros-humble-pcl-ros',
    # msgs/mavlink 在 arm64 apt 上通常可用；mavros 节点本体见 ROS_MAVROS_DEBS
    'ros-humble-mavros-msgs', 'ros-humble-mavlink',
]
# 这两个在 jammy/arm64 的 packages.ros.org 上经常无 deb，需源码编译回退
ROS_MAVROS_DEBS = [
    'ros-humble-mavros', 'ros-humble-mavros-extras',
]
# TROS/RDK 包名随镜像/BSP 有差异：按 apt-cache 的实际结果选择，绝不盲装。
VENDOR_PATTERNS = {
    'mipi_cam': [
        'tros-humble-hobot-mipi-cam', 'tros-humble-mipi-cam', 'hobot-mipi-cam',
        'tros-humble-hobot-mipi_cam',
    ],
    'hobot_stereonet': [
        'tros-humble-hobot-stereonet', 'tros-humble-hobot-stereo-net',
        'hobot-stereonet',
    ],
    'hobot_dnn': [
        'tros-humble-hobot-dnn', 'tros-humble-hobot-dnn-node', 'hobot-dnn',
    ],
}

IMPORTS = {
    # Python 模块名 → apt 包名（或 pip:包名）
    'numpy': 'python3-numpy', 'cv2': 'python3-opencv', 'PIL': 'python3-pil',
    'serial': 'python3-serial', 'paramiko': 'python3-paramiko',
    'pymavlink': 'pip:pymavlink', 'smbus': 'python3-smbus', 'smbus2': 'python3-smbus2',
    'yaml': 'python3-yaml',
}


def record(ok: bool, name: str, detail: str = '', kind: str = 'software') -> None:
    """记录一项检查结果并立即打印。kind: software / vendor / hardware。"""
    CHECKS.append((ok, name, detail, kind))
    tag = '[ OK ]' if ok else '[FAIL]'
    print(f'{tag} {name}' + (f'  {detail}' if detail else ''))


def run(cmd, *, env=None, timeout=120, capture=True):
    """执行外部命令；默认捕获 stdout/stderr，返回 CompletedProcess。"""
    return subprocess.run(cmd, text=True, capture_output=capture, env=env, timeout=timeout)


def has_cmd(name: str) -> bool:
    """PATH 中是否存在可执行文件。"""
    return shutil.which(name) is not None


def import_ok(mod: str):
    """尝试 import 模块；成功返回 (True, 版本号)，失败返回 (False, 错误信息)。"""
    try:
        m = importlib.import_module(mod)
        return True, getattr(m, '__version__', '')
    except Exception as e:
        return False, str(e)


def active_setup():
    """返回当前可用的 ROS/TROS setup.bash；TROS 优先，标准 ROS 后备。"""
    for p in SETUPS:
        if p.exists():
            return p
    return None


# Debian 包名 → ROS 包名显式映射（deb 用 '-'，ROS 常用 '_'）
ROS_DEBIAN_TO_ROS = {
    "ros-humble-ros-base": "ros_base",
    "ros-humble-rclpy": "rclpy",
    "ros-humble-cv-bridge": "cv_bridge",
    "ros-humble-mavros": "mavros",
    "ros-humble-mavros-extras": "mavros_extras",
    "ros-humble-mavros-msgs": "mavros_msgs",
    "ros-humble-mavlink": "mavlink",
    "ros-humble-rviz2": "rviz2",
    "ros-humble-tf2-ros": "tf2_ros",
    "ros-humble-tf2-eigen": "tf2_eigen",
    "ros-humble-image-transport": "image_transport",
    "ros-humble-pcl-ros": "pcl_ros",
}


def _apt_installed(pkg: str) -> bool:
    """dpkg 查询该 deb 是否已 install ok installed。"""
    try:
        r = subprocess.run(
            ["dpkg-query", "-W", "-f=${Status}", pkg],
            text=True, capture_output=True, timeout=10
        )
        return r.returncode == 0 and "install ok installed" in r.stdout
    except Exception:
        return False


def _ros_name_for_deb(pkg: str) -> str:
    """将 Debian/TROS 包名粗略转为 ros2 pkg 名。"""
    if pkg in ROS_DEBIAN_TO_ROS:
        return ROS_DEBIAN_TO_ROS[pkg]
    name = pkg
    for prefix in ("ros-humble-", "tros-humble-"):
        if name.startswith(prefix):
            name = name[len(prefix):]
            break
    return name.replace("-", "_")


def ros_pkg_exists(pkg_or_ros_name: str, setup=None) -> bool:
    """在指定 ROS/TROS setup 下验证 package 是否真正可被 ROS2 发现。

    可传入 Debian 包名（ros-humble-ros-base）或 ROS 包名（ros_base / mavros）。
    """
    setup = setup or active_setup()
    if not setup:
        return False
    ros_name = _ros_name_for_deb(pkg_or_ros_name) if pkg_or_ros_name.startswith(
        ("ros-humble-", "tros-humble-")) else pkg_or_ros_name
    # 叠加本地 mavros_ws overlay（若存在）
    mavros_setup = ROOT / "mavros_ws" / "install" / "setup.bash"
    extra = f'source "{mavros_setup}" >/dev/null 2>&1; ' if mavros_setup.exists() else ""
    p = source_and_raw(
        f'{extra}ros2 pkg prefix {ros_name}',
        setup, timeout=20)
    return p.returncode == 0 and bool((p.stdout or "").strip())


def source_and_raw(bash_cmd: str, setup=None, *, timeout=120):
    """在 bash 中 source ROS setup 后执行原始 shell 字符串命令。"""
    setup = setup or active_setup()
    if not setup:
        return subprocess.CompletedProcess(bash_cmd, 1, "", "ROS setup not found")
    env = os.environ.copy()
    env.pop("RMW_IMPLEMENTATION", None)  # 避免坏 RMW 污染探测
    env["ROS_LOG_DIR"] = env.get("ROS_LOG_DIR", "/tmp/zettatree_roslog")
    return run(
        ["bash", "-lc", f'source "{setup}" >/dev/null 2>&1 && {bash_cmd}'],
        env=env, timeout=timeout)


def source_and(cmd, setup=None, *, timeout=120):
    """在 source ROS 后执行参数列表命令（自动做 shell 引号转义）。"""
    setup = setup or active_setup()
    if not setup:
        return subprocess.CompletedProcess(cmd, 1, '', 'ROS setup not found')
    escaped = ' '.join("'{}'".format(str(x).replace("'", "'\\''")) for x in cmd)
    return source_and_raw(escaped, setup, timeout=timeout)


def apt_available(pkg: str) -> bool:
    """当前 apt 源是否能解析到该包（apt-cache show）。"""
    if not has_cmd('apt-cache'):
        return False
    p = run(['apt-cache', 'show', pkg], timeout=20)
    return p.returncode == 0 and bool(p.stdout.strip())


def apt_install(packages, yes: bool, reason='依赖') -> bool:
    """安装 apt 包列表；yes=False 时交互确认。成功返回 True。"""
    packages = list(dict.fromkeys(p for p in packages if p))
    if not packages:
        return True
    if not has_cmd('apt-get'):
        print('[FAIL] 系统没有 apt-get，无法自动安装：', ' '.join(packages))
        return False
    print(f'\n[INSTALL] {reason}:')
    print('  ' + ' '.join(packages))
    if not yes:
        ans = input('现在安装这些软件包？[Y/n] ').strip().lower()
        if ans not in ('', 'y', 'yes'):
            print('[FAIL] 用户取消安装')
            return False
    prefix = [] if os.geteuid() == 0 else ['sudo']
    if prefix and not has_cmd('sudo'):
        print('[FAIL] 需要 sudo 才能安装 apt 软件包')
        return False
    p = subprocess.run(prefix + ['apt-get', 'update'], timeout=600)
    if p.returncode != 0:
        print('[FAIL] apt-get update 失败')
        return False
    p = subprocess.run(prefix + ['apt-get', 'install', '-y'] + packages, timeout=1800)
    return p.returncode == 0


def pip_supports_break_system_packages() -> bool:
    """当前 pip 是否支持 --break-system-packages（PEP 668）。"""
    p = run([sys.executable, "-m", "pip", "install", "-h"], timeout=20)
    return "--break-system-packages" in ((p.stdout or "") + (p.stderr or ""))


def pip_install_python(pkg: str, yes: bool) -> bool:
    """用当前解释器 pip 安装 Python 包；root 时按需加 break-system-packages。"""
    print(f"[INSTALL] Python pip: {pkg}")
    if not yes:
        ans = input(f"现在通过 pip 安装 {pkg}？[Y/n] ").strip().lower()
        if ans not in ("", "y", "yes"):
            return False
    cmd = [
        sys.executable, "-m", "pip", "install",
        "--disable-pip-version-check", pkg,
    ]
    # Ubuntu 22.04 自带 pip 22.x 无 --break-system-packages；23+ 才有。
    # root 安装时：有该选项才加；否则直接装（pip 会警告但不失败）。
    if os.geteuid() == 0 and pip_supports_break_system_packages():
        cmd.append("--break-system-packages")
    r = subprocess.run(cmd, timeout=900)
    if r.returncode == 0:
        return True
    # 极端情况：带 flag 失败再无 flag 重试一次
    if "--break-system-packages" in cmd:
        cmd2 = [x for x in cmd if x != "--break-system-packages"]
        r2 = subprocess.run(cmd2, timeout=900)
        return r2.returncode == 0
    return False


def ensure_standard_apt(yes: bool):
    """确保 APT_PACKAGES / IMPORTS 对应依赖已装；缺失则 apt 或 pip。"""
    # apt-cache show 对已安装/可用包均有结果；dpkg-query 判断真正缺失。
    to_install = []
    unavailable = []
    for pkg in APT_PACKAGES:
        q = run(['dpkg-query', '-W', '-f=${Status}', pkg], timeout=10)
        if 'install ok installed' not in q.stdout:
            if apt_available(pkg):
                to_install.append(pkg)
            else:
                unavailable.append(pkg)
    ok = apt_install(to_install, yes, 'Ubuntu 基础/编译/视觉/Python 依赖')
    if to_install and not ok:
        return False
    # 某些 RDK 精简镜像可能没有 Ubuntu desktop 的可选 GUI 包；
    # 这些必须在后面的 import/command 检查中明确显示为阻断，而不是静默跳过。
    for pkg in unavailable:
        record(False, f'APT:{pkg}', '当前软件源没有该包；请检查 Ubuntu 版本/软件源', 'software')
    for mod, pkg in IMPORTS.items():
        good, detail = import_ok(mod)
        record(good, f'Python:{mod}', detail if good else f'缺失；需要 {pkg}', 'software')
        if not good and pkg.startswith('pip:'):
            pip_pkg = pkg.split(':', 1)[1]
            # pymavlink 等模块在 RDK 镜像中常没有 apt 包，自动走 pip。
            if pip_install_python(pip_pkg, yes):
                good2, detail2 = import_ok(mod)
                if good2:
                    # 把之前的 FAIL 留下会导致最终 BLOCKED，因此将其转换为成功记录。
                    CHECKS[-1] = (True, f'Python:{mod}', detail2 or 'pip 安装成功', 'software')
                    continue
            record(False, f'PIP:{pip_pkg}', '自动安装失败；请检查网络/PyPI 镜像', 'software')
        elif not good and not apt_available(pkg):
            record(False, f'APT:{pkg}', 'Python 模块缺失且 apt 源无对应包', 'software')
    return ok and not unavailable


def rmw_library_present(rmw: str) -> bool:
    """在常见路径或 ldconfig 缓存中查找 lib{rmw}.so。"""
    lib = f'lib{rmw}.so'
    for base in ('/opt/tros/humble', '/opt/ros/humble', '/usr/lib', '/usr/local/lib'):
        for p in Path(base).rglob(lib) if Path(base).exists() else []:
            return True
    p = run(['ldconfig', '-p'], timeout=20)
    return p.returncode == 0 and lib in p.stdout


def choose_rmw(setup):
    """挑选本机真实存在的 RMW；无库则返回空串，交给 ROS 默认实现。"""
    # 不让陈旧/损坏的 RMW 环境变量挡住 ros2 本身运行
    candidates = ['rmw_cyclonedds_cpp', 'rmw_fastrtps_cpp']
    for rmw in candidates:
        if rmw_library_present(rmw):
            return rmw
    return ''


def ros_env(setup):
    """构造探测用环境：清掉 RMW，固定 ROS_LOG_DIR，避免 ~/.bashrc 干扰。"""
    env = os.environ.copy()
    env.pop('RMW_IMPLEMENTATION', None)
    env['ROS_LOG_DIR'] = env.get('ROS_LOG_DIR', '/tmp/zettatree_roslog')
    return env


def ensure_ros(yes: bool):
    """确保 ROS2/TROS 可用：setup、RMW、关键 ROS 包、必要时 MAVROS。"""
    setup = active_setup()
    if not setup:
        ros_candidates = [p for p in ROS_PACKAGES if apt_available(p)]
        if ros_candidates:
            if not apt_install(ros_candidates, yes, 'ROS2 Humble / RViz / MAVROS'):
                return None
            setup = active_setup()
    if not setup:
        record(False, 'ROS2 Humble/TROS',
               '未找到 /opt/tros/humble 或 /opt/ros/humble；且当前系统没有可用的 Humble 安装源',
               'software')
        return None

    record(True, 'ROS setup', str(setup))
    clean = ros_env(setup)
    p = subprocess.run(
        ['bash', '-lc', f'source "{setup}" >/dev/null 2>&1 && ros2 -h'],
        text=True, capture_output=True, env=clean, timeout=30)
    if p.returncode != 0:
        record(False, 'ros2 命令', (p.stderr or p.stdout).strip()[:500])
        return None
    record(True, 'ros2 命令', 'CLI 可用')

    # 尽量保证至少有一个 RMW 能真正加载
    for rmw_pkg in ('ros-humble-rmw-cyclonedds-cpp', 'ros-humble-rmw-fastrtps-cpp'):
        if not (rmw_library_present('rmw_cyclonedds_cpp') if 'cyclone' in rmw_pkg
                else rmw_library_present('rmw_fastrtps_cpp')):
            if apt_available(rmw_pkg):
                apt_install([rmw_pkg], yes, 'ROS2 RMW')
    rmw = choose_rmw(setup)
    if rmw:
        test_env = ros_env(setup)
        test_env['RMW_IMPLEMENTATION'] = rmw
        rt = subprocess.run(
            ['bash','-lc',
             f'source "{setup}" >/dev/null 2>&1 && ros2 topic list'],
            text=True, capture_output=True, env=test_env, timeout=30)
        if rt.returncode == 0:
            record(True, f'RMW:{rmw}', '动态库可加载，ros2 topic list 实测通过')
        else:
            record(False, f'RMW:{rmw}', (rt.stderr or rt.stdout).strip()[:500], 'software')
            rmw = ''
    if not rmw:
        record(False, 'ROS2 RMW',
               '没有可实际加载的 rmw_cyclonedds_cpp / rmw_fastrtps_cpp',
               'software')

    # TROS 镜像可能已带部分 ROS，而 Ubuntu apt 索引未必完整。
    # 对缺失包：先看 apt-cache；索引没有也尝试 apt-get，以拿到真实源错误。
    missing = []
    for pkg in ROS_PACKAGES:
        if not ros_pkg_exists(pkg, setup) and not _apt_installed(pkg):
            missing.append(pkg)
        elif not ros_pkg_exists(pkg, setup) and _apt_installed(pkg):
            # deb 已装但 ros2 暂时看不见时，仍算候选（多数能被 setup 找到）
            pass

    really_missing = [pkg for pkg in ROS_PACKAGES if not ros_pkg_exists(pkg, setup)]
    if really_missing:
        to_apt = [pkg for pkg in really_missing if apt_available(pkg) or pkg in missing]
        # 去重并只装 apt 能解析的
        to_apt = [pkg for pkg in dict.fromkeys(to_apt) if apt_available(pkg)]
        if to_apt:
            if not apt_install(to_apt, yes, "缺失 ROS2 软件包"):
                visible = [pkg for pkg in to_apt if apt_available(pkg)]
                if visible:
                    apt_install(visible, yes, "再次安装可用 ROS2 软件包")
        hidden = [pkg for pkg in really_missing if not apt_available(pkg)]
        for pkg in hidden:
            # mavros 节点本体走源码回退，这里不记硬 FAIL
            if pkg in ROS_MAVROS_DEBS:
                continue
            record(False, f"APT:{pkg}",
                   "当前软件源无法解析该 ROS2 包；请检查 TROS/ROS2 apt 源",
                   "software")

    # MAVROS 节点：apt 有则装；没有则源码编译到 mavros_ws（仅 --yes / 交互确认时）
    if yes:
        ensure_mavros(setup, yes)
    elif not ros_pkg_exists("mavros", setup):
        record(False, "MAVROS",
               "未安装；jammy/arm64 常无 apt 包，请: bash run.sh --yes "
               "或 bash 00_env_check/setup_mavros.sh",
               "software")

    # apt / mavros 安装后重新解析 setup
    setup = active_setup() or setup
    for pkg in ROS_PACKAGES:
        ok = ros_pkg_exists(pkg, setup) or _apt_installed(pkg)
        # ros_base 等 meta 包：deb 已装即 OK（ros2 pkg 名带下划线）
        if not ok and pkg == "ros-humble-ros-base" and _apt_installed(pkg):
            ok = True
        if not ok:
            ok = ros_pkg_exists(_ros_name_for_deb(pkg), setup)
        record(ok, f"ROS2:{_ros_name_for_deb(pkg)}",
               "" if ok else f"缺失 {pkg}")
    # mavros 节点单独记录
    mav_ok = ros_pkg_exists("mavros", setup)
    record(mav_ok, "ROS2:mavros",
           "" if mav_ok else "缺失；已尝试 apt / 源码编译（见 setup_mavros.sh）")
    return setup


def ensure_mavros(setup, yes: bool) -> bool:
    """安装 MAVROS 节点。优先 apt；jammy/arm64 常无 deb 时回退源码编译。"""
    if ros_pkg_exists("mavros", setup):
        record(True, "MAVROS", "已可用")
        return True

    # 先装 msgs（import / 消息定义）
    msgs = [p for p in ("ros-humble-mavros-msgs", "ros-humble-mavlink") if apt_available(p)]
    if msgs:
        apt_install(msgs, yes, "MAVROS 消息 / mavlink")

    debs = [p for p in ROS_MAVROS_DEBS if apt_available(p)]
    if debs:
        if apt_install(debs, yes, "MAVROS apt 包") and ros_pkg_exists("mavros", setup):
            record(True, "MAVROS", "apt 安装成功")
            return True

    script = ROOT / "00_env_check" / "setup_mavros.sh"
    if not script.exists():
        record(False, "MAVROS",
               "apt 无 ros-humble-mavros，且缺少 setup_mavros.sh", "software")
        return False

    print("\n[MAVROS] apt 源无 arm64 的 ros-humble-mavros，改为源码编译（约 10~30 分钟）…")
    if not yes:
        ans = input("现在源码编译安装 MAVROS？[Y/n] ").strip().lower()
        if ans not in ("", "y", "yes"):
            record(False, "MAVROS", "用户取消源码安装", "software")
            return False

    env = os.environ.copy()
    env["DEMO_ROOT"] = str(ROOT)
    env["MAVROS_WS"] = str(ROOT / "mavros_ws")
    env["ROS_SETUP"] = str(setup) if setup else ""
    p = subprocess.run(["bash", str(script)], env=env, timeout=3600)
    if p.returncode != 0:
        record(False, "MAVROS",
               f"源码编译失败；请手动: bash {script}", "software")
        return False
    ok = ros_pkg_exists("mavros", setup)
    record(ok, "MAVROS",
           str(ROOT / "mavros_ws") if ok else "编译结束但仍不可用", "software")
    return ok

def ensure_vendor(setup, yes: bool):
    """安装 apt 源中真实存在的 TROS/RDK 包，并验证 ros2 package/executable。"""
    if not setup:
        return
    chosen = []
    for logical, candidates in VENDOR_PATTERNS.items():
        if ros_pkg_exists('mipi_cam' if logical == 'mipi_cam' else 'hobot_stereonet' if logical == 'hobot_stereonet' else 'hobot_dnn', setup):
            continue
        found = next((p for p in candidates if apt_available(p)), None)
        if found:
            chosen.append(found)
        else:
            print(f'[WARN] apt 源未发现 {logical} 候选包（可能已随 BSP 内置或需要重新安装官方 TROS/BSP）')
    if chosen and not apt_install(chosen, yes, 'RDK/TogetheROS 厂商组件'):
        return

    for pkg, logical in [('mipi_cam', 'GS130W mipi_cam'), ('hobot_stereonet', 'StereoNet/BPU'), ('hobot_dnn', 'Hobot DNN')]:
        ok = ros_pkg_exists(pkg, setup)
        record(ok, logical, '' if ok else f'ROS 包 {pkg} 不存在', 'vendor')
        if ok:
            p = source_and(['ros2', 'pkg', 'executables', pkg], setup, timeout=20)
            record(p.returncode == 0 and bool(p.stdout.strip()), f'{pkg} executables', p.stdout.strip().replace('\n', '; ')[:300], 'vendor')

    # hbm_runtime 是 Python 模块；只有已经随 BSP 提供才视为正确。
    for mod in ['hbm_runtime', 'Hobot.GPIO', 'i2cdev']:
        ok, detail = import_ok(mod)
        record(ok, f'Python:{mod}', detail if ok else '未安装；必须来自匹配的 RDK BSP/TROS', 'vendor')


def ensure_ego(yes: bool, skip_ego: bool):
    """准备 09/10 共用的 C++ EGO-Planner 工作空间（可 --skip-ego 跳过）。"""
    if skip_ego:
        record(True, 'EGO-Planner', '用户指定 --skip-ego，09/10 的完整 C++ EGO 暂不安装', 'software')
        return True
    script = ROOT / '09_depth_nav' / 'setup_full_ego.sh'
    if not script.exists():
        record(False, 'EGO-Planner', '缺少 setup_full_ego.sh', 'software')
        return False
    ws = Path(os.environ.get('EGO_WS', str(ROOT / '09_depth_nav' / 'ego_ws')))
    ready = (ws / 'install' / 'setup.bash').exists()
    pkg_ready = False
    setup = active_setup()
    if setup and ready:
        pkg_ready = ros_pkg_exists('ego_planner', setup)
    if ready and pkg_ready:
        record(True, 'EGO-Planner', str(ws), 'software')
        return True
    if not has_cmd('git'):
        record(False, 'EGO-Planner', 'git 不可用', 'software')
        return False
    print('\n[EGO] 09/10 需要完整 C++ EGO-Planner。首次安装会下载并编译，上板可能需要 10~20 分钟。')
    if not yes:
        ans = input('现在自动安装/编译 EGO-Planner？[Y/n] ').strip().lower()
        if ans not in ('', 'y', 'yes'):
            record(False, 'EGO-Planner', '用户取消；09/10 完整模式暂不可用', 'software')
            return False
    env = os.environ.copy()
    env['EGO_WS'] = str(ws)
    p = subprocess.run(['bash', str(script)], cwd=str(script.parent), env=env, timeout=3600)
    if p.returncode != 0:
        record(False, 'EGO-Planner', f'自动编译失败，日志见 {ws}/log', 'software')
        return False
    # 新建的 overlay 必须再次验证。
    if (ws / 'install' / 'setup.bash').exists():
        record(True, 'EGO-Planner', str(ws), 'software')
        return True
    record(False, 'EGO-Planner', '编译结束但 install/setup.bash 不存在', 'software')
    return False


def persist_shell_env(setup):
    """让新开终端直接有 ros2；不能修改父 shell，所以同时输出当前终端命令。"""
    bashrc = Path.home() / '.bashrc'
    marker = '# >>> zettatree_demo ROS2 environment >>>'
    mavros_setup = ROOT / 'mavros_ws' / 'install' / 'setup.bash'
    mavros_line = (
        f'if [ -f "{mavros_setup}" ]; then source "{mavros_setup}"; fi\n'
        if True else ''
    )
    block = f'''\n{marker}\nif [ -f "{setup}" ]; then source "{setup}"; fi\n{mavros_line}export ROS_LOG_DIR="${{ROS_LOG_DIR:-/tmp/zettatree_roslog}}"\nif [ -z "${{RMW_IMPLEMENTATION:-}}" ]; then\n  if ldconfig -p 2>/dev/null | grep -q 'librmw_cyclonedds_cpp.so'; then export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp;\n  elif ldconfig -p 2>/dev/null | grep -q 'librmw_fastrtps_cpp.so'; then export RMW_IMPLEMENTATION=rmw_fastrtps_cpp;\n  else unset RMW_IMPLEMENTATION; fi\nfi\n# <<< zettatree_demo ROS2 environment <<<\n'''
    try:
        text = bashrc.read_text(encoding='utf-8') if bashrc.exists() else ''
        if marker not in text:
            bashrc.parent.mkdir(parents=True, exist_ok=True)
            with bashrc.open('a', encoding='utf-8') as f:
                f.write(block)
            record(True, '~/.bashrc ROS2 环境', '已加入，重新打开终端自动生效')
        else:
            # 旧块存在时，若缺 mavros overlay 则补一行
            if str(mavros_setup) not in text and mavros_setup.exists():
                with bashrc.open('a', encoding='utf-8') as f:
                    f.write(
                        f'\n# zettatree mavros overlay\n'
                        f'if [ -f "{mavros_setup}" ]; then source "{mavros_setup}"; fi\n'
                    )
            record(True, '~/.bashrc ROS2 环境', '已配置')
    except Exception as e:
        record(False, '~/.bashrc ROS2 环境', str(e))


def ensure_groups(yes: bool):
    """把当前用户加入 dialout/i2c 组（串口与 I2C 权限，需重新登录生效）。"""
    if os.geteuid() == 0:
        return
    user = os.environ.get('USER') or os.environ.get('LOGNAME')
    if not user:
        return
    groups = set()
    p = run(['id', '-nG', user], timeout=10)
    groups.update(p.stdout.split())
    missing = [g for g in ('dialout', 'i2c') if shutil.which('getent') and run(['getent','group',g],timeout=10).returncode == 0 and g not in groups]
    if not missing:
        record(True, '设备权限组', 'dialout/i2c 已具备或不需要', 'hardware')
        return
    prefix = [] if os.geteuid() == 0 else ['sudo']
    if not yes:
        ans=input(f'当前用户缺少 {", ".join(missing)}，现在加入并为串口/I2C准备权限？[Y/n] ').strip().lower()
        if ans not in ('','y','yes'):
            record(False, '设备权限组', f'缺少 {missing}；重新登录后才会生效', 'hardware')
            return
    ok=True
    for g in missing:
        r=subprocess.run(prefix+['usermod','-aG',g,user],timeout=30)
        ok &= r.returncode==0
    if ok:
        record(True, '设备权限组', f'已加入 {", ".join(missing)}；请重新登录/重启后生效', 'hardware')
    else:
        record(False, '设备权限组', 'usermod 失败', 'hardware')


def check_devices():
    """检查飞控串口、I2C 与摄像头设备节点是否存在及可读写。"""
    for path, label in [('/dev/ttyS2','PX4 UART2 /dev/ttyS2'),('/dev/i2c-5','I2C5 /dev/i2c-5')]:
        if os.path.exists(path):
            access = os.access(path, os.R_OK | os.W_OK)
            record(access, label, '存在且当前用户可读写' if access else '存在但当前用户无读写权限', 'hardware')
        else:
            record(False, label, '设备不存在：可能未接飞控/驱动未启用', 'hardware')
    videos=sorted(Path('/dev').glob('video*'))
    record(True,'摄像头设备',', '.join(map(str,videos)) if videos else '未找到 /dev/video*；GS130W MIPI 模式不依赖 /dev/video*', 'hardware')


def check_model():
    """检查 YOLOv8 BPU 模型与 COCO 类别文件；缺失时可生成本地标签表。"""
    candidates=[
        Path('/opt/hobot/model/x5/basic/yolov8_640x640_nv12.bin'),
        Path('/opt/hobot/model/x5/basic/yolov8n_detect.bin'),
    ]
    hit=next((p for p in candidates if p.exists()),None)
    record(bool(hit),'YOLOv8 BPU 模型',str(hit) if hit else '未找到；04/05/10 不能使用 BPU YOLO', 'vendor')
    names=Path('/opt/hobot/model/x5/basic/coco_classes.names')
    local=ROOT/'04_object_detection'/'coco_classes.names'
    if not names.exists() and not local.exists():
        labels='''person\nbicycle\ncar\nmotorcycle\nairplane\nbus\ntrain\ntruck\nboat\ntraffic light\nfire hydrant\nstop sign\nparking meter\nbench\nbird\ncat\ndog\nhorse\nsheep\ncow\nelephant\nbear\nzebra\ngiraffe\nbackpack\numbrella\nhandbag\ntie\nsuitcase\nfrisbee\nskis\nsnowboard\nsports ball\nkite\nbaseball bat\nbaseball glove\nskateboard\nsurfboard\ntennis racket\nbottle\nwine glass\ncup\nfork\nknife\nspoon\nbowl\nbanana\napple\nsandwich\norange\nbroccoli\ncarrot\nhot dog\npizza\ndonut\ncake\nchair\ncouch\npotted plant\nbed\ndining table\ntoilet\ntv\nlaptop\nmouse\nremote\nkeyboard\ncell phone\nmicrowave\novel\ntoaster\nsink\nrefrigerator\nbook\nclock\nvase\nscissors\nteddy bear\nhair drier\ntoothbrush\n'''
        local.write_text(labels,encoding='utf-8')
    if not names.exists() and local.exists():
        names=local
    record(names.exists(),'COCO 类别文件',str(names) if names.exists() else '未找到；已无法生成 COCO 标签表', 'vendor')


def verify_ros_imports(setup):
    """在 source 后用一行 python 校验 rclpy/cv_bridge/mavros_msgs 等导入。"""
    code='''import rclpy; import cv_bridge; import sensor_msgs.msg; import geometry_msgs.msg; import nav_msgs.msg; import tf2_ros; import mavros_msgs.msg; import mavros_msgs.srv; print("ROS imports OK")'''
    p=source_and(['python3','-c',code],setup,timeout=30)
    record(p.returncode==0,'ROS2 Python imports',(p.stdout or p.stderr).strip()[:400])


def verify_launches(setup):
    """只解析 launch，不启动相机、MAVROS、飞控、RViz，不会产生飞行动作。"""
    launches=[
        ROOT/'02_bench_pose_sim/bench_pose_sim.launch.py',
        ROOT/'04_object_detection/object_detection.launch.py',
        ROOT/'05_obstacle_avoidance/obstacle_avoidance.launch.py',
        ROOT/'06_autonomous_cruise/autonomous_cruise.launch.py',
        ROOT/'07_target_tracking/target_tracking.launch.py',
        ROOT/'08_depth_camera/depth_camera.launch.py',
        ROOT/'09_depth_nav/depth_nav.launch.py',
        ROOT/'09_depth_nav/ego_full.launch.py',
        ROOT/'10_target_follow/target_follow.launch.py',
        ROOT/'11_formation_flight/formation_flight.launch.py',
    ]
    ok_all=True
    for f in launches:
        if not f.exists():
            record(False,f.name,'文件不存在')
            ok_all=False
            continue
        p=source_and(['ros2','launch',str(f),'--show-args'],setup,timeout=60)
        ok=p.returncode==0
        record(ok,f'Launch解析:{f.parent.name}/{f.name}',(p.stderr or '').strip()[:300] if not ok else '参数解析通过')
        ok_all &= ok
    return ok_all


def verify_commands(setup):
    """检查 rviz2/colcon/git 命令与 mavros/mipi_cam/stereonet 包是否可见。"""
    checks=[('rviz2','RViz2'),('colcon','colcon'),('git','git')]
    all_ok=True
    for cmd,label in checks:
        ok=has_cmd(cmd)
        record(ok,label,shutil.which(cmd) if ok else '命令不存在')
        all_ok &= ok
    for pkg,label in [('mavros','MAVROS'),('mipi_cam','mipi_cam'),('hobot_stereonet','hobot_stereonet')]:
        ok=ros_pkg_exists(pkg,setup)
        record(ok,label,'' if ok else f'ROS package {pkg} 不存在','vendor' if pkg!='mavros' else 'software')
        all_ok &= ok
    return all_ok


def main():
    """命令行入口：体检/修复环境，汇总 READY 或 BLOCKED。"""
    ap=argparse.ArgumentParser()
    ap.add_argument('--yes',action='store_true',help='无需询问，自动安装/编译')
    ap.add_argument('--check-only',action='store_true',help='只检查，不修改环境')
    ap.add_argument('--skip-ego',action='store_true',help='跳过 09/10 C++ EGO-Planner')
    args=ap.parse_args()
    auto=args.yes and not args.check_only

    print('RDK X5 无人机例程环境一键体检 / 修复器 v2.9')
    print('='*76)
    print('目标：脚本成功结束后，可直接进入 01~11 例程验证与开发。')
    print('不会自动解锁、不会启动飞控、不会启动相机、不会让电机转动。')
    print('Python:',sys.version.replace('\n',' '))

    if not args.check_only:
        ensure_standard_apt(auto)
    setup=ensure_ros(auto)
    if setup:
        if not args.check_only:
            persist_shell_env(setup)
            ensure_groups(auto)
        ensure_vendor(setup,auto)
        if not args.check_only:
            # EGO 构建会复用上面的 ROS 环境。
            ensure_ego(auto,args.skip_ego)
        verify_ros_imports(setup)
        verify_commands(setup)
        verify_launches(setup)
    else:
        record(False,'ROS2 总环境','无法建立 ROS setup')

    check_devices()
    check_model()

    print('='*76)
    hard_fail=[x for x in CHECKS if not x[0] and x[3]=='software']
    vendor_fail=[x for x in CHECKS if not x[0] and x[3]=='vendor']
    hw_fail=[x for x in CHECKS if not x[0] and x[3]=='hardware']
    n_ok=sum(x[0] for x in CHECKS)
    print(f'检查完成：OK={n_ok}  软件阻断={len(hard_fail)}  厂商环境阻断={len(vendor_fail)}  硬件未就绪={len(hw_fail)}')

    if not hard_fail and not vendor_fail:
        print('\nREADY：软件/ROS/RDK 运行环境已准备完成，可以进入例程验证与开发阶段。')
        if hw_fail:
            print('注意：当前剩余 FAIL 仅为硬件项；接上对应硬件并重启/重新登录后即可继续。')
        print('\n当前终端如果仍提示 ros2: command not found，请执行：')
        print(f'  source {setup}')
        print('新开终端会自动加载（脚本已写入 ~/.bashrc）。')
        return 0
    print('\nBLOCKED：仍存在软件/厂商环境阻断项。请根据上面的 FAIL 修复后重新运行本脚本。')
    return 1


if __name__=='__main__':
    raise SystemExit(main())
