#!/usr/bin/env bash
# 00 环境体检入口：转发全部参数给同目录 env_check.py。
# 常用：bash run.sh --yes（自动安装）/ bash run.sh --check-only（只检查）
set -e
SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
exec python3 "$SCRIPT_DIR/env_check.py" "$@"
