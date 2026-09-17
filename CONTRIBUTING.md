# 贡献说明

## 目录与命名

- 例程目录保持 `NN_snake_case`（`00_env_check` … `11_formation_flight`）。
- 跨例程逻辑只放 `_common/`，不要在各例程里复制一份。
- 新增文件使用 UTF-8、LF 换行；Python 3 shebang 为 `#!/usr/bin/env python3`，Shell 为 `#!/usr/bin/env bash`。

## 请勿提交

- `_tmp_*`、含板卡账号的临时脚本
- `__pycache__`、`*.pyc`、`*.log`、快照 `*_snapshot.jpg`
- `mavros/`、`09_depth_nav/ego_ws/`（板端构建产物）
- BPU `.bin` 模型文件

## 提交前检查

1. 例程 3–7 默认普通 USB 单目；深度链路（8/9/10）默认 MIPI + BPU Stereonet。
2. 飞行例程默认室内限速、拆桨；解锁必须显式 `arm:=true`。
3. 不要把本机路径（如 `D:\...`）写进 README。
