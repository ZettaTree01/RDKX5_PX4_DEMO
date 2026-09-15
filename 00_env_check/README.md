# 00 环境自检

## 例程说明

不接飞控、不接线也能运行。运行 `env_check.py` 确认本套 **无人机例程** 所需的运行环境：

- Python 库；
- 飞控 40PIN UART2 串口（`/dev/ttyS2`）；
- ROS2 / MAVROS；
- YOLO 模型。

## 本节目标

- 了解本套例程的运行依赖：RDK X5、飞控、40PIN 串口、ROS2 / MAVROS、模型文件。
- 使用一条命令完成环境自检，并根据输出定位缺失项。
- 养成先自检再上电的习惯，避免将环境问题误判为代码故障。

```bash
python3 /app/zettatree_demo/00_env_check/env_check.py
```
