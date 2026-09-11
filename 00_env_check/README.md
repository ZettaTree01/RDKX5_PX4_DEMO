# 00 环境自检

## 例程说明

不接飞控、不接线也能运行。运行 `env_check.py` 确认本套 **无人机例程** 所需的运行环境：

- Python 库；
- 飞控 40PIN UART2 串口（`/dev/ttyS2`）；
- ROS2 / MAVROS；
- YOLO 模型。

## 跑完这节能确认

- 这套例程依赖什么：RDK X5、飞控、40PIN 串口、ROS2 / MAVROS、模型文件。
- 一条命令自检，缺什么从输出里能对上。
- 先自检再上电，别把环境问题当成代码坏了。

```bash
python3 /app/zettatree_demo/00_env_check/env_check.py
```
