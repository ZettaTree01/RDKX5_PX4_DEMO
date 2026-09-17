# 01 机载计算机 ↔ 飞控主板：40PIN 针脚串口交互（例程1）

文档 `RDK_X5_AI_Tutorial.md` §1.2.7。

## 例程说明

验证 **RDK X5（机载计算机）与飞控主板 ZP-PV601 之间通过 40PIN 针脚串口的 MAVLink 交互**：
本教程使用 **UART2**，X5 上设备节点为 `/dev/ttyS2`，飞控端接 TELEM 串口（须已开启 MAVLink）。

例程分两步：**步骤 1** 只读飞控姿态，验证链路通联；**步骤 2**（可选，必须拆桨）做电机测试。

## 本节目标

- 理解机载与飞控链路：40PIN UART2 ↔ TELEM（`/dev/ttyS2@57600`）。
- 先以只读姿态确认链路通联，再进行控制。
- 掌握拆桨条件下的电机测试流程与安全边界。
- 后续 02～08 飞行例程均复用该串口。

```
RDK X5 40PIN(UART2 /dev/ttyS2@57600) --杜邦线-- 飞控主板 ZP-PV601 TELEM
```

## 实际接线（与 X5 默认 UART1 不同）

| X5 40PIN 物理脚 | 功能 | 接到飞控 | 备注 |
|---|---|---|---|
| **PIN20** | GND | GND | 共地 |
| **PIN22** | UART2_RXD | 飞控主板 **TX** | 收飞控数据 |
| **PIN15** | UART2_TXD | 飞控主板 **RX** | 发数据给飞控 |

> **为什么不用默认的 UART1（PIN8/PIN10）？** 本例程使用 PIN20/22/15，
> 该组引脚属于 **UART2**。X5 出厂设备树默认只使能 UART1，
> UART2（`34080000.serial` → `/dev/ttyS2`）**默认是 disabled**，首次使用需使能并重启，
> 完整步骤见根 README《接线与 UART2 使能》，或文档对应附录。

- 针脚串口是真实 UART，**波特率必须与飞控 TELEM 端口一致**（本教程 57600）；
  不接 USB CDC 那种“波特率随便写”的机制。
- 当前用户需在 `dialout` 组：
  `sudo usermod -aG dialout $USER`，重新登录后生效。

先确认设备存在且 UART2 已使能：

```bash
ls -l /dev/ttyS2        # crw-rw---- 1 root dialout 4, 66 ... /dev/ttyS2
```

## 验证链路（步骤 1）：读取飞控姿态

`attitude_via_usb.py`（文件名沿用例程初版命名，链路实为 40PIN UART2）从飞控主板 ZP-PV601
订阅 `ATTITUDE(30)`，打印 roll/pitch/yaw 与三轴角速度。
**能出姿态 = 机载↔飞控的针脚串口链路是通的**；一帧都没有 = 链路层有问题。

```bash
# 40PIN UART2 直连飞控（默认 /dev/ttyS2 @57600，20Hz，跑 10 秒）
python3 /app/zettatree_demo/01_uart_serial/attitude_via_usb.py --rate 20 --duration 10
```

只订阅、**不下发任何控制指令**（不会解锁或切模式）；退出时自动把 ATTITUDE 速率恢复默认。

## 电机测试（拆桨后，进阶，步骤 2 可选）

`motor_test_via_usb.py` 走同一条针脚串口，通过 MAVLink shell 执行飞控自带的
`actuator_test`，让电机实际转动。

> 注意：PX4 **不支持** `MAV_CMD_DO_MOTOR_TEST`(209)（返回 UNSUPPORTED），
> `MAV_CMD_ACTUATOR_TEST`(310) 也无响应，只能用 shell 里的 `actuator_test`。

```bash
# 只探测可用性，不转电机
python3 /app/zettatree_demo/01_uart_serial/motor_test_via_usb.py --probe

# 拆桨后斜坡加速 6 秒（怠速 → 最高 300 r/min）
python3 /app/zettatree_demo/01_uart_serial/motor_test_via_usb.py \
    --duration 6 --i-am-sure

# 只转 1 号电机
python3 /app/zettatree_demo/01_uart_serial/motor_test_via_usb.py \
    --motor 1 --duration 3 --i-am-sure

# 飞控自带流程：依次点动全部电机（逐个启停，非同步）
python3 /app/zettatree_demo/01_uart_serial/motor_test_via_usb.py --iterate --i-am-sure
```

**默认就是全部电机同步启动**：程序从 `SERVO_OUTPUT_RAW` 的非零通道数自动识别电机
数量（四旋翼为 4），然后以 20ms 间隔连续下发各电机的 `set` 命令，实现同步启动；
每个命令都带 `-t` 超时，到时自动停转。也可用 `--num-motors` 手动指定数量。

参考（四旋翼，怠速斜坡到 300 r/min 上限）：

```
静止时 PWM: 1000 1000 1000 1000
怠速起步:   约 1003（刚转）
加速中:     PWM 逐渐升高
最高速:     约 1025（约 300 r/min）
超时后:     1000 1000 1000 1000   ← -t 超时自动停转
```

安全约束：必须显式带 `--i-am-sure`；飞控已解锁时拒绝执行；输出值默认上限
`MPC_THR_MAX=0.025`（对应室内最高 **300 r/min**）；命令带 `-t` 超时自动停转；
Ctrl+C 立即发停止指令。有 DShot 遥测时若实际转速超过 300 r/min 会自动下调输出。

**关于转速**：`actuator_test` 只接受 `-1~1` 的输出值，没有直接设定 r/min 的接口；
室内用油门硬顶 + 电调遥测回收，把最高转速限制在 300 r/min。

## 用针脚串口跑后续例程

所有飞控例程（04~09）的 `run.sh` 默认使用 40PIN UART2 针脚串口 `/dev/ttyS2:57600`：

```bash
bash /app/zettatree_demo/06_autonomous_cruise/run.sh arm:=true
```

换串口（例如飞控 USB 口）：

```bash
bash /app/zettatree_demo/06_autonomous_cruise/run.sh fcu_url:=/dev/ttyACM0:115200 arm:=true
```

> **同一时刻只运行一个飞控例程**：一个串口只能被一路 MAVROS 占用。
