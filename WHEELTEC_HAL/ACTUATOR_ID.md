# WHEELTEC B585 执行器辨识固件

本目录中的实验固件对应仓库根目录的 `actuator_identification_experiment.md`。它使用编译期开关隔离正常平衡控制器，在 500 Hz SysTick 中直接采样编码器和 PWM 寄存器，并通过 USART1/ST-Link VCP 输出带 CRC16 的二进制记录。

## 当前硬件结论

`ref/` 是指向 `/share/平衡车/` 的资料链接。已核对以下资料：

- `ref/.../4.原理图/C10A原理图.pdf`
- `ref/.../4.原理图/C10A主控资源分配详情表.pdf`
- `ref/.../7.芯片资料Datasheet/AT8236电机驱动芯片手册.pdf`

已确认的连接如下：

| 功能 | 外设/引脚 | 代码位置 |
|---|---|---|
| 左电机 H 桥输入 1/2 | TIM3 CH1/CH2，PA6/PA7，接 U6 AT8236 IN1/IN2，AO1/AO2 | `HAREWARE/Inc/motor.h`、`Core/Src/tim.c`、C10A 原理图 |
| 右电机 H 桥输入 1/2 | TIM3 CH3/CH4，PB0/PB1，接 U5 AT8236 IN1/IN2，BO1/BO2 | 同上 |
| 左编码器 | TIM4 CH1/CH2，PB6/PB7 | `Core/Src/tim.c` |
| 右编码器 | TIM8 CH1/CH2，PC6/PC7 | `Core/Src/tim.c` |
| 电池电压 | ADC2 IN11，PC1 | `Core/Src/adc.c` |
| 物理急停 | PC13 | `HAREWARE/Inc/key.h` |
| 数据/命令串口 | USART1，PA9/PA10，460800 baud | `Core/Src/usart.c` |

AT8236 手册的 H 桥控制逻辑表给出：

| IN1 | IN2 | OUT1 | OUT2 | 功能 |
|---:|---:|---|---|---|
| 0 | 0 | Z | Z | 滑行，休眠 |
| 1 | 0 | H | L | 正向 |
| 0 | 1 | L | H | 反向 |
| 1 | 1 | L | L | 刹车 |

因此 `MiniBalance/Inc/actuator_id_config.h` 中当前配置为：

- `COAST`: 两个输入均为 0，对应 TIM3 CCR 写 0；
- `BRAKE`: 两个输入均为 1，对应 TIM3 CCR 写 7200；
- `ACT_ID_BRIDGE_TRUTH_TABLE_CONFIRMED=1`，允许 `IDCFG/IDSTART`。

注意：AT8236 手册还说明 `IN1=IN2=0` 维持约 1 ms 后进入低功耗休眠，H 桥禁止、电荷泵停止；从休眠恢复到正常工作也约 1 ms。因此阶跃实验中如果零平台使用 `COAST`，从 0 到 DRIVE 的响应会包含休眠唤醒时间。若要专门辨识“驱动保持激活时的 0 PWM 阶跃”，应在硬件安全验证后把 `ACT_ID_ZERO_PWM_BRIDGE_MODE` 改为 `2`，用 `BRAKE` 做零平台，并在元数据中记录该选择。

原理图显示每颗 AT8236 的 `ISEN` 通过 `R250` 接地，`VREF` 接 3V3；未看到 `ISEN` 或 `nFAULT` 接到 STM32 ADC/GPIO。因此本版本仍不能提供 MCU 侧电机电流、过流事件或电机温度遥测。过流、短路、欠压、过温由 AT8236 芯片内部保护；固件侧实现物理急停、欠压、超速、通信超时和缓冲区溢出检测。采集元数据会明确写入 `current_measurement: unavailable`。

## 烧录前配置

1. 保持车轮离地、PWM 输出为零，分别进入 COAST 和 BRAKE，手拨轮验证 COAST 明显更容易转动且滑行更久。
2. 先用 `ACT_ID_PWM_LIMIT=1380`。只有机械、急停、超速和桥状态均验证后才逐步上调，且不得超过 6900。
3. 若做延迟实验，先决定零平台语义：`ACT_ID_ZERO_PWM_BRIDGE_MODE=1` 表示 COAST，`2` 表示 BRAKE；遥测会记录实际选择，绝不把零 PWM 笼统标成 DRIVE。

不要再使用正常固件中的 `Set_Pwm(0, 0)` 表示停止：在 AT8236 上，`Set_Pwm(0,0)` 会令 `IN1=IN2=1`，实际是 `BRAKE`；历史 `Flag_Stop` 路径四路 CCR=0，实际是 `COAST/sleep`。

## Keil 工程接入

仓库当前的 `.gitignore` 排除了 `WHEELTEC_HAL/MDK-ARM/`、`MiniBalance.ioc`、`Drivers/`、`HAREWARE/` 和 `SYSTEM/`。当前工作树虽有部分依赖源码，但没有 `.uvprojx`、STM32 启动文件或链接 scatter 文件，因此无法从本仓库独立产出固件。

使用厂商原始 Keil 工程时：

1. 恢复与本板 `STM32F103RCT6` 匹配的 `MDK-ARM` 工程、启动文件以及现有 `Drivers/HAREWARE/SYSTEM` 目录。
2. 保持原工程已有 include path，并确认包含 `Core/Inc`、`MiniBalance/Inc`、`HAREWARE/Inc` 和 `SYSTEM`。
3. 在 `MiniBalance` source group 加入 `MiniBalance/actuator_id.c`；头文件会由 include path 自动找到。
4. 确认工程仍编译修改后的 `Core/Src/main.c`、`Core/Src/stm32f1xx_it.c` 和 `MiniBalance/control.c`。
5. 编译目标必须定义 `STM32F103xE`（F103RC 属于 high-density line），时钟为 72 MHz；不要启用 semihosting。
6. 编译后检查 RAM：64 条记录的环形缓冲约占 4.3 KiB。若链接器显示 RAM 紧张，可把 `ACT_ID_RING_CAPACITY` 降到 32，但更容易在主循环阻塞时丢样。
7. 烧录后先不发送 `IDCFG`，确认四路输出为 OFF、车轮不自行转动，再连接采集脚本。

恢复正常控制固件时，把 `ACTUATOR_ID_MODE` 设为 `0` 并重新完整编译；正常控制路径会恢复 DMP、OLED、RL/LQR 和增量 PI。辨识构建启动时会显式清空 PI 累加值，且实验期间不会调用 PI、RL、LQR 或运动目标更新。

## 状态机与命令

命令是发往 USART1 的 ASCII 行，遥测为同一串口上的二进制输出。上位机脚本已封装这些命令。

```text
IDCFG,<id>,<type>,<side>,<pwm_a>,<pwm_b>,<target_mrad_s>,
      <armed_ms>,<phase_a_ms>,<phase_b_ms>,<coast_ms>,
      <max_speed_mrad_s>,<undervoltage_mv>
IDSTART
IDHEART
IDCOAST
IDBRAKE
IDSTOP
```

`type` 为 1/2/3（coast/pwmhold/pwmstep），`side` 为 1/2（L/R）。运行期间每 400 ms 发送 `IDHEART`；超过 1500 ms 未收到命令会触发通信超时并进入 OFF/E_STOP。`pwmstep` 先保持 `pwm_a` 共 `phase_a_ms`，再保持 `pwm_b` 共 `phase_b_ms`。`coast` 用 `pwm_a` 加速到目标速度并稳定 150 ms，或到达 `phase_a_ms` 超时，然后切换 COAST。

协议固定帧为 73 字节：`A5 5A`、版本、类型、66 字节小端 payload、CRC16-CCITT。字段定义和解析代码位于 `actuator_id_capture.py`。`time_us` 来自 Cortex-M3 DWT 周期计数器的 64 位扩展，不由样本号推算；状态切换样本同时记录 `previous_state`、新 `state`、`event_code`、实际桥模式和 PWM 写入时间。

## 采集和检查

先用 dry-run 检查命令和文件名：

```bash
uv run python actuator_id_capture.py coast \
  --side L --direction pos --experiment-id 101 --pwm-a 690 \
  --target-speed 40 --dry-run
```

实际运行必须显式给出机械和桥状态确认：

```bash
uv run python actuator_id_capture.py coast \
  --side L --direction pos --experiment-id 101 --repeat 1 --inertia J0 \
  --pwm-a 690 --target-speed 40 --wheels-off-ground \
  --bridge-driver-part 'AT8236' \
  --coast-pin-state 'IN1=0,IN2=0,OUT1=Z,OUT2=Z' \
  --brake-pin-state 'IN1=1,IN2=1,OUT1=L,OUT2=L'
```

脚本生成 `realdata/actuator_id/<run>.csv` 和同名 JSON。采集后执行：

```bash
uv run python check_actuator_id.py realdata/actuator_id \
  --output realdata/actuator_id/quality_report.json
```

检查器验证时间戳/样本连续性、99% 周期抖动、PWM 请求值/实际值/寄存器映射、COAST 和 COMPLETE 事件、故障位、电池电压、编码器跳变和最大轮速。硬件验证前产生的数据不得作为辨识结果。
