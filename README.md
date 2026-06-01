# NXP MCXN947 PFC Firmware

基于 FreeRTOS 的 PFC（Power Factor Correction）电源控制固件，运行于 NXP FRDM-MCXN947 开发板。

## 架构

```
main.c (入口点)
  ├─ 时钟/板级初始化
  ├─ 外设配置 (ADC, UART, CAN, PWM, DAC, WWDT)
  ├─ IPC 创建 (队列/信号量)
  ├─ 10 个 FreeRTOS 任务
  └─ vTaskStartScheduler()
```

**设计原则**: ISR→Queue→Task 模式，每个模块拥有自己的硬件配置、ISR 和任务函数。

## 任务列表

| 任务名 | 优先级 | 栈大小 | 模块 | 功能 |
|--------|--------|--------|------|------|
| sensor | 4 | 1024 | sensor.c | 1kHz ADC 采集 + RMS 计算 |
| command | 3 | 512 | command.c | UART 命令接收与分发 |
| CAN_RX | 3 | 1024 | can.c | CAN 总线接收 |
| CAN_TX | 3 | 1024 | can.c | CAN 总线发送 |
| key | 2 | 125 | key.c | ADC 模拟键盘扫描 |
| fan | 2 | 1024 | fan.c | 风扇速度控制 |
| display | 1 | 512 | lcd.c | LCD 128×64 显示刷新 |
| SelfCheck | 1 | 256 | power_state.c | 上电自检 |
| PFCControl | 1 | 256 | power_state.c | PFC 启停控制 |
| DCControl | 1 | 256 | power_state.c | DC-DC 启停控制 |

## 模块说明

| 文件 | 职责 |
|------|------|
| `source/main.c` | 入口点，时钟/板级/外设初始化，IPC 创建，任务创建 |
| `source/main.h` | 共享类型: `MessageType_e`, `key_message_t` |
| `source/sensor.c` | ADC0/ADC1 配置，传感器读取，滑动窗口 RMS 计算 |
| `source/sensor.h` | ADC 通道枚举，`rms_message_t`, `sensor_status_t` |
| `source/can.c` | CAN 配置，ISR 回调，TX/RX 任务 |
| `source/can.h` | `can_message_t`, IPC 句柄 extern |
| `source/command.c` | UART 配置，ISR，命令解析，24 个命令处理器 |
| `source/command.h` | `uart_message_t`, IPC 句柄 extern |
| `source/lcd.c` | 128×64 LCD 驱动，5 页 UI，字体渲染 |
| `source/lcd.h` | `LCD_FontTypeDef`，页面/值全局变量 |
| `source/key.c` | ADC 电阻分压键盘，5 键消抖状态机 |
| `source/key.h` | 按键消息队列 extern |
| `source/fan.c` | 风扇控制 (stub) |
| `source/fan.h` | 风扇任务声明 |
| `source/power_state.c` | 上电状态机: INIT→SELFCHECK→SOFTSTART→PFC_READY→RUNNING |
| `source/power_state.h` | `PowerState_t` 枚举，状态查询接口 |
| `source/pwm_config.c` | PWM 三相驱动配置，故障保护 |
| `source/pwm_config.h` | PWM 配置接口 |

## IPC 对象

| 对象 | 类型 | 用途 |
|------|------|------|
| `xCanRxQueue` | Queue (5×can_message_t) | CAN 帧：ISR → CAN_RX 任务 |
| `xUartRxQueue` | Queue (5×uart_message_t) | UART 字节：ISR → command 任务 |
| `xKeyMessageQueue` | Queue (10×key_message_t) | 按键事件：key 任务 → display 任务 |
| `xCanTxMutex` | Mutex | CAN TX 互斥 (预留) |

## RMS 算法

滑动窗口真 AC 有效值计算：

- **窗口大小**: 200 采样点 @ 1kHz = 200ms = 10 周期 (50Hz) / 12 周期 (60Hz)
- **算法**: O(1) 递推更新 `sqrt(E[x²] - E[x]²)`，自动去除 DC 偏置
- **通道**: Vrms (输入电压) / Irms (输入电流) 独立状态
- **DC 通道**: Vout / Iout 使用指数移动平均

## UART 命令

波特率 115200，8N1，格式 `command:param`。

| 命令 | 参数 | 功能 |
|------|------|------|
| `ss_rly_enable` | - | 闭合软启继电器 |
| `ss_rly_disable` | - | 断开软启继电器 |
| `pfc1_enable` | - | 使能 PFC1 |
| `pfc1_disable` | - | 禁能 PFC1 |
| `pfc2_enable` | - | 使能 PFC2 |
| `pfc2_disable` | - | 禁能 PFC2 |
| `dcdc1_enable` | - | 使能 DCDC1 |
| `dcdc1_disable` | - | 禁能 DCDC1 |
| `dcdc2_enable` | - | 使能 DCDC2 |
| `dcdc2_disable` | - | 禁能 DCDC2 |
| `pfc1_flt_detect` | - | 读取 PFC1 故障引脚 |
| `pfc2_flt_detect` | - | 读取 PFC2 故障引脚 |
| `sys_cfg0_detect` | - | 读取系统配置 0 |
| `sys_cfg1_detect` | - | 读取系统配置 1 |
| `sys_cfg2_detect` | - | 读取系统配置 2 |
| `fan1_fb_detect` | - | 读取风扇 1 反馈 |
| `fan2_fb_detect` | - | 读取风扇 2 反馈 |
| `fan3_fb_detect` | - | 读取风扇 3 反馈 |
| `get_adc` | - | 打印全部 16 路 ADC 值 |
| `set_dac0` | 0-4095 | 设置 DAC0 输出 |
| `set_dac1` | 0-4095 | 设置 DAC1 输出 |
| `set_pwm1` | 0-100 | 设置 PWM1 占空比 |
| `set_pwm2` | 0-100 | 设置 PWM2 占空比 |
| `set_pwm3` | 0-100 | 设置 PWM3 占空比 |

## 硬件

- **开发板**: NXP FRDM-MCXN947
- **MCU**: MCXN947VDF (Cortex-M33, 浮点硬核, 非 TrustZone)
- **调试器**: 板载 MCU-Link (J17 USB-C)

## 编译

- **工具链**: ARM GNU Toolchain 14.2 (`arm-none-eabi-gcc`)
- **SDK**: NXP MCUXpresso SDK v2.16.100
- **CPU 定义**: `CPU_MCXN947VDF_cm33_core0`
- **架构标志**: `-mcpu=cortex-m33 -mthumb -mfloat-abi=hard -mfpu=fpv5-sp-d16`

```sh
# 单文件编译 (示例)
arm-none-eabi-gcc -mcpu=cortex-m33 -mthumb -mfloat-abi=hard -mfpu=fpv5-sp-d16 \
  -O0 -g -Wall -c -std=c11 -DCPU_MCXN947VDF_cm33_core0 \
  -Isource -Iboard -Idrivers -Iutilities -Idevice -ICMSIS -Icomponent -Istartup \
  -Ifreertos/freertos-kernel/include \
  -Ifreertos/freertos-kernel/portable/GCC/ARM_CM33_NTZ/non_secure \
  source/main.c -o main.o
```

## 上电时序

```
INIT → SELFCHECK (检查 Vin 90-265V)
  → SOFTSTART (20ms 延时 → 闭合继电器)
  → PFC_READY (20ms 延时 → 使能 PFC1/PFC2)
  → RUNNING (10ms 延时 → 使能 DCDC1/DCDC2)
```
