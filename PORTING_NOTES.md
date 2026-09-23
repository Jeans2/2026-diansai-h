# 天猛星扩展板移植说明

本工程按 2026-07-24 版扩展板原理图配置，目标器件为 MSPM0G3507。

## 引脚分配

| 功能 | MSPM0 引脚 |
|---|---|
| 八路灰度 OUT | PA14 / ADC0 channel 12 |
| 八路灰度 AD0 / AD1 / AD2 | PB18 / PB19 / PB10 |
| 电机 A：AT8236 AIN1(PWM) / AIN2 | PB4 / PA23 |
| 电机 B：AT8236 BIN1(PWM) / BIN2 | PB1 / PA18 |
| 电机 A 编码器 B / A | PA17 / PA24 |
| 电机 B 编码器 B / A | PA15 / PA16 |
| OLED SDA / SCL | PA28 / PA31 |
| 启停 / 加速 / 减速按键 | PA25 / PA26 / PA12 |
| 调试串口 TX / RX | PB6 / PB7，115200 baud |
| HC-05 TX / RX / STATE | PB15 / PB16 / PA7，9600 baud |

MG310 六线端子的编码器供电使用扩展板的 3V3，不接 5V。两个电机编码器
均采用 A/B 双边沿四倍频计数。

## 驱动接口

- `Grayscale_ReadAll(values)` 一次读取八路 12 位 ADC 原始值。
- `Encoder_GetMotorA()` / `Encoder_GetMotorB()` 返回累计正交计数。
- `Encoder_Reset()` 清零两个累计计数。
- `Motor_A_Set(speed)` / `Motor_B_Set(speed)` 使用 `-1000..1000` 的有符号速度。
- `Motor_StopAll()` 让两路输出滑行停止。
- `Motor_BrakeAll()` 让两路输出短路制动。

为了兼容原移植代码，电机驱动文件仍名为 `bsp_tb6612.c/.h`，但底层已经改为
新版扩展板上的 AT8236S。

## 首次上电

当前 `empty.c` 是安全联调程序，上电后不转电机。OLED 显示首尾灰度值和两个
编码器计数；PB6 调试串口持续输出全部八路灰度值、编码器计数和 HC-05 状态。

烧录前必须用万用表确认灰度模块 OUT 对 GND 的最高电压不超过 3.3V。原理图上
PA14 与 OUT 之间没有电阻分压，不能把接近 5V 的模拟信号直接送入 MSPM0 ADC。

编译产物为 `Debug/empty.hex`，格式为 Intel HEX。
