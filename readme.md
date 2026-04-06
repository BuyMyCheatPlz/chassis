# Chassis 项目

## 电机引脚映射表

四路电机的 PWM、方向控制（DIR）和编码器引脚对照：

| 电机    | PWM 引脚         | DIR_A | DIR_B | 编码器 CH1 | 编码器 CH2 |

| Motor1 | PA15 (TIM2_CH1) | PC0 | PC4 | PA6 (TIM3_CH1) | PA7 (TIM3_CH2) |
| Motor2 | PB3 (TIM2_CH2) | PC1 | PC5 | PD12 (TIM4_CH1) | PD13 (TIM4_CH2) |
| Motor3 | PA2 (TIM2_CH3) | PC2 | PA4 | PA0 (TIM5_CH1) | PA1 (TIM5_CH2) |
| Motor4 | PA3 (TIM2_CH4) | PC3 | PA5 | PC6 (TIM8_CH1) | PC7 (TIM8_CH2) |

### 说明

- **PWM 引脚**：控制电机速度（占空比 0-100%）
- **DIR 引脚**：方向控制，DIR_A 和 DIR_B 互补控制电机正反向
- **编码器引脚**：反馈电机转速，CH1/CH2 是 A/B 两相信号

### 源代码位置

- 电机配置：`Core/Src/motor.c` (Motor_GetDefaultConfig)
- 定时器配置：`Core/Src/tim.c`
- GPIO 配置：`Core/Src/gpio.c`
