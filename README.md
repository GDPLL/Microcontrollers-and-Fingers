# ESP32 第六指假肢控制系统

基于双通道肌电信号（sEMG）的智能假肢手指控制系统，使用 ESP32 微控制器和 PlatformIO 开发环境。

## 功能特性

- **双通道 EMG 信号处理**
  - 12位 ADC 高精度采样
  - 多级数字滤波（高通、陷波、移动平均）
  - 包络检测和归一化
  
- **多种控制模式**
  - 直接映射模式
  - 阈值开关模式
  - 比例差分模式
  - 机器学习手势识别（预留接口）
  
- **智能校准系统**
  - 自动记录休息状态
  - 最大收缩力度校准
  - EEPROM 持久化存储
  
- **平滑伺服控制**
  - 指数平滑算法
  - 可调平滑系数
  - 紧急停止功能

## 硬件要求

| 组件 | 规格 | 数量 |
|------|------|------|
| 微控制器 | ESP32 DevKit | 1 |
| EMG 传感器 | 双通道肌电模块 | 1 |
| 舵机 | MG996R / SG90 | 1 |
| 干电极 | 肌电采集电极 | 3 |
| LED | 3mm 指示灯 | 3 |
| 按钮 | 轻触开关 | 2 |
| 电源 | 5V/2A | 1 |

## 引脚定义

```cpp
EMG_CH1_PIN     34  // 屈肌信号输入
EMG_CH2_PIN     35  // 伸肌信号输入
SERVO_PIN       26  // 舵机 PWM 输出
LED_STATUS       2  // 状态指示灯
LED_CH1_ACTIVE  25  // 通道1激活指示
LED_CH2_ACTIVE  33  // 通道2激活指示
BUTTON_CAL       0  // 校准按钮（BOOT）
BUTTON_MODE      4  // 模式切换按钮
```

## 快速开始

### 1. 安装 PlatformIO

```bash
# 使用 VS Code 安装 PlatformIO 插件
# 或命令行安装
pip install platformio
```

### 2. 克隆项目

```bash
cd esp32_sixth_finger
```

### 3. 编译上传

```bash
# 编译
pio run

# 上传到 ESP32
pio run --target upload

# 打开串口监视器
pio device monitor
```

## 使用说明

### 首次使用校准

1. 上电后系统会自动尝试加载校准数据
2. 如果没有校准数据，按 **BOOT 按钮**开始校准
3. 按照串口提示依次完成：
   - 休息状态（放松肌肉）
   - 屈肌最大收缩
   - 伸肌最大收缩

### 控制模式切换

- 按 **模式按钮** 或串口发送 `m` 切换控制模式
- 可用模式：
  - **DIRECT**: 直接映射，CH1 弯曲 / CH2 伸展
  - **THRESHOLD**: 阈值控制，超过阈值触发动作
  - **PROPORTIONAL**: 比例差分，根据两通道差值控制
  - **GESTURE**: 机器学习手势识别（开发中）

### 串口命令

| 命令 | 功能 |
|------|------|
| `h` | 显示帮助 |
| `c` | 开始校准 |
| `m` | 切换模式 |
| `r` | 回到休息位置 |
| `s` | 紧急停止 |
| `0` | 最小角度（弯曲）|
| `5` | 中间角度（休息）|
| `9` | 最大角度（伸展）|
| `d` | 调试信息 |

## 信号处理流程

```
Raw EMG (ADC)
    ↓
DC Offset Removal (High-pass, 10Hz)
    ↓
Power Line Filter (Notch, 50Hz)
    ↓
Smoothing (Moving Average)
    ↓
Envelope Detection (Rectify + Low-pass)
    ↓
Normalization (0-1 based on calibration)
    ↓
Control Logic
    ↓
Servo Output
```

## 项目结构

```
esp32_sixth_finger/
├── platformio.ini          # PlatformIO 配置
├── README.md               # 项目说明
└── src/
    ├── main.cpp            # 主程序入口
    ├── config.h            # 配置文件
    ├── emg_processor.h/.cpp    # EMG 信号处理
    ├── servo_controller.h/.cpp # 舵机控制
    └── calibration.h/.cpp      # 校准管理
```

## 参数调整

编辑 `src/config.h` 可调整以下参数：

```cpp
// 采样率
#define SAMPLING_RATE   1000    // Hz

// 滤波器参数
#define FILTER_LOWCUT   10      // 高通截止频率
#define FILTER_HIGHCUT  450     // 低通截止频率

// 舵机参数
#define SERVO_SMOOTHING_FACTOR  0.15f
#define SERVO_MIN_ANGLE 0
#define SERVO_MAX_ANGLE 180

// 阈值参数
#define THRESHOLD_CH1_ON    300
#define PROP_GAIN_CH1       0.5f
```

## 调试

启用串口绘图器查看实时数据：

```cpp
// 在 config.h 中设置
#define OUTPUT_RAW      true
#define OUTPUT_FILTERED true
#define OUTPUT_ANGLE    true
```

数据格式：`R1:raw1 R2:raw2 E1:env1 E2:env2 TA:target CA:current`

## 许可证

MIT License

## 作者

Qoder - 2025
