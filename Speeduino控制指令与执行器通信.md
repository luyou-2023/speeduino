# Speeduino 控制指令与执行器通信

Speeduino 作为开源 **ECU（Engine Control Unit，发动机控制单元）**，通过 **PWM、GPIO、ADC** 等方式与发动机执行器进行通信和控制。

---

## 1. Speeduino 硬件通信接口
Speeduino 基于 **Arduino Mega 2560**，主要接口包括：

- **PWM 输出**：喷油嘴、急速控制阀、VVT
- **GPIO 数字 I/O**：点火信号、线控繁用电器
- **ADC 模拟输入**：传感器数据采集
- **I2C/SPI**：扩展模块
- **UART 串口**：ECU 数据通信

---

## 2. Speeduino 控制指令与执行器通信方式

### 2.1 喷油嘴控制
**通信方式**：PWM + MOSFET

```cpp
#define INJECTOR_PIN 9  
int pulseWidth = 2500;  

void setup() {
    pinMode(INJECTOR_PIN, OUTPUT);
}

void loop() {
    digitalWrite(INJECTOR_PIN, HIGH);  
    delayMicroseconds(pulseWidth);      
    digitalWrite(INJECTOR_PIN, LOW);   
}
```

---

### 2.2 点火控制
**通信方式**：GPIO / PWM

```cpp
#define IGNITION_PIN 10  
int dwellTime = 2500;    

void setup() {
    pinMode(IGNITION_PIN, OUTPUT);
}

void loop() {
    digitalWrite(IGNITION_PIN, HIGH);  
    delayMicroseconds(dwellTime);
    digitalWrite(IGNITION_PIN, LOW);   
}
```

---

### 2.3 急速控制阀
**通信方式**：PWM

```cpp
#define IAC_PIN 11  
int idlePWM = 128;  

void setup() {
    pinMode(IAC_PIN, OUTPUT);
}

void loop() {
    analogWrite(IAC_PIN, idlePWM);
}
```

---

## 3. Speeduino 与外部通信
Speeduino 通过 **UART 串口** 与外部设备通信：

```cpp
#define SERIAL_BAUDRATE 115200

void setup() {
    Serial.begin(SERIAL_BAUDRATE);
}

void loop() {
    int rpm = 3000;
    Serial.print("RPM:");
    Serial.println(rpm);
    delay(1000);
}
```

---

## 4. Speeduino 执行器控制框架

| **执行器**           | **通信方式**     | **控制逻辑** |
|----------------------|----------------|-------------|
| 喷油嘴 | PWM + MOSFET    | 计算喷油脉出宽度 |
| 点火系统 | GPIO / PWM      | 计算点火推进角 |
| 急速阀 | PWM             | 调节空气流量 |
| 电子节气门 | PWM + 反馈      | 控制节气门开度 |
| VVT | PWM             | 调节气门正时 |
| 繁用电器 | GPIO            | 控制线控繁用电器 |

---

## 5. 总结
- **喷油控制** 通过 PWM + MOSFET 实现
- **点火控制** 通过 GPIO / PWM 计算点火推进角
- **急速阀控制** 通过 PWM 调节空气流量
- **Speeduino 通过 UART 串口** 与外部设备通信

Speeduino 提供了一套完整的 **开源 ECU 解决方案**，可以灵活适配不同发动机系统，支持自定义调校！
