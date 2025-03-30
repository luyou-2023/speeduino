Speeduino 源码及开发指南
========================

本指南面向 Speeduino 固件的二次开发者，详细梳理了项目核心源码文件和控制流程。文档主要内容包括：

- **核心文件说明**
- **系统初始化与主循环**
- **传感器数据采集与预处理**
- **控制算法计算（点火与喷油）**
- **输出控制与中断调度**
- **安全保护与故障处理**

## 核心文件说明

以下是 Speeduino 项目中关键核心文件及其作用说明：

### speeduino.ino
作为固件的主入口文件，包含 Arduino 标准的 `setup()` 与 `loop()` 函数，负责硬件初始化、加载配置数据、配置中断以及周期性任务调度。

### speeduino.h
定义全局变量、常量以及核心函数的声明，为整个项目提供统一的接口。

### sensors.cpp / sensors.h
负责所有传感器数据的读取、转换及预处理工作。函数如 `readSensors()` 用于依次采集温度、压力、转速等数据，并进行滤波处理。

### ignition.cpp / ignition.h
实现点火控制逻辑。通过查表、插值计算最佳火花提前角，并生成精确的点火触发信号。关键函数包括 `calculateIgnitionAngle()` 与 `triggerIgnition()`。

### injection.cpp / injection.h
负责燃油喷射控制。根据发动机状态和传感器数据计算喷油脉宽，并通过 PWM 信号控制喷油器。关键函数包括 `calculateInjectionPulse()` 与 `triggerInjection()`。

### interrupts.cpp / interrupts.h
配置并实现实时定时中断，确保系统能够在规定时间内完成传感器采集、控制计算与输出信号生成。中断服务程序（ISR）调用 `readSensors()`、`calculateControl()`、`triggerIgnition()` 和 `triggerInjection()` 等函数。

### safety.cpp / safety.h
实现系统的安全保护和故障处理逻辑。在每个周期内检测关键参数（如发动机温度、进气压力等），并在异常时触发安全模式，保护发动机和系统。

---

## 详细源码流程梳理

### 1. 系统初始化与主循环
**文件：speeduino.ino**

```cpp
#include "speeduino.h"

void setup() {
  initHardware();
  loadCalibrationData();
  initInterrupts();
  initCommunication();
}

void loop() {
  if (sensorDataReady()) {
    updateSensorData();
    updateEngineState();
    calculateControl();
    outputControlSignals();
  }
  processCommunication();
}
```

**说明：**
- `initHardware()` 封装了各硬件模块的初始化工作。
- `initInterrupts()` 配置定时器和中断。
- 主循环负责非实时任务，实时任务由中断服务程序完成。

### 2. 传感器数据采集与预处理
**文件：sensors.cpp**

```cpp
#include "sensors.h"

void readSensors() {
  int rawTemp = analogRead(PIN_TEMP);
  engineTemp = convertTemperature(rawTemp);

  int rawPressure = analogRead(PIN_PRESSURE);
  intakePressure = convertPressure(rawPressure);

  engineRPM = readRPM();
  
  engineTemp = filterValue(engineTemp);
  intakePressure = filterValue(intakePressure);
}
```

### 3. 控制算法计算

#### 3.1 点火控制
**文件：ignition.cpp**

```cpp
#include "ignition.h"

float calculateIgnitionAngle() {
  float rpm = engineRPM;
  float load = getEngineLoad();

  float baseAngle = lookupIgnitionTable(rpm, load);
  float correction = calculateIgnitionCorrection();

  return baseAngle + correction;
}

void triggerIgnition() {
  float ignitionAngle = calculateIgnitionAngle();
  scheduleIgnitionPulse(ignitionAngle);
}
```

#### 3.2 喷油控制
**文件：injection.cpp**

```cpp
#include "injection.h"

unsigned int calculateInjectionPulse() {
  unsigned int basePulse = lookupInjectionTable(engineRPM, intakePressure, engineTemp);
  unsigned int compensation = calculateInjectionCompensation();

  return basePulse + compensation;
}

void triggerInjection() {
  unsigned int pulseWidth = calculateInjectionPulse();
  setInjectionPWM(pulseWidth);
}
```

### 4. 输出控制与中断调度
**文件：interrupts.cpp**

```cpp
#include "interrupts.h"

ISR(TIMER1_COMPA_vect) {
  readSensors();
  calculateControl();
  triggerIgnition();
  triggerInjection();
  checkSafety();
}
```

### 5. 安全保护与故障处理
**文件：safety.cpp**

```cpp
#include "safety.h"

void checkSafety() {
  if (engineTemp > MAX_ENGINE_TEMP) {
    enterSafeMode();
    logFaultCode(FAULT_OVERHEAT);
  }

  if (intakePressure < MIN_PRESSURE || intakePressure > MAX_PRESSURE) {
    logFaultCode(FAULT_PRESSURE);
  }
}

void enterSafeMode() {
  disableInjection();
  disableIgnition();
}
```

---

## 总结
本指南详细梳理了 Speeduino 固件中核心文件的实际实现流程，涵盖了从初始化、传感器采集、控制算法计算到输出控制和安全保护的全过程。二次开发时，建议深入阅读各模块源码，并结合调试工具验证各流程的运行状态，以确保在扩展新功能时不会破坏现有实时性和安全性要求。

如果需要进一步分析其他模块或更多细节，请参考最新的 **Speeduino GitHub 仓库** 中的源码，并结合官方文档和社区资源进行探讨。

