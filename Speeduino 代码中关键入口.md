## 1. 初始化入口 (setup.ino)
### setup()
- 初始化硬件外设（ADC/PWM/通信接口）
- 调用 `initialiseAll()` 加载配置、初始化传感器和中断

#### 关键子函数：
- `initialiseSchedulers()`：初始化定时器（喷油/点火调度）
- `configureADC()`：配置ADC采样（MAP/TPS/温度等）
- `triggerSetup()`：配置曲轴/凸轮轴信号中断

---

## 2. 主循环入口 (speeduino.ino)
### loop()
#### 通信处理：
- `serialReceive()`：处理串口指令（TunerStudio通信）
- `can_Command()`：处理CAN总线数据（扩展传感器/执行器）

#### 传感器读取：
- `readMAP()`：进气压力（1kHz）
- `readTPS()`：节气门位置（15Hz/30Hz）
- `readCLT()/readIAT()`：水温/进气温度（4Hz）

#### 燃油控制：
- `getVE1()`：查燃油表（VE值）
- `PW()`：计算喷油脉宽
- `setFuelSchedule()`：设置喷油计划

#### 点火控制：
- `getAdvance1()`：查点火提前角表
- `calculateIgnitionAngles()`：计算点火角度
- `setIgnitionSchedule()`：设置点火计划

#### 保护逻辑：
- `checkRevLimit()`：超速保护
- `checkEngineProtect()`：引擎保护（爆震/过热）

---

## 3. 中断处理入口 (timers.ino)
### 曲轴信号中断：
- `triggerHandler()`：曲轴齿信号处理（核心！）
   - 计算转速 (`getRPM`)
   - 更新曲轴角度 (`getCrankAngle`)
   - 同步引擎相位 (`currentStatus.hasSync`)

### 定时器中断：
- `oneMSInterval()`：1ms定时任务（更新 `LOOP_TIMER` 标志位）

---

## 4. 关键配置文件
- `globals.h`：全局状态变量（`currentStatus`）
- `config.h`：硬件引脚定义（喷油器/点火线圈/传感器引脚）
- `speeduino.h`：核心数据结构（燃油表/点火表/校正表）

---

## 5. 核心算法模块
### 燃油算法 (`corrections.ino`)
- `correctionsFuel()`：燃油修正（温度/加速/EGO闭环）

### 点火算法 (`ignition.ino`)
- `correctionsIgn()`：点火修正（爆震/转速补偿）

### 怠速控制 (`idle.ino`)
- `idleControl()`：PID控制步进电机/PWM阀

---

## 6. 曲轴同步与解码 (`decoders.ino`)
### 解码器选择：
- `decodeTrigger()`：根据齿盘类型（如 60-2、36-1）解析曲轴信号

### 同步逻辑：
- `triggerSetEndTeeth()`：设置点火触发的结束齿（顺序点火模式）

---

## 代码阅读建议
1. **从 `setup()` 和 `loop()` 入手**，理解初始化流程和主循环任务优先级。
2. **跟踪曲轴中断**：`triggerHandler()` 是实时控制的核心，影响所有时序计算。
3. **重点模块：**
   - **喷油逻辑**：`PW()` → `setFuelSchedule()`
   - **点火逻辑**：`calculateIgnitionAngles()` → `setIgnitionSchedule()`
4. **调试辅助：**
   - `comms.ino` 中的串口日志函数（如 `sendValues()`）可观察实时状态。

通过以上入口点，你可以快速定位到四冲程控制、曲轴同步及传感器处理的核心代码！

