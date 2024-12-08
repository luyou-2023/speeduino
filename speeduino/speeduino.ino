/*
Speeduino - Simple engine management for the Arduino Mega 2560 platform
Copyright (C) Josh Stewart

This program is free software; you can redistribute it and/or
modify it under the terms of the GNU General Public License
as published by the Free Software Foundation; either version 2
of the License, or (at your option) any later version.

This program is distributed in the hope that it will be useful,la
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with this program; if not, write to the Free Software
Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301, USA.
*/
/** @file
 * Speeduino initialisation and main loop.
 */
#include <stdint.h> //developer.mbed.org/handbook/C-Data-Types
//************************************************
#include "globals.h"
#include "speeduino.h"
#include "scheduler.h"
#include "comms.h"
#include "comms_legacy.h"
#include "comms_secondary.h"
#include "maths.h"
#include "corrections.h"
#include "timers.h"
#include "decoders.h"
#include "idle.h"
#include "auxiliaries.h"
#include "sensors.h"
#include "storage.h"
#include "crankMaths.h"
#include "init.h"
#include "utilities.h"
#include "engineProtection.h"
#include "scheduledIO.h"
#include "secondaryTables.h"
#include "comms_CAN.h"
#include "SD_logger.h"
#include "schedule_calcs.h"
#include "auxiliaries.h"
#include RTC_LIB_H //Defined in each boards .h file
#include BOARD_H //Note that this is not a real file, it is defined in globals.h. 


uint16_t req_fuel_uS = 0; /**< The required fuel variable (As calculated by TunerStudio) in uS */
uint16_t inj_opentime_uS = 0;

uint8_t ignitionChannelsOn; /**< The current state of the ignition system (on or off) */
uint8_t ignitionChannelsPending = 0; /**< Any ignition channels that are pending injections before they are resumed */
uint8_t fuelChannelsOn; /**< The current state of the fuel system (on or off) */
uint32_t rollingCutLastRev = 0; /**< Tracks whether we're on the same or a different rev for the rolling cut */

uint16_t staged_req_fuel_mult_pri = 0;
uint16_t staged_req_fuel_mult_sec = 0;   
#ifndef UNIT_TEST // Scope guard for unit testing

/**
传感器、MC33810 和 Arduino 之间的关系可以用以下流程来描述：

传感器：

收集数据：传感器（如温度传感器、压力传感器、氧气传感器等）监测发动机的各种参数，如温度、压力、转速等。
发送数据：传感器通过 CAN（Controller Area Network）总线将数据发送给 Arduino。
CAN 总线：

数据传输：CAN 总线用于传输传感器数据到 Arduino。它是一个高效、可靠的通信协议，适用于车辆和工业控制系统。
Arduino Mega 2560：

处理数据：Arduino 接收传感器数据并运行控制算法。它根据传感器数据和预设的控制策略来计算所需的喷油量和点火时间。
生成控制命令：Arduino 根据处理后的数据生成控制命令，这些命令指定 MC33810 的操作，如喷油器的开关状态和点火时间。
MC33810：

接收命令：MC33810 从 Arduino 接收控制命令。这些命令通过 SPI 总线传输到 MC33810。
驱动执行器：MC33810 控制发动机的喷油器和点火线圈，根据命令驱动它们以实现精准的燃油喷射和点火。
发动机：

执行操作：喷油器和点火线圈按照 MC33810 的指令进行操作，喷油器将燃油喷入发动机燃烧室，点火线圈产生点火信号点燃燃料混合物，驱动发动机运行。
总结
传感器 -> CAN -> Arduino -> MC33810 -> 发动机 是一个典型的控制流程，其中传感器收集数据，通过 CAN 总线发送给 Arduino，Arduino 处理数据并生成控制命令，MC33810 根据这些命令驱动发动机的执行器。这样，你的系统能够实时调整发动机的工作状态，实现高效和精准的控制。
**/
void setup(void)
{
  currentStatus.initialisationComplete = false; //Tracks whether the initialiseAll() function has run completely
  initialiseAll();
}

/**
核心代码逻辑
Speeduino的核心代码主要分为几个关键部分：

初始化（Initialization）:
初始化微控制器的外设，比如ADC（模数转换器）、PWM（脉冲宽度调制）等。
设置引脚模式（输入/输出）。
配置通信接口（如UART/串口）。
传感器读取（Sensor Reading）:
读取来自各种传感器的数据，例如进气温度传感器（IAT）、节气门位置传感器（TPS）、爆震传感器等。
这些数据会被用于计算发动机的工作状态。
控制算法（Control Algorithms）:
点火定时计算：基于转速、负荷和其他传感器数据来确定最佳的点火提前角。
燃油喷射量计算：根据空气流量、转速等因素来计算所需的燃油喷射量。
这些算法通常包括基于表格的查找表（lookup tables），这些表可以通过软件界面进行调整以适应特定的发动机配置。
执行器控制（Actuator Control）:
控制点火线圈触发点火。
控制喷油嘴打开和关闭的时间长度。
可能还包括对怠速控制阀、冷却风扇等其他部件的控制。
故障检测与记录（Fault Detection and Logging）:
监测传感器数据的有效性，并在发现异常时记录故障码。
记录关键参数以便于后续分析。
用户界面（User Interface）:
提供一个简单的界面或连接到外部设备（如笔记本电脑）来进行设置和监控。
可能会通过LCD屏幕或LED指示灯提供基本的状态信息。
如何与硬件集成
Speeduino与硬件的集成涉及以下几个步骤：

硬件选择:
选择合适的Arduino兼容板，如Uno、Mega等。
选择必要的传感器和执行器，确保它们与所选Arduino板兼容。
电路设计:
设计电路图，确保所有组件正确连接。
包括电源管理、信号调理（如放大器、滤波器）、保护电路（如二极管、保险丝）等。
安装与布线:
安装所有的电子元件到一块PCB（印刷电路板）上。
使用适当的电线将传感器和执行器连接到Arduino板。
软件配置:
使用Speeduino提供的工具（如Speeduino Configurator）来配置系统。
设置传感器类型、校准值、控制策略等。
测试与调试:
在实际应用前进行彻底的测试，确保所有组件正常工作。
调整控制参数以优化性能。
请注意，对于实际操作来说，需要具备一定的电子和机械工程知识。此外，由于发动机管理系统直接关系到车辆的安全性和性能，因此在安装和使用时务必谨慎，并考虑安全措施。
**/

/***
# Speeduino 硬件与软件架构

## 硬件架构

### 微控制器
- **型号**: ATmega328P
- **功能**: 中央处理单元，执行控制逻辑

### 电源模块
- **5V稳压器**: 提供电源给微控制器和其他电路

### 输入接口
- **模拟信号输入**
  - **MAP/MAF传感器**: A0
  - **水温传感器**: A1
- **数字信号输入**
  - **转速传感器**: D2 (中断)

### 输出接口
- **驱动电路**
  - **点火线圈**: D9
  - **喷油器**: D10

### 传感器
- **MAP/MAF传感器**
- **水温传感器**
- **转速传感器**

### 执行器
- **点火线圈**
- **喷油器**

## 软件架构

### Bootloader
- **功能**: 允许通过串口更新固件

### 主程序
- **输入处理**
  - **信号过滤**
  - **数据解析**
- **输出控制**
  - **点火定时**
  - **燃油喷射**
- **故障检测**
- **通信协议**
  - **串行通信**
  - **CAN总线**

### 用户界面
- **LCD显示**
- **按键输入**

## 引脚配置
| 功能       | 描述             | Arduino Pin |
|------------|------------------|-------------|
| 点火输出   | 控制点火线圈     | D9          |
| 喷油器输出 | 控制喷油器       | D10         |
| MAP/MAF    | 检测进气量       | A0          |
| 水温       | 检测发动机温度   | A1          |
| 转速       | 检测发动机转速   | D2 (中断)   |

---

文件中的主循环speeduino.ino连续运行，具有 2 个主要功能：确定发动机要求并根据这些要求设置点火/喷射计划。

主循环中执行的功能的高级描述如下：

检查串口缓冲区中是否有需要处理的请求
通过查看上次看到曲轴齿的时间来判断发动机是否转动
读取所有模拟传感器的值（TPS、IAT、CLT、MAP、O2、电池电压）。并非所有传感器都会在每个循环中读取，因为它们的变化频率不足以保证这一点
仅当引擎具有“同步”时才会发生以下功能：
检查 RPM 是否高于或低于启动阈值（发动机启动时会调整燃油和点火值）
运行所有校正功能（请参阅下面的 Corrections.ino 部分）。结果为脉冲宽度将调整的百分比（100% = 无调整，110% = 增加 10% 的燃料，90% = 减少 10% 的燃料）
从主燃料表中查找 VE
将VE转换为以uS为单位的脉冲宽度值
从点火表中查找所需的提前量
计算当前曲轴角度
根据当前发动机转速计算每个喷油器应打开的曲轴角度
根据所需停留时间和当前发动机转速计算停留角
通过从 TDC 角中减去提前角和闭合角来计算每个气缸的点火开始角
通过将上面计算的起始角度转换为未来的 uS 数，为每个喷油器设置一个“时间表”（例如，如果喷油器应在 45* ATDC 开始打开，曲轴角度当前为 10*BTDC，那么需要多长时间才能到达 55*）
对每个点火输出执行相同的时间表设置
**/



inline uint16_t applyFuelTrimToPW(trimTable3d *pTrimTable, int16_t fuelLoad, int16_t RPM, uint16_t currentPW)
{
    uint8_t pw1percent = 100U + get3DTableValue(pTrimTable, fuelLoad, RPM) - OFFSET_FUELTRIM;
    return percentage(pw1percent, currentPW);
}

/** Speeduino main loop.
 * 
 * Main loop chores (roughly in the order that they are performed):
 * - Check if serial comms or tooth logging are in progress (send or receive, prioritise communication)
 * - Record loop timing vars
 * - Check tooth time, update @ref statuses (currentStatus) variables
 * - Read sensors
 * - get VE for fuel calcs and spark advance for ignition
 * - Check crank/cam/tooth/timing sync (skip remaining ops if out-of-sync)
 * - execute doCrankSpeedCalcs()
 * 
 * single byte variable @ref LOOP_TIMER plays a big part here as:
 * - it contains expire-bits for interval based frequency driven events (e.g. 15Hz, 4Hz, 1Hz)
 * - Can be tested for certain frequency interval being expired by (eg) BIT_CHECK(LOOP_TIMER, BIT_TIMER_15HZ)
 * 
 */

 /*
 Speeduino 板- 这是 Speeduino ECU 的核心，包含所有驱动程序和 IO 电路。这可能是通用板之一（例如 v0.4）或特定型号汽车的 PNP 板
 Arduino - 这是 Speeduino 的大脑，包含处理器、内存和存储。它插入 Speeduino 板以便与车辆线路连接。通常这是一个 Arduino Mega 2560，但也支持各种基于 Teensy 和 stm32 的板。
 固件- 这是在 Arduino 开发板上运行并为其操作提供动力的系统软件。新固件会定期发布，以提供更新、性能改进和错误修复。

 v0.4 板是一个测试板，其开发目标是重现现有 v0.3 板的功能，但具有以下改进：

 成本更低（主要是由于尺寸减小，但也有一些组件的变化）
 与现成的外壳/外壳更兼容
 步进式 IAC 驱动器选项
 所有 IO 均有一个 40 针连接器（不包括 12v 电源）

 v0.4 板包含以下功能：

 4 个喷射通道
 4 个点火输出
 为 CLT、IAT、TPS 和 O2 提供全面保护的输入通道
 曲轴和凸轮输入端可选安装 VR 调节器
 MAP 传感器安装位置
 DRV8825 步进模块安装位置
 4 个中等电流备用输出（例如燃油泵、热风扇、增压控制、VVT 等）
 “原型”部分中有 5 个未填充/配置的可选低电流备用输出，包括转速表输出
 单个 40 针 IDC 连接器包含主板所需的所有针脚（12v 输入除外）

 请注意，不同版本的主板之间存在一些差异，但主 IDC40 连接器上的引脚排列保持不变。

 注意：喷油器针脚有 1/2 和 2/2 标记，这是为了更轻松、更清晰地为半顺序和批量模式布线喷油器。如果应用需要少于 4 个喷油器，只需使用针脚 1/2 或 2/2。如果应用需要 5 个或更多喷油器，建议同时使用 1/2 和 2/2（如果可用），以更均匀地分配来自喷油器线圈触发的电流。有关更多具体细节，请参阅喷油器接线。
 */

 /**
     speeduino.ino 是一个 Arduino 平台上的项目文件，其主要功能是实现一个基于开源 Speeduino 软件的汽车电子控制单元（ECU）。以下是该代码的功能总结：

     初始化设置：在 setup() 函数中进行了一些初始化设置，包括设置串行通信、初始化 LED 引脚、设置输入输出引脚等。

     基本循环：在 loop() 函数中执行主要的循环任务。该循环包括：

     读取传感器数据：通过调用 readSensors() 函数来读取各种传感器的数据，包括曲轴位置传感器、进气压力传感器、节气门位置传感器等。

     计算引擎参数：通过调用 calculate() 函数来计算引擎的参数，例如引擎转速、负荷、进气量等。

     控制执行器：通过调用 controlActuators() 函数来控制执行器，例如点火系统、燃油喷射系统等。

     更新状态：根据当前的状态更新 LED 灯状态，例如指示系统是否正在运行。

     传感器数据读取：包括读取曲轴位置传感器、进气压力传感器、节气门位置传感器等传感器数据，并将其保存在相应的变量中。

     参数计算：根据传感器数据计算出一些重要的引擎参数，例如引擎转速、负荷、进气量等。

     执行器控制：根据计算得到的引擎参数，控制执行器的工作，例如点火系统的触发、燃油喷射器的喷射时间等。

     故障检测与处理：包括检测各种传感器是否正常工作，以及根据检测结果采取相应的处理措施。

     串行通信：通过串行通信与外部设备进行数据交换，例如与调试器或监控器进行通信，以便实时监控引擎状态。

     总的来说，speeduino.ino 实现了一个简单的汽车电子控制系统，其主要功能包括读取传感器数据、计算引擎参数、控制执行器、处理故障等。

     燃油喷射控制
     点火正时控制
     故障诊断
     数据记录
     实时性能监控

 */
void loop(void)
{
      mainLoopCount++;
      LOOP_TIMER = TIMER_mask;

      //SERIAL Comms
      //Initially check that the last serial send values request is not still outstanding
      if (serialTransmitInProgress())
      {
        //// 串口通信处理
        serialTransmit();
      }

      //Check for any new or in-progress requests from serial.
      if (Serial.available()>0 || serialRecieveInProgress())
      {
        // 串口通信处理
        serialReceive();
      }
      
      //Check for any CAN comms requiring action 
      #if defined(secondarySerial_AVAILABLE)
        //if can or secondary serial interface is enabled then check for requests.
        if (configPage9.enable_secondarySerial == 1)  //secondary serial interface enabled
        {
          if ( ((mainLoopCount & 31) == 1) || (secondarySerial.available() > SERIAL_BUFFER_THRESHOLD) )
          {
            // CAN通信处理
            if (secondarySerial.available() > 0)  { secondserial_Command(); }
          } 
        }
      #endif
      #if defined (NATIVE_CAN_AVAILABLE)
        if (configPage9.enable_intcan == 1) // use internal can module
        {            
          //check local can module
          // if ( BIT_CHECK(LOOP_TIMER, BIT_TIMER_15HZ) or (CANbus0.available())
          while (CAN_read()) 
          {
            can_Command();
            readAuxCanBus();
            if (configPage2.canWBO > 0) { receiveCANwbo(); }
          }
        }   
      #endif
          
    if(currentLoopTime > micros_safe())
    {
      //Occurs when micros() has overflowed
      deferEEPROMWritesUntil = 0; //Required to ensure that EEPROM writes are not deferred indefinitely
    }

    currentLoopTime = micros_safe();
    uint32_t timeToLastTooth = (currentLoopTime - toothLastToothTime);
    if ( (timeToLastTooth < MAX_STALL_TIME) || (toothLastToothTime > currentLoopTime) ) //Check how long ago the last tooth was seen compared to now. If it was more than half a second ago then the engine is probably stopped. toothLastToothTime can be greater than currentLoopTime if a pulse occurs between getting the latest time and doing the comparison
    {
      currentStatus.longRPM = getRPM(); //Long RPM is included here
      currentStatus.RPM = currentStatus.longRPM;
      currentStatus.RPMdiv100 = div100(currentStatus.RPM);
      if(currentStatus.RPM > 0)
      {
        FUEL_PUMP_ON();
        currentStatus.fuelPumpOn = true;
      }
    }
    else
    {
      //We reach here if the time between teeth is too great. This VERY likely means the engine has stopped
      currentStatus.RPM = 0;
      currentStatus.RPMdiv100 = 0;
      currentStatus.PW1 = 0;
      currentStatus.VE = 0;
      currentStatus.VE2 = 0;
      toothLastToothTime = 0;
      toothLastSecToothTime = 0;
      //toothLastMinusOneToothTime = 0;
      currentStatus.hasSync = false;
      BIT_CLEAR(currentStatus.status3, BIT_STATUS3_HALFSYNC);
      currentStatus.runSecs = 0; //Reset the counter for number of seconds running.
      currentStatus.startRevolutions = 0;
      toothSystemCount = 0;
      secondaryToothCount = 0;
      MAPcurRev = 0;
      MAPcount = 0;
      currentStatus.rpmDOT = 0;
      AFRnextCycle = 0;
      ignitionCount = 0;
      ignitionChannelsOn = 0;
      fuelChannelsOn = 0;
      if (currentStatus.fpPrimed == true) { FUEL_PUMP_OFF(); currentStatus.fuelPumpOn = false; } //Turn off the fuel pump, but only if the priming is complete
      if (configPage6.iacPWMrun == false) { disableIdle(); } //Turn off the idle PWM
      BIT_CLEAR(currentStatus.engine, BIT_ENGINE_CRANK); //Clear cranking bit (Can otherwise get stuck 'on' even with 0 rpm)
      BIT_CLEAR(currentStatus.engine, BIT_ENGINE_WARMUP); //Same as above except for WUE
      BIT_CLEAR(currentStatus.engine, BIT_ENGINE_RUN); //Same as above except for RUNNING status
      BIT_CLEAR(currentStatus.engine, BIT_ENGINE_ASE); //Same as above except for ASE status
      BIT_CLEAR(currentStatus.engine, BIT_ENGINE_ACC); //Same as above but the accel enrich (If using MAP accel enrich a stall will cause this to trigger)
      BIT_CLEAR(currentStatus.engine, BIT_ENGINE_DCC); //Same as above but the decel enleanment
      //This is a safety check. If for some reason the interrupts have got screwed up (Leading to 0rpm), this resets them.
      //It can possibly be run much less frequently.
      //This should only be run if the high speed logger are off because it will change the trigger interrupts back to defaults rather than the logger versions
      if( (currentStatus.toothLogEnabled == false) && (currentStatus.compositeTriggerUsed == 0) ) { initialiseTriggers(); }

      VVT1_PIN_LOW();
      VVT2_PIN_LOW();
      DISABLE_VVT_TIMER();
      boostDisable();
      if(configPage4.ignBypassEnabled > 0) { digitalWrite(pinIgnBypass, LOW); } //Reset the ignition bypass ready for next crank attempt
    }
    //***Perform sensor reads***
    //-----------------------------------------------------------------------------------------------------
    if (BIT_CHECK(LOOP_TIMER, BIT_TIMER_1KHZ)) //Every 1ms. NOTE: This is NOT guaranteed to run at 1kHz on AVR systems. It will run at 1kHz if possible or as fast as loops/s allows if not. 
    {
      BIT_CLEAR(TIMER_mask, BIT_TIMER_1KHZ);
      readMAP();
    }
    if(BIT_CHECK(LOOP_TIMER, BIT_TIMER_200HZ))
    {
      BIT_CLEAR(TIMER_mask, BIT_TIMER_200HZ);
      #if defined(ANALOG_ISR)
        //ADC in free running mode does 1 complete conversion of all 16 channels and then the interrupt is disabled. Every 200Hz we re-enable the interrupt to get another conversion cycle
        BIT_SET(ADCSRA,ADIE); //Enable ADC interrupt
      #endif
    }

    /**
    在一个周期性任务中执行的，用于每隔一段时间读取油门踏板位置传感器（TPS）的数据并进行其他控制逻辑操作
    解析：
    BIT_CHECK(LOOP_TIMER, BIT_TIMER_15HZ)

    这行代码检查 LOOP_TIMER 中的 BIT_TIMER_15HZ 位是否被设置。这是一个周期性检查，目的是每 32 次循环执行一次。这通常用于控制任务的频率，确保一定时间间隔后再执行相应操作。
    BIT_CHECK 是一个宏，用于检查特定位的状态。假设 BIT_TIMER_15HZ 是一个计时器标志位。
    BIT_CLEAR(TIMER_mask, BIT_TIMER_15HZ)

    如果上面的条件成立（即每经过 32 次循环），则会清除 BIT_TIMER_15HZ 位，准备下一轮的定时器检查。
    #if TPS_READ_FREQUENCY == 15

    这部分的条件编译语句检查是否设置了 TPS_READ_FREQUENCY 为 15。如果是，则执行 readTPS() 函数。readTPS() 函数用于读取油门踏板传感器的值（TPS），通常会对传感器数据进行滤波和校准。
    #if defined(CORE_TEENSY35)

    如果在编译时定义了 CORE_TEENSY35（表示使用 Teensy 35 微控制器），则执行与该硬件相关的代码。
    if (configPage9.enable_intcan == 1)

    检查 configPage9.enable_intcan 是否为 1，表示启用内部 CAN 模块。如果启用了内部 CAN，后续的代码会执行相关的 CAN 通信操作（目前是注释掉的 sendCancommand()，这是一个测试接口的命令）。
    checkLaunchAndFlatShift();

    调用 checkLaunchAndFlatShift() 函数检查是否启用了启动控制和超车换挡（Flat Shift）功能。这两个功能通常用于性能提升，在发动机控制系统中非常常见：
    启动控制（Launch Control）：控制发动机在起步时的转速，以避免车轮打滑。
    超车换挡（Flat Shift）：在加速时，平稳换挡以避免丢失动力。
    注释部分：

    注释部分表示会检查齿轮信号的日志缓冲区是否准备好。齿轮信号通常用于同步发动机控制和变速器之间的操作。
    总结：
    这段代码的主要作用是在周期性任务中读取油门踏板位置（TPS），执行一些与硬件相关的操作（如内部CAN通讯测试），并检查是否启用启动控制和超车换挡功能。代码使用了定时器标志位（如 BIT_TIMER_15HZ）来确保在每 32 次循环后执行这些操作，从而避免频率过高导致系统不稳定。
    **/
    if (BIT_CHECK(LOOP_TIMER, BIT_TIMER_15HZ)) //Every 32 loops
    {
      BIT_CLEAR(TIMER_mask, BIT_TIMER_15HZ);
      #if TPS_READ_FREQUENCY == 15
        readTPS(); //TPS reading to be performed every 32 loops (any faster and it can upset the TPSdot sampling time)
      #endif
      #if  defined(CORE_TEENSY35)       
          if (configPage9.enable_intcan == 1) // use internal can module
          {
           // this is just to test the interface is sending
           //sendCancommand(3,((configPage9.realtime_base_address & 0x3FF)+ 0x100),currentStatus.TPS,0,0x200);
          }
      #endif     

      checkLaunchAndFlatShift(); //Check for launch control and flat shift being active

      //And check whether the tooth log buffer is ready
      if(toothHistoryIndex > TOOTH_LOG_SIZE) { BIT_SET(currentStatus.status1, BIT_STATUS1_TOOTHLOG1READY); }

      

    }
    if(BIT_CHECK(LOOP_TIMER, BIT_TIMER_10HZ)) //10 hertz
    {
      BIT_CLEAR(TIMER_mask, BIT_TIMER_10HZ);
      //updateFullStatus();
      checkProgrammableIO();
      idleControl(); //Perform any idle related actions. This needs to be run at 10Hz to align with the idle taper resolution of 0.1s
      
      // Air conditioning control
      airConControl();

      currentStatus.vss = getSpeed();
      currentStatus.gear = getGear();

      #ifdef SD_LOGGING
        if(configPage13.onboard_log_file_rate == LOGGER_RATE_10HZ) { writeSDLogEntry(); }
      #endif
    }
    if(BIT_CHECK(LOOP_TIMER, BIT_TIMER_30HZ)) //30 hertz
    {
      BIT_CLEAR(TIMER_mask, BIT_TIMER_30HZ);
      /**
      目的：控制涡轮增压系统（Boost Control），根据当前发动机的运行状态来调整增压压力。
      调用频率：这段代码注释指出，大多数增压系统的运行频率大约为30Hz。将这个函数放在这里，可以确保在足够频繁的时间内获取新的增压目标值，保证增压系统的实时响应。
      **/
      //Most boost tends to run at about 30Hz, so placing it here ensures a new target time is fetched frequently enough
      boostControl();
      /**
      目的：控制可变气门正时系统（VVT, Variable Valve Timing）。VVT 系统可以根据发动机的需求调整气门的开启和关闭时间，以优化发动机性能。
      调用频率：代码注释提到，VVT 系统可能最终需要与凸轮轴读数同步（例如每转一圈时运行一次），但目前它以30Hz的频率运行。
      **/
      //VVT may eventually need to be synced with the cam readings (ie run once per cam rev) but for now run at 30Hz

      vvtControl();
    /**
    目的：控制水甲醇注入系统（WMI, Water Methanol Injection）。WMI 系统通过向进气系统喷射水甲醇混合物来降低进气温度，从而提高发动机的抗爆性和性能。
    调用频率：尽管代码没有给出详细注释，但在30Hz的频率下调用该函数应该足以保持系统的有效控制和响应。
    **/
      //Water methanol injection
      wmiControl();
      #if defined(NATIVE_CAN_AVAILABLE)
      if (configPage2.canBMWCluster == true) { sendBMWCluster(); }
      if (configPage2.canVAGCluster == true) { sendVAGCluster(); }
      #endif
      /**
      按频率读取油门数据
      **/
      #if TPS_READ_FREQUENCY == 30
        readTPS();
      #endif
      if (configPage2.canWBO == 0)
      {
        readO2();
        readO2_2();
      }      
      #ifdef SD_LOGGING
        if(configPage13.onboard_log_file_rate == LOGGER_RATE_30HZ) { writeSDLogEntry(); }
      #endif

      //Check for any outstanding EEPROM writes.
      if( (isEepromWritePending() == true) && (serialStatusFlag == SERIAL_INACTIVE) && (micros() > deferEEPROMWritesUntil)) { writeAllConfig(); } 
    }
    if (BIT_CHECK(LOOP_TIMER, BIT_TIMER_4HZ))
    {
      BIT_CLEAR(TIMER_mask, BIT_TIMER_4HZ);
      //The IAT and CLT readings can be done less frequently (4 times per second)
      readCLT();
      readIAT();
      readBat();
      nitrousControl();

      //Lookup the current target idle RPM. This is aligned with coolant and so needs to be calculated at the same rate CLT is read
      if( (configPage2.idleAdvEnabled >= 1) || (configPage6.iacAlgorithm != IAC_ALGORITHM_NONE) )
      {
        currentStatus.CLIdleTarget = (byte)table2D_getValue(&idleTargetTable, currentStatus.coolant + CALIBRATION_TEMPERATURE_OFFSET); //All temps are offset by 40 degrees
        if(BIT_CHECK(currentStatus.airConStatus, BIT_AIRCON_TURNING_ON)) { currentStatus.CLIdleTarget += configPage15.airConIdleUpRPMAdder;  } //Adds Idle Up RPM amount if active
      }

      #ifdef SD_LOGGING
        if(configPage13.onboard_log_file_rate == LOGGER_RATE_4HZ) { writeSDLogEntry(); }
        syncSDLog(); //Sync the SD log file to the card 4 times per second. 
      #endif  
      
      currentStatus.fuelPressure = getFuelPressure();
      currentStatus.oilPressure = getOilPressure();
      
      if(auxIsEnabled == true)
      {
        //TODO dazq to clean this right up :)
        //check through the Aux input channels if enabled for Can or local use
        for (byte AuxinChan = 0; AuxinChan <16 ; AuxinChan++)
        {
          currentStatus.current_caninchannel = AuxinChan;          
          
          if (((configPage9.caninput_sel[currentStatus.current_caninchannel]&12) == 4) 
              && (((configPage9.enable_secondarySerial == 1) && ((configPage9.enable_intcan == 0)&&(configPage9.intcan_available == 1)))
              || ((configPage9.enable_secondarySerial == 1) && ((configPage9.enable_intcan == 1)&&(configPage9.intcan_available == 1))&& 
              ((configPage9.caninput_sel[currentStatus.current_caninchannel]&64) == 0))
              || ((configPage9.enable_secondarySerial == 1) && ((configPage9.enable_intcan == 1)&&(configPage9.intcan_available == 0)))))              
          { //if current input channel is enabled as external & secondary serial enabled & internal can disabled(but internal can is available)
            // or current input channel is enabled as external & secondary serial enabled & internal can enabled(and internal can is available)
            //currentStatus.canin[13] = 11;  Dev test use only!
            if (configPage9.enable_secondarySerial == 1)  // megas only support can via secondary serial
            {
              sendCancommand(2,0,currentStatus.current_caninchannel,0,((configPage9.caninput_source_can_address[currentStatus.current_caninchannel]&2047)+0x100));
              //send an R command for data from caninput_source_address[currentStatus.current_caninchannel] from secondarySerial
            }
          }  
          else if (((configPage9.caninput_sel[currentStatus.current_caninchannel]&12) == 4) 
              && (((configPage9.enable_secondarySerial == 1) && ((configPage9.enable_intcan == 1)&&(configPage9.intcan_available == 1))&& 
              ((configPage9.caninput_sel[currentStatus.current_caninchannel]&64) == 64))
              || ((configPage9.enable_secondarySerial == 0) && ((configPage9.enable_intcan == 1)&&(configPage9.intcan_available == 1))&& 
              ((configPage9.caninput_sel[currentStatus.current_caninchannel]&128) == 128))))                             
          { //if current input channel is enabled as external for canbus & secondary serial enabled & internal can enabled(and internal can is available)
            // or current input channel is enabled as external for canbus & secondary serial disabled & internal can enabled(and internal can is available)
            //currentStatus.canin[13] = 12;  Dev test use only!  
          #if defined(CORE_STM32) || defined(CORE_TEENSY)
           if (configPage9.enable_intcan == 1) //  if internal can is enabled 
           {
              sendCancommand(3,configPage9.speeduino_tsCanId,currentStatus.current_caninchannel,0,((configPage9.caninput_source_can_address[currentStatus.current_caninchannel]&2047)+0x100));  
              //send an R command for data from caninput_source_address[currentStatus.current_caninchannel] from internal canbus
           }
          #endif
          }   
          else if ((((configPage9.enable_secondarySerial == 1) || ((configPage9.enable_intcan == 1) && (configPage9.intcan_available == 1))) && (configPage9.caninput_sel[currentStatus.current_caninchannel]&12) == 8)
                  || (((configPage9.enable_secondarySerial == 0) && ( (configPage9.enable_intcan == 1) && (configPage9.intcan_available == 0) )) && (configPage9.caninput_sel[currentStatus.current_caninchannel]&3) == 2)  
                  || (((configPage9.enable_secondarySerial == 0) && (configPage9.enable_intcan == 0)) && ((configPage9.caninput_sel[currentStatus.current_caninchannel]&3) == 2)))  
          { //if current input channel is enabled as analog local pin
            //read analog channel specified
            //currentStatus.canin[13] = (configPage9.Auxinpina[currentStatus.current_caninchannel]&63);  Dev test use only!127
            currentStatus.canin[currentStatus.current_caninchannel] = readAuxanalog(pinTranslateAnalog(configPage9.Auxinpina[currentStatus.current_caninchannel]&63));
          }
          else if ((((configPage9.enable_secondarySerial == 1) || ((configPage9.enable_intcan == 1) && (configPage9.intcan_available == 1))) && (configPage9.caninput_sel[currentStatus.current_caninchannel]&12) == 12)
                  || (((configPage9.enable_secondarySerial == 0) && ( (configPage9.enable_intcan == 1) && (configPage9.intcan_available == 0) )) && (configPage9.caninput_sel[currentStatus.current_caninchannel]&3) == 3)
                  || (((configPage9.enable_secondarySerial == 0) && (configPage9.enable_intcan == 0)) && ((configPage9.caninput_sel[currentStatus.current_caninchannel]&3) == 3)))
          { //if current input channel is enabled as digital local pin
            //read digital channel specified
            //currentStatus.canin[14] = ((configPage9.Auxinpinb[currentStatus.current_caninchannel]&63)+1);  Dev test use only!127+1
            currentStatus.canin[currentStatus.current_caninchannel] = readAuxdigital((configPage9.Auxinpinb[currentStatus.current_caninchannel]&63)+1);
          } //Channel type
        } //For loop going through each channel
      } //aux channels are enabled
    } //4Hz timer
    if (BIT_CHECK(LOOP_TIMER, BIT_TIMER_1HZ)) //Once per second)
    {
      BIT_CLEAR(TIMER_mask, BIT_TIMER_1HZ);
      readBaro(); //Infrequent baro readings are not an issue.

      if ( (configPage10.wmiEnabled > 0) && (configPage10.wmiIndicatorEnabled > 0) )
      {
        // water tank empty
        if (BIT_CHECK(currentStatus.status4, BIT_STATUS4_WMI_EMPTY) > 0)
        {
          // flash with 1sec interval
          digitalWrite(pinWMIIndicator, !digitalRead(pinWMIIndicator));
        }
        else
        {
          digitalWrite(pinWMIIndicator, configPage10.wmiIndicatorPolarity ? HIGH : LOW);
        } 
      }

      #ifdef SD_LOGGING
        if(configPage13.onboard_log_file_rate == LOGGER_RATE_1HZ) { writeSDLogEntry(); }
      #endif

    } //1Hz timer

    if( (configPage6.iacAlgorithm == IAC_ALGORITHM_STEP_OL)
    || (configPage6.iacAlgorithm == IAC_ALGORITHM_STEP_CL)
    || (configPage6.iacAlgorithm == IAC_ALGORITHM_STEP_OLCL) )
    {
      idleControl(); //Run idlecontrol every loop for stepper idle.
    }

    
    //VE and advance calculation were moved outside the sync/RPM check so that the fuel and ignition load value will be accurately shown when RPM=0
    currentStatus.VE1 = getVE1();
    currentStatus.VE = currentStatus.VE1; //Set the final VE value to be VE 1 as a default. This may be changed in the section below

    currentStatus.advance1 = getAdvance1();
    currentStatus.advance = currentStatus.advance1; //Set the final advance value to be advance 1 as a default. This may be changed in the section below

    calculateSecondaryFuel();
    calculateSecondarySpark();

    //Always check for sync
    //Main loop runs within this clause
    if ((currentStatus.hasSync || BIT_CHECK(currentStatus.status3, BIT_STATUS3_HALFSYNC)) && (currentStatus.RPM > 0))
    {
        //Check whether running or cranking
        if(currentStatus.RPM > currentStatus.crankRPM) //Crank RPM in the config is stored as a x10. currentStatus.crankRPM is set in timers.ino and represents the true value
        {
          BIT_SET(currentStatus.engine, BIT_ENGINE_RUN); //Sets the engine running bit
          //Only need to do anything if we're transitioning from cranking to running
          if( BIT_CHECK(currentStatus.engine, BIT_ENGINE_CRANK) )
          {
            BIT_CLEAR(currentStatus.engine, BIT_ENGINE_CRANK);
            if(configPage4.ignBypassEnabled > 0) { digitalWrite(pinIgnBypass, HIGH); }
          }
        }
        else
        {  
          if( !BIT_CHECK(currentStatus.engine, BIT_ENGINE_RUN) || (currentStatus.RPM < (currentStatus.crankRPM - CRANK_RUN_HYSTER)) )
          {
            //Sets the engine cranking bit, clears the engine running bit
            BIT_SET(currentStatus.engine, BIT_ENGINE_CRANK);
            BIT_CLEAR(currentStatus.engine, BIT_ENGINE_RUN);
            currentStatus.runSecs = 0; //We're cranking (hopefully), so reset the engine run time to prompt ASE.
            if(configPage4.ignBypassEnabled > 0) { digitalWrite(pinIgnBypass, LOW); }

            //Check whether the user has selected to disable to the fan during cranking
            if(configPage2.fanWhenCranking == 0) { FAN_OFF(); }
          }
        }
      //END SETTING ENGINE STATUSES
      //-----------------------------------------------------------------------------------------------------

      //Begin the fuel calculation
      //Calculate an injector pulsewidth from the VE
      currentStatus.corrections = correctionsFuel();

      currentStatus.PW1 = PW(req_fuel_uS, currentStatus.VE, currentStatus.MAP, currentStatus.corrections, inj_opentime_uS);

      //Manual adder for nitrous. These are not in correctionsFuel() because they are direct adders to the ms value, not % based
      if( (currentStatus.nitrous_status == NITROUS_STAGE1) || (currentStatus.nitrous_status == NITROUS_BOTH) )
      { 
        int16_t adderRange = (configPage10.n2o_stage1_maxRPM - configPage10.n2o_stage1_minRPM) * 100;
        int16_t adderPercent = ((currentStatus.RPM - (configPage10.n2o_stage1_minRPM * 100)) * 100) / adderRange; //The percentage of the way through the RPM range
        adderPercent = 100 - adderPercent; //Flip the percentage as we go from a higher adder to a lower adder as the RPMs rise
        currentStatus.PW1 = currentStatus.PW1 + (configPage10.n2o_stage1_adderMax + percentage(adderPercent, (configPage10.n2o_stage1_adderMin - configPage10.n2o_stage1_adderMax))) * 100; //Calculate the above percentage of the calculated ms value.
      }
      if( (currentStatus.nitrous_status == NITROUS_STAGE2) || (currentStatus.nitrous_status == NITROUS_BOTH) )
      {
        int16_t adderRange = (configPage10.n2o_stage2_maxRPM - configPage10.n2o_stage2_minRPM) * 100;
        int16_t adderPercent = ((currentStatus.RPM - (configPage10.n2o_stage2_minRPM * 100)) * 100) / adderRange; //The percentage of the way through the RPM range
        adderPercent = 100 - adderPercent; //Flip the percentage as we go from a higher adder to a lower adder as the RPMs rise
        currentStatus.PW1 = currentStatus.PW1 + (configPage10.n2o_stage2_adderMax + percentage(adderPercent, (configPage10.n2o_stage2_adderMin - configPage10.n2o_stage2_adderMax))) * 100; //Calculate the above percentage of the calculated ms value.
      }

      int injector1StartAngle = 0;
      uint16_t injector2StartAngle = 0;
      uint16_t injector3StartAngle = 0;
      uint16_t injector4StartAngle = 0;

      #if INJ_CHANNELS >= 5
      uint16_t injector5StartAngle = 0;
      #endif
      #if INJ_CHANNELS >= 6
      uint16_t injector6StartAngle = 0;
      #endif
      #if INJ_CHANNELS >= 7
      uint16_t injector7StartAngle = 0;
      #endif
      #if INJ_CHANNELS >= 8
      uint16_t injector8StartAngle = 0;
      #endif
      
      //Check that the duty cycle of the chosen pulsewidth isn't too high.
      uint16_t pwLimit = calculatePWLimit();
      //Apply the pwLimit if staging is disabled and engine is not cranking
      if( (!BIT_CHECK(currentStatus.engine, BIT_ENGINE_CRANK)) && (configPage10.stagingEnabled == false) ) { if (currentStatus.PW1 > pwLimit) { currentStatus.PW1 = pwLimit; } }

      calculateStaging(pwLimit);

      //***********************************************************************************************
      //BEGIN INJECTION TIMING
      currentStatus.injAngle = table2D_getValue(&injectorAngleTable, currentStatus.RPMdiv100);
      if(currentStatus.injAngle > uint16_t(CRANK_ANGLE_MAX_INJ)) { currentStatus.injAngle = uint16_t(CRANK_ANGLE_MAX_INJ); }

      unsigned int PWdivTimerPerDegree = timeToAngleDegPerMicroSec(currentStatus.PW1); //How many crank degrees the calculated PW will take at the current speed

      injector1StartAngle = calculateInjectorStartAngle(PWdivTimerPerDegree, channel1InjDegrees, currentStatus.injAngle);

      //Repeat the above for each cylinder
      switch (configPage2.nCylinders)
      {
        //Single cylinder 单缸引擎
        /**
        如果启用了分级进气（stagingEnabled == true），并且当前正在分级进气过程中（BIT_CHECK(currentStatus.status4, BIT_STATUS4_STAGING_ACTIVE) == true），则需要重新计算PWdivTimerPerDegree，因为在分级进气时，PW2的值会与平时截然不同。
        然后，调用calculateInjectorStartAngle函数计算第二个喷油嘴的启动角度，传递的参数为重新计算的PWdivTimerPerDegree、第一个喷油嘴的喷射角度channel1InjDegrees以及当前喷射角度currentStatus.injAngle。
        */
        case 1:
          //The only thing that needs to be done for single cylinder is to check for staging. 
          if( (configPage10.stagingEnabled == true) && (BIT_CHECK(currentStatus.status4, BIT_STATUS4_STAGING_ACTIVE) == true) )
          {
            //调用timeToAngleDegPerMicroSec函数重新计算了PWdivTimerPerDegree值，这是因为在分级进气过程中，PW2的值与正常情况下的PW1值会有显著不同。因此，需要重新计算PW2对应的时间到角度的转换系数PWdivTimerPerDegree。
            //这段代码中首先重新计算了PWdivTimerPerDegree，这是因为在分级进气过程中，PW2的值会与正常情况下的PW1值有显著不同。然后使用新计算的PWdivTimerPerDegree值调用calculateInjectorStartAngle函数，计算了第二个喷油器的起始角度（injector2StartAngle）。这里通过将PWdivTimerPerDegree传递给calculateInjectorStartAngle函数，以确保根据新的PW2值准确计算第二个喷油器的起始角度。
            PWdivTimerPerDegree = timeToAngleDegPerMicroSec(currentStatus.PW2); //Need to redo this for PW2 as it will be dramatically different to PW1 when staging
            //injector3StartAngle = calculateInjector3StartAngle(PWdivTimerPerDegree);
            injector2StartAngle = calculateInjectorStartAngle(PWdivTimerPerDegree, channel1InjDegrees, currentStatus.injAngle);
          }
          break;
        //2 cylinders 双缸引擎
        /**
        首先，计算第二个喷油嘴的启动角度，同样使用calculateInjectorStartAngle函数，传递的参数为PWdivTimerPerDegree、第二个喷油嘴的喷射角度channel2InjDegrees以及当前喷射角度currentStatus.injAngle。

        如果燃油喷射配置为顺序喷射（configPage2.injLayout == INJ_SEQUENTIAL）并且启用了燃油调整（configPage6.fuelTrimEnabled > 0），则需要对燃油喷射脉冲宽度进行修正。分别应用于PW1和PW2，通过调用applyFuelTrimToPW函数，传递的参数包括燃油负荷、发动机转速和燃油脉冲宽度。

        如果启用了分级进气（stagingEnabled == true）并且当前正在分级进气过程中，则需要重新计算PWdivTimerPerDegree，因为在分级进气时PW3的值会与平时不同。然后，计算第三个和第四个喷油嘴的启动角度，同样使用calculateInjectorStartAngle函数，传递的参数为重新计算的PWdivTimerPerDegree、第一个喷油嘴的喷射角度channel1InjDegrees以及当前喷射角度currentStatus.injAngle。

        最后，调整第四个喷油嘴的启动角度，与第三个喷油嘴的启动角度相位差为CRANK_ANGLE_MAX_INJ / 2（即180度），因为当前系统不支持同时进行顺序喷射和分级进气。
        */
        case 2:
          //injector2StartAngle = calculateInjector2StartAngle(PWdivTimerPerDegree);
          injector2StartAngle = calculateInjectorStartAngle(PWdivTimerPerDegree, channel2InjDegrees, currentStatus.injAngle);
          
          if ( (configPage2.injLayout == INJ_SEQUENTIAL) && (configPage6.fuelTrimEnabled > 0) )
          {
            //传递的参数包括燃油负荷、发动机转速和燃油脉冲宽度。
            currentStatus.PW1 = applyFuelTrimToPW(&trim1Table, currentStatus.fuelLoad, currentStatus.RPM, currentStatus.PW1);
            currentStatus.PW2 = applyFuelTrimToPW(&trim2Table, currentStatus.fuelLoad, currentStatus.RPM, currentStatus.PW2);
          }
          else if( (configPage10.stagingEnabled == true) && (BIT_CHECK(currentStatus.status4, BIT_STATUS4_STAGING_ACTIVE) == true) )
          {
            PWdivTimerPerDegree = timeToAngleDegPerMicroSec(currentStatus.PW3); //Need to redo this for PW3 as it will be dramatically different to PW1 when staging
            injector3StartAngle = calculateInjectorStartAngle(PWdivTimerPerDegree, channel1InjDegrees, currentStatus.injAngle);
            injector4StartAngle = calculateInjectorStartAngle(PWdivTimerPerDegree, channel2InjDegrees, currentStatus.injAngle);

            injector4StartAngle = injector3StartAngle + (CRANK_ANGLE_MAX_INJ / 2); //Phase this either 180 or 360 degrees out from inj3 (In reality this will always be 180 as you can't have sequential and staged currently)
            if(injector4StartAngle > (uint16_t)CRANK_ANGLE_MAX_INJ) { injector4StartAngle -= CRANK_ANGLE_MAX_INJ; }
          }
          break;
        //3 cylinders
        case 3:
          /**
          在这段代码中，首先计算了第二和第三个喷油器的起始角度（injector2StartAngle和injector3StartAngle）。然后，根据配置情况和分级进气状态进行了不同的处理：

          如果喷油器布局为顺序喷射（INJ_SEQUENTIAL）且燃料调整功能启用，则对三个喷油器的脉冲宽度进行了燃料调整，分别调用了applyFuelTrimToPW函数，并将结果保存在currentStatus.PW1、currentStatus.PW2和currentStatus.PW3中。然后，如果系统支持6个或更多喷油器通道，并且分级进气功能启用且处于活动状态，重新计算了第四个、第五个和第六个喷油器的起始角度（injector4StartAngle、injector5StartAngle和injector6StartAngle）。

          如果分级进气功能启用且处于活动状态，但是喷油器布局不是顺序喷射，那么也会重新计算第四个喷油器的起始角度（injector4StartAngle）。如果系统支持6个或更多喷油器通道，则还会重新计算第五个和第六个喷油器的起始角度（injector5StartAngle和injector6StartAngle）。
          */
          //injector2StartAngle = calculateInjector2StartAngle(PWdivTimerPerDegree);
          //injector3StartAngle = calculateInjector3StartAngle(PWdivTimerPerDegree);
          injector2StartAngle = calculateInjectorStartAngle(PWdivTimerPerDegree, channel2InjDegrees, currentStatus.injAngle);
          injector3StartAngle = calculateInjectorStartAngle(PWdivTimerPerDegree, channel3InjDegrees, currentStatus.injAngle);
          
          if ( (configPage2.injLayout == INJ_SEQUENTIAL) && (configPage6.fuelTrimEnabled > 0) )
          {
            currentStatus.PW1 = applyFuelTrimToPW(&trim1Table, currentStatus.fuelLoad, currentStatus.RPM, currentStatus.PW1);
            currentStatus.PW2 = applyFuelTrimToPW(&trim2Table, currentStatus.fuelLoad, currentStatus.RPM, currentStatus.PW2);
            currentStatus.PW3 = applyFuelTrimToPW(&trim3Table, currentStatus.fuelLoad, currentStatus.RPM, currentStatus.PW3);

            #if INJ_CHANNELS >= 6
              if( (configPage10.stagingEnabled == true) && (BIT_CHECK(currentStatus.status4, BIT_STATUS4_STAGING_ACTIVE) == true) )
              {
                PWdivTimerPerDegree = timeToAngleDegPerMicroSec(currentStatus.PW4); //Need to redo this for PW4 as it will be dramatically different to PW1 when staging
                injector4StartAngle = calculateInjectorStartAngle(PWdivTimerPerDegree, channel1InjDegrees, currentStatus.injAngle);
                injector5StartAngle = calculateInjectorStartAngle(PWdivTimerPerDegree, channel2InjDegrees, currentStatus.injAngle);
                injector6StartAngle = calculateInjectorStartAngle(PWdivTimerPerDegree, channel3InjDegrees, currentStatus.injAngle);
              }
            #endif
          }
          else if( (configPage10.stagingEnabled == true) && (BIT_CHECK(currentStatus.status4, BIT_STATUS4_STAGING_ACTIVE) == true) )
          {
            PWdivTimerPerDegree = timeToAngleDegPerMicroSec(currentStatus.PW4); //Need to redo this for PW3 as it will be dramatically different to PW1 when staging
            injector4StartAngle = calculateInjectorStartAngle(PWdivTimerPerDegree, channel1InjDegrees, currentStatus.injAngle);
            #if INJ_CHANNELS >= 6
              injector5StartAngle = calculateInjectorStartAngle(PWdivTimerPerDegree, channel2InjDegrees, currentStatus.injAngle);
              injector6StartAngle = calculateInjectorStartAngle(PWdivTimerPerDegree, channel3InjDegrees, currentStatus.injAngle);
            #endif
          }
          break;
        //4 cylinders
        case 4:
          /**
          在这段代码中，针对4缸引擎，进行了以下处理：

          首先，计算了第二个喷油器的起始角度（injector2StartAngle）。

          然后，根据配置情况和当前状态进行了不同的处理：

          如果喷油器布局为顺序喷射（INJ_SEQUENTIAL）且已经发生了同步，则进一步判断是否需要切换为完全同步模式，如果当前的喷油器最大角度不等于720度，则切换为完全同步。然后计算了第三个和第四个喷油器的起始角度（injector3StartAngle和injector4StartAngle）。如果系统支持8个或更多喷油器通道，并且分级进气功能启用且处于活动状态，则重新计算了第五个至第八个喷油器的起始角度（injector5StartAngle至injector8StartAngle）。最后，如果燃料调整功能启用，则对四个喷油器的脉冲宽度进行了燃料调整，分别调用了applyFuelTrimToPW函数。
          如果分级进气功能启用且处于活动状态，但是喷油器布局不是顺序喷射，则重新计算了第三个和第四个喷油器的起始角度（injector3StartAngle和injector4StartAngle）。
          如果以上条件都不满足，则判断是否发生了半同步，并且当前的喷油器最大角度不等于360度，则切换为半同步模式。
          这段代码根据不同的引擎类型和配置情况，计算了相应喷油器的起始角度，并进行了相应的处理，以确保喷油器的工作正常。
          */
          //injector2StartAngle = calculateInjector2StartAngle(PWdivTimerPerDegree);
          injector2StartAngle = calculateInjectorStartAngle(PWdivTimerPerDegree, channel2InjDegrees, currentStatus.injAngle);

          if((configPage2.injLayout == INJ_SEQUENTIAL) && currentStatus.hasSync)
          {
            if( CRANK_ANGLE_MAX_INJ != 720 ) { changeHalfToFullSync(); }

            injector3StartAngle = calculateInjectorStartAngle(PWdivTimerPerDegree, channel3InjDegrees, currentStatus.injAngle);
            injector4StartAngle = calculateInjectorStartAngle(PWdivTimerPerDegree, channel4InjDegrees, currentStatus.injAngle);
            #if INJ_CHANNELS >= 8
              if( (configPage10.stagingEnabled == true) && (BIT_CHECK(currentStatus.status4, BIT_STATUS4_STAGING_ACTIVE) == true) )
              {
                PWdivTimerPerDegree = timeToAngleDegPerMicroSec(currentStatus.PW5); //Need to redo this for PW5 as it will be dramatically different to PW1 when staging
                injector5StartAngle = calculateInjectorStartAngle(PWdivTimerPerDegree, channel1InjDegrees, currentStatus.injAngle);
                injector6StartAngle = calculateInjectorStartAngle(PWdivTimerPerDegree, channel2InjDegrees, currentStatus.injAngle);
                injector7StartAngle = calculateInjectorStartAngle(PWdivTimerPerDegree, channel3InjDegrees, currentStatus.injAngle);
                injector8StartAngle = calculateInjectorStartAngle(PWdivTimerPerDegree, channel4InjDegrees, currentStatus.injAngle);
              }
            #endif

            if(configPage6.fuelTrimEnabled > 0)
            {
              currentStatus.PW1 = applyFuelTrimToPW(&trim1Table, currentStatus.fuelLoad, currentStatus.RPM, currentStatus.PW1);
              currentStatus.PW2 = applyFuelTrimToPW(&trim2Table, currentStatus.fuelLoad, currentStatus.RPM, currentStatus.PW2);
              currentStatus.PW3 = applyFuelTrimToPW(&trim3Table, currentStatus.fuelLoad, currentStatus.RPM, currentStatus.PW3);
              currentStatus.PW4 = applyFuelTrimToPW(&trim4Table, currentStatus.fuelLoad, currentStatus.RPM, currentStatus.PW4);
            }
          }
          else if( (configPage10.stagingEnabled == true) && (BIT_CHECK(currentStatus.status4, BIT_STATUS4_STAGING_ACTIVE) == true) )
          {
            PWdivTimerPerDegree = timeToAngleDegPerMicroSec(currentStatus.PW3); //Need to redo this for PW3 as it will be dramatically different to PW1 when staging
            injector3StartAngle = calculateInjectorStartAngle(PWdivTimerPerDegree, channel1InjDegrees, currentStatus.injAngle);
            injector4StartAngle = calculateInjectorStartAngle(PWdivTimerPerDegree, channel2InjDegrees, currentStatus.injAngle);
          }
          else
          {
            if( BIT_CHECK(currentStatus.status3, BIT_STATUS3_HALFSYNC) && (CRANK_ANGLE_MAX_INJ != 360) ) { changeFullToHalfSync(); }
          }
          break;
        //5 cylinders
        case 5:
          /**
          在这段代码中，针对5缸引擎进行了以下处理：

          首先，计算了第二个到第五个喷油器的起始角度（injector2StartAngle到injector5StartAngle）。

          如果系统支持5个或更多的喷油器通道，则进一步判断是否有第六个通道可用，并且分级进气功能启用且处于活动状态。如果满足条件，则重新计算第六个喷油器的起始角度（injector6StartAngle）。

          这段代码根据5缸引擎的特性，计算了相应喷油器的起始角度，并在需要时进行了分级进气的处理。
          */
          injector2StartAngle = calculateInjectorStartAngle(PWdivTimerPerDegree, channel2InjDegrees, currentStatus.injAngle);
          injector3StartAngle = calculateInjectorStartAngle(PWdivTimerPerDegree, channel3InjDegrees, currentStatus.injAngle);
          injector4StartAngle = calculateInjectorStartAngle(PWdivTimerPerDegree, channel4InjDegrees, currentStatus.injAngle);
          #if INJ_CHANNELS >= 5
            injector5StartAngle = calculateInjectorStartAngle(PWdivTimerPerDegree, channel5InjDegrees, currentStatus.injAngle);
          #endif

          //Staging is possible by using the 6th channel if available
          #if INJ_CHANNELS >= 6
            if( (configPage10.stagingEnabled == true) && (BIT_CHECK(currentStatus.status4, BIT_STATUS4_STAGING_ACTIVE) == true) )
            {
              PWdivTimerPerDegree = timeToAngleDegPerMicroSec(currentStatus.PW6);
              injector6StartAngle = calculateInjectorStartAngle(PWdivTimerPerDegree, channel6InjDegrees, currentStatus.injAngle);
            }
          #endif

          break;
        //6 cylinders
        /**
        这段代码处理了6缸引擎的情况：

        首先，计算了第二个到第三个喷油器的起始角度（injector2StartAngle和injector3StartAngle）。

        如果系统支持6个或更多的喷油器通道，则进一步判断是否有第四到第六个通道可用，并且喷油器布局为顺序喷射且同步信号存在。如果满足条件，则将同步信号从半同步改为全同步。

        然后计算第四到第六个喷油器的起始角度（injector4StartAngle到injector6StartAngle）。

        如果燃油修正功能启用，还会对每个喷油器的喷油脉冲宽度进行修正。

        如果系统支持8个或更多的喷油器通道，并且分级进气功能启用且处于活动状态，则将第四到第六个喷油器的起始角度重新计算，以支持分级进气。
        */
        case 6:
          injector2StartAngle = calculateInjectorStartAngle(PWdivTimerPerDegree, channel2InjDegrees, currentStatus.injAngle);
          injector3StartAngle = calculateInjectorStartAngle(PWdivTimerPerDegree, channel3InjDegrees, currentStatus.injAngle);
          
          #if INJ_CHANNELS >= 6
            if((configPage2.injLayout == INJ_SEQUENTIAL) && currentStatus.hasSync)
            {
              if( CRANK_ANGLE_MAX_INJ != 720 ) { changeHalfToFullSync(); }

              injector4StartAngle = calculateInjectorStartAngle(PWdivTimerPerDegree, channel4InjDegrees, currentStatus.injAngle);
              injector5StartAngle = calculateInjectorStartAngle(PWdivTimerPerDegree, channel5InjDegrees, currentStatus.injAngle);
              injector6StartAngle = calculateInjectorStartAngle(PWdivTimerPerDegree, channel6InjDegrees, currentStatus.injAngle);

              if(configPage6.fuelTrimEnabled > 0)
              {
                currentStatus.PW1 = applyFuelTrimToPW(&trim1Table, currentStatus.fuelLoad, currentStatus.RPM, currentStatus.PW1);
                currentStatus.PW2 = applyFuelTrimToPW(&trim2Table, currentStatus.fuelLoad, currentStatus.RPM, currentStatus.PW2);
                currentStatus.PW3 = applyFuelTrimToPW(&trim3Table, currentStatus.fuelLoad, currentStatus.RPM, currentStatus.PW3);
                currentStatus.PW4 = applyFuelTrimToPW(&trim4Table, currentStatus.fuelLoad, currentStatus.RPM, currentStatus.PW4);
                currentStatus.PW5 = applyFuelTrimToPW(&trim5Table, currentStatus.fuelLoad, currentStatus.RPM, currentStatus.PW5);
                currentStatus.PW6 = applyFuelTrimToPW(&trim6Table, currentStatus.fuelLoad, currentStatus.RPM, currentStatus.PW6);
              }

              //Staging is possible with sequential on 8 channel boards by using outputs 7 + 8 for the staged injectors
              #if INJ_CHANNELS >= 8
                if( (configPage10.stagingEnabled == true) && (BIT_CHECK(currentStatus.status4, BIT_STATUS4_STAGING_ACTIVE) == true) )
                {
                  PWdivTimerPerDegree = timeToAngleDegPerMicroSec(currentStatus.PW4); //Need to redo this for staging PW as it will be dramatically different to PW1 when staging
                  injector4StartAngle = calculateInjectorStartAngle(PWdivTimerPerDegree, channel1InjDegrees, currentStatus.injAngle);
                  injector5StartAngle = calculateInjectorStartAngle(PWdivTimerPerDegree, channel2InjDegrees, currentStatus.injAngle);
                  injector6StartAngle = calculateInjectorStartAngle(PWdivTimerPerDegree, channel3InjDegrees, currentStatus.injAngle);
                }
              #endif
            }
            else
            {
              if( BIT_CHECK(currentStatus.status3, BIT_STATUS3_HALFSYNC) && (CRANK_ANGLE_MAX_INJ != 360) ) { changeFullToHalfSync(); }

              if( (configPage10.stagingEnabled == true) && (BIT_CHECK(currentStatus.status4, BIT_STATUS4_STAGING_ACTIVE) == true) )
              {
                PWdivTimerPerDegree = timeToAngleDegPerMicroSec(currentStatus.PW4); //Need to redo this for staging PW as it will be dramatically different to PW1 when staging
                injector4StartAngle = calculateInjectorStartAngle(PWdivTimerPerDegree, channel1InjDegrees, currentStatus.injAngle);
                injector5StartAngle = calculateInjectorStartAngle(PWdivTimerPerDegree, channel2InjDegrees, currentStatus.injAngle);
                injector6StartAngle = calculateInjectorStartAngle(PWdivTimerPerDegree, channel3InjDegrees, currentStatus.injAngle); 
              }
            }
          #endif
          break;
        //8 cylinders
        /**
        这段代码处理了8缸引擎的情况：

        首先，计算了第二到第四个喷油器的起始角度（injector2StartAngle到injector4StartAngle）。

        如果系统支持8个或更多的喷油器通道，则进一步判断是否有第五到第八个通道可用，并且喷油器布局为顺序喷射且同步信号存在。如果满足条件，则将同步信号从半同步改为全同步。

        然后计算第五到第八个喷油器的起始角度（injector5StartAngle到injector8StartAngle）。

        如果燃油修正功能启用，还会对每个喷油器的喷油脉冲宽度进行修正。
        */
        case 8:
          injector2StartAngle = calculateInjectorStartAngle(PWdivTimerPerDegree, channel2InjDegrees, currentStatus.injAngle);
          injector3StartAngle = calculateInjectorStartAngle(PWdivTimerPerDegree, channel3InjDegrees, currentStatus.injAngle);
          injector4StartAngle = calculateInjectorStartAngle(PWdivTimerPerDegree, channel4InjDegrees, currentStatus.injAngle);

          #if INJ_CHANNELS >= 8
            if((configPage2.injLayout == INJ_SEQUENTIAL) && currentStatus.hasSync)
            {
              if( CRANK_ANGLE_MAX_INJ != 720 ) { changeHalfToFullSync(); }

              injector5StartAngle = calculateInjectorStartAngle(PWdivTimerPerDegree, channel5InjDegrees, currentStatus.injAngle);
              injector6StartAngle = calculateInjectorStartAngle(PWdivTimerPerDegree, channel6InjDegrees, currentStatus.injAngle);
              injector7StartAngle = calculateInjectorStartAngle(PWdivTimerPerDegree, channel7InjDegrees, currentStatus.injAngle);
              injector8StartAngle = calculateInjectorStartAngle(PWdivTimerPerDegree, channel8InjDegrees, currentStatus.injAngle);

              if(configPage6.fuelTrimEnabled > 0)
              {
                currentStatus.PW1 = applyFuelTrimToPW(&trim1Table, currentStatus.fuelLoad, currentStatus.RPM, currentStatus.PW1);
                currentStatus.PW2 = applyFuelTrimToPW(&trim2Table, currentStatus.fuelLoad, currentStatus.RPM, currentStatus.PW2);
                currentStatus.PW3 = applyFuelTrimToPW(&trim3Table, currentStatus.fuelLoad, currentStatus.RPM, currentStatus.PW3);
                currentStatus.PW4 = applyFuelTrimToPW(&trim4Table, currentStatus.fuelLoad, currentStatus.RPM, currentStatus.PW4);
                currentStatus.PW5 = applyFuelTrimToPW(&trim5Table, currentStatus.fuelLoad, currentStatus.RPM, currentStatus.PW5);
                currentStatus.PW6 = applyFuelTrimToPW(&trim6Table, currentStatus.fuelLoad, currentStatus.RPM, currentStatus.PW6);
                currentStatus.PW7 = applyFuelTrimToPW(&trim7Table, currentStatus.fuelLoad, currentStatus.RPM, currentStatus.PW7);
                currentStatus.PW8 = applyFuelTrimToPW(&trim8Table, currentStatus.fuelLoad, currentStatus.RPM, currentStatus.PW8);
              }
            }
            else
            {
              if( BIT_CHECK(currentStatus.status3, BIT_STATUS3_HALFSYNC) && (CRANK_ANGLE_MAX_INJ != 360) ) { changeFullToHalfSync(); }

              if( (configPage10.stagingEnabled == true) && (BIT_CHECK(currentStatus.status4, BIT_STATUS4_STAGING_ACTIVE) == true) )
              {
                PWdivTimerPerDegree = timeToAngleDegPerMicroSec(currentStatus.PW5); //Need to redo this for PW3 as it will be dramatically different to PW1 when staging
                injector5StartAngle = calculateInjectorStartAngle(PWdivTimerPerDegree, channel1InjDegrees, currentStatus.injAngle);
                injector6StartAngle = calculateInjectorStartAngle(PWdivTimerPerDegree, channel2InjDegrees, currentStatus.injAngle);
                injector7StartAngle = calculateInjectorStartAngle(PWdivTimerPerDegree, channel3InjDegrees, currentStatus.injAngle);
                injector8StartAngle = calculateInjectorStartAngle(PWdivTimerPerDegree, channel4InjDegrees, currentStatus.injAngle);
              }
            }

          #endif
          break;

        //Will hit the default case on 1 cylinder or >8 cylinders. Do nothing in these cases
        default:
          break;
      }

      //***********************************************************************************************
      //| BEGIN IGNITION CALCULATIONS
      /**
      这段代码实现了设置点火延迟和计算点火角度的逻辑，然后根据当前引擎状态进行相关保护操作：

      首先，根据引擎是否处于曲轴转动状态（BIT_ENGINE_CRANK）来确定使用启动时的点火延迟（cranking dwell）还是运行时的点火延迟（running dwell）。若使用运行时点火延迟，可以选择使用动态的延迟值（使用地图）或者固定的延迟值。

      接下来，将点火延迟转换为点火角度，并进行可能的修正。

      如果启用了每齿点火跟踪（perToothIgn），则执行相应的设置，确定每个点火事件的结束齿。

      进行引擎保护和限速检查，根据设定的引擎保护和限速策略，调整最大允许的引擎转速。

      最后，如果启用了硬限制（hard cut），并且当前转速超过了最大允许值，则执行相应的操作，如关闭点火和/或燃油通道，以实现硬限制功能。
      */
      //Set dwell
      //Dwell is stored as ms * 10. ie Dwell of 4.3ms would be 43 in configPage4. This number therefore needs to be multiplied by 100 to get dwell in uS
      if ( BIT_CHECK(currentStatus.engine, BIT_ENGINE_CRANK) ) {
        currentStatus.dwell =  (configPage4.dwellCrank * 100U); //use cranking dwell
      }
      else 
      {
        if ( configPage2.useDwellMap == true )
        {
          currentStatus.dwell = (get3DTableValue(&dwellTable, currentStatus.ignLoad, currentStatus.RPM) * 100U); //use running dwell from map
        }
        else
        {
          currentStatus.dwell =  (configPage4.dwellRun * 100U); //use fixed running dwell
        }
      }
      currentStatus.dwell = correctionsDwell(currentStatus.dwell);

      // Convert the dwell time to dwell angle based on the current engine speed
      calculateIgnitionAngles(timeToAngleDegPerMicroSec(currentStatus.dwell));

      //If ignition timing is being tracked per tooth, perform the calcs to get the end teeth
      //This only needs to be run if the advance figure has changed, otherwise the end teeth will still be the same
      //if( (configPage2.perToothIgn == true) && (lastToothCalcAdvance != currentStatus.advance) ) { triggerSetEndTeeth(); }
      if( (configPage2.perToothIgn == true) ) { triggerSetEndTeeth(); }

      //***********************************************************************************************
      //| BEGIN FUEL SCHEDULES
      //Finally calculate the time (uS) until we reach the firing angles and set the schedules
      //We only need to set the schedule if we're BEFORE the open angle
      //This may potentially be called a number of times as we get closer and closer to the opening time

      //Determine the current crank angle
      int crankAngle = injectorLimits(getCrankAngle());

      // if(Serial && false)
      // {
      //   if(ignition1StartAngle > crankAngle)
      //   {
      //     noInterrupts();
      //     Serial.print("Time2LastTooth:"); Serial.println(micros()-toothLastToothTime);
      //     Serial.print("elapsedTime:"); Serial.println(elapsedTime);
      //     Serial.print("CurAngle:"); Serial.println(crankAngle);
      //     Serial.print("RPM:"); Serial.println(currentStatus.RPM);
      //     Serial.print("Tooth:"); Serial.println(toothCurrentCount);
      //     Serial.print("timePerDegree:"); Serial.println(timePerDegree);
      //     Serial.print("IGN1Angle:"); Serial.println(ignition1StartAngle);
      //     Serial.print("TimeToIGN1:"); Serial.println(angleToTime((ignition1StartAngle - crankAngle), CRANKMATH_METHOD_INTERVAL_REV));
      //     interrupts();
      //   }
      // }
      
      //Check for any of the engine protections or rev limiters being turned on
      uint16_t maxAllowedRPM = checkRevLimit(); //The maximum RPM allowed by all the potential limiters (Engine protection, 2-step, flat shift etc). Divided by 100. `checkRevLimit()` returns the current maximum RPM allow (divided by 100) based on either the fixed hard limit or the current coolant temp
      //Check each of the functions that has an RPM limit. Update the max allowed RPM if the function is active and has a lower RPM than already set
      if( checkEngineProtect() && (configPage4.engineProtectMaxRPM < maxAllowedRPM)) { maxAllowedRPM = configPage4.engineProtectMaxRPM; }
      if ( (currentStatus.launchingHard == true) && (configPage6.lnchHardLim < maxAllowedRPM) ) { maxAllowedRPM = configPage6.lnchHardLim; }
      maxAllowedRPM = maxAllowedRPM * 100; //All of the above limits are divided by 100, convert back to RPM
      if ( (currentStatus.flatShiftingHard == true) && (currentStatus.clutchEngagedRPM < maxAllowedRPM) ) { maxAllowedRPM = currentStatus.clutchEngagedRPM; } //Flat shifting is a special case as the RPM limit is based on when the clutch was engaged. It is not divided by 100 as it is set with the actual RPM
    
      if( (configPage2.hardCutType == HARD_CUT_FULL) && (currentStatus.RPM > maxAllowedRPM) )
      {
        //Full hard cut turns outputs off completely. 
        switch(configPage6.engineProtectType)
        {
          case PROTECT_CUT_OFF:
            //Make sure all channels are turned on
            ignitionChannelsOn = 0xFF;
            fuelChannelsOn = 0xFF;
            currentStatus.engineProtectStatus = 0;
            break;
          case PROTECT_CUT_IGN:
            ignitionChannelsOn = 0;
            break;
          case PROTECT_CUT_FUEL:
            fuelChannelsOn = 0;
            break;
          case PROTECT_CUT_BOTH:
            ignitionChannelsOn = 0;
            fuelChannelsOn = 0;
            break;
          default:
            ignitionChannelsOn = 0;
            fuelChannelsOn = 0;
            break;
        }
      } //Hard cut check

      /**
      这段代码是一个条件语句，用于实现滚动式硬限制（rolling cut）。在滚动式硬限制模式下，如果当前引擎转速超过了允许的最大转速加上一个特定的增量，就会触发硬限制。

      在这段代码中：

      首先检查硬限制类型是否为滚动式硬限制（HARD_CUT_ROLLING），并且当前转速是否超过了允许的最大转速加上配置的滚动保护转速增量（rollingProtRPMDelta）。

      如果满足滚动式硬限制的条件，则计算需要切断点火和燃油的比例，然后根据随机数确定每个点火通道和燃油通道的切断情况。

      如果某个通道需要被切断，根据配置的引擎保护类型，选择是切断点火、燃油还是同时切断点火和燃油。

      如果满足条件并且需要切断的通道被切断后，记录当前的转动圈数，并在下一次循环中根据条件重新计算是否需要切断点火和燃油。

      如果不满足滚动式硬限制的条件，则将引擎保护状态置为0，并根据情况将所有的点火通道和燃油通道打开。

      此外，代码还处理了一些特殊情况，如等待点火通道等待燃油通道开启一圈后再重新打开，以及检查是否需要启动燃油和点火通道。
      */
      else if( (configPage2.hardCutType == HARD_CUT_ROLLING) && (currentStatus.RPM > (maxAllowedRPM + (configPage15.rollingProtRPMDelta[0] * 10))) ) //Limit for rolling is the max allowed RPM minus the lowest value in the delta table (Delta values are negative!)
      { 
        uint8_t revolutionsToCut = 1;
        if(configPage2.strokes == FOUR_STROKE) { revolutionsToCut *= 2; } //4 stroke needs to cut for at least 2 revolutions
        if( (configPage4.sparkMode != IGN_MODE_SEQUENTIAL) || (configPage2.injLayout != INJ_SEQUENTIAL) ) { revolutionsToCut *= 2; } //4 stroke and non-sequential will cut for 4 revolutions minimum. This is to ensure no half fuel ignition cycles take place

        if(rollingCutLastRev == 0) { rollingCutLastRev = currentStatus.startRevolutions; } //First time check
        if ( (currentStatus.startRevolutions >= (rollingCutLastRev + revolutionsToCut)) || (currentStatus.RPM > maxAllowedRPM) ) //If current RPM is over the max allowed RPM always cut, otherwise check if the required number of revolutions have passed since the last cut
        { 
          uint8_t cutPercent = 0;
          int16_t rpmDelta = currentStatus.RPM - maxAllowedRPM;
          if(rpmDelta >= 0) { cutPercent = 100; } //If the current RPM is over the max allowed RPM then cut is full (100%)
          else { cutPercent = table2D_getValue(&rollingCutTable, (rpmDelta / 10) ); } //
          

          for(uint8_t x=0; x<max(maxIgnOutputs, maxInjOutputs); x++)
          {  
            if( (cutPercent == 100) || (random1to100() < cutPercent) )
            {
              switch(configPage6.engineProtectType)
              {
                case PROTECT_CUT_OFF:
                  //Make sure all channels are turned on
                  ignitionChannelsOn = 0xFF;
                  fuelChannelsOn = 0xFF;
                  break;
                case PROTECT_CUT_IGN:
                  BIT_CLEAR(ignitionChannelsOn, x); //Turn off this ignition channel
                  disablePendingIgnSchedule(x);
                  break;
                case PROTECT_CUT_FUEL:
                  BIT_CLEAR(fuelChannelsOn, x); //Turn off this fuel channel
                  disablePendingFuelSchedule(x);
                  break;
                case PROTECT_CUT_BOTH:
                  BIT_CLEAR(ignitionChannelsOn, x); //Turn off this ignition channel
                  BIT_CLEAR(fuelChannelsOn, x); //Turn off this fuel channel
                  disablePendingFuelSchedule(x);
                  disablePendingIgnSchedule(x);
                  break;
                default:
                  BIT_CLEAR(ignitionChannelsOn, x); //Turn off this ignition channel
                  BIT_CLEAR(fuelChannelsOn, x); //Turn off this fuel channel
                  break;
              }
            }
            else
            {
              //Turn fuel and ignition channels on

              //Special case for non-sequential, 4-stroke where both fuel and ignition are cut. The ignition pulses should wait 1 cycle after the fuel channels are turned back on before firing again
              if( (revolutionsToCut == 4) &&                          //4 stroke and non-sequential
                  (BIT_CHECK(fuelChannelsOn, x) == false) &&          //Fuel on this channel is currently off, meaning it is the first revolution after a cut
                  (configPage6.engineProtectType == PROTECT_CUT_BOTH) //Both fuel and ignition are cut
                )
              { BIT_SET(ignitionChannelsPending, x); } //Set this ignition channel as pending
              else { BIT_SET(ignitionChannelsOn, x); } //Turn on this ignition channel
                
              
              BIT_SET(fuelChannelsOn, x); //Turn on this fuel channel
            }
          }
          rollingCutLastRev = currentStatus.startRevolutions;
        }

        //Check whether there are any ignition channels that are waiting for injection pulses to occur before being turned back on. This can only occur when at least 2 revolutions have taken place since the fuel was turned back on
        //Note that ignitionChannelsPending can only be >0 on 4 stroke, non-sequential fuel when protect type is Both
        if( (ignitionChannelsPending > 0) && (currentStatus.startRevolutions >= (rollingCutLastRev + 2)) )
        {
          ignitionChannelsOn = fuelChannelsOn;
          ignitionChannelsPending = 0;
        }
      } //Rolling cut check
      else
      {
        currentStatus.engineProtectStatus = 0;
        //No engine protection active, so turn all the channels on
        if(currentStatus.startRevolutions >= configPage4.StgCycles)
        { 
          //Enable the fuel and ignition, assuming staging revolutions are complete 
          ignitionChannelsOn = 0xff; 
          fuelChannelsOn = 0xff; 
        } 
      }


#if INJ_CHANNELS >= 1
      if( (maxInjOutputs >= 1) && (currentStatus.PW1 >= inj_opentime_uS) && (BIT_CHECK(fuelChannelsOn, INJ1_CMD_BIT)) )
      {
        uint32_t timeOut = calculateInjectorTimeout(fuelSchedule1, channel1InjDegrees, injector1StartAngle, crankAngle);
        if (timeOut>0U)
        {
            setFuelSchedule(fuelSchedule1, 
                      timeOut,
                      (unsigned long)currentStatus.PW1
                      );
        }
      }
#endif

        /*-----------------------------------------------------------------------------------------
        | A Note on tempCrankAngle and tempStartAngle:
        |   The use of tempCrankAngle/tempStartAngle is described below. It is then used in the same way for channels 2, 3 and 4+ on both injectors and ignition
        |   Essentially, these 2 variables are used to realign the current crank angle and the desired start angle around 0 degrees for the given cylinder/output
        |   Eg: If cylinder 2 TDC is 180 degrees after cylinder 1 (Eg a standard 4 cylinder engine), then tempCrankAngle is 180* less than the current crank angle and
        |       tempStartAngle is the desired open time less 180*. Thus the cylinder is being treated relative to its own TDC, regardless of its offset
        |
        |   This is done to avoid problems with very short of very long times until tempStartAngle.
        |------------------------------------------------------------------------------------------
        */
#if INJ_CHANNELS >= 2
        if( (maxInjOutputs >= 2) && (currentStatus.PW2 >= inj_opentime_uS) && (BIT_CHECK(fuelChannelsOn, INJ2_CMD_BIT)) )
        {
          uint32_t timeOut = calculateInjectorTimeout(fuelSchedule2, channel2InjDegrees, injector2StartAngle, crankAngle);
          if ( timeOut>0U )
          {
            setFuelSchedule(fuelSchedule2, 
                      timeOut,
                      (unsigned long)currentStatus.PW2
                      );
          }
        }
#endif

#if INJ_CHANNELS >= 3
        if( (maxInjOutputs >= 3) && (currentStatus.PW3 >= inj_opentime_uS) && (BIT_CHECK(fuelChannelsOn, INJ3_CMD_BIT)) )
        {
          uint32_t timeOut = calculateInjectorTimeout(fuelSchedule3, channel3InjDegrees, injector3StartAngle, crankAngle);
          if ( timeOut>0U )
          {
            setFuelSchedule(fuelSchedule3, 
                      timeOut,
                      (unsigned long)currentStatus.PW3
                      );
          }
        }
#endif

#if INJ_CHANNELS >= 4
        if( (maxInjOutputs >= 4) && (currentStatus.PW4 >= inj_opentime_uS) && (BIT_CHECK(fuelChannelsOn, INJ4_CMD_BIT)) )
        {
          uint32_t timeOut = calculateInjectorTimeout(fuelSchedule4, channel4InjDegrees, injector4StartAngle, crankAngle);
          if ( timeOut>0U )
          {
            setFuelSchedule(fuelSchedule4, 
                      timeOut,
                      (unsigned long)currentStatus.PW4
                      );
          }
        }
#endif

#if INJ_CHANNELS >= 5
        if( (maxInjOutputs >= 5) && (currentStatus.PW5 >= inj_opentime_uS) && (BIT_CHECK(fuelChannelsOn, INJ5_CMD_BIT)) )
        {
          uint32_t timeOut = calculateInjectorTimeout(fuelSchedule5, channel5InjDegrees, injector5StartAngle, crankAngle);
          if ( timeOut>0U )
          {
            setFuelSchedule(fuelSchedule5, 
                      timeOut,
                      (unsigned long)currentStatus.PW5
                      );
          }
        }
#endif

#if INJ_CHANNELS >= 6
        if( (maxInjOutputs >= 6) && (currentStatus.PW6 >= inj_opentime_uS) && (BIT_CHECK(fuelChannelsOn, INJ6_CMD_BIT)) )
        {
          uint32_t timeOut = calculateInjectorTimeout(fuelSchedule6, channel6InjDegrees, injector6StartAngle, crankAngle);
          if ( timeOut>0U )
          {
            setFuelSchedule(fuelSchedule6, 
                      timeOut,
                      (unsigned long)currentStatus.PW6
                      );
          }
        }
#endif

#if INJ_CHANNELS >= 7
        if( (maxInjOutputs >= 7) && (currentStatus.PW7 >= inj_opentime_uS) && (BIT_CHECK(fuelChannelsOn, INJ7_CMD_BIT)) )
        {
          uint32_t timeOut = calculateInjectorTimeout(fuelSchedule7, channel7InjDegrees, injector7StartAngle, crankAngle);
          if ( timeOut>0U )
          {
            setFuelSchedule(fuelSchedule7, 
                      timeOut,
                      (unsigned long)currentStatus.PW7
                      );
          }
        }
#endif

#if INJ_CHANNELS >= 8
        if( (maxInjOutputs >= 8) && (currentStatus.PW8 >= inj_opentime_uS) && (BIT_CHECK(fuelChannelsOn, INJ8_CMD_BIT)) )
        {
          uint32_t timeOut = calculateInjectorTimeout(fuelSchedule8, channel8InjDegrees, injector8StartAngle, crankAngle);
          if ( timeOut>0U )
          {
            setFuelSchedule(fuelSchedule8, 
                      timeOut,
                      (unsigned long)currentStatus.PW8
                      );
          }
        }
#endif

      //***********************************************************************************************
      //| BEGIN IGNITION SCHEDULES
      //Same as above, except for ignition

      //fixedCrankingOverride is used to extend the dwell during cranking so that the decoder can trigger the spark upon seeing a certain tooth. Currently only available on the basic distributor and 4g63 decoders.
      if ( configPage4.ignCranklock && BIT_CHECK(currentStatus.engine, BIT_ENGINE_CRANK) && (BIT_CHECK(decoderState, BIT_DECODER_HAS_FIXED_CRANKING)) )
      {
        fixedCrankingOverride = currentStatus.dwell * 3;
        //This is a safety step to prevent the ignition start time occurring AFTER the target tooth pulse has already occurred. It simply moves the start time forward a little, which is compensated for by the increase in the dwell time
        if(currentStatus.RPM < 250)
        {
          ignition1StartAngle -= 5;
          ignition2StartAngle -= 5;
          ignition3StartAngle -= 5;
          ignition4StartAngle -= 5;
#if IGN_CHANNELS >= 5
          ignition5StartAngle -= 5;
#endif
#if IGN_CHANNELS >= 6          
          ignition6StartAngle -= 5;
#endif
#if IGN_CHANNELS >= 7
          ignition7StartAngle -= 5;
#endif
#if IGN_CHANNELS >= 8
          ignition8StartAngle -= 5;
#endif
        }
      }
      else { fixedCrankingOverride = 0; }

      if(ignitionChannelsOn > 0)
      {
        //Refresh the current crank angle info
        //ignition1StartAngle = 335;
        crankAngle = ignitionLimits(getCrankAngle()); //Refresh the crank angle info

#if IGN_CHANNELS >= 1
        uint32_t timeOut = calculateIgnitionTimeout(ignitionSchedule1, ignition1StartAngle, channel1IgnDegrees, crankAngle);
        if ( (timeOut > 0U) && (BIT_CHECK(ignitionChannelsOn, IGN1_CMD_BIT)) )
        {
          setIgnitionSchedule(ignitionSchedule1, timeOut,
                    currentStatus.dwell + fixedCrankingOverride);
        }
#endif

#if defined(USE_IGN_REFRESH)
        if( (ignitionSchedule1.Status == RUNNING) && (ignition1EndAngle > crankAngle) && (configPage4.StgCycles == 0) && (configPage2.perToothIgn != true) )
        {
          unsigned long uSToEnd = 0;

          crankAngle = ignitionLimits(getCrankAngle()); //Refresh the crank angle info
          
          //ONLY ONE OF THE BELOW SHOULD BE USED (PROBABLY THE FIRST):
          //*********
          if(ignition1EndAngle > crankAngle) { uSToEnd = angleToTimeMicroSecPerDegree( (ignition1EndAngle - crankAngle) ); }
          else { uSToEnd = angleToTimeMicroSecPerDegree( (360 + ignition1EndAngle - crankAngle) ); }
          //*********
          //uSToEnd = ((ignition1EndAngle - crankAngle) * (toothLastToothTime - toothLastMinusOneToothTime)) / triggerToothAngle;
          //*********

          refreshIgnitionSchedule1( uSToEnd + fixedCrankingOverride );
        }
  #endif
        
#if IGN_CHANNELS >= 2
        if (maxIgnOutputs >= 2)
        {
            unsigned long ignition2StartTime = calculateIgnitionTimeout(ignitionSchedule2, ignition2StartAngle, channel2IgnDegrees, crankAngle);

            if ( (ignition2StartTime > 0) && (BIT_CHECK(ignitionChannelsOn, IGN2_CMD_BIT)) )
            {
              setIgnitionSchedule(ignitionSchedule2, ignition2StartTime,
                        currentStatus.dwell + fixedCrankingOverride);
            }
        }
#endif

#if IGN_CHANNELS >= 3
        if (maxIgnOutputs >= 3)
        {
            unsigned long ignition3StartTime = calculateIgnitionTimeout(ignitionSchedule3, ignition3StartAngle, channel3IgnDegrees, crankAngle);

            if ( (ignition3StartTime > 0) && (BIT_CHECK(ignitionChannelsOn, IGN3_CMD_BIT)) )
            {
              setIgnitionSchedule(ignitionSchedule3, ignition3StartTime,
                        currentStatus.dwell + fixedCrankingOverride);
            }
        }
#endif

#if IGN_CHANNELS >= 4
        if (maxIgnOutputs >= 4)
        {
            unsigned long ignition4StartTime = calculateIgnitionTimeout(ignitionSchedule4, ignition4StartAngle, channel4IgnDegrees, crankAngle);

            if ( (ignition4StartTime > 0) && (BIT_CHECK(ignitionChannelsOn, IGN4_CMD_BIT)) )
            {
              setIgnitionSchedule(ignitionSchedule4, ignition4StartTime,
                        currentStatus.dwell + fixedCrankingOverride);
            }
        }
#endif

#if IGN_CHANNELS >= 5
        if (maxIgnOutputs >= 5)
        {
            unsigned long ignition5StartTime = calculateIgnitionTimeout(ignitionSchedule5, ignition5StartAngle, channel5IgnDegrees, crankAngle);

            if ( (ignition5StartTime > 0) && (BIT_CHECK(ignitionChannelsOn, IGN5_CMD_BIT)) )
            {
              setIgnitionSchedule(ignitionSchedule5, ignition5StartTime,
                        currentStatus.dwell + fixedCrankingOverride);
            }
        }
#endif

#if IGN_CHANNELS >= 6
        if (maxIgnOutputs >= 6)
        {
            unsigned long ignition6StartTime = calculateIgnitionTimeout(ignitionSchedule6, ignition6StartAngle, channel6IgnDegrees, crankAngle);

            if ( (ignition6StartTime > 0) && (BIT_CHECK(ignitionChannelsOn, IGN6_CMD_BIT)) )
            {
              setIgnitionSchedule(ignitionSchedule6, ignition6StartTime,
                        currentStatus.dwell + fixedCrankingOverride);
            }
        }
#endif

#if IGN_CHANNELS >= 7
        if (maxIgnOutputs >= 7)
        {
            unsigned long ignition7StartTime = calculateIgnitionTimeout(ignitionSchedule7, ignition7StartAngle, channel7IgnDegrees, crankAngle);

            if ( (ignition7StartTime > 0) && (BIT_CHECK(ignitionChannelsOn, IGN7_CMD_BIT)) )
            {
              setIgnitionSchedule(ignitionSchedule7, ignition7StartTime,
                        currentStatus.dwell + fixedCrankingOverride);
            }
        }
#endif

#if IGN_CHANNELS >= 8
        if (maxIgnOutputs >= 8)
        {
            unsigned long ignition8StartTime = calculateIgnitionTimeout(ignitionSchedule8, ignition8StartAngle, channel8IgnDegrees, crankAngle);

            if ( (ignition8StartTime > 0) && (BIT_CHECK(ignitionChannelsOn, IGN8_CMD_BIT)) )
            {
              setIgnitionSchedule(ignitionSchedule8, ignition8StartTime,
                        currentStatus.dwell + fixedCrankingOverride);
            }
        }
#endif

      } //Ignition schedules on

      if ( (!BIT_CHECK(currentStatus.status3, BIT_STATUS3_RESET_PREVENT)) && (resetControl == RESET_CONTROL_PREVENT_WHEN_RUNNING) ) 
      {
        //Reset prevention is supposed to be on while the engine is running but isn't. Fix that.
        digitalWrite(pinResetControl, HIGH);
        BIT_SET(currentStatus.status3, BIT_STATUS3_RESET_PREVENT);
      }
    } //Has sync and RPM
    else if ( (BIT_CHECK(currentStatus.status3, BIT_STATUS3_RESET_PREVENT) > 0) && (resetControl == RESET_CONTROL_PREVENT_WHEN_RUNNING) )
    {
      digitalWrite(pinResetControl, LOW);
      BIT_CLEAR(currentStatus.status3, BIT_STATUS3_RESET_PREVENT);
    }
} //loop()
#endif //Unit test guard

/**
 * @brief This function calculates the required pulsewidth time (in us) given the current system state
 * 
 * @param REQ_FUEL The required fuel value in uS, as calculated by TunerStudio
 * @param VE Lookup from the main fuel table. This can either have been MAP or TPS based, depending on the algorithm used
 * @param MAP In KPa, read from the sensor (This is used when performing a multiply of the map only. It is applicable in both Speed density and Alpha-N)
 * @param corrections Sum of Enrichment factors (Cold start, acceleration). This is a multiplication factor (Eg to add 10%, this should be 110)
 * @param injOpen Injector opening time. The time the injector take to open minus the time it takes to close (Both in uS)
 * @return uint16_t The injector pulse width in uS
 */
uint16_t PW(int REQ_FUEL, byte VE, long MAP, uint16_t corrections, int injOpen)
{
  //Standard float version of the calculation
  //return (REQ_FUEL * (float)(VE/100.0) * (float)(MAP/100.0) * (float)(TPS/100.0) * (float)(corrections/100.0) + injOpen);
  //Note: The MAP and TPS portions are currently disabled, we use VE and corrections only
  uint16_t iVE;
  uint16_t iMAP = 100;
  uint16_t iAFR = 147;

  //100% float free version, does sacrifice a little bit of accuracy, but not much.
 
  //iVE = ((unsigned int)VE << 7) / 100;
  iVE = div100(((uint16_t)VE << 7U));

  //Check whether either of the multiply MAP modes is turned on
  //if ( configPage2.multiplyMAP == MULTIPLY_MAP_MODE_100) { iMAP = ((unsigned int)MAP << 7) / 100; }
  if ( configPage2.multiplyMAP == MULTIPLY_MAP_MODE_100) { iMAP = div100( ((uint16_t)MAP << 7U) ); }
  else if( configPage2.multiplyMAP == MULTIPLY_MAP_MODE_BARO) { iMAP = ((unsigned int)MAP << 7U) / currentStatus.baro; }
  
  if ( (configPage2.includeAFR == true) && (configPage6.egoType == EGO_TYPE_WIDE) && (currentStatus.runSecs > configPage6.ego_sdelay) ) {
    iAFR = ((unsigned int)currentStatus.O2 << 7U) / currentStatus.afrTarget;  //Include AFR (vs target) if enabled
  }
  if ( (configPage2.incorporateAFR == true) && (configPage2.includeAFR == false) ) {
    iAFR = ((unsigned int)configPage2.stoich << 7U) / currentStatus.afrTarget;  //Incorporate stoich vs target AFR, if enabled.
  }

  uint32_t intermediate = rshift<7U>((uint32_t)REQ_FUEL * (uint32_t)iVE); //Need to use an intermediate value to avoid overflowing the long
  if ( configPage2.multiplyMAP > 0 ) { intermediate = rshift<7U>(intermediate * (uint32_t)iMAP); }
  
  if ( (configPage2.includeAFR == true) && (configPage6.egoType == EGO_TYPE_WIDE) && (currentStatus.runSecs > configPage6.ego_sdelay) ) {
    //EGO type must be set to wideband and the AFR warmup time must've elapsed for this to be used
    intermediate = rshift<7U>(intermediate * (uint32_t)iAFR);  
  }
  if ( (configPage2.incorporateAFR == true) && (configPage2.includeAFR == false) ) {
    intermediate = rshift<7U>(intermediate * (uint32_t)iAFR);
  }

  //If corrections are huge, use less bitshift to avoid overflow. Sacrifices a bit more accuracy (basically only during very cold temp cranking)
  if (corrections < 512 ) { 
    intermediate = rshift<7U>(intermediate * div100(lshift<7U>(corrections))); 
  } else if (corrections < 1024 ) { 
    intermediate = rshift<6U>(intermediate * div100(lshift<6U>(corrections)));
  } else {
    intermediate = rshift<5U>(intermediate * div100(lshift<5U>(corrections)));
  }

  if (intermediate != 0)
  {
    //If intermediate is not 0, we need to add the opening time (0 typically indicates that one of the full fuel cuts is active)
    intermediate += injOpen; //Add the injector opening time
    //AE calculation only when ACC is active.
    if ( BIT_CHECK(currentStatus.engine, BIT_ENGINE_ACC) )
    {
      //AE Adds % of req_fuel
      if ( configPage2.aeApplyMode == AE_MODE_ADDER )
        {
          intermediate += div100(((uint32_t)REQ_FUEL) * (currentStatus.AEamount - 100U));
        }
    }

    if ( intermediate > UINT16_MAX)
    {
      intermediate = UINT16_MAX;  //Make sure this won't overflow when we convert to uInt. This means the maximum pulsewidth possible is 65.535mS
    }
  }
  return (unsigned int)(intermediate);
}

/** Lookup the current VE value from the primary 3D fuel map.
 * The Y axis value used for this lookup varies based on the fuel algorithm selected (speed density, alpha-n etc).
 * 
 * @return byte The current VE value
 */
byte getVE1(void)
{
  byte tempVE = 100;
  if (configPage2.fuelAlgorithm == LOAD_SOURCE_MAP) //Check which fuelling algorithm is being used
  {
    //Speed Density
    currentStatus.fuelLoad = currentStatus.MAP;
  }
  else if (configPage2.fuelAlgorithm == LOAD_SOURCE_TPS)
  {
    //Alpha-N
    currentStatus.fuelLoad = currentStatus.TPS * 2;
  }
  else if (configPage2.fuelAlgorithm == LOAD_SOURCE_IMAPEMAP)
  {
    //IMAP / EMAP
    currentStatus.fuelLoad = ((int16_t)currentStatus.MAP * 100U) / currentStatus.EMAP;
  }
  else { currentStatus.fuelLoad = currentStatus.MAP; } //Fallback position
  tempVE = get3DTableValue(&fuelTable, currentStatus.fuelLoad, currentStatus.RPM); //Perform lookup into fuel map for RPM vs MAP value

  return tempVE;
}

/** Lookup the ignition advance from 3D ignition table.
 * The values used to look this up will be RPM and whatever load source the user has configured.
 * 
 * @return byte The current target advance value in degrees
 */
byte getAdvance1(void)
{
  byte tempAdvance = 0;
  if (configPage2.ignAlgorithm == LOAD_SOURCE_MAP) //Check which fuelling algorithm is being used
  {
    //Speed Density
    currentStatus.ignLoad = currentStatus.MAP;
  }
  else if(configPage2.ignAlgorithm == LOAD_SOURCE_TPS)
  {
    //Alpha-N
    currentStatus.ignLoad = currentStatus.TPS * 2;

  }
  else if (configPage2.fuelAlgorithm == LOAD_SOURCE_IMAPEMAP)
  {
    //IMAP / EMAP
    currentStatus.ignLoad = ((int16_t)currentStatus.MAP * 100U) / currentStatus.EMAP;
  }
  tempAdvance = get3DTableValue(&ignitionTable, currentStatus.ignLoad, currentStatus.RPM) - OFFSET_IGNITION; //As above, but for ignition advance
  tempAdvance = correctionsIgn(tempAdvance);

  return tempAdvance;
}

/** Calculate the Ignition angles for all cylinders (based on @ref config2.nCylinders).
 * both start and end angles are calculated for each channel.
 * Also the mode of ignition firing - wasted spark vs. dedicated spark per cyl. - is considered here.
 */
void calculateIgnitionAngles(uint16_t dwellAngle)
{
  //This test for more cylinders and do the same thing
  switch (configPage2.nCylinders)
  {
    //1 cylinder
    case 1:
      calculateIgnitionAngle(dwellAngle, channel1IgnDegrees, currentStatus.advance, &ignition1EndAngle, &ignition1StartAngle);
      break;
    //2 cylinders
    case 2:
      calculateIgnitionAngle(dwellAngle, channel1IgnDegrees, currentStatus.advance, &ignition1EndAngle, &ignition1StartAngle);
      calculateIgnitionAngle(dwellAngle, channel2IgnDegrees, currentStatus.advance, &ignition2EndAngle, &ignition2StartAngle);
      break;
    //3 cylinders
    case 3:
      calculateIgnitionAngle(dwellAngle, channel1IgnDegrees, currentStatus.advance, &ignition1EndAngle, &ignition1StartAngle);
      calculateIgnitionAngle(dwellAngle, channel2IgnDegrees, currentStatus.advance, &ignition2EndAngle, &ignition2StartAngle);
      calculateIgnitionAngle(dwellAngle, channel3IgnDegrees, currentStatus.advance, &ignition3EndAngle, &ignition3StartAngle);
      break;
    //4 cylinders
    case 4:
      calculateIgnitionAngle(dwellAngle, channel1IgnDegrees, currentStatus.advance, &ignition1EndAngle, &ignition1StartAngle);
      calculateIgnitionAngle(dwellAngle, channel2IgnDegrees, currentStatus.advance, &ignition2EndAngle, &ignition2StartAngle);

      #if IGN_CHANNELS >= 4
      if((configPage4.sparkMode == IGN_MODE_SEQUENTIAL) && currentStatus.hasSync)
      {
        if( CRANK_ANGLE_MAX_IGN != 720 ) { changeHalfToFullSync(); }

        calculateIgnitionAngle(dwellAngle, channel3IgnDegrees, currentStatus.advance, &ignition3EndAngle, &ignition3StartAngle);
        calculateIgnitionAngle(dwellAngle, channel4IgnDegrees, currentStatus.advance, &ignition4EndAngle, &ignition4StartAngle);
      }
      else if(configPage4.sparkMode == IGN_MODE_ROTARY)
      {
        byte splitDegrees = 0;
        splitDegrees = table2D_getValue(&rotarySplitTable, currentStatus.ignLoad);

        //The trailing angles are set relative to the leading ones
        calculateIgnitionTrailingRotary(dwellAngle, splitDegrees, ignition1EndAngle, &ignition3EndAngle, &ignition3StartAngle);
        calculateIgnitionTrailingRotary(dwellAngle, splitDegrees, ignition2EndAngle, &ignition4EndAngle, &ignition4StartAngle);
      }
      else
      {
        if( BIT_CHECK(currentStatus.status3, BIT_STATUS3_HALFSYNC) && (CRANK_ANGLE_MAX_IGN != 360) ) { changeFullToHalfSync(); }
      }
      #endif
      break;
    //5 cylinders
    case 5:
      calculateIgnitionAngle(dwellAngle, channel1IgnDegrees, currentStatus.advance, &ignition1EndAngle, &ignition1StartAngle);
      calculateIgnitionAngle(dwellAngle, channel2IgnDegrees, currentStatus.advance, &ignition2EndAngle, &ignition2StartAngle);
      calculateIgnitionAngle(dwellAngle, channel3IgnDegrees, currentStatus.advance, &ignition3EndAngle, &ignition3StartAngle);
      calculateIgnitionAngle(dwellAngle, channel4IgnDegrees, currentStatus.advance, &ignition4EndAngle, &ignition4StartAngle);
      #if (IGN_CHANNELS >= 5)
      calculateIgnitionAngle(dwellAngle, channel5IgnDegrees, currentStatus.advance, &ignition5EndAngle, &ignition5StartAngle);
      #endif
      break;
    //6 cylinders
    case 6:
      calculateIgnitionAngle(dwellAngle, channel1IgnDegrees, currentStatus.advance, &ignition1EndAngle, &ignition1StartAngle);
      calculateIgnitionAngle(dwellAngle, channel2IgnDegrees, currentStatus.advance, &ignition2EndAngle, &ignition2StartAngle);
      calculateIgnitionAngle(dwellAngle, channel3IgnDegrees, currentStatus.advance, &ignition3EndAngle, &ignition3StartAngle);

      #if IGN_CHANNELS >= 6
      if((configPage4.sparkMode == IGN_MODE_SEQUENTIAL) && currentStatus.hasSync)
      {
        if( CRANK_ANGLE_MAX_IGN != 720 ) { changeHalfToFullSync(); }

        calculateIgnitionAngle(dwellAngle, channel4IgnDegrees, currentStatus.advance, &ignition4EndAngle, &ignition4StartAngle);
        calculateIgnitionAngle(dwellAngle, channel5IgnDegrees, currentStatus.advance, &ignition5EndAngle, &ignition5StartAngle);
        calculateIgnitionAngle(dwellAngle, channel6IgnDegrees, currentStatus.advance, &ignition6EndAngle, &ignition6StartAngle);
      }
      else
      {
        if( BIT_CHECK(currentStatus.status3, BIT_STATUS3_HALFSYNC) && (CRANK_ANGLE_MAX_IGN != 360) ) { changeFullToHalfSync(); }
      }
      #endif
      break;
    //8 cylinders
    case 8:
      calculateIgnitionAngle(dwellAngle, channel1IgnDegrees, currentStatus.advance, &ignition1EndAngle, &ignition1StartAngle);
      calculateIgnitionAngle(dwellAngle, channel2IgnDegrees, currentStatus.advance, &ignition2EndAngle, &ignition2StartAngle);
      calculateIgnitionAngle(dwellAngle, channel3IgnDegrees, currentStatus.advance, &ignition3EndAngle, &ignition3StartAngle);
      calculateIgnitionAngle(dwellAngle, channel4IgnDegrees, currentStatus.advance, &ignition4EndAngle, &ignition4StartAngle);

      #if IGN_CHANNELS >= 8
      if((configPage4.sparkMode == IGN_MODE_SEQUENTIAL) && currentStatus.hasSync)
      {
        if( CRANK_ANGLE_MAX_IGN != 720 ) { changeHalfToFullSync(); }

        calculateIgnitionAngle(dwellAngle, channel5IgnDegrees, currentStatus.advance, &ignition5EndAngle, &ignition5StartAngle);
        calculateIgnitionAngle(dwellAngle, channel6IgnDegrees, currentStatus.advance, &ignition6EndAngle, &ignition6StartAngle);
        calculateIgnitionAngle(dwellAngle, channel7IgnDegrees, currentStatus.advance, &ignition7EndAngle, &ignition7StartAngle);
        calculateIgnitionAngle(dwellAngle, channel8IgnDegrees, currentStatus.advance, &ignition8EndAngle, &ignition8StartAngle);
      }
      else
      {
        if( BIT_CHECK(currentStatus.status3, BIT_STATUS3_HALFSYNC) && (CRANK_ANGLE_MAX_IGN != 360) ) { changeFullToHalfSync(); }
      }
      #endif
      break;

    //Will hit the default case on >8 cylinders. Do nothing in these cases
    default:
      break;
  }
}

uint16_t calculatePWLimit()
{
  uint32_t tempLimit = percentage(configPage2.dutyLim, revolutionTime); //The pulsewidth limit is determined to be the duty cycle limit (Eg 85%) by the total time it takes to perform 1 revolution
  //Handle multiple squirts per rev
  if (configPage2.strokes == FOUR_STROKE) { tempLimit = tempLimit * 2; }
  //Optimise for power of two divisions where possible
  switch(currentStatus.nSquirts)
  {
    case 1:
      //No action needed
      break;
    case 2:
      tempLimit = tempLimit / 2;
      break;
    case 4:
      tempLimit = tempLimit / 4;
      break;
    case 8:
      tempLimit = tempLimit / 8;
      break;
    default:
      //Non-PoT squirts value. Perform (slow) uint32_t division
      tempLimit = tempLimit / currentStatus.nSquirts;
      break;
  }
  if(tempLimit > UINT16_MAX) { tempLimit = UINT16_MAX; }

  return tempLimit;
}

/**
这段代码用于计算阶段喷油的脉冲宽度。阶段喷油是指在发动机需要更多燃料时，通过控制主要和次要喷油嘴的脉冲宽度来实现。该函数的主要作用是根据当前的引擎参数和配置来计算主要和次要喷油嘴的脉冲宽度。

以下是该函数的主要逻辑：

首先检查阶段喷油是否启用，并且检查当前的喷油器通道数是否足够支持阶段喷油，以及当前的脉冲宽度是否大于喷油嘴的开启时间。

如果满足阶段喷油的条件，根据阶段喷油模式（table模式或auto模式）来计算主要和次要喷油嘴的脉冲宽度，并根据当前的喷油嘴通道数分配主要和次要喷油嘴的脉冲宽度。

如果不满足阶段喷油的条件，则将所有的喷油嘴的脉冲宽度都设置为当前的脉冲宽度，并清除阶段喷油激活标志。

这段代码的主要作用是根据当前的引擎参数和配置来计算主要和次要喷油嘴的脉冲宽度，并根据需要启用阶段喷油。
*/
void calculateStaging(uint32_t pwLimit)
{
  //Calculate staging pulsewidths if used
  //To run staged injection, the number of cylinders must be less than or equal to the injector channels (ie Assuming you're running paired injection, you need at least as many injector channels as you have cylinders, half for the primaries and half for the secondaries)
  if( (configPage10.stagingEnabled == true) && (configPage2.nCylinders <= INJ_CHANNELS || configPage2.injType == INJ_TYPE_TBODY) && (currentStatus.PW1 > inj_opentime_uS) ) //Final check is to ensure that DFCO isn't active, which would cause an overflow below (See #267)
  {
    //Scale the 'full' pulsewidth by each of the injector capacities
    currentStatus.PW1 -= inj_opentime_uS; //Subtract the opening time from PW1 as it needs to be multiplied out again by the pri/sec req_fuel values below. It is added on again after that calculation. 
    uint32_t tempPW1 = div100((uint32_t)currentStatus.PW1 * staged_req_fuel_mult_pri);

    if(configPage10.stagingMode == STAGING_MODE_TABLE)
    {
      uint32_t tempPW3 = div100((uint32_t)currentStatus.PW1 * staged_req_fuel_mult_sec); //This is ONLY needed in in table mode. Auto mode only calculates the difference.

      uint8_t stagingSplit = get3DTableValue(&stagingTable, currentStatus.fuelLoad, currentStatus.RPM);
      currentStatus.PW1 = div100((100U - stagingSplit) * tempPW1);
      currentStatus.PW1 += inj_opentime_uS; 

      //PW2 is used temporarily to hold the secondary injector pulsewidth. It will be assigned to the correct channel below
      if(stagingSplit > 0) 
      { 
        BIT_SET(currentStatus.status4, BIT_STATUS4_STAGING_ACTIVE); //Set the staging active flag
        currentStatus.PW2 = div100(stagingSplit * tempPW3); 
        currentStatus.PW2 += inj_opentime_uS;
      }
      else
      {
        BIT_CLEAR(currentStatus.status4, BIT_STATUS4_STAGING_ACTIVE); //Clear the staging active flag
        currentStatus.PW2 = 0; 
      }
    }
    else if(configPage10.stagingMode == STAGING_MODE_AUTO)
    {
      currentStatus.PW1 = tempPW1;
      //If automatic mode, the primary injectors are used all the way up to their limit (Configured by the pulsewidth limit setting)
      //If they exceed their limit, the extra duty is passed to the secondaries
      if(tempPW1 > pwLimit)
      {
        BIT_SET(currentStatus.status4, BIT_STATUS4_STAGING_ACTIVE); //Set the staging active flag
        uint32_t extraPW = tempPW1 - pwLimit + inj_opentime_uS; //The open time must be added here AND below because tempPW1 does not include an open time. The addition of it here takes into account the fact that pwLlimit does not contain an allowance for an open time. 
        currentStatus.PW1 = pwLimit;
        currentStatus.PW2 = udiv_32_16(extraPW * staged_req_fuel_mult_sec, staged_req_fuel_mult_pri); //Convert the 'left over' fuel amount from primary injector scaling to secondary
        currentStatus.PW2 += inj_opentime_uS;
      }
      else 
      {
        //If tempPW1 < pwLImit it means that the entire fuel load can be handled by the primaries and staging is inactive. 
        currentStatus.PW1 += inj_opentime_uS; //Add the open time back in
        BIT_CLEAR(currentStatus.status4, BIT_STATUS4_STAGING_ACTIVE); //Clear the staging active flag 
        currentStatus.PW2 = 0; //Secondary PW is simply set to 0 as it is not required
      } 
    }

    //Allocate the primary and secondary pulse widths based on the fuel configuration
    switch (configPage2.nCylinders) 
    {
      case 1:
        //Nothing required for 1 cylinder, channels are correct already
        break;
      case 2:
        //Primary pulsewidth on channels 1 and 2, secondary on channels 3 and 4
        currentStatus.PW3 = currentStatus.PW2;
        currentStatus.PW4 = currentStatus.PW2;
        currentStatus.PW2 = currentStatus.PW1;
        break;
      case 3:
        //6 channels required for 'normal' 3 cylinder staging support
        #if INJ_CHANNELS >= 6
          //Primary pulsewidth on channels 1, 2 and 3, secondary on channels 4, 5 and 6
          currentStatus.PW4 = currentStatus.PW2;
          currentStatus.PW5 = currentStatus.PW2;
          currentStatus.PW6 = currentStatus.PW2;
        #else
          //If there are not enough channels, then primary pulsewidth is on channels 1, 2 and 3, secondary on channel 4
          currentStatus.PW4 = currentStatus.PW2;
        #endif
        currentStatus.PW2 = currentStatus.PW1;
        currentStatus.PW3 = currentStatus.PW1;
        break;
      case 4:
        if( (configPage2.injLayout == INJ_SEQUENTIAL) || (configPage2.injLayout == INJ_SEMISEQUENTIAL) )
        {
          //Staging with 4 cylinders semi/sequential requires 8 total channels
          #if INJ_CHANNELS >= 8
            currentStatus.PW5 = currentStatus.PW2;
            currentStatus.PW6 = currentStatus.PW2;
            currentStatus.PW7 = currentStatus.PW2;
            currentStatus.PW8 = currentStatus.PW2;

            currentStatus.PW2 = currentStatus.PW1;
            currentStatus.PW3 = currentStatus.PW1;
            currentStatus.PW4 = currentStatus.PW1;
          #else
            //This is an invalid config as there are not enough outputs to support sequential + staging
            //Put the staging output to the non-existent channel 5
            currentStatus.PW5 = currentStatus.PW2;
          #endif
        }
        else
        {
          currentStatus.PW3 = currentStatus.PW2;
          currentStatus.PW4 = currentStatus.PW2;
          currentStatus.PW2 = currentStatus.PW1;
        }
        break;
        
      case 5:
        //No easily supportable 5 cylinder staging option unless there are at least 5 channels
        #if INJ_CHANNELS >= 5
          if (configPage2.injLayout != INJ_SEQUENTIAL)
          {
            currentStatus.PW5 = currentStatus.PW2;
          }
          #if INJ_CHANNELS >= 6
            currentStatus.PW6 = currentStatus.PW2;
          #endif
        #endif
        
          currentStatus.PW2 = currentStatus.PW1;
          currentStatus.PW3 = currentStatus.PW1;
          currentStatus.PW4 = currentStatus.PW1;
        break;

      case 6:
        #if INJ_CHANNELS >= 6
          //8 cylinder staging only if not sequential
          if (configPage2.injLayout != INJ_SEQUENTIAL)
          {
            currentStatus.PW4 = currentStatus.PW2;
            currentStatus.PW5 = currentStatus.PW2;
            currentStatus.PW6 = currentStatus.PW2;
          }
          #if INJ_CHANNELS >= 8
          else
            {
              //If there are 8 channels, then the 6 cylinder sequential option is available by using channels 7 + 8 for staging
              currentStatus.PW7 = currentStatus.PW2;
              currentStatus.PW8 = currentStatus.PW2;

              currentStatus.PW4 = currentStatus.PW1;
              currentStatus.PW5 = currentStatus.PW1;
              currentStatus.PW6 = currentStatus.PW1;
            }
          #endif
        #endif
        currentStatus.PW2 = currentStatus.PW1;
        currentStatus.PW3 = currentStatus.PW1;
        break;

      case 8:
        #if INJ_CHANNELS >= 8
          //8 cylinder staging only if not sequential
          if (configPage2.injLayout != INJ_SEQUENTIAL)
          {
            currentStatus.PW5 = currentStatus.PW2;
            currentStatus.PW6 = currentStatus.PW2;
            currentStatus.PW7 = currentStatus.PW2;
            currentStatus.PW8 = currentStatus.PW2;
          }
        #endif
        currentStatus.PW2 = currentStatus.PW1;
        currentStatus.PW3 = currentStatus.PW1;
        currentStatus.PW4 = currentStatus.PW1;
        break;

      default:
        //Assume 4 cylinder non-seq for default
        currentStatus.PW3 = currentStatus.PW2;
        currentStatus.PW4 = currentStatus.PW2;
        currentStatus.PW2 = currentStatus.PW1;
        break;
    }
  }
  else 
  { 
    if(maxInjOutputs >= 2) { currentStatus.PW2 = currentStatus.PW1; }
    else { currentStatus.PW2 = 0; }
    if(maxInjOutputs >= 3) { currentStatus.PW3 = currentStatus.PW1; }
    else { currentStatus.PW3 = 0; }
    if(maxInjOutputs >= 4) { currentStatus.PW4 = currentStatus.PW1; }
    else { currentStatus.PW4 = 0; }
    if(maxInjOutputs >= 5) { currentStatus.PW5 = currentStatus.PW1; }
    else { currentStatus.PW5 = 0; }
    if(maxInjOutputs >= 6) { currentStatus.PW6 = currentStatus.PW1; }
    else { currentStatus.PW6 = 0; }
    if(maxInjOutputs >= 7) { currentStatus.PW7 = currentStatus.PW1; }
    else { currentStatus.PW7 = 0; }
    if(maxInjOutputs >= 8) { currentStatus.PW8 = currentStatus.PW1; }
    else { currentStatus.PW8 = 0; }

    BIT_CLEAR(currentStatus.status4, BIT_STATUS4_STAGING_ACTIVE); //Clear the staging active flag
    
  } 

}

/**
这段代码用于检查启动和换挡时的状态。主要功能如下：
存储当前和上一个离合器状态。
如果启动或换挡功能已启用，则根据配置检查离合器引脚状态，并更新当前的离合器触发状态。如果离合器状态从未触发到触发状态，则存储当前的引擎转速。
初始化启动和换挡的标志为假，同时清除任何现有的启动或换挡标志。
如果启动功能已启用，并且离合器触发且当前离合器触发引擎转速低于启动控制转速，并且当前节气门位置超过了配置的启动控制节气门位置阈值，则：
检查当前转速是否高于启动限制转速。
如果当前转速高于启动限制转速，则将启动硬切标志设置为真，并设置启动火花标志。
如果启动功能未启用，则检查平顶功能是否已启用，并且离合器触发且当前离合器触发引擎转速高于等于配置的平顶控制转速，并且当前引擎转速高于平顶控制转速限制，则：
检查当前转速是否高于平顶限制转速。
如果当前转速高于平顶限制转速，则将换挡硬切标志设置为真。
这段代码的作用是根据配置检查启动和换挡时的状态，并根据条件设置相应的标志。
*/
void checkLaunchAndFlatShift()
{
  //Check for launching/flat shift (clutch) based on the current and previous clutch states
  currentStatus.previousClutchTrigger = currentStatus.clutchTrigger;
  //Only check for pinLaunch if any function using it is enabled. Else pins might break starting a board
  if(configPage6.flatSEnable || configPage6.launchEnabled)
  {
    if(configPage6.launchHiLo > 0) { currentStatus.clutchTrigger = digitalRead(pinLaunch); }
    else { currentStatus.clutchTrigger = !digitalRead(pinLaunch); }
  }
  if(currentStatus.clutchTrigger && (currentStatus.previousClutchTrigger != currentStatus.clutchTrigger) ) { currentStatus.clutchEngagedRPM = currentStatus.RPM; } //Check whether the clutch has been engaged or disengaged and store the current RPM if so

  //Default flags to off
  currentStatus.launchingHard = false; 
  BIT_CLEAR(currentStatus.spark, BIT_SPARK_HLAUNCH); 
  currentStatus.flatShiftingHard = false;

  if (configPage6.launchEnabled && currentStatus.clutchTrigger && (currentStatus.clutchEngagedRPM < ((unsigned int)(configPage6.flatSArm) * 100)) && (currentStatus.TPS >= configPage10.lnchCtrlTPS) ) 
  { 
    //Check whether RPM is above the launch limit
    uint16_t launchRPMLimit = (configPage6.lnchHardLim * 100);
    if( (configPage2.hardCutType == HARD_CUT_ROLLING) ) { launchRPMLimit += (configPage15.rollingProtRPMDelta[0] * 10); } //Add the rolling cut delta if enabled (Delta is a negative value)

    if(currentStatus.RPM > launchRPMLimit)
    {
      //HardCut rev limit for 2-step launch control.
      currentStatus.launchingHard = true; 
      BIT_SET(currentStatus.spark, BIT_SPARK_HLAUNCH); 
    }
  } 
  else 
  { 
    //If launch is not active, check whether flat shift should be active
    if(configPage6.flatSEnable && currentStatus.clutchTrigger && (currentStatus.clutchEngagedRPM >= ((unsigned int)(configPage6.flatSArm * 100)) ) ) 
    { 
      uint16_t flatRPMLimit = currentStatus.clutchEngagedRPM;
      if( (configPage2.hardCutType == HARD_CUT_ROLLING) ) { flatRPMLimit += (configPage15.rollingProtRPMDelta[0] * 10); } //Add the rolling cut delta if enabled (Delta is a negative value)

      if(currentStatus.RPM > flatRPMLimit)
      {
        //Flat shift rev limit
        currentStatus.flatShiftingHard = true;
      }
    }
  }
}
