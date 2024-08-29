/** \file speeduino.h
 * @brief Speeduino main file containing initial setup and system loop functions 
 * @author Josh Stewart
 * 
 * This file contains the main system loop of the Speeduino core and thus much of the logic of the fuel and ignition algorithms is contained within this
 * It is where calls to all the auxiliary control systems, sensor reads, comms etc are made
 * 
 * It also contains the setup() function that is called by the bootloader on system startup
 * 
 */

#ifndef SPEEDUINO_H
#define SPEEDUINO_H
//#include "globals.h"

#define CRANK_RUN_HYSTER    15

void setup(void);
void loop(void);
uint16_t PW(int REQ_FUEL, byte VE, long MAP, uint16_t corrections, int injOpen);
byte getVE1(void);
byte getAdvance1(void);
uint16_t calculatePWLimit();
void calculateStaging(uint32_t);
void calculateIgnitionAngles(uint16_t dwellAngle);
void checkLaunchAndFlatShift();

extern uint16_t req_fuel_uS; /**< The required fuel variable (As calculated by TunerStudio) in uS */
extern uint16_t inj_opentime_uS; /**< The injector opening time. This is set within Tuner Studio, but stored here in uS rather than mS */

/** @name Staging
 * These values are a percentage of the total (Combined) req_fuel value that would be required for each injector channel to deliver that much fuel.   
 * 
 * Eg:
 *  - Pri injectors are 250cc
 *  - Sec injectors are 500cc
 *  - Total injector capacity = 750cc
 * 
 *  - staged_req_fuel_mult_pri = 300% (The primary injectors would have to run 3x the overall PW in order to be the equivalent of the full 750cc capacity
 *  - staged_req_fuel_mult_sec = 150% (The secondary injectors would have to run 1.5x the overall PW in order to be the equivalent of the full 750cc capacity
*/
///@{
extern uint16_t staged_req_fuel_mult_pri;
extern uint16_t staged_req_fuel_mult_sec;
///@}

/**
Speeduino 的代码实际上是围绕 Arduino 的引脚功能进行编程的，以下是详细的解释：

1. 引脚功能
定义引脚功能：在 Speeduino 项目中，Arduino 的引脚被定义为不同的功能，例如输入信号（传感器数据）、输出信号（控制执行器）等。代码中通常会指定哪些引脚用于哪些具体功能，比如喷油器、点火器等。

设置引脚模式：代码中会设置引脚的工作模式，例如输入模式或输出模式。这确保了 Arduino 可以正确地接收数据或发送控制信号。

2. 数据处理
传感器数据：Arduino 接收来自传感器的输入数据，这些数据可能包括发动机转速、温度、油门位置等。这些数据通过定义好的引脚（通常是输入引脚）传输到 Arduino。

计算和处理：Arduino 上的代码处理这些传感器数据，根据预设的算法（如燃油映射、点火映射等）计算出需要发送给执行器的数据。例如，计算喷油量、点火时机等。

3. 输出控制
发送控制信号：经过计算处理的数据会被发送到定义的输出引脚上。这些输出信号会控制发动机的执行器，例如喷油器和点火线圈。Arduino 通过设置引脚的电平（高或低）或通过 SPI/I2C 等通信协议将信号发送到 MC33810 芯片，然后 MC33810 将这些信号转换为实际的执行控制。
4. 接口与集成
接口模块：在 Speeduino 项目中，MC33810 芯片作为一个接口模块，与 Arduino 通过低电压线连接。MC33810 接收来自 Arduino 的控制信号，并驱动实际的喷油器和点火器。Arduino 的代码不直接控制喷油器和点火器，而是通过 MC33810 等驱动模块来间接控制。
总结
引脚功能编程：Speeduino 的代码主要关注于如何利用 Arduino 的引脚功能来完成特定的控制任务。通过设置引脚的功能和模式，读取传感器数据，计算所需的输出，然后将这些输出发送到相应的引脚上。
目标数据：Arduino 计算出目标数据，并通过设置引脚的电平或通信协议将这些数据发送到 MC33810 芯片，后者负责将信号转换为实际的控制动作。
这种方法使得 Speeduino 能够灵活地处理各种输入和输出，满足发动机控制系统的需求。
**/


#endif
