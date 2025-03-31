/*
Speeduino - Simple engine management for the Arduino Mega 2560 platform
Copyright (C) Josh Stewart
A full copy of the license may be found in the projects root directory
*/
/** @file
 * Injector and Ignition (on/off) scheduling (functions).
 * There is usually 8 functions for cylinders 1-8 with same naming pattern.
 * 
 * ## Scheduling structures
 * 
 * Structures @ref FuelSchedule and @ref Schedule describe (from scheduler.h) describe the scheduling info for Fuel and Ignition respectively.
 * They contain duration, current activity status, start timing, end timing, callbacks to carry out action, etc.
 * 
 * ## Scheduling Functions
 * 
 * For Injection:
 * - setFuelSchedule*(tout,dur) - **Setup** schedule for (next) injection on the channel
 * - inj*StartFunction() - Execute **start** of injection (Interrupt handler)
 * - inj*EndFunction() - Execute **end** of injection (interrupt handler)
 * 
 * For Ignition (has more complex schedule setup):
 * - setIgnitionSchedule*(cb_st,tout,dur,cb_end) - **Setup** schedule for (next) ignition on the channel
 * - ign*StartFunction() - Execute **start** of ignition (Interrupt handler)
 * - ign*EndFunction() - Execute **end** of ignition (Interrupt handler)
 */
#include "globals.h"
#include "scheduler.h"
#include "scheduledIO.h"
#include "timers.h"
#include "schedule_calcs.h"

FuelSchedule fuelSchedule1(FUEL1_COUNTER, FUEL1_COMPARE, FUEL1_TIMER_DISABLE, FUEL1_TIMER_ENABLE);
FuelSchedule fuelSchedule2(FUEL2_COUNTER, FUEL2_COMPARE, FUEL2_TIMER_DISABLE, FUEL2_TIMER_ENABLE);
FuelSchedule fuelSchedule3(FUEL3_COUNTER, FUEL3_COMPARE, FUEL3_TIMER_DISABLE, FUEL3_TIMER_ENABLE);
FuelSchedule fuelSchedule4(FUEL4_COUNTER, FUEL4_COMPARE, FUEL4_TIMER_DISABLE, FUEL4_TIMER_ENABLE);

#if (INJ_CHANNELS >= 5)
FuelSchedule fuelSchedule5(FUEL5_COUNTER, FUEL5_COMPARE, FUEL5_TIMER_DISABLE, FUEL5_TIMER_ENABLE);
#endif
#if (INJ_CHANNELS >= 6)
FuelSchedule fuelSchedule6(FUEL6_COUNTER, FUEL6_COMPARE, FUEL6_TIMER_DISABLE, FUEL6_TIMER_ENABLE);
#endif
#if (INJ_CHANNELS >= 7)
FuelSchedule fuelSchedule7(FUEL7_COUNTER, FUEL7_COMPARE, FUEL7_TIMER_DISABLE, FUEL7_TIMER_ENABLE);
#endif
#if (INJ_CHANNELS >= 8)
FuelSchedule fuelSchedule8(FUEL8_COUNTER, FUEL8_COMPARE, FUEL8_TIMER_DISABLE, FUEL8_TIMER_ENABLE);
#endif

// 初始化第一个点火调度对象，绑定第一个定时器的计数器、比较寄存器以及定时器启用和禁用函数
IgnitionSchedule ignitionSchedule1(IGN1_COUNTER, IGN1_COMPARE, IGN1_TIMER_DISABLE, IGN1_TIMER_ENABLE);

// 初始化第二个点火调度对象，绑定第二个定时器的计数器、比较寄存器以及定时器启用和禁用函数
IgnitionSchedule ignitionSchedule2(IGN2_COUNTER, IGN2_COMPARE, IGN2_TIMER_DISABLE, IGN2_TIMER_ENABLE);

// 初始化第三个点火调度对象，绑定第三个定时器的计数器、比较寄存器以及定时器启用和禁用函数
IgnitionSchedule ignitionSchedule3(IGN3_COUNTER, IGN3_COMPARE, IGN3_TIMER_DISABLE, IGN3_TIMER_ENABLE);

// 初始化第四个点火调度对象，绑定第四个定时器的计数器、比较寄存器以及定时器启用和禁用函数
IgnitionSchedule ignitionSchedule4(IGN4_COUNTER, IGN4_COMPARE, IGN4_TIMER_DISABLE, IGN4_TIMER_ENABLE);

// 初始化第五个点火调度对象，绑定第五个定时器的计数器、比较寄存器以及定时器启用和禁用函数
IgnitionSchedule ignitionSchedule5(IGN5_COUNTER, IGN5_COMPARE, IGN5_TIMER_DISABLE, IGN5_TIMER_ENABLE);

#if IGN_CHANNELS >= 6
IgnitionSchedule ignitionSchedule6(IGN6_COUNTER, IGN6_COMPARE, IGN6_TIMER_DISABLE, IGN6_TIMER_ENABLE);
#endif
#if IGN_CHANNELS >= 7
IgnitionSchedule ignitionSchedule7(IGN7_COUNTER, IGN7_COMPARE, IGN7_TIMER_DISABLE, IGN7_TIMER_ENABLE);
#endif
#if IGN_CHANNELS >= 8
IgnitionSchedule ignitionSchedule8(IGN8_COUNTER, IGN8_COMPARE, IGN8_TIMER_DISABLE, IGN8_TIMER_ENABLE);
#endif

static void reset(FuelSchedule &schedule) 
{
    schedule.Status = OFF;
    schedule.pTimerEnable();
}

static void reset(IgnitionSchedule &schedule) 
{
    schedule.Status = OFF;
    schedule.pTimerEnable();
}

void initialiseSchedulers()
{
    reset(fuelSchedule1);
    reset(fuelSchedule2);
    reset(fuelSchedule3);
    reset(fuelSchedule4);
#if INJ_CHANNELS >= 5
    reset(fuelSchedule5);
#endif
#if INJ_CHANNELS >= 6
    reset(fuelSchedule6);
#endif
#if INJ_CHANNELS >= 7
    reset(fuelSchedule7);
#endif
#if INJ_CHANNELS >= 8
    reset(fuelSchedule8);
#endif

    reset(ignitionSchedule1);
    reset(ignitionSchedule2);
    reset(ignitionSchedule3);
    reset(ignitionSchedule4);
    reset(ignitionSchedule5);
#if (IGN_CHANNELS >= 5)
    reset(ignitionSchedule5);
#endif
#if IGN_CHANNELS >= 6
    reset(ignitionSchedule6);
#endif
#if IGN_CHANNELS >= 7
    reset(ignitionSchedule7);
#endif
#if IGN_CHANNELS >= 8
    reset(ignitionSchedule8);
#endif

  fuelSchedule1.pStartFunction = nullCallback;
  fuelSchedule1.pEndFunction = nullCallback;
  fuelSchedule2.pStartFunction = nullCallback;
  fuelSchedule2.pEndFunction = nullCallback;
  fuelSchedule3.pStartFunction = nullCallback;
  fuelSchedule3.pEndFunction = nullCallback;
  fuelSchedule4.pStartFunction = nullCallback;
  fuelSchedule4.pEndFunction = nullCallback;
#if (INJ_CHANNELS >= 5)  
  fuelSchedule5.pStartFunction = nullCallback;
  fuelSchedule5.pEndFunction = nullCallback;
#endif
#if (INJ_CHANNELS >= 6)
  fuelSchedule6.pStartFunction = nullCallback;
  fuelSchedule6.pEndFunction = nullCallback;
#endif
#if (INJ_CHANNELS >= 7)
  fuelSchedule7.pStartFunction = nullCallback;
  fuelSchedule7.pEndFunction = nullCallback;
#endif
#if (INJ_CHANNELS >= 8)
  fuelSchedule8.pStartFunction = nullCallback;
  fuelSchedule8.pEndFunction = nullCallback;
#endif

  ignitionSchedule1.pStartCallback = nullCallback;
  ignitionSchedule1.pEndCallback = nullCallback;
  ignition1StartAngle=0;
  ignition1EndAngle=0;
  channel1IgnDegrees=0; /**< The number of crank degrees until cylinder 1 is at TDC (This is obviously 0 for virtually ALL engines, but there's some weird ones) */

  ignitionSchedule2.pStartCallback = nullCallback;
  ignitionSchedule2.pEndCallback = nullCallback;
  ignition2StartAngle=0;
  ignition2EndAngle=0;
  channel2IgnDegrees=0; /**< The number of crank degrees until cylinder 2 (and 5/6/7/8) is at TDC */

  ignitionSchedule3.pStartCallback = nullCallback;
  ignitionSchedule3.pEndCallback = nullCallback;
  ignition3StartAngle=0;
  ignition3EndAngle=0;
  channel3IgnDegrees=0; /**< The number of crank degrees until cylinder 2 (and 5/6/7/8) is at TDC */

  ignitionSchedule4.pStartCallback = nullCallback;
  ignitionSchedule4.pEndCallback = nullCallback;
  ignition4StartAngle=0;
  ignition4EndAngle=0;
  channel4IgnDegrees=0; /**< The number of crank degrees until cylinder 2 (and 5/6/7/8) is at TDC */

#if (IGN_CHANNELS >= 5)
  ignitionSchedule5.pStartCallback = nullCallback;
  ignitionSchedule5.pEndCallback = nullCallback;
  ignition5StartAngle=0;
  ignition5EndAngle=0;
  channel5IgnDegrees=0; /**< The number of crank degrees until cylinder 2 (and 5/6/7/8) is at TDC */
#endif
#if (IGN_CHANNELS >= 6)
  ignitionSchedule6.pStartCallback = nullCallback;
  ignitionSchedule6.pEndCallback = nullCallback;
  ignition6StartAngle=0;
  ignition6EndAngle=0;
  channel6IgnDegrees=0; /**< The number of crank degrees until cylinder 2 (and 5/6/7/8) is at TDC */
#endif
#if (IGN_CHANNELS >= 7)
  ignitionSchedule7.pStartCallback = nullCallback;
  ignitionSchedule7.pEndCallback = nullCallback;
  ignition7StartAngle=0;
  ignition7EndAngle=0;
  channel7IgnDegrees=0; /**< The number of crank degrees until cylinder 2 (and 5/6/7/8) is at TDC */
#endif
#if (IGN_CHANNELS >= 8)
  ignitionSchedule8.pStartCallback = nullCallback;
  ignitionSchedule8.pEndCallback = nullCallback;
  ignition8StartAngle=0;
  ignition8EndAngle=0;
  channel8IgnDegrees=0; /**< The number of crank degrees until cylinder 2 (and 5/6/7/8) is at TDC */
#endif

	channel1InjDegrees = 0; /**< The number of crank degrees until cylinder 1 is at TDC (This is obviously 0 for virtually ALL engines, but there's some weird ones) */
	channel2InjDegrees = 0; /**< The number of crank degrees until cylinder 2 (and 5/6/7/8) is at TDC */
	channel3InjDegrees = 0; /**< The number of crank degrees until cylinder 3 (and 5/6/7/8) is at TDC */
	channel4InjDegrees = 0; /**< The number of crank degrees until cylinder 4 (and 5/6/7/8) is at TDC */
#if (INJ_CHANNELS >= 5)
	channel5InjDegrees = 0; /**< The number of crank degrees until cylinder 5 is at TDC */
#endif
#if (INJ_CHANNELS >= 6)
	channel6InjDegrees = 0; /**< The number of crank degrees until cylinder 6 is at TDC */
#endif
#if (INJ_CHANNELS >= 7)
	channel7InjDegrees = 0; /**< The number of crank degrees until cylinder 7 is at TDC */
#endif
#if (INJ_CHANNELS >= 8)
	channel8InjDegrees = 0; /**< The number of crank degrees until cylinder 8 is at TDC */
#endif

}

void _setFuelScheduleRunning(FuelSchedule &schedule, unsigned long timeout, unsigned long duration)
{
  schedule.duration = duration;

  //Need to check that the timeout doesn't exceed the overflow
  COMPARE_TYPE timeout_timer_compare;
  if (timeout > MAX_TIMER_PERIOD) { timeout_timer_compare = uS_TO_TIMER_COMPARE( (MAX_TIMER_PERIOD - 1) ); } // If the timeout is >4x (Each tick represents 4uS on a mega2560, other boards will be different) the maximum allowed value of unsigned int (65535), the timer compare value will overflow when applied causing erratic behaviour such as erroneous squirts
  else { timeout_timer_compare = uS_TO_TIMER_COMPARE(timeout); } //Normal case

  //The following must be enclosed in the noInterupts block to avoid contention caused if the relevant interrupt fires before the state is fully set
  noInterrupts();
  schedule.startCompare = schedule.counter + timeout_timer_compare;
  schedule.endCompare = schedule.startCompare + uS_TO_TIMER_COMPARE(duration);
  SET_COMPARE(schedule.compare, schedule.startCompare); //Use the B compare unit of timer 3
  schedule.Status = PENDING; //Turn this schedule on
  interrupts();
  schedule.pTimerEnable();
}

void _setFuelScheduleNext(FuelSchedule &schedule, unsigned long timeout, unsigned long duration)
{
  //If the schedule is already running, we can set the next schedule so it is ready to go
  //This is required in cases of high rpm and high DC where there otherwise would not be enough time to set the schedule
  schedule.nextStartCompare = schedule.counter + uS_TO_TIMER_COMPARE(timeout);
  schedule.nextEndCompare = schedule.nextStartCompare + uS_TO_TIMER_COMPARE(duration);
  schedule.hasNextSchedule = true;
}

// 设置点火调度为 "运行" 状态
void _setIgnitionScheduleRunning(IgnitionSchedule &schedule, unsigned long timeout, unsigned long duration)
{
  // 设置调度的持续时间
  schedule.duration = duration;

  // 需要检查超时是否超过溢出限制
  COMPARE_TYPE timeout_timer_compare;

  // 如果超时超过最大定时器周期，则调整超时值
  if (timeout > MAX_TIMER_PERIOD) {
    // 如果超时超过最大定时器周期（每个时钟周期代表 4 微秒，最大允许值为 65535），
    // 该值将导致定时器溢出，可能会引发不正常的行为（如错误的点火）。
    timeout_timer_compare = uS_TO_TIMER_COMPARE((MAX_TIMER_PERIOD - 1)); // 设置为最大允许值
  }
  else {
    // 否则，直接将超时转换为定时器比较值
    timeout_timer_compare = uS_TO_TIMER_COMPARE(timeout); // 正常情况
  }

  // 禁止中断，进行时间相关的操作，防止中断引发不必要的冲突
  noInterrupts();

  // 计算调度的开始比较值
  schedule.startCompare = schedule.counter + timeout_timer_compare;
  // 因为每个时钟周期代表 4 微秒，超时应该是 timeout/4 个时钟周期后触发
  // 例如：如果超时为 4 微秒，超时会在 1 个时钟周期后触发（>>2 即除以 4）。

  // 如果未由解码器设置调度结束时间，则根据持续时间来设置结束比较值
  if(schedule.endScheduleSetByDecoder == false) {
    schedule.endCompare = schedule.startCompare + uS_TO_TIMER_COMPARE(duration);
  }
  // 调用定时器设置函数，设置定时器比较值，触发点火调度 ISR(TIMER3_COMPA_vect)  schedule.startCompare 都是特定类型的变量
  SET_COMPARE(schedule.compare, schedule.startCompare);

  // 设置调度状态为 "PENDING"，表示调度已启动，等待执行
  schedule.Status = PENDING;

  // 恢复中断
  interrupts();

  // 启动定时器（此函数可能是启用硬件定时器）
  schedule.pTimerEnable();
}

void _setIgnitionScheduleNext(IgnitionSchedule &schedule, unsigned long timeout, unsigned long duration)
{
  //If the schedule is already running, we can set the next schedule so it is ready to go
  //This is required in cases of high rpm and high DC where there otherwise would not be enough time to set the schedule
  schedule.nextStartCompare = schedule.counter + uS_TO_TIMER_COMPARE(timeout);
  schedule.nextEndCompare = schedule.nextStartCompare + uS_TO_TIMER_COMPARE(duration);
  schedule.hasNextSchedule = true;
}


void refreshIgnitionSchedule1(unsigned long timeToEnd)
{
  if( (ignitionSchedule1.Status == RUNNING) && (timeToEnd < ignitionSchedule1.duration) )
  //Must have the threshold check here otherwise it can cause a condition where the compare fires twice, once after the other, both for the end
  //if( (timeToEnd < ignitionSchedule1.duration) && (timeToEnd > IGNITION_REFRESH_THRESHOLD) )
  {
    noInterrupts();
    ignitionSchedule1.endCompare = IGN1_COUNTER + uS_TO_TIMER_COMPARE(timeToEnd);
    SET_COMPARE(IGN1_COMPARE, ignitionSchedule1.endCompare);
    interrupts();
  }
}

/** Perform the injector priming pulses.
 * Set these to run at an arbitrary time in the future (100us).
 * The prime pulse value is in ms*10, so need to multiple by 100 to get to uS
 */
extern void beginInjectorPriming(void)
{
  unsigned long primingValue = table2D_getValue(&PrimingPulseTable, currentStatus.coolant + CALIBRATION_TEMPERATURE_OFFSET);
  if( (primingValue > 0) && (currentStatus.TPS < configPage4.floodClear) )
  {
    primingValue = primingValue * 100 * 5; //to achieve long enough priming pulses, the values in tuner studio are divided by 0.5 instead of 0.1, so multiplier of 5 is required.
    if ( maxInjOutputs >= 1 ) { setFuelSchedule(fuelSchedule1, 100, primingValue); }
#if (INJ_CHANNELS >= 2)
    if ( maxInjOutputs >= 2 ) { setFuelSchedule(fuelSchedule2, 100, primingValue); }
#endif
#if (INJ_CHANNELS >= 3)
    if ( maxInjOutputs >= 3 ) { setFuelSchedule(fuelSchedule3, 100, primingValue); }
#endif
#if (INJ_CHANNELS >= 4)
    if ( maxInjOutputs >= 4 ) { setFuelSchedule(fuelSchedule4, 100, primingValue); }
#endif
#if (INJ_CHANNELS >= 5)
    if ( maxInjOutputs >= 5 ) { setFuelSchedule(fuelSchedule5, 100, primingValue); }
#endif
#if (INJ_CHANNELS >= 6)
    if ( maxInjOutputs >= 6 ) { setFuelSchedule(fuelSchedule6, 100, primingValue); }
#endif
#if (INJ_CHANNELS >= 7)
    if ( maxInjOutputs >= 7) { setFuelSchedule(fuelSchedule7, 100, primingValue); }
#endif
#if (INJ_CHANNELS >= 8)
    if ( maxInjOutputs >= 8 ) { setFuelSchedule(fuelSchedule8, 100, primingValue); }
#endif
  }
}

// Shared ISR function for all fuel timers.
// This is completely inlined into the ISR - there is no function call
// overhead.
// 共享的定时器中断服务程序 (ISR)，用于处理所有燃油定时器。
// 此函数完全内联到 ISR 中，避免了函数调用的开销。
static inline __attribute__((always_inline)) void fuelScheduleISR(FuelSchedule &schedule)
{
  // 检查当前调度是否为 PENDING（待启动）状态
  if (schedule.Status == PENDING)
  {
    /**
              // 配对喷射，同时操作两个喷油器
              fuelSchedule1.pStartFunction = openInjector1;
              fuelSchedule1.pEndFunction = closeInjector1;
              fuelSchedule2.pStartFunction = openInjector2;
              fuelSchedule2.pEndFunction = closeInjector2;
              fuelSchedule3.pStartFunction = openInjector3;
              fuelSchedule3.pEndFunction = closeInjector3;
              fuelSchedule4.pStartFunction = openInjector4;
              fuelSchedule4.pEndFunction = closeInjector4;
    */
    // 调用调度的开始函数，启动该调度
    schedule.pStartFunction();

    // 将调度状态更改为 RUNNING（进行中），表示开始回调已执行，但结束回调尚未执行
    schedule.Status = RUNNING;

    // 设置定时器比较值，防止在重新启动时溢出
    // 计算调度的持续时间并设置比较值
    SET_COMPARE(schedule.compare, schedule.counter + uS_TO_TIMER_COMPARE(schedule.duration) );
  }
  // 如果调度状态是 RUNNING（进行中），表示已经启动，检查是否需要结束该调度
  else if (schedule.Status == RUNNING)
  {
      // 调用调度的结束函数
      schedule.pEndFunction();

      // 将调度状态更改为 OFF（关闭），表示调度已完成
      schedule.Status = OFF;

      // 如果有下一个调度，则激活下一个调度
      if(schedule.hasNextSchedule == true)
      {
        // 设置下一个调度的开始比较值和结束比较值
        SET_COMPARE(schedule.compare, schedule.nextStartCompare);
        SET_COMPARE(schedule.endCompare, schedule.nextEndCompare);

        // 将状态设置为 PENDING，表示下一个调度已经排队等候执行
        schedule.Status = PENDING;

        // 重置下一个调度标志
        schedule.hasNextSchedule = false;
      }
      else
      {
        // 如果没有下一个调度，则禁用定时器
        schedule.pTimerDisable();
      }
  }
  // 如果调度状态是 OFF（关闭），说明此调度不再活动
  else if (schedule.Status == OFF)
  {
    // 禁用定时器输出比较单元，确保没有进一步的操作
    schedule.pTimerDisable();
  }
}

/*******************************************************************************************************************************************************************************************************/
/** fuelSchedule*Interrupt (All 8 ISR functions below) get called (as timed interrupts) when either the start time or the duration time are reached.
* This calls the relevant callback function (startCallback or endCallback) depending on the status (PENDING => Needs to run, RUNNING => Needs to stop) of the schedule.
* The status of schedule is managed here based on startCallback /endCallback function called:
* - startCallback - change scheduler into RUNNING state
* - endCallback - change scheduler into OFF state (or PENDING if schedule.hasNextSchedule is set)
*/
// Timer3A (燃油调度1) 比较向量
#if defined(__AVR_ATmega1280__) || defined(__AVR_ATmega2560__) || defined(__AVR_ATmega2561__) // AVR芯片使用ISR
// 这是为燃油调度1和5设置的定时器比较中断服务程序
ISR(TIMER3_COMPA_vect) // cppcheck-suppress misra-c2012-8.2
#else
// 对于大多数ARM芯片，可以简单调用一个函数来处理定时器中断
void fuelSchedule1Interrupt() // 处理燃油调度1的中断
#endif
{
    // 调用共享的定时器ISR处理函数
    fuelScheduleISR(fuelSchedule1);
}


#if defined(__AVR_ATmega1280__) || defined(__AVR_ATmega2560__) || defined(__AVR_ATmega2561__) // AVR芯片使用ISR
ISR(TIMER3_COMPB_vect) // cppcheck-suppress misra-c2012-8.2
#else
// 对于大多数ARM芯片，可以简单调用一个函数来处理定时器中断
void fuelSchedule2Interrupt() // 处理燃油调度2的中断
#endif
{
    fuelScheduleISR(fuelSchedule2);
}


#if defined(__AVR_ATmega1280__) || defined(__AVR_ATmega2560__) || defined(__AVR_ATmega2561__) // AVR芯片使用ISR
ISR(TIMER3_COMPC_vect) // cppcheck-suppress misra-c2012-8.2
#else
// 对于大多数ARM芯片，可以简单调用一个函数来处理定时器中断
void fuelSchedule3Interrupt() // 处理燃油调度3的中断
#endif
{
    fuelScheduleISR(fuelSchedule3);
}


#if defined(__AVR_ATmega1280__) || defined(__AVR_ATmega2560__) || defined(__AVR_ATmega2561__) // AVR芯片使用ISR
ISR(TIMER4_COMPB_vect) // cppcheck-suppress misra-c2012-8.2
#else
// 对于大多数ARM芯片，可以简单调用一个函数来处理定时器中断
void fuelSchedule4Interrupt() // 处理燃油调度4的中断
#endif
{
    fuelScheduleISR(fuelSchedule4);
}

// 如果注射器通道数大于或等于5，处理燃油调度5
#if INJ_CHANNELS >= 5
#if defined(CORE_AVR) // AVR芯片使用ISR
ISR(TIMER4_COMPC_vect) // cppcheck-suppress misra-c2012-8.2
#else
// 对于大多数ARM芯片，可以简单调用一个函数来处理定时器中断
void fuelSchedule5Interrupt() // 处理燃油调度5的中断
#endif
{
    fuelScheduleISR(fuelSchedule5);
}
#endif

// 如果注射器通道数大于或等于6，处理燃油调度6
#if INJ_CHANNELS >= 6
#if defined(CORE_AVR) // AVR芯片使用ISR
ISR(TIMER4_COMPA_vect) // cppcheck-suppress misra-c2012-8.2
#else
// 对于大多数ARM芯片，可以简单调用一个函数来处理定时器中断
void fuelSchedule6Interrupt() // 处理燃油调度6的中断
#endif
{
    fuelScheduleISR(fuelSchedule6);
}
#endif

// 如果注射器通道数大于或等于7，处理燃油调度7
#if INJ_CHANNELS >= 7
#if defined(CORE_AVR) // AVR芯片使用ISR
ISR(TIMER5_COMPC_vect) // cppcheck-suppress misra-c2012-8.2
#else
// 对于大多数ARM芯片，可以简单调用一个函数来处理定时器中断
void fuelSchedule7Interrupt() // 处理燃油调度7的中断
#endif
{
    fuelScheduleISR(fuelSchedule7);
}
#endif

// 如果注射器通道数大于或等于8，处理燃油调度8
#if INJ_CHANNELS >= 8
#if defined(CORE_AVR) // AVR芯片使用ISR
ISR(TIMER5_COMPB_vect) // cppcheck-suppress misra-c2012-8.2
#else
// 对于大多数ARM芯片，可以简单调用一个函数来处理定时器中断
void fuelSchedule8Interrupt() // 处理燃油调度8的中断
#endif
{
    fuelScheduleISR(fuelSchedule8);
}
#endif

// Shared ISR function for all ignition timers.
// This is completely inlined into the ISR - there is no function call
// overhead.
static inline __attribute__((always_inline)) void ignitionScheduleISR(IgnitionSchedule &schedule)
{
  if (schedule.Status == PENDING) //Check to see if this schedule is turn on
  {
    schedule.pStartCallback();
    schedule.Status = RUNNING; //Set the status to be in progress (ie The start callback has been called, but not the end callback)
    schedule.startTime = micros();
    if(schedule.endScheduleSetByDecoder == true) { SET_COMPARE(schedule.compare, schedule.endCompare); }
    else { SET_COMPARE(schedule.compare, schedule.counter + uS_TO_TIMER_COMPARE(schedule.duration) ); } //Doing this here prevents a potential overflow on restarts
  }
  else if (schedule.Status == RUNNING)
  {
    schedule.pEndCallback();
    schedule.Status = OFF; //Turn off the schedule
    schedule.endScheduleSetByDecoder = false;
    ignitionCount = ignitionCount + 1; //Increment the ignition counter
    currentStatus.actualDwell = DWELL_AVERAGE( (micros() - schedule.startTime) );

    //If there is a next schedule queued up, activate it
    if(schedule.hasNextSchedule == true)
    {
      SET_COMPARE(schedule.compare, schedule.nextStartCompare);
      schedule.Status = PENDING;
      schedule.hasNextSchedule = false;
    }
    else
    { 
      schedule.pTimerDisable(); 
    }
  }
  else if (schedule.Status == OFF)
  {
    //Catch any spurious interrupts. This really shouldn't ever be called, but there as a safety
    schedule.pTimerDisable(); 
  }
}

#if defined(CORE_AVR) //AVR chips use the ISR for this
ISR(TIMER5_COMPA_vect) //cppcheck-suppress misra-c2012-8.2
#else
void ignitionSchedule1Interrupt(void) //Most ARM chips can simply call a function
#endif
  {
    ignitionScheduleISR(ignitionSchedule1);
  }

#if IGN_CHANNELS >= 2
#if defined(CORE_AVR) //AVR chips use the ISR for this
ISR(TIMER5_COMPB_vect) //cppcheck-suppress misra-c2012-8.2
#else
void ignitionSchedule2Interrupt(void) //Most ARM chips can simply call a function
#endif
  {
    ignitionScheduleISR(ignitionSchedule2);
  }
#endif

#if IGN_CHANNELS >= 3
#if defined(CORE_AVR) //AVR chips use the ISR for this
ISR(TIMER5_COMPC_vect) //cppcheck-suppress misra-c2012-8.2
#else
void ignitionSchedule3Interrupt(void) //Most ARM chips can simply call a function
#endif
  {
    ignitionScheduleISR(ignitionSchedule3);
  }
#endif

#if IGN_CHANNELS >= 4
#if defined(CORE_AVR) //AVR chips use the ISR for this
ISR(TIMER4_COMPA_vect) //cppcheck-suppress misra-c2012-8.2
#else
void ignitionSchedule4Interrupt(void) //Most ARM chips can simply call a function
#endif
  {
    ignitionScheduleISR(ignitionSchedule4);
  }
#endif

#if IGN_CHANNELS >= 5
#if defined(CORE_AVR) //AVR chips use the ISR for this
ISR(TIMER4_COMPC_vect) //cppcheck-suppress misra-c2012-8.2
#else
void ignitionSchedule5Interrupt(void) //Most ARM chips can simply call a function
#endif
  {
    ignitionScheduleISR(ignitionSchedule5);
  }
#endif

#if IGN_CHANNELS >= 6
#if defined(CORE_AVR) //AVR chips use the ISR for this
ISR(TIMER4_COMPB_vect) //cppcheck-suppress misra-c2012-8.2
#else
void ignitionSchedule6Interrupt(void) //Most ARM chips can simply call a function
#endif
  {
    ignitionScheduleISR(ignitionSchedule6);
  }
#endif

#if IGN_CHANNELS >= 7
#if defined(CORE_AVR) //AVR chips use the ISR for this
ISR(TIMER3_COMPC_vect) //cppcheck-suppress misra-c2012-8.2
#else
void ignitionSchedule7Interrupt(void) //Most ARM chips can simply call a function
#endif
  {
    ignitionScheduleISR(ignitionSchedule7);
  }
#endif

#if IGN_CHANNELS >= 8
#if defined(CORE_AVR) //AVR chips use the ISR for this
ISR(TIMER3_COMPB_vect) //cppcheck-suppress misra-c2012-8.2
#else
void ignitionSchedule8Interrupt(void) //Most ARM chips can simply call a function
#endif
  {
    ignitionScheduleISR(ignitionSchedule8);
  }
#endif

void disablePendingFuelSchedule(byte channel)
{
  noInterrupts();
  switch(channel)
  {
    case 0:
      if(fuelSchedule1.Status == PENDING) { fuelSchedule1.Status = OFF; }
      break;
    case 1:
      if(fuelSchedule2.Status == PENDING) { fuelSchedule2.Status = OFF; }
      break;
    case 2: 
      if(fuelSchedule3.Status == PENDING) { fuelSchedule3.Status = OFF; }
      break;
    case 3:
      if(fuelSchedule4.Status == PENDING) { fuelSchedule4.Status = OFF; }
      break;
    case 4:
#if (INJ_CHANNELS >= 5)
      if(fuelSchedule5.Status == PENDING) { fuelSchedule5.Status = OFF; }
#endif
      break;
    case 5:
#if (INJ_CHANNELS >= 6)
      if(fuelSchedule6.Status == PENDING) { fuelSchedule6.Status = OFF; }
#endif
      break;
    case 6:
#if (INJ_CHANNELS >= 7)
      if(fuelSchedule7.Status == PENDING) { fuelSchedule7.Status = OFF; }
#endif
      break;
    case 7:
#if (INJ_CHANNELS >= 8)
      if(fuelSchedule8.Status == PENDING) { fuelSchedule8.Status = OFF; }
#endif
      break;
  }
  interrupts();
}
void disablePendingIgnSchedule(byte channel)
{
  noInterrupts();
  switch(channel)
  {
    case 0:
      if(ignitionSchedule1.Status == PENDING) { ignitionSchedule1.Status = OFF; }
      break;
    case 1:
      if(ignitionSchedule2.Status == PENDING) { ignitionSchedule2.Status = OFF; }
      break;
    case 2: 
      if(ignitionSchedule3.Status == PENDING) { ignitionSchedule3.Status = OFF; }
      break;
    case 3:
      if(ignitionSchedule4.Status == PENDING) { ignitionSchedule4.Status = OFF; }
      break;
    case 4:
      if(ignitionSchedule5.Status == PENDING) { ignitionSchedule5.Status = OFF; }
      break;
#if IGN_CHANNELS >= 6      
    case 6:
      if(ignitionSchedule6.Status == PENDING) { ignitionSchedule6.Status = OFF; }
      break;
#endif
#if IGN_CHANNELS >= 7      
    case 7:
      if(ignitionSchedule7.Status == PENDING) { ignitionSchedule7.Status = OFF; }
      break;
#endif
#if IGN_CHANNELS >= 8      
    case 8:
      if(ignitionSchedule8.Status == PENDING) { ignitionSchedule8.Status = OFF; }
      break;
#endif
  }
  interrupts();
}
