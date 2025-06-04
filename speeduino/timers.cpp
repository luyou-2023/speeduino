/*
Speeduino - Simple engine management for the Arduino Mega 2560 platform
Copyright (C) Josh Stewart
A full copy of the license may be found in the projects root directory
*/

/*
Timers are used for having actions performed repeatedly at a fixed interval (Eg every 100ms)
They should not be confused with Schedulers, which are for performing an action once at a given point of time in the future

Timers are typically low resolution (Compared to Schedulers), with maximum frequency currently being approximately every 10ms
*/
#include "timers.h"
#include "globals.h"
#include "sensors.h"
#include "scheduler.h"
#include "scheduledIO.h"
#include "speeduino.h"
#include "scheduler.h"
#include "auxiliaries.h"
#include "comms.h"
#include "maths.h"

#if defined(CORE_AVR)
  #include <avr/wdt.h>
#endif

volatile uint16_t lastRPM_100ms; //Need to record this for rpmDOT calculation
volatile byte loop5ms;
volatile byte loop33ms;
volatile byte loop66ms;
volatile byte loop100ms;
volatile byte loop250ms;
volatile int loopSec;

volatile unsigned int dwellLimit_uS;

volatile uint8_t tachoEndTime; //The time (in ms) that the tacho pulse needs to end at
volatile TachoOutputStatus tachoOutputFlag;
volatile uint16_t tachoSweepIncr;
volatile uint16_t tachoSweepAccum;
volatile uint8_t testInjectorPulseCount = 0;
volatile uint8_t testIgnitionPulseCount = 0;

#if defined (CORE_TEENSY)
  IntervalTimer lowResTimer;
#endif

void initialiseTimers(void)
{
  lastRPM_100ms = 0;    // 100ms 内计算的 RPM（转速）值，初始化为 0
  loop5ms = 0;          // 5ms 任务循环计数器
  loop33ms = 0;         // 33ms 任务循环计数器
  loop66ms = 0;         // 66ms 任务循环计数器
  loop100ms = 0;        // 100ms 任务循环计数器
  loop250ms = 0;        // 250ms 任务循环计数器
  loopSec = 0;          // 1秒任务循环计数器
  tachoOutputFlag = TACHO_INACTIVE; // 转速表输出标志，设置为“未激活”状态
}

static inline void applyOverDwellCheck(IgnitionSchedule &schedule, uint32_t targetOverdwellTime) {
  // Check first whether each spark output is currently on. Only check its dwell time if it is
  if ((schedule.Status == RUNNING) && (schedule.startTime < targetOverdwellTime)) {
    schedule.pEndCallback();    // 执行结束回调函数
    schedule.Status = OFF;      // 将状态设置为 OFF，表示点火已结束
  }
}

//不同时间的定时器

// 定时器2溢出中断向量，定时器溢出时调用。
// 每大约1毫秒执行一次。
#if defined(CORE_AVR) // AVR芯片使用ISR
// 这个函数不能被阻塞。禁用NO_BLOCK会影响计时精度。
ISR(TIMER2_OVF_vect, ISR_NOBLOCK) // cppcheck-suppress misra-c2012-8.2
#else
void oneMSInterval(void) // 大多数ARM芯片只需调用此函数
#endif
{
  BIT_SET(TIMER_mask, BIT_TIMER_1KHZ);
  ms_counter++;

  // 增加循环计数器
  loop5ms++;
  loop33ms++;
  loop66ms++;
  loop100ms++;
  loop250ms++;
  loopSec++;

  // 检查过长点火
  uint32_t targetOverdwellTime = micros() - dwellLimit_uS; // 设置一个过去的目标时间，所有点火线圈充电必须在此时间之后开始。如果充电时间早于此，说明过长。
  bool isCrankLocked = configPage4.ignCranklock && (currentStatus.RPM < currentStatus.crankRPM); // 使用锁定起动定时的设置时，点火限制器在起动时禁用。此时必须进行RPM检查，因为依赖引擎起动标志更新可能过慢。
  if ((configPage4.useDwellLim == 1) && (isCrankLocked != true))
  {
    applyOverDwellCheck(ignitionSchedule1, targetOverdwellTime);
#if IGN_CHANNELS >= 2
    applyOverDwellCheck(ignitionSchedule2, targetOverdwellTime);
#endif
#if IGN_CHANNELS >= 3
    applyOverDwellCheck(ignitionSchedule3, targetOverdwellTime);
#endif
#if IGN_CHANNELS >= 4
    applyOverDwellCheck(ignitionSchedule4, targetOverdwellTime);
#endif
#if IGN_CHANNELS >= 5
    applyOverDwellCheck(ignitionSchedule5, targetOverdwellTime);
#endif
#if IGN_CHANNELS >= 6
    applyOverDwellCheck(ignitionSchedule6, targetOverdwellTime);
#endif
#if IGN_CHANNELS >= 7
    applyOverDwellCheck(ignitionSchedule7, targetOverdwellTime);
#endif
#if IGN_CHANNELS >= 8
    applyOverDwellCheck(ignitionSchedule8, targetOverdwellTime);
#endif
  }

  // 判断是否处于电源开启扫频模式
  if( currentStatus.tachoSweepEnabled )
  {
    if( (currentStatus.engine != 0) || (ms_counter >= TACHO_SWEEP_TIME_MS) )  { currentStatus.tachoSweepEnabled = false; }  // 扫频时间结束或开始有真实转速信号时停止扫频
    else
    {
      // 在扫频时间内平滑地将指针增加到最大
      if( ms_counter < TACHO_SWEEP_RAMP_MS ) { tachoSweepAccum += map(ms_counter, 0, TACHO_SWEEP_RAMP_MS, 0, tachoSweepIncr); }
      else                                   { tachoSweepAccum += tachoSweepIncr;                                             }

      // 每次完成一次滚动时，准备发出转速脉冲
      if( tachoSweepAccum >= MS_PER_SEC )
      {
        tachoOutputFlag = READY;
        tachoSweepAccum -= MS_PER_SEC;
      }
    }
  }

  // 转速输出检查。如果转速脉冲持续时间固定与点火线圈充电有关，则此代码不会执行任何操作。
  if(tachoOutputFlag == READY)
  {
    // 检查是否为半速转速
    if( (configPage2.tachoDiv == 0) || (currentStatus.tachoAlt == true) )
    {
      TACHO_PULSE_LOW();
      // 由于转速持续时间仅在1-6范围内，因此将ms_counter转换为字节类型，避免不必要的高精度
      tachoEndTime = (uint8_t)ms_counter + configPage2.tachoDuration;
      tachoOutputFlag = ACTIVE;
    }
    else
    {
      // 不执行该脉冲（半速转速）
      tachoOutputFlag = TACHO_INACTIVE;
    }
    currentStatus.tachoAlt = !currentStatus.tachoAlt; // 如果使用半速转速，则切换交替值
  }
  else if(tachoOutputFlag == ACTIVE)
  {
    // 如果转速输出已激活，检查是否已达到结束时间
    if((uint8_t)ms_counter == tachoEndTime)
    {
      TACHO_PULSE_HIGH();
      tachoOutputFlag = TACHO_INACTIVE;
    }
  }

  // 200Hz 循环
  if (loop5ms == 5)
  {
    loop5ms = 0; // 重置计数器
    BIT_SET(TIMER_mask, BIT_TIMER_200HZ);
  }

  // 喷油
  // 30Hz 循环
  if (loop33ms == 33)
  {
    loop33ms = 0;

    // 在30Hz下设置脉冲喷油和点火测试输出
    if( BIT_CHECK(currentStatus.testOutputs, 1) && (currentStatus.RPM == 0) )
    {
      // 检查是否为脉冲喷油器输出测试
      if(BIT_CHECK(HWTest_INJ_Pulsed, INJ1_CMD_BIT)) { openInjector1(); }
      if(BIT_CHECK(HWTest_INJ_Pulsed, INJ2_CMD_BIT)) { openInjector2(); }
      if(BIT_CHECK(HWTest_INJ_Pulsed, INJ3_CMD_BIT)) { openInjector3(); }
      if(BIT_CHECK(HWTest_INJ_Pulsed, INJ4_CMD_BIT)) { openInjector4(); }
      if(BIT_CHECK(HWTest_INJ_Pulsed, INJ5_CMD_BIT)) { openInjector5(); }
      if(BIT_CHECK(HWTest_INJ_Pulsed, INJ6_CMD_BIT)) { openInjector6(); }
      if(BIT_CHECK(HWTest_INJ_Pulsed, INJ7_CMD_BIT)) { openInjector7(); }
      if(BIT_CHECK(HWTest_INJ_Pulsed, INJ8_CMD_BIT)) { openInjector8(); }
      testInjectorPulseCount = 0;

      // 检查是否为脉冲点火输出测试
      if(BIT_CHECK(HWTest_IGN_Pulsed, IGN1_CMD_BIT)) { beginCoil1Charge(); }
      if(BIT_CHECK(HWTest_IGN_Pulsed, IGN2_CMD_BIT)) { beginCoil2Charge(); }
      if(BIT_CHECK(HWTest_IGN_Pulsed, IGN3_CMD_BIT)) { beginCoil3Charge(); }
      if(BIT_CHECK(HWTest_IGN_Pulsed, IGN4_CMD_BIT)) { beginCoil4Charge(); }
      if(BIT_CHECK(HWTest_IGN_Pulsed, IGN5_CMD_BIT)) { beginCoil5Charge(); }
      if(BIT_CHECK(HWTest_IGN_Pulsed, IGN6_CMD_BIT)) { beginCoil6Charge(); }
      if(BIT_CHECK(HWTest_IGN_Pulsed, IGN7_CMD_BIT)) { beginCoil7Charge(); }
      if(BIT_CHECK(HWTest_IGN_Pulsed, IGN8_CMD_BIT)) { beginCoil8Charge(); }
      testIgnitionPulseCount = 0;
    }

    BIT_SET(TIMER_mask, BIT_TIMER_30HZ);
  }

  // 15Hz 循环
  if (loop66ms == 66)
  {
    loop66ms = 0;
    BIT_SET(TIMER_mask, BIT_TIMER_15HZ);
  }

  // 10Hz 循环
  if (loop100ms == 100)
  {
    loop100ms = 0; // 重置计数器
    BIT_SET(TIMER_mask, BIT_TIMER_10HZ);
    /**
          currentStatus.longRPM = getRPM(); //Long RPM is included here
          currentStatus.RPM = currentStatus.longRPM;
          currentStatus.RPMdiv100 = div100(currentStatus.RPM);
    **/
    //计算过去100ms内发动机转速变化的速率（转速变化量 * 10，即每秒转速变化）
    currentStatus.rpmDOT = (currentStatus.RPM - lastRPM_100ms) * 10; // 这是引擎在上一个循环中加速或减速的每秒转速
    lastRPM_100ms = currentStatus.RPM; // 记录当前的转速以便下次计算

    if ( BIT_CHECK(currentStatus.engine, BIT_ENGINE_RUN) ) { runSecsX10++; }
    else { runSecsX10 = 0; }
    // beginInjectorPriming()执行喷油器的预喷脉冲
    if ( (currentStatus.injPrimed == false) && (seclx10 == configPage2.primingDelay) && (currentStatus.RPM == 0) ) { beginInjectorPriming(); currentStatus.injPrimed = true; }
    seclx10++;
  }

  // 4Hz 循环
  if (loop250ms == 250)
  {
    loop250ms = 0; // 重置计数器
    BIT_SET(TIMER_mask, BIT_TIMER_4HZ);
    #if defined(CORE_STM32) // 调试目的，仅用于查看代码是否运行
      digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
    #endif
  }

  // 1Hz 循环
  if (loopSec == 1000)
  {
    loopSec = 0; // 重置计数器
    BIT_SET(TIMER_mask, BIT_TIMER_1HZ);

    // 基本状态信息
    statusLED();
    refreshStatusLED();
    statusOutputs(); // 在50Hz（每20ms）时检查每个输出的状态

    // 在1秒间隔时输出用于调试的引擎状态信息
    if (configPage4.debugOutputEnabled)
    {
      printStatusText1();
      printStatusText2();
      printStatusText3();
      printStatusText4();
    }
  }
}
