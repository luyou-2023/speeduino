// Note that all functions with an underscore prefix are NOT part 
// of the public API. They are only here so we can inline them.

#include "scheduler.h"
#include "crankMaths.h"
#include "maths.h"
#include "timers.h"

static inline uint16_t calculateInjectorStartAngle(uint16_t pwDegrees, int16_t injChannelDegrees, uint16_t injAngle)
{
    // 注释:
    // 0 <= injAngle <= 720°           // 喷油角度的范围
    // 0 <= injChannelDegrees <= 720°  // 喷油通道的角度范围
    // 0 < pwDegrees <= ???            // 脉宽角度范围，可能会很大，具体取决于喷油持续时间
    // 45 <= CRANK_ANGLE_MAX_INJ <= 720 // 曲轴角度范围，可能因气缸数量不同而不同，例如8缸的情况可能为45°

    // 计算喷油器启动角度的初始值
    uint16_t startAngle = (uint16_t)injAngle + (uint16_t)injChannelDegrees;

    // 避免下溢
    while (startAngle < pwDegrees) {
        startAngle = startAngle + (uint16_t)CRANK_ANGLE_MAX_INJ;
    }

    // 确保 startAngle 大于等于 0
    startAngle = startAngle - pwDegrees;

    // 限制 startAngle 在 0 和 CRANK_ANGLE_MAX_INJ 之间
    while (startAngle > (uint16_t)CRANK_ANGLE_MAX_INJ) {
        startAngle = startAngle - (uint16_t)CRANK_ANGLE_MAX_INJ;
    }

    return startAngle;
}

static inline uint32_t _calculateInjectorTimeout(const FuelSchedule &schedule, uint16_t openAngle, uint16_t crankAngle) {
  int16_t delta = openAngle - crankAngle;
  if (delta<0)
  {
    if ((schedule.Status == RUNNING) && (delta>-CRANK_ANGLE_MAX_INJ)) 
    { 
      // Guaranteed to be >0
      delta = delta + CRANK_ANGLE_MAX_INJ; 
    }
    else
    {
      return 0;
    }
  }

  return angleToTimeMicroSecPerDegree((uint16_t)delta);
}

static inline int _adjustToInjChannel(int angle, int channelInjDegrees) {
  angle = angle - channelInjDegrees;
  if( angle < 0) { return angle + CRANK_ANGLE_MAX_INJ; }
  return angle;
}

static inline uint32_t calculateInjectorTimeout(const FuelSchedule &schedule, int channelInjDegrees, int openAngle, int crankAngle)
{
  if (channelInjDegrees==0) {
    return _calculateInjectorTimeout(schedule, openAngle, crankAngle);
  }
  return _calculateInjectorTimeout(schedule, _adjustToInjChannel(openAngle, channelInjDegrees), _adjustToInjChannel(crankAngle, channelInjDegrees));
}

/**
 * @brief 计算点火角度
 *
 * @param dwellAngle 点火保持角度
 * @param channelIgnDegrees 当前通道的点火角度
 * @param advance 点火提前角度（负值表示提前，正值表示延后）
 * @param pEndAngle 用于存储计算后的结束角度
 * @param pStartAngle 用于存储计算后的起始角度
 */
static inline void calculateIgnitionAngle(const uint16_t dwellAngle, const uint16_t channelIgnDegrees, int8_t advance, int *pEndAngle, int *pStartAngle)
{
  // 计算结束角度。如果当前通道点火角度为 0，则使用最大点火角度，减去提前角度。
  *pEndAngle = (int16_t)(channelIgnDegrees==0U ? (uint16_t)CRANK_ANGLE_MAX_IGN : channelIgnDegrees) - (int16_t)advance;

  // 如果结束角度超过最大点火角度，则进行角度归一化（减去最大点火角度）
  if(*pEndAngle > CRANK_ANGLE_MAX_IGN) {
    *pEndAngle -= CRANK_ANGLE_MAX_IGN;
  }

  // 计算起始角度。起始角度等于结束角度减去保持角度。
  *pStartAngle = *pEndAngle - dwellAngle;

  // 如果起始角度小于 0，则进行角度归一化（加上最大点火角度）
  if(*pStartAngle < 0) {
    *pStartAngle += CRANK_ANGLE_MAX_IGN;
  }
}

static inline void calculateIgnitionTrailingRotary(uint16_t dwellAngle, int rotarySplitDegrees, int leadIgnitionAngle, int *pEndAngle, int *pStartAngle)
{
  *pEndAngle = leadIgnitionAngle + rotarySplitDegrees;
  *pStartAngle = *pEndAngle - dwellAngle;
  if(*pStartAngle > CRANK_ANGLE_MAX_IGN) {*pStartAngle -= CRANK_ANGLE_MAX_IGN;}
  if(*pStartAngle < 0) {*pStartAngle += CRANK_ANGLE_MAX_IGN;}
}

static inline uint32_t _calculateIgnitionTimeout(const IgnitionSchedule &schedule, int16_t startAngle, int16_t crankAngle) {
  int16_t delta = startAngle - crankAngle;
  if (delta<0)
  {
    if ((schedule.Status == RUNNING) && (delta>-CRANK_ANGLE_MAX_IGN)) 
    { 
      // Msut be >0
      delta = delta + CRANK_ANGLE_MAX_IGN; 
    }
    else
    {
      return 0;
    }
  }
  return angleToTimeMicroSecPerDegree(delta);
}

static inline uint16_t _adjustToIgnChannel(int angle, int channelInjDegrees) {
  angle = angle - channelInjDegrees;
  if( angle < 0) { return angle + CRANK_ANGLE_MAX_IGN; }
  return angle;
}

static inline uint32_t calculateIgnitionTimeout(const IgnitionSchedule &schedule, int startAngle, int channelIgnDegrees, int crankAngle)
{
  if (channelIgnDegrees==0) {
      return _calculateIgnitionTimeout(schedule, startAngle, crankAngle);
  }
  return _calculateIgnitionTimeout(schedule, _adjustToIgnChannel(startAngle, channelIgnDegrees), _adjustToIgnChannel(crankAngle, channelIgnDegrees));
}

#define MIN_CYCLES_FOR_ENDCOMPARE 6

/** 调整曲轴角度
 * 该函数用于根据当前点火调度和曲轴角度，计算并设置点火调度的比较值，用于控制点火时序。
 *
 * @param schedule 点火调度对象，用于记录当前点火计划的状态和参数
 * @param endAngle 目标结束角度，用于计算角度差
 * @param crankAngle 当前的曲轴角度

 SET_COMPARE(schedule.compare, schedule.startCompare); 和 adjustCrankAngle 方法中涉及的 SET_COMPARE(schedule.compare, schedule.counter + uS_TO_TIMER_COMPARE(...)) 都是通过设置定时器的比较值来控制事件的触发，但它们在代码中执行的时机和目的有所不同。

 让我们深入分析这两者之间的关系。

 1. SET_COMPARE(schedule.compare, schedule.startCompare);
 这是一个直接设置定时器比较值的操作。它通常出现在主循环（loop）中，用来启动或更新定时器的比较单元，以便触发某个事件，如点火或其他调度控制。schedule.startCompare 是比较值，表示定时器应该在这个时间点触发某个动作。

 schedule.compare：定时器的比较单元。

 schedule.startCompare：设定的比较值，通常是某个未来的时间点，触发定时器事件。

 2. adjustCrankAngle 方法中的 SET_COMPARE
 adjustCrankAngle 方法在点火调度或其他时间相关操作时，根据曲轴角度差计算并设置新的比较值。其目的是根据曲轴的角度来精确调整定时器的比较时间。

 schedule.counter 是当前计时器的值，表示从程序开始运行到当前的时间点。

 uS_TO_TIMER_COMPARE 是一个宏或函数，用来将微秒级的时间转换为定时器的比较值。

 angleToTimeMicroSecPerDegree 用来将角度转换为时间（微秒），ignitionLimits 用来调整点火的限制，最终得到一个新的时间值。

 adjustCrankAngle 会根据当前的曲轴角度差（endAngle - crankAngle）来计算一个新的比较时间点，并设置定时器的比较值。这通常是在点火控制或其他周期性任务的调度中，动态调整点火时机或其他控制逻辑。

 两者之间的关系
 主循环中的 SET_COMPARE 和 adjustCrankAngle 中的 SET_COMPARE 都是更新定时器的比较值：

 SET_COMPARE(schedule.compare, schedule.startCompare); 通常是主循环的一部分，用来确保定时器在指定的时间点触发某个事件。

 adjustCrankAngle 通过计算与曲轴角度相关的时间差，动态更新比较值（schedule.compare），并使用新的时间点来控制定时器的触发。它计算出的比较值可能会传递给 SET_COMPARE，以调整时机。

 adjustCrankAngle 的作用是动态调整比较值：

 adjustCrankAngle 中的 SET_COMPARE 用来设置基于当前曲轴角度和预定点火时机计算的新的比较值。它比 loop 中的 SET_COMPARE 更精细，因为它会根据实时的曲轴状态来调整定时器的触发时机。

 loop 中的 SET_COMPARE 和 adjustCrankAngle 的结合：

 在主循环（loop）中，SET_COMPARE(schedule.compare, schedule.startCompare); 被调用来触发预定的定时任务。

 adjustCrankAngle 会在需要时计算出新的定时器比较值（比如在点火控制中），然后通过 SET_COMPARE 更新定时器。这意味着 loop 中的 SET_COMPARE 可能会定期更新定时器，而 adjustCrankAngle 会在点火时机变化时动态调整定时器的比较值。

 总结
 SET_COMPARE(schedule.compare, schedule.startCompare); 是主循环中设置定时器比较值的操作，用来控制定时器的触发。

 adjustCrankAngle 是一个计算曲轴角度差，并基于此计算定时器触发时机的函数。它会动态计算新的定时器比较值，并设置定时器。

 它们之间的关系是：adjustCrankAngle 用来根据曲轴的角度和配置动态调整定时器的触发时机，而 SET_COMPARE(schedule.compare, schedule.startCompare); 是定时器触发事件的实际执行操作。adjustCrankAngle 计算出的比较值可以直接传递给 SET_COMPARE 进行实际触发。
 adjustCrankAngle 主要用于 点火正时（Ignition Timing） 的修正，它的核心作用是根据当前的 曲轴角度（Crank Angle） 计算并调整点火的触发时间。

 在发动机点火系统中，点火正时的调整对于 燃烧效率、动力输出、油耗 以及 排放控制 至关重要。adjustCrankAngle 通过调整定时器的比较值 (SET_COMPARE) 来动态调整点火时机，使其适应不同的发动机工况，例如 发动机转速（RPM）、负载、进气压力（MAP） 等
 点火正时修正的原理
 点火时机通常用 曲轴角度（Crank Angle, °CA） 来表示，例如：

 提前点火（Advance Timing）：在活塞到达上止点（TDC）之前点火，通常用于高转速提升效率。

 延迟点火（Retarded Timing）：在活塞到达上止点之后点火，通常用于降低爆震风险或怠速稳定。

 adjustCrankAngle 主要是计算 理想点火角度 并转换成 实际的定时器触发时间：

 计算 所需的点火角度偏移量 endAngle - crankAngle。

 通过 angleToTimeMicroSecPerDegree 将角度转换为时间（微秒）。

 计算出新的点火触发时间，并用 SET_COMPARE 更新定时器。

 简单流程总结：
 代码计算出“点火”或“喷油”要触发的准确时间（以定时器计数值表示）。

 使用 SET_COMPARE(schedule.compare, ...) 把这个时间写入定时器的比较寄存器。

 定时器计数器计数到这个比较值时，硬件触发定时器比较中断。

 CPU 进入对应的 ISR（比如 TIMER3_COMPA_vect 或 fuelSchedule1Interrupt）。

 ISR 里调用调度处理函数（比如 fuelScheduleISR），执行具体的点火或喷油动作。

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
 */
inline void adjustCrankAngle(IgnitionSchedule &schedule, int endAngle, int crankAngle) {
    // 如果点火调度状态为运行中
    if (schedule.Status == RUNNING) {
        // 计算目标角度与当前曲轴角度的差值（endAngle - crankAngle），
        // 并通过 ignitionLimits 限制角度范围，
        // 将该角度差转换为对应的微秒时间，
        // 再转换为定时器比较值，
        // 最后设置到调度的比较寄存器(schedule.compare)，
        // 这样硬件定时器可以在正确的时间点触发事件。
        SET_COMPARE(schedule.compare, schedule.counter +
            uS_TO_TIMER_COMPARE(
                angleToTimeMicroSecPerDegree(
                    ignitionLimits((endAngle - crankAngle))
                )
            )
        );
    }
    // 如果点火调度还未运行，但已经经过了一定数量的曲轴转数（startRevolutions大于阈值）
    else if (currentStatus.startRevolutions > MIN_CYCLES_FOR_ENDCOMPARE) {
        // 计算目标角度与当前曲轴角度的差值对应的时间，设置为调度的结束比较值(schedule.endCompare)
        schedule.endCompare = schedule.counter +
            uS_TO_TIMER_COMPARE(
                angleToTimeMicroSecPerDegree(
                    ignitionLimits((endAngle - crankAngle))
                )
            );
        // 标记该结束比较值是由解码器设置的，方便后续处理
        schedule.endScheduleSetByDecoder = true;
    }
}
