#ifndef AVR2560_H
#define AVR2560_H

#include "globals.h"
#if defined(CORE_AVR)

#include <avr/interrupt.h>
#include <avr/io.h>

/*
***********************************************************************************************************
* General
*/
  #define PORT_TYPE uint8_t //Size of the port variables (Eg inj1_pin_port).
  #define PINMASK_TYPE uint8_t
  #define COMPARE_TYPE uint16_t
  #define COUNTER_TYPE uint16_t
  #define SERIAL_BUFFER_SIZE (256+7+1) //Size of the serial buffer used by new comms protocol. The largest single packet is the O2 calibration which is 256 bytes + 7 bytes of overhead
  #define FPU_MAX_SIZE 0 //Size of the FPU buffer. 0 means no FPU.
  #ifdef USE_SPI_EEPROM
    #define EEPROM_LIB_H "src/SPIAsEEPROM/SPIAsEEPROM.h"
    typedef uint16_t eeprom_address_t;
  #else
    #define EEPROM_LIB_H <EEPROM.h>
    typedef int eeprom_address_t;
  #endif
  #ifdef PLATFORMIO
    #define RTC_LIB_H <TimeLib.h>
  #else
    #define RTC_LIB_H <Time.h>
  #endif
  void initBoard(void);
  uint16_t freeRam(void);
  void doSystemReset(void);
  void jumpToBootloader(void);

  #if defined(TIMER5_MICROS)
    /*#define micros() (((timer5_overflow_count << 16) + TCNT5) * 4) */ //Fast version of micros() that uses the 4uS tick of timer5. See timers.ino for the overflow ISR of timer5
    #define millis() (ms_counter) //Replaces the standard millis() function with this macro. It is both faster and more accurate. See timers.ino for its counter increment.
    static inline unsigned long micros_safe(); //A version of micros() that is interrupt safe
  #else
    #define micros_safe() micros() //If the timer5 method is not used, the micros_safe() macro is simply an alias for the normal micros()
  #endif
  #define pinIsReserved(pin)  ( ((pin) == 0) ) //Forbidden pins like USB on other boards

  //Mega 2561 MCU does not have a serial3 available. 
  #if not defined(__AVR_ATmega2561__)
    #define USE_SERIAL3
  #endif

/*
***********************************************************************************************************
* Schedules
*/
  //Refer to svn.savannah.nongnu.org/viewvc/trunk/avr-libc/include/avr/iomxx0_1.h?root=avr-libc&view=markup
  #define FUEL1_COUNTER TCNT3
  #define FUEL2_COUNTER TCNT3
  #define FUEL3_COUNTER TCNT3
  #define FUEL4_COUNTER TCNT4
  #define FUEL5_COUNTER TCNT4
  #define FUEL6_COUNTER TCNT4 //Replaces ignition 4
  #define FUEL7_COUNTER TCNT5 //Replaces ignition 3
  #define FUEL8_COUNTER TCNT5 //Replaces ignition 2

  #define IGN1_COUNTER  TCNT5
  #define IGN2_COUNTER  TCNT5
  #define IGN3_COUNTER  TCNT5
  #define IGN4_COUNTER  TCNT4
  #define IGN5_COUNTER  TCNT4
  #define IGN6_COUNTER  TCNT4 //Replaces injector 4
  #define IGN7_COUNTER  TCNT3 //Replaces injector 3
  #define IGN8_COUNTER  TCNT3 //Replaces injector 2

  // 将定时器 3 的输出比较寄存器（OCR3A、OCR3B、OCR3C）分别用于燃油喷射的控制
  #define FUEL1_COMPARE OCR3A // FUEL1 使用 OCR3A 作为比较寄存器，控制燃油喷射 1
  #define FUEL2_COMPARE OCR3B // FUEL2 使用 OCR3B 作为比较寄存器，控制燃油喷射 2
  #define FUEL3_COMPARE OCR3C // FUEL3 使用 OCR3C 作为比较寄存器，控制燃油喷射 3

  // 将定时器 4 的输出比较寄存器（OCR4A、OCR4B、OCR4C）用于燃油喷射控制，同时替代点火 6、5 和 4
  #define FUEL4_COMPARE OCR4B // FUEL4 使用 OCR4B 作为比较寄存器，控制燃油喷射 4，替代点火 6
  #define FUEL5_COMPARE OCR4C // FUEL5 使用 OCR4C 作为比较寄存器，控制燃油喷射 5，替代点火 5
  #define FUEL6_COMPARE OCR4A // FUEL6 使用 OCR4A 作为比较寄存器，控制燃油喷射 6，替代点火 4

  // 将定时器 5 的输出比较寄存器（OCR5B、OCR5C）用于燃油喷射控制，同时替代点火 3 和 2
  #define FUEL7_COMPARE OCR5C // FUEL7 使用 OCR5C 作为比较寄存器，控制燃油喷射 7，替代点火 3
  #define FUEL8_COMPARE OCR5B // FUEL8 使用 OCR5B 作为比较寄存器，控制燃油喷射 8，替代点火 2

  // 定时器输出比较寄存器
  #define IGN1_COMPARE  OCR5A  // 点火控制信号 1 关联到 Timer 5 的通道 A
  #define IGN2_COMPARE  OCR5B  // 点火控制信号 2 关联到 Timer 5 的通道 B
  #define IGN3_COMPARE  OCR5C  // 点火控制信号 3 关联到 Timer 5 的通道 C
  #define IGN4_COMPARE  OCR4A  // 点火控制信号 4 关联到 Timer 4 的通道 A
  #define IGN5_COMPARE  OCR4C  // 点火控制信号 5 关联到 Timer 4 的通道 C
  #define IGN6_COMPARE  OCR4B  // 点火控制信号 6 关联到 Timer 4 的通道 B
  #define IGN7_COMPARE  OCR3C  // 点火控制信号 7 关联到 Timer 3 的通道 C
  #define IGN8_COMPARE  OCR3B  // 点火控制信号 8 关联到 Timer 3 的通道 B

  //Note that the interrupt flag is reset BEFORE the interrupt is enabled
static inline void FUEL1_TIMER_ENABLE(void) { TIFR3 |= (1<<OCF3A) ; TIMSK3 |= (1 << OCIE3A); } //Turn on the A compare unit (ie turn on the interrupt)
static inline void FUEL2_TIMER_ENABLE(void) { TIFR3 |= (1<<OCF3B); TIMSK3 |= (1 << OCIE3B); } //Turn on the B compare unit (ie turn on the interrupt)
static inline void FUEL3_TIMER_ENABLE(void) { TIFR3 |= (1<<OCF3C); TIMSK3 |= (1 << OCIE3C); } //Turn on the C compare unit (ie turn on the interrupt)
static inline void FUEL4_TIMER_ENABLE(void) { TIFR4 |= (1<<OCF4B); TIMSK4 |= (1 << OCIE4B); } //Turn on the B compare unit (ie turn on the interrupt)
static inline void FUEL5_TIMER_ENABLE(void) { TIFR4 |= (1<<OCF4C); TIMSK4 |= (1 << OCIE4C); } //Turn on the C compare unit (ie turn on the interrupt)
static inline void FUEL6_TIMER_ENABLE(void) { TIFR4 |= (1<<OCF4A); TIMSK4 |= (1 << OCIE4A); } //Turn on the A compare unit (ie turn on the interrupt)
static inline void FUEL7_TIMER_ENABLE(void) { TIFR5 |= (1<<OCF5C); TIMSK5 |= (1 << OCIE5C); } //
static inline void FUEL8_TIMER_ENABLE(void) { TIFR5 |= (1<<OCF5B); TIMSK5 |= (1 << OCIE5B); } //

static inline void FUEL1_TIMER_DISABLE(void) { TIMSK3 &= ~(1 << OCIE3A); } // //Turn off this output compare unit
static inline void FUEL2_TIMER_DISABLE(void) { TIMSK3 &= ~(1 << OCIE3B); } // //Turn off this output compare unit
static inline void FUEL3_TIMER_DISABLE(void) { TIMSK3 &= ~(1 << OCIE3C); } // //Turn off this output compare unit
static inline void FUEL4_TIMER_DISABLE(void) { TIMSK4 &= ~(1 << OCIE4B); } ////Turn off this output compare unit
static inline void FUEL5_TIMER_DISABLE(void) { TIMSK4 &= ~(1 << OCIE4C); } // //
static inline void FUEL6_TIMER_DISABLE(void) { TIMSK4 &= ~(1 << OCIE4A); } // //
static inline void FUEL7_TIMER_DISABLE(void) { TIMSK5 &= ~(1 << OCIE5C); } // //
static inline void FUEL8_TIMER_DISABLE(void) { TIMSK5 &= ~(1 << OCIE5B); } //

  //These have the TIFR5 bits set to 1 to clear the interrupt flag. This prevents a false interrupt being called the first time the channel is enabled.
static inline void IGN1_TIMER_ENABLE(void) { TIFR5 |= (1<<OCF5A); TIMSK5 |= (1 << OCIE5A); } //Turn on the A compare unit (ie turn on the interrupt)
static inline void IGN2_TIMER_ENABLE(void) { TIFR5 |= (1<<OCF5B); TIMSK5 |= (1 << OCIE5B); }//Turn on the B compare unit (ie turn on the interrupt)
static inline void IGN3_TIMER_ENABLE(void) { TIFR5 |= (1<<OCF5C); TIMSK5 |= (1 << OCIE5C); }//Turn on the C compare unit (ie turn on the interrupt)
static inline void IGN4_TIMER_ENABLE(void) { TIFR4 |= (1<<OCF4A); TIMSK4 |= (1 << OCIE4A); }//Turn on the A compare unit (ie turn on the interrupt)
static inline void IGN5_TIMER_ENABLE(void) { TIFR4 |= (1<<OCF4C); TIMSK4 |= (1 << OCIE4C); } //Turn on the A compare unit (ie turn on the interrupt)
static inline void IGN6_TIMER_ENABLE(void) { TIFR4 |= (1<<OCF4B); TIMSK4 |= (1 << OCIE4B); } //Replaces injector 4
static inline void IGN7_TIMER_ENABLE(void) { TIMSK3 |= (1 << OCIE3C); }//Replaces injector 3
static inline void IGN8_TIMER_ENABLE(void) { TIMSK3 |= (1 << OCIE3B); } //Replaces injector 2

static inline void IGN1_TIMER_DISABLE(void) { TIMSK5 &= ~(1 << OCIE5A); } //Turn off this output compare unit
static inline void IGN2_TIMER_DISABLE(void) { TIMSK5 &= ~(1 << OCIE5B); } //Turn off this output compare unit
static inline void IGN3_TIMER_DISABLE(void) { TIMSK5 &= ~(1 << OCIE5C); } //Turn off this output compare unit
static inline void IGN4_TIMER_DISABLE(void) { TIMSK4 &= ~(1 << OCIE4A); } //Turn off this output compare unit
static inline void IGN5_TIMER_DISABLE(void) { TIMSK4 &= ~(1 << OCIE4C); } //Turn off this output compare unit
static inline void IGN6_TIMER_DISABLE(void) { TIMSK4 &= ~(1 << OCIE4B); } //Replaces injector 4
static inline void IGN7_TIMER_DISABLE(void) { TIMSK3 &= ~(1 << OCIE3C); } //Replaces injector 3
static inline void IGN8_TIMER_DISABLE(void) { TIMSK3 &= ~(1 << OCIE3B); } //Replaces injector 2

  #define MAX_TIMER_PERIOD 262140UL //The longest period of time (in uS) that the timer can permit (IN this case it is 65535 * 4, as each timer tick is 4uS)
  #define uS_TO_TIMER_COMPARE(uS1) ((uS1) >> 2) //Converts a given number of uS into the required number of timer ticks until that time has passed

/*
***********************************************************************************************************
* Auxiliaries
*/
  #define ENABLE_BOOST_TIMER()  TIMSK1 |= (1 << OCIE1A)
  #define DISABLE_BOOST_TIMER() TIMSK1 &= ~(1 << OCIE1A)
  #define ENABLE_VVT_TIMER()    TIMSK1 |= (1 << OCIE1B)
  #define DISABLE_VVT_TIMER()   TIMSK1 &= ~(1 << OCIE1B)

  #define BOOST_TIMER_COMPARE   OCR1A
  #define BOOST_TIMER_COUNTER   TCNT1
  #define VVT_TIMER_COMPARE     OCR1B
  #define VVT_TIMER_COUNTER     TCNT1

/*
***********************************************************************************************************
* Idle
*/
  #define IDLE_COUNTER TCNT1
  #define IDLE_COMPARE OCR1C

  #define IDLE_TIMER_ENABLE() TIMSK1 |= (1 << OCIE1C)
  #define IDLE_TIMER_DISABLE() TIMSK1 &= ~(1 << OCIE1C)

/*
***********************************************************************************************************
* CAN / Second serial
*/
#if ( defined(__AVR_ATmega1280__) || defined(__AVR_ATmega2560__) )
  #define secondarySerial_AVAILABLE
#endif
#define SECONDARY_SERIAL_T HardwareSerial

#endif //CORE_AVR
#endif //AVR2560_H
