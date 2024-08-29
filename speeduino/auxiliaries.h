#ifndef AUX_H
#define AUX_H

#include BOARD_H // 注意这是一个虚拟文件，实际在 globals.h 中定义。

#if defined(CORE_AVR)
#include <util/atomic.h> // 如果在 AVR 核心上，包含原子操作库
#endif

// 函数原型声明
void initialiseAuxPWM(void);   // 初始化辅助 PWM
void boostControl(void);       // 控制增压
void boostDisable(void);       // 禁用增压
void boostByGear(void);        // 根据档位控制增压
void vvtControl(void);         // 控制可变气门正时
void initialiseFan(void);      // 初始化风扇
void initialiseAirCon(void);   // 初始化空调
void nitrousControl(void);     // 控制氮气
void fanControl(void);         // 控制风扇
void airConControl(void);      // 控制空调
bool READ_AIRCON_REQUEST(void); // 读取空调请求
void wmiControl(void);         // 控制水甲醇注入

// 简单的增压控制参数
#define SIMPLE_BOOST_P  1
#define SIMPLE_BOOST_I  1
#define SIMPLE_BOOST_D  1

// 硬件平台特定的宏定义
#if(defined(CORE_TEENSY) || defined(CORE_STM32))
#define BOOST_PIN_LOW()         (digitalWrite(pinBoost, LOW)) // 将增压引脚设为低电平
#define BOOST_PIN_HIGH()        (digitalWrite(pinBoost, HIGH)) // 将增压引脚设为高电平
#define VVT1_PIN_LOW()          (digitalWrite(pinVVT_1, LOW)) // 将 VVT1 引脚设为低电平
#define VVT1_PIN_HIGH()         (digitalWrite(pinVVT_1, HIGH)) // 将 VVT1 引脚设为高电平
#define VVT2_PIN_LOW()          (digitalWrite(pinVVT_2, LOW)) // 将 VVT2 引脚设为低电平
#define VVT2_PIN_HIGH()         (digitalWrite(pinVVT_2, HIGH)) // 将 VVT2 引脚设为高电平
#define FAN_PIN_LOW()           (digitalWrite(pinFan, LOW)) // 将风扇引脚设为低电平
#define FAN_PIN_HIGH()          (digitalWrite(pinFan, HIGH)) // 将风扇引脚设为高电平
#define N2O_STAGE1_PIN_LOW()    (digitalWrite(configPage10.n2o_stage1_pin, LOW)) // 将氮气阶段1引脚设为低电平
#define N2O_STAGE1_PIN_HIGH()   (digitalWrite(configPage10.n2o_stage1_pin, HIGH)) // 将氮气阶段1引脚设为高电平
#define N2O_STAGE2_PIN_LOW()    (digitalWrite(configPage10.n2o_stage2_pin, LOW)) // 将氮气阶段2引脚设为低电平
#define N2O_STAGE2_PIN_HIGH()   (digitalWrite(configPage10.n2o_stage2_pin, HIGH)) // 将氮气阶段2引脚设为高电平
#define AIRCON_PIN_LOW()        (digitalWrite(pinAirConComp, LOW)) // 将空调压缩机引脚设为低电平
#define AIRCON_PIN_HIGH()       (digitalWrite(pinAirConComp, HIGH)) // 将空调压缩机引脚设为高电平
#define AIRCON_FAN_PIN_LOW()    (digitalWrite(pinAirConFan, LOW)) // 将空调风扇引脚设为低电平
#define AIRCON_FAN_PIN_HIGH()   (digitalWrite(pinAirConFan, HIGH)) // 将空调风扇引脚设为高电平
#define FUEL_PUMP_ON()          (digitalWrite(pinFuelPump, HIGH)) // 打开燃油泵
#define FUEL_PUMP_OFF()         (digitalWrite(pinFuelPump, LOW)) // 关闭燃油泵

// 空调控制宏
#define AIRCON_ON()             { (((configPage15.airConCompPol==1)) ? AIRCON_PIN_LOW() : AIRCON_PIN_HIGH()); BIT_SET(currentStatus.airConStatus, BIT_AIRCON_COMPRESSOR); }
#define AIRCON_OFF()            { (((configPage15.airConCompPol==1)) ? AIRCON_PIN_HIGH() : AIRCON_PIN_LOW()); BIT_CLEAR(currentStatus.airConStatus, BIT_AIRCON_COMPRESSOR); }
#define AIRCON_FAN_ON()         { (((configPage15.airConFanPol==1)) ? AIRCON_FAN_PIN_LOW() : AIRCON_FAN_PIN_HIGH()); BIT_SET(currentStatus.airConStatus, BIT_AIRCON_FAN); }
#define AIRCON_FAN_OFF()        { (((configPage15.airConFanPol==1)) ? AIRCON_FAN_PIN_HIGH() : AIRCON_FAN_PIN_LOW()); BIT_CLEAR(currentStatus.airConStatus, BIT_AIRCON_FAN); }

// 风扇控制宏
#define FAN_ON()                { ((configPage6.fanInv) ? FAN_PIN_LOW() : FAN_PIN_HIGH()); }
#define FAN_OFF()               { ((configPage6.fanInv) ? FAN_PIN_HIGH() : FAN_PIN_LOW()); }
#else
// 对于 AVR 平台的引脚操作，使用原子操作来确保操作的原子性
#define BOOST_PIN_LOW()         ATOMIC_BLOCK(ATOMIC_RESTORESTATE) { *boost_pin_port &= ~(boost_pin_mask); }
#define BOOST_PIN_HIGH()        ATOMIC_BLOCK(ATOMIC_RESTORESTATE) { *boost_pin_port |= (boost_pin_mask);  }
#define VVT1_PIN_LOW()          ATOMIC_BLOCK(ATOMIC_RESTORESTATE) { *vvt1_pin_port &= ~(vvt1_pin_mask);   }
#define VVT1_PIN_HIGH()         ATOMIC_BLOCK(ATOMIC_RESTORESTATE) { *vvt1_pin_port |= (vvt1_pin_mask);    }
#define VVT2_PIN_LOW()          ATOMIC_BLOCK(ATOMIC_RESTORESTATE) { *vvt2_pin_port &= ~(vvt2_pin_mask);   }
#define VVT2_PIN_HIGH()         ATOMIC_BLOCK(ATOMIC_RESTORESTATE) { *vvt2_pin_port |= (vvt2_pin_mask);    }
#define N2O_STAGE1_PIN_LOW()    ATOMIC_BLOCK(ATOMIC_RESTORESTATE) { *n2o_stage1_pin_port &= ~(n2o_stage1_pin_mask);  }
#define N2O_STAGE1_PIN_HIGH()   ATOMIC_BLOCK(ATOMIC_RESTORESTATE) { *n2o_stage1_pin_port |= (n2o_stage1_pin_mask);   }
#define N2O_STAGE2_PIN_LOW()    ATOMIC_BLOCK(ATOMIC_RESTORESTATE) { *n2o_stage2_pin_port &= ~(n2o_stage2_pin_mask);  }
#define N2O_STAGE2_PIN_HIGH()   ATOMIC_BLOCK(ATOMIC_RESTORESTATE) { *n2o_stage2_pin_port |= (n2o_stage2_pin_mask);   }
#define FUEL_PUMP_ON()          ATOMIC_BLOCK(ATOMIC_RESTORESTATE) { *pump_pin_port |= (pump_pin_mask);     }
#define FUEL_PUMP_OFF()         ATOMIC_BLOCK(ATOMIC_RESTORESTATE) { *pump_pin_port &= ~(pump_pin_mask);    }

// 这些宏在三元操作符中无法使用 ATOMIC_BLOCK，原子操作在三元操作符中执行
#define FAN_PIN_LOW()           *fan_pin_port &= ~(fan_pin_mask)
#define FAN_PIN_HIGH()          *fan_pin_port |= (fan_pin_mask)
#define AIRCON_PIN_LOW()        *aircon_comp_pin_port &= ~(aircon_comp_pin_mask)
#define AIRCON_PIN_HIGH()       *aircon_comp_pin_port |= (aircon_comp_pin_mask)
#define AIRCON_FAN_PIN_LOW()    *aircon_fan_pin_port &= ~(aircon_fan_pin_mask)
#define AIRCON_FAN_PIN_HIGH()   *aircon_fan_pin_port |= (aircon_fan_pin_mask)

// 空调控制宏
#define AIRCON_ON()             ATOMIC_BLOCK(ATOMIC_RESTORESTATE) { ((((configPage15.airConCompPol)==1)) ? AIRCON_PIN_LOW() : AIRCON_PIN_HIGH()); BIT_SET(currentStatus.airConStatus, BIT_AIRCON_COMPRESSOR); }
#define AIRCON_OFF()            ATOMIC_BLOCK(ATOMIC_RESTORESTATE) { ((((configPage15.airConCompPol)==1)) ? AIRCON_PIN_HIGH() : AIRCON_PIN_LOW()); BIT_CLEAR(currentStatus.airConStatus, BIT_AIRCON_COMPRESSOR); }
#define AIRCON_FAN_ON()         ATOMIC_BLOCK(ATOMIC_RESTORESTATE) { ((((configPage15.airConFanPol)==1)) ? AIRCON_FAN_PIN_LOW() : AIRCON_FAN_PIN_HIGH()); BIT_SET(currentStatus.airConStatus, BIT_AIRCON_FAN); }
#define AIRCON_FAN_OFF()        ATOMIC_BLOCK(ATOMIC_RESTORESTATE) { ((((configPage15.airConFanPol)==1)) ? AIRCON_FAN_PIN_HIGH() : AIRCON_FAN_PIN_LOW()); BIT_CLEAR(currentStatus.airConStatus, BIT_AIRCON_FAN); }

// 风扇控制宏
#define FAN_ON()                ATOMIC_BLOCK(ATOMIC_RESTORESTATE) { ((configPage6.fanInv) ? FAN_PIN_LOW() : FAN_PIN_HIGH()); }
#define FAN_OFF()               ATOMIC_BLOCK(ATOMIC