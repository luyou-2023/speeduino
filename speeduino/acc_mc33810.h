#ifndef MC33810_H
#define MC33810_H

#include <SPI.h>
#include "globals.h"
#include BOARD_H // 这个头文件在 globals.h 中定义，并不是一个实际文件。

/**
acc_mc33810.h 的作用主要是定义与 MC33810 芯片相关的 Arduino 引脚和控制逻辑。这些定义和设置并不直接与 MC33810 芯片本身相关，而是用于控制 MC33810 的信号的发送方式和接口。以下是详细说明：

1. 定义引脚和控制逻辑
引脚定义：acc_mc33810.h 中定义了 Arduino 的引脚，这些引脚用于控制和与 MC33810 芯片通信。比如，定义了控制喷油器和点火器的引脚。
控制逻辑：代码中设置了这些引脚的模式和状态，以便 Arduino 可以通过这些引脚向 MC33810 发送控制信号。例如，设置引脚为输出模式，以便 Arduino 可以发送数字信号控制 MC33810。
2. 发送控制信号
SPI 接口：通常，MC33810 芯片通过 SPI 总线与 Arduino 进行通信。acc_mc33810.h 文件中可能定义了 SPI 的设置和通信逻辑，使 Arduino 能够通过 SPI 向 MC33810 发送命令。
控制命令：Arduino 通过定义好的引脚和 SPI 接口向 MC33810 发送控制命令。这些命令控制 MC33810 驱动发动机的喷油器和点火线圈。
3. 连接和集成
物理连接：MC33810 芯片的引脚（如电源、地线、SPI 接口引脚等）与 Arduino 通过物理线连接。acc_mc33810.h 文件中定义的引脚是 Arduino 上的引脚，而这些引脚通过导线或电缆连接到 MC33810 芯片的相应引脚。
集成：MC33810 芯片的作用是将 Arduino 发出的控制信号转换为可以驱动喷油器和点火线圈的信号。因此，Arduino 控制信号通过低压线（通常是信号线和电源线）传输到 MC33810 芯片上，MC33810 再将这些信号用于实际的发动机控制。
总结
acc_mc33810.h 文件中的定义是为了确保 Arduino 能正确地与 MC33810 芯片通信和控制。它定义了 Arduino 上用于发送控制信号的引脚和设置。这些引脚的信号通过连接线传输到 MC33810 芯片，后者将信号用于控制发动机的喷油器和点火线圈。因此，MC33810 芯片作为一个控制执行器模块与 Arduino 进行通信，并将 Arduino 的控制命令转化为发动机的实际操作。
**/

// 定义两个 IC 的引脚端口和掩码
extern volatile PORT_TYPE *mc33810_1_pin_port;
extern volatile PINMASK_TYPE mc33810_1_pin_mask;
extern volatile PORT_TYPE *mc33810_2_pin_port;
extern volatile PINMASK_TYPE mc33810_2_pin_mask;

// 定义命令字节
static const uint8_t MC33810_ONOFF_CMD = 0x30; //48 十进制
// 定义两个 IC 的请求和返回状态，存储当前的点火和喷油的状态
static volatile uint8_t mc33810_1_requestedState; // 第一个 IC 的当前请求状态
static volatile uint8_t mc33810_2_requestedState; // 第二个 IC 的当前请求状态
static volatile uint8_t mc33810_1_returnState; // 第一个 IC 的当前返回状态
static volatile uint8_t mc33810_2_returnState; // 第二个 IC 的当前返回状态

// 初始化 MC33810 的函数声明
void initMC33810(void);

// 激活和停用两个 IC 的宏定义
#define MC33810_1_ACTIVE() (*mc33810_1_pin_port &= ~(mc33810_1_pin_mask))
#define MC33810_1_INACTIVE() (*mc33810_1_pin_port |= (mc33810_1_pin_mask))
#define MC33810_2_ACTIVE() (*mc33810_2_pin_port &= ~(mc33810_2_pin_mask))
#define MC33810_2_INACTIVE() (*mc33810_2_pin_port |= (mc33810_2_pin_mask))

// 喷油器的默认值，可能在初始化时根据具体板卡配置被更改
extern uint8_t MC33810_BIT_INJ1;
extern uint8_t MC33810_BIT_INJ2;
extern uint8_t MC33810_BIT_INJ3;
extern uint8_t MC33810_BIT_INJ4;
extern uint8_t MC33810_BIT_INJ5;
extern uint8_t MC33810_BIT_INJ6;
extern uint8_t MC33810_BIT_INJ7;
extern uint8_t MC33810_BIT_INJ8;

// 点火器的默认值
extern uint8_t MC33810_BIT_IGN1;
extern uint8_t MC33810_BIT_IGN2;
extern uint8_t MC33810_BIT_IGN3;
extern uint8_t MC33810_BIT_IGN4;
extern uint8_t MC33810_BIT_IGN5;
extern uint8_t MC33810_BIT_IGN6;
extern uint8_t MC33810_BIT_IGN7;
extern uint8_t MC33810_BIT_IGN8;

// 打开各喷油器的宏定义
#define openInjector1_MC33810() MC33810_1_ACTIVE(); BIT_SET(mc33810_1_requestedState, MC33810_BIT_INJ1); mc33810_1_returnState = SPI.transfer16(word(MC33810_ONOFF_CMD, mc33810_1_requestedState)); MC33810_1_INACTIVE()
#define openInjector2_MC33810() MC33810_1_ACTIVE(); BIT_SET(mc33810_1_requestedState, MC33810_BIT_INJ2); mc33810_1_returnState = SPI.transfer16(word(MC33810_ONOFF_CMD, mc33810_1_requestedState)); MC33810_1_INACTIVE()
#define openInjector3_MC33810() MC33810_1_ACTIVE(); BIT_SET(mc33810_1_requestedState, MC33810_BIT_INJ3); mc33810_1_returnState = SPI.transfer16(word(MC33810_ONOFF_CMD, mc33810_1_requestedState)); MC33810_1_INACTIVE()
#define openInjector4_MC33810() MC33810_1_ACTIVE(); BIT_SET(mc33810_1_requestedState, MC33810_BIT_INJ4); mc33810_1_returnState = SPI.transfer16(word(MC33810_ONOFF_CMD, mc33810_1_requestedState)); MC33810_1_INACTIVE()
#define openInjector5_MC33810() MC33810_2_ACTIVE(); BIT_SET(mc33810_2_requestedState, MC33810_BIT_INJ5); mc33810_2_returnState = SPI.transfer16(word(MC33810_ONOFF_CMD, mc33810_2_requestedState)); MC33810_2_INACTIVE()
#define openInjector6_MC33810() MC33810_2_ACTIVE(); BIT_SET(mc33810_2_requestedState, MC33810_BIT_INJ6); mc33810_2_returnState = SPI.transfer16(word(MC33810_ONOFF_CMD, mc33810_2_requestedState)); MC33810_2_INACTIVE()
#define openInjector7_MC33810() MC33810_2_ACTIVE(); BIT_SET(mc33810_2_requestedState, MC33810_BIT_INJ7); mc33810_2_returnState = SPI.transfer16(word(MC33810_ONOFF_CMD, mc33810_2_requestedState)); MC33810_2_INACTIVE()
#define openInjector8_MC33810() MC33810_2_ACTIVE(); BIT_SET(mc33810_2_requestedState, MC33810_BIT_INJ8); mc33810_2_returnState = SPI.transfer16(word(MC33810_ONOFF_CMD, mc33810_2_requestedState)); MC33810_2_INACTIVE()

// 关闭各喷油器的宏定义
#define closeInjector1_MC33810() MC33810_1_ACTIVE(); BIT_CLEAR(mc33810_1_requestedState, MC33810_BIT_INJ1); mc33810_1_returnState = SPI.transfer16(word(MC33810_ONOFF_CMD, mc33810_1_requestedState)); MC33810_1_INACTIVE()
#define closeInjector2_MC33810() MC33810_1_ACTIVE(); BIT_CLEAR(mc33810_1_requestedState, MC33810_BIT_INJ2); mc33810_1_returnState = SPI.transfer16(word(MC33810_ONOFF_CMD, mc33810_1_requestedState)); MC33810_1_INACTIVE()
#define closeInjector3_MC33810() MC33810_1_ACTIVE(); BIT_CLEAR(mc33810_1_requestedState, MC33810_BIT_INJ3); mc33810_1_returnState = SPI.transfer16(word(MC33810_ONOFF_CMD, mc33810_1_requestedState)); MC33810_1_INACTIVE()
#define closeInjector4_MC33810() MC33810_1_ACTIVE(); BIT_CLEAR(mc33810_1_requestedState, MC33810_BIT_INJ4); mc33810_1_returnState = SPI.transfer16(word(MC33810_ONOFF_CMD, mc33810_1_requestedState)); MC33810_1_INACTIVE()
#define closeInjector5_MC33810() MC33810_2_ACTIVE(); BIT_CLEAR(mc33810_2_requestedState, MC33810_BIT_INJ5); mc33810_2_returnState = SPI.transfer16(word(MC33810_ONOFF_CMD, mc33810_2_requestedState)); MC33810_2_INACTIVE()
#define closeInjector6_MC33810() MC33810_2_ACTIVE(); BIT_CLEAR(mc33810_2_requestedState, MC33810_BIT_INJ6); mc33810_2_returnState = SPI.transfer16(word(MC33810_ONOFF_CMD, mc33810_2_requestedState)); MC33810_2_INACTIVE()
#define closeInjector7_MC33810() MC33810_2_ACTIVE(); BIT_CLEAR(mc33810_2_requestedState, MC33810_BIT_INJ7); mc33810_2_returnState = SPI.transfer16(word(MC33810_ONOFF_CMD, mc33810_2_requestedState)); MC33810_2_INACTIVE()
#define closeInjector8_MC33810() MC33810_2_ACTIVE(); BIT_CLEAR(mc33810_2_requestedState, MC33810_BIT_INJ8); mc33810_2_returnState = SPI.transfer16(word(MC33810_ONOFF_CMD, mc33810_2_requestedState)); MC33810_2_INACTIVE()

// 切换各喷油器状态的宏定义
#define injector1Toggle_MC33810() MC33810_1_ACTIVE(); BIT_TOGGLE(mc33810_1_requestedState, MC33810_BIT_INJ1); mc33810_1_returnState = SPI.transfer16(word(MC33810_ONOFF_CMD, mc33810_1_requestedState)); MC33810_1_INACTIVE()
#define injector2Toggle_MC33810() MC33810_1_ACTIVE(); BIT_TOGGLE(mc33810_1_requestedState, MC33810_BIT_INJ2); mc33810_1_returnState = SPI.transfer16(word(MC33810_ONOFF_CMD, mc33810_1_requestedState)); MC33810_1_INACTIVE()
#define injector3Toggle_MC33810() MC33810_1_ACTIVE(); BIT_TOGGLE(mc33810_1_requestedState, MC33810_BIT_INJ3); mc33810_1_returnState = SPI.transfer16(word(MC33810_ONOFF_CMD, mc33810_1_requestedState)); MC33810_1_INACTIVE()
#define injector4Toggle_MC33810() MC33810_1_ACTIVE(); BIT_TOGGLE(mc33810_1_requestedState, MC33810_BIT_INJ4); mc33810_1_returnState = SPI.transfer16(word(MC33810_ONOFF_CMD, mc33810_1_requestedState)); MC33810_1_INACTIVE()
#define injector5Toggle_MC33810