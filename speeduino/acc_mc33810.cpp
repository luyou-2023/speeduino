#include "acc_mc33810.h"
#include "globals.h"
#include <SPI.h>

// 定义喷油器（Injector）和点火器（Ignition）位的常量
uint8_t MC33810_BIT_INJ1 = 1;
uint8_t MC33810_BIT_INJ2 = 2;
uint8_t MC33810_BIT_INJ3 = 3;
uint8_t MC33810_BIT_INJ4 = 4;
uint8_t MC33810_BIT_INJ5 = 5;
uint8_t MC33810_BIT_INJ6 = 6;
uint8_t MC33810_BIT_INJ7 = 7;
uint8_t MC33810_BIT_INJ8 = 8;

uint8_t MC33810_BIT_IGN1 = 1;
uint8_t MC33810_BIT_IGN2 = 2;
uint8_t MC33810_BIT_IGN3 = 3;
uint8_t MC33810_BIT_IGN4 = 4;
uint8_t MC33810_BIT_IGN5 = 5;
uint8_t MC33810_BIT_IGN6 = 6;
uint8_t MC33810_BIT_IGN7 = 7;
uint8_t MC33810_BIT_IGN8 = 8;

// 定义指向端口和掩码的指针，用于MC33810芯片的片选引脚
volatile PORT_TYPE *mc33810_1_pin_port;
volatile PINMASK_TYPE mc33810_1_pin_mask;
volatile PORT_TYPE *mc33810_2_pin_port;
volatile PINMASK_TYPE mc33810_2_pin_mask;

// 初始化MC33810芯片
void initMC33810(void)
{
    // 设置引脚端口和掩码
    mc33810_1_pin_port = portOutputRegister(digitalPinToPort(pinMC33810_1_CS));
    mc33810_1_pin_mask = digitalPinToBitMask(pinMC33810_1_CS);
    mc33810_2_pin_port = portOutputRegister(digitalPinToPort(pinMC33810_2_CS));
    mc33810_2_pin_mask = digitalPinToBitMask(pinMC33810_2_CS);

    // 将两个MC33810芯片的输出状态设置为关闭（燃油和点火）
    mc33810_1_requestedState = 0;
    mc33810_2_requestedState = 0;
    mc33810_1_returnState = 0;
    mc33810_2_returnState = 0;

    // 设置片选引脚为输出模式
    pinMode(pinMC33810_1_CS, OUTPUT);
    pinMode(pinMC33810_2_CS, OUTPUT);

    // 初始化SPI接口
    SPI.begin();
    // 根据数据手册设置SPI通信参数
	SPI.beginTransaction(SPISettings(6000000, MSBFIRST, SPI_MODE0));

    // 将点火输出设置为GPGD模式（General Purpose Output）
    /*
    0001 = 模式选择命令
    1111 = 设置所有的GD[0...3]输出使用GPGD模式
    00000000 = 剩余的值不使用
    */
    //uint16_t cmd = 0b000111110000;
    uint16_t cmd = 0b0001111100000000;
    // IC1 配置
    MC33810_1_ACTIVE();
    SPI.transfer16(cmd);
    MC33810_1_INACTIVE();
    // IC2 配置
    MC33810_2_ACTIVE();
    SPI.transfer16(cmd);
    MC33810_2_INACTIVE();

    // 禁用开路负载的下拉电流同步（详见MC33810数据手册第31页）
    /*
    0010 = LSD故障命令
    1000 = LSD故障操作为关机（默认）
    1111 = 开路检测故障时激活（默认）
    0000 = 禁用关断时的开路检测（由1111更改为0000）
    */
    cmd = 0b0010100011110000;
    // IC1 配置
    MC33810_1_ACTIVE();
    SPI.transfer16(cmd);
    MC33810_1_INACTIVE();
    // IC2 配置
    MC33810_2_ACTIVE();
    SPI.transfer16(cmd);
    MC33810_2_INACTIVE();
}