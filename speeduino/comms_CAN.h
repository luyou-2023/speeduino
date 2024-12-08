#ifndef COMMS_CAN_H
#define COMMS_CAN_H
#if defined(NATIVE_CAN_AVAILABLE)

/**
详细分析：
CAN 消息定义
CAN_BMW_ASC1 (0x153):

Rx（接收消息）：
表明这是从 BMW 的 ACS 单元（通常是动态稳定控制系统或牵引力控制系统）接收到的消息。
包含速度信息，这可能用于调整发动机负载管理或牵引力控制。
CAN_BMW_DME1 (0x316):

Tx（发送消息）：
包含 RPM（发动机转速）信息，可能由发动机管理单元（DME, Digital Motor Electronics）发送给其他模块。
CAN_BMW_DME2 (0x329) 和 CAN_BMW_DME4 (0x545):

Tx（发送消息）：
包含 CLT（冷却液温度，Coolant Temperature）和 TPS（节气门位置传感器）信息。
这些消息可能是从 DME 发送给仪表集群或其他模块，用于监控或进一步处理。
CAN_BMW_ICL2 (0x613) 和 CAN_BMW_ICL3 (0x615):

通常与 BMW 的仪表集群（Instrument Cluster）相关，可能用于显示转速、速度、温度等数据。
与 Speeduino 的关系
TPS 的重要性：

TPS 数据 是油门踏板开度的重要参数，影响喷油量和点火时机的计算。
如果使用 Speeduino 控制发动机，TPS 数据可能直接通过模拟信号输入（传统 TPS），或者通过 CAN 总线接收（例如通过 CAN_BMW_DME2 或 CAN_BMW_DME4 消息解析）。
通过 CAN 读取 TPS 数据：

如果车辆的 TPS 数据是通过 CAN 总线提供的，Speeduino 需要配置代码来监听 0x329 或 0x545 消息。
在解析消息时，Speeduino 应提取 CAN 数据帧中的对应字节，解码出 TPS 的百分比值。
数据融合：

Speeduino 可以结合 CAN 总线数据（如 TPS 和 CLT）与本地传感器数据实现更全面的发动机控制。
***/
//For BMW e46/e39/e38, rover and mini other CAN instrument clusters
#define CAN_BMW_ASC1 0x153 //Rx message from ACS unit that includes speed
#define CAN_BMW_DME1 0x316 //Tx message that includes RPM
#define CAN_BMW_DME2 0x329 //Tx message that includes CLT and TPS
#define CAN_BMW_DME4 0x545 //Tx message that includes CLT and TPS
#define CAN_BMW_ICL2 0x613
#define CAN_BMW_ICL3 0x615

//For VAG CAN instrument clusters
#define CAN_VAG_RPM 0x280
#define CAN_VAG_VSS 0x5A0

#define CAN_WBO_RUSEFI 1

#define TS_CAN_OFFSET 0x100

void initCAN();
int CAN_read();
void CAN_write();
void sendBMWCluster();
void sendVAGCluster();
void receiveCANwbo();
void DashMessages(uint16_t DashMessageID);
void can_Command(void);
void obd_response(uint8_t therequestedPID , uint8_t therequestedPIDlow, uint8_t therequestedPIDhigh);
void readAuxCanBus();

extern CAN_message_t outMsg;
extern CAN_message_t inMsg;

#endif
#endif // COMMS_CAN_H
