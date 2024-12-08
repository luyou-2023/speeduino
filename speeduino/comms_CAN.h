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

/**
CAN（Controller Area Network）是一种用于汽车内部电子控制单元（ECUs）之间通信的网络协议。它允许车辆中的各种传感器和执行器在没有复杂布线的情况下交换信息。通过解读CAN消息，可以获取到车辆的各种状态信息，如速度、转速、温度等。

对于你提供的BMW车型（E46/E39/E38）以及Rover和Mini的其他CAN仪表集群定义，每个标识符代表了一个特定的数据报文或信号，这些报文在车辆的CAN总线上进行传输。下面是对这些定义的一个简单解读：

CAN_BMW_ASC1 0x153：这是从ACS（自动稳定性控制系统）单元接收的消息，包含了车速信息。
CAN_BMW_DME1 0x316：这是一个发送的消息，包含了发动机转速（RPM）的信息。DME指的是数字式发动机管理系统（Digital Motor Electronics）。
CAN_BMW_DME2 0x329：这也是一个来自DME的发送消息，包含了冷却液温度（CLT）和节气门位置传感器（TPS）的信息。
CAN_BMW_DME4 0x545：与CAN_BMW_DME2类似，这个消息也包含了冷却液温度（CLT）和节气门位置传感器（TPS）的信息，可能是为不同的情况或由不同的ECU发送。
CAN_BMW_ICL2 0x613 和 CAN_BMW_ICL3 0x615：这两个标识符表示了来自仪表集群（Instrument Cluster）的消息，但没有提供具体的信号内容说明。ICL可能是指仪表集群灯光（Instrument Cluster Lighting），但这只是猜测，具体含义需要查阅相关文档。
要完全理解这些CAN消息的内容，通常需要有车辆制造商提供的详细规范或数据手册，因为不同制造商甚至不同车型之间的CAN消息格式和内容可能会有所不同。此外，解读CAN消息往往还需要专门的工具和软件来捕捉和解析这些信息。如果你有兴趣深入了解或者进行开发工作，建议寻找相关的技术文档或使用专业的诊断设备。

TPS，即Throttle Position Sensor（节气门位置传感器），并不是直接控制油门踏板的，但它与油门踏板的操作密切相关。TPS安装在节气门体上，用于检测节气门开度的位置，并将此信息转换为电信号发送给发动机控制单元（ECU）。ECU根据接收到的信号来调整燃油喷射量和点火时刻等，以确保发动机运行在最佳状态。

当驾驶员踩下油门踏板时，实际上是通过机械连接或电子信号（在电控油门系统中）影响节气门的开度。现代汽车通常采用电控油门系统，也称为“drive-by-wire”，其中油门踏板位置由一个或多个传感器监测，这些传感器将踏板位置的信息发送给ECU。然后，ECU会计算所需的节气门开度，并相应地控制电动马达或其他执行器来调整节气门的位置。

因此，虽然TPS不是直接由油门踏板控制的，但它确实反映了由于油门踏板操作引起的节气门实际开度变化。这对于发动机管理系统来说是非常重要的反馈信息，因为它帮助确定驾驶者的意图以及如何适当地调整发动机输出。
**/
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
