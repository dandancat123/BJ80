#define RECRIVE_PC_ID1           0xe0          // 灯光控制
#define RECRIVE_PC_ID            0xe2          // 上位机标识符
#define SEND_TO_PC_ID0           0x30          // 发送
#define SEND_TO_PC_ID1           0x33          // 发送标识符
#define SEND_TO_PC_ID2           0x36          // 发送标识符
#define SEND_TO_PC_ID3           0x39          // 发送标识符

#define BrakePedalOffset_ID       0x111        // 制动踏板位置
#define VehSpd_ID                 0x105        // 车速ID
#define StrWhl_ID                 0x102        // 整车方向盘转角ID
#define VehAcc_ID                 0x1D0        // 整车加速度ID

#define BrakePedalOffset_EV820_ID 0x120        // 制动踏板位置
#define VehSpd_EV820_ID           0x210        // 车速ID
#define StrWhl_EV820_ID           0x080        // 整车方向盘转角ID

#define BrakePedalOffset_H8_ID    0x101        // 制动踏板位置
#define VehSpd_H8_ID              0x13B        // 车速ID
#define StrWhl_H8_ID              0xA1         // 整车方向盘转角ID

#define BrakePedalOffset_650EV_ID    0x211     // 制动踏板位置
#define VehSpd_650EV_ID              0x312     // 车速ID
#define StrWhl_650EV_ID              0x2B0     // 整车方向盘转角ID
#define StrWhl_650Torque_ID          0x278     // 方向盘扭矩


#define VehSpd_baojun_ID             0x22      // 宝骏车速
#define StrWhl_baojun_ID             0x01E5    // 方向盘转角


struct can_msg     //发送CAN信息--发送报文的结构体
{
    unsigned int id;           //ID号
    bool RTR;                  //布尔值
    unsigned char data[8];     //信息
    unsigned char len;         //长度
    unsigned char prty;        //优先级
};

void INIT_CAN0(void);
void INIT_CAN1(void);
void INIT_CAN2(void);
Bool MSCAN0SendMsg(struct can_msg msg);
Bool MSCAN0GetMsg(struct can_msg *msg);
Bool MSCAN1SendMsg(struct can_msg msg);
Bool MSCAN1GetMsg(struct can_msg *msg);
Bool MSCAN2SendMsg(struct can_msg msg);
Bool MSCAN2GetMsg(struct can_msg *msg);

