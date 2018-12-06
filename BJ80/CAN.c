#include <hidef.h>
#include "derivative.h" /* include peripheral declarations */
#include "CAN.h"

/*************************************************************/
/*                        初始化CAN0                         */
/*************************************************************/
void INIT_CAN0(void)
{
    if(CAN0CTL0_INITRQ==0)      // 查询是否进入初始化状态
        CAN0CTL0_INITRQ =1;        // 进入初始化状态
    
    while (CAN0CTL1_INITAK==0);  //等待进入初始化状态
    
    CAN0BTR0_SJW = 0;            //设置同步
    CAN0BTR0_BRP = 3;            //设置波特率
    CAN0BTR1 = 0x1c;             //设置时段1和时段2的Tq个数 ,总线频率为500kb/s
    
    //关闭滤波器
    CAN0IDMR0 = 0xFF;
    CAN0IDMR1 = 0xFF;
    CAN0IDMR2 = 0xFF;
    CAN0IDMR3 = 0xFF;
    CAN0IDMR4 = 0xFF;
    CAN0IDMR5 = 0xFF;
    CAN0IDMR6 = 0xFF;
    CAN0IDMR7 = 0xFF;
    
    CAN0CTL1 = 0xC0;             //使能MSCAN模块,设置为一般运行模式、使用总线时钟源
    
    CAN0CTL0 = 0x00;             //返回一般模式运行
    
    while(CAN0CTL1_INITAK);      //等待回到一般运行模式
    
    while(CAN0CTL0_SYNCH==0);    //等待总线时钟同步
    
    CAN0RIER_RXFIE = 1;          //使能接收中断
}

/*************************************************************/
/*                        初始化CAN1                         */
/*************************************************************/
void INIT_CAN1(void)
{
    if(CAN1CTL0_INITRQ==0)      // 查询是否进入初始化状态
        CAN1CTL0_INITRQ =1;        // 进入初始化状态
    
    while (CAN1CTL1_INITAK==0);  //等待进入初始化状态
    
    CAN1BTR0_SJW = 0;            //设置同步
    CAN1BTR0_BRP = 3;            //设置波特率
    
    
    CAN1BTR1 = 0x1c;     //设置时段1和时段2的Tq个数 ,总线频率为500kb/s
    
    // 关闭滤波器
    CAN1IDMR0 = 0xFF;
    CAN1IDMR1 = 0xFF;
    CAN1IDMR2 = 0xFF;
    CAN1IDMR3 = 0xFF;
    CAN1IDMR4 = 0xFF;
    CAN1IDMR5 = 0xFF;
    CAN1IDMR6 = 0xFF;
    CAN1IDMR7 = 0xFF;
    
    CAN1CTL1 = 0xC0;             //使能MSCAN模块,设置为一般运行模式、使用总线时钟源
    
    CAN1CTL0 = 0x00;             //返回一般模式运行
    
    while(CAN1CTL1_INITAK);      //等待回到一般运行模式
    
    while(CAN1CTL0_SYNCH==0);    //等待总线时钟同步
    
    CAN1RIER_RXFIE = 1;          //使能接收中断
}

/*************************************************************/
/*                        初始化CAN2                         */
/*************************************************************/
void INIT_CAN2(void){
     if(CAN2CTL0_INITRQ==0)      // 查询是否进入初始化状态
         CAN2CTL0_INITRQ =1;        // 进入初始化状态
    
     while (CAN2CTL1_INITAK==0)
         ;  //等待进入初始化状态
    
     CAN2BTR0_SJW = 0;            //设置同步
     CAN2BTR0_BRP = 7;            //设置波特率
    
    
     CAN2BTR1 = 0x1c;     //设置时段1和时段2的Tq个数 ,总线频率为250kb/s
    
     // 关闭滤波器
     CAN2IDMR0 = 0xFF;
     CAN2IDMR1 = 0xFF;
     CAN2IDMR2 = 0xFF;
     CAN2IDMR3 = 0xFF;
     CAN2IDMR4 = 0xFF;
     CAN2IDMR5 = 0xFF;
     CAN2IDMR6 = 0xFF;
     CAN2IDMR7 = 0xFF;
    
     CAN2CTL1 = 0xC0;             //使能MSCAN模块,设置为一般运行模式、使用总线时钟源
    
     CAN2CTL0 = 0x00;             //返回一般模式运行
    
     while(CAN2CTL1_INITAK)
         ;      //等待回到一般运行模式
    
     while(CAN2CTL0_SYNCH==0)
         ;    //等待总线时钟同步
    
     CAN2RIER_RXFIE = 1;          //使能接收中断
 }
 


/*************************************************************/
/*                       CAN0发送                            */
/*************************************************************/
Bool MSCAN0SendMsg(struct can_msg msg)
{
    unsigned char waittime=0;
    unsigned char send_buf, sp;
    
    // 检查数据长度
    if(msg.len > 8)
        return(FALSE);
    
    // 检查总线时钟
    if(CAN0CTL0_SYNCH==0)
        return(FALSE);
    
    send_buf = 0;
    waittime = 5000;
    do{
        // 寻找空闲的缓冲器
        CAN0TBSEL=CAN0TFLG;
        send_buf=CAN0TBSEL;
        waittime--;
        if(!waittime)
            return(FALSE);
    }
    while(!send_buf)
        ;
    
    // 写入标识符
    CAN0TXIDR0 = (unsigned char)(msg.id>>3);
    CAN0TXIDR1 = (unsigned char)(msg.id<<5);
    
    if(msg.RTR)
        // RTR = 阴性
        CAN0TXIDR1 |= 0x10;
    
    // 写入数据
    for(sp = 0; sp < msg.len; sp++)
        *((&CAN0TXDSR0)+sp) = msg.data[sp];
    
    // 写入数据长度
    CAN0TXDLR = msg.len;
    
    // 写入优先级
    CAN0TXTBPR = msg.prty;
    
    // 清 TXx 标志 (缓冲器准备发送)
    CAN0TFLG = send_buf;
    
    return(TRUE);
    
}

/*************************************************************/
/*                       CAN0接收                            */
/*************************************************************/
Bool MSCAN0GetMsg(struct can_msg *msg)
{
    unsigned char sp2;
    
    // 检测接收标志
    if(!(CAN0RFLG_RXF))
        return(FALSE);
    
    // 检测 CAN协议报文模式 （一般/扩展） 标识符
    if(CAN0RXIDR1_IDE)
        // IDE = Recessive (Extended Mode)
        return(FALSE);
    
    // 读标识符
    msg->id = (unsigned int)(CAN0RXIDR0<<3) |
    (unsigned char)(CAN0RXIDR1>>5);
    
    if(CAN0RXIDR1&0x10)
        msg->RTR = TRUE;
    else
        msg->RTR = FALSE;
    
    // 读取数据长度
    msg->len = CAN0RXDLR;
    
    // 读取数据
    for(sp2 = 0; sp2 < msg->len; sp2++)
        msg->data[sp2] = *((&CAN0RXDSR0)+sp2);
    
    // 清 RXF 标志位 (缓冲器准备接收)
    CAN0RFLG = 0x01;
    
    return(TRUE);
}


/*************************************************************/
/*                        CAN1发送                           */
/*************************************************************/
Bool MSCAN1SendMsg(struct can_msg msg)
{
    unsigned char waittime=0;
    unsigned char send_buf, sp;
    
    // 检查数据长度
    if(msg.len > 8)
        return(FALSE);
    
    // 检查总线时钟
    if(CAN1CTL0_SYNCH==0)
        return(FALSE);
    
    send_buf = 0;
    waittime = 5000;
    do
    {
        // 寻找空闲的缓冲器
        CAN1TBSEL=CAN1TFLG;
        send_buf=CAN1TBSEL;
        waittime--;
        if(!waittime) return(FALSE);
    }
    
    while(!send_buf);
    
    // 写入标识符
    CAN1TXIDR0 = (unsigned char)(msg.id>>3);
    CAN1TXIDR1 = (unsigned char)(msg.id<<5);
    
    if(msg.RTR)
        // RTR = 阴性
        CAN1TXIDR1 |= 0x10;
    
    // 写入数据
    for(sp= 0; sp < msg.len;sp++)
        *((&CAN1TXDSR0)+sp) = msg.data[sp];
    
    // 写入数据长度
    CAN1TXDLR = msg.len;
    
    // 写入优先级
    CAN1TXTBPR = msg.prty;
    
    // 清 TXx 标志 (缓冲器准备发送)
    CAN1TFLG = send_buf;
    
    return(TRUE);
    
}

/*************************************************************/
/*                        CAN1接收                           */
/*************************************************************/
Bool MSCAN1GetMsg(struct can_msg *msg)
{
    
    unsigned char sp2;
    
    // 检测接收标志
    if(!(CAN1RFLG_RXF))
        return(FALSE);
    
    // 检测 CAN协议报文模式 （一般/扩展） 标识符
    if(CAN1RXIDR1_IDE)
        // IDE = Recessive (Extended Mode)
        return(FALSE);
    
    // 读标识符
    msg->id = (unsigned int)(CAN1RXIDR0<<3) |
    (unsigned char)(CAN1RXIDR1>>5);
    
    if(CAN1RXIDR1&0x10)
        msg->RTR = TRUE;
    else
        msg->RTR = FALSE;
    
    // 读取数据长度
    msg->len = CAN1RXDLR;
    
    // 读取数据
    for(sp2 = 0; sp2 < msg->len; sp2++)
        msg->data[sp2] = *((&CAN1RXDSR0)+sp2);
    
    // 清 RXF 标志位 (缓冲器准备接收)
    CAN1RFLG = 0x01;
    
    return(TRUE);
}




/*************************************************************/
/*                        CAN2发送                           */
/*************************************************************/
Bool MSCAN2SendMsg(struct can_msg msg)
 {
 
     unsigned char send_buf, sp;
 
     // 检查数据长度
     if(msg.len > 8)
         return(FALSE);
     
     // 检查总线时钟
     if(CAN2CTL0_SYNCH==0)
         return(FALSE);
     
     send_buf = 0;
     do
     {
     // 寻找空闲的缓冲器
     CAN2TBSEL=CAN2TFLG;
     
     send_buf=CAN2TBSEL;
     }
     
     while(!send_buf)
         ;
     
     // 写入标识符
     CAN2TXIDR0 = (unsigned char)(msg.id>>3);
     CAN2TXIDR1 = (unsigned char)(msg.id<<5);
     
     if(msg.RTR)
     // RTR = 阴性
     CAN2TXIDR1 |= 0x10;
     
     // 写入数据
     for(sp= 0; sp < msg.len;sp++)
         *((&CAN2TXDSR0)+sp) = msg.data[sp];
     
     // 写入数据长度
     CAN2TXDLR = msg.len;
     
     // 写入优先级
     CAN2TXTBPR = msg.prty;
     
     // 清 TXx 标志 (缓冲器准备发送)
     CAN2TFLG = send_buf;
     
     return(TRUE);
     
 }

/*************************************************************/
/*                        CAN2接收                           */
/*************************************************************/
Bool MSCAN2GetMsg(struct can_msg *msg)
 {
 
     unsigned char sp2;
     
     // 检测接收标志
     if(!(CAN2RFLG_RXF))
         return(FALSE);
     
     // 检测 CAN协议报文模式 （一般/扩展） 标识符
     if(CAN2RXIDR1_IDE)
     // IDE = Recessive (Extended Mode)
         return(FALSE);
     
     // 读标识符
     msg->id = (unsigned int)(CAN2RXIDR0<<3) |
     (unsigned char)(CAN2RXIDR1>>5);
     
     if(CAN2RXIDR1&0x10)
         msg->RTR = TRUE;
     else
         msg->RTR = FALSE;
     
     // 读取数据长度
     msg->len = CAN2RXDLR;
     
     // 读取数据
     for(sp2 = 0; sp2 < msg->len; sp2++)
         msg->data[sp2] = *((&CAN2RXDSR0)+sp2);
     
     // 清 RXF 标志位 (缓冲器准备接收)
     CAN2RFLG = 0x01;
     
     return(TRUE);
 }
