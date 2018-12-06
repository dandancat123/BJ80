#include "derivative.h" /* include peripheral declarations */
#include "operate.h"
#include "init.h"


/*************************************************************/
/*                        AD采集函数                         */
/*************************************************************/
unsigned int AD_Capture(unsigned char chanel)
{
    unsigned int AD_data;
    switch(chanel)
    {
        case 0:
            ATD0CTL5 = 0x00;    //转换AD00
            while(!ATD0STAT0_SCF);
            AD_data = ATD0DR0;
            break;
            
        case 1:
            ATD0CTL5 = 0x01;    //转换AD01
            while(!ATD0STAT0_SCF);
            AD_data = ATD0DR0;
            break;
            
        case 2:
            ATD0CTL5 = 0x02;    //转换AD02
            while(!ATD0STAT0_SCF);
            AD_data = ATD0DR0;
            break;
            
        case 3:
            ATD0CTL5 = 0x03;    //转换AD03
            while(!ATD0STAT0_SCF);
            AD_data = ATD0DR0;
            break;
            
        case 4:
            ATD0CTL5 = 0x04;    //转换AD04
            while(!ATD0STAT0_SCF);
            AD_data = ATD0DR0;
            break;
            
        case 5:
            ATD0CTL5 = 0x05;    //转换AD05
            while(!ATD0STAT0_SCF);
            AD_data = ATD0DR0;
            break;
            
        case 6:
            ATD0CTL5 = 0x06;    //转换AD06
            while(!ATD0STAT0_SCF);
            AD_data = ATD0DR0;
            break;
            
        case 7:
            ATD0CTL5 = 0x07;    //转换AD07
            while(!ATD0STAT0_SCF);
            AD_data = ATD0DR0;
            break;
            
        case 8:
            ATD0CTL5 = 0x08;    //转换AD08
            while(!ATD0STAT0_SCF);
            AD_data = ATD0DR0;
            break;
            
        case 9:
            ATD0CTL5 = 0x09;    //转换AD09
            while(!ATD0STAT0_SCF);
            AD_data = ATD0DR0;
            break;
            
        case 10:
            ATD0CTL5 = 0x0a;    //转换AD10
            while(!ATD0STAT0_SCF);
            AD_data = ATD0DR0;
            break;
            
        case 11:
            ATD0CTL5 = 0x0b;    //转换AD11
            while(!ATD0STAT0_SCF);
            AD_data = ATD0DR0;
            break;
            
        case 12:
            ATD0CTL5 = 0x0c;    //转换AD12
            while(!ATD0STAT0_SCF);
            AD_data = ATD0DR0;
            break;
            
        case 13:
            ATD0CTL5 = 0x0d;    //转换AD13
            while(!ATD0STAT0_SCF);
            AD_data = ATD0DR0;
            break;
            
        case 14:
            ATD0CTL5 = 0x0e;    //转换AD14
            while(!ATD0STAT0_SCF);
            AD_data = ATD0DR0;
            break;
            
        case 15:
            ATD0CTL5 = 0x0f;    //转换AD15
            while(!ATD0STAT0_SCF);
            AD_data = ATD0DR0;
            break;
            
    }
    return(AD_data);
}


/*************************************************************/
/*                       串口0发送函数                       */
/*************************************************************/
void SCI0_Send(unsigned char data)
{
    while(!SCI0SR1_TDRE);          // 等待发送数据寄存器（缓冲器）为空
    SCI0DRL = data;
}

/*************************************************************/
/*                       串口0接收函数                       */
/*************************************************************/
unsigned char SCI0_Receive(void)
{
    while(!SCI0SR1_RDRF);          // 等待发送数据寄存器满
    return(SCI0DRL);
}

/*************************************************************/
/*                          计算SafetyFactor                 */
/*************************************************************/

int CountSafetyFactor(unsigned int dis, int relativeSpe,unsigned int Spe)
{
    float safeFactor;
    float brakeDis;
    float speed = (float)(Spe);
    float distance = (float)(dis);
    float relativeSpeed = (float)(relativeSpe);
    float acc = 0;
    
    
    if(relativeSpe<200)acc = 0.0016;
    else if(relativeSpe<300)acc = 0.0016;
    else if(relativeSpe<400)acc = 0.0016;
    else if(relativeSpe<500)acc = 0.0016;
    else if(relativeSpe<600)acc = 0.0016;
    else if(relativeSpe<700)acc = 0.0016;
    else if(relativeSpe<800)acc = 0.0016;
    else if(relativeSpe<900)acc = 0.0016;
    else acc = 0.0016;
    
    if(distance>1000)
        distance =1000;
    if(speed == 0)
        safeFactor=200;
    
    else{
        
        brakeDis= HEADER + (speed*speed-(speed-relativeSpeed)*(speed-relativeSpeed))*acc -0.1499*relativeSpeed+22.177;    // 实验标定值
        safeFactor = 36000*(distance-(brakeDis))/(speed*DRIVER_TOLERANC_EMIN);                                        // 实验标定值
    }
    if(safeFactor>200)
        safeFactor = 200;
    return((int)safeFactor);
}
