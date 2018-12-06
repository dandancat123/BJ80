#ifndef operate_H
#define operate_H


#include <hidef.h>

#define ON 1
#define OFF 0
#define DELAY_TIME 50              // AEB系统延时时间的100倍，单位s
#define DRIVER_TOLERANC_EMIN 200   // 驾驶员所能容忍的最小时距的100倍，单位s
#define ADHESIVE_FACTOR 1          // 路面附着影响因数，常规路面取1，湿滑路面大于1
#define HEADER 25                 // 车辆静止时距前车的预设距离的10倍

//void Delay(void);
//void LongDelay(void);
unsigned int AD_Capture(unsigned char chanel);
void SCI0_Send(unsigned char data);
unsigned char SCI0_Receive(void);
int CountSafetyFactor(unsigned int dis, int relativeSpe,unsigned int Spe) ;

#endif
