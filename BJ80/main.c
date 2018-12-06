/*---------------------------------------------------------*/
/************************************************************
 液压制动电动助力PID控制
 注：
 ************************************************************/
/*---------------------------------------------------------*/
//#include <hidef.h>      /* common defines and macros */
//#include "derivative.h"      /* derivative-specific definitions */
#include "init.h"
#include "operate.h"
#include "CAN.h"
#include "math.h"
#include "string.h"
#include "COP_Drv.h"


#define HALF_VECHILE_WIDTH_MAX             400                  //半车宽，单位cm
#define HALF_VECHILE_WIDTH_MIN             280                  //半车宽，单位cm
#define REF_ACC                            680                  //控制加速度预设值
#define PI                                 3.141592             //圆周率
#define D                                  1.1                  //前轮到车头的距离
#define OBJECT_NUM                         64                   //毫米波雷达的特征数
#define SPEED_IS_ZERO                      4                   //速度单位待定
#define SPEED_IS_Standby                   80                   //速度单位待定
#define PRESSURE_THRESHOLD                 10                   //制动完全解除压力阈值
#define PRE_BRAKE_PRESSURE_THRESHOLD       120                  //预制动压力阈值
#define BRAKE_PEDAL_SOFT_INTERVENTION      1020                 //制动踏板有间歇，标定得到，当制动踏板1020时压力反馈为115，为最大制动压力的10%
#define WHEEL_SPEED_MAX                    60                   //急打方向盘时方向盘转速阈值

#define SAFETY_FACTOR_BOOST_BRAKE          0                    //SafetyFactor*100自主制动时的安全因子
#define SAFETY_FACTOR_INTERVAL_BRAKE       50                   //SafetyFactor*100间歇制动时的安全因子
#define SAFETY_FACTOR_PRE_BRAKE            100                  //SafetyFactor*100预制动时的安全因子

#define Motor_PRE_BRAKE_DUTY_RATIO         200                  //ESC电机预制动PWM波占空比
#define Motor_INTERVAL_BRAKE_DUTY_RATIO    420                  //ESC电机间歇制动PWM波占空比
#define PWM_PERIOD                         500
unsigned int  m_iControlACC = 0;                                 //目标减速度


#define ReceivePC_ID           0x31       // 上位机标识符
#define SendPC_ID0             0x21       // 发送标识符
#define SendPC_ID1             0x22       // 发送标识符
#define data_len_TX            8          // 发送数据长度
#define PWM_MAX   760
#define PWM_MIN   370
#define PRESSURE_MAX 1000                 // 200+80*10
#define PRESSURE_MIN 200                  // 200+80*0
#define INIT_PRESSURE 220
#define INTERRUPTTIME 50
#define MotorDuty PWMDTY01

#define ABS(a) ((a)>0?(a):(-(a)))
#define SIGN(a) ((a)>0?1:((a)<0?-1:0))

enum Status{
    NORMAL,
    BOOSTER_PRESSURE,
    HOLD_PRESSURE,
    RELEASE_PRESSURE,
};

struct ReceiveInfo {
    unsigned char m_cLeftTurnLight;
    unsigned char m_cRightTurnLight;
    unsigned char m_cHighBeam;
    unsigned char m_cLowBeam;
    unsigned char m_cWidthLight;
    unsigned char m_cBrakeLight;
    unsigned char m_cHornCtrl;
    unsigned char m_cAccPedal;
};


struct ReceiveInfo m_sReceiveInfo;

struct CANsendInfo{
    unsigned char flag;    //标识位
    unsigned char id;      //ID号
    unsigned char data[8]; //数据信息
};
struct CANsendInfo m_cSendInfo[8];  //需要发送几帧数据

unsigned char tasksta[8];            //一个周期内需要执行的任务事件
unsigned char taskid=0;              //任务号
//unsigned char tasksta=0;           //任务状态
unsigned int sysTimer=0;             //系统基准时间
unsigned int CanSendTimer=0;         //Can发送时间
unsigned int CanSendFlag=0;          //Can发送标志
unsigned int Bakesignal = 0;         //这个什么？
//unsigned char m_cBrakePedal=0;
unsigned int AutoControl_Mode = 0;   //这个什么？
unsigned int AutoControl_ModeLast = 0;
struct can_msg Can0_msg_send, Can1_msg_send, Can2_msg_send;
struct can_msg Can0_msg_get, Can1_msg_get, Can2_msg_get;

//unsigned char AutoTest = 1;
unsigned int SendOutFlag = 0;
//unsigned char AccPedalOpening=0;
unsigned int BrakeMotorCurrentSenseAD;                   //这个是什么？
unsigned int Pedalforce1AD;                             //这个是什么？
//unsigned int Pedalforce2AD;
unsigned int MasterCylinderPrsSenseAD;                   //这个是什么？
unsigned int CylinderLocationAD;                         //这个是什么？
unsigned int PedalLocationAD2;                           //这个是什么？
//unsigned int PedalLocationAD3;
//unsigned int m_iMDCTimeCnt=0;
//unsigned char AutoSaveDataFlag=0;;
unsigned char ReceiveData0[8],ReceiveData1[8];
//unsigned char ReceiveDataCAN0[8];

unsigned int Preset_time=0;                         //这是什么时间？
unsigned int Preset_time_Last=0;                    //这是什么时间？
unsigned int AutoCurrentSet_Time=0;
signed int PedalAction;        //驾驶员踩踏板行为

unsigned int Timer=0;          //程序计时时间时间
unsigned int ECU_Timer = 0;
//unsigned int CanTimer=0;         //Can发送时间
signed int motorDuty=0;
signed int motorDutyLast=0;
unsigned  int AutoOnOffSignal = 0;          //这是什么？

signed int MCTimer=0;
signed int SinTimer=0;

unsigned int VechileSpeed;
unsigned char m_cBrakePedal=0;  //这个是什么？

float PedalForce;              //踏板力值
float BrakeMotorCurrent;       //助力电机电流
float CylinderLocation;        //主缸推杆行程
float PedalLocation;           //踏板推杆行程
float PedalLocationLast;       //上一次踏板行程
float MasterCylinderPrs;       //主缸压力
float Set_Prs=0.0;
float Set_Current;             //电机电流给定值
float CurrentFeedback;         //电机电流反馈值
float SetCurrentPID;           //将PID的目标电流输出，用作数据分析
float CurrentWentai;           //电机稳态误差补偿
float CurrentMoca;             //摩擦补偿电流值
float PedalSpeed;              //踏板推杆速度
float Clearance;               //间隙

int m_iMotorDuty;


/*************************************************************/
/*                    毫米波雷达数据处理                     */
/*************************************************************/
unsigned char RelevantData[OBJECT_NUM][8];
unsigned int ObjectID[OBJECT_NUM];           //障碍ID
int ObjectAngleInt = 0;
int ObjectRangeInt = 0;
int ObjectSpeedInt = 0;
float ObjectRange[OBJECT_NUM];               //障碍距离
float ObjectAngle[OBJECT_NUM];               //障碍所在角度
float ObjectSpeed[OBJECT_NUM];               //障碍速度
float ObjectRangeTmp[OBJECT_NUM];            //障碍距离缓存
float ObjectAngleTmp[OBJECT_NUM];            //障碍所在角度缓存
float ObjectSpeedTmp[OBJECT_NUM];            //障碍速度缓存

float ObjectRangeFliter[OBJECT_NUM];
float ObjectRangeFliterLast[OBJECT_NUM];
unsigned int DataIndex = 0;
unsigned char MaxIndexLabel = 0;
unsigned char ProcessRadarData = 0;
unsigned char NoUpdateDistanceIndex = 0;
unsigned int OilPressureAD;
int m_iRelativeSpeed = 0;                                        //与前车相对速度，距离减小为正
int m_iSafetyFactor = 200;                                       //SafetyFactor数值的10倍

unsigned int m_iWheelAngleMaxS1Record = 0;
//AEB方向盘角速度第一次大于干预阈值时方向盘转角记录

unsigned int m_iWheelAngleMaxS2Record = 0;
//AEB方向盘角速度大于干预阈值时下一个采样方向盘转角记录值

unsigned char m_cIsEmergencySteeringWheel = 0;
//            m_cIsEmergencySteeringWheel != 0；处于急转向干预
//            m_cIsEmergencySteeringWheel  = 0；未处于急转向干预
unsigned char m_cWheelSpeedMaxRecord = 0;                        //AEB干预期间方向盘转速的最大值
unsigned char m_cSingle = 1;                                     //数码管扫描段
unsigned char m_cDataReceive;                                    //串口接收的数据
unsigned char m_cStatus = 0;                                     //m_cStatus==0,Standby
//m_cStatus==1,Pre brake
//m_cStatus==2,Interval brake
//m_cStatus==3,Boost brake
//m_cStatus==4,Release brake

unsigned char m_cBoostBrake = 0;
//m_cBoostBrake=0,正常状态;m_cBoostBrake=1，全力制动状态，速度为零时设为零

unsigned char m_cSpeedStandbyCount = 0;                          //速度大于AEB作用阈值的持续时间
unsigned char m_cAutoSaveLabel = 0;                              //m_cAutoSaveLabel == 1，上位机开始记录需要存储的数据
//m_cAutoSaveLabel == 2，上位机存储数据
unsigned char m_cAutoSaveLabelCount = 0;                         //AEB主动制动结束延迟退出计数器
unsigned int m_iPressureGoalPWM = 0;                             //目标制动压力占空比
unsigned int m_iPressureGoalPedal = 0;                           //目标制动压力制动踏板开度
unsigned char m_cIntervalBrakeCount = 0;                         //间隙制动计数器
unsigned char m_cBoostBrakeDutyRatioIncr = 0;                    //主动增压时的增压值=m_cBoostBrakeDutyRatioIncr*2500/100
unsigned char Alarmlabel = 0;

union  SteerWheelAngleUnion{
    unsigned char m_cSteerWheelAngleHL[2];
    unsigned int m_iSteerWheelAngle;
};

union  SteerWheelSpeedUnion{
    unsigned char m_cSteerWheelSpeedHL[2];
    unsigned int m_iSteerWheelSpeed;
};

union  AcceleratorPedalOffsetUnion{
    unsigned char m_cAcceleratorPedalOffsetHL[2];
    unsigned int m_iAcceleratorPedalOffset;
};

union  BrakePedalOffsetUnion{                                    //这个是什么？
    unsigned char m_cBrakePedalOffsetHL[2];
    unsigned int m_iBrakePedalOffset;
};

union  DistanceUnion{
    unsigned char m_cDistanceHL[2];
    unsigned int m_iDistance;
};

union FrontBrakePressure{
    unsigned char m_cFrontBrakePressureHL[2];
    unsigned int m_iFrontBrakePressure;
};

union RearBrakePressure{
    unsigned char m_cRearBrakePressureHL[2];
    unsigned int m_iRearBrakePressure;
};
union SpeedUnion{
    unsigned char m_cSpeedHL[2];
    unsigned int m_iSpeed;
};

union AccUnion{                                                  //这个是什么？
    unsigned char m_cAccHL[2];
    unsigned int m_iAcc;
};


union DistanceUnion m_uDistanceTmp[11];                           //记录前十周期内的距离值

union SteerWheelAngleUnion m_uSteerWheelAngle;                    //方向盘转角
union SteerWheelSpeedUnion m_uSteerWheelSpeed;                    //方向盘转速
union AcceleratorPedalOffsetUnion m_uAcceleratorPedalOffset;      //加速踏板
union AcceleratorPedalOffsetUnion m_uAcceleratorPedalOffsetLast;  //上一次加速踏板
union BrakePedalOffsetUnion m_uBrakePedalOffset;                  //制动踏板
union BrakePedalOffsetUnion m_uBrakePedalOffsetLast;              //上一次制动踏板
union FrontBrakePressure m_uFrontBrakePressure;                   //前桥比例继动阀输出压力
union FrontBrakePressure m_uFrontBrakePressureLast;               //前桥比例继动阀上一次输出压力
union RearBrakePressure m_uRearBrakePressure;                     //后桥比例继动阀输出压力
union RearBrakePressure m_uRearBrakePressureLast;                 //后桥比例继动阀上一次输出压力

union DistanceUnion m_uDisTmp;                                    //车前障碍物最小距离
union DistanceUnion m_uDistance;                                  //车前障碍物最小距离
unsigned int ObjectDistance;
unsigned int ObjectFindflag;
union DistanceUnion m_uDistanceLast;                              //车前障碍物上一次最小距离
union DistanceUnion m_uDistanceLast1;                             //车前障碍物上上一次最小距离
union SpeedUnion m_uSpeed;                                        //车速
union SpeedUnion m_uSpeedLast;                                    //上一次车速
union SpeedUnion m_uRelativeSpeed;                                //用于相对速度数据采集
union AccUnion m_uAcc;
unsigned int m_iRelativeSpeedMax;                                 //记录介入第三阶段时的压力值
enum RelayValveOpertion{
    Relay_Valve_Standby,
    Relay_Valve_PreBrake,
    Relay_Valve_IntervalBrake,
    Relay_Valve_BoostBrake,
    Relay_Valve_ReleaseBrake
};

enum AEBStatus{
    AEB_STANDBY,
    AEB_PRE_BRAKE,
    AEB_INTERVAL_BRAKE,
    AEB_BOOST_BRAKE,
    AEB_RELEASE_BRAKE
};

enum PWMChanel{
    FRONT_PWM_CHANNEL,
    REAR_PWM_CHANNEL
};

/*Kalman距离滤波参数*/
long m_lDisPError = 100000;
long m_lDisQError = 10;
long m_lDisRError = 200;
long m_lDisKGain = 0;
long m_lDisPreData = 0;
long m_lLastDis = 0;
/*Kalman距离滤波参数*/

/*Kalman距离滤波参数*/
long m_lDPError = 100000;
long m_lDQError = 10;
long m_lDRError = 200;
long m_lDKGain = 0;
long m_lDPreData = 0;
long m_lLastD = 0;
/*Kalman距离滤波参数*/

/*************************************************************/
/*                      定义PID参数                          */
/*************************************************************/
float PID_KPVALUE = 0.01;                  // 积分因子
float PID_KIVALUE = 1;                     // 比例因子
float PID_KDVALUE = 0;                     // 微分因子
signed int PID_MAX = 980;                  // 压力反馈最大值,压力反馈最大值0.745Mpa
signed int PID_MIN = 0;                    // 压力反馈最小值
signed int PID_DEADLINE = 8;               // 设置死区范围

/*************************************************************/
/*                      定义PID数据结构                      */
/*************************************************************/
typedef struct PID
{
    signed int m_iRef;                    // ACC PID，ACC设定值
    signed int m_iFeedBack;               // ACC反馈设定值
    
    signed int m_iPreError;               // ACC PID，前一次压力误差,vi_Ref - m_iFeedBack
    signed int m_iPreDerror;              // ACC PID，前一次压力误差之差，d_error-PreDerror;
    
    float m_fKp;                          // ACC PID，积分因子
    float m_fKi;                          // ACC PID，比例因子
    float m_fKd;                          // ACC PID，微分因子
    
    signed int m_iPreU;                   // ACC预设值
    
}PID;
/*************************************************************/
/*                      定义PID数据初始化                    */
/*************************************************************/

PID sPID;
void PIDInit(void)
{
    sPID.m_iRef      = REF_ACC ;         // ACC设定值,0.62g
    sPID.m_iFeedBack = 0 ;               // ACC反馈值
    
    sPID.m_iPreError = 0 ;               // 前一次ACC误差,,vi_Ref - m_iFeedBack
    sPID.m_iPreDerror= 0 ;               // 前一次ACC误差之差，d_error-PreDerror;
    
    sPID.m_fKp = PID_KPVALUE;
    sPID.m_fKi = PID_KIVALUE;
    sPID.m_fKd = PID_KDVALUE;
    
    sPID.m_iPreU = 0 ;                   // ACC控制输出值
}

/*************************************************************/
/*                      PID调整值计算                        */
/*************************************************************/
signed int PIDCalc(void)
{
    
    signed int prePWM;
    signed int  error,d_error,dd_error;
    
    error = (signed int)(sPID.m_iRef - sPID.m_iFeedBack);        //当前偏差
    d_error = error - sPID.m_iPreError;                          //一阶偏差
    dd_error = d_error - sPID.m_iPreDerror;                      //二阶偏差
    sPID.m_iPreError = error;                                    //存储当前偏差
    sPID.m_iPreDerror = d_error;                                 //存储一阶偏差
    
    
    if( ( error < PID_DEADLINE ) && ( error > -PID_DEADLINE ) )  //设置调节死区
        sPID.m_iPreU = sPID.m_iPreU;
    
    else
        sPID.m_iPreU = (signed int)(sPID.m_fKp*d_error + sPID.m_fKi*error  + sPID.m_fKd*dd_error);
    
    
    if( sPID.m_iPreU >= PID_MAX )                                //防止调节最高溢出
        sPID.m_iPreU = PID_MAX;
    
    if( sPID.m_iPreU <= PID_MIN )                                //防止调节最低溢出
        sPID.m_iPreU = PID_MIN;
    
    prePWM = sPID.m_iPreU;
    if(prePWM>PID_MAX)
        prePWM = PID_MAX;
    
    return  prePWM;                                              //返回预调节占空比
    
}


/*************************************************************/
/*                       初始化变量                          */
/*************************************************************/
void Initialize(void)
{
    unsigned int i;
    for(i=0;i<OBJECT_NUM;i++){
        ObjectRange[i] = 0;
        ObjectAngle[i] = 0;
        ObjectSpeed[i] = 0;
        ObjectRangeFliter[i] = 0;
        ObjectRangeFliterLast[i] = 0;
    }
    m_uSteerWheelAngle.m_iSteerWheelAngle = 0;
    m_uSteerWheelSpeed.m_iSteerWheelSpeed = 0;
    m_uAcceleratorPedalOffset.m_iAcceleratorPedalOffset = 0;
    m_uBrakePedalOffset.m_iBrakePedalOffset = 0;
    m_uDistance.m_iDistance = 12000;
    m_uDistanceLast.m_iDistance = 12000;
    m_uRelativeSpeed.m_iSpeed = 0;
    m_iRelativeSpeedMax = 0;
    PIDInit();
}



/*************************************************************/
/*                        上位机纵向控制命令                 */
/*************************************************************/

unsigned char SysMode = 0;                 //这个是什么作用？？？
unsigned char ErrorStatus = 0;
unsigned char ModeChange = 0;
unsigned char SpeedControlEnable = 0;
unsigned char BrakeEnable = 0;
unsigned char AccPedalEnable = 0;
unsigned char SteerWhlEnable = 0;
unsigned int ExpectPressure = 0;
unsigned int ExpectSpeed = 0;
unsigned char EStop = 0;                         //这个是什么？
unsigned char EStopStatus = 0;                   //这个是什么？
unsigned int StationParkNum = 0;                 //这个是什么？
unsigned char ReceiveCount = 0;
unsigned char ReceiveCountLast = 0;
unsigned char ReceiveCountNum = 0;

float ControlSpeed = 0.0;
float ControlBrake = 0.0;
unsigned int   m_iCountTimeOut= 0;             //这个是什么作用？？？
unsigned char m_cAccMode = 0;
unsigned char m_cAccPedal = 0;
unsigned char m_cBrakeStatus = 0;             //制动状态么？还是别的什么？
unsigned int  PressureTarget=0;
unsigned char BrakePedal = 0;
int YawRate = 0;
unsigned char LightPeriodFlag = 0;             //灯光标识位
unsigned int m_iCountLeft = 0;
unsigned int m_iCountRight = 0;
unsigned int m_iCountLeftRight = 0;
unsigned char m_cHightLowFlag = 0;
int m_iStepInputAngle = 0;
unsigned int m_iCountMaxTorque = 0;
unsigned int m_iCountBrake = 0;


/*************************************************************/
/*            对PWM输出进行卡尔曼平滑，减小振动              */
/*************************************************************/
float m_fPWMPError1 = 0.02;
float m_fPWMQError1 = 0.002;
float m_fPWMRError1 = 0.542;
float m_lPWMKGain1 = 0;
float m_fPWMPreData1 = 0;
float m_fPWMLast1 = 0;

/*************************************************************/
/*                         CAN发送参数定义                   */
/*************************************************************/

float PedalLocation_CAN=0;     //踏板行程值
float PedalForce_CAN;          //踏板力值
float CylinderLocation_CAN;    //主缸推杆行程
float MasterCylinderPrs_CAN;   //主缸压力
float BrakeMotorCurrent_CAN;   //电机电流
float SetPrs_CAN;              //目标压力
float Mode_CAN;                //工作模式
float PedalSpeed_CAN;          //踏板推杆速度
float PedalSpeedFlag_CAN;      //踏板推杆速度符号位
float Set_Current_CAN;         //目标电流
float Timer_CAN;               //系统时间
float motorDuty_CAN;           //占空比
float SetCurrentPID_CAN;       //PID控制器给定电流
float SetCurrentPIDFlag_CAN;   //PID控制器给定电流符号位
float CurrentWentai_CAN;       //稳态误差补偿
float CurrentMoca_CAN;         //摩擦电流补偿
float CurrentMocaFlag_CAN;     //摩擦补偿符号位
float GapCurrent_CAN=0.0;      //间隙补偿
float Clearance_CAN=0.0;       //间隙发出
float dGpspeed_CAN=0.0;

/*************************************************************/
/*            PWM积分项等参数初始化                          */
/*************************************************************/
int m_iPWMP = 0;
int m_iPWMI = 0;
int m_iPWMD = 0;
int m_iPWMError = 0;
int m_iPWMIntegral = 0;
int m_iPWMPError = 0;
int m_iPWMIntePre = 0;
int m_iLastPWM = 0;
int m_iMaxPWM=600;



/*************************************************************/
/*                   GPS串口接受数据处理                     */
/*************************************************************/
/*unsigned char ReceiveDatagps[128];
 uint Gpsnum=0;
 uchar Gpsdatastart=0;
 uchar Gpsdataend=0;
 uchar Gpsintsflag=0;
 uchar Gpsdipsflag=0;
 float Gpspeedint=0.0;
 float Gpspeeddip=0.0;
 float Gpspeed=0.0;
 float GpspeedLast=0.0;
 float dGpspeed=0.0;
 float dGpspeedLast=0.0;
 uint Gpsintnum=0;
 uint Gpsdipnum=0;
 uint Gpsdipendnum=0;
 uint Gpsintlength =0;
 uint Gpsdiplength =0;
 int AceTimer=0;
 int AceTimerLast=0; */

float Cylinder_Err=0.0;
float Prs_Feedback=0.0;
float Set_Cylinder=0.0;
float Cylinder_Feedback=0.0;
//float Suducha=0;

/*int AD1;
 int AD2;
 int AD3;
 int AD4;
 int AD5;
 int AD6;
 float AD7;
 float AD8;
 float AD11;
 float AD12;
 float AD13;
 float AD14;  */


/*************************************************************/
/*                   定义PID变量                             */
/*************************************************************/
struct _pid{
    float SetValue;            //定义设定值
    float ActualValue;         //定义实际值
    float err;                 //定义偏差值
    float err_pre;            //定义上一个偏差值
    float err_pre_pre;         //定义上上一个偏差值
    float Integral;
    float Kp,Ki,Kd;            //定义比例、积分、微分系数
    float Prs_Kp,Prs_Ki,Prs_Kd;
    float Cylinder_Kp,Cylinder_Ki,Cylinder_Kd;
}pid;



/*************************************************************/
/*                         CAN 发送整定                       */
/*************************************************************/
void sent_pc_CAN(){
 
     PedalLocation_CAN=PedalLocationAD2;
     PedalForce_CAN=Pedalforce1AD;
     CylinderLocation_CAN=CylinderLocationAD;
     MasterCylinderPrs_CAN=MasterCylinderPrsSenseAD;
     BrakeMotorCurrent_CAN=BrakeMotorCurrentSenseAD;
     SetPrs_CAN=Set_Prs*10.0;
     Mode_CAN=0;
     PedalSpeed_CAN=PedalSpeed_CAN;            //踏板速度，已经整定
     PedalSpeedFlag_CAN=PedalSpeedFlag_CAN;
     Set_Current_CAN=Set_Current*1000;
     //Timer_CAN=Timer;
    
     if(dGpspeed>=0){
         dGpspeed_CAN= dGpspeed*1000;
     }
    
     if(dGpspeed<0){
         dGpspeed_CAN= dGpspeed*(-1000);
     }
    
     motorDuty_CAN= motorDuty;
     SetCurrentPID_CAN=SetCurrentPID*1000;    //PID 控制器电流给定整定计算
     if(SetCurrentPID_CAN>=0){
         SetCurrentPID_CAN=SetCurrentPID_CAN;
         SetCurrentPIDFlag_CAN=0;
     }
     if(SetCurrentPID_CAN<0){
         SetCurrentPID_CAN=SetCurrentPID_CAN*(-1);
         SetCurrentPIDFlag_CAN=1;
     }
    
     CurrentWentai_CAN=CurrentWentai*1000;
     CurrentMoca_CAN=CurrentMoca_CAN;
     CurrentMocaFlag_CAN=CurrentMocaFlag_CAN;
     GapCurrent_CAN=0;
     Clearance_CAN=Clearance*1000;
 }


/*************************************************************/
/*                         初始化函数                        */
/*************************************************************/
void INIT_ALL(void){
    
    INIT_PLL();                        //初始化锁相环
    INIT_IIC();                        //初始化IIC
    //INIT_SCI0();
    INIT_CAN0();                        //初始化CAN0
    INIT_CAN1();                        //初始化CAN1
    INIT_AD();                          //初始化AD模块
    //INIT_ECT();
    INIT_MDC();                         //初始化模数递减计数器
    INIT_PWM();                         //初始化PWM
    INIT_GPIO();                        //初始化GPIO
    // COP_Init();
}


/*************************************************************/
/*            对PWM输出进行卡尔曼平滑，减小振动             */
/*************************************************************/
void InitKFPWM(void)
{
    m_fPWMPError1 = 0.02;
    m_fPWMQError1 = 0.00002;
    m_fPWMRError1 = 0.542;
    m_lPWMKGain1 = 0;
    m_fPWMPreData1 = 0;
    m_fPWMLast1 = 0;
}

/*************************************************************/
/*                        PWM卡尔曼滤波                      */
/*************************************************************/
int KalmFliterPWM1(float motorDuty)
{
    float fliterPWM = motorDuty;
    
    if((fliterPWM>m_fPWMLast1)&&((fliterPWM-m_fPWMLast1)>50))
        fliterPWM = m_fPWMLast1 + 50;
    else if((fliterPWM<m_fPWMLast1)&&((m_fPWMLast1-fliterPWM)>50))
        fliterPWM = m_fPWMLast1 - 50;
    
    m_fPWMLast1 = fliterPWM;
    m_fPWMPError1 = m_fPWMPError1+m_fPWMQError1;
    fliterPWM = m_fPWMPreData1 +(m_fPWMPError1*(fliterPWM-m_fPWMPreData1)/(m_fPWMPError1 + m_fPWMRError1));
    m_fPWMPError1 = m_fPWMRError1*m_fPWMPError1/(m_fPWMPError1 + m_fPWMRError1);
    m_fPWMPreData1 = fliterPWM;
    return (int)fliterPWM;
}

/*************************************************************/
/*                       采样周期1ms函数                     */
/*************************************************************/
void SampleTimer(unsigned int time)
{
    unsigned int i;
    unsigned int p;
    for(p=0;p<time;p++){
        for(i=0;i<1;i++)
        {
            while(ECT_TFLG1_C7F == 0)
                ;
            ECT_TFLG1_C7F = 1;
            ECT_TC7 = ECT_TCNT+125;
        }
    }
}

/*************************************************************/
/*                        延时函数                           */
/*************************************************************/
void Delay(void)
{
    unsigned int i;
    for(i=0;i<100;i++);
}


void LongDelay(void)
{
    unsigned int i,j;
    for(i=0;i<500;i++)
        for(j=0;j<100;j++);
}

/*************************************************************/
/*                       AD采样滤波函数                      */
/*************************************************************/

//这个函数是干什么的？输入什么？输出什么？执行了什么操作？

void ADSampleFilter(void)
{
    unsigned int Pedalforce1ADLast = 0;
    unsigned int Pedalforce2ADLast = 0;
    unsigned int BrakeMotorCurrentSenseADLast = 0;
    unsigned int MasterCylinderPrsSenseADLast=0;
    unsigned int CylinderLocationADLast = 0;
    unsigned int PedalLocationAD2Last = 0;
    unsigned int AutoOnOffSignalLast = 0;                       //这个是什么？
    
    Bakesignal = AD_Capture(3);                                 //制动踏板信号
    if(Bakesignal<50) {
        m_cBrakePedal =0;
    }
    else {
        m_cBrakePedal =1;
        
    }
    
     Pedalforce1ADLast = Pedalforce1AD;                          //踏板力传感器
     Pedalforce1AD = AD_Capture(9);
     Pedalforce1AD = (Pedalforce1ADLast*1 + Pedalforce1AD*3)/4;
    
    
    BrakeMotorCurrentSenseADLast = BrakeMotorCurrentSenseAD;    //助力电机电流
    BrakeMotorCurrentSenseAD = AD_Capture(14);
    BrakeMotorCurrentSenseAD = (BrakeMotorCurrentSenseADLast + BrakeMotorCurrentSenseAD*3)/4;
    
    MasterCylinderPrsSenseADLast = MasterCylinderPrsSenseAD;    //主缸压力 (对)
    MasterCylinderPrsSenseAD = AD_Capture(5);
    MasterCylinderPrsSenseAD = (MasterCylinderPrsSenseADLast+MasterCylinderPrsSenseAD*3)/4;
    
    CylinderLocationADLast = CylinderLocationAD;                //主缸行程
    CylinderLocationAD =  AD_Capture(4) ;
    CylinderLocationAD = (CylinderLocationADLast+CylinderLocationAD*3)/4;
    
    PedalLocationAD2Last = PedalLocationAD2;                    //踏板行程
    PedalLocationAD2 =  AD_Capture(11) ;
    PedalLocationAD2 = (PedalLocationAD2Last+PedalLocationAD2*3)/4;
    
    AutoOnOffSignalLast = AutoOnOffSignal;
    AutoOnOffSignal = AD_Capture(10);
    AutoOnOffSignal = (AutoOnOffSignal*3 + AutoOnOffSignalLast)/4;
    
    
    
}

/*************************************************************/
/*                      最近距离卡尔曼滤波                   */
/*************************************************************/
void KalmFliterDistance(void)
{
    long fliterDis = 0;
    unsigned int disTmp = 0;
    disTmp = m_uDistance.m_iDistance;
    fliterDis = (long)(disTmp);
    if((fliterDis>m_lLastDis)&&((fliterDis-m_lLastDis)>278))fliterDis = m_lLastDis + 278;
    else if((fliterDis<m_lLastDis)&&((m_lLastDis-fliterDis)>278))fliterDis = m_lLastDis - 278;
    
    m_lLastDis = fliterDis;
    
    fliterDis = fliterDis*10;
    m_lDisPreData = m_lDisPreData*10;
    m_lDisPError = m_lDisPError+m_lDisQError;
    fliterDis = m_lDisPreData +(m_lDisPError*(fliterDis-m_lDisPreData)/(m_lDisPError + m_lDisRError));
    m_lDisPError = m_lDisRError*m_lDisPError/(m_lDisPError + m_lDisRError);
    
    fliterDis = fliterDis/10;
    m_lDisPreData = fliterDis;
    
    m_uDistance.m_iDistance = (unsigned int)(fliterDis);
}

/*************************************************************/
/*              考虑本车行驶轨迹的最近障碍物检测             */
/*************************************************************/
void FindDangerObject1(void)
{
    unsigned char i;
    unsigned char objectNumber = 0;
    float xPoint = 0;
    float yPoint = 0;
    float sinValue = 0;
    float cosValue = 0;
    unsigned int vechileWidth = 0;
    int xTmp;
    int yTmp;
    objectNumber = OBJECT_NUM;
    //m_uDistance.m_iDistance = 12000;
    ObjectFindflag = 0;
    m_iRelativeSpeed = 0;
    //j = 0;
    
    for(i=0;i<objectNumber;i++){
        //if(ObjectRangeTmp[i]==0)j++;
        if(ObjectRangeTmp[i]==0){
            continue;
        }
        
        sinValue = sin(ObjectAngleTmp[i]*PI/180);
        cosValue = cos(ObjectAngleTmp[i]*PI/180);
        xPoint = ObjectRangeTmp[i] * sinValue;
        yPoint = ObjectRangeTmp[i] * cosValue;
        
        if(xPoint<0)xPoint = -xPoint;
        xTmp = (int)(xPoint*100);
        yTmp = (int)(yPoint*100);
        
        
        if(yTmp>12000)vechileWidth = HALF_VECHILE_WIDTH_MAX;
        else if(yTmp<1500) vechileWidth = HALF_VECHILE_WIDTH_MIN;
        else vechileWidth = yTmp*(HALF_VECHILE_WIDTH_MAX-HALF_VECHILE_WIDTH_MIN)/12000+HALF_VECHILE_WIDTH_MIN;
        
        //vechileWidth = HALF_VECHILE_WIDTH_MAX;
        if(xTmp<vechileWidth){
            if(m_uDistance.m_iDistance>yTmp){
                m_uDistance.m_iDistance = yTmp;
                m_iRelativeSpeed = (int)(ObjectSpeedTmp[i]*36);
                ObjectFindflag = 1;
            }
        }
    }
    
    
    if(ObjectFindflag == 0) {
        m_uDistance.m_iDistance = 12000;
    }
    
    KalmFliterDistance();
}




#pragma CODE_SEG __NEAR_SEG NON_BANKED


/*************************************************************/
/*                 制动减速度计算函数                        */
/*************************************************************/

void Aceleration (void){
     float  GpspeedErr=0;
     float  AceTimer_Err=0;
     if(AceTimer==1){
         GpspeedErr=Gpspeed-GpspeedLast;           //踏板推杆行程
     
     Suducha =GpspeedErr;
     
     //AceTimer_Err=AceTimer-AceTimerLast;
     dGpspeed= GpspeedErr*0.28/(10.0*0.01);
     GpspeedLast=Gpspeed;
     
     dGpspeedLast=dGpspeed;
     dGpspeed=(dGpspeedLast+3*dGpspeed)/4.0;
     AceTimer=0;
     }
 }
/*************************************************************/
/*                      控制参数初始化                       */
/*************************************************************/
void InitControlPar(){
    ExpectPressure = 0;
    BrakeEnable = 0;
    AccPedalEnable = 0;
    SpeedControlEnable = 0;
    ExpectSpeed = 0;
    AccPedalCtrl = 0;
    
    motorDuty = 0;
    
    m_sReceiveInfo.m_cBrakeLight       = 0;
    m_cAccPedal         = 0;
    
    m_sReceiveInfo.m_cLeftTurnLight    = 0;
    m_sReceiveInfo.m_cRightTurnLight   = 0;
    m_sReceiveInfo.m_cLowBeam          = 0;
    m_sReceiveInfo.m_cHornCtrl         = 0;
    m_sReceiveInfo.m_cAccPedal         = 0;
    
    LeftTurnLight    = 0;
    RightTurnLight   = 0;
    LowBeam          = 0;
    WidthLight       = 0;
    BrakeLight       = 0;
    HornCtrl         = 0;
    m_cHightLowFlag  = 0;
    
    
    
}

/*************************************************************/
/*                    通过IIC向AD5315写数据                  */
/*                      AD5315 精度为10bit                   */
/*************************************************************/
void IIC_SentData(unsigned int Data)
{
    IIC0_IBCR_TXAK = 0;               //接收到数据后有应答
    IIC0_IBCR_TX_RX = 1;              //设置单片机为发送模式
    IIC0_IBCR_MS_SL = 1;              //设置单片机为主机模式，产生开始信号
    
    //发送地址
    IIC0_IBDR = 0x18;
    while(IIC0_IBSR_IBIF == 0);
    IIC0_IBSR_IBIF = 1;
    while(IIC0_IBSR_RXAK);
    //确定通道
    IIC0_IBDR = 0x03;                 // AB通道输出
    while(IIC0_IBSR_IBIF == 0);
    IIC0_IBSR_IBIF = 1;
    while(IIC0_IBSR_RXAK);
    //第一字节数据                    // 后四位为数据高4位
    IIC0_IBDR = (0x20 | (((unsigned char)(Data>>6)) & 0x0f));
    while(IIC0_IBSR_IBIF == 0);
    IIC0_IBSR_IBIF = 1;
    while(IIC0_IBSR_RXAK);
    //第二字节数据                    // 前6位为数据低6位,后2位只能为0
    IIC0_IBDR = (0x00 | (((unsigned char)(Data<<2)) & 0xfc));
    while(IIC0_IBSR_IBIF == 0);
    IIC0_IBSR_IBIF = 1;
    while(IIC0_IBSR_RXAK);
    
    IIC0_IBCR_MS_SL = 0;              // 设置单片机为从机模式，产生停止信号
}
#pragma CODE_SEG __NEAR_SEG NON_BANKED
/*************************************************************/
/*                    模数递减中断函数                       */
/*************************************************************/
interrupt void MDC_inter(void)
{
    //unsigned int temp;
    if(ECT_MCFLG_MCZF == 1)   //判断是否是模数递减中断
        ECT_MCFLG_MCZF = 1;     //清除标志位
    sysTimer++;//基准时间200us
    /* Preset_time++;
     if(Preset_time>2500)
     Preset_time=0;
     */
}
/*************************************************************/
/*                      CAN0中断接收函数                     */
/*************************************************************/

void interrupt CAN0_receive(void) {
    unsigned char controlModel = 0;
    
    if(MSCAN0GetMsg(&Can0_msg_get))
    {
        if(Can0_msg_get.id == 0x70 && (!Can0_msg_get.RTR)) // StrWhl_ID ,0x80
        {
            m_uSteerWheelAngle.m_cSteerWheelAngleHL[0] = Can0_msg_get.data[0];
            m_uSteerWheelAngle.m_cSteerWheelAngleHL[1] = Can0_msg_get.data[1];
            m_uSteerWheelSpeed.m_cSteerWheelSpeedHL[1] = Can0_msg_get.data[2];
            // m_uSteerWheelSpeed.m_cSteerWheelSpeedHL[1] = (m_uSteerWheelSpeed.m_cSteerWheelSpeedHL[1]+m_iSteerWheelSpeedLast*3)/4;
            // m_iSteerWheelSpeedLast = m_uSteerWheelSpeed.m_cSteerWheelSpeedHL[1];
            
        }
        
        if(Can0_msg_get.id == 0x30 && (!Can0_msg_get.RTR)) {
            
            VechileSpeed = Can0_msg_get.data[6]*256+Can0_msg_get.data[7];
            m_uSpeed.m_iSpeed = VechileSpeed;
        }
        
        
        if(Can0_msg_get.id == RECRIVE_PC_ID && (!Can0_msg_get.RTR)) {
            m_iCountTimeOut = 0;
            controlModel = (uchar)((Can0_msg_get.data[0] >>3) & 0x01) ;
            //0人工模式 1自动模式 2人工干预模式
            //人工模式和人工干预模式有什么区别？
            if(controlModel>1)
                AutoControl_Mode = 0;
            
            if(ErrorStatus==1){
                AutoControl_Mode = 0;
            }else{
                if(AutoControl_Mode==2){
                    if(controlModel==0)
                        AutoControl_Mode = 0;
                    else if(controlModel==1&&AutoOnOffSignal>500)
                        AutoControl_Mode = 0;
                } else {
                    if(AutoControl_Mode==0||AutoControl_Mode==1)
                        AutoControl_Mode = controlModel;
                }
                
            }
            
            if(AutoControl_Mode == 1 && AutoOnOffSignal<=500){
                BrakeEnable    = (uchar)((Can0_msg_get.data[0]>>7) & 0x01);
                //0 不使能  1使能
                AccPedalEnable = (uchar)((Can0_msg_get.data[0]>>5) & 0x01);
                //0 不使能  1使能
                m_cAccPedal = Can0_msg_get.data[1]*2;                //N%
                ExpectPressure  = Can0_msg_get.data[2];
                
            }
            
        }
        
        if(Can0_msg_get.id == RECRIVE_PC_ID1 && (!Can0_msg_get.RTR)) {
            if(AutoControl_Mode == 1){
                m_sReceiveInfo.m_cBrakeLight     = (uchar)((Can0_msg_get.data[1] >>7) & 0x01);
                m_sReceiveInfo.m_cHornCtrl       = (uchar)((Can0_msg_get.data[2] >>7) & 0x01);
                m_sReceiveInfo.m_cLowBeam        = (uchar)((Can0_msg_get.data[3] >>7) & 0x01);
                m_sReceiveInfo.m_cRightTurnLight = (uchar)((Can0_msg_get.data[5] >>7) & 0x01);
                m_sReceiveInfo.m_cLeftTurnLight  = (uchar)((Can0_msg_get.data[6] >>7) & 0x01);
            }
        }
        
        if(AutoControl_Mode == 0||AutoControl_Mode == 2||AutoOnOffSignal>500){
            InitControlPar();
        }
    }
}

/*************************************************************/
/*                      CAN1中断接收函数                     */
/*************************************************************/

void interrupt CAN1_receive(void)
{
    unsigned char i;
    if(MSCAN1GetMsg(&Can1_msg_get))
    {
        if(!Can1_msg_get.RTR)
        {
            DataIndex = Can1_msg_get.id - 0x500;
            if (DataIndex >= 0 && DataIndex < 64)
            {
                RelevantData[DataIndex][0] = Can1_msg_get.data[0];
                RelevantData[DataIndex][1] = Can1_msg_get.data[1];
                RelevantData[DataIndex][2] = Can1_msg_get.data[2];
                RelevantData[DataIndex][3] = Can1_msg_get.data[3];
                RelevantData[DataIndex][4] = Can1_msg_get.data[4];
                RelevantData[DataIndex][5] = Can1_msg_get.data[5];
                RelevantData[DataIndex][6] = Can1_msg_get.data[6];
                RelevantData[DataIndex][7] = Can1_msg_get.data[7];
                //计算角度
                ObjectAngleInt = (RelevantData[DataIndex][1] & 0x1f) * 32 + ((RelevantData[DataIndex][2] & 0xf8) >> 3);
                //RelevantData[0][1]的后五位作为高位和RelevantData[0][2]=0x48前五位作为低位
                ObjectAngleInt = (RelevantData[DataIndex][6] & 0x1f) * 64 + ((RelevantData[DataIndex][5] & 0xfc) >> 2);
                //RelevantData[0][1]的后五位作为高位和RelevantData[0][2]=0x48前五位作为低位
                
                
                //组合成一组数据，这是用补码表示的一个数
                
                
                if (1024 == (ObjectAngleInt & 1024)){
                    ObjectAngleInt = (ObjectAngleInt - 1);         //减一
                    ObjectAngleInt = ~ObjectAngleInt;              //取反
                    ObjectAngleInt = -(ObjectAngleInt & 0x3ff);    //得到最终用整数表示的角度值
                    ObjectAngle[DataIndex] = ObjectAngleInt * 0.1; //乘以比例因子。比例因子由雷达表格中查出。角度的比例因子为0.1；
                }else{
                    ObjectAngle[DataIndex] = ObjectAngleInt * 0.1;
                }
                //计算距离
                
                //RelevantData[0][1]的后五位作为高位和RelevantData[0][2]=0x48前五位作为低位
                ObjectRangeInt = (RelevantData[DataIndex][2] & 0x7) * 256 + RelevantData[DataIndex][3];
                
                //RelevantData[0][1]的后五位作为高位和RelevantData[0][2]=0x48前五位作为低位
                ObjectRangeInt = (RelevantData[DataIndex][1] & 0x7f) * 256 + RelevantData[DataIndex][0];
                
                ObjectRange[DataIndex] = ObjectRangeInt * 0.1;
                
                if (0 == ObjectAngleInt && 0 == ObjectRangeInt){
                    ;
                    break;//如果角度和距离都为0为无效数据
                }else{
                    //RelevantData[0][1]的后五位作为高位和RelevantData[0][2]=0x48前五位作为低位
                    ObjectSpeedInt = (RelevantData[DataIndex][6] & 0x3f) * 256 + RelevantData[DataIndex][7];
                    
                    //RelevantData[0][1]的后五位作为高位和RelevantData[0][2]=0x48前五位作为低位
                    ObjectSpeedInt = (RelevantData[DataIndex][3] & 0x3f) * 256 + RelevantData[DataIndex][2];}
                
                if (8192 == (ObjectSpeedInt & 8192)){
                    ObjectSpeedInt = (ObjectSpeedInt - 1);         //减一
                    ObjectSpeedInt = ~ObjectSpeedInt;              //取反
                    ObjectSpeedInt = -(ObjectSpeedInt & 0x3fff);   //得到最终用整数表示的角度值
                    ObjectSpeed[DataIndex] = ObjectSpeedInt * 0.01;
                    //乘以比例因子。比例因子由雷达表格中查出。速度的比例因子为0.1；
                }else{
                    ObjectSpeed[DataIndex] = ObjectSpeedInt * 0.01;
                }
            }
            
            
            if(DataIndex==63) {
                MaxIndexLabel = 1;
                if(ProcessRadarData == 0 || NoUpdateDistanceIndex==3){
                    NoUpdateDistanceIndex = 0;
                    for(i=0;i<OBJECT_NUM;i++){
                        ObjectRangeTmp[i] = ObjectRange[i];
                        ObjectAngleTmp[i] = ObjectAngle[i];
                        ObjectSpeedTmp[i] = ObjectSpeed[i];
                    }
                }else{
                    NoUpdateDistanceIndex++;
                }
            }
        }
    }
    
    
}
#pragma CODE_SEG DEFAULT
/*************************************************************/
/*                  电流反馈值计算                           */
/*************************************************************/
void CurrentFeed(void){
    CurrentFeedback=BrakeMotorCurrent;
}

/*************************************************************/
/*                  电压反馈值计算                           */
/*************************************************************/
void Prs_Feed(void){
    Prs_Feedback=MasterCylinderPrs*10.0;
}
/*************************************************************/
/*                  电流补偿函数                          */
/*************************************************************/
void Current_Compensation(void){
    
    if(PedalLocation<2.0)
        CurrentWentai=0;
    
    if((PedalLocation>=2.0)&&(PedalLocation<10.4))                          //线性段电流补偿1.0A
        CurrentWentai=0.5;
    
    if((PedalLocation>=10.4)&&(PedalLocation<19.5))
        CurrentWentai=0.8;                                                    //中间非线性段电流补偿1.5A
    
    if((PedalLocation>=19.5)&&(PedalLocation<25))
        CurrentWentai=1.5;                                                    //后段非线性段电流补偿2A
    
    if(PedalLocation>=25.0)
        CurrentWentai= 2;
}


/*************************************************************/
/*                电流给定计(算踏板信号）                    */
/*************************************************************/
void Current_Set (void){
    
    if(PedalLocation<3.0)
     Set_Current=0;
     if((PedalLocation>=3.0)&&(PedalLocation<8))
         Set_Current=PedalLocation*0.431*1.5;                                              //线性段
     if((PedalLocation>8)&&(PedalLocation<21))
     Set_Current=(0.0265*PedalLocation*PedalLocation-0.2387*PedalLocation+3.6605)*1.5;
    //中间非线性段
     if((PedalLocation>=21)&&(PedalLocation<25))
     Set_Current=(0.2155*PedalLocation*PedalLocation-8.1897*PedalLocation+87.284)*1.5;
    //后段非线性段
     if(PedalLocation>=25.0)
     Set_Current=17.3*1.5;*/
    
    if(PedalLocation<1.0)
     Set_Current=0;
     if((PedalLocation>=1.0)&&(PedalLocation<16))
         Set_Current=PedalLocation*0.6398+2.1826;
     if((PedalLocation>=16)&&(PedalLocation<18))
         Set_Current=PedalLocation*3.6663-43.2945;
     if(PedalLocation>=18.0)
         Set_Current=21.0;
    
    if(PedalLocation<1.0)
     Set_Current=0;
     if((PedalLocation>=1.0)&&(PedalLocation<15))
         Set_Current=PedalLocation*0.3398+2.1826;
     if((PedalLocation>=15)&&(PedalLocation<18))
         Set_Current=PedalLocation*1.8663-20.7134;
     if(PedalLocation>=18.0)
         Set_Current=12.0;
    
    
     if((PedalLocation>=2.0)&&(PedalLocation<8))
         Set_Current=PedalLocation*0.431*1.5;
        //线性段
     if((PedalLocation>=8.0)&&(PedalLocation<21.0))
         Set_Current=(0.0265*PedalLocation*PedalLocation-0.2387*PedalLocation+3.6605)*1.5;
        //中间非线性段
     if((PedalLocation>=21.0)&&(PedalLocation<25.0))
         Set_Current=(0.2155*PedalLocation*PedalLocation-8.1897*PedalLocation+87.284)*1.5;
        //后段非线性段
     if(PedalLocation>=25.0)
         Set_Current=17.3*1.5;
    
    if((PedalLocation>=0)&&(PedalLocation<0.85))
        Set_Current=0;
    if((PedalLocation>=0.85)&&(PedalLocation<10.0))
        Set_Current=PedalLocation*0.4372+3.6284;//线性段
    if((PedalLocation>=10.0)&&(PedalLocation<15.0))
        Set_Current=0.8*PedalLocation;//线性段
    if((PedalLocation>=15.0)&&(PedalLocation<25.0))
        Set_Current=PedalLocation*1.3-7.5;//线性段
    if(PedalLocation>=25.0)
        Set_Current=25.0;
    
    Set_Current = Set_Current*3/2;
    
}

/*************************************************************/
/*                电流给定计(自动测试）                    */
/*************************************************************/
void AutoCurrent_SetAndPID (void){
    
    int x;
    int y;
    x=AutoCurrentSet_Time/300;
    y=AutoCurrentSet_Time%300;
    
    if((x>=0)&&(x<=50))
        Set_Current=0.5* ((int)(AutoCurrentSet_Time/300));
    if((x>=51)&&(x<=100))
        Set_Current=(float)(50.0-0.5* ((int)(AutoCurrentSet_Time/300)));
    
    if((x>=200)&&(x<=300)){
        //sent_pc_CAN();
        //CAN0SendPCData();
    };
    
}

/*************************************************************/
/*                主缸位置给定计(算踏板信号）                */
/*************************************************************/
void Cylinder_Set (void){
    
    /*----------------底层自动给定-----------------*/
     signed Mod;
     signed Shang;
     
     Mod = MCTimer % 400;
     Shang = MCTimer / 400;
    
     if(Mod <= 200)
         Set_Cylinder =  Shang + 1.0;
     else
         Set_Cylinder = 0;
    
     if((MCTimer>=0)&&(MCTimer<200))                       //方波输出
        Set_Cylinder=3;
     if((MCTimer>=200)&&(MCTimer<400))
         Set_Cylinder=0;
    Set_Cylinder = 8.0 * ABS (sin(6.28 * ((float)SinTimer/800))); //正弦波输出
    /*----------------QT自动给定-----------------*/
    Set_Cylinder = ReceiveData0[2];
    
    /*----------------脚踩给定（标准）------------*/
    if(PedalLocation<1)
        Set_Cylinder = 0.0;
    if((PedalLocation>=1)&&(PedalLocation<7.0))
        Set_Cylinder = PedalLocation * 0.58 + 1.92;
    if((PedalLocation >= 7.0) && (PedalLocation < 12))
        Set_Cylinder=PedalLocation*0.7 + 1.1;
    if((PedalLocation>=12)&&(PedalLocation<18.0))
        Set_Cylinder=PedalLocation * 1.25 - 5.5;
    if(PedalLocation>=18)
        Set_Cylinder=PedalLocation * 1.25 - 5.5;
    
}


/*************************************************************/
/*                 AD采样值转换为实际值                      */
/*************************************************************/
void AD_to_real(void){
    
    
    signed int Im;
    float Pedalx;
    float Cylinder;
    if(PedalLocationAD2<=230)
        Pedalx=230;
    if(PedalLocationAD2>230)
        Pedalx=PedalLocationAD2;
    
    if(CylinderLocationAD<=470)
        Cylinder=470;
    if(CylinderLocationAD>470)
        Cylinder=CylinderLocationAD;
    
    PedalForce=(float)(((float)Pedalforce1AD/2048.0)*1000.0);
    //踏板力值
    CylinderLocation=(float)((float)((Cylinder-470)*39.64)/1600.0);
    //主缸推杆行程，单位毫米0~20
    
    PedalLocation=(float)((float)((Pedalx-230)*50.27)/1600.0);
    //踏板推杆行程，单位毫米0~20
    
    MasterCylinderPrs=(float)((float)MasterCylinderPrsSenseAD/102.0);
    //主缸压力，单位MPa0~15
    
    if(BrakeMotorCurrentSenseAD<=2045)
        //读取电机电流,使电机电流从0逐渐增大
        Im=2045-BrakeMotorCurrentSenseAD ;
    if(BrakeMotorCurrentSenseAD>2045)
        //读取电机电流,电机电流从2048逐渐增加
        Im=BrakeMotorCurrentSenseAD-2045;
    BrakeMotorCurrent=(float)((float)Im/16.4);
    //助力电机电流,单位A0~30
    
}

/*************************************************************/
/*                 PID参数初始化                             */
/*************************************************************/
void PID_init(){
    
    m_iPWMP = 0;
    m_iPWMI = 0;
    m_iPWMD = 0;
    m_iPWMError = 0;
    m_iPWMIntegral = 0;
    m_iPWMPError = 0;
    m_iPWMIntePre = 0;
    m_iLastPWM = 0;
    
    pid.SetValue=0;
    pid.ActualValue=0;
    pid.err=0.0;
    pid.err_pre=0.0;
    pid.err_pre_pre=0.0;
    pid.Integral=0.0;
    // 阶跃  pid.Kp=1.4;
    //pid.Ki=1.8;
    pid.Kp=22.5;
    pid.Ki=2;
    pid.Kd=1.5;
    
    
    pid.Prs_Kp=48;                                               //压力跟踪PID初始化参数
    pid.Prs_Ki=3;
    pid.Prs_Kd=2;
    
}
/*************************************************************/
/*                 PID调节算法位置式                         */
/*************************************************************/
float PID_realize_location(float Value)
{
    signed int incrementValue;
    if(Value<1&&MasterCylinderPrs<0.1)return 0;
    
    pid.Prs_Kp = 20;
    pid.Prs_Ki   = 0.2;
    pid.Prs_Kd   = 1;
    
    pid.SetValue=Value;
    
    pid.err=pid.SetValue-pid.ActualValue;
    
    m_iPWMIntePre = pid.Integral;
    if(ABS(pid.err)>1)pid.Integral=pid.Integral*0.99 + pid.err;
    
    //incrementValue=pid.Prs_Kp*pid.err;
    
    incrementValue=pid.Prs_Kp*pid.err+pid.Prs_Ki*pid.Integral;
    
    //incrementValue=pid.Prs_Kp*pid.err+pid.Prs_Ki*pid.Integral+pid.Prs_Kd*(pid.err-pid.err_pre);
    //incrementValue = KalmFliterPWM1(incrementValue);
    
    m_iPWMP = (int)(pid.Prs_Kp*pid.err);
    m_iPWMI = (int)(pid.Prs_Ki*pid.Integral);
    m_iPWMD = (int)(pid.Prs_Kd*(pid.err-pid.err_pre));
    m_iPWMError = pid.err;
    m_iPWMIntegral = pid.Integral;
    m_iPWMPError = pid.err-pid.err_pre;
    
    
    if(ABS(incrementValue)>m_iMaxPWM) incrementValue = m_iMaxPWM*SIGN(incrementValue);
    pid.err_pre=pid.err;
    m_iLastPWM = incrementValue;
    return  incrementValue;
}

/*************************************************************/
/*                 PID压力跟踪算法                           */
/*************************************************************/

int PID_Pressure(float Value) {
    signed int incrementValue;
    if(Value<1&&MasterCylinderPrs<0.1)return 0;
    
    pid.Prs_Kp = 12;
    pid.Prs_Ki   = 0.1;
    pid.Prs_Kd   = 1;
    
    pid.SetValue=Value;
    
    pid.err=pid.SetValue-MasterCylinderPrs*10;
    
    m_iPWMIntePre = pid.Integral;
    if(ABS(pid.err)>1)pid.Integral=pid.Integral + pid.err;
    
    //incrementValue=pid.Prs_Kp*pid.err;
    
    incrementValue=pid.Prs_Kp*pid.err+pid.Prs_Ki*pid.Integral;
    
    //incrementValue=pid.Prs_Kp*pid.err+pid.Prs_Ki*pid.Integral+pid.Prs_Kd*(pid.err-pid.err_pre);
    //incrementValue = KalmFliterPWM1(incrementValue);
    
    m_iPWMP = (int)(pid.Prs_Kp*pid.err);
    m_iPWMI = (int)(pid.Prs_Ki*pid.Integral);
    m_iPWMD = (int)(pid.Prs_Kd*(pid.err-pid.err_pre));
    m_iPWMError = pid.err;
    m_iPWMIntegral = pid.Integral;
    m_iPWMPError = pid.err-pid.err_pre;
    
    
    if(ABS(incrementValue)>m_iMaxPWM) incrementValue = m_iMaxPWM*SIGN(incrementValue);
    pid.err_pre=pid.err;
    m_iLastPWM = incrementValue;
    return  incrementValue;
}

/*************************************************************/
/*                 PID主缸位置反馈跟踪算法                   */
/*************************************************************/
float PID_Cylinder(float Value)
{
    signed int incrementValue;
    float c, Buchang;
    
    Buchang = 0;
    
    //pid.Cylinder_Kp = 14.0;
    //pid.Cylinder_Ki = 0,07;
    pid.Cylinder_Kd = 0;
    
    //AD7 = Value;
    
    if((Value >= 0)&&(Value < 2.0)){
        pid.Cylinder_Kp = 14.0;
        pid.Cylinder_Ki = 0.01;
    }
    
    if((Value >= 2.0)&&(Value < 4.0)){
        pid.Cylinder_Kp =14.0;
        pid.Cylinder_Ki = (-0.02 * Value)  + 0.14;
        //pid.Cylinder_Ki = 25.0;
        
    }
    
    if((Value >= 4.0)&&(Value < 10.0)){
        pid.Cylinder_Kp = (2.6 * Value)  + 3.6;
        pid.Cylinder_Ki = 0.06;
        //pid.Cylinder_Ki = 25.0;
        
    }
    
    if(Value >= 10.0){
        pid.Cylinder_Kp = (2.6 * Value)  + 3.6;;
        pid.Cylinder_Ki = 0.005*Value + 0.01 ;
    }
    
    
    if(Value>0)     //add by zk
    {
        Buchang = Value * 0.06 + 0.06;             //误差补偿曲线
    }
    else
    {
        Buchang =0;       //add by zk
    }
    
    pid.SetValue= (Value + Buchang )*10.0;     //将主缸行程给定值扩大10倍;
    
    pid.ActualValue= CylinderLocation*10.0;   //将主缸行程反馈值扩大10倍;
    
    pid.err=pid.SetValue-pid.ActualValue;
    
    Cylinder_Err = pid.err/10.0;
    
    if(ABS(pid.err)>1)
        pid.Integral = pid.Integral*0.99 + pid.err;
    
    //AD11=pid.Integral;
    
    if(Value == 0)
        pid.Integral = pid.Integral*0.95 + pid.err;
    
    incrementValue=pid.Cylinder_Kp*pid.err +
    pid.Cylinder_Ki*pid.Integral+
    pid.Cylinder_Kd*(pid.err-pid.err_pre);
    
    if(ABS(incrementValue)>m_iMaxPWM) incrementValue = m_iMaxPWM*SIGN(incrementValue);
    
    m_iPWMP = (int)(pid.Cylinder_Kp*pid.err);
    m_iPWMI = (int)(pid.Cylinder_Ki*pid.Integral);
    m_iPWMD = (int)(pid.Cylinder_Kd*(pid.err-pid.err_pre));
    
    pid.err_pre=pid.err;
    
    return  incrementValue;
}





/*************************************************************/
/*                摩擦补偿电流计算                           */
/*************************************************************/
void ActionDeside (void){
    
    float  PedalLocationErr=0;
    float  Preset_time_Err=0;
    float  dPedalLocation=0;
    float  PedalSpeedLast;
    PedalSpeedLast=PedalSpeed;
    
    PedalLocationErr=PedalLocation-PedalLocationLast;;            //踏板推杆行程
    Preset_time_Err=Preset_time-Preset_time_Last;
    Preset_time_Last=Preset_time;
    PedalLocationLast=PedalLocation;
    
    dPedalLocation= PedalLocationErr/(Preset_time_Err*0.01);
    PedalSpeed=dPedalLocation;
    PedalSpeed=(PedalSpeedLast+3*PedalSpeed)/4.0;
    
    //Test1=PedalLocationErr;
    //Test2=Preset_time_Err;
    //Test3=dPedalLocation;
    
    
    if(PedalSpeed>=15)
        CurrentMoca=3;
    if((-5<PedalSpeed)&&(PedalSpeed<15))
        CurrentMoca=PedalSpeed*0.2;
    if(PedalSpeed<=-15)
        CurrentMoca=-3;
    
    
    //电流摩擦补偿
    //踏板速度计算
    if(PedalSpeed>=0){
        PedalSpeed_CAN=PedalSpeed*(1000);
        PedalSpeedFlag_CAN=0;
        CurrentMoca_CAN=CurrentMoca*(1000);
        CurrentMocaFlag_CAN=0;
        
    }
    
    if(PedalSpeed<0){
        PedalSpeed_CAN = PedalSpeed*(-1000);
        PedalSpeedFlag_CAN=1;
        CurrentMoca_CAN = CurrentMoca*(-1000);
        CurrentMocaFlag_CAN=1;
    }
    
}

/*************************************************************/
/*                  踏板推杆与主缸齿条之间的间隙计算         */
/*************************************************************/
void Cleaance_Calculate(){
    float Sp=0;
    float Sv=0;
    Sp=PedalLocation;
    Sv=CylinderLocation+9.2;
    Clearance=Sv-Sp;
    
}


/*************************************************************/
/*                     电机电流PID跟踪                       */
/*************************************************************/
void Current_PID (void){
    
    motorDutyLast=motorDuty;                     //上一次的电机占空比
    ActionDeside();                              //驾驶员行为判断补偿
    Cleaance_Calculate();                        //踏板推杆与主缸齿条之间的间隙
    Current_Compensation();                      //电流稳态误差补偿
    CurrentFeed();                               //电流反馈值计算
    pid.ActualValue=CurrentFeedback;
    motorDuty=PID_realize_location(Set_Current); //电流跟踪PID实现
    
    //motorDuty = (motorDutyLast*1 + motorDuty*3)/4;//均值滤波
    
    
    if(Bakesignal>50)MotorDuty = 0;
    else{
        if(ABS(motorDuty)<=20)MotorDuty = 0;
        else if(ABS(motorDuty)>640) MotorDuty = 640;
        if(motorDuty>20){
            MotorDuty = motorDuty;
            StrMotor_DirL = 0;                     // 正转
            StrMotor_DirR = 1;
        }
        
        if(motorDuty<-20){
            MotorDuty = ABS(motorDuty);
            StrMotor_DirL = 1;                     // 反转
            StrMotor_DirR = 0;
        }
    }
    
}

/*************************************************************/
/*                     制动压力PID跟踪                        */
/*************************************************************/
void Prs_PID (void){
    float motorDutyLast;
    motorDutyLast=motorDuty;                     //上一次的电机占空比
    Prs_Feed();                                  //制动压力反馈值计算
    pid.ActualValue=Prs_Feedback;
    motorDuty=PID_Pressure(Set_Prs);             //电流跟踪PID实现
    
    motorDuty = (motorDutyLast + motorDuty*3)/4;//均值滤波
    
    if(motorDuty<=0) {                            //电机过流保护
        motorDuty=0;
        MotorDuty=0;
    }
    if((motorDuty>0)&&(motorDuty<=640))
        MotorDuty=motorDuty;
    if(motorDuty>640){
        motorDuty=640;
        MotorDuty=640;
    }
    
}

/*************************************************************/
/*                     调试参数给定                         */
/*************************************************************/
void Fector_Set (void){
     Set_Current=(float)(((float)ReceiveData1[0]*256+(float)ReceiveData1[1])/1000.0);
     pid.Kp=((float)((float)ReceiveData1[2]*256+(float)ReceiveData1[3]))/1000.0;
     pid.Ki=((float)((float)ReceiveData1[4]*256+(float)ReceiveData1[5]))/10000.0;
     pid.Kd=((float)((float)ReceiveData1[6]*256+(float)ReceiveData1[7]))/1000.0;
    
    Set_Prs=(float)(((float)ReceiveData1[0]*256+(float)ReceiveData1[1])/1000.0);
    pid.Prs_Kp=((float)((float)ReceiveData1[2]*256+(float)ReceiveData1[3]))/1000.0;
    pid.Prs_Ki=((float)((float)ReceiveData1[4]*256+(float)ReceiveData1[5]))/10000.0;
    pid.Prs_Kd=((float)((float)ReceiveData1[6]*256+(float)ReceiveData1[7]))/1000.0;
}


/*************************************************************/
/*                接收灯光控制命令，进行灯光控制             */
/*************************************************************/
void AutoLight(){
    
    if(m_sReceiveInfo.m_cLeftTurnLight  == 1 && m_sReceiveInfo.m_cRightTurnLight!= 1)               //左转
    {
        m_iCountLeft++;
        RightTurnLight = LED_OFF;
        if(m_iCountLeft < INTERRUPTTIME)
        {
            LeftTurnLight = LED_ON;
        }
        else if(m_iCountLeft < INTERRUPTTIME*2)
        {
            if(m_iCountLeft == INTERRUPTTIME*2 -1)
                m_iCountLeft = 0;
            LeftTurnLight = LED_OFF;
        }
    }
    else if(m_sReceiveInfo.m_cRightTurnLight == 1 && m_sReceiveInfo.m_cLeftTurnLight!=1)              //右转
    {
        m_iCountRight++;
        LeftTurnLight = LED_OFF;
        if(m_iCountRight < INTERRUPTTIME)
        {
            RightTurnLight = LED_ON;
        }
        else if(m_iCountRight < INTERRUPTTIME*2)
        {
            if(m_iCountRight == INTERRUPTTIME*2-1)
                m_iCountRight = 0;
            RightTurnLight = LED_OFF;
        }
    }
    else if(m_sReceiveInfo.m_cRightTurnLight == 1 && m_sReceiveInfo.m_cLeftTurnLight == 1)            //双闪
    {
        m_iCountLeftRight++;
        if(m_iCountLeftRight < INTERRUPTTIME)
        {
            LeftTurnLight = LED_ON;
            RightTurnLight = LED_ON;
        }
        else if(m_iCountLeftRight < INTERRUPTTIME*2)
        {
            if(m_iCountLeftRight == INTERRUPTTIME*2-1)
                m_iCountLeftRight = 0;
            LeftTurnLight = LED_OFF;
            RightTurnLight = LED_OFF;
        }
    }
    else
    {
        m_iCountLeft = 0;
        m_iCountRight = 0;
        m_iCountLeftRight = 0;
        LeftTurnLight = LED_OFF;
        RightTurnLight = LED_OFF;
    }
    
    
    //远光灯
     if(m_sReceiveInfo.m_cHighBeam == 1){
         HighBeam = LED_ON;
         WidthLight = LED_ON;
         m_cHightLowFlag = 1;
     }else{
         HighBeam = LED_OFF;
     }
    //近光灯
    if(m_sReceiveInfo.m_cLowBeam == 1){
        LowBeam = LED_ON;
        WidthLight = LED_ON;
        m_cHightLowFlag = 1;
    }else{
        LowBeam = LED_OFF;
    }
    
    
    if((m_sReceiveInfo.m_cLowBeam == 0)&&(m_sReceiveInfo.m_cHighBeam == 0))
        m_cHightLowFlag = 0;
    
    
    //位置灯
    if(m_sReceiveInfo.m_cWidthLight == 1){
        WidthLight = LED_ON;
    }else{
        if(m_cHightLowFlag == 1)
        WidthLight = LED_ON;
        else
        WidthLight = LED_OFF;
    }
    
    
    //喇叭
    if(m_sReceiveInfo.m_cHornCtrl == 1){
        HornCtrl = LED_ON;
    }else{
        HornCtrl = LED_OFF;
    }
    
    //刹车灯,主动制动控制优先
    if(BrakeEnable==0){
        if(m_sReceiveInfo.m_cBrakeLight == 1){
            BrakeLight = LED_ON;
        }else{
            BrakeLight = LED_OFF;
        }
    }
    
}

/*************************************************************/
/*                          油门控制                         */
/*************************************************************/
void AutoThrottle() {
    if(m_cAccPedal>1){
        IIC_SentData(m_cAccPedal+225);                           //发送油门踏板控制信号
        AccPedalCtrl_Enable;
    }else{
        IIC_SentData(m_cAccPedal+200);                           //发送油门踏板控制信号
        AccPedalCtrl_Enable;
    }
}

/*************************************************************/
/*                        制动控制                           */
/*************************************************************/
void BrakeStatus()
{
    /***********************************************
    
    Set_Current = 30   ;MasterPrs = 651
    Set_Current = 28   ;MasterPrs = 600
    Set_Current = 26   ;MasterPrs = 550
    Set_Current = 24   ;MasterPrs = 520
    Set_Current = 22   ;MasterPrs = 480
    Set_Current = 20   ;MasterPrs = 449
    Set_Current = 18   ;MasterPrs = 425
    Set_Current = 16   ;MasterPrs = 385
    Set_Current = 14   ;MasterPrs = 352
    Set_Current = 12   ;MasterPrs = 309
    Set_Current = 10   ;MasterPrs = 259
    Set_Current = 8    ;MasterPrs = 173
    Set_Current = 6    ;MasterPrs = 82
    Set_Current = 4    ;MasterPrs = 10
    Set_Current = 2    ;MasterPrs = 5
    Set_Current = 1    ;MasterPrs = 5
    y = 0.002x^2 + 0.237x + 1.3555
    
    ************************************************/
    unsigned int pwmTemp = 0;
    /*
     目标压力<85，压力状态为Normal;否则,为HOLD状态。
     目标压力>100,目标压力为100.
     由目标压力计算输出电流。
     
     */
    if(PressureTarget < 85)
        m_cBrakeStatus = NORMAL;          //正常状态
    else
        m_cBrakeStatus = HOLD_PRESSURE;   //HOLD状态
    
    if(ExpectPressure>100)
        ExpectPressure = 100;
    Set_Current = 0.003*ExpectPressure*ExpectPressure + 0.247*ExpectPressure + 2.421;
    

     if(m_cBrakeStatus==NORMAL){
         MotorDuty = 0;
         M1_DT_DIS_CTRL = 1;    //这个是什么
         M2_DT_DIS_CTRL = 1;    //这个是什么
     }else{
         M1_DT_DIS_CTRL = 0;    //这个是什么
         M2_DT_DIS_CTRL = 0;    //这个是什么
     }
}

/*************************************************************/
/*                接收制动命令，进行主动制动                 */
/*************************************************************/
void AutoBrake() {
    unsigned int preTmp = 0;
    preTmp = ExpectPressure + 9;
    if(preTmp>109)
        preTmp = 109;
    PressureTarget = preTmp*41/5 + 4;
    BrakeStatus();
    
}

/*************************************************************/
/*                    EStop紧急停车                          */
/*************************************************************/
void AutoEStopBrake(){
     if(m_uInitSpeed.m_iSpeed!=0){
         PressureTarget = 50*82/10+4;   //制动压力4MPa;  =410，为什么？
         StationParkNum = 0;
     }else {
         PressureTarget = 40*82/10+4;   //制动压力2MPa;  =332，为什么？
         StationParkNum++;
     }
    
     if(StationParkNum>10000){
         StationParkNum = 0;
         BrakeDuty = 0;
         InternalSVCtrl = 1;
         m_cBrakeStatus==NORMAL;
     }else
         BrakeStatus();
}

/*************************************************************/
/*                方向盘转角转换                             */
/*************************************************************/
//若方向盘角度小于32767，直接角度取整后返回，
//若方向盘角度大于32767，将角度减65536后取整并返回

int WheelAngleProcess(unsigned int wheelAngle)
{
    int angle =0;
    if(wheelAngle<32767)
        angle = (int)(wheelAngle);
    else if(wheelAngle>32767)
        angle = (int)(wheelAngle-65536);
    return angle;
}

/*************************************************************/
/*                 方向盘是否回正                            */
/*************************************************************/
//方向盘转角为720～9000或在0～370或在56536～65536返回1，否则返回0
unsigned char IsWheelNotReturnToCenter()
{
    if((9000>m_uSteerWheelAngle.m_iSteerWheelAngle && m_uSteerWheelAngle.m_iSteerWheelAngle>720)||
       (370>m_uSteerWheelAngle.m_iSteerWheelAngle && m_uSteerWheelAngle.m_iSteerWheelAngle>=0)||(65536>m_uSteerWheelAngle.m_iSteerWheelAngle && m_uSteerWheelAngle.m_iSteerWheelAngle>=56536))
        return 1;
    else
        return 0;
}
/*************************************************************/
/*                判断是否在进行急转向操作                   */
/*************************************************************/


unsigned char IsEmergencySteeringWheel(void)
{
    int wheelAngle = 0;
    unsigned char isWheelNotReturnToCenter = IsWheelNotReturnToCenter();
    
    if(m_cIsEmergencySteeringWheel==0)
        m_cWheelSpeedMaxRecord = 0;
    //未处于急转向干预时，AEB干预期间方向盘转速的最大值(m_cWheelSpeedMaxRecord)为0
    
    else if(m_cIsEmergencySteeringWheel==1){
        m_cIsEmergencySteeringWheel++;
        m_iWheelAngleMaxS2Record = m_uSteerWheelAngle.m_iSteerWheelAngle;
        return 1;
    //急转向干预值=1时，急转向干预值加1，记录下一个采样方向盘转角值
    }else {
        if((WheelAngleProcess(m_iWheelAngleMaxS1Record)>\
            WheelAngleProcess(m_iWheelAngleMaxS2Record)&&\
            WheelAngleProcess(m_uSteerWheelAngle.m_iSteerWheelAngle)>\
            WheelAngleProcess(m_iWheelAngleMaxS1Record))||\
           //若第一次记录值大于第二次记录值且现在速度大于第一次记录值或者
           (WheelAngleProcess(m_iWheelAngleMaxS1Record)<\
            WheelAngleProcess(m_iWheelAngleMaxS2Record)&&\
            WheelAngleProcess(m_uSteerWheelAngle.m_iSteerWheelAngle)<\
            WheelAngleProcess(m_iWheelAngleMaxS1Record))){
           //第一次记录值小于第二次记录值且现在速度小于第一次记录值
               m_cIsEmergencySteeringWheel = 0;
           //那么返回0
         }else{
               return 1;}
    }
    
    if(m_cStatus != AEB_STANDBY){
        if(m_cWheelSpeedMaxRecord<m_uSteerWheelSpeed.m_cSteerWheelSpeedHL[1]){
            m_cWheelSpeedMaxRecord = m_uSteerWheelSpeed.m_cSteerWheelSpeedHL[1];
            m_iWheelAngleMaxS1Record = m_uSteerWheelAngle.m_iSteerWheelAngle;
        }
        
        if(m_cWheelSpeedMaxRecord>WHEEL_SPEED_MAX && (isWheelNotReturnToCenter==1)){
            m_cIsEmergencySteeringWheel = 1;
            return 1;
        }
    }
    return 0;
}

/*************************************************************/
/*     判断车速大于阈值的持续时间，主要去除冲击速度          */
/*************************************************************/
// 若相对速度大于8km/h，将m_cSpeedStandbyCount++；否则，m_cSpeedStandbyCount = 0,并返回1。
// 若m_cSpeedStandbyCount>30且距离大于0，m_cSpeedStandbyCount设为30，并返回0.
// m_cSpeedStandbyCount为车速大于阈值的持续时间，这里应该算是30次的循环时间。
// 返回1可表示为速度为大于8km/h为瞬时性,返回0可表示为速度为大于8km/h为恒定性。

unsigned char IsSpeedStandby(int disDistance)
{
    if(m_iRelativeSpeed>SPEED_IS_Standby)
        m_cSpeedStandbyCount++;
    else {
        m_cSpeedStandbyCount = 0;
        return 1;
    }
    
    if(m_cSpeedStandbyCount>30 && disDistance>=0){
        m_cSpeedStandbyCount = 30;
        return 0;
    }
    return 1;
}

/*************************************************************/
/*                驾驶员干预子函数                           */
/*************************************************************/
void DriverIntervene(void)
{
    switch(m_cStatus){
        case AEB_STANDBY:
             AlarmOperation(ALARM_ClOSE);
             RelayValveControl(Relay_Valve_Standby);
            break;
            
        case AEB_PRE_BRAKE:
             m_cStatus=AEB_STANDBY;
             RelayValveControl(Relay_Valve_Standby);
             AlarmOperation(ALARM_ClOSE);
            break;
            
        case AEB_INTERVAL_BRAKE:
             m_cStatus=AEB_STANDBY;
             RelayValveControl(Relay_Valve_Standby);
             AlarmOperation(ALARM_ClOSE);
            break;
            
        case AEB_BOOST_BRAKE:
            if(m_cBrakePedal == 0){
                m_cStatus = AEB_STANDBY;
                BrakeLightOperation(LED_OFF);
                AlarmOperation(ALARM_ClOSE);
            }else{
                 PWMDTY23 = 0;
                 MotorValveControl = 0;
            }
            break;
        case AEB_RELEASE_BRAKE:
            AlarmOperation(ALARM_ClOSE);
            if(OilPressureAD<PRESSURE_THRESHOLD){
                m_cStatus = AEB_STANDBY;
                BrakeLightOperation(LED_OFF);
                AlarmOperation(ALARM_ClOSE);
            }else{
                RelayValveControl(Relay_Valve_ReleaseBrake);
            }
            break;
            
        default:
            break;
    }
    
}

/*************************************************************/
/*                    计算目标制动压力                       */
/*************************************************************/
void CalculatePressureGoal()
{
    unsigned int percert = 0;
    unsigned int pidCalc = 0;
    unsigned int controlSpeed = m_iRelativeSpeedMax/2;
    int tmp0 = 0;
    int tmp1 = 0;
    int tmp2 = 0;
    float tmp3 = 0;
    float tmp4 = 0;
    float tmp5 = 0;
    float tmp6 = 0;
    float tmp7 = 0;
    if(m_iRelativeSpeedMax==0)m_iRelativeSpeedMax = m_iRelativeSpeed;
    tmp0 = m_iRelativeSpeed;
    tmp1 = m_uDistance.m_iDistance;
    tmp2 = 10*HEADER;
    
    tmp3 = (float)(tmp0);
    tmp4 = (float)(tmp1);
    tmp5 = (float)(tmp2);
    
    if((m_cStatus==3)&&((m_uAcc.m_iAcc>400)&&(m_uAcc.m_iAcc<981))){
        sPID.m_iRef = REF_ACC;
        sPID.m_iFeedBack = m_uAcc.m_iAcc;
        m_iControlACC = PIDCalc() + sPID.m_iRef;
    }else {
        m_iControlACC = REF_ACC;
    }
    
    if((m_cStatus==3)&&(controlSpeed>m_iRelativeSpeed)){
        tmp6 = (tmp3*tmp3)*3.846;
        if(tmp4>tmp5)tmp7 = (tmp4-tmp5)
            ;
        else
            tmp7 = tmp4;
        sPID.m_iRef = (int)(tmp6/tmp7);
        //13 = 3.6*3.6
        
        if(sPID.m_iRef>980)
            sPID.m_iRef = 980;
        else if(sPID.m_iRef<1)
            sPID.m_iRef = 0;
        
        sPID.m_iFeedBack = m_uAcc.m_iAcc;
        m_iControlACC = PIDCalc() + sPID.m_iRef;
    }
    
    percert = m_iControlACC/10;
    if(percert>65)
        percert = 65;
    else if(percert<1)
        percert = 1;
    
    m_iPressureGoalPedal = 967+(1126-967)*percert/100;
    //制动踏板满行程的50%，制动踏板初值967，最大行程1126；967+（1126-967）*50%
    if(m_iPressureGoalPedal>1000)
        m_iPressureGoalPedal = 1000;
    m_iPressureGoalPWM = PWM_PERIOD;
    
}
/*************************************************************/
/*                    AEB第三阶段操作                        */
/*************************************************************/
void AEB_BoostBrake(void)
{
    CalculatePressureGoal();
    
    switch(m_cStatus){
     case AEB_STANDBY:                                   //待机模式
         if(m_uBrakePedalOffset.m_iBrakePedalOffset<m_iPressureGoalPedal)//若 小于目标压力
         {
             m_cStatus = AEB_BOOST_BRAKE;                //更新状态
             BrakeLightOperation(LED_ON);                //没有这个函数？？
             AlarmOperation(ALARM_URGENT_STATE);         //没有这个函数？？
             RelayValveControl(Relay_Valve_BoostBrake);  //比例继动阀控制
         }
         break;
     
     case AEB_PRE_BRAKE:                                  //预制动模式
         if(m_uBrakePedalOffset.m_iBrakePedalOffset<m_iPressureGoalPedal)
         {
             m_cStatus = AEB_BOOST_BRAKE;
             BrakeLightOperation(LED_ON);
             AlarmOperation(ALARM_URGENT_STATE);
             RelayValveControl(Relay_Valve_BoostBrake);
         }
         else
         {
             m_cStatus = AEB_STANDBY;
             AlarmOperation(ALARM_ClOSE);
         }
         break;
            
            67591903
     
     case AEB_INTERVAL_BRAKE:                                  //间歇制动模式
         if(m_uBrakePedalOffset.m_iBrakePedalOffset<m_iPressureGoalPedal)
         {
             m_cStatus = AEB_BOOST_BRAKE;
             BrakeLightOperation(LED_ON);
             AlarmOperation(ALARM_URGENT_STATE);
             RelayValveControl(Relay_Valve_BoostBrake);
         }
         else
         {
             m_cStatus = AEB_STANDBY;
             AlarmOperation(ALARM_ClOSE);
         }
         break;
     
     case AEB_BOOST_BRAKE:                                  //自主制动模式
         if(m_uBrakePedalOffset.m_iBrakePedalOffset>m_iPressureGoalPedal)
         {
             if(OilPressureAD<PRESSURE_THRESHOLD)
             {
                 m_cStatus = AEB_STANDBY;
                 BrakeLightOperation(LED_OFF);
             }
             else
             {
                 m_cStatus = AEB_RELEASE_BRAKE;
                 m_iRelativeSpeedMax = 0;
                 RelayValveControl(Relay_Valve_ReleaseBrake);
                 AlarmOperation(ALARM_ClOSE);
             }
         }
         else
         {
             AlarmOperation(ALARM_URGENT_STATE);
             RelayValveControl(Relay_Valve_BoostBrake);
         }
         break;
            
     case AEB_RELEASE_BRAKE:                                //释放制动踏板模式
         if(m_uBrakePedalOffset.m_iBrakePedalOffset>m_iPressureGoalPedal)
         {
             AlarmOperation(ALARM_ClOSE);
             if(OilPressureAD<PRESSURE_THRESHOLD)
             {
                 m_cStatus = AEB_STANDBY;
                 BrakeLightOperation(LED_OFF);
             }
         }
         else
         {
             RelayValveControl(Relay_Valve_BoostBrake);
             m_cStatus = AEB_BOOST_BRAKE;
             AlarmOperation(ALARM_URGENT_STATE);
         }
         break;
     
     default:
        break;
     }
    
}

/*************************************************************/
/*                    AEB第二阶段操作                        */
/*************************************************************/
void AEB_IntervalBrake(void)
{
    switch(m_cStatus){
     case AEB_STANDBY:
     if(m_uBrakePedalOffset.m_iBrakePedalOffset<BRAKE_PEDAL_SOFT_INTERVENTION){
     m_cStatus = AEB_INTERVAL_BRAKE;
     BrakeLightOperation(LED_ON);
     AlarmOperation(ALARM_INTERVAL_STATE);
     RelayValveControl(Relay_Valve_IntervalBrake);
     m_cIntervalBrakeCount++;
     }
     break;
     
     case AEB_PRE_BRAKE:
     if(m_uBrakePedalOffset.m_iBrakePedalOffset>BRAKE_PEDAL_SOFT_INTERVENTION){
         m_cStatus = AEB_STANDBY;
         AlarmOperation(ALARM_ClOSE);
         RelayValveControl(Relay_Valve_Standby);
     }else{
         m_cStatus = AEB_INTERVAL_BRAKE;
         BrakeLightOperation(LED_ON);
         AlarmOperation(ALARM_INTERVAL_STATE);
         RelayValveControl(Relay_Valve_IntervalBrake);
         m_cIntervalBrakeCount++;
     }
     break;
     
     case AEB_INTERVAL_BRAKE:
     if(m_uBrakePedalOffset.m_iBrakePedalOffset>BRAKE_PEDAL_SOFT_INTERVENTION){
         m_cStatus = AEB_STANDBY;
         AlarmOperation(ALARM_ClOSE);
         RelayValveControl(Relay_Valve_Standby);
     }else{
         RelayValveControl(Relay_Valve_IntervalBrake);
         AlarmOperation(ALARM_INTERVAL_STATE);
         m_cIntervalBrakeCount++;
     }
     break;
     
     case AEB_BOOST_BRAKE:
     if(m_uBrakePedalOffset.m_iBrakePedalOffset>BRAKE_PEDAL_SOFT_INTERVENTION){
         if(OilPressureAD<PRESSURE_THRESHOLD){
             m_cStatus = AEB_STANDBY;
             BrakeLightOperation(LED_OFF);
         }else{
             m_cStatus = AEB_RELEASE_BRAKE;
             RelayValveControl(Relay_Valve_ReleaseBrake);
             AlarmOperation(ALARM_ClOSE);
         }
     }else{
         if(OilPressureAD<PRESSURE_THRESHOLD){
             m_cStatus = AEB_STANDBY;
             BrakeLightOperation(LED_OFF);
         }else{
             m_cStatus = AEB_RELEASE_BRAKE;
             RelayValveControl(Relay_Valve_ReleaseBrake);
         }
     }
     break;
     
    case AEB_RELEASE_BRAKE:
     AlarmOperation(ALARM_ClOSE);
     if(OilPressureAD<PRESSURE_THRESHOLD){
         m_cStatus = AEB_STANDBY;
         BrakeLightOperation(LED_OFF);
     }else{
         RelayValveControl(Relay_Valve_ReleaseBrake);
     }
     break;
     
     default:
     break;
     }
}

/*************************************************************/
/*                    AEB第一阶段操作                        */
/*************************************************************/
void AEB_PreBrake(void)
{
    m_iRelativeSpeedMax = 0;
    switch(m_cStatus){
     case AEB_STANDBY:
         if(m_uBrakePedalOffset.m_iBrakePedalOffset<BRAKE_PEDAL_SOFT_INTERVENTION){
             m_cStatus = AEB_PRE_BRAKE;
             AlarmOperation(ALARM_GERNERAL_STATE);
             RelayValveControl(Relay_Valve_PreBrake);
         }
     break;
     
     case AEB_PRE_BRAKE:
         if(m_uBrakePedalOffset.m_iBrakePedalOffset>BRAKE_PEDAL_SOFT_INTERVENTION){
             m_cStatus = AEB_STANDBY;
             RelayValveControl(Relay_Valve_Standby);
             AlarmOperation(ALARM_ClOSE);
         }else{
             m_cStatus = AEB_PRE_BRAKE;
             AlarmOperation(ALARM_GERNERAL_STATE);
             RelayValveControl(Relay_Valve_PreBrake);
         }
     break;
     
     case AEB_INTERVAL_BRAKE:
         if(m_uBrakePedalOffset.m_iBrakePedalOffset>BRAKE_PEDAL_SOFT_INTERVENTION){
             m_cStatus = AEB_STANDBY;
             AlarmOperation(ALARM_ClOSE);
             RelayValveControl(Relay_Valve_Standby);
         }else{
             m_cStatus = AEB_PRE_BRAKE;
             RelayValveControl(Relay_Valve_PreBrake);
         }
     break;
     
     case AEB_BOOST_BRAKE:
         if(m_uBrakePedalOffset.m_iBrakePedalOffset>BRAKE_PEDAL_SOFT_INTERVENTION){
             if(OilPressureAD<PRESSURE_THRESHOLD){
                 m_cStatus = AEB_STANDBY;
                 BrakeLightOperation(LED_OFF);
             }else{
                 m_cStatus = AEB_RELEASE_BRAKE;
                 RelayValveControl(Relay_Valve_ReleaseBrake);
                 AlarmOperation(ALARM_ClOSE);
             }
         }else{
             if(OilPressureAD<PRESSURE_THRESHOLD){
                 m_cStatus = AEB_STANDBY;
                 BrakeLightOperation(LED_OFF);
             }else{
                 m_cStatus = AEB_RELEASE_BRAKE;
                 RelayValveControl(Relay_Valve_ReleaseBrake);
             }
         }
     break;
            
     case AEB_RELEASE_BRAKE:
         AlarmOperation(ALARM_ClOSE);
         if(OilPressureAD<PRESSURE_THRESHOLD){
             m_cStatus = AEB_STANDBY;
             BrakeLightOperation(LED_OFF);
         }else{
             RelayValveControl(Relay_Valve_ReleaseBrake);
         }
         break;
         
     default:
        break;
     }
}

/*************************************************************/
/*                 比例继动阀控制                            */
/*        间歇制动次数有控制周期时间决定（30和60两个参数）   */
/*        间歇制动目前的持续时间为0.1s,共两次                */
/*************************************************************/
void RelayValveControl(unsigned char relayValveOperation)
{
    switch(relayValveOperation) {
     case Relay_Valve_Standby:
         m_cPWMDuty = 0;
         m_cMotorValveControl = 1;
         m_cBoostBrakeDutyRatioIncr = 0;
         break;
     
     case Relay_Valve_PreBrake:
         m_cPWMDuty = Motor_PRE_BRAKE_DUTY_RATIO;
         m_cMotorValveControl = 0;
         break;
     
     case Relay_Valve_IntervalBrake:
         if(m_cIntervalBrakeCount<4||(m_cIntervalBrakeCount<12 && m_cIntervalBrakeCount>=8)){
             m_cPWMDuty = Motor_INTERVAL_BRAKE_DUTY_RATIO;
             m_cMotorValveControl = 0;
             }
         else if(4<=m_cIntervalBrakeCount&&m_cIntervalBrakeCount<8){
             m_cPWMDuty = 0;
             m_cMotorValveControl = 1;
             }
         else if(m_cIntervalBrakeCount>=12){
             m_cIntervalBrakeCount = 0;
             m_cPWMDuty = Motor_PRE_BRAKE_DUTY_RATIO;
             m_cMotorValveControl = 0;
             }
         
         break;
     
     case Relay_Valve_BoostBrake:
         if(m_cBoostBrakeDutyRatioIncr>=100){
             m_cPWMDuty = m_iPressureGoalPWM;
             m_cMotorValveControl = 0;
             m_cBoostBrakeDutyRatioIncr = 100;
             }
         else{
             unsigned int pwmDutyRatio = 0;
             m_cPWMDuty = PWM_PERIOD;
             m_cMotorValveControl = 0;
             m_cBoostBrakeDutyRatioIncr+=30;
             }
         
         break;
     
     case Relay_Valve_ReleaseBrake:
         if(m_cBoostBrakeDutyRatioIncr<=1||m_cBoostBrakeDutyRatioIncr>=100)
             m_cBoostBrakeDutyRatioIncr = 0;
         if(m_cBoostBrakeDutyRatioIncr!=0&&OilPressureAD>PRESSURE_THRESHOLD){
             unsigned int pwmDutyRatio = m_iPressureGoalPWM*m_cBoostBrakeDutyRatioIncr/100;
             m_cPWMDuty = pwmDutyRatio;
             m_cMotorValveControl = 0;
             m_cBoostBrakeDutyRatioIncr-=10;
             }
         else{
             m_cPWMDuty = 0;
             m_cMotorValveControl = 1;
             }
         break;
     
     default:
     break;
     }
     
     SendControlCommand();
}



/*************************************************************/
/*                      判断当前人工干预状态                 */
/*************************************************************/
void JudgeManualInter(){
    //判断制动踏板人工干预
    if(AutoControl_Mode==1){
        if(Bakesignal<50){
            AutoControl_Mode = 2;
        }
    }
    
    /*启停遥控器接入底层控制*/
    if(AutoMode==1&&EStop == '4'){
       EStopStatus = 1;
    }
    
    if(EStopStatus==1&&EStop == '1'){
       EStopStatus = 0;
    }
}
/*************************************************************/
/*                      判断当前控制状态                     */
/*************************************************************/
void JudgeControlMode() {
    
    JudgeManualInter();
    
    if(EStopStatus==1){
        AccPedalCtrl = 0;
        AutoEStopBrake();
        return;
    }
    
    /*---------------------主动制动控制-------------------------*/
    if(AutoControl_Mode==1&&BrakeEnable==1) {
        if(((ExpectPressure==0)&&(MasterCylinderPrsSenseAD>=8))){
            StrMotor_DirL =1;
            StrMotor_DirR =0;
        } if((ExpectPressure>0)||(MasterCylinderPrsSenseAD<8)){
            StrMotor_DirL = 0;                  // 正转
            StrMotor_DirR = 1;
        }
        
        m_iMotorDuty=PID_Pressure(ExpectPressure); //主缸压力PID跟踪调节
        
        
        if(ABS(m_iMotorDuty)<=20)PWMDTY01 = 0;
        if(m_iMotorDuty>20){
            PWMDTY01 = m_iMotorDuty;
            StrMotor_DirL = 0;                     // 正转
            StrMotor_DirR = 1;
        }
        
        if(m_iMotorDuty<-20){
            PWMDTY01 = ABS(m_iMotorDuty);
            StrMotor_DirL = 1;                     // 反转
            StrMotor_DirR = 0;
        }
        
        BrakeLight_Enable;
        
    }else{
        /*--------------以主缸压力判断消除--------------------------*/
        if(((PedalLocationAD2<=265)&&(MasterCylinderPrsSenseAD>=8))){
            StrMotor_DirL = 1;                      // 反转消除残压
            StrMotor_DirR = 0;
            PWMDTY01=100;
            
        } if((PedalLocationAD2>265)||(MasterCylinderPrsSenseAD<8)){
            StrMotor_DirL = 0;                      // 正转
            StrMotor_DirR = 1;
        }
        Current_Set();
        Current_PID();
        m_cBrakeStatus == NORMAL;
        BrakeLight_DisEnable;
    }
    
    /*---------------------主动油门控制-------------------------*/
    if(AutoControl_Mode==1&&AccPedalEnable==1){
        AutoThrottle();
    }
    /*---------------------主动灯光控制-------------------------*/
    if(LightPeriodFlag==1){
        AutoLight();
        LightPeriodFlag = 0;
    }
    
    if(AutoControl_Mode==1&&(AccPedalEnable==1||SpeedControlEnable == 1)){
        ;
    }else
        AccPedalCtrl = 0;
    
}

/*************************************************************/
/*                     目标位移控制整定                      */
/*************************************************************/
void MC_Location_PID(void) {
    
    Cylinder_Set();                                   //主缸位置给定
    motorDuty=PID_Cylinder(Set_Cylinder);             //主缸位置跟踪
    
    if(motorDuty <= 0) motorDuty = 0;
    
    if((motorDuty > 0) && (motorDuty <=600)) MotorDuty = motorDuty;
    
    if(motorDuty > 640) MotorDuty = 600;
    
    
    /*--------------------------占空比输出处理--------------------*/
    if(ABS(motorDuty)<=10)MotorDuty = 0;
    
    if(ABS(motorDuty)>=600) MotorDuty = 600;
    
    if(motorDuty>10){
        StrMotor_DirL=0 ;            // 正转
        StrMotor_DirR=1 ;
        if(motorDuty>600)MotorDuty = 600;
        MotorDuty = motorDuty;
    } else if(motorDuty<-10){
        StrMotor_DirL = 1;          // 反转
        StrMotor_DirR = 0;
        if(motorDuty<-600) MotorDuty=600;
        MotorDuty = ABS(motorDuty);
    } else {
        
        MotorDuty = 0;
    }
    
}

void CanSend_init()
{
    m_cSendInfo[0].flag=0;
    m_cSendInfo[1].flag=0;
    m_cSendInfo[2].flag=0;
    m_cSendInfo[3].flag=0;
    m_cSendInfo[4].flag=0;
    m_cSendInfo[5].flag=0;
    m_cSendInfo[6].flag=0;
    m_cSendInfo[7].flag=0;
    
    m_cSendInfo[0].id=0x21;
    m_cSendInfo[1].id=0x22;
    m_cSendInfo[2].id=0x23;
    m_cSendInfo[3].id=0x24;
    m_cSendInfo[4].id=0x30;
    m_cSendInfo[5].id=0x31;
    m_cSendInfo[6].id=0x32;
    m_cSendInfo[7].id=0x33;
}

void CanSend_data()
{
    Bool Can_ok;
    uint i;
    for(i=0;i<8;i++)
    {
        if((m_cSendInfo[i].flag)==1){
            Can0_msg_send.id = m_cSendInfo[i].id;
            Can0_msg_send.data[0] = m_cSendInfo[i].data[0];
            Can0_msg_send.data[1] = m_cSendInfo[i].data[1];
            Can0_msg_send.data[2] = m_cSendInfo[i].data[2];
            Can0_msg_send.data[3] = m_cSendInfo[i].data[3];
            Can0_msg_send.data[4] = m_cSendInfo[i].data[4];
            Can0_msg_send.data[5] = m_cSendInfo[i].data[5];
            Can0_msg_send.data[6] = m_cSendInfo[i].data[6];
            Can0_msg_send.data[7] = m_cSendInfo[i].data[7];
            Can0_msg_send.len = data_len_TX;
            Can0_msg_send.RTR = FALSE;
            Can0_msg_send.prty = 0;
            Can_ok=MSCAN0SendMsg(Can0_msg_send); /**/
            
            if(Can_ok==1){
                m_cSendInfo[i].flag =0;
            }
        }
        
        
    }
    
}


void Candata_Inbufer()
{
    unsigned int brakeMotorCurrent = (int)(BrakeMotorCurrent*10);
    unsigned int brakeTargetCurrent = (int)(Set_Current*10);
    unsigned int PWMduty;
    PWMduty= PWMDTY01;
    if(CylinderLocation>25) CylinderLocation = 25;
    if(PedalLocation>25)PedalLocation = 25;
    if(MasterCylinderPrs>25)MasterCylinderPrs = 25;
    
    m_cSendInfo[0].flag = 1;
    m_cSendInfo[0].id = 0x21;
    m_cSendInfo[0].data[0] = (uchar)(brakeTargetCurrent>>8);
    m_cSendInfo[0].data[1] = (uchar)(brakeTargetCurrent);
    m_cSendInfo[0].data[2] = m_iRelativeSpeed>>8;
    m_cSendInfo[0].data[3] = m_iRelativeSpeed;
    m_cSendInfo[0].data[4] = m_uDistance.m_iDistance>>8;
    m_cSendInfo[0].data[5] = m_uDistance.m_iDistance;
    m_cSendInfo[0].data[6] = m_uSpeed.m_iSpeed>>8;
    m_cSendInfo[0].data[7] = m_uSpeed.m_iSpeed;
    
    
    m_cSendInfo[1].flag = 1;    //HMI通信帧
    m_cSendInfo[1].id = 0x123;
    m_cSendInfo[1].data[0] = 0x01;//系统默认正常
    m_cSendInfo[1].data[1] = m_cStatus;
    m_cSendInfo[1].data[2] = 0x03;//light num
    m_cSendInfo[1].data[3] = Alarmlabel;
    m_cSendInfo[1].data[4] = 0;
    m_cSendInfo[1].data[5] = 0;
    m_cSendInfo[1].data[6] = 0;
    m_cSendInfo[1].data[7] = 0;
    
     m_cSendInfo[1].flag = 1;
     m_cSendInfo[1].id = 0x22;
     m_cSendInfo[1].data[0] = (uchar)(MasterCylinderPrs*10);
     m_cSendInfo[1].data[1] = (uchar)(ExpectPressure);
     m_cSendInfo[1].data[2] = (uchar)(PWMduty>>8);
     m_cSendInfo[1].data[3] = (uchar)(PWMduty);
     m_cSendInfo[1].data[4] = (uchar)(CylinderLocation*10);
     m_cSendInfo[1].data[5] = (uchar)(PedalLocation*10);
     m_cSendInfo[1].data[6] = (uchar)(brakeMotorCurrent>>8);
     m_cSendInfo[1].data[7] = (uchar)(brakeMotorCurrent);
     
     m_cSendInfo[2].flag = 1;
     m_cSendInfo[2].id = 0x23;
     m_cSendInfo[2].data[0] = (uchar)(m_iPWMP>>8);
     m_cSendInfo[2].data[1] = (uchar)(m_iPWMP);
     m_cSendInfo[2].data[2] = (uchar)(m_iPWMI>>8);
     m_cSendInfo[2].data[3] = (uchar)(m_iPWMI);
     m_cSendInfo[2].data[4] = (uchar)(m_iPWMD>>8);
     m_cSendInfo[2].data[5] = (uchar)(m_iPWMD);
     m_cSendInfo[2].data[6] = ((ModeChange & 0x03)<<6);
     m_cSendInfo[2].data[7] = (uchar)(m_cAccPedal);
     
     m_cSendInfo[3].flag = 1;
     m_cSendInfo[3].id = 0x24;
     m_cSendInfo[3].data[0] = (uchar)(m_iPWMError>>8);
     m_cSendInfo[3].data[1] = (uchar)(m_iPWMError);
     m_cSendInfo[3].data[2] = (uchar)(m_iPWMIntePre>>8);
     m_cSendInfo[3].data[3] = (uchar)(m_iPWMIntePre);
     m_cSendInfo[3].data[4] = (uchar)(m_iPWMIntegral>>8);
     m_cSendInfo[3].data[5] = (uchar)(m_iPWMIntegral);
     m_cSendInfo[3].data[6] = (uchar)(m_iPWMPError>>8);
     m_cSendInfo[3].data[7] = (uchar)(m_iPWMPError);
     
     m_cSendInfo[4].flag = 1;
     m_cSendInfo[4].id = 0x25;
     m_cSendInfo[4].data[0] = (uchar)((int)(PedalForce_CAN)>>8);
     m_cSendInfo[4].data[1] = (uchar)((int)(PedalForce_CAN));
     m_cSendInfo[4].data[2] = (uchar)((int)(PedalLocation*100.0)>>8);
     m_cSendInfo[4].data[3] = (uchar)((int)(PedalLocation*100.0));
     m_cSendInfo[4].data[4] = (uchar)((int)(CylinderLocation*100.0)>>8);
     m_cSendInfo[4].data[5] = (uchar)((int)(CylinderLocation*100.0));
     m_cSendInfo[4].data[6] = (uchar)((int)(MasterCylinderPrs_CAN)>>8);
     m_cSendInfo[4].data[7] = (uchar)((int)(MasterCylinderPrs_CAN));
     
     m_cSendInfo[5].flag = 1;
     m_cSendInfo[5].id = 0x26;
     m_cSendInfo[5].data[0] = (uchar)((int)( ECU_Timer)>>8);
     m_cSendInfo[5].data[1] = (uchar)((int)( ECU_Timer));
     m_cSendInfo[5].data[2] = (uchar)((int)(0)>>8);                //积分项高8位
     m_cSendInfo[5].data[3] = (uchar)((int)(0));                   //积分项低8位
     m_cSendInfo[5].data[4] = (uchar)((int)(0)>>8);                //微分项高8位
     m_cSendInfo[5].data[5] = (uchar)((int)(0));                   //微分项低8位
     m_cSendInfo[5].data[6] = (uchar)((int)(0)>>8);                //微分项高8位
     m_cSendInfo[5].data[7] = (uchar)((int)(0));                   //微分项低8位
     
     m_cSendInfo[6].flag = 1;
     m_cSendInfo[6].id = 0x27;
     m_cSendInfo[6].data[0] = (uchar)((int)(SetPrs_CAN)>>8);             //踏板位置信号高8位
     m_cSendInfo[6].data[1] = (uchar)((int)(SetPrs_CAN));                //踏板位置信号低4位和制动踏板力高4位
     m_cSendInfo[6].data[2] = (uchar)((int)(BrakeMotorCurrent_CAN)>>8);  //制动踏板力低8位
     m_cSendInfo[6].data[3] = (uchar)((int)(BrakeMotorCurrent_CAN));     //主缸行程高8位
     m_cSendInfo[6].data[4] = (uchar)((int)(motorDuty_CAN)>>8);          //主缸行程低4位和主缸压力高4位
     m_cSendInfo[6].data[5] = (uchar)((int)(motorDuty_CAN));             //主缸压力低8位
     m_cSendInfo[6].data[6] = (uchar)((int)(Set_Cylinder*100.0)>>8);           //主缸位置信号高8位
     m_cSendInfo[6].data[7] = (uchar)((int)(Set_Cylinder*100.0));              //主缸位置信号低8位
     
     m_cSendInfo[7].flag = 1;
     m_cSendInfo[7].id = 0x28;
     m_cSendInfo[7].data[0] = (uchar)((int)( Cylinder_Err*100.0)>>8);  //主缸位置信号高8位
     m_cSendInfo[7].data[1] = (uchar)((int)( Cylinder_Err*100.0));     //主缸位置信号低8位
     m_cSendInfo[7].data[2] = (uchar)((int)(m_iPWMP)>>8);                //比例项高8位
     m_cSendInfo[7].data[3] = (uchar)((int)(m_iPWMP));                   //比例项低8位
     m_cSendInfo[7].data[4] = (uchar)((int)(m_iPWMI)>>8);                //积分项高8位
     m_cSendInfo[7].data[5] = (uchar)((int)(m_iPWMI));                   //积分项低8位
     m_cSendInfo[7].data[6] = (uchar)((int)(m_iPWMD)>>8);                //微分项高8位
     m_cSendInfo[7].data[7] = (uchar)((int)(m_iPWMD));                   //微分项低8位  */
    
}



/*
if1:系统基准时间（系统基准时间是什么？？？)
 在(0-1)*200微秒，无动作
 在(1-2)*200微秒，若tasksta[1]不为1，则设taskid =1
 在(2-5)*200微秒，若tasksta[2]不为1，则设taskid =2
 ...
 在(15-16)*200微秒，若tasksta[6]不为1，则设taskid =6
 在(16-25)*200微秒，则设taskid =7
 在(25-**)*200微秒，tasksta所有值设为0，sysTimer重置为0，Preset_time++， CanSendTimer++;
 if2:CanSendTimer>=4的话，CanSendTimer = 0， CanSendFlag  = 1;
 if3:Preset_time>5000的话，Preset_time=0;
 
 这个函数的核心作用是什么？
 */
void task_Sacn()
{
    if(sysTimer<1)// (0-1)*200us
    {
        
    }
    else if(sysTimer<2)// (1-2)*200us
    {
        if(tasksta[1]!=1) {  //一个周期只执行一次
            
            taskid =1;
        }
        
        
    }
    else if(sysTimer<5) //(2-5)*200us
    {
        if(tasksta[2]!=1) {
            taskid =2;
        }
        
    }
    else if(sysTimer<6) //(5-6)*200us
    {
        if(tasksta[3]!=1) {
            taskid =3;
        }
        
    }
    else if(sysTimer<10) //(6-10)*200us
    {
        if(tasksta[4]!=1) {
            taskid =4;
        }
    }
    else if(sysTimer<15) //(10-15)*200us
    {
        if(tasksta[5]!=1) {
            taskid =5;
        }
    }
    else if(sysTimer<16) //(15-16)*200us
    {
        if(tasksta[6]!=1)
        {
            taskid =6;
        }
        
    }
    else if(sysTimer<25) //(16-25)*200us
    {
        taskid =7;
        
    }
    else {
        tasksta[1]=0;
        tasksta[2]=0;
        tasksta[3]=0;
        tasksta[4]=0;
        tasksta[5]=0;
        tasksta[6]=0;
        tasksta[7]=0;
        sysTimer=0;
        Preset_time++;
        CanSendTimer++;
    }
    
    if(CanSendTimer>=4) //5ms控制周期，20ms CAN发送周期
    {
        CanSendTimer = 0;
        CanSendFlag  = 1;
    }
    if(Preset_time>5000)
        Preset_time=0;
}



/*************************************************************/
/*                         主函数                            */
/*************************************************************/
int main(void){
    unsigned char isEmergencySW = 0;                  //是否在紧急转向，若在紧急转向，则退出AEB模式
    unsigned char isSpeedStandby = 1;                 //这是什么？
    unsigned char isWheelNotReturnToCenter = 0;       //这是什么？
    unsigned char isCalcReSpeed = 0;                  //这是什么？
    unsigned char currentIndex = 0;                   //这是什么？
    
    int wheelAngle = 0;
    unsigned int speedTmp =0 ;
    int disDistance = 6500;
    int distance = 0;
    int countMinSpeed = 0;                           //这个是什么？
    int countPeriod = 0;
    
    DisableInterrupts;                               //这是什么？
    INIT_ALL();                                      //初始化函数
    EnableInterrupts;                                //这是什么？
    StrMotor_DrvEN_Enable;                           //这是什么？
    PID_init();                                      //初始化PID参数--制动压力控制
    CanSend_init();                                  //初始化CAN发送函数
    for(;;)
    {
        switch(taskid){
            case 0: task_Sacn();
                break;
                
            case 1:
                ADSampleFilter();
                tasksta[1]=1;
                taskid=0;
                break;
                
            case 2:
                AD_to_real();
                tasksta[2]=1;
                taskid=0;
                break;
                
            case 3:
                sent_pc_CAN();
                tasksta[3]=1;
                taskid=0;
                break;
                
            case 4:
                MC_Location_PID();
                tasksta[4]=1;
                taskid=0;
                break;
                
            case 5:
                FindDangerObject1();
                
                //由于前方障碍物静止，把车速赋值给相对速度
                m_iRelativeSpeed = VechileSpeed;
                
                //判断是否在紧急转向状态
                isEmergencySW = IsEmergencySteeringWheel();
                
                //判断是否车速在待机状态
                isSpeedStandby = IsSpeedStandby(disDistance);
                
                //若速度小于某一较小值且处于全力制动状态，则恢复到正常状态。
                //这里时经过一段全力制动后，速度下降到一较小值时，解除制动状态。
                if(m_uSpeed.m_iSpeed<SPEED_IS_ZERO && m_cBoostBrake==1)
                    m_cBoostBrake = 0;
                
                //若处于急转向 或 处于待机状态且正常状态 或 处于**状态 进行驾驶员干预处理，安全系数设为200.
                if((isEmergencySW == 1)||(isSpeedStandby==1&&m_cBoostBrake==0)\
                   || m_cBrakePedal == 0){
                    
                    DriverIntervene();//驾驶员干预处理
                    countMinSpeed = 0;
                    m_iSafetyFactor = 200;
                }else{
                //未紧急转向等其他情况下
                    distance = m_uDistance.m_iDistance/10;
                    m_iSafetyFactor= CountSafetyFactor(distance,m_iRelativeSpeed,m_uSpeed.m_iSpeed);
                    //计算SafetyFactor安全系数
                    
                    //若安全系数低于自主制动安全系数或处于全力制动状态
                    if( m_iSafetyFactor<=SAFETY_FACTOR_BOOST_BRAKE||m_cBoostBrake==1){
                        if(m_cBoostBrake==0)    //若若安全系数低于自主制动安全系数但未处于全力制动状态
                            m_cBoostBrake = 1;  //设为处于全力制动状态
                            AEB_BoostBrake();   //进行AEB第三阶段操作
                    }else if(m_iSafetyFactor<=SAFETY_FACTOR_INTERVAL_BRAKE){
                        AEB_IntervalBrake();
                    }else if(m_iSafetyFactor<=SAFETY_FACTOR_PRE_BRAKE){
                        AEB_PreBrake();
                    }else{
                        DriverIntervene();
                    }
                    
                    //档位控制，现在无用了
                    /*if(Gear_enable==1) {
                        //计算SafetyFactor
                         m_iSafetyFactor=CountSafetyFactor(distance,\
                                                       m_iRelativeSpeed,m_uSpeed.m_iSpeed);
                     } else{
                         m_iSafetyFactor = 200;
                     }*/
                }
                
                 JudgeControlMode();  线控功能
                 if(AutoOnOffSignal<=500 && AutoControl_Mode==1){
                     ;      //这是什么？无操作么？
                 }else{
                     HighBeam = LED_OFF;
                 }
                 
                 if(AutoOnOffSignal>500){
                     InitControlPar();
                     InitKFPWM();
                 }
                 
                 if(m_iCountTimeOut>50){
                     AutoControl_Mode = 0;
                     m_iCountTimeOut = 80;
                 }
                 if(HighBeam==LED_ON)
                     SysMode = 1;
                 else
                     SysMode = 0;
                 tasksta[5]=1;
                 taskid=0;
                 break;   //case5结尾
                
            case 6:
                if(CanSendFlag==1){
                    Candata_Inbufer();
                    CanSendFlag=0;
                }
                tasksta[6]=1;
                taskid=0;
                break;
                
            case 7:
                CanSend_data();
                tasksta[7]=1;
                taskid=0;
                break;
        }
        
        
     ADSampleFilter();   //70us   task1
     AD_to_real();       //220us  task2
     sent_pc_CAN();      //90us
    
     MC_Location_PID();  //260us     task3
    
     CAN0SendQTData();
    
    
     /*-------线控主动制动程序        ------------*/
    
     JudgeControlMode();  //260us          tsak4
        
        
     if(AutoOnOffSignal<=500&&AutoControl_Mode==1){
         ;
     }else{
         HighBeam = LED_OFF;
     }
         
     if(AutoOnOffSignal>500){
         InitControlPar();   //10us
         InitKFPWM();
     }
     
     if(m_iCountTimeOut>50){
         AutoControl_Mode = 0;
         m_iCountTimeOut = 80;
     }
     
    if(HighBeam==LED_ON){
         SysMode = 1;
    }else{
         SysMode = 0;
    }
     
     if(SendOutFlag==2){
         m_iCountTimeOut++;
         CANSendPCData();
         SendOutFlag=0;
         AutoControl_ModeLast = AutoControl_Mode;
     }
        
    }//for块结尾。
    
    return 0;
}
