#include "derivative.h" /* include peripheral declarations */
#include "init.h"

/*************************************************************/
/*                      初始化锁相环                         */
/*************************************************************/
void INIT_PLL(void)
{
    CLKSEL &= 0x7f;       //set OSCCLK as sysclk
    PLLCTL &= 0x8F;       //Disable PLL circuit
    CRGINT &= 0xDF;
    
#if(BUS_CLOCK == 40000000)
    SYNR = 0x44;
#elif(BUS_CLOCK == 32000000)
    SYNR = 0x43;
#elif(BUS_CLOCK == 24000000)
    SYNR = 0x42;
#endif
    
    REFDV = 0x81;         //PLLCLK=2×OSCCLK×(SYNR+1)/(REFDV+1)＝64MHz ,fbus=32M
    PLLCTL =PLLCTL|0x70;  //Enable PLL circuit
    asm NOP;
    asm NOP;
    while(!(CRGFLG&0x08)); //PLLCLK is Locked already
    CLKSEL |= 0x80;        //set PLLCLK as sysclk
}

/*************************************************************/
/*                        初始化IIC                          */
/*************************************************************/
void INIT_IIC(void)
{
    IIC0_IBFD = 0x94;   //总线时钟32MHz,设置SCL主频为100KHz
    IIC0_IBCR = 0x80;   //使能IIC模块,禁止中断
    IIC0_IBSR_IBAL = 1; //清除IBAL标志位
}

/*************************************************************/
/*                         初始化串口0                       */
/*************************************************************/
void INIT_SCI0(void)
{
    
    SCI0BD = BUS_CLOCK/16/BAUD; // 设置SCI0波特率为115200
    SCI0CR1 = 0x00;             // 设置SCI0为正常模式，八位数据位，无奇偶校验
    SCI0CR2 = 0x2c;             // 允许发送数据，禁止中断功能
}


/*************************************************************/
/*                      初始化AD模块                         */
/*************************************************************/
void INIT_AD(void)
{
    ATD0CTL2 = 0x40;  //启动A/D模块,快速清零,禁止中断
    ATD0CTL1_SRES=2;  //选用12位模数转换
    ATD0CTL3 = 0x88;   //每次只转换一个通道
    ATD0CTL4 = 0x01;   //AD模块时钟频率为8MHz
}




/*************************************************************/
/*                  模数递减计数器初始化                     */
/*************************************************************/
void INIT_MDC(void)
{
    ECT_MCCTL = 0xc7;         //使能模数计数器，分频常数为16，即模块时钟周期为0.5us，采用模数循环计数方式，允许中断
    ECT_MCCNT = 4000;        //模数常数为20000，则10ms产生一次中断
    ECT_MCCTL_FLMC = 1;       //将模数常数加载到模数计数器
}


/************************************************************/
/*                    初始化ECT模块                         */
/************************************************************/
void INIT_ECT(void)
{
    ECT_TSCR1_TFFCA = 1;   // 定时器标志位快速清除
    ECT_TSCR1_TEN = 1;     // 定时器使能位. 1=允许定时器正常工作; 0=使主定时器不起作用(包括计数器)
    ECT_TIOS  = 0x80;      // 指定通道7为输出比较方式
    ECT_TCTL1 = 0x00;        // 后四个通道设置为定时器与输出引脚断开
    ECT_TCTL2 = 0x00;      // 后四个通道设置为定时器与输出引脚断开
    ECT_TCTL4 = 0x00;        // 设置通道4为捕捉上升沿方式
    ECT_DLYCT = 0x00;        // 延迟控制功能禁止
    ECT_ICOVW = 0x00;        // 对应的寄存器允许被覆盖;  NOVWx = 1, 对应的寄存器不允许覆盖
    ECT_ICSYS = 0x00;        // 禁止IC及PAC的保持寄存器
    ECT_TIE   = 0x00;      // 允许通道2中断
    ECT_TSCR2 = 0x07;        // 预分频系数pr2-pr0:111,时钟周期为4us,
    ECT_TFLG1 = 0xff;        // 清除各IC/OC中断标志位
    ECT_TFLG2 = 0xff;      // 清除自由定时器中断标志位
    
}


/*************************************************************/
/*                        初始化PWM                          */
/*************************************************************/
void INIT_PWM(void)
{
    PWMCTL_CON01= 1;       //通道0、1为8位的PWM
    PWMCTL_CON23= 1;       //通道2、3为8位的PWM
    PWMPOL_PPOL1= 1;       //通道1的极性为高电平有效
    PWMPOL_PPOL3= 1;       //通道3的极性为高电平有效
    PWMPRCLK = 0x00;       //A时钟和B时钟的分频系数为0,频率为32MHz
    PWMSCLA  =  1;         //SA时钟频率为16000KHz
    PWMSCLB  =  1;         //SB时钟频率为16000KHz
    PWMCLK   = 0x0a;       //通道0,1用SA时钟作为时钟源,通道2,3用SB时钟作为时钟源
    PWMCAE   = 0x00;       //脉冲模式为左对齐模式
    PWMPER01  = 800;       //通道1的周期为20000Hz
    PWMPER23  = 800;       //通道3的周期为20000Hz
    PWMDTY01  = 0;         //通道1的占空比为  %
    PWMDTY23  = 0;         //通道3的占空比为  %
    PWME_PWME1 = 1;        //使能通道1
    PWME_PWME3 = 1;        //使能通道3
}

/*************************************************************/
/*                        初始化GPIO                          */
/*************************************************************/
void INIT_GPIO(void)
{
     Dir_OutPut_ElectVavle;
     Dir_OutPut_LeftTurnLight;
     Dir_OutPut_RightTurnLight;
     Dir_OutPut_BrakeLight;
     Dir_OutPut_HighBeam;
     Dir_OutPut_LowBeam;
     Dir_OutPut_WidthLight;
     Dir_OutPut_HornCtrl;
    Dir_OutPut_LeftTurnLight;
    Dir_OutPut_RightTurnLight;
    Dir_OutPut_BrakeLight;
    Dir_OutPut_EEPROM_WP;
    Dir_OutPut_AccPedalCtrl;
    Dir_OutPut_BAT12VIN_EN;
    Dir_OutPut_StrMotor_DirL;
    Dir_OutPut_StrMotor_DirR;
    Dir_OutPut_StrMotor_DrvEN;
    
     Dir_OutPut_ModeLight;
     Dir_OutPut_Reserved1;
     Dir_OutPut_Reserved2;
     Dir_OutPut_LED0;
     Dir_OutPut_LED1;
     
     Dir_Input_StrMotor_Error;
     
     ElectVavle_DisEnable;
     
     HighBeam_DisEnable;
     LowBeam_DisEnable;
     WidthLight_DisEnable;
     HornCtrl_DisEnable; 
    LeftTurnLight_DisEnable;
    RightTurnLight_DisEnable;
    BrakeLight_DisEnable;
    EEPROM_WP_DisEnable;
    AccPedalCtrl_DisEnable;
    ////////////////////////////
    BAT12VIN_EN_Enable;
    StrMotor_DirL_Enable;
    StrMotor_DirR_DisEnable;
    StrMotor_DrvEN_DisEnable;
    
     ModeLight_DisEnable;
     Reserved1_DisEnable;
     Reserved2_DisEnable;
     
     LED0_DisEnable;
     LED1_DisEnable;
    
    
}
