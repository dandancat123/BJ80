#include <hidef.h>

#define LED_ON 0
#define LED_OFF 1
#define BUS_CLOCK         32000000        //总线频率
#define OSC_CLOCK         16000000        //晶振频率
#define BAUD 115200                 //串口波特率

//输出端口
#define ElectVavle          PORTB_PB0
#define LeftTurnLight       PORTB_PB1
#define RightTurnLight      PORTB_PB2
#define BrakeLight          PORTB_PB3
#define HighBeam            PORTB_PB4
#define LowBeam             PORTB_PB5
#define WidthLight          PORTB_PB6
#define HornCtrl            PORTB_PB7
#define EEPROM_WP           PORTA_PA0
#define AccPedalCtrl        PORTA_PA1

#define BAT12VIN_EN         PTJ_PTJ1
#define StrMotor_DirL       PORTK_PK2
#define StrMotor_DirR       PORTK_PK3
#define StrMotor_DrvEN      PORTK_PK5

#define Reserved1           PTP_PTP5
#define Reserved2           PTP_PTP7
#define ModeLight           PTH_PTH7
#define LED0                PTH_PTH4
#define LED1                PTH_PTH5
//#define GearShiftR          PTH_PTH7
//#define GearShiftD          PT1AD0_PT1AD02

#define Dir_OutPut_ElectVavle         DDRB_DDRB0=1
#define Dir_OutPut_LeftTurnLight      DDRB_DDRB1=1
#define Dir_OutPut_RightTurnLight     DDRB_DDRB2=1
#define Dir_OutPut_BrakeLight         DDRB_DDRB3=1
#define Dir_OutPut_HighBeam           DDRB_DDRB4=1
#define Dir_OutPut_LowBeam            DDRB_DDRB5=1
#define Dir_OutPut_WidthLight         DDRB_DDRB6=1
#define Dir_OutPut_HornCtrl           DDRB_DDRB7=1
#define Dir_OutPut_EEPROM_WP          DDRA_DDRA0=1
#define Dir_OutPut_AccPedalCtrl       DDRA_DDRA1=1
#define Dir_OutPut_BAT12VIN_EN        DDRJ_DDRJ1=1
#define Dir_OutPut_StrMotor_DirL      DDRK_DDRK2=1
#define Dir_OutPut_StrMotor_DirR      DDRK_DDRK3=1
#define Dir_OutPut_StrMotor_DrvEN     DDRK_DDRK5=1
#define Dir_OutPut_Reserved1          DDRP_DDRP5=1
#define Dir_OutPut_Reserved2          DDRP_DDRP7=1
#define Dir_OutPut_ModeLight          DDRH_DDRH7=1
#define Dir_OutPut_LED0               DDRH_DDRH4=1
#define Dir_OutPut_LED1               DDRH_DDRH5=1
//#define Dir_OutPut_GearShiftD         DDRH_DDRH7=1
//#define Dir_OutPut_GearShiftR         DDR1AD0_DDR1AD02=1




#define ElectVavle_Enable        ElectVavle=1
#define ElectVavle_DisEnable       ElectVavle=0

#define LeftTurnLight_Enable     LeftTurnLight=1
#define LeftTurnLight_DisEnable    LeftTurnLight=0

#define RightTurnLight_Enable      RightTurnLight=1
#define RightTurnLight_DisEnable   RightTurnLight=0

#define BrakeLight_Enable          BrakeLight=1
#define BrakeLight_DisEnable       BrakeLight=0

#define HighBeam_Enable            HighBeam=1
#define HighBeam_DisEnable         HighBeam=0

#define LowBeam_Enable             LowBeam=1
#define LowBeam_DisEnable          LowBeam=0

#define WidthLight_Enable          WidthLight=1
#define WidthLight_DisEnable       WidthLight=0

#define HornCtrl_Enable            HornCtrl=1
#define HornCtrl_DisEnable         HornCtrl=0

#define EEPROM_WP_Enable           EEPROM_WP=0
#define EEPROM_WP_DisEnable        EEPROM_WP=1

#define StrMotor_DrvEN_Enable      StrMotor_DrvEN=0
#define StrMotor_DrvEN_DisEnable   StrMotor_DrvEN=1

/*#define StrMotor_RelayEN_Enable      StrMotor_RelayEN=1
 #define StrMotor_RelayEN_Disable   StrMotor_RelayEN=0*/
#define Reserved1_Enable            Reserved1=1
#define Reserved1_DisEnable           Reserved1=0

#define Reserved2_Enable            Reserved2=1
#define Reserved2_DisEnable           Reserved2=0

#define ModeLight_Enable             ModeLight=0
#define ModeLight_DisEnable          ModeLight=1

#define AccPedalCtrl_Enable        AccPedalCtrl=1
#define AccPedalCtrl_DisEnable     AccPedalCtrl=0

#define BAT12VIN_EN_Enable         BAT12VIN_EN=1
#define BAT12VIN_EN_DisEnable      BAT12VIN_EN=0

#define StrMotor_DirL_Enable       StrMotor_DirL=1
#define StrMotor_DirL_DisEnable    StrMotor_DirL=0

#define StrMotor_DirR_Enable       StrMotor_DirR=1
#define StrMotor_DirR_DisEnable    StrMotor_DirR=0

/*#define GearShiftD_Enable          GearShiftD=1
 #define GearShiftD_Disable       GearShiftD=0
 
 #define GearShiftR_Enable          GearShiftR=1
 #define GearShiftR_Disable       GearShiftR=0 */

#define LED0_Enable                LED0=1
#define LED0_DisEnable            LED0=0

#define LED1_Enable                LED1=1
#define LED1_DisEnable             LED1=0




//输入端口
#define StrMotor_Error             PORTK_PK4

#define Dir_Input_StrMotor_Error   DDRK_DDRK4=0



void INIT_SCI0(void);
void INIT_PLL(void);
void INIT_IIC(void);
void INIT_AD(void);
//void INIT_ECT(void);
void INIT_PWM(void);
void INIT_MDC(void);
void INIT_GPIO(void);

