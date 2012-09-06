/***************************************************************
//FileName: ll060x.h
//Date: 2011-11-06
//author:
//Version: v1.0
****************************************************************/
#ifndef LL060X_H
#define LL060X_H

typedef     unsigned char           UCHAR;
typedef     char                    CHAR;

#define     VERSION                 0x31    //Version

#define		ON			            1
#define		OFF			            0
#define		PWR                     0		//电源键
#define		LIGHT                   1		//调整亮度按键
#define		WORKINGKEY              2		//调试模式组合按键
//brightness
#define		BRIGHTNESS_RED          0x02    //
#define		BRIGHTNESS_BLUE         0x20    //  0x20
#define		TOUCH_BRIGHTNESS_RED	0x7e    //有触摸时的最高亮度
#define     TOUCH_BRIGHTNESS_BLUE   0x7c
//#define     BRIGHTNESS_0            0x58    //各亮度等级参数
//#define     BRIGHTNESS_1            0x65
//#define     BRIGHTNESS_2            0x70
//#define     BRIGHTNESS_3            0x7f
#define		LONG_KEY                50      //长按键
//A/D channel
#define     BATT_TEMP               0x19
#define     LED_TEMP                0x15
#define     VOLTAGE                 0x1d
//temperature
#define		BATT_TEMP_LOW	0xb3	//电池温度
#define		BATT_TEMP_HIGH	0xc2    //43 degree
#define		BATT_TEMP_HIGH2	0xc7	//小电流充电时用到，防止电池温度继续升高
#define		LED_TEMP_LOW	0xaa	//LED 温度0xae=36 degree 0xae
#define		LED_TEMP_HIGH	0xba    //40 degree 0xbe
//BATT USED
#define		BATT_EMPTY		0x52	//各等级电池电压
#define		BATT_0			0x51
#define		BATT_1			0x57
#define		BATT_2			0x5c
#define		BATT_3			0x61
#define		BATT_FULL		0x66
//other
#define     TOTAL_CHARGE_TIME   0xf0
#define     PRE_CHARGE_TIME     0x0f
#define     CHARGE_LOW          0x00    //小电源充电
#define     CHARGE_HIGH         0x4c    //大电流充电
#define		IDLE_DOWN           0x1		//定时关机时间]
//定义Flag的各位含义
#if 0
#define		MS_500		0		//500ms标志
#define		MIN_1		1		//1min标志
#define		PWR_OFF		2		//关机标志
#define		LAMP_ON		3		//有触摸LED点亮标志
#define		TEMP_HIGH	4		//温度过高标志
#define		CHARGE		5		//充电标志
#define		RESERVE		6		//保留
#define		USED_FLAG	7		//触摸LED使用标志
#endif
struct Flag {
	UCHAR ms_500:1;				//500ms标志
	UCHAR min_1:1;				//1min标志
	UCHAR lamp_on:1;			//有触摸LED点亮标志 1=yes, 0=no
	UCHAR temp_high:1;			//高温标志 1=high, 0=low
	UCHAR temp_flash:1;			//温度计定时闪烁标志
	UCHAR pwr_off:1;			//关机标志
    UCHAR start_charge:1;       //开始充电标志
    //UCHAR reserved:1;			//
    UCHAR start:1;              //开机指示
}bitFlag;
typedef struct {
	CHAR btn:1;					//触摸按钮标志,1=touch, 0=none
	CHAR debug:1;				//是否是debug模式, 1=yes, 0=no
	CHAR used:1;				//使用标志, used=1, nouse=0
	CHAR workStatus:1;          //工作状态：1=正常开机，0=进入调试
    CHAR post_charge:1;         //后充电标志
    CHAR DEBUG_SHOW:1;
    CHAR touch_off:1;           //用于持续使用时，触摸需离开下方可使用
    UCHAR led:1;				//表示LED 的通道, 0=BLUE, 1=RED
}oFlag;
oFlag FFlags;
//Const
//LCD, PORTA
#define		COM0		RA0		//LCD端口
#define		COM1		RA1
#define		COM2		RA2
#define		COM3		RA3
#define		ENVBAT		RA4		//电压检测控制
#define		LCD_VCC		RA5
#define		SEG_P		RA6
#define		COM_P		RA7
//PORTB I/O
#define		PWR_KEY		RB0		//电源按键,0=按下, 1=无键按下
//#define		LIGHT_KEY	RB1		//亮度调整键, 0=按下, 1=无键按下
#define		BACKLIGHT_POWER RB1	// Backlight power, 1=on, 0=off
//#define		PWR_VCC		RB3		//电源供电开关,0=关， 1=开
#define     CHG_PWM     RB3
#define		DC_IN		RB4		//充电检测，0=插入充电电源，1=无
//#define		LT_PWM		RB5		//pwm调制
#define		ENVTEST		RB5			//温度检测控制
//PORTC I/O	

#define     LT_PWM      RC7         //LT_PWM  RC2         

//#define     CHG_PWM     RC2     //CHARGE PWM
//#define		POWER_33	RC3		//3.3v电源开关, 1=on, 0=off
//#define		BACKLIGHT	RC4		//背光LED开关,1=on, 0=off	
//#define		RX_TEST		RC7		//
#define		EN_BEEP		RC2         //WARNING BEEP
#define		TX_TEST		RC6		//	
//A/D control
//#define     AD_TEMP     RD6     //电池，LED温度A/D转换控制
//#define     AD_VBAT     RD7
//LED Channel select
#define		LED1_EN		RD6     //RED
#define		LED2_EN		RD7     //BLUE
//Golobal variable
//LCD
CHAR	lcd_ram0;
CHAR	lcd_ram1;
CHAR	lcd_ram2;
CHAR	lcd_ram3;
CHAR    portData;
//mtouch variables
bank1 unsigned int average;         //[16] runing average for button
bank1 unsigned int average_n;       //[16] runing average for button
bank1 unsigned int threshold_stop;  //cps算法末值阈值
bank1 unsigned int threshold_start; //cps算法启动阈值

bank1 unsigned int cps_cnt;         //cps算法, 窗口计数器

bank1 unsigned int bigval;          //current button bigval -for averaging technique
bank1 unsigned int smallavg;        //current button smallavg - for averaging technique
unsigned char index;                // index value relates ea. button and scanning sequence
unsigned char ad;
//time
unsigned int iCount;
bank3 unsigned char t1_tick;
bank3 unsigned int off_time;
UCHAR sec;
UCHAR min;
UCHAR charge_count;
UCHAR on_touch;
unsigned char cnt;               //电池低压计数器   
//UCHAR hour;
//Charge
//char    min_cnt;                //
//UCHAR   max_volt;
//UCHAR   mid_volt;
//UCHAR   min_volt;
UCHAR   beep_5;
UCHAR   beep_en;                //beep enable ?, 0=no, 1=yes
UCHAR   beep_1;                 // beep time ctrl.
UCHAR   mode;                   //working mode, 0=auto, 1=manual
//
CHAR    sel;
UCHAR	t0tick;					//timer0中断计数器
CHAR	lcd_s;
CHAR 	lcd_temp;
UCHAR	lamp_pr;				//PWM占空比=lamp_pr
UCHAR   used_day;               //使用间隔计时
CHAR    batt_n;                 //电池状态
UCHAR   key_mode;				//按键模式选择
UCHAR   key_debug;				//debug mode
UCHAR   shortKey;               //
UCHAR   ledIndex;               //led index
UCHAR   key_press_num;          //用于按键标识、长短键标识计数

unsigned int batvolt;           // AD读数, 电池电压
unsigned int battemp;           // AD读数，电池温度
unsigned int ledtemp;           // AD读数，灯头温度

//Function:
//init function
void hw_init(void);
void cap_init(void);
void variable_init(void);
void power_on_hw_init(void);
void power_off_hw_init(void);
void start_charge_init(void);
void end_charge_init(void);
//LCD routine
void clr_lcd_ram(void);
void dis_lcd_all_seg(void);
void lcd_dig_high(CHAR chH);
void lcd_dig_low(CHAR chL);
void lcd_display_msg(UCHAR ch);
void lcd_display_batt(UCHAR batt);
void lcd_display_tmp(UCHAR flag);
void lcd_display_light(UCHAR level);
//time delay
#define DelayUs(x)	{ UCHAR dcnt; \
					dcnt = (x)/((12*1000L)/(8*1000L)) | 1; \
					while(--dcnt != 0) \
					continue; }
void DelayMs(UCHAR msec);
//other roution
CHAR hex_to_bcd(UCHAR hex);
void enter_working_loop(void);
void sensor_touch(void);
void sensor_Notouch(void);
void auto_shut_down(void);
void shut_down_pro(void);		//关机状态处理
void power_on_pro(void);        //开机状态处理
CHAR detect_touching(void);
UCHAR adc_read(UCHAR channel);
void man_shut_down(void);
void power_off_loop(void);
void enter_charge_loop(void);
void used_time_acc(void);
CHAR key_scan(CHAR key);
void wait_key_release(CHAR key);
void liMn_charge(void);
void init_charge_var(void);
void batt_test(void);
void show_debug_msg(void);
void change_led_channel(void);
void led_select(UCHAR channel);
void beep_on_off(UCHAR onOff);
void beep_pro(void);
void interrupt isr(void);
#endif

//END File
#if 0
// 版本修改记录:
------------------------------------------------------------------------------------------
VERSION: V3.2  LAST VERSION: V3.1  DATE: 5/9/2012  ENDDATE:5/9/2012  AUTHOR: WKF
------------------------------------------------------------------------------------------

------------------------------------------------------------------------------------------
FILE: ll060x.h
--------------------------------------- ACTION: ADD --------------------------------------

unsigned int batvolt;           // AD读数, 电池电压
unsigned int battemp;           // AD读数，电池温度
unsigned int ledtemp;           // AD读数，灯头温度

--------------------------------------- ACTION: DELETE -----------------------------------
--------------------------------------- ACTION: MODIFY -----------------------------------
------------------------------------------------------------------------------------------
FILE: LL060X.C
--------------------------------------- ACTION: ADD --------------------------------------
--------------------------------------- ACTION: DELETE -----------------------------------
--------------------------------------- ACTION: MODIFY -----------------------------------
------------------------------------------------------------------------------------------
END OF VERSION: V3.2  LAST VERSION: V3.1  DATE: 5/9/2012  ENDDATE:5/9/2012  AUTHOR: WKF
------------------------------------------------------------------------------------------
#endif


