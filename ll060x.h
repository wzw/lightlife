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
#define		PWR                     0		//��Դ��
#define		LIGHT                   1		//�������Ȱ���
#define		WORKINGKEY              2		//����ģʽ��ϰ���
//brightness
#define		BRIGHTNESS_RED          0x02    //
#define		BRIGHTNESS_BLUE         0x20    //  0x20
#define		TOUCH_BRIGHTNESS_RED	0x7e    //�д���ʱ���������
#define     TOUCH_BRIGHTNESS_BLUE   0x7c
//#define     BRIGHTNESS_0            0x58    //�����ȵȼ�����
//#define     BRIGHTNESS_1            0x65
//#define     BRIGHTNESS_2            0x70
//#define     BRIGHTNESS_3            0x7f
#define		LONG_KEY                50      //������
//A/D channel
#define     BATT_TEMP               0x19
#define     LED_TEMP                0x15
#define     VOLTAGE                 0x1d
//temperature
#define		BATT_TEMP_LOW	0xb3	//����¶�
#define		BATT_TEMP_HIGH	0xc2    //43 degree
#define		BATT_TEMP_HIGH2	0xc7	//С�������ʱ�õ�����ֹ����¶ȼ�������
#define		LED_TEMP_LOW	0xaa	//LED �¶�0xae=36 degree 0xae
#define		LED_TEMP_HIGH	0xba    //40 degree 0xbe
//BATT USED
#define		BATT_EMPTY		0x52	//���ȼ���ص�ѹ
#define		BATT_0			0x51
#define		BATT_1			0x57
#define		BATT_2			0x5c
#define		BATT_3			0x61
#define		BATT_FULL		0x66
//other
#define     TOTAL_CHARGE_TIME   0xf0
#define     PRE_CHARGE_TIME     0x0f
#define     CHARGE_LOW          0x00    //С��Դ���
#define     CHARGE_HIGH         0x4c    //��������
#define		IDLE_DOWN           0x1		//��ʱ�ػ�ʱ��]
//����Flag�ĸ�λ����
#if 0
#define		MS_500		0		//500ms��־
#define		MIN_1		1		//1min��־
#define		PWR_OFF		2		//�ػ���־
#define		LAMP_ON		3		//�д���LED������־
#define		TEMP_HIGH	4		//�¶ȹ��߱�־
#define		CHARGE		5		//����־
#define		RESERVE		6		//����
#define		USED_FLAG	7		//����LEDʹ�ñ�־
#endif
struct Flag {
	UCHAR ms_500:1;				//500ms��־
	UCHAR min_1:1;				//1min��־
	UCHAR lamp_on:1;			//�д���LED������־ 1=yes, 0=no
	UCHAR temp_high:1;			//���±�־ 1=high, 0=low
	UCHAR temp_flash:1;			//�¶ȼƶ�ʱ��˸��־
	UCHAR pwr_off:1;			//�ػ���־
    UCHAR start_charge:1;       //��ʼ����־
    //UCHAR reserved:1;			//
    UCHAR start:1;              //����ָʾ
}bitFlag;
typedef struct {
	CHAR btn:1;					//������ť��־,1=touch, 0=none
	CHAR debug:1;				//�Ƿ���debugģʽ, 1=yes, 0=no
	CHAR used:1;				//ʹ�ñ�־, used=1, nouse=0
	CHAR workStatus:1;          //����״̬��1=����������0=�������
    CHAR post_charge:1;         //�����־
    CHAR DEBUG_SHOW:1;
    CHAR touch_off:1;           //���ڳ���ʹ��ʱ���������뿪�·���ʹ��
    UCHAR led:1;				//��ʾLED ��ͨ��, 0=BLUE, 1=RED
}oFlag;
oFlag FFlags;
//Const
//LCD, PORTA
#define		COM0		RA0		//LCD�˿�
#define		COM1		RA1
#define		COM2		RA2
#define		COM3		RA3
#define		ENVBAT		RA4		//��ѹ������
#define		LCD_VCC		RA5
#define		SEG_P		RA6
#define		COM_P		RA7
//PORTB I/O
#define		PWR_KEY		RB0		//��Դ����,0=����, 1=�޼�����
//#define		LIGHT_KEY	RB1		//���ȵ�����, 0=����, 1=�޼�����
#define		BACKLIGHT_POWER RB1	// Backlight power, 1=on, 0=off
//#define		PWR_VCC		RB3		//��Դ���翪��,0=�أ� 1=��
#define     CHG_PWM     RB3
#define		DC_IN		RB4		//����⣬0=�������Դ��1=��
//#define		LT_PWM		RB5		//pwm����
#define		ENVTEST		RB5			//�¶ȼ�����
//PORTC I/O	

#define     LT_PWM      RC7         //LT_PWM  RC2         

//#define     CHG_PWM     RC2     //CHARGE PWM
//#define		POWER_33	RC3		//3.3v��Դ����, 1=on, 0=off
//#define		BACKLIGHT	RC4		//����LED����,1=on, 0=off	
//#define		RX_TEST		RC7		//
#define		EN_BEEP		RC2         //WARNING BEEP
#define		TX_TEST		RC6		//	
//A/D control
//#define     AD_TEMP     RD6     //��أ�LED�¶�A/Dת������
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
bank1 unsigned int threshold_stop;  //cps�㷨ĩֵ��ֵ
bank1 unsigned int threshold_start; //cps�㷨������ֵ

bank1 unsigned int cps_cnt;         //cps�㷨, ���ڼ�����

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
unsigned char cnt;               //��ص�ѹ������   
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
UCHAR	t0tick;					//timer0�жϼ�����
CHAR	lcd_s;
CHAR 	lcd_temp;
UCHAR	lamp_pr;				//PWMռ�ձ�=lamp_pr
UCHAR   used_day;               //ʹ�ü����ʱ
CHAR    batt_n;                 //���״̬
UCHAR   key_mode;				//����ģʽѡ��
UCHAR   key_debug;				//debug mode
UCHAR   shortKey;               //
UCHAR   ledIndex;               //led index
UCHAR   key_press_num;          //���ڰ�����ʶ�����̼���ʶ����

unsigned int batvolt;           // AD����, ��ص�ѹ
unsigned int battemp;           // AD����������¶�
unsigned int ledtemp;           // AD��������ͷ�¶�

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
void shut_down_pro(void);		//�ػ�״̬����
void power_on_pro(void);        //����״̬����
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
// �汾�޸ļ�¼:
------------------------------------------------------------------------------------------
VERSION: V3.2  LAST VERSION: V3.1  DATE: 5/9/2012  ENDDATE:5/9/2012  AUTHOR: WKF
------------------------------------------------------------------------------------------

------------------------------------------------------------------------------------------
FILE: ll060x.h
--------------------------------------- ACTION: ADD --------------------------------------

unsigned int batvolt;           // AD����, ��ص�ѹ
unsigned int battemp;           // AD����������¶�
unsigned int ledtemp;           // AD��������ͷ�¶�

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


