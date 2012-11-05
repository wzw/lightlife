/***************************************************************
//FileName: ll060x.h
//Date: 2011-11-06
//author:
//Version: v1.0
****************************************************************/
#ifndef LL060X_H
#define LL060X_H

// Program config. word 1
__CONFIG(DEBUGDIS & PLLEN & BORV19 & BORDIS & UNPROTECT & MCLRDIS & PWRTEN & WDTDIS & INTIO & 0x3FFF); 
// Program config. word 2
__CONFIG(VCAPDIS);

// REGISTER 8-1: CONFIG1: CONFIGURATION WORD REGISTER 1
//  bit 13 DEBUG: In-Circuit Debugger Mode bit
//      1 = In-Circuit Debugger disabled, RB6/ICSPCLK and RB7/ICSPDAT are general purpose I/O pins
//      0 = In-Circuit Debugger enabled, RB6/ICSPCLK and RB7/ICSPDAT are dedicated to the debugger
//  bit 12 PLLEN: INTOSC PLL Enable bit
//      0 = INTOSC Frequency is 500 kHz
//      1 = INTOSC Frequency is 16 MHz (32x)
//  bit 11 Unimplemented: Read as
//  bit 10 BORV: Brown-out Reset Voltage selection bit
//      0 = Brown-out Reset Voltage (VBOR) set to 2.5 V nominal
//      1 = Brown-out Reset Voltage (VBOR) set to 1.9 V nominal
//  bit 9-8 BOREN<1:0>: Brown-out Reset Selection bits(1)
//      0x = BOR disabled (Preconditioned State)
//      10 = BOR enabled during operation and disabled in Sleep
//      11 = BOR enabled
//  bit 7 Unimplemented: Read as
//  bit 6 CP: Code Protection bit(2)
//      1 = Program memory code protection is disabled
//      0 = Program memory code protection is enabled
//  bit 5 MCLRE: RE3/MCLR pin function select bit(3)
//      1 = RE3/MCLR pin function is MCLR
//      0 = RE3/MCLR pin function is digital input, MCLR internally tied to VDD
//  bit 4 PWRTE: Power-up Timer Enable bit
//      1 = PWRT disabled
//      0 = PWRT enabled
//  bit 3 WDTE: Watchdog Timer Enable bit
//      1 = WDT enabled
//      0 = WDT disabled
//  Note 1: Enabling Brown-out Reset does not automatically enable Power-up Timer.
//       2: The entire program memory will be erased when the code protection is turned off.
//       3: When MCLR is asserted in INTOSC or RC mode, the internal clock oscillator is disabled.
//       4: MPLAB IDE masks unimplemented Configuration bits to.
//  bit 2-0 FOSC<2:0>: Oscillator Selection bits
//      111 = RC oscillator: CLKOUT function on RA6/OSC2/CLKOUT pin, RC on RA7/OSC1/CLKIN
//      110 = RCIO oscillator: I/O function on RA6/OSC2/CLKOUT pin, RC on RA7/OSC1/CLKIN
//      101 = INTOSC oscillator: CLKOUT function on RA6/OSC2/CLKOUT pin, I/O function on RA7/OSC1/CLKIN
//      100 = INTOSCIO oscillator: I/O function on RA6/OSC2/CLKOUT pin, I/O function on RA7/OSC1/CLKIN
//      011 = EC: I/O function on RA6/OSC2/CLKOUT pin, CLKIN on RA7/OSC1/CLKIN
//      010 = HS oscillator: High-speed crystal/resonator on RA6/OSC2/CLKOUT and RA7/OSC1/CLKIN
//      001 = XT oscillator: Crystal/resonator on RA6/OSC2/CLKOUT and RA7/OSC1/CLKIN
//      000 = LP oscillator: Low-power crystal on RA6/OSC2/CLKOUT and RA7/OSC1/CLKIN
// REGISTER 8-1: CONFIG1: CONFIGURATION WORD REGISTER 1 (CONTINUED)
//  Note 1: Enabling Brown-out Reset does not automatically enable Power-up Timer.
//       2: The entire program memory will be erased when the code protection is turned off.
//       3: When MCLR is asserted in INTOSC or RC mode, the internal clock oscillator is disabled.
//       4: MPLAB® IDE masks unimplemented Configuration bits to '0'.
//
// REGISTER 8-2: CONFIG2: CONFIGURATION WORD REGISTER 2
//  bit 13-6 Unimplemented: Read as
//  bit 5-4 VCAPEN<1:0>: Voltage Regulator Capacitor Enable bits
//  For the PIC16LF72X:
//      These bits are ignored. All VCAP pin functions are disabled.
//  For the PIC16F72X:
//      00 = VCAP functionality is enabled on RA0
//      01 = VCAP functionality is enabled on RA5
//      10 = VCAP functionality is enabled on RA6
//      11 = All VCAP functions are disabled (not recommended)
//  bit 3-0 Unimplemented: Read as
//  Note 1: MPLAB IDE masks unimplemented Configuration bits to '0'.
//
typedef     unsigned char           UCHAR;
typedef     char                    CHAR;

#define     VERSION_NUMBER          0x36
#ifdef __DEBUG
#define     VERSION                 VERSION_NUMBER-1
#warning PICC in DEBUG Mode.
#else
#define     VERSION                 VERSION_NUMBER
#endif

#define		ON			            1
#define		OFF			            0
#define		PWR                     0		//µçÔ´¼ü
#define		LIGHT                   1		//µ÷ÕûÁÁ¶È°´¼ü
#define		WORKINGKEY              2		//µ÷ÊÔÄ£Ê½×éºÏ°´¼ü
//brightness
#define		BRIGHTNESS_RED          14      //
#define		BRIGHTNESS_BLUE         14      // 
#define		TOUCH_BRIGHTNESS_RED	14      //ÓÐ´¥ÃþÊ±µÄ×î¸ßÁÁ¶È
#define     TOUCH_BRIGHTNESS_BLUE   14
//#define     BRIGHTNESS_0            0x58    //¸÷ÁÁ¶ÈµÈ¼¶²ÎÊý
//#define     BRIGHTNESS_1            0x65
//#define     BRIGHTNESS_2            0x70
//#define     BRIGHTNESS_3            0x7f
#define		POWER_ONOFF             2       //power on/off key got
#define		LONG_KEY                20      //enter debug mode key got
//other
#define     TAIL_CHARGE_TIME        0x20    // 32 minutes
#define     FULL_CHARGE_TIME        0x78    // 120 minutes
#define     PRE_CHARGE_TIME         0x20    // 32 minutes
#define     START_CHARGE_TIME       0x5     // 5 minutes
#define		IDLE_DOWN               0x2		// idle timeout value
#define     TMR0_VALUE              95      // Timer 0 should be 160 cycles interrupt, 16/2000000us*160=1.28ms
#define     POWEROFF_TMR1L          0x00    // TIMER 1 register when poweroff, start value = 0x6000( as 10 seconds overflow.)
#define     POWEROFF_TMR1H          0x60    // 40960/4096
#define     CHARGING_TMR1L          0xF0    // TIMER 1 register when charging, start value = 0xFFF0( as 3.9 microseconds overflow.)
#define     CHARGING_TMR1H          0xFF    // 16000/4096 ms(charge PWM period width. 
#define     PWM_WIDTH               15      // (PWM width should be 16 periods as 20.48ms) 16*1.28=20.48ms
#define     CHARGE_LOW              0       // small current charge, 1:16  PWM
#define     CHARGE_HIGH             11      // large current charge, 11:16 PWM
#define     CHARGE_AD               13      // start AD after CHARGE_HIGH and before next PWM cycle.
#define     T2_PR2                  124     // Timer2 overflow willbe 1ms(125*(1000/125)us)
#define     T2_125MS                124     // Timer2 count for 125ms
#define     T2_250MS                249     // Timer2 count for 250ms
#define     T2_500MS                499     // Timer2 count for 500ms
#define     T2_1S                   999     // Timer2 count for 1 second
#define     T1_500ms                127     // Timer1 count for 500ms
#define     OPTION_VALUE            0b10000011      // OPTION_REG: RBPU=dis,INTEDG=falling,T0CS=FOSC/4,T0SE=0,PSA=timer0,PS=16
                                            //bit 7 RBPU: PORTB Pull-up Enable bit
                                            //* 1 = PORTB pull-ups are disabled
                                            //  0 = PORTB pull-ups are enabled by individual port latch values
                                            //bit 6 INTEDG: Interrupt Edge Select bit
                                            //  1 = Interrupt on rising edge of INT pin
                                            //* 0 = Interrupt on falling edge of INT pin
                                            //bit 5 T0CS: TMR0 Clock Source Select bit
                                            //  1 = Transition on T0CKI pin or CPSOSC signal
                                            //* 0 = Internal instruction cycle clock (FOSC/4)(2MHz)
                                            //bit 4 T0SE: TMR0 Source Edge Select bit
                                            //  1 = Increment on high-to-low transition on T0CKI pin
                                            //* 0 = Increment on low-to-high transition on T0CKI pin
                                            //bit 3 PSA: Prescaler Assignment bit
                                            //  1 = Prescaler is assigned to the WDT
                                            //* 0 = Prescaler is assigned to the Timer0 module
                                            //bit 2-0 PS<2:0>: Prescaler Rate Select bits
                                            //  BIT VALUE    TMR0 RATE       WDT RATE
                                            //    000         1 : 2           1 : 1
                                            //    001         1 : 4           1 : 2
                                            //    010         1 : 8           1 : 4
                                            //*   011         1 : 16          1 : 8
                                            //    100         1 : 32          1 : 16
                                            //    101         1 : 64          1 : 32
                                            //    110         1 : 128         1 : 64
                                            //    111         1 : 256         1 : 128
#define     ADCON0_POWEROFF         0b00000000
#define     ADCON0_POWERON          0b00000000
                                            // ADCON0: A/D CONTROL REGISTER 0
                                            //bit 7-6 Unimplemented: Read as ¡®0¡¯
                                            //bit 5-2 CHS<3:0>: Analog Channel Select bits
                                            //  0000 = AN0
                                            //  0001 = AN1
                                            //  0010 = AN2
                                            //  0011 = AN3
                                            //  0100 = AN4
                                            //  0101 = AN5
                                            //  0110 = AN6
                                            //  0111 = AN7
                                            //  1000 = AN8
                                            //  1001 = AN9
                                            //  1010 = AN10
                                            //  1011 = AN11
                                            //  1100 = AN12
                                            //  1101 = AN13
                                            //  1110 = Reserved
                                            //  1111 = Fixed Voltage Reference (FVREF)
                                            //bit 1 GO/DONE: A/D Conversion Status bit
                                            //  1 = A/D conversion cycle in progress. Setting this bit starts an A/D conversion cycle.
                                            //  This bit is automatically cleared by hardware when the A/D conversion has completed.
                                            //  0 = A/D conversion completed/not in progress
                                            //bit 0 ADON: ADC Enable bit
                                            //  1 = ADC is enabled
                                            //  0 = ADC is disabled and consumes no operating current
//A/D channel
#define     BATT_TEMP               0b00011000      //AN6
#define     LED_TEMP                0b00010100      //AN5
#define     VOLTAGE                 0b00011100      //AN7
//temperature
#define		BATT_TEMP_LOW           0xb3	//45 Celsius degree
#define		BATT_TEMP_HIGH          0xc7    //55 Celsius degree
#define		LED_TEMP_LOW            0xa9	//40 Celsius degree
#define		LED_TEMP_HIGH           0xbd    //50 Celsius degree
//BATT USED
#define		BATT_EMPTY		        0x47	//2.9V
#define		BATT_0			        0x56    //3.5V
#define		BATT_1			        0x5B    //3.7V
#define		BATT_2                  0x5E    //3.8V
#define		BATT_3                  0x60    //3.9V
#define		BATT_FULL               0x68    //4.2V

#define     ADCON1_VALUE            0b01000000
                                            // ADCON1: A/D CONTROL REGISTER 1
                                            //bit 7 Unimplemented: Read as ¡®0¡¯
                                            //bit 6-4 ADCS<2:0>: A/D Conversion Clock Select bits
                                            //  000 = FOSC/2
                                            //  001 = FOSC/8
                                            //  010 = FOSC/32
                                            //  011 = FRC (clock supplied from a dedicated RC oscillator)
                                            //* 100 = FOSC/4
                                            //  101 = FOSC/16
                                            //  110 = FOSC/64
                                            //  111 = FRC (clock supplied from a dedicated RC oscillator)
                                            //bit 3-2 Unimplemented: Read as ¡®0¡¯
                                            //bit 1-0 ADREF<1:0>: Voltage Reference Configuration bits
                                            //* 0x = VREF is connected to VDD
                                            //  10 = VREF is connected to external VREF (RA3/AN3)
                                            //  11 = VREF is connected to internal Fixed Voltage Reference
#define     FVRCON_VALUE            0b00000000
                                            //FVRCON: FIXED VOLTAGE REFERENCE REGISTER
                                            //bit 7 FVRRDY(1): Fixed Voltage Reference Ready Flag bit
                                            //    0 = Fixed Voltage Reference output is not active or stable
                                            //    1 = Fixed Voltage Reference output is ready for use
                                            //bit 6 FVREN(2): Fixed Voltage Reference Enable bit
                                            //*   0 = Fixed Voltage Reference is disabled
                                            //    1 = Fixed Voltage Reference is enabled
                                            //bit 5-2 Unimplemented: Read as ¡®0¡¯
                                            //bit 1-0 ADFVR<1:0>: A/D Converter Fixed Voltage Reference Selection bits
                                            //*   00 = A/D Converter Fixed Voltage Reference Peripheral output is off.
                                            //    01 = A/D Converter Fixed Voltage Reference Peripheral output is 1x (1.024V)
                                            //    10 = A/D Converter Fixed Voltage Reference Peripheral output is 2x (2.048V)(2)
                                            //    11 = A/D Converter Fixed Voltage Reference Peripheral output is 4x (4.096V)(2)
                                            //Note 1: FVRRDY is always ¡®1¡¯ for the PIC16F72X devices.
                                            //     2: Fixed Voltage Reference output cannot exceed VDD.

struct Flag {
	UCHAR shortKey:1;           //bit0 short key press flag, < 1.5s
	UCHAR toggle:1;             //bit1 timer toggle flag
	UCHAR lamp_on:1;            //bit2 ÓÐ´¥ÃþLEDµãÁÁ±êÖ¾ 1=yes, 0=no
	UCHAR temp_high:1;          //bit3 ¸ßÎÂ±êÖ¾ 1=high, 0=low
	UCHAR toggle_500ms:1;       //bit4 sync 500ms timer toggle flag
	UCHAR pwr_off:1;            //bit5 ¹Ø»ú±êÖ¾
    UCHAR start_charge:1;       //bit6 ¿ªÊ¼³äµç±êÖ¾
    UCHAR start:1;              //bit7 ¿ª»úÖ¸Ê¾
}bitFlag;

struct n_Flag {
	UCHAR auto_mode:1;          //bit0 auto mode
	UCHAR touchoff_waiting:1;   //bit1
	UCHAR bit2:1;               //bit2 
	UCHAR bit3:1;               //bit3 
	UCHAR bit4:1;               //bit4 
	UCHAR bit5:1;               //bit5 
    UCHAR bit6:1;               //bit6 
    UCHAR bit7:1;               //bit7 
}nFlag;

typedef struct {
	UCHAR btn:1;                //bit0,´¥Ãþ°´Å¥±êÖ¾,1=touch, 0=none
	UCHAR debug:1;              //bit1,ÊÇ·ñÊÇdebugÄ£Ê½, 1=yes, 0=no
	UCHAR used:1;               //bit2 Ê¹ÓÃ±êÖ¾, used=1, nouse=0
	UCHAR preChargetimeout:1;   //bit3 1 when PRE_CHARGE_TIME exceed in precharging
    UCHAR FULLChargetimeout:1;  //bit4 1 when FULL_CHARGE_TIME exceed in FULLcharging
    UCHAR beeping:1;            //bit5 enable beep
    UCHAR mtouch_toggle:1;      //bit6 new mtouch counter got
    UCHAR led:1;                //bit7 ±íÊ¾LED µÄÍ¨µÀ, 0=BLUE, 1=RED
}oFlag;
oFlag FFlags;

//Const
//LCD, PORTA
#define		COM0		RA0         //LCD¶Ë¿Ú
#define		COM1		RA1
#define		COM2		RA2
#define		COM3		RA3
#define		ENVBAT		RA4         //µçÑ¹¼ì²â¿ØÖÆ
#define		LCD_VCC		RA5
#define		SEG_P		RA6
#define		COM_P		RA7
//PORTB I/O
#define		PWR_KEY		RB0         //µçÔ´°´¼ü,0=°´ÏÂ, 1=ÎÞ¼ü°´ÏÂ
#define		BACKLIGHT_POWER RB1     // Backlight power, 1=on, 0=off
#define     CHG_PWM     RB3
#define		DC_IN		RB4         //³äµç¼ì²â£¬0=²åÈë³äµçµçÔ´£¬1=ÎÞ
#define		ENVTEST		RB5         //ÎÂ¶È¼ì²â¿ØÖÆ
#define     DEBUG1      RB6
#define     DEBUG2      RB7
//PORTC I/O	
#define		EN_BEEP		RC2         //WARNING BEEP
#define		TX_TEST		RC6         //	
#define     LT_PWM      RC7         //LT_PWM  RC2         
//PORTD I/O	
//LED Channel select
#define		LED1_EN		RD6         //RED
#define		LED2_EN		RD7         //BLUE
//PORTE I/O	
#define     LEDTEMP     RE0         //LED TEMPRETURE AD PORT
#define     BATTEMP     RE1         //BATTERY TEMPRETURE AD PORT
#define     BATVOLT     RE2         //BAT VOLTAGE AD PORT
//Golobal variable
//LCD
CHAR	lcd_ram0;
CHAR	lcd_ram1;
CHAR	lcd_ram2;
CHAR	lcd_ram3;
CHAR    portData;
//mtouch variables
UCHAR   cps_tmr1h;
UCHAR   cps_tmr1l;
bank1 unsigned int average;         //[16] runing average for button
bank1 unsigned int average_n;       //[16] runing average for button
bank1 unsigned int threshold_stop;  //cpsËã·¨Ä©ÖµãÐÖµ
bank1 unsigned int threshold_start; //cpsËã·¨Æô¶¯ãÐÖµ

bank1 unsigned int cps_cnt;         //cpsËã·¨, ´°¿Ú¼ÆÊýÆ÷

bank1 unsigned int bigval;          //current button bigval -for averaging technique
bank1 unsigned int smallavg;        //current button smallavg - for averaging technique
unsigned char ad;
//time
unsigned int iCount;
unsigned int msCount;
unsigned int beep_count;            // beep time counter
bank3 unsigned char t10_tick;       // 10s timer counter
bank3 unsigned int off_time;
UCHAR sec;
UCHAR min;
UCHAR charge_count;                 // 500ms charging timer count
UCHAR on_touch;
unsigned char cnt;                  // count when battery low
UCHAR debug_cnt;
UCHAR message;
UCHAR adresult;
//Charge
//char    min_cnt;                //
//UCHAR   max_volt;
//UCHAR   mid_volt;
//UCHAR   min_volt;
//
CHAR    sel;
UCHAR	t0tick;					//timer0ÖÐ¶Ï¼ÆÊýÆ÷
CHAR	lcd_s;
CHAR 	lcd_temp;
UCHAR	lamp_pr;				//PWMÕ¼¿Õ±È=lamp_pr
UCHAR   used_day;               //Ê¹ÓÃ¼ä¸ô¼ÆÊ±
CHAR    batt_n;                 //µç³Ø×´Ì¬
UCHAR   key_mode;				//°´¼üÄ£Ê½Ñ¡Ôñ
UCHAR   key_press_num;          //ÓÃÓÚ°´¼ü±êÊ¶¡¢³¤¶Ì¼ü±êÊ¶¼ÆÊý

char    chargephase;            // 0 pre_charging; 1 full current charging; 2 full voltage charge;
unsigned int batvolt;           // AD out of Battery Voltage
unsigned int battemp;           // AD out of Battery Tempreture
unsigned int ledtemp;           // AD out of LED BOARD Tempreture

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
void shut_down_pro(void);		//¹Ø»ú×´Ì¬´¦Àí
void power_on_pro(void);        //¿ª»ú×´Ì¬´¦Àí
CHAR detect_touching(void);
UCHAR adc_read(UCHAR channel);
void man_shut_down(void);
void enable_ADC(void);
void disable_ADC(void);
void power_off_loop(void);
void enter_charge_loop(void);
void used_time_acc(void);
CHAR key_scan(void);
void init_charge_var(void);
void check_environment(void);
void show_debug_msg(void);
void light_led_on(void);
void led_select(void);
void on_touch_check(void);
void interrupt isr(void);
#endif

//END File
