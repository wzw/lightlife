/*****************************************************************
//FileName: ll060x.c
//Date: 2011-12-06
//MCU TYPE: PIC16F727
//Version: v1.0

//Copyright:
******************************************************************/
#include <stdio.h>
#include <pic.h>
#include "ll060x.h"

// digtab,0123456789abcdef
unsigned char digtab[16] = { 0xdd, 0x14, 0x79, 0x75, 0xb4, 0xe5, 0xed, 0x54, 
							 0xfd, 0xf5, 0xfc, 0xad, 0x29, 0x3d, 0xe9, 0xe8 };
//---------------------------------------------------------------
//Function: hw_init()
//init hardware setting
//---------------------------------------------------------------
void hw_init(void)
{
	OSCCON = 0x20;			//7:6 00 = Unimplemented: Read as ‘0’
	                        //5:4 10 = 8 MHz (POR value)
	                        //3    0 = 16 MHz/500 kHz Internal Oscillator (HFIOSC) has not yet locked.
	                        //2    0 = 16 MHz/500 kHz Internal Oscillator (HFIOSC) has not yet reached its maximum accuracy
	                        //1:0 00 = Unimplemented: Read as ‘0’

	OPTION = OPTION_VALUE;  // OPTION_REG: RBPU=dis,INTEDG=falling,T0CS=FOSC/4,T0SE=0,PSA=timer0,PS=16

	TRISA = 0b00001111;		//设置I/O口的方向，1=input, 0=output
	TRISB = 0b00010101;
	TRISC = 0b00000010;
	TRISD = 0b11111111;
	TRISE = 0b00000111;

	ANSELA = 0x00;			//set Digital I/O, 0=digital, 1= analog input
	ANSELD = 0x00;
	ANSELE = 0b00000111;	//设置PORTE,0,1,2为模拟I/O，其它为数字I/O

	ANSELB = 0b00000100;	//CPS2 select, TOUCH

	WPUB = 0x00;			//禁止PORTB内部弱上拉,disable portB weak pull_up
	PORTA = 0x00;			//clear port
	PORTB = 0x00;
	PORTC = 0x00;

	APFCON = 0b00000001;	//CCP2 function is on RB3/CCP2

	OSCTUNE = 0b01111111;   //oscillator tuning ,maximum frequency
	INTCON = 0x00;      	//disable interrupts ,GIE--PEIE--T0IE--INTE--RBIE--T0IF--INTF--RBIF(bit7---bit0)

    CHG_PWM = OFF;
#ifdef __DEBUG
    DEBUG1 = 0;
    DEBUG2 = 0;
#endif
    return;
}
//---------------------------------------------------------------
//Function: cap_init()
//Initialize the Capacitive Sense Module and Time Base Modules
//---------------------------------------------------------------
void cap_init(void)
{
	average = 0;
	average_n = 0;
    cps_cnt = 0;
    
	T1CON = 0b11000101;		//T1CON: TIMER1 CONTROL REGISTER
                            //bit 7-6 TMR1CS<1:0>: Timer1 Clock Source Select bits
                            //* 11 =Timer1 clock source is Capacitive Sensing Oscillator (CAPOSC)
                            //  10 =Timer1 clock source is pin or oscillator:
                            //      If T1OSCEN = 0:
                            //          External clock from T1CKI pin (on the rising edge)
                            //      If T1OSCEN = 1:
                            //          Crystal oscillator on T1OSI/T1OSO pins
                            //  01 =Timer1 clock source is system clock (FOSC)
                            //  00 =Timer1 clock source is instruction clock (FOSC/4)
                            //bit 5-4 T1CKPS<1:0>: Timer1 Input Clock Prescale Select bits
                            //  11 = 1:8 Prescale value
                            //  10 = 1:4 Prescale value
                            //  01 = 1:2 Prescale value
                            //* 00 = 1:1 Prescale value
                            //bit 3 T1OSCEN: LP Oscillator Enable Control bit
                            //   1 = Dedicated Timer1 oscillator circuit enabled
                            //*  0 = Dedicated Timer1 oscillator circuit disabled
                            //bit 2  T1SYNC: Timer1 External Clock Input Synchronization Control bit
                            //      TMR1CS<1:0> = 1X
                            //*         1 = Do not synchronize external clock input
                            //          0 = Synchronize external clock input with system clock (FOSC)
                            //      TMR1CS<1:0> = 0X
                            //          This bit is ignored. Timer1 uses the internal clock when TMR1CS<1:0> = 1X.
                            //bit 1 Unimplemented: Read as ‘0’
                            //bit 0  TMR1ON: Timer1 On bit
                            //*     1 = Enables Timer1
                            //      0 = Stops Timer1
                            //      Clears Timer1 Gate flip-flop
                            
    T1GCON = 0b11100010;    //T1GCON: TIMER1 GATE CONTROL REGISTER
                            //bit 7 TMR1GE: Timer1 Gate Enable bit
                            //  If TMR1ON = 0:
                            //      This bit is ignored
                            //  If TMR1ON = 1:
                            //*     1 = Timer1 counting is controlled by the Timer1 gate function
                            //      0 = Timer1 counts regardless of Timer1 gate function
                            //bit 6 T1GPOL: Timer1 Gate Polarity bit
                            //* 1 = Timer1 gate is active-high (Timer1 counts when gate is high)
                            //  0 = Timer1 gate is active-low (Timer1 counts when gate is low)
                            //bit 5 T1GTM: Timer1 Gate Toggle Mode bit
                            //* 1 = Timer1 Gate Toggle mode is enabled.
                            //  0 = Timer1 Gate Toggle mode is disabled and toggle flip flop is cleared
                            //      Timer1 gate flip flop toggles on every rising edge.
                            //bit 4 T1GSPM: Timer1 Gate Single Pulse Mode bit
                            //  1 = Timer1 gate Single-Pulse mode is enabled and is controlling Timer1 gate
                            //* 0 = Timer1 gate Single-Pulse mode is disabled
                            //bit 3 T1GGO/DONE: Timer1 Gate Single-Pulse Acquisition Status bit
                            //  1 = Timer1 gate single-pulse acquisition is ready, waiting for an edge
                            //* 0 = Timer1 gate single-pulse acquisition has completed or has not been started
                            //  This bit is automatically cleared when T1GSPM is cleared.
                            //bit 2 T1GVAL: Timer1 Gate Current State bit
                            //  Indicates the current state of the Timer1 gate that could be provided to TMR1H:TMR1L.
                            //  Unaffected by Timer1 Gate Enable (TMR1GE).
                            //bit 1-0 T1GSS<1:0>: Timer1 Gate Source Select bits
                            //  00 = Timer1 Gate pin
                            //  01 = Timer0 Overflow output
                            //* 10 = TMR2 Match PR2 output
                            //  11 = Watchdog Timer scaler overflow
                            //  Watchdog Timer oscillator is turned on if TMR1GE = 1, regardless of the state of TMR1ON
    TMR2IF = 0;
	TMR2IE = 1;	
    PR2 = T2_PR2;           // Timer2 overflow willbe 2ms(250*(1000/125)us)
    T2CON = 0b00000110;		//T2CON: TIMER2 CONTROL REGISTER
                            //bit 7 Unimplemented: Read as ‘0’
                            //bit 6-3 TOUTPS<3:0>: Timer2 Output Postscaler Select bits
                            //* 0000 = 1:1 Postscaler
                            //  0001 = 1:2 Postscaler
                            //  0010 = 1:3 Postscaler
                            //  0011 = 1:4 Postscaler
                            //  0100 = 1:5 Postscaler
                            //  0101 = 1:6 Postscaler
                            //  0110 = 1:7 Postscaler
                            //  0111 = 1:8 Postscaler
                            //  1000 = 1:9 Postscaler
                            //  1001 = 1:10 Postscaler
                            //  1010 = 1:11 Postscaler
                            //  1011 = 1:12 Postscaler
                            //  1100 = 1:13 Postscaler
                            //  1101 = 1:14 Postscaler
                            //  1110 = 1:15 Postscaler
                            //  1111 = 1:16 Postscaler
                            //bit 2 TMR2ON: Timer2 On bit
                            //* 1 = Timer2 is on
                            //  0 = Timer2 is off
                            //bit 1-0 T2CKPS<1:0>: Timer2 Clock Prescale Select bits
                            //  00 = Prescaler is 1
                            //  01 = Prescaler is 4
                            //* 1x = Prescaler is 16 (125KHz)

	CPSCON0 = 0b10001100;   //CPSCON0: CAPACITIVE SENSING CONTROL REGISTER 0
                            //bit 7 CPSON: Capacitive Sensing Module Enable bit
                            //* 1 = Capacitive sensing module is operating
                            //  0 = Capacitive sensing module is shut off and consumes no operating current
                            //bit 6-4 Unimplemented: Read as ‘0’
                            //bit 3-2 CPSRNG<1:0>: Capacitive Sensing Oscillator Range bits
                            //  00 = Oscillator is off.
                            //  01 = Oscillator is in low range. Charge/discharge current is nominally 0.1 μA.
                            //  10 = Oscillator is in medium range. Charge/discharge current is nominally 1.2 μA.
                            //* 11 = Oscillator is in high range. Charge/discharge current is nominally 18 μA.
                            //bit 1 CPSOUT: Capacitive Sensing Oscillator Status bit
                            //  1 = Oscillator is sourcing current (Current flowing out the pin)
                            //* 0 = Oscillator is sinking current (Current flowing into the pin)
                            //bit 0 T0XCS: Timer0 External Clock Source Select bit
                            //  If T0CS = 1
                            //  The T0XCS bit controls which clock external to the core/Timer0 module supplies Timer0:
                            //  1 = Timer0 Clock Source is the capacitive sensing oscillator
                            //  0 = Timer0 Clock Source is the T0CKI pin
                            //  If T0CS = 0
                            //* Timer0 clock source is controlled by the core/Timer0 module and is FOSC/4.

    CPSCON1 = 0b0010;       //CPSCON1: CAPACITIVE SENSING CONTROL REGISTER 1
                            //bit 7-4 Unimplemented: Read as'0'
                            //bit 3-0 CPSCH<3:0>: Capacitive Sensing Channel Select bits
                            //  If CPSON = 0:
                            //      These bits are ignored. No channel is selected.
                            //  If CPSON = 1:
                            //      0000 = channel 0, (CPS0)
                            //      0001 = channel 1, (CPS1)
                            //*     0010 = channel 2, (CPS2)
                            //      0011 = channel 3, (CPS3)
                            //      0100 = channel 4, (CPS4)
                            //      0101 = channel 5, (CPS5)
                            //      0110 = channel 6, (CPS6)
                            //      0111 = channel 7, (CPS7)
                            //      1000 = channel 8, (CPS8(1))
                            //      1001 = channel 9, (CPS9(1))
                            //      1010 = channel 10, (CPS10(1))
                            //      1011 = channel 11, (CPS11(1))
                            //      1100 = channel 12, (CPS12(1))
                            //      1101 = channel 13, (CPS13(1))
                            //      1110 = channel 14, (CPS14(1))
                            //      1111 = channel 15, (CPS15(1))
                            // Note 1: These channels(8-15) are not implemented on the PIC16F722/723/726/PIC16LF722/723/726.
                            //      2: This bit(bit 3) is not implemented on PIC16F722/723/726/PIC16LF722/723/726, Read as'0'

	TMR1GIF = 0;            //Enable gate interrupt
	TMR1GIE = 1;
                            //关闭振荡电路，防止开机时LED闪烁
    lamp_pr = 0;            //关闭振荡电路
    DelayMs(2);

    LED1_EN = OFF;          //blue led disabled
    LED2_EN = OFF;          //red led disabled

    FFlags.led = 0;         //FFlags.led == 1? YES = BLUE, NO = RED
#ifdef __DEBUG
    DEBUG1 = 0;
    DEBUG2 = 0;
#endif
    return;
}
//---------------------------------------------------------------
//Function: variable_init()
//init global variable & flag
//---------------------------------------------------------------
void variable_init(void)
{
    iCount = 0;
    used_day = 0;               //使用间隔为0
    off_time = 0;
    t10_tick = 0;
    FFlags.used = 0;            //清使用标志
    FFlags.debug = 0;           //debug mode? 0=no, 1=yes
    key_mode = 0;
    bitFlag.start = 0;
    bitFlag.shortKey = 0;
    cnt = 0;
    nFlag.auto_mode = 0;         // set auto mode
    nFlag.touchoff_waiting = 0;
    return;
}
//---------------------------------------------------------------
//Function: power_on_hw_init()
//
//---------------------------------------------------------------
void power_on_hw_init(void)
{
    TMR1IE = 0;                 //disable timer1 interrupt
    T1SYNC = 0;
	LCD_VCC = ON;

	OPTION = OPTION_VALUE;      // OPTION_REG: RBPU=dis,INTEDG=falling,T0CS=FOSC/4,T0SE=0,PSA=timer0,PS=16

	TMR0 = TMR0_VALUE;			//Timer 0 should be 1ms interrupt
                                // Timer0 clk source is FOSC/4(2MHz), RATE = 1MHz, TMR0=95, 160 cycles should be count
                                // Timer0 = 6250Hz, 160 us interrupt

	t0tick = 0;
	enable_ADC();               //Enable ADC

	T0IE = 1;                   //Enable timer0 interrupt
    INTE = 0;                   //disable RB0/INT interrupt
    GIE = 1;                    //enable global interrupt
    return;
}
//---------------------------------------------------------------
// Function: enable_ADC()
// 
//---------------------------------------------------------------
void enable_ADC(void)
{
    ENVBAT = ON;
	ENVTEST = ON;

    FVRCON = FVRCON_VALUE;      // FVREN, A/D Converter Fixed Voltage Reference Peripheral output is 2x (2.048V)(2)
    ADCON1 = ADCON1_VALUE;      // ADCON1: a/d clock=fosc/4, VREF = VREF is connected to internal Fixed Voltage Reference
    return;
}
//---------------------------------------------------------------
// Function: disable_ADC()
// 
//---------------------------------------------------------------
void disable_ADC(void)
{
	ENVBAT = OFF;
	ENVTEST = OFF;
    FVRCON = 0;
    ADCON1 = ADCON1_VALUE;      // ADCON1: a/d clock=fosc/4, VREF = VREF is connected to internal Fixed Voltage Reference
    ADCON0 = ADCON0_POWEROFF;
    return;
}
//---------------------------------------------------------------
//Function: power_off_hw_init()
//
//---------------------------------------------------------------
void power_off_hw_init(void)
{
    LT_PWM = 0;                 // LED off
    GIE = 0;                    // 0 = Disables all interrupts
    T0IE = 0;                   // 0 = Disables the Timer0 interrupt
    INTE = 1;                   // 1 = Enables the RB0/INT external interrupt for exit sleep
    
    TRISA = 0x00;               //
    TRISD = 0x00;

    PORTA = 0x00;
    PORTD = 0x00;
    shut_down_pro();
    TMR2ON = 0;                 //timer2 off
    TMR2IE = 0;
    TMR1GIE = 0;                //DISABLE gate interrupt
    T1GCON = 0x00;              //Timer1 Gate Disable
    CPSCON0 = 0x00;             //disable CPS module
    CPSCON1 = 0b0010;           //CPSCON1: select channel 2
    
    TMR1ON = 0;
    TMR1H = POWEROFF_TMR1H;     //TIMER 1 register when poweroff, start value = 0x6000( as 10 seconds overflow.)
    TMR1L = POWEROFF_TMR1L;
    T1CON = 0b10111100;         //T1CON: TIMER1 CONTROL REGISTER
                                //bit 7-6 TMR1CS<1:0>: Timer1 Clock Source Select bits
                                //  11 =Timer1 clock source is Capacitive Sensing Oscillator (CAPOSC)
                                //* 10 =Timer1 clock source is pin or oscillator:
                                //      If T1OSCEN = 0:
                                //          External clock from T1CKI pin (on the rising edge)
                                //      If T1OSCEN = 1:
                                //*         Crystal oscillator on T1OSI/T1OSO pins
                                //  01 =Timer1 clock source is system clock (FOSC)
                                //  00 =Timer1 clock source is instruction clock (FOSC/4)
                                //bit 5-4 T1CKPS<1:0>: Timer1 Input Clock Prescale Select bits
                                //* 11 = 1:8 Prescale value
                                //  10 = 1:4 Prescale value
                                //  01 = 1:2 Prescale value
                                //  00 = 1:1 Prescale value
                                //bit 3 T1OSCEN: LP Oscillator Enable Control bit
                                //*  1 = Dedicated Timer1 oscillator circuit enabled
                                //   0 = Dedicated Timer1 oscillator circuit disabled
                                //bit 2  T1SYNC: Timer1 External Clock Input Synchronization Control bit
                                //      TMR1CS<1:0> = 1X
                                //*         1 = Do not synchronize external clock input
                                //          0 = Synchronize external clock input with system clock (FOSC)
                                //      TMR1CS<1:0> = 0X
                                //          This bit is ignored. Timer1 uses the internal clock when TMR1CS<1:0> = 1X.
                                //bit 1 Unimplemented: Read as ‘0’
                                //bit 0  TMR1ON: Timer1 On bit
                                //      1 = Enables Timer1
                                //      0 = Stops Timer1
                                //      Clears Timer1 Gate flip-flop
	 
	disable_ADC();              //disalbe ADC,

    T1SYNC = 1;                 //Do not synchronize external clock input
    TMR1ON = 1;
	T0IF = 0;                   //clear T0IF
	RBIF = 0;
    INTF = 0;
    PIR1 = 0;
    PIR2 = 0;
    PIE1 = 0;
    PIE2 = 0;
    TMR1IF = 0; 
    PEIE = 1;                   //Enables all unmasked peripheral interrupts
    TMR1IE = 1;
    FFlags.debug = 0;
	return;
}
//---------------------------------------------------------------
//Function: clr_lcd_ram()
//clear lcd ram, init
//---------------------------------------------------------------
void clr_lcd_ram(void)
{
	lcd_ram0 = 0x00;
	lcd_ram1 = 0x00;
	lcd_ram2 = 0x00;
	lcd_ram3 = 0x00;
	return;
}
//---------------------------------------------------------------
//Function: dis_lcd_all_seg()
//点亮所有字段初始化
//---------------------------------------------------------------
void dis_lcd_all_seg(void)
{
	lcd_ram0 = 0xff;
	lcd_ram1 = 0xff;
	lcd_ram2 = 0xff;
	lcd_ram3 = 0xff;
	return;
}
//---------------------------------------------------------------
//Function: lcd_dig_high(CHAR chH)
//input: chH
//return: none
//显示一个十六进制值高字节
//---------------------------------------------------------------
void lcd_dig_high(CHAR chH)
{
	lcd_ram0 &= 0b11111001;
	lcd_ram1 &= 0b11111001;
	lcd_ram2 &= 0b11111001;
	lcd_ram3 &= 0b11111011;	

	if( digtab[chH] & 0x80 ) lcd_ram0 |= 0b00000010;
	if( digtab[chH] & 0x40 ) lcd_ram0 |= 0b00000100;

	if( digtab[chH] & 0x20 ) lcd_ram1 |= 0b00000010;
	if( digtab[chH] & 0x10 ) lcd_ram1 |= 0b00000100;

	if( digtab[chH] & 0x08 ) lcd_ram2 |= 0b00000010;
	if( digtab[chH] & 0x04 ) lcd_ram2 |= 0b00000100;

	if( digtab[chH] & 0x01 ) lcd_ram3 |= 0b00000100;	
	return;
}
//---------------------------------------------------------------
//Function: lcd_dig_low(CHAR chL)
//input: chL
//return: none
//显示一个十六进制值低字节
//---------------------------------------------------------------
void lcd_dig_low(CHAR chL)
{
	lcd_ram0 &= 0b01100111;
	lcd_ram1 &= 0b01100111;
	lcd_ram2 &= 0b01100111;
	lcd_ram3 &= 0b01101111;

	if( digtab[chL] & 0x80 ) lcd_ram0 |= 0b00001000;
	if( digtab[chL] & 0x40 ) lcd_ram0 |= 0b00010000;

	if( digtab[chL] & 0x20 ) lcd_ram1 |= 0b00001000;
	if( digtab[chL] & 0x10 ) lcd_ram1 |= 0b00010000;

	if( digtab[chL] & 0x08 ) lcd_ram2 |= 0b00001000;
	if( digtab[chL] & 0x04 ) lcd_ram2 |= 0b00010000;

	if( digtab[chL] & 0x01 ) lcd_ram3 |= 0b00010000;
	return;
}
//---------------------------------------------------------------
//Function: lcd_display_msg(CHAR ch)
//input: ch
//显示一个十六进制值
//---------------------------------------------------------------
void lcd_display_msg(UCHAR ch)
{
	CHAR tmp;

//	GIE = 0;			                                    //先临时关闭总中断,disable all interrupt
	
	tmp = ch&0x0f;		                                    //low 4bits
	lcd_dig_low(tmp);

	tmp = ch >> 4;
	tmp &= 0x0f;
	lcd_dig_high(tmp);	                                    //high 4bits

//	GIE = 1;			                                    //Enable interrupt
	return;
}
//---------------------------------------------------------------
//Function: lcd_display_batt(UCHAR batt)
//input: batt
//显示电池状态
//---------------------------------------------------------------
void lcd_display_batt(UCHAR batt)
{
	lcd_ram0 &= 0b11111110;
	lcd_ram1 &= 0b11111110;
	lcd_ram2 &= 0b11111110;
	lcd_ram3 &= 0b11111110;

	batt &= 0x07;
	switch(batt) {
    	case 0:
        	lcd_ram3 ^= 0b00001000;
    		break;
    	case 1:
    		lcd_ram0 |= 0b00000001;
        	lcd_ram3 |= 0b00001000;
    		break;
    	case 2:
    		lcd_ram1 |= 0b00000001;
    		lcd_ram0 |= 0b00000001;
        	lcd_ram3 |= 0b00001000;
    		break;
    	case 3:
    		lcd_ram2 |= 0b00000001;
    		lcd_ram1 |= 0b00000001;
    		lcd_ram0 |= 0b00000001;
        	lcd_ram3 |= 0b00001000;
    		break;
    	case 4:                                             // blink battery case
        	lcd_ram3 |= 0b00001000;
    		lcd_ram3 |= 0b00000001;
    		lcd_ram2 |= 0b00000001;
    		lcd_ram1 |= 0b00000001;
    		lcd_ram0 |= 0b00000001;
    		break;
    	default:
    		break;
	}
    return;
}

//---------------------------------------------------------------
//Function: lcd_display_light(UCHAR level)
//input: level
//显示亮度等级,level =0, 1, 2, 3
//---------------------------------------------------------------
void lcd_display_light(UCHAR level)
{
	lcd_ram0 &= 0b11011111;
	lcd_ram1 &= 0b11011111;
	lcd_ram2 &= 0b11011111;
	lcd_ram3 &= 0b11011111;

	level &= 0x03;
	switch(level) {
    	case 0:
    		lcd_ram0 |= 0b00100000;
    		break;
    	case 1:
    		lcd_ram1 |= 0b00100000;
    		lcd_ram0 |= 0b00100000;
    		break;
    	case 2:
    		lcd_ram2 |= 0b00100000;
    		lcd_ram1 |= 0b00100000;
    		lcd_ram0 |= 0b00100000;
    		break;
    	case 3:
    		lcd_ram3 |= 0b00100000;
    		lcd_ram2 |= 0b00100000;
    		lcd_ram1 |= 0b00100000;
    		lcd_ram0 |= 0b00100000;
    		break;
	}
    return;
}
//---------------------------------------------------------------
//Function: led_select(void)
//input: FFlags.led, 0=LED1, Other = LED2
//LED 通道选择
//---------------------------------------------------------------
void led_select(void)
{
	if(FFlags.led) {	
        LED2_EN = ON;
        LED1_EN = OFF;
	} else {
		LED1_EN = ON;
		LED2_EN = OFF;
	}
    return;
}
//---------------------------------------------------------------
//Function: light_led_on(void)
//input:none
//LED Channel select
//---------------------------------------------------------------
void light_led_on(void)
{
    lamp_pr = 0;                                            //close LED PWM
    DelayMs(2);
	led_select();
	if(!bitFlag.lamp_on) {
	    if(FFlags.led) lamp_pr = BRIGHTNESS_RED;
        else lamp_pr = BRIGHTNESS_BLUE;
        DelayMs(5);
        lamp_pr = 0;
        DelayMs(2);
        LED1_EN = OFF;
        LED2_EN = OFF;
	} else {
        while(1) {                                          //lighting LED slowly
            if(FFlags.mtouch_toggle) {
                lamp_pr++;
                if(FFlags.led) {
                    if(lamp_pr == TOUCH_BRIGHTNESS_RED) break;
                } else {
                    if(lamp_pr == TOUCH_BRIGHTNESS_BLUE) break;
                }
    			if( !detect_touching()) {
    				sensor_Notouch();
    				break;
                }
            }
        }
    }
    return;
}
//---------------------------------------------------------------
//Function: CHAR hex_to_bcd(UCHAR hex)
//input: hex
//16进制转2位BCD码,返回BCD码
//---------------------------------------------------------------
CHAR hex_to_bcd(UCHAR hex)
{
	UCHAR var;
	CHAR bcd;
	var = hex % 100;
	bcd = (var/10) << 4;
	var %= 10;
	bcd |= var;
	return bcd;
}
//---------------------------------------------------------------
//Function: DelayMs(UCHAR msec)
//延时子程序,延时1ms
//---------------------------------------------------------------
void DelayMs(UCHAR msec)
{
	UCHAR i;
	do {
		DelayUs(125);
	} while(--msec);
    return;
}
//---------------------------------------------------------------
//Function: UCHAR adc_read(UCHAR channel)
//input:channel
//#define		BATT_TEMP	0x19
//#define		LED_TEMP	0x15
//#define		VOLTAGE		0x1d
//返回AD转换结果
//---------------------------------------------------------------
UCHAR adc_read(UCHAR channel)
{
	ADCON0 = channel;
    ADON = 1;
	asm("nop");
	asm("nop");
    asm("nop");
	asm("nop");
    asm("nop");
	asm("nop");
    asm("nop");
	asm("nop");
    asm("nop");
	asm("nop");
	ADGO = 1;			                                    //start a/d conversion
	while(ADGO)
		continue;		                                    //wait for conversion complete
    adresult = ADRES;
	ADCON0 = ADCON0_POWEROFF;                               //clear channel	
	asm("nop");
	asm("nop");
    asm("nop");
	asm("nop");
	return (adresult);
}
//---------------------------------------------------------------
//Function: key_scan(UCHAR key)
//input: key
//------PWR: 电源键
//------LIGHT: 亮度键
//------DEBUGKEY：进入调试模式组合键
//按键扫描,有键按下返回0， 无键按下返回非零值
//---------------------------------------------------------------
CHAR key_scan(void)
{
	if( !PWR_KEY ) {
		DelayMs(15);
		if( !PWR_KEY ) return 0;
	}
	return 1;
}
//---------------------------------------------------------------
//Function: charge_init(void)
//input: none
//充电初始化
//---------------------------------------------------------------
void start_charge_init(void)
{
    BACKLIGHT_POWER = ON;
    LCD_VCC = ON;
	OPTION = OPTION_VALUE;      // OPTION_REG: RBPU=dis,INTEDG=falling,T0CS=FOSC/4,T0SE=0,PSA=timer0,PS=16

    TMR1IE = 0;
    TMR1ON = 0;
    TMR1L = CHARGING_TMR1L;     //TIMER 1 register when charging, start value = 0xFFF0( as 3.8 microseconds overflow.)
    TMR1H = CHARGING_TMR1H;     //16000/4192 ms(charge PWM period width. 
    T1CON = 0b10111000;         //T1CON: TIMER1 CONTROL REGISTER
                                //bit 7-6 TMR1CS<1:0>: Timer1 Clock Source Select bits
                                //  11 =Timer1 clock source is Capacitive Sensing Oscillator (CAPOSC)
                                //* 10 =Timer1 clock source is pin or oscillator:
                                //      If T1OSCEN = 0:
                                //          External clock from T1CKI pin (on the rising edge)
                                //      If T1OSCEN = 1:
                                //*         Crystal oscillator on T1OSI/T1OSO pins
                                //  01 =Timer1 clock source is system clock (FOSC)
                                //  00 =Timer1 clock source is instruction clock (FOSC/4)
                                //bit 5-4 T1CKPS<1:0>: Timer1 Input Clock Prescale Select bits
                                //* 11 = 1:8 Prescale value
                                //  10 = 1:4 Prescale value
                                //  01 = 1:2 Prescale value
                                //  00 = 1:1 Prescale value
                                //bit 3 T1OSCEN: LP Oscillator Enable Control bit
                                //*  1 = Dedicated Timer1 oscillator circuit enabled
                                //   0 = Dedicated Timer1 oscillator circuit disabled
                                //bit 2  T1SYNC: Timer1 External Clock Input Synchronization Control bit
                                //      TMR1CS<1:0> = 1X
                                //          1 = Do not synchronize external clock input
                                //*         0 = Synchronize external clock input with system clock (FOSC)
                                //      TMR1CS<1:0> = 0X
                                //          This bit is ignored. Timer1 uses the internal clock when TMR1CS<1:0> = 1X.
                                //bit 1 Unimplemented: Read as ‘0’
                                //bit 0  TMR1ON: Timer1 On bit
                                //      1 = Enables Timer1
                                //*     0 = Stops Timer1
                                //      Clears Timer1 Gate flip-flop
// Timer1 = (32768Hz) /8 /16 = 256Hz, sec = 256Hz / 128 = 2Hz

    T1SYNC = 0;                 //Synchronize external clock input with system clock (FOSC)
    TMR1IF = 0; 
    TMR1IE = 1;                 //Enable timer1
    TMR1ON = 1;                 //start timer1

	TMR0 = TMR0_VALUE;			//Timer 0 should be 1ms interrupt
	msCount = 0;
	t0tick = 0;
    T0IE = 1;
    INTE = 0;
	enable_ADC();               //Enable ADC
    GIE = 1;
    return;
}
//---------------------------------------------------------------
//Function: init_charge_var(void)
//input: none
//varible init used in charging.
//---------------------------------------------------------------
void init_charge_var(void)
{
    charge_count = 0;                                       // charging timer
    iCount = 0;                                             // charging PWM count
    batt_n = 0;
    sec = 0;
    cnt=0;
    clr_lcd_ram();
    chargephase = 0;
    message = 0;
    FFlags.preChargetimeout = 0;                            //clear precharge time out error!
    FFlags.FULLChargetimeout = 0;
    return;
}
//---------------------------------------------------------------
//Function: check_environment(void)
//input: none
//  Note :when testing battery, CHG_PWM should be OFF.
//---------------------------------------------------------------
void check_environment(void)
{
    ad = adc_read( VOLTAGE );                               //get battery voltage.
    batvolt = (batvolt + (unsigned int)ad) >> 1;            //AD out should be smooth.

    ad = adc_read( BATT_TEMP );                             //get battery tempreture.
    battemp = (battemp + (unsigned int)ad) >> 1;

    ad = adc_read( LED_TEMP );                              //get led temp.
    ledtemp = (ledtemp + (unsigned int)ad) >> 1;            //AD out should be smooth.

	if((battemp > BATT_TEMP_HIGH)|(ledtemp > LED_TEMP_HIGH)) {
        bitFlag.temp_high = 1;
		if(sec & 0x01)
			lcd_ram3 |= 0b00000010;
		else
			lcd_ram3 &= 0b11111101;
    }else {
        lcd_ram3 &= 0b11111101;                             // clear tempreture high on LCD
        bitFlag.temp_high = 0;
    }
    return;
}

//---------------------------------------------------------------
//Function: enter_charge_loop(void)
//input: none
//charge process
//---------------------------------------------------------------
void enter_charge_loop(void)
{
    start_charge_init();
    init_charge_var();
    while(1) {
        if(DC_IN) break;                                    //No DC plug in, stop charge.
        
        if(bitFlag.toggle) {                                // 20.48ms
#ifdef __DEBUG1
    DEBUG2 = !DEBUG2;
#endif
            bitFlag.toggle = 0;
            cnt++;
            cnt &= 0x3f;
            if(!cnt) {
                check_environment();                        // check when 64*20.48ms=1290.24ms
#ifdef __DEBUG
                lcd_display_msg(batvolt);                   // show chargephase
#endif
            } else {
                if(!(cnt & 0x1f))lcd_display_msg(message);  // show charging counter or message
            }
            
// check tempreture
            if(bitFlag.temp_high) {                         //if Battery too hot, stop charge.
                message=0xe1;                               //E1:display charge error! 0xe1
                bitFlag.start_charge = 0;                   //stop charge
                continue;
            }
            
// check if timeout happened
            if( !bitFlag.start_charge ) {
                if(FFlags.preChargetimeout | FFlags.FULLChargetimeout) continue;
                if (batvolt > BATT_3) {
                    lcd_display_batt(4);                    //indicate no start charging
                    message = 0;
                    continue;                               // if batvolt>3.9V, don't start charge
                }
                bitFlag.start_charge = 1;
                charge_count = 0;
            }
            
// chargephase process
            switch(chargephase) {
                case 0:                                     // 0 pre_charging;
                    if (batvolt > BATT_EMPTY & (charge_count > START_CHARGE_TIME)) {
                        chargephase++;
                        charge_count = 0;
                    }
                    if(!cnt) batt_n++;
#ifdef __DEBUG
                    message = charge_count;
#endif
                    if (charge_count > PRE_CHARGE_TIME){
                        message = 0xe2;                     //E2:display precharge time out error! 0xe2
                        FFlags.preChargetimeout = 1;        //precharge time out error!
                        bitFlag.start_charge = 0;           //stop charge, will start after charge_count = 0
                    }
                    break;
                case 1:                                     // 1 full current charging;
                    if (batvolt > BATT_FULL) {
                        chargephase++;
                        charge_count = 0;
                    }
#ifdef __DEBUG
                        message = charge_count;
#endif
                    if (charge_count > FULL_CHARGE_TIME){
                        message = 0xe3;                     //E3:display FULLcharge time out error! 0xe3
                        FFlags.FULLChargetimeout = 1;       //FULLcharge time out error!
                        bitFlag.start_charge = 0;           //stop charge, will start after charge_count = 0
                    }
                    if(!(cnt & 0xF)) batt_n++;
                    break;
                case 2:                                     // 2 full voltage charge;
                    if(!cnt) batt_n++;
#ifdef __DEBUG
                        message = charge_count;
#endif
                    if (charge_count > TAIL_CHARGE_TIME){
                        message = 0;                        // charge completed.
                        batt_n = 4;
                        bitFlag.start_charge = 0;           // stop charge
                    }
                    break;
            }
            if(batt_n > 4) batt_n = 0;
            lcd_display_batt(batt_n);                       // indicate charging process
        }
    }
    power_off_hw_init();
    return;
}
//---------------------------------------------------------------
//Function: used_time_acc(void)
//input: none
//使用间隔累加(timer1 wake up counter,from sleep mode)
//---------------------------------------------------------------
void used_time_acc(void)
{
    TMR1IF = 0;                                             //reset timer1 interrupt flag
    TMR1H = POWEROFF_TMR1H;                                 //reset timer1(10 seconds overflow.)
    TMR1L = POWEROFF_TMR1L;

    t10_tick++;
    if(t10_tick > 60 ) {                                    // 600 seconds(10 minutes)
        t10_tick = 0;
        off_time++;
        if(off_time > 144) {                                // 24 hours (24*6 t10_tick(s))
            off_time = 0;
            if(FFlags.used)
                used_day++;
        }
    }
    return;
}
//---------------------------------------------------------------
//Function: power_off_loop(void)
//input: none
//进入关机状态，或刚插入电源时进入的状态
//---------------------------------------------------------------
void power_off_loop(void)
{
	power_off_hw_init();
    sec = 0;
	key_mode = 0;
    t10_tick = 0;
    off_time = 0;
    bitFlag.start = 0;
    
	while(1) {
		if( !DC_IN ) enter_charge_loop();                   //进入充电状态

        asm("clrwdt");
        asm("sleep");                                       //sleep status, exit by INT(RB0) and TIMER1 interrupt
        asm("nop");
        asm("nop");
        asm("nop");
        asm("nop");
        asm("nop");
        asm("nop");
        
#ifdef __DEBUG1
    DEBUG1 = !DEBUG1;
#endif
        if( TMR1IF ) used_time_acc();                       //关机计时
			
		if(!key_scan()) {                                   //key down got
            cap_init();
    		power_on_hw_init();
            while(!key_scan()) {
                if(bitFlag.start){
                    if(key_mode == LONG_KEY) {
                        key_mode++;
                        FFlags.beeping = 1;
                        while(FFlags.beeping | beep_count){};
                        FFlags.beeping = 1;
                        while(FFlags.beeping | beep_count){};
                    } else {
                        if(key_mode < LONG_KEY) key_mode = sec;     //500ms do ++
                    }
                }else{
                    key_mode = sec;                         //500ms do ++
                    if(key_mode > POWER_ONOFF){
        				power_on_pro();
                        lcd_display_msg(VERSION);           //LCD show soft version
                        bitFlag.start = 1;
                        FFlags.beeping = 1;
                        min = 0;
                    }
                }
            }
// Key released
            if(key_mode < POWER_ONOFF){
            	power_off_hw_init();                        // not long enough
            	key_mode = 0;
            }else{
                if(key_mode < LONG_KEY){
                    bitFlag.start = 1;
                }else{
                    FFlags.debug = 1;
                }
                break;
            }
        }
	}
    key_mode = 0;
    return;
}
//---------------------------------------------------------------
//Function: void shut_down_pro(void)
//input:none
//关机状态处理
//---------------------------------------------------------------
void shut_down_pro(void)
{
	//turn PWM off
	lamp_pr = 0;                                            //PWM占空比=lamp_pr
	clr_lcd_ram();                                          //shut down lcd display
    LCD_VCC = OFF;

	BACKLIGHT_POWER = OFF;                                  // backlight
	bitFlag.temp_high = 0;
    bitFlag.lamp_on = 0;
    FFlags.debug = 0;
	// led enable ?
	LED1_EN = OFF;
	LED2_EN = OFF;
    return;
}
//---------------------------------------------------------------
//Function: void power_on_pro(void)
//input:none
//关机状态处理
//---------------------------------------------------------------
void power_on_pro(void)
{
    BACKLIGHT_POWER = ON;
    cnt=0;
    dis_lcd_all_seg();
    bitFlag.pwr_off = 0;
    nFlag.auto_mode = 0;         // set auto mode
    nFlag.touchoff_waiting = 0;
    return;
}
//---------------------------------------------------------------
//Function: man_shut_down(void)
//input:none
//手动关机,(按电源键关机)
//---------------------------------------------------------------
void man_shut_down(void)
{
	if(key_scan()){                                         // no key down
	    if(!key_mode) return;                               // not realsing key
        if(key_mode > POWER_ONOFF){                         // almost 1.5 second
 			bitFlag.pwr_off = 1;                            // 设置关机标志
			shut_down_pro();
        }else{
            bitFlag.shortKey = 1;
            FFlags.beeping = 1;
        } 
    	key_mode = 0;                                       //reset counter time
    } else {                                                // key down
	    if(!key_mode) key_mode = 1;
        if(bitFlag.toggle_500ms) {
            if(key_mode > POWER_ONOFF) return;              // almost 1.5 second
            if(key_mode == POWER_ONOFF) {
                FFlags.beeping = 1;
                while(FFlags.beeping | beep_count){};
                BACKLIGHT_POWER = OFF;
                lamp_pr = 0;
                key_mode = POWER_ONOFF+1;
            } else {
                key_mode++;                                 // 500ms do ++
            }
        }
    }
	return;
}
//---------------------------------------------------------------
//Function: auto_shut_down(void)
//input:none
//定时自动关机，到时间了设置关机标志
//---------------------------------------------------------------
void auto_shut_down(void)
{	
	if(min > IDLE_DOWN) {
        while(FFlags.beeping | beep_count){};
        FFlags.beeping = 1;
        while(FFlags.beeping | beep_count){};
		bitFlag.pwr_off = 1;
		return;
	}
	if( !DC_IN ) {                                          //充电处理
		bitFlag.pwr_off = 1;
		shut_down_pro();
		return;
	}
    if(bitFlag.toggle_500ms) {
        if(sec & 0x01) {                                    // do every 500 ms
            if(batvolt < BATT_0) {
                lcd_display_batt(0);
                if(detect_touching() & (cnt & 0x01)) FFlags.beeping = 1;
                cnt++;
            	if(cnt > 39) {                              // shutdown when battery low
                    cnt = 0; 
                    bitFlag.pwr_off = 1; 
                }
                return;
            }
            cnt=0;
        	if(batvolt < BATT_1) { lcd_display_batt(1); return; }
        	if(batvolt < BATT_2) { lcd_display_batt(2); return; }
        	if(batvolt < BATT_3) lcd_display_batt(3);
            else lcd_display_batt(4);
        }
    }
    return;
}
//---------------------------------------------------------------
//Function: sensor_Notouch(void)
//input:none
//无触摸处理
//---------------------------------------------------------------
void sensor_Notouch(void)
{    
	bitFlag.lamp_on = 0;	                                //set close LED flag
    lamp_pr = 0;
    DelayMs(2);
    LED1_EN = OFF;
    LED2_EN = OFF;

	BACKLIGHT_POWER = ON;		                            //backlight of LCD on
	return;
}
//---------------------------------------------------------------
//Function: CHAR detect_touching(void)
//input:none
//touched: return 1=yes, 0=no
//---------------------------------------------------------------
CHAR detect_touching(void)
{
#ifdef __DEBUG
    DEBUG1 = !DEBUG1;
#endif
    if(FFlags.mtouch_toggle) {
#ifdef __DEBUG
    DEBUG2 = !DEBUG2;
#endif
        FFlags.mtouch_toggle = 0;
        bigval = cps_tmr1l + (unsigned int)(cps_tmr1h << 8);
        bigval = bigval << 4;

//avg >> 7 = 1/128 = 0.78% as a threshold value below average		
//avg >> 6 = 1/64  = 1.56% as a threshold value below average
//avg >> 5 = 1/32  = 3.12% as a threshold value below average
//avg >> 4 = 1/16  = 6.25% as a threshold value below average
//avg >> 3 = 1/8   = 12.5% as a threshold value below average
//avg >> 2 = 1/4   = 25%   as a threshold value below average
//avg >> 1 = 1/2   = 50%   as a threshold value below average

        if(FFlags.btn) {                                        //touched
            threshold_start = average >> 5;
            if( bigval > average + threshold_start ) {          //got key
                threshold_stop = average >> 6;
                if(average_n > (bigval - threshold_stop)) {     //match bigval
                    FFlags.btn = 0;
                    average = bigval;                           //window move
                    average_n = average;
                    cps_cnt = 1;
                }
            }else{
                threshold_stop = average >> 5;
                if(average_n < average - threshold_stop){
                    average = average_n;                        //window move
                }
            }
        } else {                                                //no touch
            threshold_start = average >> 4;
            if( bigval < average - threshold_start ) {          //got key
                threshold_stop = average >> 5;
                if(average_n < (bigval + threshold_stop)) {     //match bigval
                    FFlags.btn = 1;	
                    average = bigval;                           //window move
                    average_n = average;
                    cps_cnt = 1;
                }
            }else{
                threshold_stop = average >> 5;
                if(average_n > average + threshold_stop){
                    average = bigval;                           //window move
                }
            }
        }
        smallavg = average_n >> 4;
        average_n += bigval>>4;
        average_n -= smallavg;
        cps_cnt++;
        cps_cnt &= 0xFFF;
        if(!cps_cnt) average = average_n;                       // window move

#ifdef __DEBUG
        if(bitFlag.toggle_500ms) {
            switch(debug_cnt&0x07){
                case 0:
                    lcd_display_msg((min<<7) | sec);
                    break;
                case 1:
                    lcd_display_msg(bigval>>8);
                    break;
                case 2:
                    lcd_display_msg(bigval);
                    break;
                case 3:
                    lcd_display_msg(average_n>>8);
                    break;
                case 4:
                    lcd_display_msg(average_n);
                    break;
                case 5:
                    lcd_display_msg(average>>8);
                    break;
                case 6:
                    lcd_display_msg(average);
                    break;
                case 7:
                    lcd_display_msg(FFlags.btn);
                    break;
            }
            debug_cnt++;
        }
#endif
    }
    if(FFlags.debug) {                                      //debug mode?
        if(FFlags.btn) return(0);
        else return(1);
    } else {
	    if( FFlags.btn ) {
            return(1);
        } else return(0);
    }
}
//---------------------------------------------------------------
//Function: show_debug_msg(void)
//input:none
//在调试状态下的信息显示
//---------------------------------------------------------------
void show_debug_msg(void)
{
    if(sec & 0x2) {
        if(sec & 0x4) {
            if(sec & 0x8) lcd_display_msg(VERSION);         //version
            else lcd_display_msg(battemp);                  //bat_temp
        } else {
            if(sec & 0x8) lcd_display_msg(batvolt);         // VOLTAGE
            else lcd_display_msg(ledtemp);                  // led_temp 
        }
    }
    return;
}
//---------------------------------------------------------------
//Function: on_touch_check(void)
//input:none
//beep process, when touched
//---------------------------------------------------------------
void on_touch_check(void)
{
    if(bitFlag.toggle_500ms){
#ifndef __DEBUG
        lcd_display_msg(on_touch);
#endif
        if(FFlags.led){                                     // Red light
            if((sec == 0) & (on_touch == 3)) {
                FFlags.beeping = 1;                         // long beep
                while(FFlags.beeping | beep_count){};
                FFlags.beeping = 1;
                while(FFlags.beeping | beep_count){};
                FFlags.beeping = 1;
                sensor_Notouch();                           // turn off led, turn on lcd
//                while(detect_touching());
                if(!nFlag.auto_mode) {                      // manu mode
                    FFlags.led = 0;                         // set back to blue led
                }
                nFlag.touchoff_waiting = 1;
            }
        }else{                                              // Blue light
            if((sec == 0) & (on_touch == 1)) {
                FFlags.beeping = 1;                         // 1 beep
            }else{
                if((!(sec & 1)) & (on_touch == 2)) {        // beep twice
                    FFlags.beeping = 1;
                } 
                if((sec == 2) & (on_touch == 2)){
                    if(!nFlag.auto_mode) {                  // manu mode
                        FFlags.led = 1;                     // set to red led
                        on_touch = 0;
                        light_led_on();
                    }else{
                        sensor_Notouch();                   // turn off led, turn on lcd
                        nFlag.touchoff_waiting = 1;
                    }
                }
            }
        }
    }
    return;
}
//---------------------------------------------------------------
//Function: sensor_touch(void)
//input:none
//进入待机状态，或正常功能
//---------------------------------------------------------------
void sensor_touch(void)
{
	if(bitFlag.temp_high) {                                 //温度过高时，温度计闪动
        lamp_pr = 0;
		BACKLIGHT_POWER = ON;
        if( detect_touching() ) {
            if(bitFlag.toggle_500ms){
                if(sec & 0x3) {
                    FFlags.beeping = 1;                     //beeping in 2 seconds
                }
            }
        }
		return;	
	}
    
    if(nFlag.touchoff_waiting){
        if(!detect_touching()) nFlag.touchoff_waiting = 0;
        return;
    }
    
	if(bitFlag.lamp_on)	{                                   // lamp on?
        min = 0;                                            // clr idle counter
		if( detect_touching() == 0 ) {
			sensor_Notouch();
			return;
		}
        if(!FFlags.debug) {                                 //debug 模式下不做以下控制
            on_touch_check();
        } else {
            on_touch = 0;
            if(bitFlag.toggle_500ms){
#ifdef __DEBUG1
    DEBUG2 = !DEBUG2;
#endif
                if(!sec) {
                    FFlags.beeping = 1;                     //beeping in debug mode
                }
            }
        }
    } else {
        if( detect_touching() ) {
#ifndef __DEBUG
            BACKLIGHT_POWER = OFF;                          //backlight off
#endif
            on_touch = 0;
            bitFlag.lamp_on = 1;                            //设置LED开启标志
            FFlags.used = 1;                                //设置使用标志
            used_day = 0;                                   //使用间隔清零
            min = 0;
            sec = 0;
            light_led_on();
       }
    }
    return;
}
//---------------------------------------------------------------
//Function: enter_working_loop(void)
//input:none
//进入待机状态，或正常功能
//---------------------------------------------------------------
void enter_working_loop(void)
{
    clr_lcd_ram();                                          //clear LCD
    lcd_display_light(3);                                   //lightness
    if(!FFlags.debug)                                       //not in debug mode
        lcd_display_msg(hex_to_bcd(used_day));              //display days after last using
    lcd_display_batt(4);
    
    while(1) {
        if(bitFlag.toggle){                                 // do AD every 500ms
#ifdef __DEBUG1
    DEBUG1 = !DEBUG1;
#endif
            bitFlag.toggle = 0;
            bitFlag.toggle_500ms = 1;
        }
        sensor_touch();                                     //检测触摸感应

        auto_shut_down();                                   //定时关机

        man_shut_down();                                    //电源按键关机

        if(bitFlag.toggle_500ms){                           // do AD every 500ms
            check_environment();
            if(FFlags.debug) {                              //display debug msg
                show_debug_msg();
            }
            bitFlag.toggle_500ms = 0;
        }
        
        if(bitFlag.pwr_off) {
            break;
        }
        if( !DC_IN ) break;                                 //如果充电，关机
        if(bitFlag.shortKey) {
            FFlags.led = !FFlags.led;                       // change led channel
            light_led_on();                   
            nFlag.auto_mode = 1;                            // set manu mode
        	bitFlag.shortKey = 0;
        }
    }
    return;
}
//---------------------------------------------------------------
//Function: main()
//main loop
//---------------------------------------------------------------
void main(void)
{
	hw_init();
	BACKLIGHT_POWER = ON;
    DelayMs(10);
    variable_init();
	//main loop
	while(1) {
		power_off_loop();
		enter_working_loop();
	}
}

//---------------------------------------------------------------
//Function: isr()
//Interruput Service Routine
//1.lcd display refresh 
//2.some flag setting:
//3.and make decision if button/cap sensor has been touched
//---------------------------------------------------------------
void interrupt isr(void)
{   
	//Timer0 interrupt
	if(T0IE && T0IF) {
		T0IF = 0;						                    //clear T0IF every time
    	TMR0 = TMR0_VALUE;			                        //Timer 0 should be 160 cycles interrupt, 16/2000000us*160=1.28ms
        //time counter
		t0tick++;						                    // counter ++
		
        if(t0tick > PWM_WIDTH) t0tick = 0;                  // (PWM width should be 16 periods as 20.48ms) 16*1.28=20.48ms
        if(lamp_pr == 0) {LT_PWM = 0; goto isr_go; }        // lamp_pr=0时，PWM = 0
        if( !t0tick ) { LT_PWM = 1; goto isr_go; }          //PWM
		if( t0tick > lamp_pr ) LT_PWM = 0; 
        
isr_go:
        if( bitFlag.start_charge ) {
            switch(chargephase) {
                case 0:                                     // 0 pre_charging;
                    if (!FFlags.preChargetimeout){
                        if (t0tick > CHARGE_LOW){
                            if(t0tick == CHARGE_AD)bitFlag.toggle = 1;  //20.48ms 
                            CHG_PWM = OFF;
                        } else CHG_PWM = ON;
                    }
                    break;
                case 1:                                     // 1 full current charging;
                    if (!FFlags.FULLChargetimeout){
                        if (t0tick > CHARGE_HIGH){
                            if(t0tick == CHARGE_AD)bitFlag.toggle = 1;
                            CHG_PWM = OFF;
                        } else CHG_PWM = ON;
                    }
                    break;
                case 2:                                     // 2 full voltage charge;
                    if (t0tick > CHARGE_LOW){
                        if(t0tick == CHARGE_AD)bitFlag.toggle = 1;
                        CHG_PWM = OFF;
                    } else CHG_PWM = ON;
                    break;
            }
        }else {
            if(!DC_IN) {
                if(t0tick == CHARGE_AD)bitFlag.toggle = 1;
            }
        }
//		if( !(t0tick & 0x1) )                               //LCD scan: COM, SEG	
//		{
			if( lcd_s & 0x01 ) {
				COM_P = ON;
				SEG_P = OFF;
			}
			else {
				COM_P = OFF;
				SEG_P = ON;
			}
			sel = lcd_s & 0x07;
			lcd_s++;
			switch( sel ) {
                case 0:
                	COM0 = ON;
                	lcd_temp = ~lcd_ram0;
                    portData = PORTD;
                    portData &= 0xc0;
                    lcd_temp &= 0x3F;
                    portData |= lcd_temp;
                	PORTD = portData;
                	TRISD = lcd_temp;
                	TRISA = 0b00001110;
                	break;
                case 1:
                	COM0 = OFF;
                    portData = PORTD;
                    portData &= 0xc0;
                    lcd_ram0 &= 0x3f;
                    portData |= lcd_ram0;
                	PORTD = portData;
                	break;
                case 2:
                	COM1 = ON;
                	lcd_temp = ~lcd_ram1;
                    portData = PORTD;
                    portData &= 0xc0;
                    lcd_temp &= 0x3F;
                    portData |= lcd_temp;
                	PORTD = portData;
                	TRISD = lcd_temp;
                	TRISA = 0b00001101;
                	break;
                case 3:
                	COM1 = OFF;
                    portData = PORTD;
                    portData &= 0xc0;
                    lcd_ram1 &= 0x3f;
                    portData |= lcd_ram1;
                	PORTD = portData;
                	break;
                case 4:
                	COM2 = ON;
                	lcd_temp = ~lcd_ram2;
                    portData = PORTD;
                    portData &= 0xc0;
                    lcd_temp &= 0x3F;
                    portData |= lcd_temp;
                	PORTD = portData;
                	TRISD = lcd_temp;
                	TRISA = 0b00001011;
                	break;
                case 5:
                	COM2 = OFF;
                    portData = PORTD;
                    portData &= 0xc0;
                    lcd_ram2 &= 0x3f;
                    portData |= lcd_ram2;
                	PORTD = portData;
                	break;
                case 6:
                	COM3 = ON;
                    lcd_temp = ~lcd_ram3;
                	portData = PORTD;
                    portData &= 0xc0;
                    lcd_temp &= 0x3F;
                    portData |= lcd_temp;
                	PORTD = portData;
                	TRISD = lcd_temp;
                	TRISA = 0b00000111;
                	break;
                case 7:
                	COM3 = OFF;
                    portData = PORTD;
                    portData &= 0xc0;
                    lcd_ram3 &= 0x3f;
                    portData |= lcd_ram3;
                	PORTD = portData;
                	break;
            }
//        }
    }//end if(T0IE && T0IF)
	
	//Timer2 Interrupt(working in working loop, 0.512ms)
    if(TMR2IE && TMR2IF) {
        TMR2IF = 0;
        iCount++;
        if(iCount > T2_500MS ) {                            //match 500 ms
            iCount = 0;
            bitFlag.toggle = 1;
            sec++;
            if(sec > 119) {                                 //120 for 1 minute
                sec = 0;
                min++;
                if(!bitFlag.temp_high) on_touch++;
            }
        } 
        if(beep_count){                                     // if beeping
            beep_count--;
            if(!beep_count){                                // if beep end
                //CCP Module off
                CCP1CON = 0;	                            //PWM Mode, CCP1
                CCPR1L = 0;	                                //Duty cycle
                EN_BEEP = OFF;
            }
        }
        if(FFlags.beeping){
            EN_BEEP = ON;
            CCPR1H = 0;
            CCPR1L = 0b01000000;	                        //Duty cycle
            CCP1CON = 0b00001100;	                        //PWM Mode, CCP1
            beep_count = T2_125MS;
            FFlags.beeping = 0;
        }
	}

	//Timer1 Gate Interrupt
	if(TMR1GIF && TMR1GIE) {
        TMR1GIF = 0;                                        //clear Flag
        TMR1ON = 0;                                         //stop Timer1

        cps_tmr1h = TMR1H;
        cps_tmr1l = TMR1L;
        CPSCON1 = 0b0010;                                   //CPSCON1: select channel 2
        TMR1L = 0;                                          //Reset timer1
        TMR1H = 0;
        TMR1ON = 1;                                         //Restart Timer1
        FFlags.mtouch_toggle = 1;
    }

//timer1 interrupt
// Timer1 = (32768Hz) /8 /16 = 256Hz, sec = 256Hz / 128 = 2Hz
    if(TMR1IE && TMR1IF) {
        TMR1IF = 0;
        TMR1L = CHARGING_TMR1L;
        TMR1H = CHARGING_TMR1H;                             //reset timer1 reg.

#ifdef __DEBUG
    DEBUG1 = !DEBUG1;
#endif
        iCount++;
        if(iCount > T1_500ms) {                             // PWM width = 128Hz
            iCount = 0;
            sec++;                                          // 0.5 second counter
#ifdef __DEBUG
    DEBUG2 = !DEBUG2;
#endif
            if(sec > 119) {                                 // match 1 minute
                sec = 0;                            
                if (FFlags.preChargetimeout || FFlags.FULLChargetimeout){
                    charge_count--;
                    if (!charge_count) {
                        FFlags.preChargetimeout = 0;
                        FFlags.FULLChargetimeout = 0;
                    }
                }else charge_count++;                       // charging timer(minute)
            }
        }
    }
}
//
//end file
