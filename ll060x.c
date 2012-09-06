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
	OSCCON = 0x20;			//内部时钟, 8M

	TRISA = 0b00001111;		//设置I/O口的方向，1=input, 0=output
	TRISB = 0b00010101;
	TRISC = 0b00000001;
	TRISD = 0b11111111;
	TRISE = 0b00000111;

	ANSELA = 0x00;			//set Digital I/O, 0=digital, 1= analog input
//	ANSELB = 0x00;			//设置为数字I/O
	ANSELD = 0x00;
	ANSELE = 0b00000111;	//设置PORTE,0,1,2为模拟I/O，其它为数字I/O

	ANSELB = 0b00000100;	//CPS2 select, TOUCH

	WPUB = 0x00;			//禁止PORTB内部弱上拉,disable portB weak pull_up
	PORTA = 0x00;			//clear port
	PORTB = 0x00;
	PORTC = 0x00;
	//
	APFCON = 0b00000001;	//CCP2 function is on RB3/CCP2
    //01010000
    ADCON1 = 0b01000000;    //a/d init, a/d clock=fosc/4, VREF = VDD

	OSCTUNE = 0b01111111;   //oscillator tuning ,maximum frequency
	//T1CON = 0b10111100;
	INTCON = 0x00;      	//disable interrupts ,GIE--PEIE--T0IE--INTE--RBIE--T0IF--INTF--RBIF(bit7---bit0)
    PEIE = 1;               //允许外设中断

    //LT_PWM = 1;             //关闭XC9104，防止上电时LED闪
}
//---------------------------------------------------------------
//Function: cap_init()
//Initialize the Capacitive Sense Module and Time Base Modules
//---------------------------------------------------------------
void cap_init(void)
{
	//set up variables
//	for(index=0; index<16; index++)
	average = 0;        //[index]
	average_n = 0;      //[index]
    cps_cnt = 0;
    
	T1CON = 0b11000101;		//timer1 ,1:1 prescale,TMR1ON=1
//	T1GCON = 0b11100001;	//timer1 gate init
//	T1GCON &= 0b11111100;	//clr T1GSS bits
//	T1GCON |= 0b00000010;	//set T1GSS for timer2 match PR2
    T1GCON = 0xE2;

    TMR2IF = 0;
	TMR2IE = 1;	
    //PR2 = 0xB4;			//
    PR2 = 0x40;
    T2CON = 0b00000110;		//T2ON
    
	CPSCON0 = 0b10001100;   //CPS module enable
	CPSCON1 = 0b0100;       // channel 2

	TMR1GIF = 0;            //Enable gate interrupt
	TMR1GIE = 1;
//	index = 2;
                            //关闭振荡电路，防止开机时LED闪烁
    lamp_pr = 0;            //关闭振荡电路
    DelayMs(2);

    //led enable
    LED1_EN = OFF;          //blue led disabled
    LED2_EN = OFF;          //red led disabled
    //init var

    FFlags.led = 0;         //FFlags.led == 1? YES = BLUE, NO = RED
 
    sec = 0;
    iCount = 0;
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
    t1_tick = 0;
    FFlags.used = 0;            //清使用标志
    FFlags.debug = 0;           //debug mode? 0=no, 1=yes
    FFlags.workStatus = 0;
    key_mode = 0;
	key_debug = 0;
    bitFlag.start =  0;         //
    ledIndex = 0;
    shortKey = 0;
    cnt = 0;
}
//---------------------------------------------------------------
//Function: power_on_hw_init()
//上电或开机初始设置
//---------------------------------------------------------------
void power_on_hw_init(void)
{
    //TMR1ON = 0;                 //timer1 off
    TMR1IE = 0;                 //disable timer1 interrupt
    T1SYNC = 0;
	LCD_VCC = ON;

	OPTION = 0b10000000;		//PORTB pull_up disabled, 
								//prescaler, timero module
								//internal clock(Fosc/4)
	//INTCON = 0b11100000;		//GIE-PEIE-T0IE-INTE-RBIE-T0IF-INTF-RBIF
	T0IE = 1;                   //Enable timer0 interrupt
    INTE = 0;                   //不允许RB0/INT中断
    GIE = 1;                    //全局中断允许

}
//---------------------------------------------------------------
//Function: power_off_hw_init()
//进入关机状态设置
//---------------------------------------------------------------
void power_off_hw_init(void)
{
    LT_PWM = 0;
    // INTCON = 0b01010000;    //GIE--PEIE--T0IE--INTE--RBIE--T0IF--INTF--RBIF(bit7---bit0)
    GIE = 0;
    T0IE = 0;
    INTE = 1;               //RB0/INT外部中断允许位
    
    TRISA = 0x00;           //
    TRISD = 0x00;

    PORTA = 0x00;
    PORTD = 0x00;
    shut_down_pro();
    TMR2ON = 0;             //timer2 off
    TMR2IE = 0;
    TMR1GIE = 0;            //DISABLE gate interrupt
    T1GCON = 0x00;          //Timer1 Gate Disable
    CPSCON0 = 0x00;         //disable CPS module
    CPSCON1 = 0x00;
    
    TMR1ON = 0;
    TMR1H = 0xee;
    TMR1L = 0x00;
    T1CON = 0b10111100;
    TMR1IE = 1;
    T1SYNC = 1;
    TMR1ON = 1;
	//disalbe ADC, 
	ENVBAT = OFF;
	ENVTEST = OFF;
    //var
    FFlags.debug = 0;
    
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
}
//---------------------------------------------------------------
//Function: lcd_display_msg(CHAR ch)
//input: ch
//显示一个十六进制值
//---------------------------------------------------------------
void lcd_display_msg(UCHAR ch)
{
	CHAR tmp;

	GIE = 0;			//先临时关闭总中断,disable all interrupt
	
	tmp = ch&0x0f;		//low 4bits
	lcd_dig_low(tmp);

	tmp = ch >> 4;
	tmp &= 0x0f;
	lcd_dig_high(tmp);	//high 4bits

	GIE = 1;			//Enable interrupt	
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
	lcd_ram3 |= 0b00001000;

	batt &= 0x07;
	switch(batt) {
	case 0:
		break;
	case 1:
		lcd_ram0 |= 0b00000001;
		break;
	case 2:
		lcd_ram1 |= 0b00000001;
		lcd_ram0 |= 0b00000001;
		break;
	case 3:
		lcd_ram2 |= 0b00000001;
		lcd_ram1 |= 0b00000001;
		lcd_ram0 |= 0b00000001;
		break;
	default:
		lcd_ram3 |= 0b00000001;
		lcd_ram2 |= 0b00000001;
		lcd_ram1 |= 0b00000001;
		lcd_ram0 |= 0b00000001;
		break;
	}
}
//---------------------------------------------------------------
//Function: lcd_display_tmp(UCHAR flag)
//input: flag
//显示温度计，flag=1显示，=0不显示
//---------------------------------------------------------------
void lcd_display_tmp(UCHAR flag)
{
	if(flag)
		lcd_ram3 |= 0b00000010;
	else
		lcd_ram3 &= 0b11111101;
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
}
//---------------------------------------------------------------
//Function: led_select(UCHAR channel)
//input: channel, 0=LED1, Other = LED2
//LED 通道选择
//---------------------------------------------------------------
void led_select(UCHAR channel)
{
	if(channel)
	{	
        LED1_EN = OFF;
        LED2_EN = ON;

	}
	else
	{
		LED1_EN = ON;
		LED2_EN = OFF;
	}
}
//---------------------------------------------------------------
//Function: change_led_channel(void)
//input:none
//LED Channel select
//---------------------------------------------------------------
void change_led_channel(void)
{
    if(bitFlag.start)
        return;
    //
    mode = 1;           //enter manual_mode
    //
    lamp_pr = 0;        //关闭振荡电路
    DelayMs(2);
    
    FFlags.led = ! FFlags.led;
	led_select(FFlags.led);
    DelayMs(1);
    
	if(!bitFlag.lamp_on) 
	{ 
	    if(FFlags.led)
            lamp_pr = BRIGHTNESS_RED;
        else
            lamp_pr = BRIGHTNESS_BLUE;
        DelayMs(5);
        lamp_pr = 0;
        LED1_EN = OFF;
        LED2_EN = OFF;
        
	}
    else {
        if(FFlags.led)
            lamp_pr = TOUCH_BRIGHTNESS_RED;
        else
            lamp_pr = TOUCH_BRIGHTNESS_BLUE;
    }    
	shortKey = 0;
	wait_key_release(WORKINGKEY);
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
	var = hex%100;
	bcd = (var/10)<<4;
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
	ADGO = 1;			//start a/d conversion
	while(ADGO)
		continue;		//wait for conversion complete
	ADCON0 = 0x00;      //clear channel	
	asm("nop");
	asm("nop");
    asm("nop");
	asm("nop");
	return (ADRES);
}
//---------------------------------------------------------------
//Function: wait_key_release(CHAR key)
//input: key
//------PWR: 电源键
//------LIGHT: 亮度键
//------DEBUGKEY：进入调试模式组合键
//等待按键释放
//---------------------------------------------------------------
void wait_key_release(CHAR key)
{
	//handle key press
	switch(key) {
	case PWR:
		while(1) {
			if( !PWR_KEY )
            {
                //	continue;
                key_press_num++;
                DelayMs(10);
                if(key_press_num > 200)         // >2s, enter into debug mode
                {
                    FFlags.debug = 1;

                    lamp_pr = 0;                //关闭振荡电路
                    DelayMs(2);
                    //led enable
                    LED1_EN = ON;               //blue led enable
                    LED2_EN = OFF;              //red led disabled
                    lamp_pr = TOUCH_BRIGHTNESS_BLUE;
                    if(!PWR_KEY)
                        continue;
                }
            }
            else
            {
                key_press_num = 0;              //reset
		        break;
            }
		}
		break;
	case WORKINGKEY:
        while(1) {
			if( !PWR_KEY )
				continue;
			break;
		}
		break;
	}
}
//---------------------------------------------------------------
//Function: key_scan(UCHAR key)
//input: key
//------PWR: 电源键
//------LIGHT: 亮度键
//------DEBUGKEY：进入调试模式组合键
//按键扫描,有键按下返回0， 无键按下返回非零值
//---------------------------------------------------------------
CHAR key_scan(UCHAR key)
{
	//key handle
	if( !PWR_KEY )
    {
		DelayMs(15);
		if( !PWR_KEY )
			return 0;
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
	OPTION = 0b10000000;		//PORTB pull_up disabled, 
								//prescaler, timero module
								//internal clock(Fosc/4)
    TMR1IE = 0;
    TMR1ON = 0;
    TMR1L = 0x00;
    TMR1H = 0xf8;               //TIMER1 init
    T1CON = 0b10111000;          //synchronize external clock input 
    T1SYNC = 0;                 //synchronize external clock input 
    TMR1IF = 0; 
    TMR1IE = 1;                 //Enable timer1
    TMR1ON = 1;                 //start timer1

    PR2 = 0xff;
    T2CON = 0b00000110;         //1:16prescaler, timer2 on

	//Enable ADC, before starting charge
    ENVBAT = ON;
	ENVTEST = ON;
	//INTCON = 0b11100000;		//GIE-PEIE-T0IE-INTE-RBIE-T0IF-INTF-RBIF
    T0IE = 1;
    INTE = 0;
    GIE = 1;
    
}
//---------------------------------------------------------------
//Function: end_charge_init(void)
//input: none
//结束充电设置
//---------------------------------------------------------------

void end_charge_init(void)
{
    clr_lcd_ram();
    LCD_VCC = OFF;
    //INTCON = 0b01010000;    //
    GIE = 0;
    T0IE = 0;
    INTE = 1;

	BACKLIGHT_POWER = OFF;

    TMR1IE = 0;
    TMR1ON = 0;
    TMR1H = 0xf0;           //TIMER1 init
    TMR1L = 0x00;
    T1CON = 0b10111100;
    T1SYNC = 1;
    TMR1IE = 1;
    TMR1ON = 1;
    
    T2CON = 0x00;           //timer2 off

    PORTA = 0x00;
    PORTD = 0x00;
    
	//Disable ADC, when charge finished
	ENVBAT = OFF;
	ENVTEST = OFF;

//    max_volt = 0;
//    mid_volt = 0;
//    min_volt = 0;
}
//---------------------------------------------------------------
//Function: init_charge_var(void)
//input: none
//充电用到的一些变量的初始化
//---------------------------------------------------------------
void init_charge_var(void)
{
    charge_count = 0;
    bitFlag.min_1 = 0;
    batt_n = 0;
//    min_cnt = 0;
//    max_volt = 0;
//    mid_volt = 0;
//    min_volt = 0;
    sec = 0;
    clr_lcd_ram();
    bitFlag.ms_500 = 0;
    //DelayMs(1);
}
//---------------------------------------------------------------
//Function: batt_test(void)
//input: none
//充电初始条件判断和设置
//---------------------------------------------------------------
void batt_test(void)
{
    ad = adc_read( VOLTAGE );
    if(ad > BATT_FULL)              //电压足够，不需要充电
    {
        lcd_display_batt( 4 );
        lcd_display_msg( 0xe0 );    //E0, 表示电压足够
        return;
    }

    ad = adc_read( BATT_TEMP );     //电池温度适合充电?
    if(ad > BATT_TEMP_HIGH )
    {
        lcd_display_msg( 0xe1 );   //E1:display charge error! 0xe1
        return;
    }

    //enable charge, setting
    CHG_PWM = ON;
    //set enable charge flag
    bitFlag.start_charge = 1;       //Enable charge
}

//---------------------------------------------------------------
//Function: enter_charge_loop(void)
//input: none
//进入充电模式
//---------------------------------------------------------------
void liMn_charge(void)
{
    if( !bitFlag.start_charge ) return;
    batt_n++;
    if(batt_n == 5 )
        batt_n = 0;
    lcd_display_batt(batt_n);       //更新电池框显示

 //   lcd_display_msg(hex_to_bcd(sec));
    
    if(bitFlag.min_1 != 1) return;  //每分钟去测试电压
    bitFlag.min_1 = 0;
    charge_count++;                 //1min conuter 
    if(charge_count < 120)          /*TOTAL_CHARGE_TIME*/
        return;
    CHG_PWM = OFF;
    bitFlag.start_charge = 0;       //end charge
    lcd_display_batt(4);
}
//---------------------------------------------------------------
//Function: enter_charge_loop(void)
//input: none
//进入充电模式
//---------------------------------------------------------------
void enter_charge_loop(void)
{
    start_charge_init();
    init_charge_var();
    batt_test();
    lcd_display_batt(4);
    while(1)
    {
        if( DC_IN )                 //未有充电电源结束充电
            break;
        if(bitFlag.ms_500)          //每隔500ms 去测试电压值
        {
            bitFlag.ms_500 = 0;
            liMn_charge();
        }
    }
    end_charge_init();
}
//---------------------------------------------------------------
//Function: used_time_acc(void)
//input: none
//使用间隔累加(timer1 wake up counter,from sleep mode)
//---------------------------------------------------------------
void used_time_acc(void)
{
    TMR1IF = 0;                         //清timer1 中断标志
    TMR1H = 0xee;                       //reset timer1(1s 中断一次)
    TMR1L = 0x00;

    t1_tick++;
    if(t1_tick == 32 )                  // second_15 125
    {   
        //BACKLIGHT_POWER = ON;
        //BACKLIGHT_POWER = ! BACKLIGHT_POWER;
        t1_tick = 0;
        off_time++;
        if(off_time == 225)             //incf day(5400)(hour=225)288
        {
            off_time = 0;
            //BACKLIGHT_POWER = ON;
            if(FFlags.used)
                used_day++;
        }
    }
}
//---------------------------------------------------------------
//Function: power_off_loop(void)
//input: none
//进入关机状态，或刚插入电源时进入的状态
//---------------------------------------------------------------
void power_off_loop(void)
{
	power_off_hw_init();
    
	while(1) {

		if( !DC_IN )
			enter_charge_loop();        //进入充电状态

        //TX_TEST = 0;
        asm("clrwdt");
        asm("sleep");                   //sleep status
        asm("nop");
        asm("nop");
        asm("nop");
        asm("nop");
        asm("nop");
        asm("nop");
        //TX_TEST = 1;
        
        if( TMR1IF )
			used_time_acc();            //关机计时
			
        while(1)
    	{
			if(key_scan(PWR_KEY))			
				break;
			
			key_mode++;
			if(key_mode > LONG_KEY)	    //长按时开机
			{
				key_mode = 0;
				power_on_pro();
				cap_init();
				power_on_hw_init();
                lcd_display_msg(VERSION);
                bitFlag.start = 1;
                key_press_num = 0;
				wait_key_release(PWR);
                min = 0;
    			goto    EXIT;
			}
    	}

		key_mode = 0;
	}

EXIT:
    key_mode = 0;
    return;
}

//---------------------------------------------------------------
//Function: beep_on_off(UCHAR onOff)
//input:ON\OFF
//beep on, off control
//---------------------------------------------------------------
void beep_on_off(UCHAR onOff)
{
    if(onOff)   //ON status
    {
        EN_BEEP = ON;
        CCPR1L = 0b00011110;	        //Duty cycle
        CCP1CON = 0b00111100;	        //PWM Mode, CCP1
    }
    else        //OFF status
    {
        //CCP Module off
        CCP1CON = 0;	                //PWM Mode, CCP1
        CCPR1L = 0;	                    //Duty cycle
        EN_BEEP = OFF;
    }

}
//---------------------------------------------------------------
//Function: void shut_down_pro(void)
//input:none
//关机状态处理
//---------------------------------------------------------------
void shut_down_pro(void)
{
    
	//turn PWM off
	lamp_pr = 0;                        //PWM占空比=lamp_pr
	// led enable ?
	LED1_EN = OFF;
	LED2_EN = OFF;
    //
	clr_lcd_ram();                      //shut down lcd display
    LCD_VCC = OFF;

    beep_on_off(OFF);
    // backlight
	BACKLIGHT_POWER = OFF;
	bitFlag.temp_high = 0;
    bitFlag.lamp_on = 0;
    FFlags.debug = 0;
    //
    mode = 0;                           //mode: return auto
}
//---------------------------------------------------------------
//Function: void power_on_pro(void)
//input:none
//关机状态处理
//---------------------------------------------------------------
void power_on_pro(void)
{
    BACKLIGHT_POWER = ON;
    dis_lcd_all_seg();
    
    bitFlag.pwr_off = 0;
    //lcd_display_light(4);
    //work status
    mode = 0;                           //mode: auto
    beep_en = 0;
    //
    on_touch = 0;
    //enable a/d
    ENVBAT = ON;
	ENVTEST = ON;
}
//---------------------------------------------------------------
//Function: man_shut_down(void)
//input:none
//手动关机,(按电源键关机)
//---------------------------------------------------------------
void man_shut_down(void)
{
    while(1)
	{
		//if(key_scan(PWR))			
		//	break;
		if( PWR_KEY )
            break;
        
        key_mode++;
        
        if(bitFlag.lamp_on)
            DelayMs(2);
        else
            DelayMs(10);
        //delete first key action
        if(bitFlag.start)
        {
            shortKey = 0;
            key_mode = 0;
            bitFlag.start = 0;
            return;
        }
        
        //
        if(key_mode > 25)			    //长按时开机
		{
			key_mode = 0;
			bitFlag.pwr_off = 1;		//设置关机标志
			shut_down_pro();
            ledIndex = 0;
			wait_key_release(WORKINGKEY);
			break;
		}else
		    shortKey = 1;
	}
  
	key_mode = 0;		                //reset counter time

}
//---------------------------------------------------------------
//Function: auto_shut_down(void)
//input:none
//定时自动关机，到时间了设置关机标志
//---------------------------------------------------------------
void auto_shut_down(void)
{	
	 
	if(min > IDLE_DOWN)
	{
		bitFlag.pwr_off = 1;
		return;
	}
	if( !DC_IN )	                    //充电处理
	{
		bitFlag.pwr_off = 1;
		shut_down_pro();
		return;
	}
    if(bitFlag.ms_500) 
    {
        bitFlag.ms_500 = 0;

    	ad = adc_read(VOLTAGE);

        //lcd_display_msg(ad);
        //return;
    	if(ad < BATT_EMPTY) cnt++;
    	if(cnt > 39) { cnt=0; bitFlag.pwr_off=1; return;}	//电池电压过小自动关机
    	if(ad < BATT_0) { lcd_display_batt(0); return; }
    	if(ad < BATT_1) { lcd_display_batt(1); return; }
    	if(ad < BATT_2) { lcd_display_batt(2); return; }
    	if(ad < BATT_3) lcd_display_batt(3);
        else
            lcd_display_batt(4);
    }
}
//---------------------------------------------------------------
//Function: sensor_Notouch(void)
//input:none
//无触摸处理
//---------------------------------------------------------------
void sensor_Notouch(void)
{    
	bitFlag.lamp_on = 0;	            //设置未开启标志
    //CCP module off
    beep_on_off(OFF);
    
    //led low status
    lamp_pr = 0;
    DelayUs(2);
    LED1_EN = OFF;
    LED2_EN = OFF;

	BACKLIGHT_POWER = ON;		        //打开背光
}
//---------------------------------------------------------------
//Function: CHAR detect_touching(void)
//input:none
//判断是否有触摸? return 1=yes, 0=no,与debug 相反
//---------------------------------------------------------------
CHAR detect_touching(void)
{
//    if( FFlags.btn ) return(1);
//	    else return(0);
    if(FFlags.debug)                    //yes
    {
        if(FFlags.btn) return(0);
        else return(1);
    }
    else
    {
	    if( FFlags.btn ) 
        {
            if(FFlags.touch_off)        //持续使用时间超过10min，不可使用
                return(0);
            else
                return(1);
        }
	    else return(0);
    }
}
//---------------------------------------------------------------
//Function: show_debug_msg(void)
//input:none
//在调试状态下的信息显示
//---------------------------------------------------------------
void show_debug_msg(void)
{
    UCHAR temp2;
    if(FFlags.DEBUG_SHOW)                       //YES
    {
        if(!(sec&0x2))
            FFlags.DEBUG_SHOW = 0;
    }
    else                                        //NO
    {
        if(sec&0x2)                             //YES
        {
            FFlags.DEBUG_SHOW = 1;
            if(sec&0x4)                         //YES
            {
                if(sec&0x8)                     //YES   version
                {
                    lcd_display_msg(VERSION);
                }
                else                            //NO    bat_temp
                {
                    temp2 = adc_read(BATT_TEMP);
                    lcd_display_msg(temp2);
                }
            }
            else                                //NO
            {
                if(sec&0x8)                     //YES VOLTAGE
                {
                    temp2 = adc_read(VOLTAGE);
                    lcd_display_msg(temp2);
                }
                else                            //NO led_temp 
                {
                    temp2 = adc_read(LED_TEMP);
                    lcd_display_msg(temp2);
                }
            }
        }
    }
}
//---------------------------------------------------------------
//Function: beep_pro(void)
//input:none
//beep process, when touched
//---------------------------------------------------------------
void beep_pro(void)
{
    if(beep_1 == 1)
        beep_on_off(OFF);
    if( !beep_en)
    {
        beep_5 = 0;
        return;
    }
// 5 min processer
    if(on_touch == 0x5)     // 5min
    {   
        if(beep_5 == 0)
        {   
            //BACKLIGHT_POWER = ON;
            beep_5 = 1;
            beep_1 = 0;
            beep_on_off(ON);
            return;
        }
        if(beep_1 < 2) return;
        
        //BACKLIGHT_POWER = OFF;
        beep_on_off(ON);
        beep_1 = 0;
        beep_en = 0;
        //
        lamp_pr = 0;
        DelayMs(2);
        FFlags.led = ! FFlags.led;
    	led_select(FFlags.led);
        DelayMs(2);
        if(FFlags.led)
            lamp_pr = TOUCH_BRIGHTNESS_RED;
        else
            lamp_pr = TOUCH_BRIGHTNESS_BLUE;
        //FFlags.touch_off = 1;           //disabled touch

        return;
       
    }
    // 1min
    beep_en = 0;
    beep_on_off(ON);
    beep_1 = 0;
}
//---------------------------------------------------------------
//Function: sensor_touch(void)
//input:none
//进入待机状态，或正常功能
//---------------------------------------------------------------
void sensor_touch(void)
{
    UCHAR temp;

    //display debug msg
    if(FFlags.debug) 
    {
        on_touch = 0;
        show_debug_msg();
    }
    if( !FFlags.btn ) {on_touch = 0; FFlags.touch_off = 0; }
	if(bitFlag.temp_high)					//温度过高时，温度计闪动
	{
	    min = 0;
		if(bitFlag.temp_flash)
			lcd_ram3 |= 0b00000010;
		else
			lcd_ram3 &= 0b11111101;
        temp = adc_read(LED_TEMP);          //温度降下来继续工作
        if(temp < LED_TEMP_LOW)
        {
            lcd_ram3 &= 0b11111101;         //
            bitFlag.temp_high = 0;
            if(FFlags.led)
                lamp_pr = TOUCH_BRIGHTNESS_RED;
            else
                lamp_pr = TOUCH_BRIGHTNESS_BLUE;
            BACKLIGHT_POWER = OFF;		    //Turn backlight off
        }     
		return;	
	}
    
	if(bitFlag.lamp_on)					    //LED触摸是否已经开启?
	{
        min = 0;                            //有触摸时，空闲计时清零
        //yes 已经开启
		if( detect_touching()== 0 ) 
		{
			sensor_Notouch();
			return;
		}

        if(!FFlags.debug)                   //debug 模式下不做以下控制
        {
            // beep ctrl
            beep_pro();

            if(on_touch > 12)               //持续使用超过10min，离开下方可使用
            {
                FFlags.touch_off = 1;
                lcd_display_msg(0xee);      //持续使用时间过长保护0xEE
            }
        } else {
            temp = sec;
            if((temp & 0x3) == 0x0){
                beep_on_off(ON);
            }
            else {
                beep_on_off(OFF);
            }
        }
        //
		temp = adc_read(BATT_TEMP);       
		if(temp > BATT_TEMP_HIGH)
			goto	high_temp_set;
		temp = adc_read(LED_TEMP); 
        //lcd_display_msg(temp);
		if(temp > LED_TEMP_HIGH)
			goto	high_temp_set;
		return;
high_temp_set:
		lcd_ram3 |= 0b00000010;		        //显示温度计
		bitFlag.temp_high = 1;              //设置高温标志
        //led low
        lamp_pr = 0;

		BACKLIGHT_POWER = ON;
		return;
	}
///
	if( detect_touching() == 1 )
	{
	    BACKLIGHT_POWER = OFF;              //backlight off
	    
//	    if(!FFlags.debug)
//        {
            if(FFlags.led){ LED1_EN = OFF; LED2_EN = ON;}
                else {LED1_EN = ON; LED2_EN = OFF;}
//        }

        lamp_pr = 0;
        while(1) {                          //缓慢点亮
            DelayMs(2);
            lamp_pr++;
            if(FFlags.led)
            {
                if(lamp_pr == TOUCH_BRIGHTNESS_RED)
                    break;
            }
            else
            {
                if(lamp_pr == TOUCH_BRIGHTNESS_BLUE)
                    break;
            }
			if( detect_touching() == 0 ) 
			{
				sensor_Notouch();
				break;
			}
		}
        
		bitFlag.lamp_on = 1;            //设置LED开启标志
		FFlags.used = 1;                //设置使用标志
		used_day = 0;                   //使用间隔清零
		min = 0;
        off_time = 0;
        t1_tick = 0;
        //
        iCount = 0;
        sec = 0;
	}		
}
//---------------------------------------------------------------
//Function: enter_working_loop(void)
//input:none
//进入待机状态，或正常功能
//---------------------------------------------------------------
void enter_working_loop(void)
{
    clr_lcd_ram();                                  //clear LCD
    lcd_display_light(3);                          //lightness
    if(!FFlags.debug)                               //not in debug mode
        lcd_display_msg(hex_to_bcd(used_day));      //使用间隔 
    lcd_display_batt(4);
    
	while(1) {
		sensor_touch();                 //检测触摸感应

        auto_shut_down();               //定时关机

        man_shut_down();                //电源按键关机
		
		if(bitFlag.pwr_off)
			break;
        if( !DC_IN )                    //如果充电，关机
            break;
		if(shortKey)
		    change_led_channel();       //按键调整选择LED 通道 
	}
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
	if(T0IE && T0IF)
	{
		T0IF = 0;						//clear T0IF every time
		TMR0 = 215;						//恢复timer0 初始值
        //time counter
        //------------------------------
		t0tick++;						//中断计数加 1
		
        if(lamp_pr == 0) {LT_PWM = 0; goto isr_go; }            // lamp_pr=0时，关闭振荡驱动
        if(t0tick == 0x7F) t0tick = 0;
        if( !t0tick ) { LT_PWM = 1; goto isr_go ;}              //PWM
		//if( lamp_pr == TOUCH_BRIGHTNESS ) goto	isr_go;
		if( t0tick == lamp_pr ) LT_PWM = 0; 
        
isr_go:
		if( !(t0tick&0x1F))				//循环扫描COM, SEG	
		{
			if( lcd_s&0x01 ) {
				COM_P = ON;
				SEG_P = OFF;
			}
			else {
				COM_P = OFF;
				SEG_P = ON;
			}
			sel = lcd_s&0x07;
			lcd_s++;
			switch( sel )
			{
			case 0:
				COM0 = ON;
				lcd_temp = ~lcd_ram0;
                portData = PORTD;
                portData &= 0xc0;
                lcd_temp &= 0x3F;       //
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
                lcd_temp &= 0x3F;       //
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
                lcd_temp &= 0x3F;       //
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
                lcd_temp &= 0x3F;       //
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
          }
	}//end if(T0IE && T0IF)
	
	//Timer2 Interrupt
	if(TMR2IE && TMR2IF)
	{
		TMR2IF = 0;
        iCount++;
        if(iCount == 995 )
        {
            iCount = 0;
            bitFlag.ms_500 = 1;
            beep_1++;
            if(bitFlag.temp_high)
                bitFlag.temp_flash = !bitFlag.temp_flash;
            sec++;
            if(sec == 120)          //120
            {
                sec = 0;
                min++;
                on_touch++;
                beep_en = 1;        //enable beep
            }

        } 
	}

	//Timer1 Gate Interrupt
	if(TMR1GIF && TMR1GIE)
	{
        TMR1GIF = 0;                                //clear Flag
        TMR1ON = 0;                                 //stop Timer1

        bigval = TMR1L + (unsigned int)(TMR1H << 8);
        bigval = bigval*16;

//avg >> 7 = 1/128 = 0.78% as a threshold value below average		
//avg >> 6 = 1/64  = 1.56% as a threshold value below average
//avg >> 5 = 1/32  = 3.12% as a threshold value below average
//avg >> 4 = 1/16  = 6.25% as a threshold value below average
//avg >> 3 = 1/8   = 12.5% as a threshold value below average
//avg >> 2 = 1/4   = 25%   as a threshold value below average
//avg >> 1 = 1/2   = 50%   as a threshold value below average
        threshold_start = average >> 5;
        threshold_stop  = average >> 7;

        if(FFlags.btn)
        {                                           //touched
            if( bigval > average + threshold_start )     //[index], got key, 减少延迟
            {
                if(average_n > (bigval - threshold_stop))     //match bigval
                {
                    FFlags.btn = 0;
                    average = bigval;               //窗口立即移动
                    cps_cnt = 1;
                }
            }
        }else
        {                                           //no touch
            if( bigval < average - threshold_start )     //[index], got key
            {
                if(average_n < (bigval + threshold_stop))     //match bigval
                {
                    FFlags.btn = 1;	
                    average = bigval;               //窗口立即移动
                    cps_cnt = 1;
                }
            }
        }
        smallavg = average_n / 16;                  //[index]
        average_n += bigval/16 - smallavg;          //[index]
        cps_cnt++;
        if((cps_cnt & 0x3FF) == 0x0)                // 窗口移动
        {
            average = average_n;
        }
        CPSCON1 = 2;                                // Select external pin CPS0..CPS15
		
        TMR1L = 0;                                  //Reset timer1
        TMR1H = 0;
        TMR1ON = 1;                                 //Restart Timer1
    }

#ifdef wkf
	{
        TMR1GIF = 0;                                //clear Flag
        TMR1ON = 0;                                 //stop Timer1

        bigval = TMR1L + (unsigned int)(TMR1H << 8);
        bigval = bigval*16;
        //reading = bigval;
        smallavg = average / 16;                    //[index]
		
        //threshold128	= average[index] >> 7;      //avg >> 7 = 1/128 = 0.78% as a threshold value below average
        //threshold16   = average[index] >> 6;      //avg >> 6 = 1/64  = 1.56% as a threshold value below average
        threshold16     = average >> 5;             //[index]avg >> 5 = 1/32  = 3.12% as a threshold value below average
        //threshold16   = average[index] >> 4;      //avg >> 4 = 1/16  = 6.25% as a threshold value below average
        //threshold16   = average[index] >> 3;      //avg >> 3 = 1/8   = 12.5% as a threshold value below average
        //threshold4    = average[index] >> 2;      //avg >> 2 = 1/4   = 25%   as a threshold value below average
        //threshold2    = average[index] >> 1;      //avg >> 1 = 1/2   = 50%   as a threshold value below average

//        threshold = threshold16;

        if( bigval < average - threshold16 )        //[index]
        {
            FFlags.btn = 1;	
/*
            switch(index)
            {
            case 2:	
                FFlags.btn = 1;	
                break;
            default:
                break;
            }
*/
        }
        else 
        {
            FFlags.btn = 0;	
/*
            switch(index)
            {
            case 2:	
                FFlags.btn = 0;	
                break;
            default:
                break;
            }
*/
            average += bigval/16 - smallavg;        //[index]
        }
        CPSCON1 = 2;                    // Select external pin CPS0..CPS15
		
        TMR1L = 0;                      //Reset timer1
        TMR1H = 0;
        TMR1ON = 1;                     //Restart Timer1
    }
#endif
    //timer1 interrupt
    if(TMR1IE && TMR1IF)
    {
        TMR1IF = 0;
        TMR1H = 0xf8;                   //reset timer1
        //TMR1L = 0x00;
        bitFlag.ms_500 = 1;
        sec++;
        if(sec == 120)
        {
            sec = 0;
            bitFlag.min_1 = 1;
        }
    }
}
//
//end file
