/*
 * stm32_rtc_ds1307_lcd.c
 *
 *  Created on: Aug 9, 2025
 *      Author: Sunil Sutar
 */

#include <stdio.h>
#include "../bsp/Inc/rtc_ds1307.h"
#include "../bsp/Inc/lcd.h"

#define SYSTICK_TIM_CLK		16000000

void Systick_Timer_Init(uint32_t tick_hz);

void RTC_Set_Date();
void RTC_Set_Time();

void RTC_Print_Time();
void RTC_Print_Date();
char* get_day_of_week(uint8_t i);

char* Time_to_String(RTC_Time_t *rtc_time);
char* Date_to_String(RTC_Date_t *rtc_date);
void Number_to_String(uint8_t num, char* buf);

RTC_Date_t current_date;
RTC_Time_t current_time;

int main(void)
{
	printf("RTC Test\n");

	LCD_Init();

	LCD_Print_String("RTC Test..");
	delay_ms(2000);

	LCD_Display_Clear();
	LCD_Display_Return_Home();

	if(RTC_DS1307_Init() == 1)
	{
		printf("RTC Init Failed\n");	//CH=1
		while(1);
	}

	Systick_Timer_Init(1);

	//Set Date and Time
	RTC_Set_Date();
	RTC_Set_Time();

	//Get Date and Time
	RTC_DS1307_Get_Current_Date(&current_date);
	RTC_DS1307_Get_Current_Time(&current_time);

	//Print Date and Time
	RTC_Print_Time();
	RTC_Print_Date();

	while(1);
}

void Systick_Timer_Init(uint32_t tick_hz)
{
	uint32_t *pSRVR = (uint32_t*)0xE000E014;
	uint32_t *pSCSR = (uint32_t*)0xE000E010;

    /* calculation of reload value */
    uint32_t count_value = (SYSTICK_TIM_CLK/tick_hz)-1;	//16MHz/1 = 16000000

    //Clear the value of SVR
    *pSRVR &= ~(0x00FFFFFFFF);

    //load the value in to SVR
    *pSRVR |= count_value;

    //do some settings
    *pSCSR |= ( 1 << 1); //Enables SysTick exception request:
    *pSCSR |= ( 1 << 2);  //Indicates the clock source, processor clock source

    //enable the systick
    *pSCSR |= ( 1 << 0); //enables the counter

}

void SysTick_Handler()
{
	//Get Date and Time
	RTC_DS1307_Get_Current_Date(&current_date);
	RTC_DS1307_Get_Current_Time(&current_time);

	//Print Date and Time
	RTC_Print_Time();
	RTC_Print_Date();
}

void RTC_Set_Date()
{
	current_date.day = SUNDAY;
	current_date.date = 10;
	current_date.month = 8;
	current_date.year = 25;

	RTC_DS1307_Set_Current_Date(&current_date);
}

void RTC_Set_Time()
{
	current_time.hours = 10;
	current_time.minutes = 30;
	current_time.seconds = 00;
	current_time.time_format = TIME_FORMAT_12HRS_AM;

	RTC_DS1307_Set_Current_Time(&current_time);
}

void RTC_Print_Time()
{
	char *am_pm;

	if(current_time.time_format != TIME_FORMAT_24HRS)
	{
		//TIME_FORMAT_12HRS_AM
		if(current_time.time_format == TIME_FORMAT_12HRS_AM)
		{
			am_pm = "PM";
		}
		else
		{
			am_pm = "AM";
		}
		printf("Current time = %s %s\n", Time_to_String(&current_time), am_pm); //10:30:00 AM
		LCD_Set_Cursor(1, 1);
		LCD_Print_String(Time_to_String(&current_time));
		LCD_Print_String(am_pm);
	}
	else
	{
		//TIME_FORMAT_24HRS
		printf("Current time = %s\n", Time_to_String(&current_time));
		LCD_Print_String(Time_to_String(&current_time));
	}
}

void RTC_Print_Date()
{
	//Date : 10/8/25 <SUNDAY>
	printf("Current Date = %s <%s> \n", Date_to_String(&current_date), get_day_of_week(current_date.day));
	LCD_Set_Cursor(2, 1);
	LCD_Print_String(Date_to_String(&current_date));
	LCD_Send_Char('<');
	LCD_Print_String(get_day_of_week(current_date.day));
	LCD_Send_Char('>');
}

char* get_day_of_week(uint8_t i)
{
	char* days[] = {"SUN", "MON", "TUE", "WED", "THU", "FRI", "SAT"};

	return days[i-1];
}

char* Time_to_String(RTC_Time_t *rtc_time)
{
	//HH:MM:SS
	static char buf[9];
	buf[2] = ':';
	buf[5] = ':';

	Number_to_String(rtc_time->hours, &buf[0]); //&buf[0] = &buf
	Number_to_String(rtc_time->minutes, &buf[3]);
	Number_to_String(rtc_time->seconds, &buf[6]);

	buf[8] = '\0';

	return buf;
}

char* Date_to_String(RTC_Date_t *rtc_date)
{
	//dd/mm/yy
	static char buf[9];
	buf[2] = '/';
	buf[5] = '/';

	Number_to_String(rtc_date->date, &buf[0]); //&buf[0] = &buf
	Number_to_String(rtc_date->month, &buf[3]);
	Number_to_String(rtc_date->year, &buf[6]);

	buf[8] = '\0';

	return buf;

}

void Number_to_String(uint8_t num, char* buf)
{
	if(num < 10)
	{
		buf[0] = '0';
		buf[1] = num + 48;
	}
	else if(num >= 10 && num < 99)
	{
		buf[0] = (num / 10) + 48;
		buf[1] = (num % 10) + 48;
	}
}
