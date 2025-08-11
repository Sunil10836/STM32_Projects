/*
 * rtc_ds1307.c
 *
 *  Created on: Aug 9, 2025
 *      Author: Sunil Sutar
 */

#include <stdint.h>
#include <string.h>					//for memset()
#include "../Inc/rtc_ds1307.h"

static void RTC_DS1307_I2C_Pin_Config();
static void RTC_DS1307_I2C_Config();
static void RTC_DS1307_Write(uint8_t value, uint8_t reg_addr);
static uint8_t RTC_DS1307_Read(uint8_t reg_addr);

static uint8_t Binary_to_BCD(uint8_t value);
static uint8_t BCD_to_Binary(uint8_t value);

//I2C1 Handle
I2C_Handle_t I2C1Handle;


/*
 * If this function return 1 then CH=1, init failed
 * If this function return 0 then CH=0, init success
 */
uint8_t RTC_DS1307_Init(void)
{
	//1. Initialize I2C GPIO Pins
	RTC_DS1307_I2C_Pin_Config();

	//2. Initialize I2C Peripheral
	RTC_DS1307_I2C_Config();

	//3. Enable I2C Peripheral
	I2C_PeripheralControl(RTC_DS1307_I2C, ENABLE);

	//4. Enable RTC Oscillator by making CH=0
	RTC_DS1307_Write(0x00, RTC_DS1307_ADDR_SEC);

	//5. Read back Clock Halt(CH) bit
	uint8_t clock_state = RTC_DS1307_Read(RTC_DS1307_ADDR_SEC);

	return ((clock_state >> 7) & 0x1);

}

void RTC_DS1307_Set_Current_Time(RTC_Time_t *rtc_time)
{
	uint8_t seconds, minutes, hours;
	//convert Binary to BCD
	seconds = Binary_to_BCD(rtc_time->seconds);

	/********** Configure Seconds Register *************/
	seconds &= ~(1 << 7);	//CH=0
	RTC_DS1307_Write(seconds, RTC_DS1307_ADDR_SEC);

	/********** Configure minutes Register *************/
	minutes = Binary_to_BCD(rtc_time->minutes);
	RTC_DS1307_Write(minutes, RTC_DS1307_ADDR_MIN);

	/********** Configure hour Register ***************/
	hours = Binary_to_BCD(rtc_time->hours);

	if(rtc_time->time_format == TIME_FORMAT_24HRS)
	{
		//TIME_FORMAT_24HRS
		//Clear 6th bit of Hour register
		hours &= ~(1 << 6);
	}
	else
	{
		//12 Hrs Format
		//Set 6th bit of Hour register
		hours |= (1 << 6);

		if(hours == TIME_FORMAT_12HRS_PM)
		{
			//TIME_FORMAT_12HRS_PM
			//Set 5th bit of Hour register
			hours |= (1 << 5);
		}
		else
		{
			//TIME_FORMAT_12HRS_AM
			//Clear 5th bit of Hour register
			hours &= ~(1 << 5);
		}
	}
	RTC_DS1307_Write(hours, RTC_DS1307_ADDR_HRS);
}

void RTC_DS1307_Get_Current_Time(RTC_Time_t *rtc_time)
{
	uint8_t seconds, minutes, hours;

	/********** Read Seconds Register *************/
	seconds = RTC_DS1307_Read(RTC_DS1307_ADDR_SEC);
	seconds &= ~(1 << 7);	//CH=0
	rtc_time->seconds = BCD_to_Binary(seconds);

	/********** Read minutes Register *************/
	minutes = RTC_DS1307_Read(RTC_DS1307_ADDR_MIN);
	rtc_time->minutes = BCD_to_Binary(minutes);

	/********** Read hours Register *************/
	hours = RTC_DS1307_Read(RTC_DS1307_ADDR_HRS);

	if(hours & (1 << 6))
	{
		//12 Hr Format
		if(hours & (1 << 5))
		{
			//TIME_FORMAT_12HRS_PM
			rtc_time->time_format = TIME_FORMAT_12HRS_PM;
		}
		else
		{
			//TIME_FORMAT_12HRS_AM
			rtc_time->time_format = TIME_FORMAT_12HRS_AM;
		}

		//clear 5th and 6th bit
		hours &= ~(1 << 5);
		hours &= ~(1 << 6);
	}
	else
	{
		//24 Hr Format
		rtc_time->time_format = TIME_FORMAT_24HRS;
	}

	rtc_time->hours = BCD_to_Binary(hours);
}

void RTC_DS1307_Set_Current_Date(RTC_Date_t *rtc_date)
{
	uint8_t day, date, month, year;

	/********** Configure day Register ***************/
	day = Binary_to_BCD(rtc_date->day);
	RTC_DS1307_Write(day, RTC_DS1307_ADDR_DAY);

	/********** Configure date Register ***************/
	date = Binary_to_BCD(rtc_date->date);
	RTC_DS1307_Write(date, RTC_DS1307_ADDR_DATE);

	/********** Configure month Register ***************/
	month = Binary_to_BCD(rtc_date->month);
	RTC_DS1307_Write(month, RTC_DS1307_ADDR_MONTH);

	/********** Configure year Register ***************/
	year = Binary_to_BCD(rtc_date->year);
	RTC_DS1307_Write(year, RTC_DS1307_ADDR_YEAR);

}

void RTC_DS1307_Get_Current_Date(RTC_Date_t *rtc_date)
{
	uint8_t day, date, month, year;

	/********** Read day Register *************/
	day = RTC_DS1307_Read(RTC_DS1307_ADDR_DAY);
	rtc_date->day = BCD_to_Binary(day);

	/********** Read date Register *************/
	date = RTC_DS1307_Read(RTC_DS1307_ADDR_DATE);
	rtc_date->date = BCD_to_Binary(date);

	/********** Read month Register *************/
	month = RTC_DS1307_Read(RTC_DS1307_ADDR_MONTH);
	rtc_date->month = BCD_to_Binary(month);

	/********** Read year Register *************/
	year = RTC_DS1307_Read(RTC_DS1307_ADDR_YEAR);
	rtc_date->year = BCD_to_Binary(year);
}

static void RTC_DS1307_I2C_Pin_Config()
{
	GPIO_Handle_t i2c_sda, i2c_scl;

	memset(&i2c_sda, 0, sizeof(i2c_sda));
	memset(&i2c_scl, 0, sizeof(i2c_scl));

	/*
	 * I2C1_SCL => PB6 and I2C1_SDA => PB7
	*/
	i2c_sda.pGPIOx = RTC_DS1307_I2C_GPIO_PORT;
	i2c_sda.GPIO_PinConfig.GPIO_PinNumber = RTC_DS1307_I2C_PIN_SDA;
	i2c_sda.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_ALTFN;
	i2c_sda.GPIO_PinConfig.GPIO_PinAltFunMode = 4;
	i2c_sda.GPIO_PinConfig.GPIO_PinOpType = GPIO_OP_TYPE_OD;
	i2c_sda.GPIO_PinConfig.GPIO_PinPuPdControl = RTC_DS1307_I2C_PUPD;
	i2c_sda.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;

	GPIO_Init(&i2c_sda);

	i2c_scl.pGPIOx = RTC_DS1307_I2C_GPIO_PORT;
	i2c_scl.GPIO_PinConfig.GPIO_PinNumber = RTC_DS1307_I2C_PIN_SCL;
	i2c_scl.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_ALTFN;
	i2c_scl.GPIO_PinConfig.GPIO_PinAltFunMode = 4;
	i2c_scl.GPIO_PinConfig.GPIO_PinOpType = GPIO_OP_TYPE_OD;
	i2c_scl.GPIO_PinConfig.GPIO_PinPuPdControl = RTC_DS1307_I2C_PUPD;
	i2c_scl.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;

	GPIO_Init(&i2c_scl);

}

static void RTC_DS1307_I2C_Config()
{
	I2C1Handle.pI2Cx = RTC_DS1307_I2C;	//I2C1
	I2C1Handle.I2CConfig.I2C_ACKControl = I2C_ACK_ENABLE;
	I2C1Handle.I2CConfig.I2C_SCLSpeed = RTC_DS1307_I2C_SCL_SPEED; //SM
	I2C_Init(&I2C1Handle);
}

static void RTC_DS1307_Write(uint8_t value, uint8_t reg_addr)
{
	uint8_t tx[2];
	tx[0] = value;
	tx[1] = reg_addr;
	I2C_MasterSendData(&I2C1Handle, tx, 2, RTC_DS1307_I2C_ADDR, I2C_SR_DISABLE);
}

static uint8_t RTC_DS1307_Read(uint8_t reg_addr)
{
	uint8_t data;
	//first write register address and then read from that register address
	I2C_MasterSendData(&I2C1Handle, &reg_addr, 1, RTC_DS1307_I2C_ADDR, I2C_SR_DISABLE); //write
	I2C_MasterReceiveData(&I2C1Handle, &data, 1, RTC_DS1307_I2C_ADDR, I2C_SR_DISABLE);	//read
	return data;
}

static uint8_t Binary_to_BCD(uint8_t value)
{
	uint8_t m, n;
	uint8_t bcd;

	if(value >= 10)
	{
		m = value / 10;
		n = value % 10;
		bcd = (uint8_t)((m << 4) | n);
	}
	bcd = value;	//if value < 10

	return bcd;
}

static uint8_t BCD_to_Binary(uint8_t value)
{
	uint8_t m, n;
	uint8_t bin;

	m = (uint8_t)((value >> 4) * 10);
	n = value & (uint8_t)0x0F;
	bin = m + n;

	return bin;
}
