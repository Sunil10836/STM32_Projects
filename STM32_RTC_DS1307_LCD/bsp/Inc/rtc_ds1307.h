/*
 * rtc_ds1307.h
 *
 *  Created on: Aug 9, 2025
 *      Author: Sunil Sutar
 */

#ifndef INC_RTC_DS1307_H_
#define INC_RTC_DS1307_H_

#include "stm32f446xx.h"

/* Application Configuration Parameters */
#define RTC_DS1307_I2C				I2C1
#define RTC_DS1307_I2C_GPIO_PORT	GPIOB
#define RTC_DS1307_I2C_PIN_SDA		GPIO_PIN_7
#define RTC_DS1307_I2C_PIN_SCL		GPIO_PIN_6
#define RTC_DS1307_I2C_SCL_SPEED	I2C_SCL_SPEED_SM
#define RTC_DS1307_I2C_PUPD			GPIO_PIN_PU				//Internal Pullup, External Pullup used 3.3KOhms

/* DS1307 RTC I2C Slave Address */
#define RTC_DS1307_I2C_ADDR		0x68

/* DS1307 RTC Register Address */
#define RTC_DS1307_ADDR_SEC		0x00
#define RTC_DS1307_ADDR_MIN		0x01
#define RTC_DS1307_ADDR_HRS		0x02
#define RTC_DS1307_ADDR_DAY		0x03
#define RTC_DS1307_ADDR_DATE	0x04
#define RTC_DS1307_ADDR_MONTH	0x05
#define RTC_DS1307_ADDR_YEAR	0x06

/*Time Formats*/
#define TIME_FORMAT_12HRS_AM	0
#define TIME_FORMAT_12HRS_PM	1
#define TIME_FORMAT_24HRS		2

/* Day Information*/
#define SUNDAY					1
#define MONDAY					2
#define TUESDAY					3
#define WEDNESDAY				4
#define THURSDAY				5
#define FRIDAY					6
#define SATURDAY				7

//RTC Date Structure
typedef struct
{
	uint8_t day;
	uint8_t date;
	uint8_t month;
	uint8_t year;
}RTC_Date_t;

//RTC Time Structure
typedef struct
{
	uint8_t seconds;
	uint8_t minutes;
	uint8_t hours;
	uint8_t time_format;
}RTC_Time_t;

/******************** Function prototypes **********************************/

uint8_t RTC_DS1307_Init(void);

void RTC_DS1307_Set_Current_Time(RTC_Time_t *);
void RTC_DS1307_Get_Current_Time(RTC_Time_t *);

void RTC_DS1307_Set_Current_Date(RTC_Date_t *);
void RTC_DS1307_Get_Current_Date(RTC_Date_t *);


#endif /* INC_RTC_DS1307_H_ */
