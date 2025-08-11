/*
 * lcd.h
 *
 *  Created on: Aug 9, 2025
 *      Author: Sunil Sutar
 */

#ifndef INC_LCD_H_
#define INC_LCD_H_

#include "stm32f446xx.h"

/* Application Configuration Parameters */
#define LCD_GPIO_PORT	GPIOD
#define LCD_GPIO_RS		GPIO_PIN_0
#define LCD_GPIO_RW		GPIO_PIN_1
#define LCD_GPIO_EN		GPIO_PIN_2
#define LCD_GPIO_D4		GPIO_PIN_3
#define LCD_GPIO_D5		GPIO_PIN_4
#define LCD_GPIO_D6		GPIO_PIN_5
#define LCD_GPIO_D7		GPIO_PIN_6

/*LCD Commands*/
#define LCD_CMD_4DL_2N_5X8F			0x28		//Function Set Command
#define LCD_CMD_DON_CURON			0x0E
#define LCD_CMD_INCADDR				0x06		//Entry Mode Set Command
#define LCD_CMD_DIS_CLEAR			0x01
#define LCD_CMD_DIS_RETURN_HOME		0x02

/******************** Function prototypes **********************************/
void LCD_Init();
void LCD_Send_Command(uint8_t cmd);
void LCD_Send_Char(uint8_t data);
void LCD_Print_String(char *message);

void LCD_Display_Clear();
void LCD_Display_Return_Home(void);
void LCD_Set_Cursor(uint8_t row, uint8_t column);

void delay_ms(uint32_t cnt);
void delay_us(uint32_t cnt);

#endif /* INC_LCD_H_ */
