/*
 * lcd.c
 *
 *  Created on: Aug 9, 2025
 *      Author: Sunil Sutar
 */

#include "../Inc/lcd.h"

static void LCD_Write_Nibble(uint8_t value);
static void LCD_Enable();
static void LCD_GPIO_Pin_Config();

void LCD_Init()
{
	//1. Configure GPIO Pins for LCD Pins
	LCD_GPIO_Pin_Config();

	//2. Do LCD Init
	delay_ms(40);

	//RS=0, for LCD Command and make R/W=0
	GPIO_WriteToOutputPin(LCD_GPIO_PORT, LCD_GPIO_RS, GPIO_PIN_RESET);  //RS=0
	GPIO_WriteToOutputPin(LCD_GPIO_PORT, LCD_GPIO_RW, GPIO_PIN_RESET);  //R/W=1

	LCD_Write_Nibble(0x3); 	// 0 0 1 1, D4=1, D5=1, D6=0, D7=0
	delay_ms(5);

	LCD_Write_Nibble(0x3);
	delay_us(150);

	LCD_Write_Nibble(0x3);
	LCD_Write_Nibble(0x2);

	//Function Set Command
	LCD_Send_Command(LCD_CMD_4DL_2N_5X8F);

	//Display On/Off Control Command, Display ON, Cursor ON
	LCD_Send_Command(LCD_CMD_DON_CURON);

	//Display Clear Command
	LCD_Display_Clear();

	//Entry Mode Set Command
	LCD_Send_Command(LCD_CMD_INCADDR);

}

void LCD_Send_Command(uint8_t cmd)
{
	//Make RS=0 and R/W=0
	GPIO_WriteToOutputPin(LCD_GPIO_PORT, LCD_GPIO_RS, GPIO_PIN_RESET);  //RS=0
	GPIO_WriteToOutputPin(LCD_GPIO_PORT, LCD_GPIO_RW, GPIO_PIN_RESET);  //R/W=1

	//Send Higher Nibble of command
	LCD_Write_Nibble(cmd >> 4);

	//Send Lower Nibble of command
	LCD_Write_Nibble(cmd & 0x0F);

	//Make HIGH to LOW transition on Enable Pin
	LCD_Enable();
}

void LCD_Send_Char(uint8_t data)
{
	//Make RS=1 for Data and R/W=0
	GPIO_WriteToOutputPin(LCD_GPIO_PORT, LCD_GPIO_RS, GPIO_PIN_RESET);  //RS=0
	GPIO_WriteToOutputPin(LCD_GPIO_PORT, LCD_GPIO_RW, GPIO_PIN_RESET);  //R/W=1

	//Send Higher Nibble of data
	LCD_Write_Nibble(data >> 4);

	//Send Lower Nibble of data
	LCD_Write_Nibble(data & 0x0F);

	//Make HIGH to LOW transition on Enable Pin
	LCD_Enable();
}

void LCD_Print_String(char *message)
{
	do
	{
		LCD_Send_Char((uint8_t)*message++);

	}while(*message != '\0');
}

static void LCD_Write_Nibble(uint8_t value)
{
	GPIO_WriteToOutputPin(LCD_GPIO_PORT, LCD_GPIO_D4, ((value >> 0) & 0x1));
	GPIO_WriteToOutputPin(LCD_GPIO_PORT, LCD_GPIO_D5, ((value >> 1) & 0x1));
	GPIO_WriteToOutputPin(LCD_GPIO_PORT, LCD_GPIO_D6, ((value >> 2) & 0x1));
	GPIO_WriteToOutputPin(LCD_GPIO_PORT, LCD_GPIO_D7, ((value >> 3) & 0x1));

	LCD_Enable();
}

/*
 * Set LCD Cursor to specified Location given by row and column information
 * Row Number : 1 to 2
 * Column Number : 1 to 16, Assuming a 16x2 Character Display
 */
void LCD_Set_Cursor(uint8_t row, uint8_t column)
{
	column--;
	switch(row)
	{
		case 1:
			LCD_Send_Command((column |= 0x80));
			break;
		case 2:
			LCD_Send_Command((column |= 0xC0));
			break;
		default:
			break;
	}
}

static void LCD_Enable()
{
	GPIO_WriteToOutputPin(LCD_GPIO_PORT, LCD_GPIO_EN, GPIO_PIN_SET);
	delay_ms(10);
	GPIO_WriteToOutputPin(LCD_GPIO_PORT, LCD_GPIO_EN, GPIO_PIN_RESET);
	delay_ms(100);
}

void LCD_Display_Clear()
{
	LCD_Send_Command(LCD_CMD_DIS_CLEAR);
	delay_ms(2);
}

void LCD_Display_Return_Home(void)
{
	LCD_Send_Command(LCD_CMD_DIS_RETURN_HOME);
	delay_ms(2);
}

static void LCD_GPIO_Pin_Config()
{
	GPIO_Handle_t lcdPins;
	lcdPins.pGPIOx = LCD_GPIO_PORT;
	lcdPins.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_OUTPUT;
	lcdPins.GPIO_PinConfig.GPIO_PinOpType = GPIO_OP_TYPE_PP;
	lcdPins.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;
	lcdPins.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;

	lcdPins.GPIO_PinConfig.GPIO_PinNumber = LCD_GPIO_RS;
	GPIO_Init(&lcdPins);

	lcdPins.GPIO_PinConfig.GPIO_PinNumber = LCD_GPIO_RW;
	GPIO_Init(&lcdPins);

	lcdPins.GPIO_PinConfig.GPIO_PinNumber = LCD_GPIO_EN;
	GPIO_Init(&lcdPins);

	lcdPins.GPIO_PinConfig.GPIO_PinNumber = LCD_GPIO_D4;
	GPIO_Init(&lcdPins);

	lcdPins.GPIO_PinConfig.GPIO_PinNumber = LCD_GPIO_D5;
	GPIO_Init(&lcdPins);

	lcdPins.GPIO_PinConfig.GPIO_PinNumber = LCD_GPIO_D6;
	GPIO_Init(&lcdPins);

	lcdPins.GPIO_PinConfig.GPIO_PinNumber = LCD_GPIO_D7;
	GPIO_Init(&lcdPins);

	//Write 0 to all LCD Pins
	GPIO_WriteToOutputPin(LCD_GPIO_PORT, LCD_GPIO_RS, GPIO_PIN_RESET);
	GPIO_WriteToOutputPin(LCD_GPIO_PORT, LCD_GPIO_RW, GPIO_PIN_RESET);
	GPIO_WriteToOutputPin(LCD_GPIO_PORT, LCD_GPIO_EN, GPIO_PIN_RESET);
	GPIO_WriteToOutputPin(LCD_GPIO_PORT, LCD_GPIO_D4, GPIO_PIN_RESET);
	GPIO_WriteToOutputPin(LCD_GPIO_PORT, LCD_GPIO_D5, GPIO_PIN_RESET);
	GPIO_WriteToOutputPin(LCD_GPIO_PORT, LCD_GPIO_D6, GPIO_PIN_RESET);
	GPIO_WriteToOutputPin(LCD_GPIO_PORT, LCD_GPIO_D7, GPIO_PIN_RESET);
}

void delay_ms(uint32_t cnt)
{
	for(uint32_t i=0; i < (cnt * 1000); i++);
}

void delay_us(uint32_t cnt)
{
	for(uint32_t i=0; i < (cnt * 1); i++);
}
