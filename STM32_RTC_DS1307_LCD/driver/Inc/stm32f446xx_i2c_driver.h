/*
 * stm32f446xx_i2c_driver.h
 *
 *  Created on: Aug 8, 2025
 *      Author: Sunil Sutar
 */

#ifndef INC_STM32F446XX_I2C_DRIVER_H_
#define INC_STM32F446XX_I2C_DRIVER_H_

#include "stm32f446xx.h"

/*
 * Configuration Structure for I2Cx Peripheral
 */
typedef struct
{
	uint32_t I2C_SCLSpeed;
	uint8_t  I2C_DeviceAddress;		//user give I2C_DeviceAddress value
	uint8_t  I2C_ACKControl;
	uint8_t  I2C_FMDutyCycle;
}I2C_Config_t;

/*
 *  Handle Structure for SPIx Peripheral
 */
typedef struct
{
	I2C_RegDef_t *pI2Cx;		/*!< This holds the base address of I2Cx (x=1,2,3) peripherals */
	I2C_Config_t I2CConfig;
}I2C_Handle_t;

/*
 * @I2C_SCLSpeed
 */
#define I2C_SCL_SPEED_SM		100000
#define I2C_SCL_SPEED_FM4K		400000
#define I2C_SCL_SPEED_FM2K		200000

/*
 * @I2C_ACKControl
 */
#define I2C_ACK_ENABLE			1
#define I2C_ACK_DISABLE			0

/*
 * @I2C_FMDutyCycle
 */
#define I2C_FM_DUTY_2			0
#define I2C_FM_DUTY_16_9		1


/*
 * I2C Init and DeInit
 */
#define I2C_FLAG_TXE	   (1 << I2C_SR1_TxE)
#define I2C_FLAG_RXNE	   (1 << I2C_SR1_RxNE)
#define I2C_FLAG_SB	 	   (1 << I2C_SR1_SB)
#define I2C_FLAG_ADDR	   (1 << I2C_SR1_ADDR)
#define I2C_FLAG_BTF	   (1 << I2C_SR1_BTF)

//I2C Repeated Start Condition
#define I2C_SR_DISABLE			RESET
#define I2C_SR_ENABLE			SET

/******************************************************************************************
 *								APIs supported by this driver
 *		 For more information about the APIs check the function definitions
 ******************************************************************************************/


/*
 * I2C Peripheral Clock Control
 */
void I2C_PeriClockControl(I2C_RegDef_t *pI2Cx, uint8_t EnorDi);

/*
 * SPI Init and DeInit
 */
void I2C_Init(I2C_Handle_t *pI2CHandle);
void I2C_DeInit(I2C_RegDef_t *pI2Cx);

/*
 * I2C Data Send and Receive
 */
void I2C_MasterSendData(I2C_Handle_t *pI2CHandle, uint8_t *pTxBuffer, uint32_t Len, uint8_t SlaveAddr, uint8_t Sr);
void I2C_MasterReceiveData(I2C_Handle_t *pI2CHandle, uint8_t *pRxBuffer, uint32_t Len, uint8_t SlaveAddr, uint8_t Sr);

/*
 * SPI IRQ Configuration and ISR Handling
 */
void I2C_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnorDi);
void I2C_IRQPriorityConfig(uint8_t IRQNumber, uint32_t IRQPriority);

/*
 * Other Peripheral Control APIs
 */
void I2C_PeripheralControl(I2C_RegDef_t *pI2Cx, uint8_t EnorDi);
uint8_t I2C_GetFlagStatus(I2C_RegDef_t *pI2Cx, uint32_t FlagName);
void I2C_ACKControl(I2C_RegDef_t *pI2Cx, uint8_t EnorDi);

#endif /* INC_STM32F446XX_I2C_DRIVER_H_ */
