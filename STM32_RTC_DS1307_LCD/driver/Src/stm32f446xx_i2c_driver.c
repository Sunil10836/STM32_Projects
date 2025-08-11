/*
 * stm32f446xx_i2c_driver.c
 *
 *  Created on: Aug 8, 2025
 *      Author: Sunil Sutar
 */

#include "stm32f446xx_i2c_driver.h"
#include "stm32f407xx_rcc_driver.h"

static void I2C_GenerateStartCondition(I2C_RegDef_t *pI2Cx);
static void I2C_GenerateStopCondition(I2C_RegDef_t *pI2Cx);
static void I2C_SendAddressWrite( I2C_RegDef_t *pI2Cx, uint8_t SlaveAddr);
static void I2C_SendAddressRead( I2C_RegDef_t *pI2Cx, uint8_t SlaveAddr);
static void I2C_ClearADDRFlag( I2C_RegDef_t *pI2Cx);

static void I2C_GenerateStartCondition(I2C_RegDef_t *pI2Cx)
{
	pI2Cx->CR1 |= (1 << I2C_CR1_START);
}

static void I2C_GenerateStopCondition(I2C_RegDef_t *pI2Cx)
{
	pI2Cx->CR1 |= (1 << I2C_CR1_STOP);
}

static void I2C_SendAddressWrite( I2C_RegDef_t *pI2Cx, uint8_t SlaveAddr)
{
	SlaveAddr = SlaveAddr << 1;	//7-bit Slave Address
	SlaveAddr &= ~(1);			//R/W=0
	pI2Cx->DR = SlaveAddr;	//SlaveAddr + R/W
}

static void I2C_SendAddressRead( I2C_RegDef_t *pI2Cx, uint8_t SlaveAddr)
{
	SlaveAddr = SlaveAddr << 1;	//7-bit Slave Address
	SlaveAddr |= (1);			//R/W=1
	pI2Cx->DR = SlaveAddr;	//SlaveAddr + R/W
}

static void I2C_ClearADDRFlag( I2C_RegDef_t *pI2Cx)
{
	uint32_t dummyRead = pI2Cx->SR1;
	dummyRead = pI2Cx->SR2;
	(void)dummyRead;

}

uint8_t I2C_GetFlagStatus(I2C_RegDef_t *pI2Cx, uint32_t FlagName)
{
	if(pI2Cx->SR1 & FlagName)
	{
		return FLAG_SET;
	}
	return FLAG_RESET;
}

void I2C_ACKControl(I2C_RegDef_t *pI2Cx, uint8_t EnorDi)
{
	if(EnorDi == I2C_ACK_ENABLE)
	{
		//enable ACK
		pI2Cx->CR1 |= (1 << I2C_CR1_ACK);
	}
	else
	{
		//disable ACK
		pI2Cx->CR1 &= ~(1 << I2C_CR1_ACK);
	}
}

/*********************************************************************
 * @fn      		  - I2C_PeriClockControl
 *
 * @brief             -
 *
 * @param[in]         - Pointer to SPI Handle Structure
 * @param[in]         - I2Cx Clock Enable or Disable
 * @param[in]         -
 *
 * @return            -
 *
 * @Note              -

 */
void I2C_PeriClockControl(I2C_RegDef_t *pI2Cx, uint8_t EnorDi)
{
	if(EnorDi == ENABLE)
	{
		if(pI2Cx == I2C1)
		{
			I2C1_PCLK_EN();
		}
		else if(pI2Cx == I2C2)
		{
			I2C2_PCLK_EN();
		}
		else if(pI2Cx == I2C3)
		{
			I2C3_PCLK_EN();
		}
	}
	else
	{
		if(pI2Cx == I2C1)
		{
			I2C1_PCLK_DI();
		}
		else if(pI2Cx == I2C2)
		{
			I2C2_PCLK_DI();
		}
		else if(pI2Cx == I2C3)
		{
			I2C3_PCLK_DI();
		}
	}
}

/*********************************************************************
 * @fn      		  - I2C_PeripheralControl
 *
 * @brief             -
 *
 * @param[in]         -
 * @param[in]         -
 * @param[in]         -
 *
 * @return            -
 *
 * @Note              -

 */
void I2C_PeripheralControl(I2C_RegDef_t *pI2Cx, uint8_t EnorDi)
{
	if(EnorDi == ENABLE)
	{
		pI2Cx->CR1 |= (1 << I2C_CR1_PE);
	}
	else
	{
		pI2Cx->CR1 &= ~(1 << I2C_CR1_PE);
	}
}

/*********************************************************************
 * @fn      		  - I2C_Init
 *
 * @brief             -
 *
 * @param[in]         -
 * @param[in]         -
 * @param[in]         -
 *
 * @return            -
 *
 * @Note              -

 */

void I2C_Init(I2C_Handle_t *pI2CHandle)
{
	uint32_t tempreg = 0;

	//Enable I2C Peripheral Clock
	I2C_PeriClockControl(pI2CHandle->pI2Cx, ENABLE);

	/******************************** Configuration of CR1******************************************/
	tempreg |= pI2CHandle->I2CConfig.I2C_ACKControl << I2C_CR1_ACK;

	/******************************** Configuration of CR2******************************************/
	//Configure FREQ field of CR2
	tempreg = 0;
	tempreg |= RCC_GetPCLK1Value()/1000000;		//16
	pI2CHandle->pI2Cx->CR2 = (tempreg & 0x3F);

	/******************************** Configuration of OAR1******************************************/
	tempreg = 0;
	tempreg |= pI2CHandle->I2CConfig.I2C_DeviceAddress << 1;
	tempreg |= (1 << 14);						//given in RM: 14 bit set to 1 by software
	pI2CHandle->pI2Cx->OAR1 = tempreg;

	/******************************** Configuration of CCR******************************************/
	//CCR Calculations
	uint16_t ccr_value;
	tempreg = 0;
	if(pI2CHandle->I2CConfig.I2C_SCLSpeed == I2C_SCL_SPEED_SM)
	{
		//standard mode
		ccr_value = RCC_GetPCLK1Value() / (2 * pI2CHandle->I2CConfig.I2C_SCLSpeed);
		tempreg |= ccr_value & 0xFFF;
	}
	else
	{
		//fast mode
		tempreg |= (1 << 15);	//fast mode
		tempreg |= (pI2CHandle->I2CConfig.I2C_FMDutyCycle << 14);
		if(pI2CHandle->I2CConfig.I2C_FMDutyCycle == I2C_FM_DUTY_2)
		{
			ccr_value = RCC_GetPCLK1Value() / (3 * pI2CHandle->I2CConfig.I2C_SCLSpeed);
		}
		else
		{
			ccr_value = RCC_GetPCLK1Value() / (25 * pI2CHandle->I2CConfig.I2C_SCLSpeed);
		}
		tempreg |= ccr_value & 0xFFF;
	}
	pI2CHandle->pI2Cx->CCR = tempreg;
}

/*********************************************************************
 * @fn      		  - I2C_MasterSendData
 *
 * @brief             -
 *
 * @param[in]         -
 * @param[in]         -
 * @param[in]         -
 *
 * @return            -
 *
 * @Note              -

 */

void I2C_MasterSendData(I2C_Handle_t *pI2CHandle, uint8_t *pTxBuffer, uint32_t Len, uint8_t SlaveAddr, uint8_t Sr)
{
	//1. Generate START Condition
	I2C_GenerateStartCondition(pI2CHandle->pI2Cx);

	//2. Confirm the START generatioj by checking SB flag in SR
	while(! (I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_FLAG_SB)));

	//SB is cleared by reading SR and writing to DR
	//3. send the address of slave with R/W bit set to 0
	I2C_SendAddressWrite(pI2CHandle->pI2Cx, SlaveAddr);

	//4. Confirm the Address is sent by checking ADDR Flag in SR1
	while(! (I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_FLAG_ADDR)) );

	//5. ADDR flag is cleared by SR1 followed by SR2
	I2C_ClearADDRFlag(pI2CHandle->pI2Cx);

	//6. Send data until length becomes 0
	while(Len > 0)
	{
		//wait until TxE flag is set
		while(! (I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_FLAG_TXE)));
		pI2CHandle->pI2Cx->DR = *pTxBuffer;
		pTxBuffer++;
		Len--;
	}

	//7. When Len becomes zero, wait for TXE=1 and BTF=1 before generating STOP Condition
	while(! (I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_FLAG_TXE)));

	while(! (I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_FLAG_BTF)));

	//8. Generate STOP Condition
	if(Sr == I2C_SR_DISABLE)
	{
		I2C_GenerateStopCondition(pI2CHandle->pI2Cx);
	}
}

void I2C_MasterReceiveData(I2C_Handle_t *pI2CHandle, uint8_t *pRxBuffer, uint32_t Len, uint8_t SlaveAddr, uint8_t Sr)
{
	//1. Generate START Condition
	I2C_GenerateStartCondition(pI2CHandle->pI2Cx);

	//2. Confirm the START generatioj by checking SB flag in SR
	while(! (I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_FLAG_SB)));

	//SB is cleared by reading SR and writing to DR
	//3. send the address of slave with R/W bit set to 0
	I2C_SendAddressRead(pI2CHandle->pI2Cx, SlaveAddr);

	//4. Confirm the Address is sent by checking ADDR Flag in SR1
	while(! (I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_FLAG_ADDR)) );

	//procedure to read only one byte
	if(Len == 1)
	{
		//Disable ACKing
		I2C_ACKControl(pI2CHandle->pI2Cx, I2C_ACK_DISABLE);

		//clear ADDR Flag
		I2C_ClearADDRFlag(pI2CHandle->pI2Cx);

		//wait until RxNE flag is set
		while(! (I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_FLAG_RXNE)));

		if(Sr == I2C_SR_DISABLE)
		{
			//generate stop condition
			I2C_GenerateStopCondition(pI2CHandle->pI2Cx);
		}

		//read the data from DR into buffer
		*pRxBuffer = pI2CHandle->pI2Cx->DR;
	}

	//procedure to read more than one byte
	if(Len > 1)
	{
		//clear ADDR Flag
		I2C_ClearADDRFlag(pI2CHandle->pI2Cx);

		//read data until Len becomes zero
		for(uint32_t i=Len; i > 0; i--)
		{
			//wait until RxNE flag is set
			while(! (I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_FLAG_RXNE)));

			if(i==2)	//if last 2 byte are remaining
			{
				//Clear ACK bit i.e. Disable ACK, ACK=0 i.e send NACK
				I2C_ACKControl(pI2CHandle->pI2Cx, I2C_ACK_DISABLE);

				if(Sr == I2C_SR_DISABLE)
				{
					//Generate STOP Condition
					I2C_GenerateStopCondition(pI2CHandle->pI2Cx);
				}
			}

			//read the data from DR into buffer
			*pRxBuffer = pI2CHandle->pI2Cx->DR;

			//increment buffer address
			pRxBuffer++;
		}
	}

	if(pI2CHandle->I2CConfig.I2C_ACKControl == I2C_ACK_ENABLE)
	{
		//Re-Enable ACKing
		I2C_ACKControl(pI2CHandle->pI2Cx, I2C_ACK_ENABLE);
	}
}


