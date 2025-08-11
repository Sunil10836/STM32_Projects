/*
 * stm32f446xx_gpio_driver.h
 *
 *  Created on: Jul 22, 2025
 *      Author: Sunil Sutar
 */

#ifndef INC_STM32F446XX_GPIO_DRIVER_H_
#define INC_STM32F446XX_GPIO_DRIVER_H_

#include "stm32f446xx.h"

/*
 * This is Configuration Structure for GPIO Pin
 */
typedef struct
{
	uint8_t GPIO_PinNumber;				/*!< Possible Values from @GPIO_PIN_NO >*/
	uint8_t GPIO_PinMode;				/*!< Possible Values from @GPIO_PIN_MODES >*/
	uint8_t GPIO_PinSpeed;				/*!< Possible Values from @GPIO_PIN_SPEED >*/
	uint8_t GPIO_PinPuPdControl;		/*!< Possible Values from @GPIO_PIN_PUPD >*/
	uint8_t GPIO_PinOpType;				/*!< Possible Values from @GPIO_OP_TYPE >*/
	uint8_t GPIO_PinAltFunMode;
}GPIO_PinConfig_t;

/*
 * This is Handle Structure for GPIO Pin
 */
typedef struct
{
	GPIO_RegDef_t *pGPIOx;				/*!< This holds base address of GPIO port >*/
	GPIO_PinConfig_t GPIO_PinConfig;	/*!< This holds GPIO pin configuration settings >*/

}GPIO_Handle_t;


/*
 * @GPIO_PIN_NO
 * GPIO Pin Numbers
 */
#define GPIO_PIN_0				0
#define GPIO_PIN_1				1
#define GPIO_PIN_2				2
#define GPIO_PIN_3				3
#define GPIO_PIN_4				4
#define GPIO_PIN_5				5
#define GPIO_PIN_6				6
#define GPIO_PIN_7				7
#define GPIO_PIN_8				8
#define GPIO_PIN_9				9
#define GPIO_PIN_10				10
#define GPIO_PIN_11				11
#define GPIO_PIN_12				12
#define GPIO_PIN_13				13
#define GPIO_PIN_14				14
#define GPIO_PIN_15				15

/*
 * @GPIO_PIN_MODES
 * GPIO Pin Possible Modes
 */
#define	GPIO_MODE_INPUT			0
#define	GPIO_MODE_OUTPUT		1
#define	GPIO_MODE_ALTFN			2
#define	GPIO_MODE_ANALOG		3
#define GPIO_MODE_IT_FT			4
#define GPIO_MODE_IT_RT			5
#define GPIO_MODE_IT_RFT		6

/*
 * @GPIO_OP_TYPE
 * GPIO Possible Output Types
 */
#define GPIO_OP_TYPE_PP			0
#define GPIO_OP_TYPE_OD			1

/*
 * @GPIO_PIN_SPEED
 * GPIO Sppeed
 */
#define GPIO_SPEED_LOW			0
#define GPIO_SPEED_MEDIUM		1
#define GPIO_SPEED_FAST			2
#define GPIO_SPEED_HIGH			3

/*
 * @GPIO_PIN_PUPD
 * GPIO PuPd Configuration macros
 */
#define GPIO_NO_PUPD			0
#define GPIO_PIN_PU				1
#define GPIO_PIN_PD				2


/************************ GPIO Driver APIs ************************************/

/*
 * GPIO Peripheral Clock Control
 */
void GPIO_PeriClockControl(GPIO_RegDef_t *pGPIOx, uint8_t EnorDi);

/*
 * GPIO Init and DeInit
 */
void GPIO_Init(GPIO_Handle_t *pGPIOHandle);
void GPIO_DeInit(GPIO_RegDef_t *pGPIOx);		//Check AHBx_RSTR Register

/*
 * GPIO Data Read and Write
 */
uint8_t GPIO_ReadFromInputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber);
uint16_t GPIO_ReadFromInputPort(GPIO_RegDef_t *pGPIOx);
void GPIO_WriteToOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber, uint8_t Value);
void GPIO_WriteToOutputPort(GPIO_RegDef_t *pGPIOx, uint8_t Value);
void GPIO_ToggleOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber);

/*
 * GPIO IRQ Configuration and ISR Handling
 */
void GPIO_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnorDi);
void GPIO_IRQPriorityConfig(uint8_t IRQNumber, uint32_t IRQPriority);
void GPIO_IRQHanding(uint8_t PinNumber);




#endif /* INC_STM32F446XX_GPIO_DRIVER_H_ */
