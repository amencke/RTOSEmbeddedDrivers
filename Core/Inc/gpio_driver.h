/*
 * gpio_driver.h
 *
 *  Created on: Jan 1, 2024
 *      Author: arthurmencke
 */

#ifndef INC_GPIO_DRIVER_H_
#define INC_GPIO_DRIVER_H_

#include "stm32f407xx.h"
#include "macros.h"

/*
 * This is a Configuration structure for a GPIO pin
 */
typedef struct
{
	uint8_t GPIO_PinNumber;
	uint8_t GPIO_PinMode;			/*!< possible values from @GPIO_PIN_MODES >*/
	uint8_t GPIO_PinSpeed;			/*!< possible values from @GPIO_PIN_SPEED >*/
	uint8_t GPIO_PinPuPdControl;
	uint8_t GPIO_PinOPType;
	uint8_t GPIO_PinAltFunMode;
}GPIO_PinConfig_t;

/*
 * This is a Handle structure for a GPIO pin
 */

typedef struct
{
	GPIO_TypeDef *pGPIOx;       		/*!< This holds the base address of the GPIO port to which the pin belongs >*/
	GPIO_PinConfig_t GPIO_PinConfig;

}GPIO_Handle_t;

/*
 *  returns port code for given GPIOx base address
 */
/*
 * This macro returns a code( between 0 to 7) for a given GPIO base address(x)
 */
#define GPIO_BASEADDR_TO_CODE(x)      ( (x == GPIOA)?0:\
										(x == GPIOB)?1:\
										(x == GPIOC)?2:\
										(x == GPIOD)?3:\
								        (x == GPIOE)?4:\
								        (x == GPIOF)?5:\
								        (x == GPIOG)?6:\
								        (x == GPIOH)?7: \
								        (x == GPIOI)?8:0)

#define GPIO_PIN_NO_0  				0
#define GPIO_PIN_NO_1  				1
#define GPIO_PIN_NO_2  				2
#define GPIO_PIN_NO_3  				3
#define GPIO_PIN_NO_4  				4
#define GPIO_PIN_NO_5  				5
#define GPIO_PIN_NO_6  				6
#define GPIO_PIN_NO_7  				7
#define GPIO_PIN_NO_8  				8
#define GPIO_PIN_NO_9  				9
#define GPIO_PIN_NO_10  			10
#define GPIO_PIN_NO_11 				11
#define GPIO_PIN_NO_12  			12
#define GPIO_PIN_NO_13 				13
#define GPIO_PIN_NO_14 				14
#define GPIO_PIN_NO_15 				15

/*
 * GPIO pin possible modes
 */
#define GPIO_MODE_IN 		0
#define GPIO_MODE_OUT 		1
#define GPIO_MODE_ALTFN 	2
#define GPIO_MODE_ANALOG 	3
#define GPIO_MODE_IT_FT     4 // interrupt triggered on falling edge
#define GPIO_MODE_IT_RT     5 // interrupt triggered on rising edge
#define GPIO_MODE_IT_RFT    6 // interrupt triggered on rising and falling edge

/*
 * GPIO pin possible output types
 */
#define GPIO_OP_TYPE_PP   0
#define GPIO_OP_TYPE_OD   1


/*
 * GPIO pin possible output speeds
 */
#define GPIO_SPEED_LOW			0
#define GPIO_SPEED_MEDIUM		1
#define GPIO_SPEED_FAST			2
#define GPOI_SPEED_HIGH			3


/*
 * GPIO pin pull up AND pull down configuration macros
 */
#define GPIO_NO_PUPD   		0
#define GPIO_PIN_PU			1
#define GPIO_PIN_PD			2

/*
 * Peripheral Clock setup
 */
void GPIO_PeriClockControl(GPIO_TypeDef *pGPIOx, uint8_t toggle);

/*
 * Init and De-init
 */
void GPIO_Init(GPIO_Handle_t *pGPIOHandle);
void GPIO_DeInit(GPIO_TypeDef *pGPIOx);

#endif /* INC_GPIO_DRIVER_H_ */
