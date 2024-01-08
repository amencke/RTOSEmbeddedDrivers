/*
 * gpio_driver.c
 *
 *  Created on: Jan 1, 2024
 *      Author: arthurmencke
 */

#include "gpio_driver.h"

/**
 * \brief
 *
 * \param pGPIOx base address of some GPIO peripheral
 * \param toggle ENABLE or DISABLE macro
 */
void GPIO_PeriClockControl(GPIO_TypeDef *pGPIOx, uint8_t toggle)
{
	if(toggle == ENABLE)
	{
		if (pGPIOx == GPIOA)
		{
			RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;
		}
		else if (pGPIOx == GPIOB)
		{
			RCC->AHB1ENR |= RCC_AHB1ENR_GPIOBEN;
		}
		else if (pGPIOx == GPIOC)
		{
			RCC->AHB1ENR |= RCC_AHB1ENR_GPIOCEN;
		}
		else if (pGPIOx == GPIOD)
		{
			RCC->AHB1ENR |= RCC_AHB1ENR_GPIODEN;
		}
		else if (pGPIOx == GPIOE)
		{
			RCC->AHB1ENR |= RCC_AHB1ENR_GPIOEEN;
		}
		else if (pGPIOx == GPIOF)
		{
			RCC->AHB1ENR |= RCC_AHB1ENR_GPIOFEN;
		}
		else if (pGPIOx == GPIOG)
		{
			RCC->AHB1ENR |= RCC_AHB1ENR_GPIOGEN;
		}
		else if (pGPIOx == GPIOH)
		{
			RCC->AHB1ENR |= RCC_AHB1ENR_GPIOHEN;
		}
		else if (pGPIOx == GPIOI)
		{
			RCC->AHB1ENR |= RCC_AHB1ENR_GPIOIEN;
		}
	}
	else
	{
		if (pGPIOx == GPIOA)
		{
			RCC->AHB1ENR &= ~RCC_AHB1ENR_GPIOAEN;
		}
		else if (pGPIOx == GPIOB)
		{
			RCC->AHB1ENR &= ~RCC_AHB1ENR_GPIOBEN;
		}
		else if (pGPIOx == GPIOC)
		{
			RCC->AHB1ENR &= ~RCC_AHB1ENR_GPIOCEN;
		}
		else if (pGPIOx == GPIOD)
		{
			RCC->AHB1ENR &= ~RCC_AHB1ENR_GPIODEN;
		}
		else if (pGPIOx == GPIOE)
		{
			RCC->AHB1ENR &= ~RCC_AHB1ENR_GPIOEEN;
		}
		else if (pGPIOx == GPIOF)
		{
			RCC->AHB1ENR &= ~RCC_AHB1ENR_GPIOFEN;
		}
		else if (pGPIOx == GPIOG)
		{
			RCC->AHB1ENR &= ~RCC_AHB1ENR_GPIOGEN;
		}
		else if (pGPIOx == GPIOH)
		{
			RCC->AHB1ENR &= ~RCC_AHB1ENR_GPIOHEN;
		}
		else if (pGPIOx == GPIOI)
		{
			RCC->AHB1ENR &= ~RCC_AHB1ENR_GPIOIEN;
		}
	}
}

/**
 * \brief Initialize the GPIO with the requested config
 *
 * \param pGPIOHandle
 */
void GPIO_Init(GPIO_Handle_t *pGPIOHandle)
{
	// Note: in this case, there is one or more register for each config item which configures all of the pins for each GPIO A-I
	 uint32_t cfgRegTmp = 0x00000000;

	 // Enable the peripheral clock

	 GPIO_PeriClockControl(pGPIOHandle->pGPIOx, ENABLE);

	// 1. Configure the mode of GPIO pin

	if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode <= GPIO_MODE_ANALOG_)
	{
		// Non interrupt mode
		cfgRegTmp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinMode << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber ) );
		pGPIOHandle->pGPIOx->MODER &= ~( 0x3 << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber)); //clearing
		pGPIOHandle->pGPIOx->MODER |= cfgRegTmp; //setting

	}
	else
	{
		// Interrupt mode
		if (pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_IT_FT )
		{
			// 1. configure the FTSR
			EXTI->FTSR |= ( 1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
			// 2. clear the corresponding RTSR bit
			EXTI->RTSR &= ~( 1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);

		}
		else if (pGPIOHandle->GPIO_PinConfig.GPIO_PinMode ==GPIO_MODE_IT_RT )
		{
			// 1. configure the RTSR
			EXTI->RTSR |= ( 1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
			// 2. clear the corresponding RTSR bit
			EXTI->FTSR &= ~( 1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);

		}
		else if (pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_IT_RFT )
		{
			//1. configure both FTSR and RTSR
			EXTI->RTSR |= ( 1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
			EXTI->FTSR |= ( 1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
		}

		//2. configure the GPIO port selection in SYSCFG_EXTICR

		// Each 32 bit register configures 4 EXTIs in the lest significant 16 bits
		uint8_t CRRegOffset = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber / 4;
		uint8_t CRPinOffset = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber % 4;
		uint8_t portcode = GPIO_BASEADDR_TO_CODE(pGPIOHandle->pGPIOx);
		// Enabled peripheral clock for SYSCFG peripheral if it's not already enabled
		if (!(RCC->APB2ENR &= ~(1 << RCC_APB2ENR_SYSCFGEN_Pos)))
		{
			RCC->APB2ENR |= 1 << RCC_APB2ENR_SYSCFGEN_Pos;
		}
		SYSCFG->EXTICR[CRRegOffset] = portcode << ( CRPinOffset * 4); // EXTICR is of type uint32_t *

		// 3. enable the exti interrupt delivery using IMR
		EXTI->IMR |= 1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber;
	}

	//2. configure the speed. The multiplication by 2 here is simply because each pin requires 2 bits: pin 0 uses bits 0 and 1, etc
	cfgRegTmp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinSpeed << ( 2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber) );
	pGPIOHandle->pGPIOx->OSPEEDR &= ~( 0x3 << ( 2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber)); //clearing
	pGPIOHandle->pGPIOx->OSPEEDR |= cfgRegTmp;

	//3. configure the pupd settings
	cfgRegTmp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinPuPdControl << ( 2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber) );
	pGPIOHandle->pGPIOx->PUPDR &= ~( 0x3 << ( 2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber)); //clearing
	pGPIOHandle->pGPIOx->PUPDR |= cfgRegTmp;


	//4. configure the optype
	cfgRegTmp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinOPType << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber );
	pGPIOHandle->pGPIOx->OTYPER &= ~( 0x1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber); //clearing
	pGPIOHandle->pGPIOx->OTYPER |= cfgRegTmp;

	//5. configure the alt functionality
	if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_ALTFN)
	{
		//configure the alt function registers.
		uint8_t CRRegOffset, CRPinOffset;

		CRRegOffset = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber / 8;
		CRPinOffset = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber  % 8;
		pGPIOHandle->pGPIOx->AFR[CRRegOffset] &= ~(0xF << ( 4 * CRPinOffset ) ); //clearing
		pGPIOHandle->pGPIOx->AFR[CRRegOffset] |= (pGPIOHandle->GPIO_PinConfig.GPIO_PinAltFunMode << ( 4 * CRPinOffset ) );
	}

}
