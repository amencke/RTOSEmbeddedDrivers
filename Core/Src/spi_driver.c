/*
 * spi_driver.c
 *
 *  Created on: Jan 1, 2024
 *      Author: arthurmencke
 */
#include "spi_driver.h"

/**
 * \brief enabled/disable the clock for the provided SPI peripheral
 *
 * \param pSPIx		base address of a SPI peripheral
 * \param toggle	ENABLE of DISABLE macro
 */
void SPI_ClockControl(SPI_TypeDef *pSPIx, uint8_t toggle)
{
	if (toggle == ENABLE)
	{
		if (pSPIx == SPI1)
		{
			RCC->APB2ENR |= RCC_APB2ENR_SPI1EN;
		}
		else if (pSPIx == SPI2)
		{
			RCC->APB1ENR |= RCC_APB1ENR_SPI2EN;
		}
		else if (pSPIx == SPI3)
		{
			RCC->APB1ENR |= RCC_APB1ENR_SPI3EN;
		}
	}
	else if (toggle == DISABLE)
	{
		if (pSPIx == SPI1)
		{
			RCC->APB2ENR &= ~RCC_APB2ENR_SPI1EN;
		}
		else if (pSPIx == SPI2)
		{
			RCC->APB1ENR &= ~RCC_APB1ENR_SPI2EN;
		}
		else if (pSPIx == SPI3)
		{
			RCC->APB1ENR &= ~RCC_APB1ENR_SPI3EN;
		}
	}
}

/**
 * \brief Initialize a SPI peripheral with the provided config
 *
 * \param pSPIHandle
 */
void SPI_Init(SPI_Handle_t *pSPIHandle)
{
	uint32_t cfgRegTmp = 0x0000;

	cfgRegTmp |= pSPIHandle->SPIConfig.DeviceMode << SPI_CR1_MSTR_Pos;
	// only full duplex is supported so ignore pSPIHandle->SPIConfig.BusConfig
	cfgRegTmp &= ~(1 << SPI_CR1_BIDIMODE_Pos);
	cfgRegTmp |= pSPIHandle->SPIConfig.SCLKDiv << SPI_CR1_BR_Pos;
	cfgRegTmp |= pSPIHandle->SPIConfig.DFF << SPI_CR1_DFF_Pos;
	cfgRegTmp |= pSPIHandle->SPIConfig.CPOL << SPI_CR1_CPOL_Pos;
	cfgRegTmp |= pSPIHandle->SPIConfig.CPHA << SPI_CR1_CPHA_Pos;

	pSPIHandle->pSPIx->CR1 = cfgRegTmp;


}

/**
 * \brief Reset the provided SPI peripheral
 *
 * \param pSPIHandle
 */
void SPI_DeInit(SPI_Handle_t *pSPIHandle)
{
	pSPIHandle->pSPIx->CR1 = SPI_CR_RESET;
	pSPIHandle->pSPIx->CR2 = SPI_CR_RESET;
}

/**
 * \brief Blocking SPI send data API
 *
 * \param pSPIx			SPI peripheral base address
 * \param pTxBuffer		Data to send
 * \param len			Length of data in bytes
 */
void SPI_SendData(SPI_TypeDef *pSPIx, uint8_t *pTxBuffer, uint32_t len)
{
	while (len > 0)
	{
		while (!(pSPIx->SR  & (1 << SPI_SR_TXE_Pos)));

		if (!(pSPIx->CR1 & (1 << SPI_CR1_DFF_Pos)))
		{
			// 8 bit data frames
			pSPIx->DR = *((uint16_t *)pTxBuffer);
			pTxBuffer++;
			len--;
		}
		else
		{
			// 16 bit data frames
			pSPIx->DR = *((uint16_t *)pTxBuffer);
			pTxBuffer += 2;
			len -= 2;
		}
	}
}

void SPI_RecvData(SPI_TypeDef *pSPIx, uint8_t *pRxBuffer, uint32_t len);

/*
 * IRQ config and ISR handling
 */
void SPI_IRQConfig(uint8_t IRQNum, uint8_t toggle);
void SPI_IRQPrio(uint8_t IRQNum, uint32_t prio);
void SPI_IRQHandling(SPI_Handle_t *pHandle);

