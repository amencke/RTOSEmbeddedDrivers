/*
 * spi_driver.c
 *
 *  Created on: Jan 1, 2024
 *      Author: arthurmencke
 */
#include "spi_driver.h"

static void  spi_txe_interrupt_handle(SPI_Handle_t *pSPIHandle);
static void  spi_rxne_interrupt_handle(SPI_Handle_t *pSPIHandle);
static void  spi_ovr_err_interrupt_handle(SPI_Handle_t *pSPIHandle);

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

	SPI_ClockControl(pSPIHandle->pSPIx, ENABLE);

	uint32_t cfgRegTmp = 0x0000;

	cfgRegTmp |= pSPIHandle->SPIConfig.DeviceMode << SPI_CR1_MSTR_Pos;
	// only full duplex is supported so ignore pSPIHandle->SPIConfig.BusConfig
	cfgRegTmp &= ~(1 << SPI_CR1_BIDIMODE_Pos);
	cfgRegTmp |= pSPIHandle->SPIConfig.SCLKDiv << SPI_CR1_BR_Pos;
	cfgRegTmp |= pSPIHandle->SPIConfig.DFF << SPI_CR1_DFF_Pos;
	cfgRegTmp |= pSPIHandle->SPIConfig.SSM << SPI_CR1_SSM_Pos;
	cfgRegTmp |= pSPIHandle->SPIConfig.SSI << SPI_CR1_SSI_Pos;
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
 * \brief Enable or disable the provided SPI peripheral
 *
 * \param pSPIx	base address of SPI peripheral
 * \param toggle enable or disable
 */
void SPI_PeripheralToggle(SPI_TypeDef *pSPIx, uint8_t toggle)
{
	if (toggle == ENABLE)
	{
		pSPIx->CR1 |= (1 << SPI_CR1_SPE_Pos);
	}
	else
	{
		pSPIx->CR1 &= ~(1 << SPI_CR1_SPE_Pos);
	}
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
			pSPIx->DR = *pTxBuffer;
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


/**
 * \brief Blocking SPI send data API
 *
 * \param pSPIx			SPI peripheral base address
 * \param pRxBuffer		Buffer to store received data
 * \param len			Length of data in bytes
 */
void SPI_RecvData(SPI_TypeDef *pSPIx, uint8_t *pRxBuffer, uint32_t len)
{
	while (len > 0)
	{
		while (!(pSPIx->SR  & (1 << SPI_SR_RXNE_Pos)));

		if (!(pSPIx->CR1 & (1 << SPI_CR1_DFF_Pos)))
		{
			// 8 bit data frames
			*pRxBuffer = pSPIx->DR;
			pRxBuffer++;
			len--;
		}
		else
		{
			// 16 bit data frames
			pSPIx->DR = *((uint16_t *)pRxBuffer);
			pRxBuffer += 2;
			len -= 2;
		}
	}
	uint8_t x;
	(void)x;
}

/**
 * \brief Nonblocking (interrupt based) SPI send data API
 *
 * \param pSPIHandle	SPI handle structure
 * \param pRxBuffer		Buffer containing data to send
 * \param len			Length of data in bytes
 */
uint8_t SPI_SendDataIT(SPI_Handle_t *pSPIHandle, uint8_t *pTxBuffer, uint32_t len)
{
	uint8_t state = pSPIHandle->TxState;

	if(state != SPI_BUSY_IN_TX)
	{
		//1 . Save the Tx buffer address and Len information in some global variables
		pSPIHandle->pTxBuffer = pTxBuffer;
		pSPIHandle->TxLen = len;
		//2.  Mark the SPI state as busy in transmission so that
		//    no other code can take over same SPI peripheral until transmission is over
		pSPIHandle->TxState = SPI_BUSY_IN_TX;

		//3. Enable the TXEIE control bit to get interrupt whenever TXE flag is set in SR
		pSPIHandle->pSPIx->CR2 |= ( 1 << SPI_CR2_TXEIE_Pos );

	}

	return state;
}

/**
 * \brief Nonblocking (interrupt based) SPI receive data API
 *
 * \param pSPIHandle	SPI handle structure
 * \param pRxBuffer		Buffer to store received data
 * \param len			Length of data in bytes
 */
uint8_t SPI_RecvDataIT(SPI_Handle_t *pSPIHandle, uint8_t *pRxBuffer, uint32_t len)
{
	uint8_t state = pSPIHandle->RxState;

	if(state != SPI_BUSY_IN_RX)
	{
		//1 . Save the Rx buffer address and Len information in some global variables
		pSPIHandle->pRxBuffer = pRxBuffer;
		pSPIHandle->RxLen = len;
		//2.  Mark the SPI state as busy in reception so that
		//    no other code can take over same SPI peripheral until reception is over
		pSPIHandle->RxState = SPI_BUSY_IN_RX;

		//3. Enable the RXNEIE control bit to get interrupt whenever RXNEIE flag is set in SR
		pSPIHandle->pSPIx->CR2 |= ( 1 << SPI_CR2_RXNEIE_Pos );

	}

	return state;

}

/**
 *
 */
void SPI_IRQHandling(SPI_Handle_t *pHandle)
{
	__NVIC_DisableIRQ(SPI2_IRQn); // do not allow interrupts during interrupt handling
	uint8_t temp1 , temp2;
	//first lets check for TXE
	temp1 = pHandle->pSPIx->SR & ( 1 << SPI_SR_TXE_Pos);
	temp2 = pHandle->pSPIx->CR2 & ( 1 << SPI_CR2_TXEIE_Pos);

	if( temp1 && temp2)
	{
		//handle TXE
		spi_txe_interrupt_handle(pHandle);
	}

	// check for RXNE
	temp1 = pHandle->pSPIx->SR & ( 1 << SPI_SR_RXNE_Pos);
	temp2 = pHandle->pSPIx->CR2 & ( 1 << SPI_CR2_RXNEIE_Pos);

	if( temp1 && temp2)
	{
		//handle RXNE
		spi_rxne_interrupt_handle(pHandle);
	}

	// check for ovr flag
	temp1 = pHandle->pSPIx->SR & ( 1 << SPI_SR_OVR_Pos);
	temp2 = pHandle->pSPIx->CR2 & ( 1 << SPI_CR2_ERRIE_Pos);

	if( temp1 && temp2)
	{
		//handle ovr error
		spi_ovr_err_interrupt_handle(pHandle);
	}
	__NVIC_EnableIRQ(SPI2_IRQn);
}

//some helper function implementations

static void  spi_txe_interrupt_handle(SPI_Handle_t *pSPIHandle)
{
	// check the DFF bit in CR1
	if( (pSPIHandle->pSPIx->CR1 & ( 1 << SPI_CR1_DFF_Pos) ) )
	{
		//16 bit DFF
		//1. load the data in to the DR
		pSPIHandle->pSPIx->DR =   *((uint16_t*)pSPIHandle->pTxBuffer);
		pSPIHandle->TxLen -= 2;
		pSPIHandle->pTxBuffer += 2;
	}
	else
	{
		//8 bit DFF
		pSPIHandle->pSPIx->DR =   *pSPIHandle->pTxBuffer;
		pSPIHandle->TxLen--;
		pSPIHandle->pTxBuffer++;
	}

	if(! pSPIHandle->TxLen)
	{
		//TxLen is zero , so close the spi transmission and inform the application that
		//TX is over.

		//this prevents interrupts from setting up of TXE flag
		SPI_CloseTransmisson(pSPIHandle);
		SPI_ApplicationEventCallback(pSPIHandle,SPI_EVENT_TX_CMPLT);
	}

}


static void  spi_rxne_interrupt_handle(SPI_Handle_t *pSPIHandle)
{
	//do rxing as per the dff
	if(pSPIHandle->pSPIx->CR1 & ( 1 << SPI_CR1_DFF_Pos))
	{
		//16 bit
		*((uint16_t*)pSPIHandle->pRxBuffer) = (uint16_t) pSPIHandle->pSPIx->DR;
		pSPIHandle->RxLen -= 2;
//		pSPIHandle->pRxBuffer += 2;

	}
	else
	{
		//8 bit
		*(pSPIHandle->pRxBuffer) = (uint8_t) pSPIHandle->pSPIx->DR;
		pSPIHandle->RxLen--;
//		pSPIHandle->pRxBuffer++;
	}

	if(! pSPIHandle->RxLen)
	{
		//reception is complete
		SPI_CloseReception(pSPIHandle);
		SPI_ApplicationEventCallback(pSPIHandle,SPI_EVENT_RX_CMPLT);
	}
}

static void  spi_ovr_err_interrupt_handle(SPI_Handle_t *pSPIHandle)
{

	uint8_t temp;
	//1. clear the ovr flag
	if(pSPIHandle->TxState != SPI_BUSY_IN_TX)
	{
		temp = pSPIHandle->pSPIx->DR;
		temp = pSPIHandle->pSPIx->SR;
	}
	(void)temp;
	//2. inform the application
	SPI_ApplicationEventCallback(pSPIHandle,SPI_EVENT_OVR_ERR);

}

void SPI_CloseTransmisson(SPI_Handle_t *pSPIHandle)
{
	pSPIHandle->pSPIx->CR2 &= ~( 1 << SPI_CR2_TXEIE_Pos);
	pSPIHandle->pTxBuffer = NULL;
	pSPIHandle->TxLen = SPI_TX_MSG_LEN_BYTES;
	pSPIHandle->TxState = SPI_READY;

}

void SPI_CloseReception(SPI_Handle_t *pSPIHandle)
{
//	pSPIHandle->pSPIx->CR2 &= ~( 1 << SPI_CR2_RXNEIE_Pos);
//	pSPIHandle->pRxBuffer = NULL;
	pSPIHandle->RxLen = SPI_RX_MSG_LEN_BYTES;
	pSPIHandle->RxState = SPI_READY;

}

void SPI_ClearOVRFlag(SPI_TypeDef *pSPIx)
{
	uint8_t temp;
	temp = pSPIx->DR;
	temp = pSPIx->SR;
	(void)temp;

}

__attribute__((weak)) void SPI_ApplicationEventCallback(SPI_Handle_t *pSPIHandle, uint8_t AppEv)
{

	//This is a weak implementation . the user application may override this function.
}

