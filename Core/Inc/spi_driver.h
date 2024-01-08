#ifndef _SPI_DRIVER_H_
#define _SPI_DRIVER_H_

#include<stddef.h>

#include "stm32f407xx.h"
#include "macros.h"

#define SPI_CR_RESET 	0x0000

typedef struct
{
	// See TRM page 916
	uint8_t DeviceMode;
	uint8_t BusConfig;
	uint8_t SCLKDiv;			// serial clock divisor
	uint8_t DFF; 				// data frame format
	uint8_t CPOL;				// clock polarity
	uint8_t CPHA;				// clock phase
	uint8_t SSM;				// software slave mgmt
	uint8_t SSI;				//
} SPI_Config_t;

typedef struct
{
	SPI_TypeDef 	*pSPIx;		// base address of SPI peripheral
	SPI_Config_t 	SPIConfig;
	uint8_t 		*pTxBuffer; /* !< To store the app. Tx buffer address > */
	uint8_t 		*pRxBuffer;	/* !< To store the app. Rx buffer address > */
	uint32_t 		TxLen;		/* !< To store Tx len > */
	uint32_t 		RxLen;		/* !< To store Tx len > */
	uint8_t 		TxState;	/* !< To store Tx state > */
	uint8_t 		RxState;	/* !< To store Rx state > */
} SPI_Handle_t;


/*
 * SPI application states
 */
#define SPI_READY 					0
#define SPI_BUSY_IN_RX 				1
#define SPI_BUSY_IN_TX 				2

/*
 * Possible SPI Application events
 */
#define SPI_EVENT_TX_CMPLT   1
#define SPI_EVENT_RX_CMPLT   2
#define SPI_EVENT_OVR_ERR    3
#define SPI_EVENT_CRC_ERR    4

/*
 * SPI device mode
 */
#define SPI_MASTER_MODE 1
#define SPI_SLAVE_MODE 	0

/*
 * SPI bus config
 */
#define SPI_BUS_CONFIG_FD	1 // full duplex
#define SPI_BUS_CONFIG_HD	2 // half duplex


/*
 * SPI clock speed divisors (TRM page 917)
 */
#define SPI_SCLK_SPD_DIV2	0
#define SPI_SCLK_SPD_DIV4	1
#define SPI_SCLK_SPD_DIV8	2
#define SPI_SCLK_SPD_DIV16	3
#define SPI_SCLK_SPD_DIV32	4
#define SPI_SCLK_SPD_DIV64	5
#define SPI_SCLK_SPD_DIV128	6
#define SPI_SCLK_SPD_DIV256	7

/*
 * SPI data frame format
 */
#define SPI_DFF_8BIT	0
#define SPI_DFF_16BIT 	16

/*
 * Clock polarity
 */
#define SPI_CPOL_HIGH	1
#define SPI_CPOL_LOW	0

/*
 * Clock phase (1st/2nd clock transition)
 */
#define SPI_CPHA_FIRST	0
#define SPI_CPHA_SECOND	1

/*
 * SPI software slave management
 */
#define SPI_SSM_EN 	1
#define SPI_SSM_DI	0

/*
 * Peripheral clock setup
 */
void SPI_ClockControl(SPI_TypeDef *pSPIx, uint8_t toggle);

/*
 * Init and de-init
 */
void SPI_Init(SPI_Handle_t *pSPIHandle);
void SPI_DeInit(SPI_Handle_t *pSPIHandle);

/*
 * Data send and receive
 */
void SPI_SendData(SPI_TypeDef *pSPIx, uint8_t *pTxBuffer, uint32_t len);
void SPI_RecvData(SPI_TypeDef *pSPIx, uint8_t *pRxBuffer, uint32_t len);

uint8_t SPI_SendDataIT(SPI_Handle_t *pSPIHandle, uint8_t *pTxBuffer, uint32_t len);
uint8_t SPI_RecvDataIT(SPI_Handle_t *pSPIHandle, uint8_t *pRxBuffer, uint32_t len);

/*
 * IRQ config and ISR handling
 */
void SPI_IRQConfig(uint8_t IRQNum, uint8_t toggle);
void SPI_IRQPrio(uint8_t IRQNum, uint32_t prio);
void SPI_IRQHandling(SPI_Handle_t *pHandle);

/**
 * Other SPI peripheral APIs
 */
void SPI_PeripheralToggle(SPI_TypeDef *, uint8_t toggle);

void SPI_ClearOVRFlag(SPI_TypeDef *pSPIx);
void SPI_CloseTransmisson(SPI_Handle_t *pSPIHandle);
void SPI_CloseReception(SPI_Handle_t *pSPIHandle);

/*
 * Application callback
 */
void SPI_ApplicationEventCallback(SPI_Handle_t *pSPIHandle,uint8_t AppEv);


#endif // _SPI_DRIVER_H_
