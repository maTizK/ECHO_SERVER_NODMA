#include <stm32f4xx.h>
#include <stm32f4xx_spi.h>
#include <stm32f4xx_dma.h>
#include <stm32f4xx_rcc.h>
#include <stm32f4xx_gpio.h>
#include "spi.h"

	
#define RESET_LOW()	GPIOA->BSRRL |= GPIO_Pin_3;
#define RESET_HIGH()    GPIOA->BSRRH |= GPIO_Pin_3;
#define CSON()		GPIOA->BSRRH |= GPIO_Pin_4; // chip select  
#define CSOFF()		GPIOA->BSRRL |= GPIO_Pin_4; // chip select  

#define MAX_BUFFER_LENGTH 2000
/* ----------------------------------------------------------*/
/* --    this function initializes the SPI1 peripheral     --*/
uint8_t bufferTX[MAX_BUFFER_LENGTH];
uint8_t bufferRX[MAX_BUFFER_LENGTH];
int bufferIndex = 0; 
int datalength = MAX_BUFFER_LENGTH;
void init_SPI1(void){
	
	GPIO_InitTypeDef GPIO_InitStruct;
	SPI_InitTypeDef SPI_InitStruct;
	DMA_InitTypeDef DMA_InitStruct;
	NVIC_InitTypeDef NVIC_InitStruct;

	
	
	// enable clock for used IO pins
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
	
	/* configure pins used by SPI1
	 * PA5 = SCK
	 * PA6 = MISO
	 * PA7 = MOSI
	 */
	// MOSI 
	GPIO_InitStruct.GPIO_Pin = SPIx_MOSI_PIN ;
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStruct.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_DOWN;
	GPIO_Init(GPIOA, &GPIO_InitStruct);
	// SCK
	GPIO_InitStruct.GPIO_Pin = SPIx_SCK_PIN;
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStruct.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_DOWN;
	GPIO_Init(GPIOA, &GPIO_InitStruct);
	// MISO 
	GPIO_InitStruct.GPIO_Pin = SPIx_MISO_PIN;
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStruct.GPIO_OType = GPIO_OType_OD;
	GPIO_InitStruct.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_Init(GPIOA, &GPIO_InitStruct);
	
	// connect SPI1 pins to SPI alternate function
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource5, GPIO_AF_SPI1);
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource6, GPIO_AF_SPI1);
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource7, GPIO_AF_SPI1);

	
	/* Configure the chip select pin
	   in this case we will use PA4 */
	GPIO_InitStruct.GPIO_Pin = GPIO_Pin_4;
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStruct.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_Init(GPIOA, &GPIO_InitStruct);
	
	GPIOA->BSRRL |= GPIO_Pin_4; // set PA4 high

	/* Configure the hard reset pin
	   in this case we will use PA3 */
	GPIO_InitStruct.GPIO_Pin = GPIO_Pin_3;
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStruct.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_Init(GPIOA, &GPIO_InitStruct);
	
	GPIOA->BSRRL |= GPIO_Pin_3; // set PA3 high
	
	
	 // set PA3 high
	

	// enable peripheral clock
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_SPI1, ENABLE);
	
	/* configure SPI1 in Mode 0 
	 * CPOL = 0 --> clock is low when idle
	 * CPHA = 0 --> data is sampled at the first edge
	 */
	SPI_InitStruct.SPI_Direction = SPI_Direction_2Lines_FullDuplex; // set to full duplex mode, seperate MOSI and MISO lines
	SPI_InitStruct.SPI_Mode = SPI_Mode_Master;     // transmit in master mode, NSS pin has to be always high
	SPI_InitStruct.SPI_DataSize = SPI_DataSize_8b; // one packet of data is 8 bits wide
	SPI_InitStruct.SPI_CPOL = SPI_CPOL_Low;        // clock is low when idle
	SPI_InitStruct.SPI_CPHA = SPI_CPHA_1Edge;      // data sampled at second edge
	SPI_InitStruct.SPI_NSS = SPI_NSS_Soft | SPI_NSSInternalSoft_Set; // set the NSS management to internal and pull internal NSS high
	SPI_InitStruct.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_4; // SPI frequency is APB2 frequency / 4
	SPI_InitStruct.SPI_FirstBit = SPI_FirstBit_MSB;// data is transmitted MSB first
	SPI_Init(SPI1, &SPI_InitStruct); 
	


	

	/* setup DMA */

	// enable clock 
	RCC_AHB1PeriphClockCmd (SPIx_DMA_CLK, ENABLE); 
	
	/*// start with blank DMA configuration
	DMA_DeInit (SPIx_TX_DMA_STREAM);
	DMA_DeInit (SPIx_RX_DMA_STREAM);

	// check if DMA stream is disabled before enabling 
	// this is useful when stream is enabled and disabled multiple times. 
	while (DMA_GetCmdStatus (SPIx_TX_DMA_STREAM) != DISABLE);
	while (DMA_GetCmdStatus (SPIx_RX_DMA_STREAM) != DISABLE);
	
	
	DMA_StructInit(&DMA_InitStruct);
  Configure DMA Initialization Structure */
  	//DMA_InitStruct.DMA_BufferSize = 2000;
 /*	DMA_InitStruct.DMA_FIFOMode = DMA_FIFOMode_Enable ;
 	DMA_InitStruct.DMA_FIFOThreshold = DMA_FIFOThreshold_HalfFull ;
  	DMA_InitStruct.DMA_MemoryBurst = DMA_MemoryBurst_Single ;
  	DMA_InitStruct.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
  	DMA_InitStruct.DMA_MemoryInc = DMA_MemoryInc_Enable;
  	DMA_InitStruct.DMA_Mode = DMA_Mode_Normal;
  	DMA_InitStruct.DMA_PeripheralBaseAddr =(uint32_t) (&(SPIx->DR)) ;
  	DMA_InitStruct.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;
  	DMA_InitStruct.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
  	DMA_InitStruct.DMA_PeripheralInc = DMA_PeripheralInc_Enable;
  	DMA_InitStruct.DMA_Priority = DMA_Priority_High;
  	 Configure TX DMA */
  /*	DMA_InitStruct.DMA_Channel = SPIx_TX_DMA_CHANNEL ;
  	DMA_InitStruct.DMA_DIR = DMA_DIR_MemoryToPeripheral ;
  	DMA_InitStruct.DMA_Memory0BaseAddr = (uint32_t) &bufferTX ;
	DMA_InitStruct.DMA_BufferSize = MAX_BUFFER_LENGTH;
  	DMA_Init(SPIx_TX_DMA_STREAM, &DMA_InitStruct);
  	 Configure RX DMA */
//  	DMA_InitStruct.DMA_Channel = SPIx_RX_DMA_CHANNEL ;
  //	DMA_InitStruct.DMA_DIR = DMA_DIR_PeripheralToMemory ;
  //	DMA_InitStruct.DMA_Memory0BaseAddr = (uint32_t)&bufferRX; 
//	DMA_InitStruct.DMA_BufferSize = MAX_BUFFER_LENGTH;
	
//	DMA_ITConfig(SPIx_TX_DMA_STREAM, DMA_IT_TC, ENABLE); 
  
//	DMA_Init(SPIx_RX_DMA_STREAM, &DMA_InitStruct);
  //	SPI_I2S_DMACmd (SPIx, SPI_I2S_DMAReq_Tx, ENABLE);
  //	SPI_I2S_DMACmd (SPIx, SPI_I2S_DMAReq_Rx, ENABLE);

//	SPI_I2S_ClearFlag(SPIx, SPI_I2S_FLAG_TXE);
//	SPI_I2S_ClearFlag(SPIx, SPI_I2S_FLAG_RXNE);
  	
	// enable the interrupt in the NVIC
 //	NVIC_InitStruct.NVIC_IRQChannel = DMA1_Stream3_IRQn;
  //	NVIC_InitStruct.NVIC_IRQChannelPreemptionPriority = 0;
  //	NVIC_InitStruct.NVIC_IRQChannelSubPriority = 1;
  //	NVIC_InitStruct.NVIC_IRQChannelCmd = ENABLE;
  //	NVIC_Init (&NVIC_InitStruct);
  	// Enable dma tx and rx request
	

	

/*	   in this case we will use PA0 */
	GPIO_InitStruct.GPIO_Pin = GPIO_Pin_0;
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_IN;
	GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStruct.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_Init(GPIOA, &GPIO_InitStruct);	
	
	EXTI_InitTypeDef EXTI_InitStruct;
	/* Connect EXTI Line to appropriate GPIO Pin */ 
	SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOA, EXTI_PinSource0);
	
	/* Configure EXTI Line */
	EXTI_InitStruct.EXTI_Line = EXTI_Line0;
 	EXTI_InitStruct.EXTI_Mode = EXTI_Mode_Interrupt;
	EXTI_InitStruct.EXTI_Trigger = EXTI_Trigger_Falling;
	EXTI_InitStruct.EXTI_LineCmd = ENABLE;
	EXTI_Init(&EXTI_InitStruct);
 	
 	/* Enable and set EXTI Line Interrupt */
	NVIC_InitStruct.NVIC_IRQChannel = EXTI0_IRQn;
 	NVIC_InitStruct.NVIC_IRQChannelPreemptionPriority = 0x01;
	NVIC_InitStruct.NVIC_IRQChannelSubPriority = 0x01;
 	NVIC_InitStruct.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStruct);

	SPI_Cmd(SPI1, ENABLE);		
	
}




void SPI_IRQHandler()
{
	if (SPIx->SR & SPI_I2S_FLAG_TXE)
	{
		if (bufferIndex < datalength)
		{
			SPIx->DR = bufferTX[bufferIndex++];
		}
	}



	if (!(SPIx->SR & SPI_I2S_FLAG_BSY))
	{
		SPI_I2S_ITConfig(SPIx,  SPI_I2S_IT_TXE, DISABLE);
		CSOFF();
	}

	SPI_I2S_ClearITPendingBit(SPIx, SPI_I2S_IT_TXE);
}

int SPI1_DMA_TX_IRQHandler()
{
  // Test if DMA Stream Transfer Complete interrupt
  if (DMA_GetITStatus (SPIx_TX_DMA_STREAM, DMA_IT_TCIF4)) {
    DMA_ClearITPendingBit (SPIx_TX_DMA_STREAM, DMA_IT_TCIF4);
    while (SPI_I2S_GetFlagStatus (SPIx, SPI_I2S_FLAG_BSY) == SET);
    /*
     * The DMA stream is disabled in hardware at the end of the transfer
     * Now we can deselect the display. If more than one peripheral was being run
     * on this SPI peripheral, we would have to do both/all of them, or work out
     * which one was active and deselect that one.i
	
     */

	CSOFF();
	return 0;
  }
	return 1;
}


/* This funtion is used to transmit and receive data 
 * with SPI1
 * 			data --> data to be transmitted
 * 			returns received value

*/
uint8_t SPI1_send(uint8_t data){

	
	while(!SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_TXE)); 
	SPI_I2S_SendData(SPI1, data);
	
	while(!SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_RXNE));
	uint8_t d = SPI_I2S_ReceiveData(SPI1);
	return d;
 	
}

uint8_t SPI1_recive(void)
{

	while(!SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_TXE)); 
	SPI_I2S_SendData(SPI1,0x0 );
	
	while(!SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_RXNE));
	return SPI_I2S_ReceiveData(SPI1);
	
}


/*---------------------------- Matic Knap 25 Jun 2014 ---------------------*/

void spi_dma_send(uint16_t address, uint16_t data_len, uint8_t *data_buf)
{
		CSON(); // chip select  
		bufferIndex = 0; 
		bufferTX[0] = ((address & 0xff00) >> 8); // addres byte 1 
		bufferTX[1] = ((address & 0x00ff)); //address byte 2 
		bufferTX[2] = ((0x80 | (data_len & 0x7f00) >> 8 ));
		bufferTX[3] = (data_len & 0x00ff);

		int i;
		for (i = 4 ; i < data_len; i++) 
			bufferTX[i] = (data_buf[i-4]);
		datalength = data_len + 4;
	//	SPI_IRQHandler();
		DMA_SetCurrDataCounter(SPIx_TX_DMA_CHANNEL, datalength);
		DMA_Cmd(SPIx_TX_DMA_CHANNEL, ENABLE);	
		while(SPI_DMA_TX_IRQHandler());
		CSOFF(); // chip deselect 
			
	
}

void spi_send(uint16_t address, uint16_t data_len, uint8_t *data_buf)
{
		GPIOA->BSRRH |= GPIO_Pin_4; // chip select  
		
		SPI1_send((address & 0xff00) >> 8); // addres byte 1 
		SPI1_send((address & 0x00ff)); //address byte 2 
		SPI1_send((0x80 | (data_len & 0x7f00) >> 8 ));
		SPI1_send(data_len & 0x00ff);

		int i;
		for (i = 0; i < data_len; i++) SPI1_send(data_buf[i]);
		GPIOA->BSRRL |= GPIO_Pin_4; // chip deselect 
			
	
}

void spi_send2B(uint16_t address,  uint16_t data_buf)
{
		int16_t data_len = 2; 
		GPIOA->BSRRH |= GPIO_Pin_4; // chip select  
		
		SPI1_send((address & 0xff00) >> 8); // addres byte 1 
		SPI1_send((address & 0x00ff)); //address byte 2 
		SPI1_send((0x80 | (data_len & 0x7f00) >> 8 ));
		SPI1_send(data_len & 0x00ff);

		SPI1_send((data_buf & 0xff00) >> 8);
		SPI1_send(data_buf & 0x00ff);

		GPIOA->BSRRL |= GPIO_Pin_4; // chip deselect 
			
	
}


void spi_sendByte(uint16_t address,  uint8_t data_buf)
{
		int16_t data_len = 1; 
		GPIOA->BSRRH |= GPIO_Pin_4; // chip select  
		
		SPI1_send((address & 0xff00) >> 8); // addres byte 1 
		SPI1_send((address & 0x00ff)); //address byte 2 
		SPI1_send((0x80 | (data_len & 0x7f00) >> 8 ));
		SPI1_send(data_len & 0x00ff);

		SPI1_send(data_buf);
		GPIOA->BSRRL |= GPIO_Pin_4; // chip deselect 
			
	
}


uint16_t spi_read2B(uint16_t address)
{
		uint16_t data_len = 2; 
		uint8_t data_buf[2];
		GPIOA->BSRRH |= GPIO_Pin_4; // chip select  
		
		SPI1_send((address & 0xff00) >> 8); // addres byte 1 
		SPI1_send((address & 0x00ff)); //address byte 2 
		SPI1_send((0x00 | (data_len & 0x7f00) >> 8 ));
		SPI1_send(data_len & 0x00ff);

		int i;
		for (i = 0; i < data_len; i++)
		{
		
			 data_buf[i] = SPI1_recive();
			
		}

		GPIOA->BSRRL |= GPIO_Pin_4; // chip deselect 
		
		data_len = 0x0000;
		return (data_buf[0] << 8) | data_buf[1];

}
void spi_read(uint16_t address, uint16_t data_len, uint8_t *data_buf)
{
		GPIOA->BSRRH |= GPIO_Pin_4; // chip select  
		
		SPI1_send((address & 0xff00) >> 8); // addres byte 1 
		SPI1_send((address & 0x00ff)); //address byte 2 
		SPI1_send((0x00 | (data_len & 0x7f00) >> 8 ));
		SPI1_send(data_len & 0x00ff);

		int i;
		for (i = 0; i < data_len; i++)
		{
		
			 data_buf[i] = SPI1_recive();
			
		}
		GPIOA->BSRRL |= GPIO_Pin_4; // chip deselect 
			
	
}
int spi_master_transfer(uint32_t spiport, uint8_t *cmdbuff, uint8_t cmdbufflen, uint8_t *data, uint8_t datalen)
{
	
	uint8_t idx, d;
	// sending data  
	if (datalen == 0)
	{
		
		GPIOA->BSRRH |= GPIO_Pin_4; // chip select  
		
		for (idx = 0; idx < cmdbufflen; idx++)
		{
			SPI1_send(cmdbuff[idx]);
		}
		for (idx = 0; idx < datalen; idx++)
		{
			 SPI1_send(data[idx]);
		}
		GPIOA->BSRRL |= GPIO_Pin_4; // chip deselect 

		return 1;
	}
	
	// reading data
      else{
		GPIOA->BSRRH |= GPIO_Pin_4; // chip select 
		
		for (idx = 0; idx < cmdbufflen; idx++)
		{
			d = SPI1_send(cmdbuff[idx]);

		}

		GPIOA->BSRRL |= GPIO_Pin_4; // chip deselect 
		


		return 1; 
	}

	return 0;
}
/*---------------------------- Matic Knap 25 Jun 2014 ---------------------*/

