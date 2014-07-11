
#ifndef SPI_H
#define SPI_H

#define SPIx                           SPI1
#define SPIx_CLK                       RCC_APB1Periph_SPI1
#define SPIx_CLK_INIT                  RCC_APB1PeriphClockCmd
#define SPIx_IRQn                      SPI1_IRQn
#define SPIx_IRQHANDLER                SPI1_IRQHandler

#define SPIx_SCK_PIN                   GPIO_Pin_5
#define SPIx_SCK_GPIO_PORT             GPIOA
#define SPIx_SCK_GPIO_CLK              RCC_AHB1Periph_GPIOA
#define SPIx_SCK_SOURCE                GPIO_PinSource5
#define SPIx_SCK_AF                    GPIO_AF_SPI1

#define SPIx_MISO_PIN                  GPIO_Pin_6
#define SPIx_MISO_GPIO_PORT            GPIOA
#define SPIx_MISO_GPIO_CLK             RCC_AHB1Periph_GPIOA
#define SPIx_MISO_SOURCE               GPIO_PinSource6
#define SPIx_MISO_AF                   GPIO_AF_SPI1

#define SPIx_MOSI_PIN                  GPIO_Pin_7
#define SPIx_MOSI_GPIO_PORT            GPIOA
#define SPIx_MOSI_GPIO_CLK             RCC_AHB1Periph_GPIOA
#define SPIx_MOSI_SOURCE               GPIO_PinSource7
#define SPIx_MOSI_AF                   GPIO_AF_SPI1

#define SPIx_DMA                       DMA1
#define SPIx_DMA_CLK                   RCC_AHB1Periph_DMA1
#define SPIx_TX_DMA_CHANNEL            DMA_Channel_3
#define SPIx_TX_DMA_STREAM             DMA1_Stream3
#define SPIx_TX_DMA_FLAG_TCIF          DMA_FLAG_TCIF3
#define SPIx_RX_DMA_CHANNEL            DMA_Channel_2
#define SPIx_RX_DMA_STREAM             DMA1_Stream2
#define SPIx_RX_DMA_FLAG_TCIF          DMA_FLAG_TCIF2

void init_SPI1(void);
uint8_t SPI1_recive(void);
void spi_send(uint16_t address, uint16_t data_len, uint8_t *data_buf);
void spi_read(uint16_t address, uint16_t data_len, uint8_t *data_buf);
void spi_sendByte(uint16_t address, uint8_t data_buf);
void spi_send2B(uint16_t address, uint16_t data_buf);
uint16_t spi_read2B(uint16_t address);

int spi_master_transfer(uint32_t spiport, uint8_t *cmdbuff, uint8_t cmdbufflen, uint8_t *data, uint8_t datalen);
#endif
