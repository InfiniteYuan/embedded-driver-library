#ifndef _SPI_H
#define _SPI_H


#include "stm32f10x.h"
#include "stm32f10x_spi.h"
#include "stm32f10x_dma.h"
#include "stm32f10x_rcc.h"
#include "stm32f10x_gpio.h"
#include "FIFOBuffer.h"
#include "GPIO.h"


/******************************************************************************************************/
							/****configuration  ʹ��ǰ��������****/
		#define SPI_TX_BUFFER_SIZE     64              //SPI BUFFER FIFO SIZE
		#define SPI_RX_BUFFER_SIZE     64              //SPI BUFFER FIFO SIZE
		#define SPI_DMA_TX_BUFFER_SIZE 20               //SPI DMA BUFFER SIZE
		
/*******************************************************************************************************/

typedef enum
{
	SPI_RXNE_IRQ,				//SPI���ջ������ǿ��ж�
	SPI_TXE_IRQ,				//SPI���ͻ��������ж�
	SPI_ERR_IRQ					//SPI�����ж�
}SPIIrqType;

typedef enum
{
	DMA_HT_IRQ,					//DMA��������ж�
	DMA_TC_IRQ,					//DMA��������ж�
	DMA_TE_IRQ					//DMA�����ж�
}DMAIrqType;

class SPI{

private:

	FIFOBuffer<u8,SPI_TX_BUFFER_SIZE>  SPIbufferTx;
	FIFOBuffer<u8,SPI_RX_BUFFER_SIZE>  SPIbufferRx;
	u8 bufferTxDma[SPI_DMA_TX_BUFFER_SIZE];
	DMA_Channel_TypeDef* dmaChannel;				//dmaͨ��
	uint32_t dmaTCFlagChannel;								//dma��������жϱ�־λ
	uint32_t dmaGLFlagChannel;								//dmaȫ���жϱ�־λ
	uint32_t dmaTEFlagChannel;								//dma�����жϱ�־λ
	bool isBusySend;
	bool mUseDma;
	unsigned char mPrecision;
/*-------SPI Pin Configuration------*/
	SPI_TypeDef* SPIx;
	GPIO mSCK;
	GPIO mCSN;
	GPIO mMISO;
	GPIO mMOSI;

	void RCC_Configuration(void);

public:
	
	SPI(SPI_TypeDef* SPI, bool useDMA = false, u8 remap = 0, u8 Prioritygroup = 3,uint8_t preemprionPriority = 7,uint8_t subPriority = 1,u8 dmaPriority = 3);
	~SPI();
	
	u8 SPI_RW(u8 dat);
	bool SendData(uint8_t *pbuffer, uint32_t size);
	bool GetReceivedData(u8* buffer, u16 number);

	void SetSpeed(u8 SPI_BaudRatePrescaler);

	u16 ReceiveBufferSize();
	u16 SendBufferSize();

	void ClearReceiveBuffer();
	void ClearSendBuffer();

	GPIO& GetmMISOx();
	GPIO& GetmMOSIx();

	SPI_TypeDef* getSPI();
	bool UseDma();

	SPIIrqType SpiIrq();
	DMAIrqType DmaIrq();
	
	void SetCSNPin(u8 value);

};


#endif

