/**
*@file USART.cpp
*@author Neutree
*@version v1.0
*@brief stm32f10x串口驱动文件，使用时引入 USART.h USART.cpp FIFOBuffer.h三个文件，然后根据需要配置USART.h中开头的配置部分（注意中断函数配置）
* 然后定义对象初始化，既可以使用
*@copyright
*
*/

#include "USART.h"
#include "Interrupt.h"


///////////////////
///串口发送数据
///@param pbuffer 需要发送数据的首地址
///@param size 需要发送的数据的长度
///@retval 发送是否成功 -false:发送失败（队列溢出）  -true:发送成功（即将发送）
///////////////////
bool USART::SendData(uint8_t *pbuffer, uint32_t size)
{
	//将需要发送的数据放入缓冲区队列
	bufferTx.Puts(pbuffer, size);
	if (mUseDma && !isBusySend)//使用DMA并且没有忙于发送，则开启发送
	{
		//断发送队列里是否还存在数据，如果存在，继续发送，否则关闭DMA
		if (bufferTx.Size() > 0)
		{
			isBusySend = true;                               //标记忙于发送
			if (bufferTx.Size() <= USART_DMA_TX_BUFFER_SIZE)  //队列中的数据长度小于dma缓冲区的最大长度
			{
				dmaChannelTx->CNDTR = (uint32_t)bufferTx.Size();
				bufferTx.Gets(bufferTxDma, bufferTx.Size());//将剩下的数据放到dma缓冲区
			}
			else
			{
				dmaChannelTx->CNDTR = USART_DMA_TX_BUFFER_SIZE;
				bufferTx.Gets(bufferTxDma, USART_DMA_TX_BUFFER_SIZE);//将数据放到dma缓冲区
			}
			DMA_Cmd(dmaChannelTx, ENABLE); 	//使能DMA,开始发送
		}
	}
	if (!mUseDma && !isBusySend)//使用中断发送
	{
		if (bufferTx.Size() > 0)//数据区确实存在数据
		{
			USART_ClearITPendingBit(usart, USART_IT_TC);	  //清标记，防止第一个数据发送不出去的情况，一定是要先清再打开
			USART_ITConfig(usart, USART_IT_TC, ENABLE);//开启发送中断，然后发送数据
			USART_GetFlagStatus(usart, USART_FLAG_TC);//读取sr，清除发送完成标志，不然有时可能会有第一个字符发送不出去的情况
			isBusySend = true;

			static u8 dataToSend = 0;
			bufferTx.Get(dataToSend);
			USART_SendData(usart, dataToSend);
		}
	}
	return 1;
}



///////////////////
///获取缓冲区中的数据
///@param buffer -数据要保存到的地方
///@param number -数据个数
///@retval 获取数据是否成功
///////////////////
bool USART::GetReceivedData(u8* buffer, u16 number)
{
	if (bufferRx.Size() < number)//没有足够长的数据
		return false;
	else
	{
		bufferRx.Gets(buffer, number);//数据出队
		return true;
	}
}

void fun(){}

/////////////////////
///获取缓冲区有效数据的长度
///@retval -缓冲区有效数据的长度
/////////////////////
u16 USART::ReceiveBufferSize()
{
	return bufferRx.Size();
}

/////////////////////
///获取发送缓冲区有效数据的长度
///@retval -缓冲区有效数据的长度
/////////////////////
u16 USART::SendBufferSize()
{
	return bufferTx.Size();
}


////////////////////
///@brief 带参数的构造函数
///@param USART	 选择串口编号 1-3 ,对应的引脚请看下面重映射处        default:1
///@param baud -串口的波特率              default:9600
///@param useDMA  true:if you want to send data by DMA false:don't use dma to send data
///@param remap -if remap 0x00:no remap  0x01:half remap 0x11:remap  default:0x00
///	remap		0x00		0x01		0x11
/// usart1Tx	PA9			PB6
/// usart1Rx	PA10		PB7
/// usart2Tx	PA2			PD5
/// usart2Rx	PA3			PD6
/// usart3Tx	PB10		PC10		PD8
/// usart3Rx	PB11		PC11		PD9
///@param Prioritygroup -中断优先级分组   default:3  优先级分组详情请看文件末尾
///@param preemprionPriority -抢占优先级  default:7  优先级分组详情请看文件末尾
///@param subPriority -响应优先级         default:1  优先级分组详情请看文件末尾
///@param dmaPriority set the priority of DMA default:3(low) (0:DMA_Priority_VeryHigh 1:DMA_Priority_High 2:DMA_Priority_Medium 3:DMA_Priority_Low)
///@param  parity usart parity value: USART_Parity_No  USART_Parity_Even  USART_Parity_Odd      default:USART_Parity_No
///@param  wordLength   USART_WordLength_9b  USART_WordLength_8b                                default:USART_WordLength_8b
///@param  stopBits  USART_StopBits_1 USART_StopBits_0_5  USART_StopBits_2  USART_StopBits_1_5  default:USART_StopBits_1
////////////////////
USART::USART(u8 USART, uint32_t baud, bool useDMA, u8 remap, u8 Prioritygroup, uint8_t preemprionPriority, uint8_t subPriority, u8 dmaPriority
	, uint16_t parity, uint16_t wordLength, uint16_t stopBits)
	:isBusySend(0), mUseDma(useDMA), mPrecision(3)
{
	////////////////////////////
	//add by jason
	uint16_t txPin = 0, rxPin = 0;
	/////////////////////////////

	GPIO_TypeDef* USART_GPIO;//端口号
	USART_InitTypeDef USART_InitStructure;//串口配置
	uint8_t usartIrqChannel, dmaIrqChannel;//中断通道,串口发送dma通道

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);

	if (USART == 1)
	{
		usart = USART1;
		usartIrqChannel = USART1_IRQn;
		if (mUseDma)
		{
			dmaChannelTx = DMA1_Channel4;
			dmaIrqChannel = DMA1_Channel4_IRQn;
			dmaTcFlagChannel = DMA1_FLAG_TC4;
			dmaGlFlagChannel = DMA1_IT_GL4;
		}
		pUSART1 = this;
		if (remap == 0x01)
		{
			RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE);
			GPIO_PinRemapConfig(GPIO_Remap_USART1, ENABLE);

			txPin = 6;
			rxPin = 7;

			USART_GPIO = GPIOB;

		}
		else
		{
			RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE);

			txPin = 9;
			rxPin = 10;

			USART_GPIO = GPIOA;
		}
	}
	else if (USART == 2)
	{
		usart = USART2;
		usartIrqChannel = USART2_IRQn;
		if (mUseDma)
		{
			dmaChannelTx = DMA1_Channel7;
			dmaIrqChannel = DMA1_Channel7_IRQn;
			dmaTcFlagChannel = DMA1_FLAG_TC7;
			dmaGlFlagChannel = DMA1_IT_GL7;
		}
		pUSART2 = this;
		if (remap == 0x01)
		{
			//CLOCK
			RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE);
			GPIO_PinRemapConfig(GPIO_Remap_USART2, ENABLE);

			txPin = 5;
			rxPin = 6;

			USART_GPIO = GPIOD;
		}
		else
		{
			RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE);
			txPin = 2;
			rxPin = 3;

			USART_GPIO = GPIOA;
		}
	}
	else if (USART == 3)
	{
		usart = USART3;
		usartIrqChannel = USART3_IRQn;
		if (mUseDma)
		{
			dmaChannelTx = DMA1_Channel2;
			dmaIrqChannel = DMA1_Channel2_IRQn;
			dmaTcFlagChannel = DMA1_FLAG_TC2;
			dmaGlFlagChannel = DMA1_IT_GL2;
		}
		pUSART3 = this;
		if (remap == 0x01)
		{
			RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3, ENABLE);
			GPIO_PinRemapConfig(GPIO_PartialRemap_USART3, ENABLE);

			txPin = 10;
			rxPin = 11;

			USART_GPIO = GPIOC;
		}
		else if (remap == 0x11)
		{
			RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3, ENABLE);
			GPIO_PinRemapConfig(GPIO_FullRemap_USART3, ENABLE);

			txPin = 8;
			rxPin = 9;

			USART_GPIO = GPIOD;
		}
		else
		{
			RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3, ENABLE);

			txPin = 10;
			rxPin = 11;

			USART_GPIO = GPIOB;
		}
	}

	//add by jason
	GPIO tx(USART_GPIO, txPin, GPIO_Mode_AF_PP, GPIO_Speed_10MHz);
	GPIO rx(USART_GPIO, rxPin, GPIO_Mode_IN_FLOATING, GPIO_Speed_10MHz);

	mTx = tx;
	mRx = rx;

	/* USART mode config */
	USART_InitStructure.USART_BaudRate = baud;
	USART_InitStructure.USART_WordLength = wordLength;
	USART_InitStructure.USART_StopBits = stopBits;
	USART_InitStructure.USART_Parity = parity;
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
	USART_Init(usart, &USART_InitStructure);
	USART_Cmd(usart, ENABLE);

	//jason add
	mBaud = baud;


	//Enable translate DMA
	if (mUseDma)
	{
		USART_DMACmd(usart, USART_DMAReq_Tx, ENABLE);

		DMA_InitTypeDef DMA_InitStructure;

		/*开启DMA时钟*/
		RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);

		//设置DMA源：串口数据寄存器地址
		DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)&usart->DR;

		//内存地址(要传输的变量的指针)
		DMA_InitStructure.DMA_MemoryBaseAddr = (u32)bufferTxDma;/*dmaChannelTx->CMAR =  (u32)bufferTxDma;*/
		//方向：从内存到外设
		DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralDST;

		//传输大小DMA_BufferSize=SENDBUFF_SIZE，初始值设置为1，如果为零，在eclipse编译通不过（assert检查要大于零）
		DMA_InitStructure.DMA_BufferSize = 1;

		//外设地址不增
		DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;

		//内存地址自增
		DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;

		//外设数据单位
		DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;

		//内存数据单位 8bit
		DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;

		//DMA模式：不循环
		DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;

		//优先级：低
		switch (dmaPriority){
		case 0:
			DMA_InitStructure.DMA_Priority = DMA_Priority_VeryHigh;
			break;
		case 1:
			DMA_InitStructure.DMA_Priority = DMA_Priority_High;
			break;
		case 2:
			DMA_InitStructure.DMA_Priority = DMA_Priority_Medium;
			break;
		default:
			DMA_InitStructure.DMA_Priority = DMA_Priority_Low;
			break;
		}

		//禁止内存到内存的传输	
		DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;

		//配置DMA1的通道 Tx通道
		DMA_Init(dmaChannelTx, &DMA_InitStructure);

		//失能DMA
		DMA_Cmd(dmaChannelTx, DISABLE);
		/* DMA_ITConfig(DMA1_Channel4,DMA_IT_TC,ENABLE);  //配置DMA发送完成后产生中断 这里没有设置，在优先级设置里面设置 */
	}
	else  //use interrupt way to send data
	{
		USART_ITConfig(usart, USART_IT_TC, DISABLE);//enable the transport complete interrupt
		//USART_GetFlagStatus(usart, USART_FLAG_TC);//清除发送完成标志，不然有时可能会有第一个字符发送不出去的情况
	}
	//优中断先级设置
	NVIC_InitTypeDef NVIC_InitStructure;
	switch (Prioritygroup)
	{
	case 0:
		NVIC_PriorityGroupConfig(NVIC_PriorityGroup_0);
		break;
	case 1:
		NVIC_PriorityGroupConfig(NVIC_PriorityGroup_1);
		break;
	case 2:
		NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
		break;
	default:
		NVIC_PriorityGroupConfig(NVIC_PriorityGroup_3);
		break;
	case 4:
		NVIC_PriorityGroupConfig(NVIC_PriorityGroup_4);
		break;
	}

	/******************注册串口1中断服务函数************************/
	NVIC_InitStructure.NVIC_IRQChannel = usartIrqChannel;//配置串口中断
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = preemprionPriority;//占先式优先级设置为
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = subPriority; //副优先级设置为
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;//中断开启
	NVIC_Init(&NVIC_InitStructure);//中断初始化
	//使能串口1接收中断
	USART_ITConfig(usart, USART_IT_RXNE, ENABLE);

	if (mUseDma)
	{
		NVIC_InitStructure.NVIC_IRQChannel = dmaIrqChannel;
		NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = preemprionPriority;
		NVIC_InitStructure.NVIC_IRQChannelSubPriority = subPriority;
		NVIC_Init(&NVIC_InitStructure);
		/* Enable DMA TX Channel TCIT  */
		dmaChannelTx->CCR |= DMA_IT_TC;  //打开发送完成中断
		/* Enable DMA TX Channel TEIT  */
		dmaChannelTx->CCR |= DMA_IT_TE; //打开错误中断
		/* Enable DMA TX Channel HTIT  */
		/*DMA1_Channel4->CCR &= ~DMA_IT_HT;//关闭发送一般产生中断*/
	}
}

void USART::SetBaudRate(uint32_t baudRate)
{
	//	assert_param(IS_USART_BAUDRATE(baudRate));
	uint32_t tmpreg = 0x00, apbclock = 0x00;
	uint32_t integerdivider = 0x00;
	uint32_t fractionaldivider = 0x00;
	uint32_t usartxbase = 0;
	RCC_ClocksTypeDef RCC_ClocksStatus;


	usartxbase = (uint32_t)usart;


	/*---------------------------- USART BRR Configuration -----------------------*/
	/* Configure the USART Baud Rate -------------------------------------------*/
	RCC_GetClocksFreq(&RCC_ClocksStatus);
	if (usartxbase == USART1_BASE)
	{
		apbclock = RCC_ClocksStatus.PCLK2_Frequency;
	}
	else
	{
		apbclock = RCC_ClocksStatus.PCLK1_Frequency;
	}

	/* Determine the integer part */
	if ((usart->CR1 & ((u16)0x8000)) != 0)//CR1_OVER8_Set=((u16)0x8000)
	{
		/* Integer part computing in case Oversampling mode is 8 Samples */
		integerdivider = ((25 * apbclock) / (2 * (baudRate)));
	}
	else /* if ((USARTx->CR1 & CR1_OVER8_Set) == 0) */
	{
		/* Integer part computing in case Oversampling mode is 16 Samples */
		integerdivider = ((25 * apbclock) / (4 * (baudRate)));
	}
	tmpreg = (integerdivider / 100) << 4;

	/* Determine the fractional part */
	fractionaldivider = integerdivider - (100 * (tmpreg >> 4));

	/* Implement the fractional part in the register */
	if ((usart->CR1 & ((u16)0x8000)) != 0)
	{
		tmpreg |= ((((fractionaldivider * 8) + 50) / 100)) & ((uint8_t)0x07);
	}
	else /* if ((USARTx->CR1 & CR1_OVER8_Set) == 0) */
	{
		tmpreg |= ((((fractionaldivider * 16) + 50) / 100)) & ((uint8_t)0x0F);
	}

	/* Write to USART BRR */
	usart->BRR = (uint16_t)tmpreg;
}


USART_TypeDef* USART::GetUSART()
{
	return usart;
}

uint32_t USART::GetBaudRate()
{
	return mBaud;
}

///////////////////////////
///@brief Destructor
///////////////////////////
USART::~USART()
{
	if (USART1 == usart)
		pUSART1 = 0;
	else if (USART2 == usart)
		pUSART2 = 0;
	else if (USART3 == usart)
		pUSART3 = 0;
}

///////////////////////////
///@brief output character reload
///@param val the integer value that will be print to USART as characters
///////////////////////////
USART& USART::operator<<(int val)
{
	u8 sign = 0, len = 0, data[10];
	if (val < 0)
	{
		sign = 1;
		val = -val;
	}
	do
	{
		len++;
		data[10 - len] = val % 10 + '0';
		val = val / 10;
	} while (val);
	if (sign == 1)
		data[10 - (++len)] = '-';
	SendData(data + 10 - len, len);
	return *this;
}


USART& USART::operator<<(double val)
{
	u8 sign = 0, len = 0, data[20];
	if (val < 0)
	{
		sign = 1;
		val = -val;
	}
	u8 prec = mPrecision;
	while (prec--)
		val *= 10;
	u32 t = val;
	do
	{
		if (++len == mPrecision + 1) data[20 - len] = '.';
		else
		{
			data[20 - len] = t % 10 + '0';
			t = t / 10;
		}
	} while (t || len < mPrecision + 2);
	//if(len==3) data[20-(++len)] = '.';
	//if(len==4) data[20-(++len)] = '0';
	if (sign == 1)
		data[20 - (++len)] = '-';
	SendData(data + 20 - len, len);
	return *this;
}

USART& USART::Setprecision(const unsigned char precision)
{
	mPrecision = precision;
	return *this;
}
///////////////////////////
///@brief output character reload
///@param the string's first character adress value that will be print to USART as characters
///////////////////////////
USART& USART::operator<<(const char* pStr)
{
	unsigned int length = 0;
	for (int i = 0; pStr[i] != '\0'; ++i)
	{
		++length;
	}
	SendData((u8*)pStr, length);
	return *this;
}
USART& USART::operator<<(char charactor)
{
	SendData((uint8_t*)&charactor, 1);
	return *this;
}
//////////////////////
///串口中断处理函数
//////////////////////
UsartIrqType USART::Irq()
{
	UsartIrqType type;
	if (USART_GetITStatus(usart, USART_IT_RXNE) != RESET)
	{
		bufferRx.Put(USART_ReceiveData(usart));
		USART_ClearITPendingBit(usart, USART_IT_RXNE);        //清除中断标志
		type = USART_RX_IRQ;
	}
	if (USART_GetITStatus(usart, USART_IT_TXE) != RESET)//一个字节发送完成
	{
		static u8 dataToSend = 0;
		if (bufferTx.Size() > 0)
		{
			bufferTx.Get(dataToSend);
			USART_SendData(usart, dataToSend);
		}
		if (bufferTx.Size() == 0)//发送完毕，关发送中断
		{
			USART_ITConfig(usart, USART_IT_TXE, DISABLE);
			isBusySend = false;
		}

		type = USART_TX_IRQ;
	}
	if (USART_GetITStatus(usart, USART_IT_TC) != RESET)//一个字节发送完成
	{
		static u8 dataToSend = 0;
		if (bufferTx.Size() > 0)
		{
			bufferTx.Get(dataToSend);
			USART_SendData(usart, dataToSend);
		}
		if (bufferTx.Size() == 0)//发送完毕，关发送中断
		{
			USART_ITConfig(usart, USART_IT_TC, DISABLE);
			isBusySend = false;
			// bufferTx.Clear();//缓冲区复位
		}
		type = USART_TX_IRQ;
	}

	return type;
}


/////////////////////
///dma中断处理函数
/////////////////////
void USART::DmaIrq()
{
	//判断是否为DMA发送完成中断
	if (DMA_GetFlagStatus(dmaTcFlagChannel) == SET)
	{
		DMA_ClearITPendingBit(dmaGlFlagChannel);//清除全局中断标志
		DMA_ClearFlag(dmaTcFlagChannel);   //清除传输完成标志
		DMA_Cmd(dmaChannelTx, DISABLE);    //close dma
		//发送完成，判断发送队列里是否还存在数据，如果存在，继续发送，否则关闭DMA
		if (bufferTx.Size() > 0)
		{
			if (bufferTx.Size() <= USART_DMA_TX_BUFFER_SIZE)//队列中的数据长度小于dma缓冲区的最大长度
			{
				dmaChannelTx->CNDTR = bufferTx.Size();
				bufferTx.Gets(bufferTxDma, bufferTx.Size());//将剩下的数据放到dma缓冲区			
			}
			else
			{
				dmaChannelTx->CNDTR = USART_DMA_TX_BUFFER_SIZE;
				bufferTx.Gets(bufferTxDma, USART_DMA_TX_BUFFER_SIZE);//将数据放到dma缓冲区
			}
			DMA_Cmd(dmaChannelTx, ENABLE); 	//使能DMA,开始发送
		}
		else
			isBusySend = false;               //将忙标志
	}
}


////////////////////////////////
///@brief Use DMA or not
///@retval true:use dma  false:don't use dma 
////////////////////////////////
bool USART::UseDma()
{
	return mUseDma;
}

//////////////////////////
///brief 清空接收缓冲区中的数据
//////////////////////////
void USART::ClearReceiveBuffer()
{
	bufferRx.Clear();
}

//////////////////////////
///brief 清空发送缓冲区中的数据
//////////////////////////
void USART::ClearSendBuffer()
{
	bufferTx.Clear();
}

//得到USART的Tx所使用的引脚
GPIO& USART::GetmTx()
{
	return mTx;
}

//得到USART的Rx所使用的引脚
GPIO& USART::GetmRx()
{
	return mRx;
}

