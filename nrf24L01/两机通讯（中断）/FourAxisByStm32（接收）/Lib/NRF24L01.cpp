#include "NRF24L01.h"
#include "USART.h"
#include "Interrupt.h"

u8 TX_ADDRESS[TX_ADR_WIDTH] = { 0x34, 0x43, 0x10, 0x10, 0x01 };  // ����һ����̬���͵�ַ
u8 RX_ADDRESS[RX_ADR_WIDTH] = { 0x34, 0x43, 0x10, 0x10, 0x01 };

void Delay(__IO u32 nCount)
{
	for (; nCount != 0; nCount--);
}

u8 NRF24L01::SendData(uint8_t *pbuffer, uint32_t size){//����ʹ��

//	u8 state;
	u8 dataToSend[32];
//	EXTI->IMR &= ~(1 << 2);			//���Ϳ�ʼʱ�������ж���
	mSPI.SetSpeed(SPI_BaudRatePrescaler_8);//spi�ٶ�Ϊ9Mhz��24L01�����SPIʱ��Ϊ10Mhz��  
	NRFbufferTx.Puts(pbuffer, size);
//	state = NRF_ReadReg(STATUS);
	NRF_CE_LOW();
	if ((size < TX_PLOAD_WIDTH) && (size>0)){//��������
		NRFbufferTx.Gets(dataToSend, size);
		NRF_WriteBuf(WR_TX_PLOAD, dataToSend, TX_PLOAD_WIDTH);	 //д���ݵ�TXBUF 32���ֽ�
		NRF_CE_HIGH();  					//CEΪ�ߣ���������
//		EXTI->PR = 1 << 2;					//���line���ж�	
//		EXTI->IMR |= 1 << 2;				//���ͽ������������ж���
		return TX_DS;
	}
	else if (size >= TX_PLOAD_WIDTH){
		NRFbufferTx.Gets(dataToSend, TX_PLOAD_WIDTH);
		NRF_WriteBuf(WR_TX_PLOAD, dataToSend, TX_PLOAD_WIDTH);	 //д���ݵ�TXBUF 32���ֽ�
		NRF_CE_HIGH();  	//CEΪ�ߣ���������
//		while (NRF_Read_IRQ() != 0);//�ȴ��������
//		state =NRF_ReadReg(STATUS);
//		NRF_WriteReg(NRF_WRITE_REG + STATUS, state);				//���NRF24L01�жϱ�־	
//		EXTI->PR = 1 << 2;					//���line���ж�	
//		EXTI->IMR |= 1 << 2;				//���ͽ������������ж���
//		if (state&MAX_RT){//����ط�����
//			NRF_WriteReg(FLUSH_TX, NOP); //���TX FIFO������
//			return MAX_RT;
//		}if (state&TX_DS){//�������
//			return TX_DS;
//		}
		return nrf_flag;
	}
	return FALSE;
}

NRF24L01::NRF24L01(SPI &SPIx) :mSPI(SPIx), isSending(false){
	pNRF = this;
	SetCEPin(GPIOA, 1);
	SetIRQPin(GPIOA, 2);
	NRF_Reset();
	NRF_CE_LOW();
	NRF_CSN_HIGH();
//	NRF_TX_Mode();//��������
}

//NRF24L01��/дһ�ֽ�����
u8 NRF24L01::SPI_NRF_RW(u8 dat){
//	u8 retry = 0;
//	while (SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_TXE) == RESET) //���ָ����SPI��־λ�������:���ͻ���ձ�־λ
//	{
//		retry++;
//		if (retry > 200)return 0;
//	}
//	SPI_I2S_SendData(SPI1, dat); //ͨ������SPIx����һ������
//	retry = 0;

//	while (SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_RXNE) == RESET)//���ָ����SPI��־λ�������:���ܻ���ǿձ�־λ
//	{
//		retry++;
//		if (retry > 200)return 0;
//	}
//	return SPI_I2S_ReceiveData(SPI1); //����ͨ��SPIx������յ�����
	return mSPI.SPI_RW(dat);
}

bool NRF24L01::NRF_SendData(uint8_t *pbuffer, uint32_t size){			//����һ������
	return mSPI.SendData(pbuffer, size);
}

bool NRF24L01::NRF_ReceiverData(u8 *buffer, u16 number){					//����һ������
	return mSPI.GetReceivedData(buffer, number);
}

//��NRF24L01�ض��Ĵ���д��1���ֽ�����
u8 NRF24L01::NRF_WriteReg(u8 reg, u8 dat){

	u8 status;
//	NRF_CE_LOW();   //�����ֻ�ܽ���һ�����ݣ��Ǿ����ڽ������ݺ�û������CE
	NRF_CSN_LOW();           //�õ�CSN��ʹ��SPI����
	status = SPI_NRF_RW(reg);//���ͼĴ�����	
	SPI_NRF_RW(dat);         //��Ĵ���д������	   
	NRF_CSN_HIGH();					 //CSN���ߣ����		
	return(status);          //����״̬�Ĵ�����ֵ
}

//��NRF24L01�ض��Ĵ�������1���ֽ�����
u8 NRF24L01::NRF_ReadReg(u8 reg){

	u8 reg_val;
//	NRF_CE_LOW();   //�����ֻ�ܽ���һ�����ݣ��Ǿ����ڽ������ݺ�û������CE
	NRF_CSN_LOW();							//�õ�CSN��ʹ��SPI����	 
	SPI_NRF_RW(reg); 						//���ͼĴ����� 
	reg_val = SPI_NRF_RW(NOP);	//��ȡ�Ĵ�����ֵ  	
	NRF_CSN_HIGH();							//CSN���ߣ����  	
	return reg_val;
}

//���ض��Ĵ�������ָ���ֽ�����
u8 NRF24L01::NRF_ReadBuf(u8 reg, u8 *pBuf, u8 bytes){

	u8 status;
	u8 i;

//	NRF_CE_LOW();   //�����ֻ�ܽ���һ�����ݣ��Ǿ����ڽ������ݺ�û������CE
	NRF_CSN_LOW();										//�õ�CSN��ʹ��SPI����
	status = SPI_NRF_RW(reg); 				//�����ض��Ĵ���

	for (i = 0; i < bytes; i++)							//��ȡ���� 
		pBuf[i] = SPI_NRF_RW(NOP);
	//	NRF_ReceiverData(pBuf,bytes);

	NRF_CSN_HIGH();	//CSN���ߣ����	
	return status;	//���ؼĴ���״ֵ̬
}

//���ض��Ĵ���д��ָ���ֽ�����
u8 NRF24L01::NRF_WriteBuf(u8 reg, u8 *pBuf, u8 bytes){

	u8 status;
//	u8 i;

//	NRF_CE_LOW();   //�����ֻ�ܽ���һ�����ݣ��Ǿ����ڽ������ݺ�û������CE
	NRF_CSN_LOW();			               //�õ�CSN��ʹ��SPI����	
	status = SPI_NRF_RW(reg); //���ʷ��ͼĴ���

	NRF_SendData(pBuf, bytes);
	//	for (i = 0; i < bytes; i++)		 //�򻺳���д������
	//		SPI_NRF_RW(pBuf[i]);

	NRF_CSN_HIGH();		//CSN���ߣ���� 
	return (status);	//����NRF24L01��״̬ 
}

//��NRF24L01����һ֡����
//����OK=1 ������ɣ�FALSE=0 ����
u8 NRF24L01::NRF_RecievePacket(u8 *RxBuf){

	u8 state;
	mSPI.SetSpeed(SPI_BaudRatePrescaler_8);//spi�ٶ�Ϊ9Mhz��24L01�����SPIʱ��Ϊ10Mhz��  
	state = NRF_ReadReg(STATUS);
	NRF_WriteReg(NRF_WRITE_REG + STATUS, state);				//���NRF24L01�жϱ�־	
	if (state&RX_DR){//���յ�����
		NRF_ReadBuf(RD_RX_PLOAD, RxBuf, RX_PLOAD_WIDTH);//������	
		NRF_WriteReg(FLUSH_RX, NOP);								   		//���RX FIFO������
		return OK;
	}
	return FALSE;
}

//��NRF24L01����һ֡����
//����OK=1 ������ɣ�FALSE=0 ���� MAX_RT ����ط�����
u8 NRF24L01::NRF_SendPacket(u8 *TxBuf){

	u8 sta;
	EXTI->IMR &= ~(1 << 2);			//���Ϳ�ʼʱ�������ж���
	mSPI.SetSpeed(SPI_BaudRatePrescaler_8);//spi�ٶ�Ϊ9Mhz��24L01�����SPIʱ��Ϊ10Mhz��   
	NRF_CE_LOW();			//�������ģʽ
	NRF_WriteBuf(WR_TX_PLOAD, TxBuf, TX_PLOAD_WIDTH);//д���ݵ�TX BUF  32���ֽ�
	NRF_CE_HIGH();		//CE���� �������� 
	while (NRF_Read_IRQ() != RESET);//�ȴ��������
	sta = NRF_ReadReg(STATUS);  //��ȡ״̬�Ĵ�����ֵ	   
	NRF_WriteReg(NRF_WRITE_REG + STATUS, sta); //���TX_DS��MAX_RT�жϱ�־
	EXTI->PR = 1 << 2;					//���line���ж�	
	EXTI->IMR |= 1 << 2;				//���ͽ������������ж���
	if (sta&MAX_RT)//�ﵽ����ط�����
	{
		NRF_WriteReg(FLUSH_TX, 0xff);//���TX FIFO�Ĵ��� 
		return MAX_RT;
	}
	if (sta&TX_DS)//�������
	{
		return TX_DS;
	}
	return 0xff;//����ԭ����ʧ��
}

//���ò��������ģʽ
void NRF24L01::NRF_RX_Mode(void){

	NRF_CE_LOW();
	NRF_WriteBuf(NRF_WRITE_REG + RX_ADDR_P0, (u8*)RX_ADDRESS, RX_ADR_WIDTH);//дRX�ڵ��ַ

	NRF_WriteReg(NRF_WRITE_REG + EN_AA, 0x01);    //ʹ��ͨ��0���Զ�Ӧ��    
	NRF_WriteReg(NRF_WRITE_REG + EN_RXADDR, 0x01);//ʹ��ͨ��0�Ľ��յ�ַ  	 
	NRF_WriteReg(NRF_WRITE_REG + RF_CH, CHANAL);	     //����RFͨ��Ƶ��		  
	NRF_WriteReg(NRF_WRITE_REG + RX_PW_P0, RX_PLOAD_WIDTH);//ѡ��ͨ��0����Ч���ݿ�� 	    
	NRF_WriteReg(NRF_WRITE_REG + RF_SETUP, 0x0f);//����TX�������,0db����,2Mbps,���������濪��   
	NRF_WriteReg(NRF_WRITE_REG + CONFIG, 0x0f);//���û�������ģʽ�Ĳ���;PWR_UP,EN_CRC,16BIT_CRC,����ģʽ
	mSPI.SetSpeed(SPI_BaudRatePrescaler_8); //spi�ٶ�Ϊ9Mhz��24L01�����SPIʱ��Ϊ10Mhz�� 
	NRF_WriteReg(FLUSH_RX, 0xff);//���RX FIFO�Ĵ��� 
	NRF_WriteReg(NRF_WRITE_REG + STATUS, 0xff);	//***һ��Ҫ���״̬�Ĵ�������������**
	NRF_CE_HIGH();		//CE���ߣ��������ģʽ
}


//���ò����뷢��ģʽ
void NRF24L01::NRF_TX_Mode(void){

	NRF_CE_LOW();
	NRF_WriteBuf(NRF_WRITE_REG + TX_ADDR, (u8*)TX_ADDRESS, TX_ADR_WIDTH);//дTX�ڵ��ַ 
	NRF_WriteBuf(NRF_WRITE_REG + RX_ADDR_P0, (u8*)RX_ADDRESS, RX_ADR_WIDTH); //����TX�ڵ��ַ,��ҪΪ��ʹ��ACK	  

	NRF_WriteReg(NRF_WRITE_REG + EN_AA, 0x01);     //ʹ��ͨ��0���Զ�Ӧ��    
	NRF_WriteReg(NRF_WRITE_REG + EN_RXADDR, 0x01); //ʹ��ͨ��0�Ľ��յ�ַ  
	NRF_WriteReg(NRF_WRITE_REG + SETUP_RETR, 0x1a);//�����Զ��ط����ʱ��:500us + 86us;����Զ��ط�����:10��
	NRF_WriteReg(NRF_WRITE_REG + RF_CH, CHANAL);       //����RFͨ��Ϊ40
	NRF_WriteReg(NRF_WRITE_REG + RF_SETUP, 0x0f);  //����TX�������,0db����,2Mbps,���������濪��   
	NRF_WriteReg(NRF_WRITE_REG + CONFIG, 0x0e);    //���û�������ģʽ�Ĳ���;PWR_UP,EN_CRC,16BIT_CRC,����ģʽ,���������ж�
	NRF_WriteReg(FLUSH_TX, 0xff);//���TX FIFO�Ĵ��� 
	NRF_WriteReg(FLUSH_RX, 0xff);//���RX FIFO�Ĵ��� 
	NRF_WriteReg(NRF_WRITE_REG + STATUS, 0xff);	//���״̬�Ĵ���	
	NRF_CE_HIGH();//CE���ߣ����뷢��ģʽ
}


//NRF24L01�����ж�
u8 NRF24L01::NRF_Check(void){

	u8 i, buf[5] = { 0xC2, 0xC2, 0xC2, 0xC2, 0xC2 };
	u8 buf1[5] = { 0x00, 0x00, 0x00, 0x00, 0x00 };
	mSPI.SetSpeed(SPI_BaudRatePrescaler_4);
	NRF_WriteBuf(NRF_WRITE_REG + TX_ADDR, buf, 5);//д��5���ֽڵĵ�ַ
	NRF_ReadBuf(TX_ADDR, buf1, 5); 							//����д��ĵ�ַ

	for (i = 0; i < 5; i++){//�Ƚ�
		if (buf1[i] != 0xC2)
			break;
	}
	if (i == 5)
		return SUCCESS;        //MCU��NRF�ɹ����� 
	else
		return ERROR;        //MCU��NRF����������
}

//NRF24L01����
void NRF24L01::NRF_Reset(void){

	u8 state;
	state = NRF_ReadReg(STATUS);								//��NRF24L01״̬��־
	NRF_WriteReg(NRF_WRITE_REG + STATUS, state);	//���NRF24L01�жϱ�־	
	NRF_WriteReg(FLUSH_TX, NOP); //���TX FIFO������
	NRF_WriteReg(FLUSH_RX, NOP); //���RX FIFO������

	NRF_CE_LOW();
	NRF_CSN_HIGH();
}

void NRF24L01::SetIRQPin(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin){

	NVIC_InitTypeDef NVIC_InitStructure;
	EXTI_InitTypeDef EXTI_InitStructure;

	IRQPin = GPIO(GPIOx, GPIO_Pin, GPIO_Mode_IPU, GPIO_Speed_50MHz);
	//��GPIO�ܽ����ⲿ�ж�������
	GPIO_EXTILineConfig(GPIO_PortSourceGPIOA, GPIO_PinSource2);

	//Ƕ���ж���������   ������ɻ��߽�����ɺ��ж�����Ϊ�ɸߵ�ƽ��Ϊ�͵�ƽ IRQ�õ�
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_1);
	NVIC_InitStructure.NVIC_IRQChannel = EXTI2_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
	NVIC_Init(&NVIC_InitStructure);

	//�ⲿ�ж�����
	EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
	EXTI_InitStructure.EXTI_Line = EXTI_Line2;
	EXTI_InitStructure.EXTI_LineCmd = ENABLE;
	EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Falling;
	EXTI_Init(&EXTI_InitStructure);
	EXTI_ClearITPendingBit(EXTI_Line2);
}

void NRF24L01::IRQIrq(void){

	u8 istatus;
	static u8 dataToSend[RX_PLOAD_WIDTH];
	if (EXTI_GetITStatus(EXTI_Line2) != RESET){
		if (IRQPin.GetLevel() == 0){
			istatus = NRF_ReadReg(STATUS);
			nrf_flag = istatus;
			if (istatus & 0x40){	//���ݽ����ж�
				NRF_ReadBuf(RD_RX_PLOAD, TempBuf, RX_PLOAD_WIDTH);//��ȡ����
				NRF_WriteReg(FLUSH_RX, 0xff);//���RX FIFO�Ĵ���
				
				NRFbufferRx.Puts(TempBuf,RX_PLOAD_WIDTH);//���뻺����
			}
			else if ((istatus & 0x10) > 0){//�ﵽ����ʹ����ж�
				NRF_WriteReg(FLUSH_TX, 0xff);//���TX FIFO�Ĵ���
				isSending = false;
			}
			else if ((istatus & 0x20) > 0){//TX��������ж�
				NRF_WriteReg(FLUSH_TX, 0xff);//���TX FIFO�Ĵ���
				if ((NRFbufferTx.Size() > 0) && (NRFbufferTx.Size() <= TX_PLOAD_WIDTH)){
					u8 size = NRFbufferTx.Size();
					NRFbufferTx.Gets(dataToSend, size);
					NRF_WriteBuf(WR_TX_PLOAD, dataToSend, size);
				}
				else if (NRFbufferTx.Size() > TX_PLOAD_WIDTH){
					NRFbufferTx.Gets(dataToSend, TX_PLOAD_WIDTH);
					NRF_WriteBuf(WR_TX_PLOAD, dataToSend, TX_PLOAD_WIDTH);
				}
				else if (NRFbufferTx.Size() == 0){
					isSending = false;
				}
			}
			NRF_WriteReg(NRF_WRITE_REG + STATUS, istatus);//���״̬�Ĵ���
		}
		EXTI_ClearITPendingBit(EXTI_Line2);
	}
}

void NRF24L01::SetCEPin(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin){
	CEPin = GPIO(GPIOx, GPIO_Pin, GPIO_Mode_Out_PP, GPIO_Speed_50MHz);
}

bool NRF24L01::GetReceivedData(u8* buffer, u16 number){
	if (NRFbufferRx.Size() < number){
		return false;
	}
	else {
		NRFbufferRx.Gets(buffer, number);
		return true;
	}
}

u16 NRF24L01::ReceiveBufferSize(){
	return NRFbufferRx.Size();
}

u16 NRF24L01::SendBufferSize(){
	return NRFbufferTx.Size();
}

void NRF24L01::ClearReceiveBuffer(){
	NRFbufferRx.Clear();
}

void NRF24L01::ClearSendBuffer(){
	NRFbufferTx.Clear();
}

