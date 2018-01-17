#include "NRF24L01.h"
#include "SPI.h"
#include "USART.h"

void RCC_Configuration(void){
	
	RCC_DeInit();
	RCC_HSEConfig(RCC_HSE_ON);

	if(RCC_WaitForHSEStartUp() == SUCCESS) {

		RCC_HCLKConfig(RCC_SYSCLK_Div1);
		RCC_PCLK2Config(RCC_HCLK_Div1);
		RCC_PCLK1Config(RCC_HCLK_Div2);

		RCC_PLLConfig(RCC_PLLSource_HSE_Div1, RCC_PLLMul_9);
		RCC_PLLCmd(ENABLE);
		while(RCC_GetFlagStatus(RCC_FLAG_PLLRDY) == RESET);

		RCC_SYSCLKConfig(RCC_SYSCLKSource_PLLCLK);
		while(RCC_GetSYSCLKSource() != 0x08);
	}
	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB,ENABLE);	//��GPIOBʱ��
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO,ENABLE);		//��AFIOʱ��
}

int main(void){
	
	SystemInit();
	RCC_Configuration();
	SPI spi(SPI1);
	NRF24L01 nrf(spi);
	nrf.SetCEPin(GPIOA,1);
	USART usart1(1,115200,false);//Can't use DMA?
	nrf.NRF_Reset();
	nrf.NRF_TX_Mode();
	usart1<<"USART init complete!\n";
	usart1<<nrf.NRF_ReadReg(STATUS);
	if(nrf.NRF_Check() == 0){
		usart1<<"������\n";
	}else {
		usart1<<"����\n";
	}
	usart1<<nrf.NRF_Check();
	while(1){
		usart1<<nrf.NRF_Check();
	}
}

void HardFault_Handler(void)  
{  
   
  if (CoreDebug->DHCSR & 1) {  //check C_DEBUGEN == 1 -> Debugger Connected  
      __breakpoint(0);  // halt program execution here         
  }  
  while (1)  
  {  
  }  
}  




