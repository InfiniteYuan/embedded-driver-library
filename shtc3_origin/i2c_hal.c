//==============================================================================
//    S E N S I R I O N   AG,  Laubisruetistr. 50, CH-8712 Staefa, Switzerland
//==============================================================================
// Project   :  SHTC3 Sample Code (V1.0)
// File      :  i2c_hal.c (V1.0)
// Author    :  RFU
// Date      :  24-Nov-2017
// Controller:  STM32F100RB
// IDE       :  µVision V5.17.0.0
// Compiler  :  Armcc
// Brief     :  I2C hardware abstraction layer
//==============================================================================

#include "i2c_hal.h"

//-- Defines for IO-Pins -------------------------------------------------------
// I2C IO-Pins                         /* -- adapt the defines for your uC -- */

// SDA on port B, bit 9
#define SDA_LOW()  (GPIOB->BSRR = 0x02000000) // set SDA to low
#define SDA_OPEN() (GPIOB->BSRR = 0x00000200) // set SDA to open-drain
#define SDA_READ   (GPIOB->IDR  & 0x0200)     // read SDA

// SCL on port B, bit 8                /* -- adapt the defines for your uC -- */
#define SCL_LOW()  (GPIOB->BSRR = 0x01000000) // set SCL to low
#define SCL_OPEN() (GPIOB->BSRR = 0x00000100) // set SCL to open-drain
#define SCL_READ   (GPIOB->IDR  & 0x0100)     // read SCL

//------------------------------------------------------------------------------
void I2c_Init(void)                       /* -- adapt the init for your uC -- */
{
  RCC->APB2ENR |= 0x00000008;  // I/O port B clock enabled

  SDA_OPEN();                  // I2C-bus idle mode SDA released
  SCL_OPEN();                  // I2C-bus idle mode SCL released

  // SDA on port B, bit 9
  // SCL on port B, bit 8
  GPIOB->CRH   &= 0xFFFFFF00;  // set open-drain output for SDA and SCL
  GPIOB->CRH   |= 0x00000055;  //
}

//------------------------------------------------------------------------------
void I2c_StartCondition(void){
  SDA_OPEN();
  DelayMicroSeconds(1);
  SCL_OPEN();
  DelayMicroSeconds(1);
  SDA_LOW();
  DelayMicroSeconds(10);  // hold time start condition (t_HD;STA)
  SCL_LOW();
  DelayMicroSeconds(10);
}

//------------------------------------------------------------------------------
void I2c_StopCondition(void){
  SCL_LOW();
  DelayMicroSeconds(1);
  SDA_LOW();
  DelayMicroSeconds(1);
  SCL_OPEN();
  DelayMicroSeconds(10);
  SDA_OPEN();
  DelayMicroSeconds(10);
}

//------------------------------------------------------------------------------
etError I2c_WriteByte(uint8_t txByte){
  etError error = NO_ERROR;

  for(uint8_t mask = 0x80; mask > 0; mask >>= 1) {
    // masking txByte
    if((mask & txByte) == 0) {
      SDA_LOW();  // write 0 to SDA-Line
    } else {
      SDA_OPEN(); // write 1 to SDA-Line
    }

    // data set-up time
    DelayMicroSeconds(1);

    // generate clock pulse on SCL
    SCL_OPEN();
    DelayMicroSeconds(5);
    SCL_LOW();

    // data hold time
    DelayMicroSeconds(1);
  }

  // release SDA-line
  SDA_OPEN();

  // ack reading
  SCL_OPEN();
  DelayMicroSeconds(1);

  // check ack from i2c slave
  if(SDA_READ) {
    error = ACK_ERROR;
  }

  SCL_LOW();

  return error;
}

//------------------------------------------------------------------------------
uint8_t I2c_ReadByte(etI2cAck ack){
  uint8_t rxByte = 0;

  // release SDA-line
  SDA_OPEN();

  for(uint8_t mask = 0x80; mask > 0; mask >>= 1) {
    SCL_OPEN();
    DelayMicroSeconds(1);

    // wait while clock stretching
    while(SCL_READ == 0);

    DelayMicroSeconds(3);

    // read bit
    if(SDA_READ) {
      rxByte = rxByte | mask;
    }

    SCL_LOW();
    DelayMicroSeconds(1);
  }

  // send acknowledge if necessary
  if(ack == ACK) {
    SDA_LOW();
  } else {
    SDA_OPEN();
  }

  // data set-up time
  DelayMicroSeconds(1);

  // generate clock pulse on SCL
  SCL_OPEN();
  DelayMicroSeconds(5);
  SCL_LOW();

  // release SDA-line
  SDA_OPEN();

  return rxByte;
}
