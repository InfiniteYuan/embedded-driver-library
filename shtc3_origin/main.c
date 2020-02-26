//==============================================================================
//    S E N S I R I O N   AG,  Laubisruetistr. 50, CH-8712 Staefa, Switzerland
//==============================================================================
// Project   :  SHTC3 Sample Code (V1.0)
// File      :  main.c (V1.0)
// Author    :  RFU
// Date      :  24-Nov-2017
// Controller:  STM32F100RB
// IDE       :  µVision V5.17.0.0
// Compiler  :  Armcc
// Brief     :  This code shows how to implement the basic commands for the
//              SHTC3 sensor chip.
//              Due to compatibility reasons the I2C interface is implemented
//              as "bit-banging" on normal I/O's. This code is written for an
//              easy understanding and is neither optimized for speed nor code
//              size.
//
// Porting to a different microcontroller (uC):
//   - adapt the led functions for your platform              in main.c
//   - adapt the port functions / definitions for your uC     in i2c_hal.h/.c
//   - adapt the timing of the delay function for your uC     in system.c
//   - adapt the SystemInit()                                 in system.c
//   - change the uC register definition file <stm32f10x.h>   in system.h
//==============================================================================

#include "system.h"
#include "shtc3.h"
#include "stdbool.h"

static void LedInit(void);
static void LedBlue(bool on);
static void LedGreen(bool on);

//------------------------------------------------------------------------------
int main(void)
{
  etError  error;       // error code
  uint16_t id;          // sensor ID
  float    temperature; // temperature
  float    humidity;    // relative humidity

  SystemInit();
  LedInit();

  // initalize sensor module with the i2c address 0x70
  SHTC3_Init(0x70);

  // wake up the sensor from sleep mode
  SHTC3_Wakeup();

  // demonstartion of SoftReset command
  error = SHTC3_SoftReset();

  // wait for sensor to reset
  DelayMicroSeconds(100);

  // demonstartion of GetId command
  error = SHTC3_GetId(&id);

  while(1) {
    // read temperature and relative humidity
    error = SHTC3_GetTempAndHumiPolling(&temperature, &humidity);

    // if no error occurs -> the green LED lights up
    LedGreen(error == NO_ERROR);

    // if the Relative Humidity is over 50% -> the blue LED lights up
    LedBlue(humidity > 50);

    // activate the sleep mode of the sensor to save energy
    SHTC3_Sleep();
    // wait 1 second
    DelayMicroSeconds(1000000);
    // wake up the sensor from sleep mode
    SHTC3_Wakeup();
  }
}

//------------------------------------------------------------------------------
static void LedInit(void)          /* -- adapt this code for your platform -- */
{
  RCC->APB2ENR |= 0x00000010;  // I/O port C clock enabled
  GPIOC->CRH   &= 0xFFFFFF00;  // set general purpose output mode for LEDs
  GPIOC->CRH   |= 0x00000011;  //
  GPIOC->BSRR   = 0x03000000;  // LEDs off
}

//------------------------------------------------------------------------------
static void LedBlue(bool on)       /* -- adapt this code for your platform -- */
{
  if(on) {
    GPIOC->BSRR = 0x00000100;
  } else {
    GPIOC->BSRR = 0x01000000;
  }
}

//------------------------------------------------------------------------------
static void LedGreen(bool on)      /* -- adapt this code for your platform -- */
{
  if(on) {
    GPIOC->BSRR = 0x00000200;
  } else {
    GPIOC->BSRR = 0x02000000;
  }
}
