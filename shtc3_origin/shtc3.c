//==============================================================================
//    S E N S I R I O N   AG,  Laubisruetistr. 50, CH-8712 Staefa, Switzerland
//==============================================================================
// Project   :  SHTC3 Sample Code (V1.0)
// File      :  shtc3.c (V1.0)
// Author    :  RFU
// Date      :  24-Nov-2017
// Controller:  STM32F100RB
// IDE       :  µVision V5.17.0.0
// Compiler  :  Armcc
// Brief     :  Sensor Layer: Implementation of functions for sensor access.
//==============================================================================

#include "shtc3.h"
#include "i2c_hal.h"

typedef enum{
  READ_ID            = 0xEFC8, // command: read ID register
  SOFT_RESET         = 0x805D, // soft reset
  SLEEP              = 0xB098, // sleep
  WAKEUP             = 0x3517, // wakeup
  MEAS_T_RH_POLLING  = 0x7866, // meas. read T first, clock stretching disabled
  MEAS_T_RH_CLOCKSTR = 0x7CA2, // meas. read T first, clock stretching enabled
  MEAS_RH_T_POLLING  = 0x58E0, // meas. read RH first, clock stretching disabled
  MEAS_RH_T_CLOCKSTR = 0x5C24  // meas. read RH first, clock stretching enabled
}etCommands;

static etError SHTC3_StartWriteAccess(void);
static etError SHTC3_StartReadAccess(void);
static void SHTC3_StopAccess(void);
static etError SHTC3_Read2BytesAndCrc(uint16_t *data);
static etError SHTC3_WriteCommand(etCommands cmd);
static etError SHTC3_CheckCrc(uint8_t data[], uint8_t nbrOfBytes,
                              uint8_t checksum);
static float SHTC3_CalcTemperature(uint16_t rawValue);
static float SHTC3_CalcHumidity(uint16_t rawValue);

static uint8_t _Address;

//------------------------------------------------------------------------------
void SHTC3_Init(uint8_t address){
  _Address = address;
  I2c_Init(); // init I2C
}

//------------------------------------------------------------------------------
etError SHTC3_GetTempAndHumi(float *temp, float *humi){
  etError  error;        // error code
  uint16_t rawValueTemp; // temperature raw value from sensor
  uint16_t rawValueHumi; // humidity raw value from sensor

  error = SHTC3_StartWriteAccess();

  // measure, read temperature first, clock streching enabled
  error |= SHTC3_WriteCommand(MEAS_T_RH_CLOCKSTR);

  // if no error, read temperature and humidity raw values
  if(error == NO_ERROR) {
    error |= SHTC3_StartReadAccess();
    error |= SHTC3_Read2BytesAndCrc(&rawValueTemp);
    error |= SHTC3_Read2BytesAndCrc(&rawValueHumi);
  }

  SHTC3_StopAccess();

  // if no error, calculate temperature in °C and humidity in %RH
  if(error == NO_ERROR) {
    *temp = SHTC3_CalcTemperature(rawValueTemp);
    *humi = SHTC3_CalcHumidity(rawValueHumi);
  }

  return error;
}

//------------------------------------------------------------------------------
etError SHTC3_GetTempAndHumiPolling(float *temp, float *humi){
  etError  error;           // error code
  uint8_t  maxPolling = 20; // max. retries to read the measurement (polling)
  uint16_t rawValueTemp;    // temperature raw value from sensor
  uint16_t rawValueHumi;    // humidity raw value from sensor

  error  = SHTC3_StartWriteAccess();

  // measure, read temperature first, clock streching disabled (polling)
  error |= SHTC3_WriteCommand(MEAS_T_RH_POLLING);

  // if no error, ...
  if(error == NO_ERROR) {
    // poll every 1ms for measurement ready
    while(maxPolling--) {
      // check if the measurement has finished
      error = SHTC3_StartReadAccess();

      // if measurement has finished -> exit loop
      if(error == NO_ERROR) break;

      // delay 1ms
      DelayMicroSeconds(1000);
    }

    // if no error, read temperature and humidity raw values
    if(error == NO_ERROR) {
      error |= SHTC3_Read2BytesAndCrc(&rawValueTemp);
      error |= SHTC3_Read2BytesAndCrc(&rawValueHumi);
    }
  }

  SHTC3_StopAccess();

  // if no error, calculate temperature in °C and humidity in %RH
  if(error == NO_ERROR) {
    *temp = SHTC3_CalcTemperature(rawValueTemp);
    *humi = SHTC3_CalcHumidity(rawValueHumi);
  }

  return error;
}

//------------------------------------------------------------------------------
etError SHTC3_GetId(uint16_t *id){
  etError error; // error code

  error = SHTC3_StartWriteAccess();

  // write ID read command
  error |= SHTC3_WriteCommand(READ_ID);

  // if no error, read ID
  if(error == NO_ERROR) {
    SHTC3_StartReadAccess();
    error = SHTC3_Read2BytesAndCrc(id);
  }

  SHTC3_StopAccess();

  return error;
}

//------------------------------------------------------------------------------
etError SHTC3_Sleep(void) {
  etError error = SHTC3_StartWriteAccess();

  if(error == NO_ERROR) {
    error |= SHTC3_WriteCommand(SLEEP);
  }

  SHTC3_StopAccess();

  return error;
}

//------------------------------------------------------------------------------
etError SHTC3_Wakeup(void) {
  etError error = SHTC3_StartWriteAccess();

  if(error == NO_ERROR) {
    error |= SHTC3_WriteCommand(WAKEUP);
  }

  SHTC3_StopAccess();

  DelayMicroSeconds(100); // wait 100 us

  return error;
}

//------------------------------------------------------------------------------
etError SHTC3_SoftReset(void){
  etError error; // error code

  error = SHTC3_StartWriteAccess();

  // write reset command
  error |= SHTC3_WriteCommand(SOFT_RESET);

  SHTC3_StopAccess();

  return error;
}

//------------------------------------------------------------------------------
etError SHTC3_StartWriteAccess(void){
  etError error; // error code

  // write a start condition
  I2c_StartCondition();

  // write the sensor I2C address with the write flag
  error = I2c_WriteByte(_Address << 1);

  return error;
}

//------------------------------------------------------------------------------
etError SHTC3_StartReadAccess(void){
  etError error; // error code

  // write a start condition
  I2c_StartCondition();

  // write the sensor I2C address with the read flag
  error = I2c_WriteByte((_Address << 1) | 0x01);

  return error;
}

//------------------------------------------------------------------------------
void SHTC3_StopAccess(void){
  // write a stop condition
  I2c_StopCondition();
}

//------------------------------------------------------------------------------
static etError SHTC3_WriteCommand(etCommands cmd){
  etError error; // error code

  // write the upper 8 bits of the command to the sensor
  error = I2c_WriteByte(cmd >> 8);

  // write the lower 8 bits of the command to the sensor
  error |= I2c_WriteByte(cmd & 0xFF);

  return error;
}

//------------------------------------------------------------------------------
static etError SHTC3_Read2BytesAndCrc(uint16_t *data){
  etError error;    // error code
  uint8_t bytes[2]; // read data array
  uint8_t checksum; // checksum byte

  // read two data bytes and one checksum byte
  bytes[0] = I2c_ReadByte(ACK);
  bytes[1] = I2c_ReadByte(ACK);
  checksum = I2c_ReadByte(ACK);

  // verify checksum
  error = SHTC3_CheckCrc(bytes, 2, checksum);

  // combine the two bytes to a 16-bit value
  *data = (bytes[0] << 8) | bytes[1];

  return error;
}

//------------------------------------------------------------------------------
static etError SHTC3_CheckCrc(uint8_t data[], uint8_t nbrOfBytes,
                              uint8_t checksum){
  uint8_t bit;        // bit mask
  uint8_t crc = 0xFF; // calculated checksum
  uint8_t byteCtr;    // byte counter

  // calculates 8-Bit checksum with given polynomial
  for(byteCtr = 0; byteCtr < nbrOfBytes; byteCtr++) {
    crc ^= (data[byteCtr]);
    for(bit = 8; bit > 0; --bit) {
      if(crc & 0x80) {
        crc = (crc << 1) ^ CRC_POLYNOMIAL;
      } else {
        crc = (crc << 1);
      }
    }
  }

  // verify checksum
  if(crc != checksum) {
    return CHECKSUM_ERROR;
  } else {
    return NO_ERROR;
  }
}

//------------------------------------------------------------------------------
static float SHTC3_CalcTemperature(uint16_t rawValue){
  // calculate temperature [°C]
  // T = -45 + 175 * rawValue / 2^16
  return 175 * (float)rawValue / 65536.0f - 45.0f;
}

//------------------------------------------------------------------------------
static float SHTC3_CalcHumidity(uint16_t rawValue){
  // calculate relative humidity [%RH]
  // RH = rawValue / 2^16 * 100
  return 100 * (float)rawValue / 65536.0f;
}
