/*
 * ESPRESSIF MIT License
 *
 * Copyright (c) 2017 <ESPRESSIF SYSTEMS (SHANGHAI) PTE LTD>
 *
 * Permission is hereby granted for use on ESPRESSIF SYSTEMS products only, in which case,
 * it is free of charge, to any person obtaining a copy of this software and associated
 * documentation files (the "Software"), to deal in the Software without restriction, including
 * without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense,
 * and/or sell copies of the Software, and to permit persons to whom the Software is furnished
 * to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all copies or
 * substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS
 * FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR
 * COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER
 * IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN
 * CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 *
 */
#ifndef _IOT_FT6206_H
#define _IOT_FT6206_H

#ifdef __cplusplus
extern "C"
{
#endif

#include "driver/i2c.h"
#include "iot_i2c_bus.h"

#define WRITE_BIT      I2C_MASTER_WRITE         /*!< I2C master write */
#define READ_BIT       I2C_MASTER_READ          /*!< I2C master read */
#define ACK_CHECK_EN   0x1                      /*!< I2C master will check ack from slave*/
#define ACK_CHECK_DIS  0x0                      /*!< I2C master will not check ack from slave */
#define ACK_VAL        0x0                      /*!< I2C ack value */
#define NACK_VAL       0x1                      /*!< I2C nack value */

#define FT62XX_ADDR            0x38
#define FT62XX_G_FT5201ID      0xA8
#define FT62XX_REG_NUMTOUCHES  0x02

#define FT62XX_NUM_X           0x33
#define FT62XX_NUM_Y           0x34

#define FT62XX_REG_MODE        0x00
#define FT62XX_REG_CALIBRATE   0x02
#define FT62XX_REG_WORKMODE    0x00
#define FT62XX_REG_FACTORYMODE 0x40
#define FT62XX_REG_THRESHHOLD  0x80
#define FT62XX_REG_POINTRATE   0x88
#define FT62XX_REG_FIRMVERS    0xA6
#define FT62XX_REG_CHIPID      0xA3
#define FT62XX_REG_VENDID      0xA8

#define FT62XX_VENDID  0x11
#define FT6206_CHIPID  0x06
#define FT6236_CHIPID  0x36
#define FT6236U_CHIPID 0x64 // mystery!

// calibrated for Adafruit 2.8" ctp screen
#define FT62XX_DEFAULT_THRESHOLD 128

typedef void* ft6206_handle_t; /*handle of ft6206*/

ft6206_handle_t iot_ft6206_create(i2c_bus_handle_t bus, uint16_t dev_addr);

esp_err_t iot_ft6206_delete(ft6206_handle_t dev, bool del_bus);

esp_err_t iot_ft6206_write_byte(ft6206_handle_t dev, uint8_t addr,
        uint8_t data);

esp_err_t iot_ft6206_write(ft6206_handle_t dev, uint8_t start_addr,
        uint8_t write_num, uint8_t *data_buf);

esp_err_t iot_ft6206_read_byte(ft6206_handle_t dev, uint8_t addr,
        uint8_t *data);

esp_err_t iot_ft6206_read(ft6206_handle_t dev, uint8_t start_addr,
        uint8_t read_num, uint8_t *data_buf);

#ifdef __cplusplus
}
#endif

#ifdef __cplusplus
class TS_Point
{
public:
    TS_Point(void);
    TS_Point(int16_t x, int16_t y, int16_t z);

    bool operator==(TS_Point);
    bool operator!=(TS_Point);

    int16_t x; /*!< X coordinate */
    int16_t y; /*!< Y coordinate */
    int16_t z; /*!< Z coordinate (often used for pressure) */
};

class CFT6206
{
private:
    ft6206_handle_t m_dev_handle;
    CI2CBus *bus;

    void writeRegister8(uint8_t reg, uint8_t val);
    uint8_t readRegister8(uint8_t reg);

    void readData(void);
    uint8_t touches;
    uint16_t touchX[2], touchY[2], touchID[2];

public:

    CFT6206(CI2CBus *p_i2c_bus, uint8_t addr = FT62XX_ADDR);
    bool begin(uint8_t thresh = FT62XX_DEFAULT_THRESHOLD);
    uint8_t touched(void);
    TS_Point getPoint(uint8_t n = 0);


};

#endif

#endif //ADAFRUIT_FT6206_LIBRARY

