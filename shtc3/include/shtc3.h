// Copyright 2015-2016 Espressif Systems (Shanghai) PTE LTD
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at

//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
#ifndef _SHTC3_H_
#define _SHTC3_H_
#ifdef __cplusplus
extern "C" {
#endif

#include "driver/i2c.h"
#include "i2c_bus.h"

#define WRITE_BIT      I2C_MASTER_WRITE  /*!< I2C master write */
#define READ_BIT       I2C_MASTER_READ   /*!< I2C master read */
#define ACK_CHECK_EN   0x1               /*!< I2C master will check ack from slave*/
#define ACK_CHECK_DIS  0x0               /*!< I2C master will not check ack from slave */
#define ACK_VAL        0x0               /*!< I2C ack value */
#define NACK_VAL       0x1               /*!< I2C nack value */

#define SHTC3_I2C_ADDRESS 0x70

#define CRC_POLYNOMIAL  0x131 // P(x) = x^8 + x^5 + x^4 + 1 = 100110001

typedef enum {
    READ_ID_REG            = 0xEFC8, // command: read ID register
    SOFT_RESET_REG         = 0x805D, // soft reset
    SLEEP_REG              = 0xB098, // sleep
    WAKEUP_REG             = 0x3517, // wakeup
    MEAS_T_RH_POLLING_REG  = 0x7866, // meas. read T first, clock stretching disabled
    MEAS_T_RH_CLOCKSTR_REG = 0x7CA2, // meas. read T first, clock stretching enabled
    MEAS_RH_T_POLLING_REG  = 0x58E0, // meas. read RH first, clock stretching disabled
    MEAS_RH_T_CLOCKSTR_REG = 0x5C24  // meas. read RH first, clock stretching enabled
} etCommands;

typedef void *shtc3_handle_t;

/**
 * @brief Write value to one register of shtc3
 *
 * @param sensor object handle of shtc3
 * @param reg_addr register address
 * @param data_buf register value point
 * @param write_num register value number
 *
 * @return
 *     - ESP_OK Success
 *     - ESP_FAIL Fail
 */
esp_err_t iot_shtc3_write_bytes(shtc3_handle_t sensor, uint16_t reg_addr, uint8_t *data_buf, uint8_t write_num);

/**
 * @brief Read value from register of shtc3
 *
 * @param sensor object handle of shtc3
 * @param reg_addr register address
 * @param data_buf register value point
 * @param read_num register value number
 *
 * @return
 *     - ESP_OK Success
 *     - ESP_FAIL Fail
 */
esp_err_t iot_shtc3_read_bytes(shtc3_handle_t sensor, uint16_t reg_addr, uint8_t *data_buf, uint8_t read_num);

/**
 * @brief Get device identification of shtc3
 *
 * @param sensor object handle of shtc3
 * @param deviceid a pointer of device ID
 *
 * @return
 *     - ESP_OK Success
 *     - ESP_FAIL Fail
 */
esp_err_t iot_shtc3_get_deviceid(shtc3_handle_t sensor, uint16_t *deviceid);

/**
 * @brief Read shtc3 Humidity and Temperature output registers, and calculate humidity and temperature
 *
 * @param sensor object handle of shtc3
 * @param temperature pointer to the returned temperature value
 * @param humidity pointer to the returned humidity value
 *
 * @return
 *     - ESP_OK Success
 *     - ESP_FAIL Fail
 */
esp_err_t iot_shtc3_get_temperature_humidity(shtc3_handle_t sensor, float *temperature, float *humidity);

/**
 * @brief Read shtc3 Humidity and Temperature output registers, and calculate humidity and temperature
 *
 * @param sensor object handle of shtc3
 * @param temperature pointer to the returned temperature value
 * @param humidity pointer to the returned humidity value
 *
 * @return
 *     - ESP_OK Success
 *     - ESP_FAIL Fail
 */
esp_err_t iot_shtc3_get_temperature_humidity_polling(shtc3_handle_t sensor, float *temperature, float *humidity);

/**
 * @brief Reset shtc3 sensor thought write command by software
 *
 * @param sensor object handle of shtc3
 *
 * @return
 *     - ESP_OK Success
 *     - ESP_FAIL Fail
 */
esp_err_t iot_shtc3_soft_reset(shtc3_handle_t sensor);

/**
 * @brief Read shtc3 Humidity output registers, and calculate humidity
 *
 * @param sensor object handle of shtc3
 * @param humidity pointer to the returned humidity value that must be divided by 10 to get the value in [%]
 *
 * @return
 *     - ESP_OK Success
 *     - ESP_FAIL Fail
 */
esp_err_t iot_shtc3_sleep(shtc3_handle_t sensor);

/**
 * @brief wakeup shtc3 sensor
 *
 * @param sensor object handle of shtc3
 *
 * @return
 *     - ESP_OK Success
 *     - ESP_FAIL Fail
 */
esp_err_t iot_shtc3_wakeup(shtc3_handle_t sensor);
/**
 * @brief Create and init sensor object and return a sensor handle
 *
 * @param bus I2C bus object handle
 * @param dev_addr I2C device address of sensor
 *
 * @return
 *     - NULL Fail
 *     - Others Success
 */
shtc3_handle_t iot_shtc3_create(i2c_bus_handle_t bus, uint16_t dev_addr);

/**
 * @brief Delete and release a sensor object
 *
 * @param sensor object handle of shtc3
 * @param del_bus Whether to delete the I2C bus
 *
 * @return
 *     - ESP_OK Success
 *     - ESP_FAIL Fail
 */
esp_err_t iot_shtc3_delete(shtc3_handle_t sensor, bool del_bus);

#ifdef __cplusplus
}
#endif

#endif
