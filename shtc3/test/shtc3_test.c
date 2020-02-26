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
#include <stdio.h>
#include "unity.h"
#include "esp_system.h"
#include "driver/i2c.h"

#include "i2c_bus.h"
#include "shtc3.h"

#define I2C_MASTER_SCL_IO           17          /*!< gpio number for I2C master clock */
#define I2C_MASTER_SDA_IO           16          /*!< gpio number for I2C master data  */
#define I2C_MASTER_NUM              I2C_NUM_1   /*!< I2C port number for master dev */
#define I2C_MASTER_TX_BUF_DISABLE   0           /*!< I2C master do not need buffer */
#define I2C_MASTER_RX_BUF_DISABLE   0           /*!< I2C master do not need buffer */
#define I2C_MASTER_FREQ_HZ          400000      /*!< I2C master clock frequency */

static i2c_bus_handle_t i2c_bus = NULL;
static shtc3_handle_t shtc3 = NULL;

/**
 * @brief i2c master initialization
 */
void i2c_sensor_shtc3_init()
{
    int i2c_master_port = I2C_MASTER_NUM;
    i2c_config_t conf;
    conf.mode = I2C_MODE_MASTER;
    conf.sda_io_num = I2C_MASTER_SDA_IO;
    conf.sda_pullup_en = GPIO_PULLUP_ENABLE;
    conf.scl_io_num = I2C_MASTER_SCL_IO;
    conf.scl_pullup_en = GPIO_PULLUP_ENABLE;
    conf.master.clk_speed = I2C_MASTER_FREQ_HZ;
    i2c_bus = i2c_bus_create(i2c_master_port, &conf);
    shtc3 = iot_shtc3_create(i2c_bus, SHTC3_I2C_ADDRESS);
}

void shtc3_test()
{
    esp_log_level_set("*", ESP_LOG_INFO);
    esp_log_level_set("shtc3", ESP_LOG_DEBUG);

    uint16_t device_id = 0;
    float hum = 0;
    float temp = 0;
    i2c_sensor_shtc3_init();
    iot_shtc3_wakeup(shtc3);
    vTaskDelay(100);
    iot_shtc3_soft_reset(shtc3);
    vTaskDelay(100);
    iot_shtc3_get_deviceid(shtc3, &device_id);
    ESP_LOGI("shtc3", "ID: %04x", device_id);
    iot_shtc3_sleep(shtc3);
    while (1) {
        iot_shtc3_wakeup(shtc3);
        vTaskDelay(100); // must delay after wakeup shtc3
        iot_shtc3_get_temperature_humidity(shtc3, &temp, &hum);
        ESP_LOGI("shtc3", "hum: %f， temp: %f", hum, temp);
        iot_shtc3_sleep(shtc3);
        vTaskDelay(1000);
    }

    iot_shtc3_get_temperature_humidity(shtc3, &hum, &temp);
    ESP_LOGI("shtc3", "hum: %f， temp: %f", hum, temp);
}

TEST_CASE("Sensor shtc3 test", "[shtc3][sensor]")
{
    shtc3_test();
}

