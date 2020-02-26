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

#include "esp_log.h"
#include "driver/i2c.h"

#include "shtc3.h"

static const char *TAG = "shtc3";
typedef struct {
    i2c_bus_handle_t bus;
    uint16_t dev_addr;
    uint8_t scl_io_num;
} shtc3_dev_t;

static esp_err_t iot_shtc3_check_crc(uint8_t data[], uint8_t number,
                                     uint8_t checkcrc)
{
    uint8_t bit, byte_index;    // bit mask, byte counter
    uint8_t crc = 0xFF;         // calculated checkcrc

    // calculates 8-Bit checkcrc with given polynomial
    for (byte_index = 0; byte_index < number; byte_index++) {
        crc ^= (data[byte_index]);
        for (bit = 8; bit > 0; --bit) {
            if (crc & 0x80) {
                crc = (crc << 1) ^ CRC_POLYNOMIAL;
            } else {
                crc = (crc << 1);
            }
        }
    }

    ESP_LOGD(TAG, "calc crc: %x, read crc: %x", crc, checkcrc);
    // verify checkcrc
    if (crc != checkcrc) {
        return ESP_FAIL;
    }
    return ESP_OK;
}

esp_err_t iot_shtc3_write_bytes(shtc3_handle_t sensor, uint16_t reg_addr,
                                uint8_t *data_buf, uint8_t write_num)
{
    shtc3_dev_t *sens = (shtc3_dev_t *) sensor;
    esp_err_t ret = ESP_OK;
    uint8_t i = 0;
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (sens->dev_addr << 1) | WRITE_BIT, ACK_CHECK_EN);
    i2c_master_write_byte(cmd, reg_addr >> 8, ACK_CHECK_EN);
    i2c_master_write_byte(cmd, reg_addr & 0xFF, ACK_CHECK_EN);
    for (i = 0; i < write_num; i++) {
        i2c_master_write_byte(cmd, data_buf[i], ACK_CHECK_EN);
    }
    i2c_master_stop(cmd);
    ret |= i2c_bus_cmd_begin(sens->bus, cmd, 1000 / portTICK_RATE_MS);
    i2c_cmd_link_delete(cmd);
    ESP_LOGD(TAG, "iot_shtc3_write_bytes ret: %d", ret);
    return ret;
}

esp_err_t iot_shtc3_write_command(shtc3_handle_t sensor, uint16_t reg_addr)
{
    shtc3_dev_t *sens = (shtc3_dev_t *) sensor;
    esp_err_t ret = ESP_OK;
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (sens->dev_addr << 1) | WRITE_BIT, ACK_CHECK_EN);
    i2c_master_write_byte(cmd, reg_addr >> 8, ACK_CHECK_EN);
    i2c_master_write_byte(cmd, reg_addr & 0xFF, ACK_CHECK_EN);
    i2c_master_stop(cmd);
    ret |= i2c_bus_cmd_begin(sens->bus, cmd, 1000 / portTICK_RATE_MS);
    i2c_cmd_link_delete(cmd);
    ESP_LOGD(TAG, "iot_shtc3_write_command reg_addr: %04x, ret: %d", reg_addr, ret);
    return ret;
}

esp_err_t iot_shtc3_read_bytes(shtc3_handle_t sensor, uint16_t reg_addr,
                               uint8_t *data_buf, uint8_t read_num)
{
    shtc3_dev_t *sens = (shtc3_dev_t *) sensor;
    esp_err_t ret = ESP_OK;
    uint8_t i = 0;
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (sens->dev_addr << 1) | WRITE_BIT, ACK_CHECK_EN);
    i2c_master_write_byte(cmd, reg_addr >> 8, ACK_CHECK_EN);
    i2c_master_write_byte(cmd, reg_addr & 0xFF, ACK_CHECK_EN);
    i2c_master_stop(cmd);
    ret |= i2c_bus_cmd_begin(sens->bus, cmd, 1000 / portTICK_RATE_MS);
    i2c_cmd_link_delete(cmd);
    if (ret == ESP_FAIL) {
        return ret;
    }

    cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (sens->dev_addr << 1) | READ_BIT, ACK_CHECK_EN);
    for (i = 0; i < read_num - 1; i++) {
        i2c_master_read_byte(cmd, &data_buf[i], ACK_VAL);
    }
    i2c_master_read_byte(cmd, &data_buf[i], NACK_VAL);
    i2c_master_stop(cmd);
    ret |= i2c_bus_cmd_begin(sens->bus, cmd, 1000 / portTICK_RATE_MS);
    i2c_cmd_link_delete(cmd);
    ESP_LOGD(TAG, "iot_shtc3_read_bytes ret: %d", ret);
    return ret;
}

esp_err_t iot_shtc3_get_deviceid(shtc3_handle_t sensor, uint16_t *deviceid)
{
    esp_err_t ret = ESP_OK;
    uint8_t id_reg_buf[3];
    ret |= iot_shtc3_read_bytes(sensor, READ_ID_REG, id_reg_buf, sizeof(id_reg_buf) / sizeof(uint8_t));
    ret |= iot_shtc3_check_crc(id_reg_buf, 2, id_reg_buf[2]);
    *deviceid = (id_reg_buf[0] << 8) | id_reg_buf[1];
    ESP_LOGD(TAG, "id[0]: %02x, id[1]: %02x, id[2]: %02x", id_reg_buf[0], id_reg_buf[1], id_reg_buf[2]);
    ESP_LOGD(TAG, "device id: 0x%04x", *deviceid);
    return ret;
}

esp_err_t iot_shtc3_get_humidity_temperature(shtc3_handle_t sensor, float *humidity, float *temperature)
{
    esp_err_t ret = ESP_OK;
    uint8_t data[6] = {0x00};
    uint16_t rawvaluetemp; // temperature raw value from sensor
    uint16_t rawvaluehumi; // humidity raw value from sensor

    ret |= iot_shtc3_read_bytes(sensor, MEAS_T_RH_CLOCKSTR_REG, data, sizeof(data) / sizeof(uint8_t));
    ret |= iot_shtc3_check_crc(data, 2, data[2]);
    ret |= iot_shtc3_check_crc(data + 3, 2, data[5]);
    rawvaluetemp = (data[0] << 8) | data[1];
    rawvaluehumi = (data[3] << 8) | data[4];
    ESP_LOGD(TAG, "ret: %d, data[0]: %02x, data[1]: %02x, data[2]: %02x, data[3]: %02x, data[4]: %02x, data[5]: %02x",
             ret, data[0], data[1], data[2], data[3], data[4], data[5]);

    // calculate temperature [�C]
    // T = -45 + 175 * rawValue / 2^16
    *temperature = 175 * (float)rawvaluetemp / 65536.0f - 45.0f;
    // calculate relative humidity [%RH]
    // RH = rawValue / 2^16 * 100
    *humidity = 100 * (float)rawvaluehumi / 65536.0f;

    return ret;
}

// ToDo:
esp_err_t iot_shtc3_get_temperature_humidity_polling(shtc3_handle_t sensor, float *temperature, float *humidity)
{
    // esp_err_t ret = ESP_OK;
    // uint8_t data[6] = {0x00};
    // uint16_t rawvaluetemp; // temperature raw value from sensor
    // uint16_t rawvaluehumi; // humidity raw value from sensor

    // shtc3_dev_t *sens = (shtc3_dev_t *) sensor;
    // uint8_t i = 0;
    // uint8_t  maxPolling = 20; // max. retries to read the measurement (polling)
    // uint16_t reg_addr = MEAS_T_RH_POLLING_REG;
    // i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    // i2c_master_start(cmd);
    // i2c_master_write_byte(cmd, (sens->dev_addr << 1) | WRITE_BIT, ACK_CHECK_EN);
    // i2c_master_write_byte(cmd, reg_addr >> 8, ACK_CHECK_EN);
    // i2c_master_write_byte(cmd, reg_addr & 0xFF, ACK_CHECK_EN);
    // i2c_master_stop(cmd);
    // ret = i2c_bus_cmd_begin(sens->bus, cmd, 1000 / portTICK_RATE_MS);
    // i2c_cmd_link_delete(cmd);
    // ESP_LOGD(TAG, "write address phase ret: %d", ret);

    // cmd = i2c_cmd_link_create();
    // i2c_master_start(cmd);
    // i2c_master_write_byte(cmd, (sens->dev_addr << 1) | READ_BIT, ACK_CHECK_EN);
    // while (maxPolling--) {
    //     ret |= i2c_bus_cmd_begin(sens->bus, cmd, 100 / portTICK_RATE_MS);
    //     ESP_LOGD(TAG, "start read address phase ret: %d", ret);
    //     if (ret == ESP_OK)
    //         break;
    // }

    // i2c_cmd_link_delete(cmd);
    // cmd = i2c_cmd_link_create();
    // for (i = 0; i < sizeof(data) / sizeof(uint8_t) - 1; i++) {
    // // for (i = 0; i < 3 - 1; i++) {
    //     i2c_master_read_byte(cmd, &data[i], ACK_VAL);
    // }
    // i2c_master_read_byte(cmd, &data[i], NACK_VAL);
    // i2c_master_stop(cmd);
    // ret = i2c_bus_cmd_begin(sens->bus, cmd, 1000 / portTICK_RATE_MS);
    // ESP_LOGD(TAG, "read value phase ret: %d", ret);

    // ret = iot_shtc3_check_crc(data, 2, data[2]);
    // ret = iot_shtc3_check_crc(data + 3, 2, data[5]);
    // rawvaluetemp = (data[0] << 8) | data[1];
    // rawvaluehumi = (data[3] << 8) | data[4];
    // ESP_LOGD(TAG, "ret: %d, data[0]: %02x, data[1]: %02x, data[2]: %02x, data[3]: %02x, data[4]: %02x, data[5]: %02x",
    //          ret, data[0], data[1], data[2], data[3], data[4], data[5]);

    // // calculate temperature [�C]
    // // T = -45 + 175 * rawValue / 2^16
    // *temperature = 175 * (float)rawvaluetemp / 65536.0f - 45.0f;
    // // calculate relative humidity [%RH]
    // // RH = rawValue / 2^16 * 100
    // *humidity = 100 * (float)rawvaluehumi / 65536.0f;

    return ESP_OK;
}

esp_err_t iot_shtc3_get_temperature_humidity(shtc3_handle_t sensor, float *temperature, float *humidity)
{
    esp_err_t ret = ESP_OK;
    uint8_t data[6] = {0x00};
    uint16_t rawvaluetemp; // temperature raw value from sensor
    uint16_t rawvaluehumi; // humidity raw value from sensor

    shtc3_dev_t *sens = (shtc3_dev_t *) sensor;
    uint8_t i = 0;
    uint8_t  maxPolling = 20; // max. retries to read the measurement (polling)
    uint16_t reg_addr = MEAS_T_RH_CLOCKSTR_REG;
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (sens->dev_addr << 1) | WRITE_BIT, ACK_CHECK_EN);
    i2c_master_write_byte(cmd, reg_addr >> 8, ACK_CHECK_EN);
    i2c_master_write_byte(cmd, reg_addr & 0xFF, ACK_CHECK_EN);
    i2c_master_stop(cmd);
    ret |= i2c_bus_cmd_begin(sens->bus, cmd, 1000 / portTICK_RATE_MS);
    i2c_cmd_link_delete(cmd);
    ESP_LOGD(TAG, "write address phase ret: %d", ret);

    cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (sens->dev_addr << 1) | READ_BIT, ACK_CHECK_EN);
    while (maxPolling--) {
        ret |= i2c_bus_cmd_begin(sens->bus, cmd, 1000 / portTICK_RATE_MS);
        ESP_LOGD(TAG, "start read address phase ret: %d", ret);
        if (ret == ESP_OK)
            break;
    }
    vTaskDelay(50);

    i2c_cmd_link_delete(cmd);
    cmd = i2c_cmd_link_create();
    for (i = 0; i < sizeof(data) / sizeof(uint8_t) - 1; i++) {
        i2c_master_read_byte(cmd, &data[i], ACK_VAL);
    }
    i2c_master_read_byte(cmd, &data[i], NACK_VAL);
    i2c_master_stop(cmd);
    ret |= i2c_bus_cmd_begin(sens->bus, cmd, 1000 / portTICK_RATE_MS);
    ESP_LOGD(TAG, "read value phase ret: %d", ret);

    ret |= iot_shtc3_check_crc(data, 2, data[2]);
    ret |= iot_shtc3_check_crc(data + 3, 2, data[5]);
    rawvaluetemp = (data[0] << 8) | data[1];
    rawvaluehumi = (data[3] << 8) | data[4];
    ESP_LOGD(TAG, "ret: %d, data[0]: %02x, data[1]: %02x, data[2]: %02x, data[3]: %02x, data[4]: %02x, data[5]: %02x",
             ret, data[0], data[1], data[2], data[3], data[4], data[5]);

    // calculate temperature [�C]
    // T = -45 + 175 * rawValue / 2^16
    *temperature = 175 * (float)rawvaluetemp / 65536.0f - 45.0f;
    // calculate relative humidity [%RH]
    // RH = rawValue / 2^16 * 100
    *humidity = 100 * (float)rawvaluehumi / 65536.0f;

    return ESP_OK;
}

esp_err_t iot_shtc3_soft_reset(shtc3_handle_t sensor)
{
    return iot_shtc3_write_command(sensor, SOFT_RESET_REG);
}

esp_err_t iot_shtc3_sleep(shtc3_handle_t sensor)
{
    return iot_shtc3_write_command(sensor, SLEEP_REG);
}

esp_err_t iot_shtc3_wakeup(shtc3_handle_t sensor)
{
    return iot_shtc3_write_command(sensor, WAKEUP_REG);
}

shtc3_handle_t iot_shtc3_create(i2c_bus_handle_t bus, uint16_t dev_addr)
{
    shtc3_dev_t *sensor = (shtc3_dev_t *) calloc(1, sizeof(shtc3_dev_t));
    sensor->bus = bus;
    sensor->dev_addr = dev_addr;
    return (shtc3_handle_t) sensor;
}

esp_err_t iot_shtc3_delete(shtc3_handle_t sensor, bool del_bus)
{
    shtc3_dev_t *sens = (shtc3_dev_t *) sensor;
    if (del_bus) {
        i2c_bus_delete(sens->bus);
        sens->bus = NULL;
    }
    free(sens);
    return ESP_OK;
}
