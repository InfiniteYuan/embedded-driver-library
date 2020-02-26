// Copyright 2019 Espressif Systems (Shanghai) PTE LTD
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
#include <string.h>
#include "driver/i2c.h"
#include "esp_log.h"
#include "iot_i2c_bus.h"

#include "iot_mpr121.h"

static const char* TAG = "iot_mpr121";


#define MPR121_CHECK(a, str, ret)  if(a) {                                     \
    ESP_LOGE(TAG,"%s:%d (%s):%s", __FILE__, __LINE__, __FUNCTION__, str);      \
    return (ret);                                                              \
    }

#define WRITE_BIT      I2C_MASTER_WRITE         /*!< I2C master write */
#define READ_BIT       I2C_MASTER_READ          /*!< I2C master read */
#define ACK_CHECK_EN   0x1                      /*!< I2C master will check ack from slave*/
#define ACK_CHECK_DIS  0x0                      /*!< I2C master will not check ack from slave */
#define ACK_VAL        0x0                      /*!< I2C ack value */
#define NACK_VAL       0x1                      /*!< I2C nack value */
#define I2C_TIMEOUT_MS 1000                     /*!< I2C timeout ms */

#define MPR121_RESET_CMD 0x63

typedef struct {
    i2c_bus_handle_t bus;
    uint8_t dev_addr;
} mpr121_dev_t;

static mpr121_settings_t g_settings = {0};

mpr121_handle_t iot_mpr121_create(i2c_bus_handle_t bus, uint8_t dev_addr)
{
    mpr121_dev_t* seg = (mpr121_dev_t*) calloc(1, sizeof(mpr121_dev_t));
    seg->bus = bus;
    seg->dev_addr = dev_addr;
    return (mpr121_handle_t) seg;
}

esp_err_t iot_mpr121_delete(mpr121_handle_t dev, bool del_bus)
{
    mpr121_dev_t* seg = (mpr121_dev_t*) dev;
    if (del_bus) {
        iot_i2c_bus_delete(seg->bus);
        seg->bus = NULL;
    }
    free(seg);
    return ESP_OK;
}

esp_err_t iot_mpr121_write_byte(mpr121_handle_t dev, uint8_t address, uint8_t data)
{
    mpr121_dev_t* seg = (mpr121_dev_t*) dev;
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (seg->dev_addr << 1) | WRITE_BIT, ACK_CHECK_EN);
    i2c_master_write_byte(cmd, address, ACK_CHECK_EN);
    i2c_master_write_byte(cmd, data, ACK_CHECK_EN);
    i2c_master_stop(cmd);
    int ret = iot_i2c_bus_cmd_begin(seg->bus, cmd, I2C_TIMEOUT_MS/portTICK_RATE_MS);
    i2c_cmd_link_delete(cmd);
    return ret;
}

esp_err_t iot_mpr121_read_byte(mpr121_handle_t dev, uint8_t address, uint8_t *data)
{
    return iot_mpr121_read(dev, address, data, 1);
}

esp_err_t iot_mpr121_read(mpr121_handle_t dev, uint8_t address, uint8_t *buf, uint8_t len)
{
    mpr121_dev_t* seg = (mpr121_dev_t*) dev;
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (seg->dev_addr << 1) | WRITE_BIT, ACK_CHECK_EN);
    i2c_master_write_byte(cmd, address, ACK_CHECK_EN);
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (seg->dev_addr << 1) | READ_BIT, ACK_CHECK_EN);
    i2c_master_read(cmd, buf,len, I2C_MASTER_LAST_NACK);
    i2c_master_stop(cmd);
    int ret = iot_i2c_bus_cmd_begin(seg->bus, cmd, I2C_TIMEOUT_MS/portTICK_RATE_MS);
    i2c_cmd_link_delete(cmd);
    return ret;
}

esp_err_t iot_mpr121_config(mpr121_handle_t dev, mpr121_settings_t *settings)
{
    esp_err_t ret = ESP_OK;
    memcpy(&g_settings, settings, sizeof(mpr121_settings_t));

    iot_mpr121_set_reset(dev);

    iot_mpr121_write_byte(dev, MPR121_MHDR, settings->mhdr); //maximum half delta
    MPR121_CHECK(ret != ESP_OK, "iot_mpr121_write_byte MPR121_MHDR error", ESP_FAIL);
    iot_mpr121_write_byte(dev, MPR121_NHDR, settings->nhdr); //noise half delta
    MPR121_CHECK(ret != ESP_OK, "iot_mpr121_write_byte MPR121_NHDR error", ESP_FAIL);
    iot_mpr121_write_byte(dev, MPR121_NCLR, settings->nclr); //noise count limit
    MPR121_CHECK(ret != ESP_OK, "iot_mpr121_write_byte MPR121_NCLR error", ESP_FAIL);
    iot_mpr121_write_byte(dev, MPR121_FDLR, settings->fdlr); //filter delay
    MPR121_CHECK(ret != ESP_OK, "iot_mpr121_write_byte MPR121_FDLR error", ESP_FAIL);

    iot_mpr121_write_byte(dev, MPR121_MHDF, settings->mhdf); //maximum half delta
    MPR121_CHECK(ret != ESP_OK, "iot_mpr121_write_byte MPR121_MHDF error", ESP_FAIL);
    iot_mpr121_write_byte(dev, MPR121_NHDF, settings->nhdf); //noise half delta
    MPR121_CHECK(ret != ESP_OK, "iot_mpr121_write_byte MPR121_NHDF error", ESP_FAIL);
    iot_mpr121_write_byte(dev, MPR121_NCLF, settings->nclf); //noise count limit
    MPR121_CHECK(ret != ESP_OK, "iot_mpr121_write_byte MPR121_NCLF error", ESP_FAIL);
    iot_mpr121_write_byte(dev, MPR121_FDLF, settings->fdlf); //filter delay
    MPR121_CHECK(ret != ESP_OK, "iot_mpr121_write_byte MPR121_FDLF error", ESP_FAIL);

    iot_mpr121_write_byte(dev, MPR121_NHDT, settings->nhdt); //noise half delta
    MPR121_CHECK(ret != ESP_OK, "iot_mpr121_write_byte MPR121_NHDT error", ESP_FAIL);
    iot_mpr121_write_byte(dev, MPR121_NCLT, settings->nclt); //noise count limit
    MPR121_CHECK(ret != ESP_OK, "iot_mpr121_write_byte MPR121_NCLT error", ESP_FAIL);
    iot_mpr121_write_byte(dev, MPR121_FDLT, settings->fdlt); //filter delay
    MPR121_CHECK(ret != ESP_OK, "iot_mpr121_write_byte MPR121_FDLT error", ESP_FAIL);

    iot_mpr121_write_byte(dev, MPR121_MHDPROXR, settings->mhdproxr); //maximum half delta
    MPR121_CHECK(ret != ESP_OK, "iot_mpr121_write_byte MPR121_MHDPROXR error", ESP_FAIL);
    iot_mpr121_write_byte(dev, MPR121_NHDPROXR, settings->nhdproxr); //noise half delta
    MPR121_CHECK(ret != ESP_OK, "iot_mpr121_write_byte MPR121_MHDPROXR error", ESP_FAIL);
    iot_mpr121_write_byte(dev, MPR121_NCLPROXR, settings->nclproxr); //noise count limit
    MPR121_CHECK(ret != ESP_OK, "iot_mpr121_write_byte MPR121_MHDPROXR error", ESP_FAIL);
    iot_mpr121_write_byte(dev, MPR121_FDLPROXR, settings->fdlproxr); //filter delay
    MPR121_CHECK(ret != ESP_OK, "iot_mpr121_write_byte MPR121_MHDPROXR error", ESP_FAIL);

    iot_mpr121_write_byte(dev, MPR121_MHDPROXF, settings->mhdproxf); //maximum half delta
    MPR121_CHECK(ret != ESP_OK, "iot_mpr121_write_byte MPR121_MHDPROXF error", ESP_FAIL);
    iot_mpr121_write_byte(dev, MPR121_NHDPROXF, settings->nhdproxf); //noise half delta
    MPR121_CHECK(ret != ESP_OK, "iot_mpr121_write_byte MPR121_NHDPROXF error", ESP_FAIL);
    iot_mpr121_write_byte(dev, MPR121_NCLPROXF, settings->nclproxf); //noise count limit
    MPR121_CHECK(ret != ESP_OK, "iot_mpr121_write_byte MPR121_NCLPROXF error", ESP_FAIL);
    iot_mpr121_write_byte(dev, MPR121_FDLPROXF, settings->fdlproxf); //filter delay
    MPR121_CHECK(ret != ESP_OK, "iot_mpr121_write_byte MPR121_FDLPROXF error", ESP_FAIL);

    iot_mpr121_write_byte(dev, MPR121_NHDPROXT, settings->nhdproxt); //noise half delta
    MPR121_CHECK(ret != ESP_OK, "iot_mpr121_write_byte MPR121_NHDPROXT error", ESP_FAIL);
    iot_mpr121_write_byte(dev, MPR121_NCLPROXT, settings->nclproxt); //noise count limit
    MPR121_CHECK(ret != ESP_OK, "iot_mpr121_write_byte MPR121_NCLPROXT error", ESP_FAIL);
    iot_mpr121_write_byte(dev, MPR121_FDLPROXT, settings->fdlproxt); //filter delay
    MPR121_CHECK(ret != ESP_OK, "iot_mpr121_write_byte MPR121_FDLPROXT error", ESP_FAIL);

    iot_mpr121_write_byte(dev, MPR121_DTR, (((settings->dr)<<4)|settings->dt));
    MPR121_CHECK(ret != ESP_OK, "iot_mpr121_write_byte MPR121_DTR error", ESP_FAIL);

    iot_mpr121_write_byte(dev, MPR121_AFE1, (((settings->ffi)<<6)|settings->cdc));
    MPR121_CHECK(ret != ESP_OK, "iot_mpr121_write_byte MPR121_AFE1 error", ESP_FAIL);
    iot_mpr121_write_byte(dev, MPR121_AFE2, (((settings->cdt)<<5)|((settings->sfi)<<3)|settings->esi));
    MPR121_CHECK(ret != ESP_OK, "iot_mpr121_write_byte MPR121_AFE2 error", ESP_FAIL);

    iot_mpr121_write_byte(dev, MPR121_USL, settings->usl);
    MPR121_CHECK(ret != ESP_OK, "iot_mpr121_write_byte MPR121_USL error", ESP_FAIL);
    iot_mpr121_write_byte(dev, MPR121_LSL, settings->lsl);
    MPR121_CHECK(ret != ESP_OK, "iot_mpr121_write_byte MPR121_LSL error", ESP_FAIL);
    iot_mpr121_write_byte(dev, MPR121_TL, settings->tl);
    MPR121_CHECK(ret != ESP_OK, "iot_mpr121_write_byte MPR121_TL error", ESP_FAIL);

    iot_mpr121_write_byte(dev, MPR121_ACCR0, (((settings->ffi)<<6)|((settings->retry)<<4)|((settings->cl)<<2)|((settings->are)<<1)|settings->ace));
    MPR121_CHECK(ret != ESP_OK, "iot_mpr121_write_byte MPR121_ACCR0 error", ESP_FAIL);
    iot_mpr121_write_byte(dev, MPR121_ACCR1, (((settings->scts)<<7)|((settings->oorie)<<2)|((settings->arfie)<<1)|settings->acfie));
    MPR121_CHECK(ret != ESP_OK, "iot_mpr121_write_byte MPR121_ACCR1 error", ESP_FAIL);

    iot_mpr121_set_touch_hreshold(dev, settings->tthresh);
    iot_mpr121_set_release_hreshold(dev, settings->rthresh);

    iot_mpr121_write_byte(dev, MPR121_ECR, (((settings->cl)<<6)|((settings->eleporx_en)<<4)|settings->ele_en));
    MPR121_CHECK(ret != ESP_OK, "iot_mpr121_write_byte MPR121_ECR error", ESP_FAIL);

    return ESP_OK;
}

esp_err_t iot_mpr121_set_pin_mode(mpr121_handle_t dev, mpr121_electrode_t electrode, mpr121_pin_mode_t mode)
{
    MPR121_CHECK(electrode < MPR121_ELE_4, "iot_mpr121_set_pin_mode electrode error", ESP_FAIL);
    uint8_t bitmask = 1<<(electrode - MPR121_ELE_4);
    uint8_t temp_buf = 0;

    switch(mode){
    case MPR121_INPUT:
        iot_mpr121_read_byte(dev, MPR121_EN, &temp_buf);
        iot_mpr121_write_byte(dev, MPR121_EN,  temp_buf| bitmask);
        iot_mpr121_read_byte(dev, MPR121_DIR, &temp_buf);
        iot_mpr121_write_byte(dev, MPR121_DIR, temp_buf & ~bitmask);
        iot_mpr121_read_byte(dev, MPR121_CTL0, &temp_buf);
        iot_mpr121_write_byte(dev, MPR121_CTL0, temp_buf & ~bitmask);
        iot_mpr121_read_byte(dev, MPR121_CTL1, &temp_buf);
        iot_mpr121_write_byte(dev, MPR121_CTL1, temp_buf & ~bitmask);
        break;

    case MPR121_INPUT_PULLUP:
        iot_mpr121_read_byte(dev, MPR121_EN, &temp_buf);
        iot_mpr121_write_byte(dev, MPR121_EN,  temp_buf| bitmask);
        iot_mpr121_read_byte(dev, MPR121_DIR, &temp_buf);
        iot_mpr121_write_byte(dev, MPR121_DIR, temp_buf & ~bitmask);
        iot_mpr121_read_byte(dev, MPR121_CTL0, &temp_buf);
        iot_mpr121_write_byte(dev, MPR121_CTL0, temp_buf | bitmask);
        iot_mpr121_read_byte(dev, MPR121_CTL1, &temp_buf);
        iot_mpr121_write_byte(dev, MPR121_CTL1, temp_buf | bitmask);
        break;

    case MPR121_INPUT_PULLDOWN:
        iot_mpr121_read_byte(dev, MPR121_EN, &temp_buf);
        iot_mpr121_write_byte(dev, MPR121_EN,  temp_buf| bitmask);
        iot_mpr121_read_byte(dev, MPR121_DIR, &temp_buf);
        iot_mpr121_write_byte(dev, MPR121_DIR, temp_buf & ~bitmask);
        iot_mpr121_read_byte(dev, MPR121_CTL0, &temp_buf);
        iot_mpr121_write_byte(dev, MPR121_CTL0, temp_buf | bitmask);
        iot_mpr121_read_byte(dev, MPR121_CTL1, &temp_buf);
        iot_mpr121_write_byte(dev, MPR121_CTL1, temp_buf & ~bitmask);
        break;

    case MPR121_OUTPUT_HIGHSIDE:
        iot_mpr121_read_byte(dev, MPR121_EN, &temp_buf);
        iot_mpr121_write_byte(dev, MPR121_EN,  temp_buf| bitmask);
        iot_mpr121_read_byte(dev, MPR121_DIR, &temp_buf);
        iot_mpr121_write_byte(dev, MPR121_DIR, temp_buf | bitmask);
        iot_mpr121_read_byte(dev, MPR121_CTL0, &temp_buf);
        iot_mpr121_write_byte(dev, MPR121_CTL0, temp_buf | bitmask);
        iot_mpr121_read_byte(dev, MPR121_CTL1, &temp_buf);
        iot_mpr121_write_byte(dev, MPR121_CTL1, temp_buf | bitmask);
        break;

    case MPR121_OUTPUT_LOWSIDE:
        iot_mpr121_read_byte(dev, MPR121_EN, &temp_buf);
        iot_mpr121_write_byte(dev, MPR121_EN,  temp_buf| bitmask);
        iot_mpr121_read_byte(dev, MPR121_DIR, &temp_buf);
        iot_mpr121_write_byte(dev, MPR121_DIR, temp_buf | bitmask);
        iot_mpr121_read_byte(dev, MPR121_CTL0, &temp_buf);
        iot_mpr121_write_byte(dev, MPR121_CTL0, temp_buf | bitmask);
        iot_mpr121_read_byte(dev, MPR121_CTL1, &temp_buf);
        iot_mpr121_write_byte(dev, MPR121_CTL1, temp_buf & ~bitmask);
        break;

    case MPR121_OUTPUT:
        iot_mpr121_read_byte(dev, MPR121_EN, &temp_buf);
        iot_mpr121_write_byte(dev, MPR121_EN,  temp_buf| bitmask);
        iot_mpr121_read_byte(dev, MPR121_DIR, &temp_buf);
        iot_mpr121_write_byte(dev, MPR121_DIR, temp_buf | bitmask);
        iot_mpr121_read_byte(dev, MPR121_CTL0, &temp_buf);
        iot_mpr121_write_byte(dev, MPR121_CTL0, temp_buf & ~bitmask);
        iot_mpr121_read_byte(dev, MPR121_CTL1, &temp_buf);
        iot_mpr121_write_byte(dev, MPR121_CTL1, temp_buf & ~bitmask);
        break;
    default:
        break;
  }
  return ESP_OK;
}

esp_err_t iot_mpr121_get_touch_status(mpr121_handle_t dev, uint16_t *status)
{
    return iot_mpr121_read(dev, MPR121_TS1, (uint8_t *)status, 2);
}

esp_err_t iot_mpr121_get_out_of_range(mpr121_handle_t dev,uint16_t *status)
{
    return iot_mpr121_read(dev, MPR121_OORS1, (uint8_t *)status, 2);
}

esp_err_t iot_mpr121_get_hreshold(mpr121_handle_t dev, uint8_t *hreshold, size_t hreshold_len)
{
    return iot_mpr121_read(dev, MPR121_E0RTH, hreshold, hreshold_len);
}

esp_err_t iot_mpr121_set_touch_hreshold(mpr121_handle_t dev, uint8_t touch)
{
    esp_err_t ret;
    for(mpr121_electrode_t i = MPR121_ELE_0; i < MPR121_ELE_MAX; i++){
        ret = iot_mpr121_write_byte(dev, MPR121_E0TTH + (i<<1), touch);
        MPR121_CHECK(ret != ESP_OK, "iot_mpr121_set_touch_hreshold error", ESP_FAIL);
    }
    g_settings.tthresh = touch;
    return ret;
}

esp_err_t iot_mpr121_set_release_hreshold(mpr121_handle_t dev, uint8_t release)
{
    esp_err_t ret;
    for(mpr121_electrode_t i = MPR121_ELE_0; i < MPR121_ELE_MAX; i++){
        ret = iot_mpr121_write_byte(dev, MPR121_E0RTH + (i<<1), release);
        MPR121_CHECK(ret != ESP_OK, "iot_mpr121_set_release_hreshold error", ESP_FAIL);
    }
    g_settings.rthresh = release;
    return ret;
}

esp_err_t iot_mpr121_get_filtered_data(mpr121_handle_t dev, int16_t *data, size_t data_len)
{
    return iot_mpr121_read(dev, MPR121_E0FDL, (uint8_t *)data, (data_len<<1));
}

esp_err_t iot_mpr121_get_electrode_filtered_data(mpr121_handle_t dev, mpr121_electrode_t electrode, uint16_t *data)
{
    return iot_mpr121_read(dev, MPR121_E0FDL + (electrode<<1), (uint8_t *)data, 2);
}

esp_err_t iot_mpr121_get_baseline_data(mpr121_handle_t dev, int16_t *data, size_t data_len)
{
    esp_err_t ret;
    int i = 0;
    uint8_t baseline[13];
    ret = iot_mpr121_read(dev, MPR121_E0BV, baseline, 13);
    for(i= 0; i<data_len; i++){
        *(data+i) = baseline[i]<<2;
    }
    return ret;
}

esp_err_t iot_mpr121_set_baseline_data(mpr121_handle_t dev, uint8_t baseline)
{
    esp_err_t ret;
    for(mpr121_electrode_t i = MPR121_ELE_0; i < MPR121_ELE_MAX; i++){
        ret = iot_mpr121_write_byte(dev, MPR121_E0BV + (i<<1), baseline);
        MPR121_CHECK(ret != ESP_OK, "iot_mpr121_set_release_hreshold error", ESP_FAIL);
    }
    return ret;
}

esp_err_t iot_mpr121_set_reset(mpr121_handle_t dev)
{
    return iot_mpr121_write_byte(dev, MPR121_SRST, MPR121_RESET_CMD);
}

esp_err_t iot_mpr121_set_run(mpr121_handle_t dev)
{
    return iot_mpr121_write_byte(dev, MPR121_ECR, (((g_settings.cl)<<6)|((g_settings.eleporx_en)<<4)|g_settings.ele_en));
}

esp_err_t iot_mpr121_set_stop(mpr121_handle_t dev)
{
    return iot_mpr121_write_byte(dev, MPR121_ECR, (((g_settings.cl)<<6)|((g_settings.eleporx_en)<<4)|MPR121_ELE_DIS));
}
