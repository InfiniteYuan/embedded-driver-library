// Copyright 2017 Espressif Systems (Shanghai) PTE LTD
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
#include "math.h"
#include "sm2335.h"
#include "sdkconfig.h"

static const char *TAG = "SM2335";

#define SM2335_CHECK(a, str, ret)  if(!(a)) {                                      \
        ESP_LOGE(TAG,"%s:%d (%s):%s", __FILE__, __LINE__, __FUNCTION__, str);      \
        return (ret);                                                              \
    }

#define I2C_SDA_GPIO 20
#define I2C_SCL_GPIO 19
#define I2C_FREQ_HZ 100000
#define I2C_MASTER_TX_BUF_DISABLE 0             /*!< I2C master doesn't need buffer */
#define I2C_MASTER_RX_BUF_DISABLE 0             /*!< I2C master doesn't need buffer */
#define ACK_CHECK_EN 0x1                        /*!< I2C master will check ack from slave*/
#define ACK_CHECK_DIS 0x0                       /*!< I2C master will not check ack from slave */
#define ACK_VAL 0x0                             /*!< I2C ack value */
#define NACK_VAL 0x1                            /*!< I2C nack value */

/**
 * SM2335 register address
 */
#define REG_CONFIG_OUT1    0x00
#define REG_CONFIG_OUT2    0x01
#define REG_CONFIG_OUT3    0x02
#define REG_CONFIG_OUT4    0x03
#define REG_CONFIG_OUT5    0x04

#define _CHANNEL_MAX 5
#define _CHANNEL_INVALID 255

static i2c_port_t g_i2c_master_num = 0;
static int32_t g_ch2gpio[_CHANNEL_MAX] = {0};
static uint8_t g_is_stdby = 0;
static uint8_t g_max_current = 0x24;

#define LEDC_FIXED_Q (8)
#define FLOATINT_2_FIXED(X, Q) ((int)((X)*(0x1U << Q)))
#define FIXED_2_FLOATING(X, Q) ((int)((X)/(0x1U << Q)))
#define GET_FIXED_INTEGER_PART(X, Q) (X >> Q)
#define GET_FIXED_DECIMAL_PART(X, Q) (X & ((0x1U << Q) - 1))
#define LEDC_VALUE_TO_DUTY(value) (value * ((1 << LEDC_TIMER_8_BIT) - 1) / UINT16_MAX)
#define LEDC_DUTY_TO_VALUE(value) (value * UINT16_MAX / ((1 << LEDC_TIMER_8_BIT) - 1) )

typedef struct {
    int cur;
    int final;
    int step;
    int cycle;
    size_t num;
} ledc_fade_data_t;


typedef struct {
    ledc_fade_data_t fade_data[_CHANNEL_MAX];
} iot_light_t;

static DRAM_ATTR iot_light_t *g_light_config = NULL;
static DRAM_ATTR uint16_t *g_gamma_table = NULL;
static DRAM_ATTR bool g_hw_timer_started = false;


/**
 * @brief i2c master initialization
 */
static esp_err_t i2c_master_init(gpio_num_t sda_io_num, gpio_num_t scl_io_num, uint32_t clk_freq_hz)
{
    i2c_config_t conf = {0};
    conf.mode = I2C_MODE_MASTER;
    conf.sda_io_num = sda_io_num;
    conf.sda_pullup_en = GPIO_PULLUP_ENABLE;
    conf.scl_io_num = scl_io_num;
    conf.scl_pullup_en = GPIO_PULLUP_ENABLE;
    conf.master.clk_speed = clk_freq_hz;
    ESP_LOGI(TAG, "I2C_SDA=GPIO%d, I2C_SCL=GPIO%d", conf.sda_io_num, conf.scl_io_num);
    i2c_param_config(g_i2c_master_num, &conf);
    return i2c_driver_install(g_i2c_master_num, conf.mode, I2C_MASTER_RX_BUF_DISABLE, I2C_MASTER_TX_BUF_DISABLE, 0);
}

/**
 * @brief The chip SM2335EH doesn't support a real I2C interface, it's similar
 *
 */
static esp_err_t sm2335_reg_write(uint8_t out_addr, uint8_t *data_wr, size_t size)
{
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    uint8_t reg_config = 0xc0;
    reg_config |= g_is_stdby ? (0 << 3) : (3 << 3);
    reg_config |= out_addr & 0x07;
    i2c_master_write_byte(cmd, reg_config, ACK_CHECK_DIS);
    i2c_master_write_byte(cmd, g_max_current, ACK_CHECK_DIS);
    if (data_wr) {
        i2c_master_write(cmd, data_wr, size, ACK_CHECK_DIS);
    }
    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(g_i2c_master_num, cmd, 1000 / portTICK_RATE_MS);
    i2c_cmd_link_delete(cmd);
    return ret;
}

static esp_err_t sm2335_set_standby(bool isenable)
{
    g_is_stdby = isenable;
    return sm2335_reg_write(REG_CONFIG_OUT1, NULL, 0); /** ignore REG_CONFIG_xx when set max current */
}
/*static */esp_err_t sm2335_init(i2c_port_t i2c_master_port, gpio_num_t sda_io_num, gpio_num_t scl_io_num, uint32_t clk_freq_hz)
{
    esp_err_t ret = ESP_OK;
    g_i2c_master_num = i2c_master_port;
    ret = i2c_master_init(sda_io_num, scl_io_num, clk_freq_hz);
    SM2335_CHECK(ESP_OK == ret, "i2c init failed", ESP_FAIL);
    ret = sm2335_set_standby(0);
    sm2335_set_channel(0, 0);
    sm2335_set_channel(1, 0);
    sm2335_set_channel(2, 0);
    sm2335_set_channel(3, 0);
    sm2335_set_channel(4, 0);
    SM2335_CHECK(ESP_OK == ret, "sm2135 set standby failed", ESP_FAIL);
    return ESP_OK;
}

static esp_err_t sm2335_deinit(void)
{
    sm2335_set_standby(1);
    i2c_driver_delete(g_i2c_master_num);
    return ESP_OK;
}

/*static */esp_err_t sm2335_set_max_current(rgb_curr_t rgb_curr, wy_curr_t wy_curr)
{
    SM2335_CHECK(rgb_curr < RGB_CURR_MAX, "paramter invalid", ESP_ERR_INVALID_ARG);
    SM2335_CHECK(wy_curr < WY_CURR_MAX, "paramter invalid", ESP_ERR_INVALID_ARG);
    g_max_current = (rgb_curr << 4) | (wy_curr);
    return sm2335_reg_write(REG_CONFIG_OUT1, NULL, 0); /** ignore REG_CONFIG_xx when set max current */
}

/*static */esp_err_t sm2335_set_channel(uint8_t channel, uint16_t value)
{
    if (_CHANNEL_INVALID == channel) {
        return ESP_OK;
    }

    SM2335_CHECK(channel < _CHANNEL_MAX, "paramter invalid", ESP_ERR_INVALID_ARG);
    uint8_t reg_temp[2];
    reg_temp[0] = value>>8;
    reg_temp[1] = value&0xff;
    ESP_LOGI(TAG, "[%d, %d]", reg_temp[0], reg_temp[1]);
    return sm2335_reg_write(REG_CONFIG_OUT1 + channel, reg_temp, 2);
}

// /**
//  * @brief 
//  * 
//  * @param gamma_table 
//  * @param correction 
//  */
// static void gamma_table_create(uint16_t *gamma_table, float correction)
// {
//     float value_tmp = 0;

//     /**
//      * @brief gamma curve formula: y=a*x^(1/gm)
//      * x ∈ (0,(GAMMA_TABLE_SIZE-1)/GAMMA_TABLE_SIZE)
//      * a = GAMMA_TABLE_SIZE
//      */
//     for (int i = 0; i < GAMMA_TABLE_SIZE; i++) {
//         value_tmp = (float)(i) / GAMMA_TABLE_SIZE;
//         value_tmp = powf(value_tmp, 1.0f / correction);
//         gamma_table[i] = (uint16_t)FLOATINT_2_FIXED((value_tmp * GAMMA_TABLE_SIZE), LEDC_FIXED_Q);
//     }
// }

// static IRAM_ATTR uint32_t gamma_value_to_duty(int value)
// {
//     uint32_t tmp_q = GET_FIXED_INTEGER_PART(value, LEDC_FIXED_Q);
//     uint32_t tmp_r = GET_FIXED_DECIMAL_PART(value, LEDC_FIXED_Q);

//     uint16_t cur = LEDC_VALUE_TO_DUTY(g_gamma_table[tmp_q]);
//     uint16_t next = LEDC_VALUE_TO_DUTY(g_gamma_table[tmp_q + 1]);
//     return (cur + (next - cur) * tmp_r / (0x1U << LEDC_FIXED_Q));
// }

/*
// #define iot_timer_start(a) { \
//         xTaskCreate(_timer_task, "light_timer_task", 1024*4, (void*)(a), configMAX_PRIORITIES-1, NULL); \
//         g_hw_timer_started = true; \
//     }

// #define iot_timer_stop() { \
//         g_hw_timer_started = false; \
//         vTaskDelete(NULL); \
//     }
*/

// static IRAM_ATTR void fade_timercb(void *para)
// {
//     int idle_channel_num = 0;

//     for (int channel = 0; channel < _CHANNEL_MAX; channel++) {
//         ledc_fade_data_t *fade_data = g_light_config->fade_data + channel;

//         if (fade_data->num > 0) {
//             fade_data->num--;

//             if (fade_data->step) {
//                 fade_data->cur += fade_data->step;
//                 sm2335_set_channel(g_ch2gpio[channel], gamma_value_to_duty(fade_data->cur));
//             } else {
//                 sm2335_set_channel(g_ch2gpio[channel], gamma_value_to_duty(fade_data->cur));
//             }
//         } else if (fade_data->cycle) {
//             fade_data->num = fade_data->cycle - 1;

//             if (fade_data->step) {
//                 fade_data->step *= -1;
//                 fade_data->cur  += fade_data->step;
//             } else {
//                 fade_data->cur = (fade_data->cur == fade_data->final) ? 0 : fade_data->final;
//             }

//             sm2335_set_channel(g_ch2gpio[channel], gamma_value_to_duty(fade_data->cur));
//         } else {
//             idle_channel_num++;
//         }
//     }

//     if (idle_channel_num >= _CHANNEL_MAX) {
//         iot_timer_stop();
//     }
// }

// static void _timer_task(void *args)
// {
//     while (1) {
//         fade_timercb(NULL);
//         vTaskDelay(DUTY_SET_CYCLE / portTICK_PERIOD_MS);
//     }
// }

// esp_err_t iot_led_init(ledc_timer_t timer_num, ledc_mode_t speed_mode, uint32_t freq_hz)
// {
//     esp_err_t ret = MDF_OK;
//     ret = sm2335_init(I2C_NUM_1, I2C_SDA_GPIO, I2C_SCL_GPIO, I2C_FREQ_HZ);
//     MDF_ERROR_CHECK(ret != MDF_OK, ret, "LEDC timer configuration");

//     if (g_gamma_table == NULL) {
//         g_gamma_table = MDF_CALLOC(GAMMA_TABLE_SIZE, sizeof(uint16_t));
//         gamma_table_create(g_gamma_table, GAMMA_CORRECTION);
//     } else {
//         MDF_LOGE("gamma_table has been initialized");
//     }

//     if (g_light_config == NULL) {
//         g_light_config = MDF_CALLOC(1, sizeof(iot_light_t));

//     } else {
//         MDF_LOGE("g_light_config has been initialized");
//     }

//     memset(g_ch2gpio, _CHANNEL_INVALID, sizeof(g_ch2gpio));

//     return ESP_OK;
// }

// esp_err_t iot_led_deinit()
// {
//     if (g_hw_timer_started == true) {
//         return MDF_ERR_INVALID_STATE;
//     }

//     if (g_gamma_table) {
//         MDF_FREE(g_gamma_table);
//     }

//     if (g_light_config) {
//         MDF_FREE(g_light_config);
//     }

//     memset(g_ch2gpio, _CHANNEL_INVALID, sizeof(g_ch2gpio));
//     sm2135_deinit();
//     return ESP_OK;
// }

// esp_err_t iot_led_regist_channel(ledc_channel_t channel, gpio_num_t out_num)
// {
//     esp_err_t ret = MDF_OK;
//     MDF_ERROR_CHECK(g_light_config == NULL, MDF_ERR_NOT_INIT, "iot_led_init() must be called first");
// #ifdef CONFIG_SPIRAM_SUPPORT
//     MDF_ERROR_CHECK(gpio_num != GPIO_NUM_16 || gpio_num != GPIO_NUM_17, MDF_ERR_INVALID_ARG,
//                     "gpio_num must not conflict to PSRAM(IO16 && IO17)");
// #endif
//     MDF_ERROR_CHECK(channel >= _CHANNEL_MAX, MDF_ERR_INVALID_ARG, "channel must be in (0~%d)", _CHANNEL_MAX - 1);
//     MDF_ERROR_CHECK(out_num >= _CHANNEL_MAX, MDF_ERR_INVALID_ARG, "out_num must be in (0~%d)", _CHANNEL_MAX - 1);
//     g_ch2gpio[channel] = out_num;
//     return ret;
// }

// esp_err_t iot_led_get_channel(ledc_channel_t channel, uint8_t *dst)
// {
//     MDF_ERROR_CHECK(g_light_config == NULL, MDF_ERR_NOT_INIT, "iot_led_init() must be called first");
//     MDF_ERROR_CHECK(dst == NULL, MDF_ERR_INVALID_ARG, "dst should not be NULL");
//     int cur = g_light_config->fade_data[channel].cur;
//     *dst = FIXED_2_FLOATING(cur, LEDC_FIXED_Q);
//     return MDF_OK;
// }

// esp_err_t iot_led_set_channel(ledc_channel_t channel, uint8_t value, uint32_t fade_ms)
// {
//     MDF_ERROR_CHECK(g_light_config == NULL, MDF_ERR_NOT_INIT, "iot_led_init() must be called first");
//     ledc_fade_data_t *fade_data = g_light_config->fade_data + channel;

//     fade_data->final = FLOATINT_2_FIXED(value, LEDC_FIXED_Q);

//     if (fade_ms < DUTY_SET_CYCLE) {
//         fade_data->num = 1;
//     } else {
//         fade_data->num   = fade_ms / DUTY_SET_CYCLE;
//     }

//     fade_data->step  = abs(fade_data->cur - fade_data->final) / fade_data->num;

//     if (fade_data->cur > fade_data->final) {
//         fade_data->step *= -1;
//     }

//     if (fade_data->cycle != 0) {
//         fade_data->cycle = 0;
//     }

//     if (g_hw_timer_started != true) {

//         iot_timer_start(NULL);
//     }

//     return MDF_OK;
// }

// esp_err_t iot_led_start_blink(ledc_channel_t channel, uint8_t value, uint32_t period_ms, bool fade_flag)
// {
//     MDF_ERROR_CHECK(g_light_config == NULL, MDF_ERR_NOT_INIT, "iot_led_init() must be called first");
//     ledc_fade_data_t *fade_data = g_light_config->fade_data + channel;

//     fade_data->final = fade_data->cur = FLOATINT_2_FIXED(value, LEDC_FIXED_Q);
//     fade_data->cycle = period_ms / 2 / DUTY_SET_CYCLE;
//     fade_data->num = (fade_flag) ? period_ms / 2 / DUTY_SET_CYCLE : 0;
//     fade_data->step  = (fade_flag) ? fade_data->cur / fade_data->num * -1 : 0;

//     if (g_hw_timer_started != true) {
//         iot_timer_start(NULL);
//     }

//     return MDF_OK;
// }

// esp_err_t iot_led_stop_blink(ledc_channel_t channel)
// {
//     MDF_ERROR_CHECK(g_light_config == NULL, MDF_ERR_NOT_INIT, "iot_led_init() must be called first");
//     ledc_fade_data_t *fade_data = g_light_config->fade_data + channel;
//     fade_data->cycle = fade_data->num = 0;

//     return MDF_OK;
// }

// esp_err_t iot_led_set_gamma_table(const uint16_t gamma_table[GAMMA_TABLE_SIZE])
// {
//     MDF_ERROR_CHECK(g_gamma_table == NULL, MDF_ERR_NOT_INIT, "iot_led_init() must be called first");
//     memcpy(g_gamma_table, gamma_table, GAMMA_TABLE_SIZE * sizeof(uint16_t));
//     return MDF_OK;
// }
