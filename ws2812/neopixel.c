#include <stdio.h>
#include <time.h>
#include <stdlib.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "driver/i2c.h"
#include "driver/rmt.h"
#include "esp_log.h"

#define NEOPIXEL_IO                        25               /*!< gpio number for neopixel */
#define RMT_TX_CHANNEL                     RMT_CHANNEL_0
#define NEOPIXEL_CNT                       (3*1)

static void IRAM_ATTR u8_to_rmt(const void* src, rmt_item32_t* dest, size_t src_size, 
                         size_t wanted_num, size_t* translated_size, size_t* item_num)
{
    if(src == NULL || dest == NULL) {
        *translated_size = 0;
        *item_num = 0;
        return;
    }
    const rmt_item32_t bit0 = {{{ 8, 0, 17, 1 }}}; //Logical 0
    const rmt_item32_t bit1 = {{{ 16, 0, 9, 1 }}}; //Logical 1
    size_t size = 0;
    size_t num = 0;
    uint8_t *psrc = (uint8_t *)src;
    rmt_item32_t* pdest = dest;
    while (size < src_size && num < wanted_num) {
        for(int i = 0; i < 8; i++) {
            if(*psrc & (0x80 >> i)) {
                pdest->val =  bit1.val; 
            } else {
                pdest->val =  bit0.val;
            }
            num++;
            pdest++;
        }
        size++;
        psrc++;
    }
    *translated_size = size;
    *item_num = num;
}

static void neopixel_init(){
    rmt_config_t config;
    config.rmt_mode = RMT_MODE_TX;
    config.channel = RMT_TX_CHANNEL;
    config.gpio_num = NEOPIXEL_IO;
    config.mem_block_num = 1;
    config.tx_config.loop_en = 0;
    // set the maximum clock divider to be able to output
    config.clk_div = 4;

    ESP_ERROR_CHECK(rmt_config(&config));
    ESP_ERROR_CHECK(rmt_driver_install(config.channel, 0, 0));
    ESP_ERROR_CHECK(rmt_translator_init(config.channel, u8_to_rmt));
    ESP_ERROR_CHECK(rmt_set_idle_level(RMT_TX_CHANNEL,1,1));
}

void neopixel_show(){
    uint8_t neopixel_value[NEOPIXEL_CNT] = {};
    // ESP_ERROR_CHECK(rmt_set_idle_level(RMT_TX_CHANNEL,1,1));

    
    while(1){
        for(int i=0; i < NEOPIXEL_CNT; i++){
            neopixel_value[i] = rand() % 128;
        }

        ESP_ERROR_CHECK(rmt_write_sample(RMT_TX_CHANNEL, neopixel_value, NEOPIXEL_CNT, false)); 
        // vTaskDelay(1);

        printf("write rmt ok!\n");

        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
}


void app_main()
{
    neopixel_init();
    srand(time(NULL));   // set a seed for random, should only be called once

    xTaskCreate(neopixel_show, "neopixel_show", 2048*2, NULL, 3, NULL);
}
