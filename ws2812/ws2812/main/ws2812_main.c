/* Hello World Example

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/
#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "esp_log.h"

#include "dev_ws2812.h"

void app_main()
{
    led_state_t led;

    dev_ws2812_init();

    while (1) {
        led.leds[0] = esp_random();
        dev_ws2812_write_leds(led);
        vTaskDelay(pdMS_TO_TICKS(500));
    }
}
