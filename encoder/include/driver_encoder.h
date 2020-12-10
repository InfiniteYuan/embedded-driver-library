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

#ifndef _DRIVER_ENCODER_H_
#define _DRIVER_ENCODER_H_

#ifdef __cplusplus
extern "C" {
#endif

#define ENCODER_VALUE_MAX   (20)
#define ENCODER_VALUE_MIN   (-20)
#define ENCODER_PIN_A   18
#define ENCODER_PIN_B   5
#define ENCODER_PIN_D   2 // button
#define ENCODER_PIN_PRESS 19

uint8_t switch_flag;
bool encoder_get_button_state();

int16_t encoder_get_new_moves();

void encoder_init();

#ifdef __cplusplus
}
#endif

#endif
