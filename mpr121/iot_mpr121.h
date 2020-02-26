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
#ifndef __IOT_MPR121_H__
#define __IOT_MPR121_H__

#include "driver/i2c.h"
#include "iot_i2c_bus.h"

#define MPR121_ADDR_CON_GND    (0x5A)
#define MPR121_ADDR_CON_VCC    (0x5B)
#define MPR121_ADDR_CON_SDA    (0x5C)
#define MPR121_ADDR_CON_SCL    (0x5D)

#define MPR121_TS1    0x00
#define MPR121_TS2    0x01
#define MPR121_OORS1  0x02
#define MPR121_OORS2  0x03

// filtered data
#define MPR121_E0FDL  0x04
#define MPR121_E0FDH  0x05 //b it0 ~ bit4 | bit7
#define MPR121_E1FDL  0x06
#define MPR121_E1FDH  0x07 // bit0 ~ bit4 | bit6 ~ bit7
#define MPR121_E2FDL  0x08
#define MPR121_E2FDH  0x09 // bit0 ~ bit1
#define MPR121_E3FDL  0x0A
#define MPR121_E3FDH  0x0B // bit0 ~ bit1
#define MPR121_E4FDL  0x0C
#define MPR121_E4FDH  0x0D // bit0 ~ bit1
#define MPR121_E5FDL  0x0E
#define MPR121_E5FDH  0x0F // bit0 ~ bit1
#define MPR121_E6FDL  0x10
#define MPR121_E6FDH  0x11 // bit0 ~ bit1
#define MPR121_E7FDL  0x12
#define MPR121_E7FDH  0x13 // bit0 ~ bit1
#define MPR121_E8FDL  0x14
#define MPR121_E8FDH  0x15 // bit0 ~ bit1
#define MPR121_E9FDL  0x16
#define MPR121_E9FDH  0x17 // bit0 ~ bit1
#define MPR121_E10FDL  0x18
#define MPR121_E10FDH  0x19 // bit0 ~ bit1
#define MPR121_E11FDL  0x1A
#define MPR121_E11FDH  0x1B // bit0 ~ bit1
#define MPR121_PROXFDL  0x1C
#define MPR121_PROXFDH  0x1D // bit0 ~ bit1

// baseline values
#define MPR121_E0BV  0x1E
#define MPR121_E1BV  0x1F
#define MPR121_E2BV  0x20
#define MPR121_E3BV  0x21
#define MPR121_E4BV  0x22
#define MPR121_E5BV  0x23
#define MPR121_E6BV  0x24
#define MPR121_E7BV  0x25
#define MPR121_E8BV  0x26
#define MPR121_E9BV  0x27
#define MPR121_E10BV  0x28
#define MPR121_E11BV  0x29
#define MPR121_PROXBV  0x2A

// general electrode touch sense baseline filters
// rising filter
#define MPR121_MHDR  0x2B // bit0 ~ bit5
#define MPR121_NHDR  0x2C // bit0 ~ bit5
#define MPR121_NCLR  0x2D
#define MPR121_FDLR  0x2E

// falling filter
#define MPR121_MHDF  0x2F // bit0 ~ bit5
#define MPR121_NHDF  0x30 // bit0 ~ bit5
#define MPR121_NCLF  0x31
#define MPR121_FDLF  0x32

// touched filter
#define MPR121_NHDT  0x33 // bit0 ~ bit5
#define MPR121_NCLT  0x34
#define MPR121_FDLT  0x35

// proximity electrode touch sense baseline filters
// rising filter
#define MPR121_MHDPROXR  0x36 // bit0 ~ bit5
#define MPR121_NHDPROXR  0x37 // bit0 ~ bit5
#define MPR121_NCLPROXR  0x38
#define MPR121_FDLPROXR  0x39

// falling filter
#define MPR121_MHDPROXF  0x3A // bit0 ~ bit5
#define MPR121_NHDPROXF  0x3B // bit0 ~ bit5
#define MPR121_NCLPROXF  0x3C
#define MPR121_FDLPROXF  0x3D

// touched filter
#define MPR121_NHDPROXT  0x3E // bit0 ~ bit5
#define MPR121_NCLPROXT  0x3F
#define MPR121_FDLPROXT  0x40

//touch / release threshold
#define MPR121_E0TTH  0x41
#define MPR121_E0RTH  0x42
#define MPR121_E1TTH  0x43
#define MPR121_E1RTH  0x44
#define MPR121_E2TTH  0x45
#define MPR121_E2RTH  0x46
#define MPR121_E3TTH  0x47
#define MPR121_E3RTH  0x48
#define MPR121_E4TTH  0x49
#define MPR121_E4RTH  0x4A
#define MPR121_E5TTH  0x4B
#define MPR121_E5RTH  0x4C
#define MPR121_E6TTH  0x4D
#define MPR121_E6RTH  0x4E
#define MPR121_E7TTH  0x4F
#define MPR121_E7RTH  0x50
#define MPR121_E8TTH  0x51
#define MPR121_E8RTH  0x52
#define MPR121_E9TTH  0x53
#define MPR121_E9RTH  0x54
#define MPR121_E10TTH  0x55
#define MPR121_E10RTH  0x56
#define MPR121_E11TTH  0x57
#define MPR121_E11RTH  0x58
#define MPR121_PROXTTH  0x59
#define MPR121_PROXRTH  0x5A

// debounce settings
#define MPR121_DTR   0x5B  // bit0 ~ bit3 | bit5 ~ bit7

// configuration registers
#define MPR121_AFE1  0x5C
#define MPR121_AFE2  0x5D
#define MPR121_ECR   0x5E

// electrode currents
#define MPR121_CDC0  0x5F //bit0 ~ bit5
#define MPR121_CDC1  0x60 //bit0 ~ bit5
#define MPR121_CDC2  0x61 //bit0 ~ bit5
#define MPR121_CDC3  0x62 //bit0 ~ bit5
#define MPR121_CDC4  0x63 //bit0 ~ bit5
#define MPR121_CDC5  0x64 //bit0 ~ bit5
#define MPR121_CDC6  0x65 //bit0 ~ bit5
#define MPR121_CDC7  0x66 //bit0 ~ bit5
#define MPR121_CDC8  0x67 //bit0 ~ bit5
#define MPR121_CDC9  0x68 //bit0 ~ bit5
#define MPR121_CDC10  0x69 //bit0 ~ bit5
#define MPR121_CDC11  0x6A //bit0 ~ bit5
#define MPR121_CDCPROX  0x6B //bit0 ~ bit5

// electrode charge times
#define MPR121_CDT01  0x6C //bit0 ~ bit2 | bit4 ~ bit6
#define MPR121_CDT23  0x6D //bit0 ~ bit2 | bit4 ~ bit6
#define MPR121_CDT45  0x6E //bit0 ~ bit2 | bit4 ~ bit6
#define MPR121_CDT67  0x6F //bit0 ~ bit2 | bit4 ~ bit6
#define MPR121_CDT89  0x70 //bit0 ~ bit2 | bit4 ~ bit6
#define MPR121_CDT1011  0x71 //bit0 ~ bit2 | bit4 ~ bit6
#define MPR121_CDTPROX  0x72 //bit0 ~ bit2

// GPIO
#define MPR121_CTL0  0x73
#define MPR121_CTL1  0x74
#define MPR121_DAT  0x75
#define MPR121_DIR  0x76
#define MPR121_EN   0x77
#define MPR121_SET  0x78
#define MPR121_CLR  0x79
#define MPR121_TOG  0x7A

// auto-config
#define MPR121_ACCR0  0x7B
#define MPR121_ACCR1  0x7C //bit0 ~ bit2 | bit7
#define MPR121_USL    0x7D
#define MPR121_LSL    0x7E
#define MPR121_TL     0x7F

// soft reset
#define MPR121_SRST  0x80

// PWM
#define MPR121_PWM0  0x81
#define MPR121_PWM1  0x82
#define MPR121_PWM2  0x83
#define MPR121_PWM3  0x84

// Debounce number, The value range is 0~7.
//All 13 channels use the same set of touch and release debounce numbers. The status bits in Status Register 0x00 and 0x01 will
//only take place after the number of consecutive touch or release detection meets the debounce number setting. The debounce
//setting can be very useful in avoiding possible noise glitches. Using the debounce setting, the status bit change will have a delay
//of {ESI x SFI x DR (or DT)}.
typedef enum {
  MPR121_DEBOUNCE_DIS,
  MPR121_DEBOUNCE_NUM_1,
  MPR121_DEBOUNCE_NUM_2,
  MPR121_DEBOUNCE_NUM_3,
  MPR121_DEBOUNCE_NUM_4,
  MPR121_DEBOUNCE_NUM_5,
  MPR121_DEBOUNCE_NUM_6,
  MPR121_DEBOUNCE_NUM_7,
}mpr121_debounce_t;

//First Filter Iterations - The first filter iterations field selects the number of samples taken as
//input to the first level of filtering.
typedef enum {
  MPR121_FFI_SET_6, //(Default)
  MPR121_FFI_SET_10,
  MPR121_FFI_SET_18,
  MPR121_FFI_SET_34,
}mpr121_ffi_t;

//Charge Discharge Current - Selects the global value of charge discharge current applied to
//electrode. The maximum is 63 μA, 1 μA step.
typedef enum {
  MPR121_CDC_DIS = 0,
  MPR121_CDC_1uA = 1,
  //~
  MPR121_CDC_16uA = 16, //(Default)
  //~
  MPR121_CDC_63uA = 63,
}mpr121_cdc_t;

//Charge Discharge Time - Selects the global value of charge time applied to electrode.
//The maximum is 32 μs, programmable as 2 ^(n-2) μs.
typedef enum {
  MPR121_CDT_DIS,
  MPR121_CDT_0_5us, //(Default)
  MPR121_CDT_1us,
  //~
  MPR121_CDT_32us = 7,
}mpr121_cdt_t;

//Second Filter Iterations - Selects the number of samples taken for the second level filter
typedef enum {
  MPR121_SFI_SET_4, //(Default)
  MPR121_SFI_SET_6,
  MPR121_SFI_SET_10,
  MPR121_SFI_SET_18,
}mpr121_sfi_t;

//Electrode Sample Interval - Selects the period between samples used for the second level
//of filtering. The maximum is 128ms, Programmable to 2^n ms
typedef enum
{
  MPR121_ESI_1MS,
  MPR121_ESI_2MS,
  MPR121_ESI_4MS,
  MPR121_ESI_8MS,
  MPR121_ESI_16MS, //(Default)
  MPR121_ESI_32MS,
  MPR121_ESI_64MS,
  MPR121_ESI_128MS,
}mpr121_esi_t;

//Calibration Lock - Controls the baseline tracking and how the baseline initial value is loaded
typedef enum {
  MPR121_CL_EN,          // Baseline tracking enabled, initial baseline value is current value in baseline value register(Default)
  MPR121_CL_DIS,         // baseline tracking disabled
  MPR121_CL_5_BIT_COPY,  // Baseline tracking enabled, initial baseline value is loaded with the 5 high bits of the first 10-bit electrode data value
  MPR121_CL_10_BIT_COPY  // Baseline tracking enabled, initial baseline value is loaded with all 10 bits of the first electrode data value
}mpr121_cl_t;

//Proximity Enable - Controls the operation of 13th Proximity Detection
typedef enum {
  MPR121_ELEPROX_EN_DIS,  // proximity mode disabled
  MPR121_ELEPROX_0_1,     // proximity mode for ELE0..ELE1
  MPR121_ELEPROX_0_3,     // proximity mode for ELE0..ELE3
  MPR121_ELEPROX_0_11     // proximity mode for ELE0..ELE11
}mpr121_eleporx_en_t;

//Electrode Enable - Controls the operation of 12 electrodes detection
typedef enum
{
  MPR121_ELE_DIS,         // electrode detection enabled for disabled
  MPR121_ELE_EN_0,        // electrode detection enabled for ELE0
  MPR121_ELE_EN_0_1,      // electrode detection enabled for ELE0..ELE1
  MPR121_ELE_EN_0_2,      // electrode detection enabled for ELE0..ELE2
  MPR121_ELE_EN_0_3,      // electrode detection enabled for ELE0..ELE3
  MPR121_ELE_EN_0_4,      // electrode detection enabled for ELE0..ELE4
  MPR121_ELE_EN_0_5,      // electrode detection enabled for ELE0..ELE5
  MPR121_ELE_EN_0_6,      // electrode detection enabled for ELE0..ELE6
  MPR121_ELE_EN_0_7,      // electrode detection enabled for ELE0..ELE7
  MPR121_ELE_EN_0_8,      // electrode detection enabled for ELE0..ELE8
  MPR121_ELE_EN_0_9,      // electrode detection enabled for ELE0..ELE9
  MPR121_ELE_EN_0_10,     // electrode detection enabled for ELE0..ELE10
  MPR121_ELE_EN_0_11,     // electrode detection enabled for ELE0..ELE11
}mpr121_ele_en_t;

//Specifies the number of retries for autoconfiguration and autoreconfiguration if the configuration fails before setting OOR.
typedef enum {
  MPR121_RETRY_DIS,   // No retry
  MPR121_RETRY_2,     // retry 2 times
  MPR121_RETRY_4,     // retry 4 times
  MPR121_RETRY_8      // retry 8 times
}mpr121_retry_t;

//When Enabled, the autoconfiguration will operate once at the beginning
//of the transition from Stop Mode to Run Mode. This includes search and update of the CDCx and CDTx for each enabled channel
//(if SCTS = 0).
typedef enum {
  MPR121_ACE_DIS,   // Auto-Configuration Disable.
  MPR121_ACE_EN,     // Auto-Configuration Enable
}mpr121_ace_t;

//When enabled, if the OOR is set for a channel after autoconfiguration,
//autoreconfiguration will operate on that channel on each sampling interval until the OOR is cleared.
typedef enum {
  MPR121_ARE_DIS,   // Auto-Reconfiguration Disable.
  MPR121_ARE_EN,     // Auto-Reconfiguration Enable
}mpr121_are_t;

//1: Skip CDTx search and update when autoconfiguration or autoreconfiguration, and current global CDT or CDTx are used
//for respective channels. CDT or CDTx needs to be specified by the designer manually before operation. Setting the SCTS
//to “1” results in a shorter time to complete autoconfiguration. This is useful for when the designer has obtained the correct
//CDTx / CDT, and is confident that the current CDT and CDTx settings work in all conditions.
//0: Both CDTx and CDCx will be searched and set by autoconfiguration and/or autoreconfiguration.
typedef enum {
  MPR121_SCTS_DIS,    // Skip Charge Time Search Disable
  MPR121_SCTS_EN,     // Skip Charge Time Search Enable
}mpr121_scts_t;

typedef enum {
  MPR121_OORIE_DIS,    // Out-of-range interrupt Disable
  MPR121_OORIE_EN,     // Out-of-range interrupt enable
}mpr121_oorie_t;

typedef enum {
  MPR121_ARFIE_DIS,    // Auto-reconfiguration fail interrupt Disable
  MPR121_ARFIE_EN,     // Auto-reconfiguration fail interrupt enable
}mpr121_arfie_t;

typedef enum {
  MPR121_ACFIE_DIS,    // Auto-configuration fail interrupt Disable
  MPR121_ACFIE_EN,     // Auto-configuration fail interrupt enable
}mpr121_acfie_t;

// GPIO pin function constants
typedef enum {
  MPR121_INPUT,            // digital input
  MPR121_INPUT_PULLUP,     // digital input with pullup
  MPR121_INPUT_PULLDOWN,   // digital input with pulldow
  MPR121_OUTPUT,           // digital output (push-pull)
  MPR121_OUTPUT_HIGHSIDE,  // digital output, open collector (high side)
  MPR121_OUTPUT_LOWSIDE    // digital output, open collector (low side)
}mpr121_pin_mode_t;

// sample intervals
typedef enum
{
  MPR121_ELE_0,
  MPR121_ELE_1,
  MPR121_ELE_2,
  MPR121_ELE_3,
  MPR121_ELE_4,
  MPR121_ELE_5,
  MPR121_ELE_6,
  MPR121_ELE_7,
  MPR121_ELE_8,
  MPR121_ELE_9,
  MPR121_ELE_10,
  MPR121_ELE_11,
  MPR121_ELEPROX,
  MPR121_ELE_MAX,
}mpr121_electrode_t;

typedef struct mpr121_settings {

  // touch and release thresholds
  uint8_t tthresh;
  uint8_t rthresh;

  // general electrode touch sense baseline filters
  // rising filter
  uint8_t mhdr;
  uint8_t nhdr;
  uint8_t nclr;
  uint8_t fdlr;

  // falling filter
  uint8_t mhdf;
  uint8_t nhdf;
  uint8_t nclf;
  uint8_t fdlf;

  // touched filter
  uint8_t nhdt;
  uint8_t nclt;
  uint8_t fdlt;

  // proximity electrode touch sense baseline filters
  // rising filter
  uint8_t mhdproxr;
  uint8_t nhdproxr;
  uint8_t nclproxr;
  uint8_t fdlproxr;

  // falling filter
  uint8_t mhdproxf;
  uint8_t nhdproxf;
  uint8_t nclproxf;
  uint8_t fdlproxf;

  // touched filter
  uint8_t nhdproxt;
  uint8_t nclproxt;
  uint8_t fdlproxt;

  // debounce settings
  mpr121_debounce_t dr;
  mpr121_debounce_t dt;

  // configuration registers
  mpr121_ffi_t ffi;
  mpr121_cdc_t cdc;
  mpr121_cdt_t cdt;

  mpr121_sfi_t sfi;
  mpr121_esi_t esi;

  mpr121_cl_t cl;
  mpr121_eleporx_en_t eleporx_en;
  mpr121_ele_en_t  ele_en;

  // auto-configuration registers
  mpr121_retry_t retry;
  mpr121_are_t are;
  mpr121_ace_t ace;

  mpr121_scts_t scts;
  mpr121_oorie_t oorie;
  mpr121_arfie_t arfie;
  mpr121_acfie_t acfie;

  uint8_t usl;
  uint8_t lsl;
  uint8_t tl;
} mpr121_settings_t;


#define DEFAULT_TOUCH_THRESHOLD   0x0A
#define DEFAULT_RELEASE_THRESHOLD 0x06

#define DEFAULT_MHDR 0x01
#define DEFAULT_NHDR 0x01
#define DEFAULT_NCLR 0x10
#define DEFAULT_FDLR 0x20

#define DEFAULT_MHDF 0x01
#define DEFAULT_NHDF 0x01
#define DEFAULT_NCLF 0x10
#define DEFAULT_FDLF 0x20

#define DEFAULT_NHDT 0x01
#define DEFAULT_NCLT 0x10
#define DEFAULT_FDLT 0xff

#define DEFAULT_PROX_MHDR 0x0f
#define DEFAULT_PROX_NHDR 0x0f
#define DEFAULT_PROX_NCLR 0x00
#define DEFAULT_PROX_FDLR 0x00

#define DEFAULT_PROX_MHDF 0x01
#define DEFAULT_PROX_NHDF 0x01
#define DEFAULT_PROX_NCLF 0xFF
#define DEFAULT_PROX_FDLF 0xFF

#define DEFAULT_PROX_NHDT 0x00
#define DEFAULT_PROX_NCLT 0x00
#define DEFAULT_PROX_FDLT 0x00

#define DEFAULT_USL 0xC8  //(VDD - 0.7)/VDD x 256
#define DEFAULT_TL  0xB4  //(VDD - 0.7)/VDD x 256 x 0.9
#define DEFAULT_LSL 0x82  //(VDD - 0.7)/VDD x 256 x 0.65

#define DEFSULT_CONFIG {DEFAULT_TOUCH_THRESHOLD, DEFAULT_RELEASE_THRESHOLD,\
                        DEFAULT_MHDR, DEFAULT_NHDR, DEFAULT_NCLR, DEFAULT_FDLR,\
                        DEFAULT_MHDF, DEFAULT_NHDF, DEFAULT_NCLF, DEFAULT_FDLF,\
                        DEFAULT_NHDT, DEFAULT_NCLT, DEFAULT_FDLT,\
                        DEFAULT_PROX_MHDR, DEFAULT_PROX_NHDR, DEFAULT_PROX_NCLR, DEFAULT_PROX_FDLR,\
                        DEFAULT_PROX_MHDF, DEFAULT_PROX_NHDF, DEFAULT_PROX_NCLF, DEFAULT_PROX_FDLF,\
                        DEFAULT_PROX_NHDT, DEFAULT_PROX_NCLT, DEFAULT_PROX_FDLT,\
                        MPR121_DEBOUNCE_NUM_1, MPR121_DEBOUNCE_NUM_1,\
                        MPR121_FFI_SET_6, MPR121_CDC_16uA,\
                        MPR121_CDT_0_5us, MPR121_SFI_SET_4, MPR121_ESI_16MS,\
                        MPR121_CL_10_BIT_COPY, MPR121_ELEPROX_EN_DIS, MPR121_ELE_EN_0_11,\
                        MPR121_RETRY_2, MPR121_ARE_DIS, MPR121_ACE_EN,\
                        MPR121_SCTS_DIS, MPR121_OORIE_DIS, MPR121_ARFIE_DIS, MPR121_ACFIE_DIS,\
                        DEFAULT_USL, DEFAULT_LSL, DEFAULT_TL }

typedef void* mpr121_handle_t;

#ifdef __cplusplus
extern "C"
{
#endif

/**
 * @brief   Create and initialization device object and return a device handle
 *
 * @param   bus I2C bus object handle
 * @param   dev_addr I2C device address of device
 *
 * @return
 *     - device object handle of mpr121_handle_t
 */
mpr121_handle_t iot_mpr121_create(i2c_bus_handle_t bus, uint8_t dev_addr);

/**
 * @brief   Delete and release a device object
 *
 * @param   dev object handle of mpr121_handle_t
 * @param   del_bus Whether to delete the I2C bus
 *
 * @return
 *     - ESP_OK Success
 *     - ESP_FAIL Fail
 */
esp_err_t iot_mpr121_delete(mpr121_handle_t dev, bool del_bus);

/**
 * @brief   Write config or data to mpr121_handle_t
 *
 * @param   dev object handle of mpr121_handle_t
 * @param   config  will write config
 *
 * @return
 *     - ESP_OK Success
 *     - ESP_FAIL Fail
 */
esp_err_t iot_mpr121_config(mpr121_handle_t dev, mpr121_settings_t *settings);

/**
 * @brief   Write RAM or byte data to mpr121_handle_t
 *
 * @param   dev object handle of mpr121_handle_t
 * @param   address RAM address
 * @param   buf data value
 *
 * @return
 *     - ESP_OK Success
 *     - ESP_FAIL Fail
 */
esp_err_t iot_mpr121_write_byte(mpr121_handle_t dev, uint8_t address, uint8_t data);

/**
 * @brief   Read RAM or byte data to mpr121_handle_t
 *
 * @param   dev object handle of mpr121_handle_t
 * @param   address RAM address
 * @param   buf data value
 *
 * @return
 *     - ESP_OK Success
 *     - ESP_FAIL Fail
 */
esp_err_t iot_mpr121_read_byte(mpr121_handle_t dev, uint8_t address, uint8_t *data);

/**
 * @brief   Read RAM or data to mpr121_handle_t
 *
 * @param   dev object handle of mpr121_handle_t
 * @param   address RAM address
 * @param   *buf data value pointer
 * @param   *buf data value len
 *
 * @return
 *     - ESP_OK Success
 *     - ESP_FAIL Fail
 */
esp_err_t iot_mpr121_read(mpr121_handle_t dev, uint8_t address, uint8_t *buf, uint8_t len);

/**
 * @brief   Set up so channel all touch hreshold
 *
 * @param   dev object handle of mpr121_handle_t
 * @param   touch  touch hreshold
 *
 * @return
 *     - ESP_OK Success
 *     - ESP_FAIL Fail
 */
esp_err_t iot_mpr121_set_touch_hreshold(mpr121_handle_t dev, uint8_t touch);

/**
 * @brief   Set up so channel all release hreshold
 *
 * @param   dev object handle of mpr121_handle_t
 * @param   release  release hreshold
 *
 * @return
 *     - ESP_OK Success
 *     - ESP_FAIL Fail
 */
esp_err_t iot_mpr121_set_release_hreshold(mpr121_handle_t dev, uint8_t release);

/**
 * @brief
 *
 * @param   dev object handle of mpr121_handle_t
 * @param   baseline Touch baseline value
 *
 * @return
 *     - ESP_OK Success
 *     - ESP_FAIL Fail
 */
esp_err_t iot_mpr121_set_baseline_data(mpr121_handle_t dev, uint8_t baseline);

/**
 * @brief
 *
 * @param   dev object handle of mpr121_handle_t
 * @param   *data data value pointer
 * @param   *data_len data value len
 *
 * @return
 *     - ESP_OK Success
 *     - ESP_FAIL Fail
 */
esp_err_t iot_mpr121_get_baseline_data(mpr121_handle_t dev, int16_t *data, size_t data_len);

/**
 * @brief   The MPR121 provides filtered electrode output data for all 13 channels.
 *
 * @param   dev object handle of mpr121_handle_t
 * @param   *data data value pointer
 * @param   *data_len data value len
 *
 * @return
 *     - ESP_OK Success
 *     - ESP_FAIL Fail
 */
esp_err_t iot_mpr121_get_filtered_data(mpr121_handle_t dev, int16_t *data, size_t data_len);

/**
 * @brief   The MPR121 acquires the filtered electrode output data of the selected electrode's.
 *
 * @param   dev object handle of mpr121_handle_t
 * @param   electrode electrode ID
 * @param   *data data value pointer
 *
 * @return
 *     - ESP_OK Success
 *     - ESP_FAIL Fail
 */
esp_err_t iot_mpr121_get_electrode_filtered_data(mpr121_handle_t dev, mpr121_electrode_t electrode, uint16_t *data);

/**
 * @brief   Get the touch status of all channels
 *
 * @param   dev object handle of mpr121_handle_t
 * @param   *status  Touch status buf
 *
 * @return
 *     - ESP_OK Success
 *     - ESP_FAIL Fail
 */
esp_err_t iot_mpr121_get_touch_status(mpr121_handle_t dev, uint16_t *status);

/**
 * @brief   Set the mode of the selected electrode's GPIO
 *
 * @param   dev object handle of mpr121_handle_t
 * @param   electrode electrode ID
 * @param   mode GPIO mode
 *
 * @return
 *     - ESP_OK Success
 *     - ESP_FAIL Fail
 */
esp_err_t iot_mpr121_set_pin_mode(mpr121_handle_t dev, mpr121_electrode_t electrode, mpr121_pin_mode_t mode);

/**
 * @brief   mpr121 reset
 *
 * @param   dev object handle of mpr121_handle_t
 *
 * @return
 *     - ESP_OK Success
 *     - ESP_FAIL Fail
 */
esp_err_t iot_mpr121_set_reset(mpr121_handle_t dev);

/**
 * @brief   Run mpr121 data collection
 *
 * @param   dev object handle of mpr121_handle_t
 *
 * @return
 *     - ESP_OK Success
 *     - ESP_FAIL Fail
 */
esp_err_t iot_mpr121_set_run(mpr121_handle_t dev);

/**
 * @brief   Stop mpr121 data collection
 *
 * @param   dev object handle of mpr121_handle_t
 *
 * @return
 *     - ESP_OK Success
 *     - ESP_FAIL Fail
 */
esp_err_t iot_mpr121_set_stop(mpr121_handle_t dev);

/**
 * @brief   get out of range status
 *
 * @param   dev object handle of mpr121_handle_t
 * @param   *status  Error status buf
 *
 * @return
 *     - ESP_OK Success
 *     - ESP_FAIL Fail
 */
esp_err_t iot_mpr121_get_out_of_range(mpr121_handle_t dev,uint16_t *status);

#ifdef __cplusplus
}
#endif
#endif

