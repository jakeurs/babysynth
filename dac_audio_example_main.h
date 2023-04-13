/*
 * SPDX-FileCopyrightText: 2022 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: CC0-1.0
 */

#include <inttypes.h>
#include <string.h>
#include <math.h>
// #include <cstdint>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "driver/dac_continuous.h"
#include "esp_adc/adc_oneshot.h"
#include "esp_check.h"
#include "driver/i2c.h"
#include "driver/gpio.h"

#define MPU_ADDR 0x68
#define I2C_MASTER_NUM 0            /*!< I2C master i2c port number, the number of i2c peripheral interfaces available will depend on the chip */
#define I2C_MASTER_FREQ_HZ 400000   /*!< I2C master clock frequency */
#define I2C_MASTER_TX_BUF_DISABLE 0 /*!< I2C master doesn't need buffer */
#define I2C_MASTER_RX_BUF_DISABLE 0 /*!< I2C master doesn't need buffer */
#define I2C_MASTER_TIMEOUT_MS 1000
#define I2C_MASTER_SCL_IO 22           /*!< GPIO number used for I2C master clock 19 */
#define I2C_MASTER_SDA_IO 21           /*!< GPIO number used for I2C master data  18 */
#define MPU6050_WHO_AM_I_REG_ADDR 0x75 /*!< Register addresses of the "who am I" register */

#define MPU6050_PWR_MGMT_1_REG_ADDR 0x6B /*!< Register addresses of the power managment register */
#define MPU6050_RESET_BIT 7

#define MPU6050_REG_GYRO_X_H 0x43
#define MPU6050_REG_GYRO_X_L 0x44

static const char *TAG = "dac_audio";

// #define CONFIG_EXAMPLE_AUDIO_SAMPLE_RATE = 48000

#define SAMPLEBUFFER_SIZE 4096
#define SAMPLE_RATE 24000

const uint16_t SAMPLEBUFFER_LENGTH_MS = (SAMPLEBUFFER_SIZE * 1000) / SAMPLE_RATE;
const float SAMPLE_PERIOD = 1.0 / SAMPLE_RATE * 1000.0;
const uint16_t FREQUENCY_MIN = 30;
const uint16_t FREQUENCY_MAX = SAMPLE_RATE / 2u;
const uint16_t FREQUENCY_RANGE = FREQUENCY_MAX - FREQUENCY_MIN;