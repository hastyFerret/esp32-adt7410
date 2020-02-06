/*
 *  esp32-adt7410.h
 *
 *  LIMITATIONS:
 *  - Only supports ADT7410 continuous conversion mode
 */

#ifndef ESP32_ADT7410_H
#define ESP32_ADT7410_H

#include "esp_err.h"
#include "driver/i2c.h"

#define ADT7410_TEMP_UNIT_CELCIUS   0x00
#define ADT7410_TEMP_UNIT_FARENHEIT 0x01
#define ADT7410_TEMP_UNIT_KELVIN    0x02

#define ADT7410_CONFIG_RES_13BIT 0x00
#define ADT7410_CONFIG_RES_16BIT 0x80

#define ADT7410_CONFIG_MODE_CONT     0x00
#define ADT7410_CONFIG_MODE_ONESHOT  0b00100000
#define ADT7410_CONFIG_MODE_1SPS     0b01000000
#define ADT7410_CONFIG_MODE_SHUTDOWN 0b01100000

#define I2C_CMD_TIMEOUT (1000 / portTICK_RATE_MS)

typedef uint8_t adt7410_config_t;

typedef struct {
  bool init;
  i2c_port_t i2c_port;
  uint8_t adt7410_addr;
  uint8_t reg_config;
} adt7410_info_t;

esp_err_t adt7410_init(adt7410_info_t *adt7410_info, const i2c_port_t i2c_port, const uint8_t adt7410_addr, const adt7410_config_t adt7410_config);

esp_err_t adt7410_get_temperature(const adt7410_info_t *adt7410_info, uint8_t temp_units, float *temperature);

#endif
