
#include <math.h>
#include "esp_log.h"

#include "esp32-adt7410.h"

#define I2C_ACK_VAL   true
#define I2C_NACK_VAL  false

#define ADT7410_REG_TEMPVAL_MSB               0x00
#define ADT7410_REG_TEMPVAL_LSB               0x01
#define ADT7410_REG_STATUS                    0x02
#define ADT7410_REG_CONFIG                    0x03
#define ADT7410_REG_TEMP_HIGH_SETPOINT_MSB    0x04
#define ADT7410_REG_TEMP_HIGH_SETPOINT_LSB    0x05
#define ADT7410_REG_TEMP_LOW_SETPOINT_MSB     0x06
#define ADT7410_REG_TEMP_LOW_SETPOINT_LSB     0x07
#define ADT7410_REG_TEMP_CRIT_SETPOINT_MSB    0x08
#define ADT7410_REG_TEMP_CRIT_SETPOINT_LSB    0x09
#define ADT7410_REG_TEMP_HYST_SETPOINT        0x0a
#define ADT7410_REG_ID                        0x0b
#define ADT7410_REG_SOFT_RESET                0x2f

#define ADT7410_MANUFACTURER_ID               0b11001000

static const char * TAG = "esp32-adt7410";

static bool _is_init(const adt7410_info_t *adt7410_info)
{
  if(adt7410_info == NULL)
  {
    ESP_LOGE(TAG, "adt7410_info is NULL");
    return(false);
  }
  if(!adt7410_info->init)
  {
    ESP_LOGE(TAG, "adt7410_info is not initialized");
    return(false);
  }
  return(true);
}

esp_err_t adt7410_init(adt7410_info_t *adt7410_info,
                       const i2c_port_t i2c_port,
                       const uint8_t adt7410_addr,
                       const adt7410_config_t adt7410_config) {
  uint8_t id;
  esp_err_t ret;

  if(adt7410_info == NULL) {
    ESP_LOGE(TAG, "adt7410_info is NULL");
    return(ESP_FAIL);
  }

  /* Store settings in adt7410_info */
  adt7410_info->i2c_port = i2c_port;
  adt7410_info->adt7410_addr = adt7410_addr;
  adt7410_info->reg_config = adt7410_config;
  adt7410_info->init = true;

  i2c_cmd_handle_t cmd = i2c_cmd_link_create();
  /* Check ADT7410 manufacturer ID */
  i2c_master_start(cmd);
  i2c_master_write_byte(cmd, adt7410_info->adt7410_addr << 1 | I2C_MASTER_WRITE, I2C_ACK_VAL);
  i2c_master_write_byte(cmd, ADT7410_REG_ID, I2C_ACK_VAL);

  i2c_master_start(cmd);
  i2c_master_write_byte(cmd, adt7410_info->adt7410_addr << 1 | I2C_MASTER_READ, I2C_ACK_VAL);
  i2c_master_read_byte(cmd, &id, I2C_ACK_VAL);
  i2c_master_stop(cmd);

  ret = i2c_master_cmd_begin(adt7410_info->i2c_port, cmd, I2C_CMD_TIMEOUT);
  i2c_cmd_link_delete(cmd);

  if(ret != ESP_OK) {
    ESP_LOGE(TAG, "I2C Bus Error while reading Manufacturer ID");
    return(ret);
  }

  if((id & 0b11111000) != ADT7410_MANUFACTURER_ID) {
    ESP_LOGE(TAG, "ADT7410 not found at I2C address");
    return(ESP_FAIL);
  }

  vTaskDelay( 500 / portTICK_RATE_MS);
  /* Set sensor configuration */
  cmd = i2c_cmd_link_create();

  i2c_master_start(cmd);
  i2c_master_write_byte(cmd, adt7410_info->adt7410_addr << 1 | I2C_MASTER_WRITE, I2C_ACK_VAL);
  i2c_master_write_byte(cmd, ADT7410_REG_CONFIG, I2C_ACK_VAL);

  i2c_master_start(cmd);
  i2c_master_write_byte(cmd, adt7410_info->adt7410_addr << 1 | I2C_MASTER_WRITE, I2C_ACK_VAL);
  i2c_master_write_byte(cmd, adt7410_info->reg_config, I2C_ACK_VAL);
  i2c_master_stop(cmd);

  ret = i2c_master_cmd_begin(adt7410_info->i2c_port, cmd, I2C_CMD_TIMEOUT);
  if(ret != ESP_OK) {
    ESP_LOGE(TAG, "I2C Bus Error while writing sensor config");
    return(ret);
  }

  i2c_cmd_link_delete(cmd);

  vTaskDelay( 500 / portTICK_RATE_MS);

  return(ret);
}

esp_err_t adt7410_get_temperature(const adt7410_info_t *adt7410_info, const uint8_t temp_units, float *temperature) {
  //uint8_t temp_msb, temp_lsb;
  int8_t temp_neg;
  float temp;
  uint8_t temp_reg[2];
  uint16_t temp_adc;

  if(!_is_init(adt7410_info)) {
    return(ESP_FAIL);
  }

  i2c_cmd_handle_t cmd = i2c_cmd_link_create();

  i2c_master_start(cmd);
  i2c_master_write_byte(cmd, (adt7410_info->adt7410_addr << 1) | I2C_MASTER_WRITE, I2C_ACK_VAL);
  i2c_master_write_byte(cmd, ADT7410_REG_TEMPVAL_MSB, I2C_ACK_VAL);

  i2c_master_start(cmd);
  i2c_master_write_byte(cmd, (adt7410_info->adt7410_addr << 1) | I2C_MASTER_READ, I2C_ACK_VAL);
  i2c_master_read(cmd, temp_reg, 2, I2C_NACK_VAL);

  i2c_master_stop(cmd);

  esp_err_t ret = i2c_master_cmd_begin(adt7410_info->i2c_port, cmd, I2C_CMD_TIMEOUT);
  i2c_cmd_link_delete(cmd);

  if(ret == ESP_OK) {
    temp_adc = temp_reg[0] * 0x100 + temp_reg[1];
    temp_neg = (temp_adc & 0x8000) ? 1 : 0;
    if(adt7410_info->reg_config & ADT7410_CONFIG_RES_16BIT) {
      temp_adc -= 65536*temp_neg;
      temp = (float)temp_adc / 128.0;
    } else {
      temp_adc = (temp_adc & 0xfff8) >> 3;
      temp_adc -= 8192*temp_neg;
      temp = (float)temp_adc / 16.0;
    }
    switch(temp_units) {
      case ADT7410_TEMP_UNIT_CELCIUS:
        *temperature = temp;
        break;
      case ADT7410_TEMP_UNIT_FARENHEIT:
        *temperature = temp*9/5 + 32;
        break;
      case ADT7410_TEMP_UNIT_KELVIN:
        *temperature = temp + 273.15;
        break;
      default:
        *temperature = NAN;
    }
  } else {
    ESP_LOGE(TAG,"Error while reading temperature from sensor");
  }
  return(ret);
}
