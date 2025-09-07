/** MPU6050 LOW LEVEL API
 * @file mpu6050_ll.c
 * @date 2025/08/30
 * @author Luong Huu Phuc
 * @note Ham de read-write du lieu tu I2C
 */

#ifdef __cplusplus
extern "C" {
#endif

#include "stdio.h"
#include "stdint.h"
#include "esp_err.h"
#include "I2C_dev.h"

/* WRITE BYTE*/

esp_err_t mpu6050_write_bytes_to_reg(I2C_dev_init_t *dev, uint8_t reg, uint8_t *data, uint8_t len){
  if(data == NULL || len == 0){
    return ESP_ERR_INVALID_ARG;
  }

  if(!i2c_dev_write_bytes(dev, dev->address, reg, len, data, NULL)){
    return ESP_FAIL;
  }
  return ESP_OK;
}

esp_err_t mpu6050_write_byte_to_reg(I2C_dev_init_t *dev, uint8_t reg, uint8_t data){
  if(dev == NULL){
    return ESP_ERR_INVALID_ARG;
  }

  if(!i2c_dev_write_byte(dev, dev->address, reg, data, NULL)){
    return ESP_FAIL;
  }
  return ESP_OK;
}

/* READ BYTE */

esp_err_t mpu6050_read_bytes_from_reg(I2C_dev_init_t *dev, uint8_t reg, uint8_t *data, uint8_t len){
  if(data == NULL || len == 0){
    return ESP_ERR_INVALID_ARG;
  }

  if(i2c_dev_read_bytes(dev, dev->address, reg, len, data, I2CDEV_DEFAULT_READ_TIMEOUT, NULL) != ESP_OK){
    return ESP_FAIL;
  }
  return ESP_OK;
}

esp_err_t mpu6050_read_byte_from_reg(I2C_dev_init_t *dev, uint8_t reg, uint8_t *data){
  if(data == NULL){
    return ESP_ERR_INVALID_ARG;
  }

  if(i2c_dev_read_byte(dev, dev->address, reg, data, I2CDEV_DEFAULT_READ_TIMEOUT, NULL) != ESP_OK){
    return ESP_FAIL;
  }
  return ESP_OK;
}


#ifdef __cplusplus
}
#endif