/** MPU6050 LOW LEVEL API
 * @file mpu6050_ll.h
 * @date 2025/08/30
 * @author Luong Huu Phuc
 * @note Ham de read-write du lieu tu I2C
 */

#ifndef __INC_MPU6050_LL_H
#define __INC_MPU6060_LL_H

#ifdef __cplusplus
extern "C" {
#endif

#pragma once 

#include "stdio.h"
#include "stdint.h"
#include "esp_err.h"
#include "I2C_dev.h"

/**
 * @brief Ham ghi nhieu bytes vao thanh thanh ghi
 * 
 * @param dev Doi tuong I2C  
 * @param data Mang chua du lieu cac bytes can ghi vao 
 * @param reg Thanh ghi can ghi vao
 * @param len Chieu dai bytes can ghi vao
 * @return ESP-OK -> Success
 */
esp_err_t mpu6050_write_bytes_to_reg(I2C_dev_init_t *dev, uint8_t reg, uint8_t *data, uint8_t len);

/**
 * @brief Ham ghi 1 byte vao thanh ghi 
 * 
 * @param dev Doi tuong I2C
 * @param data Byte can ghi vao 
 * @param reg Thanh ghi can ghi vao
 * @return ESP-OK -> Success
 */
esp_err_t mpu6050_write_byte_to_reg(I2C_dev_init_t *dev, uint8_t reg, uint8_t data);

/**
 * @brief Ham de doc nhieu bytes tu thanh ghi 
 * 
 * @note Can truyen bien vao `data` de luu gia tri doc duoc 
 * @param dev Doi tuong I2C
 * @param reg Thanh ghi can doc
 * @param data Bytes can doc
 * @param len Chieu dai bytes
 * @return ESP-OK -> Success
 */
esp_err_t mpu6050_read_bytes_from_reg(I2C_dev_init_t *dev, uint8_t reg, uint8_t *data, uint8_t len);

/**
 * @brief Ham de 1 byte tu thanh ghi
 * 
 * @note Can truyen bien vao `data` de luu gia tri doc duoc 
 * @param dev Doi tuong I2C
 * @param reg Thanh ghi can doc
 * @param data Bytes can doc
 * @return ESP-OK -> Success
 */
esp_err_t mpu6050_read_byte_from_reg(I2C_dev_init_t *dev, uint8_t reg, uint8_t *data);


#ifdef __cplusplus
}
#endif //__cplusplus
 
#endif //__INC_MPU6050_LL_H 
