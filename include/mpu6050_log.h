/** @file mpu6050_log.h
 * @date 2025/08/29
 * @author Luong Huu Phuc
 * @brief MPU6050 config logger
 */

#ifndef __INC_MPU6050_LOG_H
#define __INC_MPU6050_LOG_H

#ifdef __cplusplus
extern "C" {
#endif //__cplusplus

#pragma once 

#include "stdio.h"
#include "stdint-gcc.h"
#include "stdbool.h"
#include "string.h"
#include "esp_log.h"
#include "esp_err.h"
#include "mpu6050.h"

/* PROTOTYPE - FILE HEADER khong can ghi Extern vi mac dinh prototype cua ham da co tinh chat Extern */

//Source clock logger
const char *mpu6050_src_clock_to_str(mpu6050_clock_src_t clk);

//Sleep mode logger
const char *mpu6050_sleep_mode_to_str(mpu6050_sleep_mode_t sleep_mode);

//Low Power Cycle mode logger
const char *mpu6050_low_pwr_cycle_mode_to_str(mpu6050_low_pwr_cycle_mode_t cycle_mode);

//Wake-ups frequency logger
const char *mpu6050_wakeup_freq_to_str(mpu6050_wakeup_freq_t wakeup_freq);

//Temperature sensor enable/disable logger
const char *mpu6050_temp_sensor_to_str(mpu6050_config_t *config);

//Standby mode logger
const char *mpu6050_standby_mode_to_str(mpu6050_standby_mode_t standby_mode);

//Gyro range logger
const char *mpu6050_gyro_range_to_str(mpu6050_gyro_range_t range);

//Accel range logger
const char *mpu6050_accel_range_to_str(mpu6050_accel_range_t range);

//Digital Low Pass Filter logger
const char *mpu6050_config_dlpf_to_str(mpu6050_dlpf_t dlpf_cfg);

//Soft Reset logger
const char *mpu6050_soft_reset_to_str(mpu6050_soft_reset_t softReset, uint8_t custom_mask);

//Interrupt enable logger
const char *mpu6050_interrupt_enable_to_str(mpu6050_interrupt_enable_t *int_enable);

//Interrupt config logger
const char *mpu6050_interrupt_config_to_str(mpu6050_interrupt_config_t *int_cfg);

//FIFO config logger
const char *mpu6050_fifo_config_to_str(mpu6050_config_t *config);


#ifdef __cplusplus
}
#endif //__cplusplus

#endif //__INC_MPU6050_LOG_H