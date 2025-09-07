/** @file mpu6050_log.c
 * @date 2025/08/29
 * @author Luong Huu Phuc
 * @brief MPU6050 config logger
 */

#ifdef __cplusplus
extern "C" {
#endif //__cplusplus

#include "stdio.h"
#include "stdint-gcc.h"
#include "stdbool.h"
#include "string.h"
#include "esp_log.h"
#include "esp_err.h"
#include "mpu6050.h"

const char *mpu6050_src_clock_to_str(mpu6050_clock_src_t clk){
  switch(clk){
    case MPU6050_CLK_INTERNAL: return "Internal 8MHz oscillator";
    case MPU6050_CLK_PPL_XGYRO: return "PPL with X axis gyro reference";
    case MPU6050_CLK_PPL_YGYRO: return "PPL with Y axis gyro reference";
    case MPU6050_CLK_PPL_ZGYRO: return "PPL with Z axis gyro reference";
    case MPU6050_CLK_EXT_32K: return "External 32.768 kHz reference";
    case MPU6050_CLK_EXT_19M: return "External 19MHz reference";
    case MPU6050_CLK_STOP: return "Stop clock (Sensor sleep)";
    default: return "Unknown clock source";
  }
}

const char *mpu6050_sleep_mode_to_str(mpu6050_sleep_mode_t sleep_mode){
  switch(sleep_mode){
    case MPU6050_SLEEP_MODE_DISABLE: return "Sleep mode OFF";
    case MPU6050_SLEEP_MODE_ENABLE: return "Sleep mode ON";
    default: return "Invalid arg for Sleep mode";
  }
}

const char *mpu6050_low_pwr_cycle_mode_to_str(mpu6050_low_pwr_cycle_mode_t cycle_mode){
  switch(cycle_mode){
    case MPU6050_CYCLE_MODE_DISABLE: return "Low Power Cycle mode OFF";
    case MPU6050_CYCLE_MODE_ENABLE: return "Low Power Cycle mode ON";
    default: return "Invalid arg for Low Power Cycle mode";
  }
}

const char *mpu6050_wakeup_freq_to_str(mpu6050_wakeup_freq_t wakeup_freq){
  switch(wakeup_freq){
    case MPU6050_WAKEUP_FREQ_1_25HZ: return "Wake-ups freq: 1.25Hz";
    case MPU6050_WAKEUP_FREQ_5HZ: return "Wake-ups freq: 5Hz";
    case MPU6050_WAKEUP_FREQ_20HZ: return "Wake-ups freq: 20Hz";
    case MPU6050_WAKEUP_FREQ_40HZ: return "Wake-ups freq: 40Hz";
    default: return "Invalid arg for Wake-ups freq";
  }  
}

const char *mpu6050_temp_sensor_to_str(mpu6050_config_t *config){
  static char buffer[50];
  buffer[0] = '\0'; //Clear

  if(config->temperature){ //Neu gia tri nhan duoc la true
    strcpy(buffer, "Temperature sensor ON");
  }else{
    strcpy(buffer, "Temperature sensor OFF"); 
  }
  return buffer;
}

const char *mpu6050_standby_mode_to_str(mpu6050_standby_mode_t standby_mode){
  static char buffer[128];
  buffer[0] = '\0'; //Clear

  //strcat() de xau cac chuoi lai voi nhau
  if(standby_mode.standby_x_accel) strcat(buffer, "X-Accel standby |");
  if(standby_mode.standby_y_accel) strcat(buffer, "Y-Accel standby |");
  if(standby_mode.standby_z_accel) strcat(buffer, "Z-Accel standby |");
  if(standby_mode.standby_x_gyro) strcat(buffer, "X-Gyro standby |");
  if(standby_mode.standby_y_gyro) strcat(buffer, "Y-Gyro standby |");
  if(standby_mode.standby_z_gyro) strcat(buffer, "Z-Gyro standby |");

  if(strlen(buffer) == 0 || standby_mode.disable_standby == true) strcpy(buffer, "All axies active !"); //Neu buffer khong co ky tu nao
  return buffer;
}

const char *mpu6050_gyro_range_to_str(mpu6050_gyro_range_t range){
  switch(range){
    case MPU6050_GYRO_FS_250DPS: return "Gyro range set 250dps (131 LSB°/s)";
    case MPU6050_GYRO_FS_500DPS: return "Gyro range set 500dps (66.5 LSB°/s)";
    case MPU6050_GYRO_FS_1000DPS: return "Gyro range set 1000dps (32.8 LSB°/s)";
    case MPU6050_GYRO_FS_2000DPS: return "Gyro range set 2000dps (16.4 LSB°/s)";
    default: return "Invalid arg for Gyro Range !";
  }
}


const char *mpu6050_accel_range_to_str(mpu6050_accel_range_t range){
  switch(range){
    case MPU6050_ACCEL_FS_2G: return "Accel range set 2g (16384 LSB/g)";
    case MPU6050_ACCEL_FS_4G: return "Accel range set 4g (8192 LSB/g)";
    case MPU6050_ACCEL_FS_8G: return "Accel range set 8g (4096 LSB/g)";
    case MPU6050_ACCEL_FS_16G: return "Accel range set 16g (2048 LSB/g)";
    default: return "Invalid arg for Accel Range !";
  }
}


const char *mpu6050_config_dlpf_to_str(mpu6050_dlpf_t dlpf_cfg){
  switch(dlpf_cfg){
    case MPU6050_DLPF_5HZ: return "Bandwidth 5Hz";
    case MPU6050_DLPF_10HZ: return "Bandwidth 10Hz";
    case MPU6050_DLPF_21HZ: return "Bandwidth 21Hz";
    case MPU6050_DLPF_44HZ: return "Bandwidth 44Hz";
    case MPU6050_DLPF_94HZ: return "Bandwidth 94Hz";
    case MPU6050_DLPF_184HZ: return "Bandwidth 184Hz";
    case MPU6050_DLPF_260HZ: return "Bandwidth 260Hz";
    case MPU6050_DLPF_OFF: return "No Bandwidth ";
    default: return "Invalid arg for DLPF config";
  }
}

const char *mpu6050_soft_reset_to_str(mpu6050_soft_reset_t softReset, uint8_t custom_mask){
  switch(softReset){
    case MPU6050_RESET_ALL: return "Reset Accel + Gyro + Temp";
    case MPU6050_RESET_ACCEL: return "Reset Accel Only";
    case MPU6050_RESET_GYRO: return "Reset Gyro Only";
    case MPU6050_RESET_TEMP: return "Reset Temp Only";
    case MPU6050_RESET_CUSTOM:
      if(custom_mask == (MPU6050_RESET_ACCEL | MPU6050_RESET_GYRO)) return "Reset Accel + Gyro";
      if(custom_mask == (MPU6050_RESET_ACCEL | MPU6050_RESET_TEMP)) return "Reset Accel + Temp";
      if(custom_mask == (MPU6050_RESET_GYRO | MPU6050_RESET_TEMP)) return "Reset Gyro + Temp";
      return "Invalid custom mask !"; //Return trong case vi complier khong chac if kia se return 
    default: return "Invalid arg for soft reset !";
  }
}


const char *mpu6050_interrupt_enable_to_str(mpu6050_interrupt_enable_t *int_enable){
  static char buffer[100];
  buffer[0] = '\0';
  
  if(int_enable->disable_int == true){
    strcpy(buffer, "But no int enable !");
  }else{
    if(int_enable->data_ready_int) strcat(buffer, "Data_Ready_Int | ");
    if(int_enable->fifo_overflow_int) strcat(buffer, "FIFO_overflow_Int | ");
    if(int_enable->i2c_master_int) strcat(buffer, "I2C_master_int | ");
  }
  return buffer;
}


const char *mpu6050_interrupt_config_to_str(mpu6050_interrupt_config_t *int_cfg){
  static char buffer[200];
  buffer[0] = '\0'; //Clear buffer

  if(strlen(buffer) == 0 || int_cfg->disable == true){
    strcpy(buffer, "Interrupt no config, auto disable all !"); //Neu khong cau hinh ngat
  }else{
    //INT level
    strcat(buffer, "INT level: ");
    strcat(buffer, int_cfg->int_level ? "Active Low, " : "Active High, ");

    //INT pin mode
    strcat(buffer, "INT Open: ");
    strcat(buffer, int_cfg->int_open ? "Open-Drain, " : "Push-Pull, ");

    //Latch or pulse
    strcat(buffer, "Latch: ");
    strcat(buffer, int_cfg->latch_int_en ? "Hold until int cleared, " : "Pulse 50us, ");

    //Clear behaviour
    strcat(buffer, "Clear: ");
    strcat(buffer, int_cfg->int_rd_clear ? " Any Register Read, " : "INT_STATUS Read, ");

    //FSYNC interrupt level
    strcat(buffer, "FSYNC level: ");
    strcat(buffer, int_cfg->fsync_int_level ? "Active Low, " : "Active High, ");

    //FSYNC interrupt enable
    strcat(buffer, "FSYNC Initerrupt: ");
    strcat(buffer, int_cfg->fsync_int_en ? " Enabled, " : "Disabled, ");

    //I2C bypass
    strcat(buffer, "I2C Bypass: ");
    strcat(buffer, int_cfg->i2c_bypass_en ? "Enablled, " : "Disabled, ");
  }
  return buffer;
}     

const char *mpu6050_fifo_config_to_str(mpu6050_config_t *config){
  static char buffer[100];
  buffer[0] = '\0';

  strcat(buffer, "Accel FIFO: ");
  strcat(buffer, config->accel_en ? "Enabled, " : "Disabled, ");

  strcat(buffer, "Gyro FIFO: ");
  strcat(buffer, config->gyro_en ? "Enabled, " : "Disabled, ");

  strcat(buffer, "Temp FIFO: ");
  strcat(buffer, config->temp_en ? "Enabled, " : "Disabled, ");

  return buffer;
}

#ifdef __cplusplus
}
#endif //__cplusplus