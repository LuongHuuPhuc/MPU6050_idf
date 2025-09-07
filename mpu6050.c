/** @file mpu6050.c
 * @date 2025/08/29
 * @author Luong Huu Phuc
 * @brief MPU 6050 driver
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
#include "driver/i2c.h"
#include "I2C_dev.h"
#include "mpu6050_ll.h"
#include "mpu6050.h"
#include "mpu6050_log.h"

static const float accel_sensitivity[4] = {
  16384.0f, // ±2g
  8192.0f,  // ±4g
  4096.0f,  // ±8g
  2048.0f   // ±16g
};

static const float gyro_sensitivity[4] = {
  131.0f, // ±250 dps
  65.5f,  // ±500 dps
  32.8f,  // ±1000 dps
  16.4f   // ±2000 dps
};

esp_err_t mpu6050_I2C_dev_config(I2C_dev_init_t *dev, i2c_port_t port, uint8_t sda_pin, uint8_t scl_pin){
  IS_NULL_ARG(dev);

  dev->address = MPU6050_DEFAULT_ADRR_0;
  dev->port = port;
  dev->cfg.mode = I2C_MODE_MASTER;
  dev->cfg.scl_io_num = scl_pin;
  dev->cfg.sda_io_num = sda_pin;
  dev->cfg.scl_pullup_en = GPIO_PULLUP_DISABLE;
  dev->cfg.sda_pullup_en = GPIO_PULLUP_DISABLE;
  dev->timeout_ticks = pdMS_TO_TICKS(I2CDEV_DEFAULT_READ_TIMEOUT);

#if defined(CONFIG_IDF_TARGET_ESP32C3)
  dev->cfg.master.clk_speed = I2CDEV_FREQ_HZ;
#endif

  i2c_dev_create_mutex(dev);

  return ESP_OK;
}

esp_err_t __attribute__((unused))mpu6050_I2C_driver_config(i2c_port_t port, uint8_t sda_pin, uint8_t scl_pin){
  i2c_config_t config = {
    .mode = I2C_MODE_MASTER,
    .master.clk_speed = I2CDEV_FREQ_HZ,
    .scl_io_num = scl_pin,
    .sda_io_num = sda_pin,
    .scl_pullup_en = GPIO_PULLUP_DISABLE,
    .sda_pullup_en = GPIO_PULLUP_DISABLE
  };
  ESP_ERROR_CHECK(i2c_param_config(I2C_NUM, &config));
  ESP_ERROR_CHECK(i2c_driver_install(I2C_NUM, config.mode, 0, 0, 0));

  return ESP_OK;
}

esp_err_t mpu6050_verify_whoami(I2C_dev_init_t *dev){
  uint8_t ret_data = 0;

  I2C_DEV_TAKE_MUTEX(dev);
  I2C_DEV_CHECK(dev, mpu6050_read_byte_from_reg(dev, MPU6050_WHO_AM_I_REG, &ret_data));
  I2C_DEV_GIVE_MUTEX(dev);

  if(ret_data == MPU6050_DEFAULT_ADRR_0 || ret_data == MPU6050_DEFAULT_ADDR_1){
    return ESP_OK;
  }else return ESP_FAIL;
}

esp_err_t mpu6050_clear_reg(I2C_dev_init_t *dev, uint8_t reg){
  esp_err_t err = mpu6050_write_byte_to_reg(dev, reg, 0x00);
  if(err != ESP_OK){
    ESP_LOGE("MPU6050", "Failed to clear register 0x%02X: %s", reg, esp_err_to_name(err));
    return err;
  }
  return ESP_OK;
}

esp_err_t mpu6050_softReset(I2C_dev_init_t *dev, mpu6050_soft_reset_t reset_type, uint8_t custom_mask){
  uint8_t ret_data = 0;

  switch(reset_type){
    case MPU6050_RESET_ALL:
      ret_data = ((1u << 0) | (1u << 1) | (1u << 2)); //Reset ACCEL + GYRO + TEMP
      break;
    case MPU6050_RESET_GYRO: //Bit2
      ret_data = (1 << 2); //0x04
      break;
    case MPU6050_RESET_ACCEL: //Bit1
      ret_data = (1 << 1); //0x02
      break;
    case MPU6050_RESET_TEMP: //Bit0
      ret_data = (1 << 0); //0x01
      break;
    case MPU6050_RESET_CUSTOM:
      ret_data = custom_mask; //Tu custom 
      break;
    default:
      return ESP_ERR_INVALID_ARG;
  }
  esp_err_t ret = mpu6050_write_byte_to_reg(dev, MPU6050_SIGNAL_PATH_RESET, ret_data);
  if(ret != ESP_OK) return ret;
  vTaskDelay(pdMS_TO_TICKS(5));
  return ret;
}

esp_err_t mpu6050_deviceReset(I2C_dev_init_t *dev){
  esp_err_t ret = mpu6050_write_byte_to_reg(dev, MPU6050_PWR_MGMT_1, MPU6050_DEVICE_RESET);
  if(ret != ESP_OK) return ret;
  vTaskDelay(pdMS_TO_TICKS(100)); //Datasheet yeu cau delay xong (100ms)
  return ESP_OK;
}

esp_err_t mpu6050_sleep_mode(I2C_dev_init_t *dev, mpu6050_sleep_mode_t sleep){
  uint8_t ret_data = 0;

  //Doc gia tri hien tai cua thanh ghi PWR_MMGT_1
  esp_err_t ret = mpu6050_read_byte_from_reg(dev, MPU6050_PWR_MGMT_1, &ret_data);
  if(ret != ESP_OK) return ret;

  switch(sleep){
    case MPU6050_SLEEP_MODE_ENABLE:
      ret_data |= MPU6050_BITMASK_6; //0x40
      break;
    case MPU6050_SLEEP_MODE_DISABLE:
      ret_data &= ~MPU6050_BITMASK_6; //Clear bit6
      break;
    default: 
      ret_data &= ~MPU6050_BITMASK_6; //Khong set gi thi mac dinh disable
      break;
  }

  I2C_DEV_TAKE_MUTEX(dev);
  I2C_DEV_CHECK(dev, mpu6050_write_byte_to_reg(dev, MPU6050_PWR_MGMT_1, ret_data)); //Ghi lai vao thanh ghi
  I2C_DEV_GIVE_MUTEX(dev);

  return ESP_OK;
}

esp_err_t mpu6050_low_pwr_cycle_mode(I2C_dev_init_t *dev, mpu6050_low_pwr_cycle_mode_t cycle_mode){
  IS_NULL_ARG(dev);
  uint8_t value = 0;

  //Doc thanh ghi PWR_MGMT_1 0x6B hien tai de kiem tra 
  I2C_DEV_TAKE_MUTEX(dev);
  I2C_DEV_CHECK(dev, mpu6050_read_byte_from_reg(dev, MPU6050_PWR_MGMT_1, &value));
  I2C_DEV_GIVE_MUTEX(dev);

  switch(cycle_mode){
    case MPU6050_CYCLE_MODE_ENABLE:
      //Neu Bit6 SLEEP dang enable
      if(value & MPU6050_BITMASK_6) value &= ~(MPU6050_BITMASK_6); //Clear Bit6
      value |= MPU6050_BITMASK_5; //Enable Bit5 CYCLE
      break;
    case MPU6050_CYCLE_MODE_DISABLE:
      value &= ~(MPU6050_BITMASK_5); //Clear Bit5
      break;
    default: 
      value &= ~(MPU6050_BITMASK_5); //Clear Bit5
  }

  //Ghi lai vao thanh ghi PWR_MGMT_1
  I2C_DEV_TAKE_MUTEX(dev);
  I2C_DEV_CHECK(dev, mpu6050_write_byte_to_reg(dev, MPU6050_PWR_MGMT_1, value));
  I2C_DEV_GIVE_MUTEX(dev);

  return ESP_OK;
}

esp_err_t mpu6050_set_wakeups_freq(I2C_dev_init_t *dev, mpu6050_wakeup_freq_t wakeup_freq){
  uint8_t ret_data = 0;
  uint8_t value = 0;

  //Doc gia tri thanh ghi PWR_MGMT_2 hien tai
  I2C_DEV_TAKE_MUTEX(dev);
  I2C_DEV_CHECK(dev, mpu6050_read_byte_from_reg(dev, MPU6050_PWR_MGMT_2, &ret_data));
  I2C_DEV_GIVE_MUTEX(dev);

  //Clear bit[7:6]
  ret_data &= ~(0x03 << 6); //1100 0000 -> 0011 1111 

  //Set tan so sau khi da clear 2 bit[7:6]
  switch(wakeup_freq){
    case MPU6050_WAKEUP_FREQ_1_25HZ:
      value = (0x00 << 6); 
      break;
    case MPU6050_WAKEUP_FREQ_5HZ:
      value = (0x01 << 6);
      break;
    case MPU6050_WAKEUP_FREQ_20HZ:
      value = (0x02 << 6);
      break;
    case MPU6050_WAKEUP_FREQ_40HZ:
      value = (0x03 << 6);
      break;
    default:
      return ESP_ERR_INVALID_ARG;
  }

  ret_data |= value;

  I2C_DEV_TAKE_MUTEX(dev);
  I2C_DEV_CHECK(dev, mpu6050_write_byte_to_reg(dev, MPU6050_PWR_MGMT_2, ret_data));
  I2C_DEV_GIVE_MUTEX(dev);

  return ESP_OK;
}

esp_err_t mpu6050_enable_temp(I2C_dev_init_t *dev, bool enable){
  uint8_t ret_data = 0;

  //Doc gia tri hien tai cua Bit3 trong PWR_MGMT_1 
  esp_err_t ret = mpu6050_read_byte_from_reg(dev, MPU6050_PWR_MGMT_1, &ret_data);
  if(ret != ESP_OK) return ret;

  if(enable){
    ret_data &= ~(1 << 3); //Clear bit3 de bat 
  } else {
    ret_data |= (1 << 3); // Set bit3 -> tat temp sensor
  }

  I2C_DEV_TAKE_MUTEX(dev);
  I2C_DEV_CHECK(dev, mpu6050_write_byte_to_reg(dev, MPU6050_PWR_MGMT_1, ret_data));
  I2C_DEV_GIVE_MUTEX(dev);

  return ESP_OK;
}

esp_err_t mpu6050_set_accel_range(I2C_dev_init_t *dev, mpu6050_accel_range_t range, mpu6050_sensitivity_t *sens){
  uint8_t ret_data = 0;
  mpu6050_sensitivity_t sens_local;

  //Doc gia tri hien tai cua thanh ghi MPU6050_ACCEL_CONFIG 0x1C
  esp_err_t ret = mpu6050_read_byte_from_reg(dev, MPU6050_ACCEL_CONFIG, &ret_data);
  if(ret != ESP_OK) return ret;

  //Clear bits[4:3]
  ret_data &= ~(0x3 << 3); //0001 1000 -> 1110 0111

  //Set theo range sau khi clear
  ret_data |= (range << 3);

  I2C_DEV_TAKE_MUTEX(dev);
  I2C_DEV_CHECK(dev, mpu6050_write_byte_to_reg(dev, MPU6050_ACCEL_CONFIG, ret_data));
  I2C_DEV_GIVE_MUTEX(dev);

  //Update sensitivity 
  memset(&sens_local.accel_lsb_per_g, 0, sizeof(float));
  sens_local.accel_lsb_per_g = accel_sensitivity[range];
  memcpy(&sens->accel_lsb_per_g, &sens_local.accel_lsb_per_g, sizeof(float));

  return ESP_OK;
}

esp_err_t mpu6050_set_gyro_range(I2C_dev_init_t *dev, mpu6050_gyro_range_t range, mpu6050_sensitivity_t *sens){
  uint8_t ret_data = 0;
  mpu6050_sensitivity_t sens_local;

  //Doc gia tri hien tai cua thanh ghi
  esp_err_t ret = mpu6050_read_byte_from_reg(dev, MPU6050_GYRO_CONFIG, &ret_data);
  if(ret != ESP_OK) return ret;

  //Clear bit[4:3]
  ret_data &= ~(0x3 << 3); //0000 0011 -> 0001 1000 -> 1110 0111

  //Set theo range sau khi clear
  ret_data |= (range << 3); //ex: 0000 0001 -> 0000 1000 | 1110 0111 

  I2C_DEV_TAKE_MUTEX(dev);
  I2C_DEV_CHECK(dev, mpu6050_write_byte_to_reg(dev, MPU6050_GYRO_CONFIG, ret_data));
  I2C_DEV_GIVE_MUTEX(dev);

  //Update sensitivity
  memset(&sens_local, 0, sizeof(float));
  sens_local.gyro_lsb_per_dps = gyro_sensitivity[range];
  memcpy(&sens->gyro_lsb_per_dps, &sens_local.gyro_lsb_per_dps, sizeof(float));
  return ESP_OK;
}

esp_err_t mpu6050_standby_mode(I2C_dev_init_t *dev, mpu6050_standby_mode_t *standby_mode){
  IS_NULL_ARG(dev);
  uint8_t reg_val = 0;
  esp_err_t ret = ESP_OK;

  //Doc gia tri hien tai cua thanh ghi PWR_MGMT_2
  I2C_DEV_TAKE_MUTEX(dev);
  ret = mpu6050_read_byte_from_reg(dev, MPU6050_PWR_MGMT_2, &reg_val);
  I2C_DEV_GIVE_MUTEX(dev);

  if(ret != ESP_OK){
    ESP_LOGE(pcTaskGetName(NULL), "mpu6050_standby_mode: read PWR_MGMT_2 failed at: %s", esp_err_to_name(ret));
    return ret;
  }

  ESP_LOGD(pcTaskGetName(NULL), "mpu6050_standby_mode: PWR_MGMT_2 before = 0x%02x", reg_val);

  //Set clear tung bit dua vao struct
  reg_val &= ~(MPU6050_BITMASK_0 | MPU6050_BITMASK_1 | MPU6050_BITMASK_2 | MPU6050_BITMASK_3 |
                MPU6050_BITMASK_4 | MPU6050_BITMASK_5); //Clear Bit5->Bit0

  if(standby_mode->disable_standby == true){ //Neu tham so truyen vao la NULL hoac 0
    //Giu nguyen khong thay doi gi
    ESP_LOGD(pcTaskGetName(NULL), "mpu6050_standby_mode: standby mode disabled (no axis in standby)");
  }else{
    reg_val |= (standby_mode->standby_z_gyro ? (1 << 0) : 0);
    reg_val |= (standby_mode->standby_y_gyro ? (1 << 1) : 0);
    reg_val |= (standby_mode->standby_x_gyro ? (1 << 2) : 0);
    reg_val |= (standby_mode->standby_z_accel ? (1 << 3) : 0);
    reg_val |= (standby_mode->standby_y_accel ? (1 << 4) : 0);
    reg_val |= (standby_mode->standby_x_accel ? (1 << 5) : 0);
    ESP_LOGD(pcTaskGetName(NULL), "mpu6050_standby_mode: setting PWR_MGMT_2 -> 0x%02x", reg_val);
  }

  I2C_DEV_TAKE_MUTEX(dev);
  ret = mpu6050_write_byte_to_reg(dev, MPU6050_PWR_MGMT_2, reg_val);
  I2C_DEV_GIVE_MUTEX(dev);

  if(ret != ESP_OK){
    ESP_LOGE(pcTaskGetName(NULL), "mpu6050_standby_mode: write PWR_MMGT_2 failed: %s", esp_err_to_name(ret));
    return ret;
  }
  return ESP_OK;
}

esp_err_t mpu6050_config_dlpf(I2C_dev_init_t *dev, mpu6050_dlpf_t bandwidth){
  uint8_t value = 0;
  
  //Doc gia tri hien tai cua thanh ghi CONFIG 0x1A
  I2C_DEV_TAKE_MUTEX(dev);
  I2C_DEV_CHECK(dev, mpu6050_read_byte_from_reg(dev, MPU6050_CONFIG, &value));
  I2C_DEV_GIVE_MUTEX(dev);

  //Clear 3 bit (Bit2 -> Bit0) de chuan bi ghi lai
  value &= ~(0x07);

  //Set lai gia tri DLPF theo enum bandwidth
  value |= (bandwidth & 0x07);

  //Ghi lai vao thanh ghi CONFIG
  I2C_DEV_TAKE_MUTEX(dev);
  I2C_DEV_CHECK(dev, mpu6050_write_byte_to_reg(dev, MPU6050_CONFIG, value));
  I2C_DEV_GIVE_MUTEX(dev);

  return ESP_OK;
}

esp_err_t mpu6050_set_sampleRate_div(I2C_dev_init_t *dev, uint8_t smprt_div){
  I2C_DEV_TAKE_MUTEX(dev);
  I2C_DEV_CHECK(dev, mpu6050_write_byte_to_reg(dev, MPU6050_SMPRT_DIV, smprt_div));
  I2C_DEV_GIVE_MUTEX(dev);

  return ESP_OK;
}

uint8_t mpu6050_get_sampleRate_div(I2C_dev_init_t *dev){
  uint8_t value = 0;
  I2C_DEV_TAKE_MUTEX(dev);
  I2C_DEV_CHECK(dev, mpu6050_read_byte_from_reg(dev, MPU6050_SMPRT_DIV, &value));
  I2C_DEV_GIVE_MUTEX(dev);

  return (value);
}

esp_err_t mpu6050_set_sampleRate_Hz(I2C_dev_init_t *dev, float freq_hz){
  uint8_t cfg = 0;
  esp_err_t ret; 

  //Doc thanh ghi CONFIG de xem DLPF_CFG hien tai
  I2C_DEV_TAKE_MUTEX(dev);
  I2C_DEV_CHECK(dev, mpu6050_read_byte_from_reg(dev, MPU6050_CONFIG, &cfg));
  I2C_DEV_GIVE_MUTEX(dev);

  uint8_t dlpf_cfg = cfg & 0x07;

  //Tan so goc (fs_base) phu thuoc vao DLPF_CFG 
  float fs_base = (dlpf_cfg == 0 || dlpf_cfg == 7) ? 8000.0f : 1000.0f;

  int divider = (int)((fs_base / freq_hz) - 1.0f);

  //Gioi han gia tri
  if(divider > 255) divider = 255;
  if(divider < 0) divider = 0;

  //Ghi vao thanh ghi SMPRT_DIV
  uint8_t div_val = (uint8_t)divider;
  ret = mpu6050_set_sampleRate_div(dev, div_val);

  return ret;
}

float mpu6050_get_sampleRate_Hz(I2C_dev_init_t *dev){
  uint8_t smprt_div = 0;
  uint8_t config = 0;
  float fs_base = 0.0f;

  //Doc de lay gia tri so lan chia tan so trong SMPRT_DIV
  smprt_div = mpu6050_get_sampleRate_div(dev);

  //Doc de lay gia tri DLPG_CFG tu thanh ghi CONFIG
  I2C_DEV_TAKE_MUTEX(dev);
  I2C_DEV_CHECK(dev, mpu6050_read_byte_from_reg(dev, MPU6050_CONFIG, &config));
  I2C_DEV_GIVE_MUTEX(dev);

  uint8_t dlpf_cfg = config & 0x07; // 3 bit [2:0]

  if(dlpf_cfg == 0 || dlpf_cfg == 7){
    fs_base = 8000.0f; //8kHz (datatsheet)
  }else {
    fs_base = 1000.0f; //1kHz (datasheet)
  }

  return (fs_base / (1 + smprt_div));
}

esp_err_t mpu6050_set_source_clock(I2C_dev_init_t *dev, mpu6050_clock_src_t src_clk){
  IS_NULL_ARG(dev);
  uint8_t value = 0;

  I2C_DEV_TAKE_MUTEX(dev);
  I2C_DEV_CHECK(dev, mpu6050_read_byte_from_reg(dev, MPU6050_PWR_MGMT_1, &value));
  I2C_DEV_GIVE_MUTEX(dev);

  //Clear Bit2->Bit0 CLKSEL[2:0]
  value &= ~(MPU6050_BITMASK_0 | MPU6050_BITMASK_1 | MPU6050_BITMASK_2);

  //Ghi gia tri moi vao CLKSEL
  value |= src_clk & 0x07;

  //Ghi lai vao thanh ghi PWR_MGMT_1
  I2C_DEV_TAKE_MUTEX(dev);
  I2C_DEV_CHECK(dev, mpu6050_write_byte_to_reg(dev, MPU6050_PWR_MGMT_1, value));
  I2C_DEV_GIVE_MUTEX(dev);

  return ESP_OK;
}

esp_err_t mpu6050_interrupt_enable(I2C_dev_init_t *dev, mpu6050_interrupt_enable_t *enable){
  IS_NULL_ARG(dev);
  uint8_t value = 0;

  //Clear toan bo bit (Thanh ghi nay chi su dung bit4, bit3 va bit0)
  mpu6050_clear_reg(dev, MPU6050_INT_ENABLE);

  //Build bitmask
  if(enable->disable_int == true || enable == NULL) value = 0x00; //Clear toan bo thanh ghi 
  else{
    if(enable->data_ready_int) value |= (0x01 << 0);
    if(enable->i2c_master_int) value |= (0x01 << 3);
    if(enable->fifo_overflow_int) value |= (0x01 << 4);
  }

  //Ghi lai vao thanh ghi INT_ENABLE
  I2C_DEV_TAKE_MUTEX(dev);
  I2C_DEV_CHECK(dev, mpu6050_write_byte_to_reg(dev, MPU6050_INT_ENABLE, value));
  I2C_DEV_GIVE_MUTEX(dev);

  return ESP_OK;
}

esp_err_t mpu6050_interrupt_get_status(I2C_dev_init_t *dev, mpu6050_interrupt_enable_t *status){
  IS_NULL_ARG(dev);
  IS_NULL_ARG(status);
  uint8_t ret_data = 0;

  //Doc thanh ghi INT_STATUS - Bit dung o thanh ghi nay giong voi thanh ghi INT_ENABLE
  //Neu bit ngat duoc kich hoat o thanh ghi INT_ENABLE, bit o thanh ghi nay cung duoc set thanh 1
  //Sau khi doc thanh ghi nay, cac bit se clear ve 0
  I2C_DEV_TAKE_MUTEX(dev);
  I2C_DEV_CHECK(dev, mpu6050_read_byte_from_reg(dev, MPU6050_INT_STATUS, &ret_data));
  I2C_DEV_GIVE_MUTEX(dev);

  status->data_ready_int = ret_data & (0x01 << 0);
  status->i2c_master_int = ret_data & (0x01 << 3);
  status->fifo_overflow_int = ret_data & (0x01 << 4);

  return ESP_OK;
}

esp_err_t mpu6050_interrupt_config(I2C_dev_init_t *dev, mpu6050_interrupt_config_t *int_cfg){
  IS_NULL_ARG(dev);
  IS_NULL_ARG(int_cfg);
  uint8_t value = 0;

  //Build bitmask dua tren struct config 
  if(int_cfg->disable == true) value = 0x00;
  else{
    if(int_cfg->int_level) value |= (1 << 7);
    if(int_cfg->int_open) value |= (1 << 6);
    if(int_cfg->latch_int_en) value |= (1 << 5);
    if(int_cfg->int_rd_clear) value |= (1 << 4);
    if(int_cfg->fsync_int_level) value |= (1 << 3);
    if(int_cfg->fsync_int_en) value |= (1 << 2);
    if(int_cfg->i2c_bypass_en) value |= (1 << 1);
  }

  //Ghi vao thanh ghi cau hinh ngat INT_PIN_CFG
  I2C_DEV_TAKE_MUTEX(dev);
  I2C_DEV_CHECK(dev, mpu6050_write_byte_to_reg(dev, MPU6050_INT_PIN_CFG, value));
  I2C_DEV_GIVE_MUTEX(dev);

  return ESP_OK;
}

esp_err_t mpu6050_fifo_config(I2C_dev_init_t *dev, bool temp, bool accel, bool gyro){
  IS_NULL_ARG(dev);
  uint8_t value = 0;

  if(temp == true) value |= MPU6050_FIFO_TEMP; //Set 1 to Bit7 TEMP_OUT
  if(accel == true) value |= MPU6050_FIFO_ACCEL; //Set 1 to Bit3
  if(gyro == true) value |= (MPU6050_FIFO_XG | MPU6050_FIFO_YG | MPU6050_FIFO_ZG); //Set 1 to Bit6-Bit5-Bit4
  //if(gyro == true) value |= (0x07 << 4);
  
  I2C_DEV_TAKE_MUTEX(dev);
  I2C_DEV_CHECK(dev, mpu6050_write_byte_to_reg(dev, MPU6050_FIFO_CONFIG, value));
  I2C_DEV_GIVE_MUTEX(dev);

  esp_err_t ret = mpu6050_clear_fifo(dev, true); //Clear FIFO
  if(ret != ESP_OK) return ret;
  
  return ESP_OK;
}

esp_err_t mpu6050_fifo_enable(I2C_dev_init_t *dev, bool enable){
  IS_NULL_ARG(dev);
  uint8_t value = 0;

  //Doc thanh ghi USER_CTRL de kiem tra xem Bit6 FIFO_EN co dang bat khong
  I2C_DEV_TAKE_MUTEX(dev);
  I2C_DEV_CHECK(dev, mpu6050_read_byte_from_reg(dev, MPU6050_USER_CTRL,&value));
  I2C_DEV_GIVE_MUTEX(dev);
  
  if(enable == true){ //Neu lenh la bat
    value |= MPU6050_USER_CTRL_FIFO_EN_BIT; //Bat Bit6 len
  }else{
    value &= ~MPU6050_USER_CTRL_FIFO_EN_BIT; //Clear Bit6
  }

  //Ghi lai vao thanh ghi USER_CTRL
  I2C_DEV_TAKE_MUTEX(dev);
  I2C_DEV_CHECK(dev, mpu6050_write_byte_to_reg(dev, MPU6050_USER_CTRL, value));
  I2C_DEV_GIVE_MUTEX(dev);

  return ESP_OK;
}

esp_err_t mpu6050_is_fifo_overflow(I2C_dev_init_t *dev, bool *is_overflow){
  IS_NULL_ARG(dev);
  IS_NULL_ARG(is_overflow);
  uint8_t int_status = 0;

  //Doc trang thai bit4 hien tai o thanh ghi INT_STATUS
  I2C_DEV_TAKE_MUTEX(dev);
  I2C_DEV_CHECK(dev, mpu6050_read_byte_from_reg(dev, MPU6050_INT_STATUS, &int_status));
  I2C_DEV_GIVE_MUTEX(dev);

  *is_overflow = (int_status & MPU6050_BITMASK_4) ? 1 : 0;

  return ESP_OK;
}

esp_err_t mpu6050_clear_fifo(I2C_dev_init_t *dev, bool restore_prev_state){
  IS_NULL_ARG(dev);
  uint8_t user = 0;

  /* 1. Doc USER_CTRL de biet trang thai cu */
  I2C_DEV_TAKE_MUTEX(dev);
  I2C_DEV_CHECK(dev, mpu6050_read_byte_from_reg(dev, MPU6050_USER_CTRL, &user));
  I2C_DEV_GIVE_MUTEX(dev);

  //Xem Bit6 FIFO_EN cua thanh ghi USER_CTRL co dang bat khong
  uint8_t prev_fifo_en = (user & MPU6050_USER_CTRL_FIFO_EN_BIT) ? 1 : 0;

  if(prev_fifo_en){ //Neu dang bat

    /* 2. Tat FIFO_EN (khong cho ghi) truoc khi reset */
    uint8_t tmp = user & ~MPU6050_USER_CTRL_FIFO_EN_BIT; //Set Bit6 FIFO_EN ve 0

    //Ghi lai vao thanh ghi USER_CTRL
    I2C_DEV_TAKE_MUTEX(dev);
    I2C_DEV_CHECK(dev, mpu6050_write_byte_to_reg(dev, MPU6050_USER_CTRL, tmp));
    I2C_DEV_GIVE_MUTEX(dev);

    /* 3. Bat dau moi Reset/Clear FIFO buffer bang Bit2 FIFO_RESET */
    tmp |= MPU6050_USER_CTRL_FIFO_RESET_BIT; //Set 1 cho Bit2 de RESET
    I2C_DEV_TAKE_MUTEX(dev);
    I2C_DEV_CHECK(dev, mpu6050_write_byte_to_reg(dev, MPU6050_USER_CTRL, tmp));
    I2C_DEV_GIVE_MUTEX(dev);

    /* 4. Delay de reset hoan tat */
    vTaskDelay(pdMS_TO_TICKS(10));

    //Bit2 FIFO_RESET la 1 self-clearing bit -> Sau khi Reset hoan tat, Bit2 FIFO_RESET se tu dong clear ve 0
    if(restore_prev_state && prev_fifo_en){
      tmp |= MPU6050_USER_CTRL_FIFO_EN_BIT; //Bat lai Bit FIFO_EN neu truoc duoc bat
    }else {
      tmp &= ~MPU6050_USER_CTRL_FIFO_EN_BIT; //De tat (disable FIFO buffer)
    }

    I2C_DEV_TAKE_MUTEX(dev);
    I2C_DEV_CHECK(dev, mpu6050_write_byte_to_reg(dev, MPU6050_USER_CTRL, tmp));
    I2C_DEV_GIVE_MUTEX(dev);

  }else{ //Neu dang disable
    ESP_LOGI("MPU6050 FIFO clear", "Can not clear FIFO because FIFO is not enable !");
    return ESP_FAIL;
  }
  return ESP_OK;
}

esp_err_t mpu6050_sensor_config(I2C_dev_init_t *dev, mpu6050_config_t *config){
  IS_NULL_ARG(dev);
  IS_NULL_ARG(config);

  esp_err_t ret;
  
  /* 1. Reset cam bien */
  ret = mpu6050_deviceReset(dev); //Device reset
  if(ret != ESP_OK){
    ESP_LOGE(pcTaskGetName(NULL), "Device Reset failed !\r\n");
    return ret;
  }
  ESP_LOGI(pcTaskGetName(NULL), "Device Reset OK! \r\n");

  ret = mpu6050_softReset(dev, config->reset, config->custom_mask); //Soft reset
  if(ret != ESP_OK){
    ESP_LOGE(pcTaskGetName(NULL), "Soft Reset failed !\r\n");
    return ret;
  }else{
    ESP_LOGI(pcTaskGetName(NULL), "Soft Reset OK !: %s\r\n", mpu6050_soft_reset_to_str(config->reset, config->custom_mask));
  }

  /* 2. Thiet lap cac che do clock & sleep & cycle */
  ret = mpu6050_set_source_clock(dev, config->clock_source); //Clock source
  if(ret != ESP_OK){
    ESP_LOGE(pcTaskGetName(NULL), "Source Clock set failed !\r\n");
    return ret;
  }
  ESP_LOGI(pcTaskGetName(NULL), "Source Clock set OK !: %s \r\n", mpu6050_src_clock_to_str(config->clock_source));

  ret = mpu6050_sleep_mode(dev, config->sleep_mode); //Sleep mode
  if(ret != ESP_OK){
    ESP_LOGE(pcTaskGetName(NULL), "Sleep mode set failed !\r\n");
    return ret;
  }
  ESP_LOGI(pcTaskGetName(NULL), "Sleep mode set OK !: %s\r\n", mpu6050_sleep_mode_to_str(config->sleep_mode));

  ret = mpu6050_low_pwr_cycle_mode(dev, config->cycle_mode); //Cycle mode
  if(ret == ESP_OK){
    if(config->cycle_mode == MPU6050_CYCLE_MODE_ENABLE){ //Neu trang thai la Enable
      ESP_LOGI(pcTaskGetName(NULL), "Cycle mode set OK !: %s | ", mpu6050_low_pwr_cycle_mode_to_str(config->cycle_mode));
      ret = mpu6050_set_wakeups_freq(dev, config->wakeup_freq); //Wake-ups freq
      if(ret != ESP_OK) return ret;
      ESP_LOGI(pcTaskGetName(NULL), "Wake-ups frequency set OK !: %s\r\n", mpu6050_wakeup_freq_to_str(config->wakeup_freq));
    }else{
      ESP_LOGI(pcTaskGetName(NULL), "Cycle mode set OK !: %s\r\n", mpu6050_low_pwr_cycle_mode_to_str(config->cycle_mode));
    }
  }else{
    ESP_LOGE(pcTaskGetName(NULL), "Cycle mode set failed ! \r\n");
    return ret;
  }

  /* 3. Bat/tat cam bien nhiet do */
  ret = mpu6050_enable_temp(dev, config->temperature);
  if(ret != ESP_OK){
    ESP_LOGE(pcTaskGetName(NULL), "Temperature sensor enable/disable set failed ! \r\n");
    return ret;
  }else{
    if(config->temperature == true){
      ESP_LOGI(pcTaskGetName(NULL), "Temperature sensor set OK !: set ON\r\n");
    }else{
      ESP_LOGI(pcTaskGetName(NULL), "Temperature sensor set OK !: set OFF\r\n");
    }
  }

  /* 4. Cau hinh Digital Low Pass Filter */
  ret = mpu6050_config_dlpf(dev, config->dlfp_cfg);
  if(ret != ESP_OK){
    ESP_LOGE(pcTaskGetName(NULL), "Digital Low Pass Filter config failed ! \r\n");
    return ret;
  }else{
    ESP_LOGI(pcTaskGetName(NULL), "Digital Low Pass Filter config OK !: %s\r\n", mpu6050_config_dlpf_to_str(config->dlfp_cfg));
  }
  
  /* 5. Set tan so lay mau (Hz)*/
  ret = mpu6050_set_sampleRate_Hz(dev, config->freq_hz);
  if(ret != ESP_OK){
    ESP_LOGE(pcTaskGetName(NULL), "Sample Rate (Hz) set failed !\r\n");
    return ret;
  }else{
    ESP_LOGI(pcTaskGetName(NULL), "Sample Rate (Hz) set OK ! Sample rate (Hz): %f\r\n", config->freq_hz);
  }

  /* 6. Set dai do Accel & Gyro */
  ret = mpu6050_set_accel_range(dev, config->accel_range, &config->sens);
  if(ret != ESP_OK){
    ESP_LOGE(pcTaskGetName(NULL), "Accel Range & Sens set failed !\r\n");
    return ret;
  }else{
    ESP_LOGI(pcTaskGetName(NULL), "Accel Range & Sens set OK !: %s\r\n", mpu6050_accel_range_to_str(config->accel_range));
  }

  ret = mpu6050_set_gyro_range(dev, config->gyro_range, &config->sens);
  if(ret != ESP_OK){
    ESP_LOGE(pcTaskGetName(NULL), "Gyro Range & Sens set failed ! \r\n");
    return ret;
  }else {
    ESP_LOGI(pcTaskGetName(NULL), "Gyro Range & Sens set OK !: %s\r\n", mpu6050_gyro_range_to_str(config->gyro_range));
  }

  /* 7. Set che do Standby cho Accel va Gyro */
  ret = mpu6050_standby_mode(dev, &config->standby_mode);
  if(ret != ESP_OK){
    ESP_LOGE(pcTaskGetName(NULL), "Standby mode set failed !\r\n");
    return ret;
  }else{
    ESP_LOGI(pcTaskGetName(NULL), "Standby mode set OK !: %s\r\n", mpu6050_standby_mode_to_str(config->standby_mode));
  }

  /* 8. Cau hinh ngat */
  ret = mpu6050_interrupt_config(dev, &config->int_cfg); //Cau hinh tin hieu ngat
  if(ret != ESP_OK){
    ESP_LOGE(pcTaskGetName(NULL), "Interrupt config failed !\r\n");
    return ret;
  }else{
    ESP_LOGI(pcTaskGetName(NULL), "Interrupt config OK !: %s\r\n", mpu6050_interrupt_config_to_str(&config->int_cfg));
    
    ret = mpu6050_interrupt_enable(dev, &config->interrupts); //Set thanh phan ngat
    if(ret != ESP_OK){
      ESP_LOGE(pcTaskGetName(NULL), "Interrupt enable/disable failed !\r\n");
      return ret;
    }else{
      if(config->interrupts.disable_int == true){ //Neu khong kich hoat interrupt
        ESP_LOGI(pcTaskGetName(NULL), "Interrupt disabled set OK !\r\n");
      }else{
        ESP_LOGI(pcTaskGetName(NULL), "Interrupt enabled set OK !: %s\r\n", mpu6050_interrupt_enable_to_str(&config->interrupts));
      }
    }
  }

  /* 9. Cau hinh FIFO */
  ret = mpu6050_fifo_enable(dev, config->fifo_enable); //Bat FIFO buffer
  if(ret != ESP_OK){
    ESP_LOGI(pcTaskGetName(NULL), "FIFO enable failed !\r\n");
    return ret;
  }
  if(config->fifo_enable == true){
    ESP_LOGI(pcTaskGetName(NULL), "FIFO enable set OK !\r\n");
  }else{
    ESP_LOGI(pcTaskGetName(NULL), "FIFO disable set OK !\r\n");
  }

  ret = mpu6050_fifo_config(dev, config->temp_en, config->accel_en, config->gyro_en); //Cau hinh FIFO
  if(ret != ESP_OK){
    ESP_LOGI(pcTaskGetName(NULL), "FIFO config set failed !\r\n");
    return ret;
  }else{
    ESP_LOGI(pcTaskGetName(NULL), "FIFO config set OK !: %s\r\n", mpu6050_fifo_config_to_str(config));
  }

  ESP_LOGI(pcTaskGetName(NULL), "All sensor config set OK !\r\n");
  return ESP_OK;
}

/**************************************************************************************************/

size_t mpu6050_get_sample_size(I2C_dev_init_t *dev){
  IS_NULL_ARG(dev);
  size_t size = 0;
  uint8_t value = 0;

  //Doc trang thai thanh ghi FIFO_CONFIG 0x23 hien tai
  I2C_DEV_TAKE_MUTEX(dev);
  I2C_DEV_CHECK(dev, mpu6050_read_byte_from_reg(dev, MPU6050_FIFO_CONFIG, &value));
  I2C_DEV_GIVE_MUTEX(dev);

  //Thuc hien so sanh bit de xem gia tri nao dang duoc enable
  if(value & MPU6050_FIFO_TEMP) size += 2; //Neu chi co temp -> 2 bytes
  if(value & MPU6050_FIFO_XG) size += 2;
  if(value & MPU6050_FIFO_YG) size += 2;
  if(value & MPU6050_FIFO_ZG) size += 2;
  if(value & MPU6050_FIFO_ACCEL) size += 6; //Neu chi co Accel -> 6 bytes

  return size;
}

esp_err_t mpu6050_fifo_read_raw(I2C_dev_init_t *dev, mpu6050_raw_data_t *raw_data, size_t samples){
  IS_NULL_ARG(dev);

  uint8_t fifo_mask = 0;

  //Doc gia tri fifo mask hien tai trong thanh ghi FIFO_CONFIG
  I2C_DEV_TAKE_MUTEX(dev);
  I2C_DEV_CHECK(dev, mpu6050_read_byte_from_reg(dev, MPU6050_FIFO_CONFIG, &fifo_mask));
  I2C_DEV_GIVE_MUTEX(dev);

  //Lay kich thuoc bytes thuc te trong 1 sample sau khi duoc thiet lap de tinh toan kich thuoc tong byte cho n samples
  size_t size = mpu6050_get_sample_size(dev);
  uint16_t needed = size * samples; //So bytes can doc cho n samples (la boi so cua size)

  //Doc du lieu tu thanh ghi FIFO_COUNT_H/L (16-bit)
  uint16_t fifo_count; //Bieu thi so bytes hien dang luu tru trong FIFO
  uint8_t count_buf[2];
  // uint8_t count_l, count_h;

  I2C_DEV_TAKE_MUTEX(dev);
  I2C_DEV_CHECK(dev, mpu6050_read_bytes_from_reg(dev, MPU6050_FIFO_COUNT_H, count_buf, 2)); //Doc lien mach 2 byte tu FIFO_COUNT_H 
  // I2C_DEV_CHECK(dev, mpu6050_read_byte_from_reg(dev, MPU6050_FIFO_COUNT_H, &count_h)); //Doc FIFO cao truoc
  // I2C_DEV_CHECK(dev, mpu6050_read_byte_from_reg(dev, MPU6050_FIFO_COUNT_L, &count_l)); //Doc FIFO thap sau
  I2C_DEV_GIVE_MUTEX(dev);

  //Ghep lai thanh gia tri 16-bits bieu thi tong so bytes dang co trong FIFO
  fifo_count = ((uint16_t)count_buf[0] << 8) | count_buf[1];
  // fifo_count = ((uint16_t)count_h << 8) | count_l; 

  if(fifo_count < needed){ //Kiem tra xem FIFO co du so byte de doc chua
    return ESP_FAIL; //Chua du du lieu de doc
  }

  uint8_t *buf = malloc(needed); //Mang chua tong so byte cua cac samples
  if(buf == NULL) return ESP_ERR_NO_MEM;

  //Doc du lieu FIFO xuat ra tu thanh ghi FIFO_R_W 0x74 (Cua truy xuat du lieu FIFO)
  esp_err_t ret = mpu6050_read_bytes_from_reg(dev, MPU6050_FIFO_R_W, buf, needed);
  if(ret != ESP_OK){
    free(buf);
    return ret;
  }

  //Parse tung sample
  for(size_t i = 0; i < samples; i++){
    size_t base = i * size; //Buoc nhay (boi so cua size) de nhay sang sample tiep theo
    size_t offset = 0;

    //Theo thu tu FIFO: TEMP -> GYRO_X -> GYRO_Y -> GYRO_Z -> ACCEL 
    if(fifo_mask & MPU6050_FIFO_TEMP){
      raw_data[i].temp = (int16_t)((buf[base + offset] << 8) | buf[base + offset + 1]); //Byte cao -> Byte thap
      offset += 2;
    }
    
    if(fifo_mask & MPU6050_FIFO_XG){
      raw_data[i].gyro_x = (int16_t)((buf[base + offset] << 8) | buf[base + offset + 1]);//Byte cao -> Byte thap
      offset += 2;
    }

    if(fifo_mask & MPU6050_FIFO_YG){
      raw_data[i].gyro_y = (int16_t)((buf[base + offset] << 8) | buf[base + offset + 1]);//Byte cao -> Byte thap
      offset += 2;
    }

    if(fifo_mask & MPU6050_FIFO_ZG){
      raw_data[i].gyro_z = (int16_t)((buf[base + offset] << 8) | buf[base + offset + 1]); //Byte cao -> Byte thap
      offset += 2;
    }

    if(fifo_mask & MPU6050_FIFO_ACCEL){
      raw_data[i].accel_x = (int16_t)((buf[base + offset]) << 8 | buf[base + offset + 1]); //Byte cao -> Byte thap
      raw_data[i].accel_y = (int16_t)((buf[base + offset + 2]) << 8 | buf[base + offset + 3]); //Byte cao -> Byte thap
      raw_data[i].accel_z = (int16_t)((buf[base + offset + 4]) << 8 | buf[base + offset + 5]); //Byte cao -> Byte thap
      offset += 6;
    }
  }

  free(buf);
  return ESP_OK;
}

esp_err_t mpu6050_register_read_raw(I2C_dev_init_t *dev, mpu6050_raw_data_t *raw_data, size_t samples){
  IS_NULL_ARG(dev);

  //Moi lan doc truc tiep duoc 14 bytes (Accel 6 -> Temp 2 -> Gyro 6)
  //Muon doc nhieu sample hon -> phai lap lai "samples" nhieu lan
  for(size_t i = 0; i < samples; i++){
    uint8_t buf[14];

    //Doc lien tuc 14 bytes tu ACCEL_XOUT_H (0x3B)
    I2C_DEV_TAKE_MUTEX(dev);
    I2C_DEV_CHECK(dev, mpu6050_read_bytes_from_reg(dev, MPU6050_ACCEL_XOUT_H, buf, sizeof(buf)));
    I2C_DEV_GIVE_MUTEX(dev);

    raw_data[i].accel_x = (int16_t)((buf[0] << 8) | buf[1]);
    raw_data[i].accel_y = (int16_t)((buf[2] << 8) | buf[3]);
    raw_data[i].accel_z = (int16_t)((buf[4] << 8) | buf[5]);
    
    raw_data[i].temp = (int16_t)((buf[6] << 8) | buf[7]);

    raw_data[i].gyro_x = (int16_t)((buf[8] << 8) | buf[9]);
    raw_data[i].gyro_y = (int16_t)((buf[10] << 8) | buf[11]);
    raw_data[i].gyro_z = (int16_t)((buf[12] << 8) | buf[13]);
  }
  return ESP_OK;
}

esp_err_t mpu6050_read_raw_sample(I2C_dev_init_t *dev, mpu6050_raw_data_t *raw_data, 
                                  mpu6050_read_mode_t read_mode){
  IS_NULL_ARG(dev);
  esp_err_t ret;

  if(read_mode == MPU6050_READ_FIFO){
    ret = mpu6050_fifo_read_raw(dev, raw_data, 1); //Doc 1 sample
    if(ret != ESP_OK) return ret;

  }else if(read_mode == MPU6050_READ_REGISTER){
    ret = mpu6050_register_read_raw(dev, raw_data, 1); //Doc 1 sample
    if(ret != ESP_OK) return ret;
  }
  return ESP_OK;
}

esp_err_t mpu6050_read_raw_multi_samples(I2C_dev_init_t *dev, mpu6050_raw_data_t *raw_data, 
                                        mpu6050_read_mode_t read_mode, size_t samples){
  IS_NULL_ARG(dev);
  esp_err_t ret;

  if(read_mode == MPU6050_READ_FIFO){
    ret = mpu6050_fifo_read_raw(dev, raw_data, samples); //Doc nhieu sample
    if(ret != ESP_OK) return ret;

  }else if(read_mode == MPU6050_READ_REGISTER){
    ret = mpu6050_register_read_raw(dev, raw_data, samples); //Doc nhieu sample
    if(ret != ESP_OK) return ret;
  }
  return ESP_OK;
}

esp_err_t mpu6050_read_scaled_sample(I2C_dev_init_t *dev, mpu6050_scaled_data_t *scaled_data, 
                                    mpu6050_read_mode_t read_mode, mpu6050_sensitivity_t sens){
  IS_NULL_ARG(dev);
  mpu6050_raw_data_t raw_data;

  esp_err_t ret = mpu6050_read_raw_sample(dev, &raw_data, read_mode); //Tan dung ham `read_raw_sample`
  if(ret != ESP_OK) return ret;

  //Chuyen ve dang scaled
  scaled_data->accel_x_g = (float)raw_data.accel_x / sens.accel_lsb_per_g;
  scaled_data->accel_y_g = (float)raw_data.accel_y / sens.accel_lsb_per_g;
  scaled_data->accel_z_g = (float)raw_data.accel_z / sens.accel_lsb_per_g;

  //TEMP = (TEMP_OUT / 340 (he so scale factor)) + 36.53 (offset)
  scaled_data->temp_c = (float)((raw_data.temp / 340.0f) + 36.53f); 

  scaled_data->gyro_x_dps = (float)raw_data.gyro_x / sens.gyro_lsb_per_dps;
  scaled_data->gyro_y_dps = (float)raw_data.gyro_y / sens.gyro_lsb_per_dps;
  scaled_data->gyro_z_dps = (float)raw_data.gyro_z / sens.gyro_lsb_per_dps;

  return ESP_OK;
}


esp_err_t mpu6050_read_scaled_multi_samples(I2C_dev_init_t *dev, mpu6050_scaled_data_t *scaled_data, 
                                          mpu6050_read_mode_t read_mode, mpu6050_sensitivity_t sens, 
                                          size_t samples){
  IS_NULL_ARG(dev);  
  
  esp_err_t ret;
  mpu6050_raw_data_t raw_data[samples];

  ret = mpu6050_read_raw_multi_samples(dev, raw_data, read_mode, samples);
  if(ret != ESP_OK) return ret;

  //Chuyen ve dang scaled
  for(size_t i = 0; i < samples; i++){
    scaled_data[i].accel_x_g = (float)raw_data[i].accel_x / sens.accel_lsb_per_g;
    scaled_data[i].accel_y_g = (float)raw_data[i].accel_y / sens.accel_lsb_per_g;
    scaled_data[i].accel_z_g = (float)raw_data[i].accel_z / sens.accel_lsb_per_g;

    //TEMP = (TEMP_OUT / 340 (he so scale factor)) + 36.53 (offset)
    scaled_data[i].temp_c = (float)((raw_data[i].temp / 340.0f) + 36.53f); 

    scaled_data[i].gyro_x_dps = (float)raw_data[i].gyro_x / sens.gyro_lsb_per_dps;
    scaled_data[i].gyro_y_dps = (float)raw_data[i].gyro_y / sens.gyro_lsb_per_dps;
    scaled_data[i].gyro_z_dps = (float)raw_data[i].gyro_z / sens.gyro_lsb_per_dps;
  }
  return ESP_OK;
}


#ifdef __cplusplus
}
#endif 