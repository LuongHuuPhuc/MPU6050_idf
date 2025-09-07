/** @file mpu6050.h
 * @date 2025/08/29
 * @author Luong Huu Phuc
 * @brief MPU6050 driver (n time write sensor driver ! try my best !)
 */

#ifndef __INC_MPU6050_H_
#define __INC_MPU6050_h

#ifdef __cplusplus
extern "C" {
#endif //__cplusplus

#pragma once 

#include "stdio.h"
#include "stdint-gcc.h"
#include "stdbool.h"
#include "esp_log.h"
#include "esp_err.h"
#include "I2C_dev.h"
#include "mpu6050_ll.h"

/***************** DEFINE ****************/

// ===== GENERAL CONFIG REGISTER =====

#define MPU6050_CONFIG          0x1A //(Write-Read) Dung de dong bo du lieu voi tin hieu ngoai (FSYNC) va loc tin hieu gyro va accel (DLPF) truoc khi xuat ra (thanh ghi quyet dinh Gyro Output Rate + Filter)
//Gom 2 truong EXT_SYNC_SET[2:0] - dong bo tin hieu va DLPF_CFG[2:0] - quyet dinh tan so goc (base) (Gyro Output Rate)
//Neu DLPF_CFG = 0 hoac 7 (OFF) -> Gyro Output Rate = 8kHz (Fs_base)
//Neu DLPF_CFG = 1 den 6 (ON) -> Gyro Output Rate = 1kHz (Fs_base)
//Accel Output Rate luon mac dinh la 1kHz (Fs_base) doi voi cac trang thai DLPF_CFG

#define MPU6050_WHO_AM_I_REG    0x75 //(Read-only) Thanh ghi de xac minh danh tinh (verify identity) cua cam bien [6:1] - Bit cuoi quyet dinh boi AD0 pin -> Tra ve gia tri 7-bit I2C address
#define MPU6050_DEFAULT_ADRR_0  0x68 //(Read-only) Dia chi I2C mac dinh neu AD0 = 0 (6-bit dau, khong co bit cuoi)
#define MPU6050_DEFAULT_ADDR_1  0x69 //(Read-only) Dia chi I2C mac dinh neu AD0 = 1 (6-bit dau, khong co bit cuoi)
#define MPU6050_SMPRT_DIV       0x19 //(Write-Read) Dung de tao tan so lay mau cuoi cung tu thanh ghi 0x1A (8-bit) - `Sample Rate = Gyro Output Rate / (SMPRT_DIV + 1)`
//Trong do Gyro Output Rate phu thuoc vao cau hinh DLPF-CFG o thanh ghi MPU6050_CONFIG (0x1A) de sinh ra Sample Rate cuoi cung


#define MPU6050_GYRO_CONFIG     0x1B //(Write-Read) Dung de trigger self-test (tu kiem tra) va cau hinh dai do gyro
//Self-test: XG_ST, YG_ST, ZG_ST - bit7, bit6, bit5: Dung de kiem tra xem gyro con hoat dong dung khong 
//Bit[4:3]: FG_SEL[1:0]: Chon dai do (scale) cua gyro -> Dai cang lon, do duoc toc do quay nhanh hon nhung do phan giai (LSB/s) giam
//Bit[2:0]: Khong dung
//Scale: 0 - 250dps, 1 - 500dps, 2 - 1000dps, 3 - 2000dps

#define MPU6050_ACCEL_CONFIG    0x1C //(Write-Read) Dung trigger self-test va cau hinh dai do cua accel. Thanh ghi nay cung dung de cau hih DHPF (Digital High Pass Filter)
//Self-test: XA_ST, YA_ST, ZA_ST - bit7, bit6, bit5: Dung de kiem tra xem accel con hoat dong dung khong 
//Bit[4:3]: AFS_SEL[1:0]: Chon dai do (scale) cua accel -> Dai cang lon, do duoc toc do quay nhanh hon nhung do phan giai (LSB/s) giam
//Bit[2:0]: Khong dung
//Scale: 0 - 2g, 1 - 4g, 2 - 8g, 3 - 16g

// ===== SELF TEST REGISTER =====

#define MPU6050_SELF_TEST_X     0x0D
#define MPU6050_SELF_TEST_Y     0x0E 
#define MPU6050_SELF_TEST_Z     0x0F
#define MPU6050_SELF_TEST_A     0x10


// ===== FIFO REGISTER =====

#define MPU6050_FIFO_R_W        0x74 //(Write-Read) Thanh ghi dung de doc-ghi data tu FIFO buffer [7:0] - Cua so truy xuat du lieu FIFO ra ngoai (moi lan chi duoc 1 byte)
//Neu muon doc 1 frame gom nhieu byte thi phai doc nhieu lan
//12 byte cho Accel + Gyro (Moi truc 6 byte), 14 byte cho Accel + Gyro + Temp 

#define MPU6050_FIFO_COUNT_H    0x72 //(Read-only) Thanh ghi de doc so byte hien co trong FIFO buffer (Cao) [15:8] - Doc thanh ghi nay truoc vi byte se update o day (cao->thap)
#define MPU6050_FIFO_COUNT_L    0x73 //(Read-only) Thanh ghi de doc so byte hien co trong FIFO buffer (thap) [7:0] - doc so byte tu FIFO_COUNT_H truoc
//Kich thuoc FIFO_COUNT = FIFO_COUNT_H + FIFO_COUNT_L la 2 bytes (16-bit) - Kich thuoc FIFO toi da 1024 bytes (datasheet)

#define MPU6050_FIFO_CONFIG     0x23 //(Write-Read) Thanh ghi quyet dinh gia tri nao cua cam bien se duoc day vao FIFO buffer (8-bit)
//Bit7: TEMP_FIFO_EN - Khi set = 1, cho phep ghi TEMP_OUT_H va TEMP_OUT_L vao FIFO buffer -> Tong 2 bytes cho 1 sample data temp
//Bit6: XG_FIFO_EN - Khi set = 1, cho phep ghi GYRO_XOUT_H va GYRO_XOUT_L vao FIFO buffer -> Tong 2 bytes cho 1 sample data x
//Bit5: YG_FIFO_EN - Khi set = 1, cho phep ghi GYRO_YOUT_H va GYRO_YOUT_L vao FIFO buffer -> Tong 2 bytes cho 1 sample data y
//Bit4: ZG_FIFO_EN - Khi set = 1, cho phep ghi GYRO_ZOUT_H va GYRO_ZOUT_L vao FIFO buffer -> Tong 2 bytes cho 1 sample data z
//Bit3: ACCEL_FIFO_EN - Khi set = 1, cho phep ghi cac gia cua ACCEL vao FIFO buffer -> Tong 2 bytes cho 1 sample data -> Ca x, y, z la 6 bytes
//Bit2: SLV2_FIFO_EN
//Bit1: SLV1_FIFO_EN
//Bit0: SLV0_FIFO_EN


// ===== INTERRUPT REGISTER =====

#define MPU6050_INT_STATUS      0x3A //(Read-Only) Vi tri cac bi giong voi INT_ENABLE - Dung de doc trang thai ngat moi khi co bit = 1 (Detect loi)
#define MPU6050_INT_PIN_CFG     0x37 //(Read-Write) Dung de cau hinh INT (cach tin hieu ngat duoc xuat ra)
#define MPU6050_INT_ENABLE      0x38 //(Read-Write) Dung de bat/tat ngat 
//Bit4: Ngat khi FIFO bi tran
//Bit3: Ngat khi co su kien tu I2C Master (cam bien ben ngoai)
//Bit0: Ngat khi data moi san sang (accel, gyro, temp)


// ===== I2C MASTER INTERFACE REGISTER FOR EXTERNAL SLAVE =====

#define MPU6050_I2C_MST_CTRL        0x24
#define MPU6050_I2C_MST_STATUS      0x36
#define MPU6050_I2C_MST_DELAY_CTRL  0x67

#define MPU6050_I2C_SLV0_ADDR     0x25
#define MPU6050_I2C_SLV0_REG      0x26
#define MPU6050_I2C_SLV0_CTRL     0x27
#define MPU6050_I2C_SLV0_DO       0x63

#define MPU6050_I2C_SLV1_ADDR     0x28
#define MPU6050_I2C_SLV1_REG      0x29
#define MPU6050_I2C_SLV1_CTRL     0x2A
#define MPU6050_I2C_SLV1_DO       0x64

#define MPU6050_I2C_SLV2_ADDR     0x2B
#define MPU6050_I2C_SLV2_REG      0x2C
#define MPU6050_I2C_SLV2_CTRL     0x2D
#define MPU6050_I2C_SLV2_DO       0x65

#define MPU6050_I2C_SLV3_ADDR     0x2E
#define MPU6050_I2C_SLV3_REG      0x2F
#define MPU6050_I2C_SLV3_CTRL     0x30
#define MPU6050_I2C_SLV3_DO       0x66

#define MPU6050_I2C_SLV4_ADDR     0x31
#define MPU6050_I2C_SLV4_REG      0x32
#define MPU6050_I2C_SLV4_DO       0x33
#define MPU6050_I2C_SLV4_CTRL     0x34
#define MPU6050_I2C_SLV4_DI       0x35

// ===== ACCEL & GYRO & TEMP REGISTER =====
//Data cua moi gia tri Accel, Gyro, Temp se la 2 bytes bao gom byte cao + byte thap

#define MPU6050_ACCEL_XOUT_H        0x3B //(Read-Only)[15:8] byte X cao
#define MPU6050_ACCEL_XOUT_L        0x3C //(Read_Only)[7:0] byte X thap 
#define MPU6050_ACCEL_YOUT_H        0x3D //(Read-Only)[15:8] byte Y cao
#define MPU6050_ACCEL_YOUT_L        0x3E //(Read-Only)[7:0] byte Y thap
#define MPU6050_ACCEL_ZOUT_H        0x3F //(Read-Only)[15:8] byte Z cao
#define MPU6050_ACCEL_ZOUT_L        0x40 //(Read-Only)[7:0] byte Z thap
//Dung de luu cac gia tri Accel - Du lieu cac thanh ghi nay duoc cap nhat o Sample Rate (thanh ghi 0x19)

#define MPU6050_GYRO_XOUT_H         0x43
#define MPU6050_GYRO_XOUT_L         0x44
#define MPU6050_GYRO_YOUT_H         0x45
#define MPU6050_GYRO_YOUT_L         0x46
#define MPU6050_GYRO_ZOUT_H         0x47
#define MPU6050_GYRO_ZOUT_L         0x48
//Dung de luu cac gia tri Gyro - Du lieu cac thanh ghi nay duoc cap nhat o Sample Rate (thanh ghi 0x19)

#define MPU6050_TEMP_OUT_H          0x41 //(Read-Only)Thanh ghi luu byte cao gia tri nhiet do cam bien [15:8]
#define MPU6050_TEMP_OUT_L          0x42 //(Read-Only)Thanh ghi luu byte thap gia tri nhiet do cam bien [7:0]
//Ket hop 2 byte -> tao thanh gia tri 16-bit co dau goi la TEMP_OUT
//Du lieu 2 thanh ghi nay duoc cap nhat o Sample Rate (thanh ghi 0x19 MPU6050_SMPRT_DIV)
//Cong thuc: TEMP = (TEMP_OUT / 340 (he so scale factor)) + 36.53 (offset)

// ===== EXTERNAL SENSOR DATA =====

#define MPU6050_EXT_SENS_DATA_00    0x49
#define MPU6050_EXT_SENS_DATA_01    0x4A
#define MPU6050_EXT_SENS_DATA_02    0x4B
#define MPU6050_EXT_SENS_DATA_03    0x4C
#define MPU6050_EXT_SENS_DATA_04    0x4D
#define MPU6050_EXT_SENS_DATA_05    0x4E
#define MPU6050_EXT_SENS_DATA_06    0x4F
#define MPU6050_EXT_SENS_DATA_07    0x50
#define MPU6050_EXT_SENS_DATA_08    0x51
#define MPU6050_EXT_SENS_DATA_09    0x52
#define MPU6050_EXT_SENS_DATA_10    0x53
#define MPU6050_EXT_SENS_DATA_11    0x54
#define MPU6050_EXT_SENS_DATA_12    0x55
#define MPU6050_EXT_SENS_DATA_13    0x56
#define MPU6050_EXT_SENS_DATA_14    0x57
#define MPU6050_EXT_SENS_DATA_15    0x58
#define MPU6050_EXT_SENS_DATA_16    0x59
#define MPU6050_EXT_SENS_DATA_17    0x5A
#define MPU6050_EXT_SENS_DATA_18    0x5B
#define MPU6050_EXT_SENS_DATA_19    0x5C
#define MPU6050_EXT_SENS_DATA_20    0x5D
#define MPU6050_EXT_SENS_DATA_21    0x5E
#define MPU6050_EXT_SENS_DATA_22    0x5F
#define MPU6050_EXT_SENS_DATA_23    0x60
//(Read-Only)[7:0] - Day la cac thanh ghi de ghi du lieu tu cac cam bien ngoai ket noi qua I2C phu (I2C Auxiliary Interface) cua MPU6050
//Ben trong MPU6050 co 1 I2C Master phu cho phep ket noi toi da 4 thiet bi slave phu (thuong dung them cac cam bien tu truong de tao thanh cam bien 9 truc)
//Du lieu doc tu cac cam bien do se duoc luu tam vao cac thanh ghi EXT_SENS_DATA_xx, sau do MCU chi can doc lai cac thanh ghi nay qua I2C -> MPU6050

// ===== BITMASK =====

#define MPU6050_BITMASK_7        (0x01 << 7) //0x80
#define MPU6050_BITMASK_6        (0x01 << 6) //0x40
#define MPU6050_BITMASK_5        (0x01 << 5) //0x20
#define MPU6050_BITMASK_4        (0x01 << 4) //0x10
#define MPU6050_BITMASK_3        (0x01 << 3) //0x08
#define MPU6050_BITMASK_2        (0x01 << 2) //0x04
#define MPU6050_BITMASK_1        (0x01 << 1) //0x02
#define MPU6050_BITMASK_0        (0x01 << 0) //0x01

// ===== MISCELLANEOUS REGISTER =====

#define MPU6050_DEVICE_RESET       (MPU6050_BITMASK_7) //Reset toan bo cam bien (Bit7 (1 << 7) - DEVICE_RESET)
#define MPU6050_SIGNAL_PATH_RESET  0x68 //(Read-Write) Thanh ghi de reset(xoa, lam moi) cac khoi ananlog/digital cua cam bien
//No la thanh ghi tai dia chi 0x68h trong khong gian thanh ghi ben trong MPU6050 (dung nham voi dia chi I2C 0x68)
//Bit[7:3] - Khong dung 
//Bit2 - GYRO_RESET, Bit1 - ACCEL_RESET, Bit0 - TEMP_RESET

#define MPU6050_USER_CTRL_FIFO_EN_BIT        MPU6050_BITMASK_6 //Bitmask Bit6 trong USER_CTRL
#define MPU6050_USER_CTRL_FIFO_RESET_BIT     MPU6050_BITMASK_2 //Bit2 trong USER_CTRL
#define MPU6050_USER_CTRL          0x6A //(Write-Read) Thanh ghi de bat/tat FIFO & I2C_MASTER
//Bit6: FIFO_EN - Bat/tat FIFO buffer
//Bit5: I2C_MST_EN
//Bit4: I2C_IF_DIS
//Bit2: FIFO_RESET - Reset FIFO buffer 
//Bit1: I2C_MST_RESET 
//Bit0: SIG_COND_RESET

#define MPU6050_PWR_MGMT_1         0x6B //(Write-Read) Thanh ghi de cau hinh clk_src va power_mode, ngoai ra con de reset chip & tat/bat cam bien nhiet do
//Bit7: DEVICE_RESET - Ghi 1 de reset toan bo device ve mac dinh -> bit nay sau do se tu clear ve 0 (dung cho soft reset chip)
//Bit6: SLEEP - Ghi 1 de dua MPU6050 vao che do sleep (cuc thap dien nang), 0 de wake-up  
//Bit5:　CYCLE - Ghi 1 cho phep Cycle mode - chip se tu dong lap giua Sleep va Wake theo tan suat duoc cau hinh trong bit LP_WAKE_CTRL(o thanh ghi PWR_MGMT_2 0x6C)
//Bit4: Khong dung
//Bit3: TEMP_DIS - Ghi 1 de tat cam bien nhiet do, 0 - enable
//Bit[2:0] CLKSEL (Clock Source Select) - Dung de cap nguon clock cho cam bien

#define MPU6050_PWR_MGMT_2         0x6C //(Write-Read) Thanh ghi bo sung cho PWR_MGMT_1 0x6B
//Cho phep chon tan so wake-up trong che do Accelerometer Only Low PowerMode 
//Cho phep disable (standby) tung truc cua Accel va Gyro rieng biet

#define MPU6050_FIFO_TEMP         (MPU6050_BITMASK_7) //Bit7
#define MPU6050_FIFO_XG           (MPU6050_BITMASK_6) //Bit6
#define MPU6050_FIFO_YG           (MPU6050_BITMASK_5) //Bit5
#define MPU6050_FIFO_ZG           (MPU6050_BITMASK_4) //Bit4
#define MPU6050_FIFO_ACCEL        (MPU6050_BITMASK_3) //Bit3
//Thu tu bit trong thanh ghi FIFO_CONFIG 0x23 -> Day cung la thu tu de doc data tu FIFO

/****************** STRUCT & ENUM ****************/

//Source clock (Cau hinh boi PWR_MGMT_1[2:0] Bit2->Bit0 `CLKSEL`)
typedef enum {
  MPU6050_CLK_INTERNAL = 0, //Internal 8MHz oscillator
  MPU6050_CLK_PPL_XGYRO = 1, //PPL with X axis gyro reference 
  MPU6050_CLK_PPL_YGYRO = 2, //PPL with Y
  MPU6050_CLK_PPL_ZGYRO = 3,
  MPU6050_CLK_EXT_32K = 4, //External 32.768 kHz reference
  MPU6050_CLK_EXT_19M = 5, //External 19.2 MHz ref
  MPU6050_CLK_STOP = 7 //Stop clock, sensor sleep
} mpu6050_clock_src_t;

//Sleep mode (PWR_MGMT_1[6])
typedef enum {
  MPU6050_SLEEP_MODE_DISABLE = 0,
  MPU6050_SLEEP_MODE_ENABLE = 1
} mpu6050_sleep_mode_t;

//Cycle mode (PWR_MGMT_1[5])
typedef enum {
  MPU6050_CYCLE_MODE_DISABLE = 0,
  MPU6050_CYCLE_MODE_ENABLE = 1
} mpu6050_low_pwr_cycle_mode_t;

//Tat tung truc cua Accel va Gyro rieng biet (PWR_MGMT_2)
typedef struct {
  bool standby_x_accel;
  bool standby_y_accel;
  bool standby_z_accel;
  bool standby_x_gyro;
  bool standby_y_gyro;
  bool standby_z_gyro;
  bool disable_standby;
} mpu6050_standby_mode_t;

//GYRO full-scale range (dai do gyro) (GYRO_CONFIG[4:3])
typedef enum {
  MPU6050_GYRO_FS_250DPS = 0,
  MPU6050_GYRO_FS_500DPS = 1, 
  MPU6050_GYRO_FS_1000DPS = 2,
  MPU6050_GYRO_FS_2000DPS = 3
} mpu6050_gyro_range_t;

//ACCEL full-scale range (dai do accel) (ACCEL_CONFIG[4:3])
typedef enum {
  MPU6050_ACCEL_FS_2G = 0,
  MPU6050_ACCEL_FS_4G = 1,
  MPU6050_ACCEL_FS_8G = 2,
  MPU6050_ACCEL_FS_16G = 3
} mpu6050_accel_range_t;

// Digital Low Pass Filter (Thanh ghi CONFIG[2:0] 0x1A)
typedef enum {
  MPU6050_DLPF_260HZ = 0, 
  MPU6050_DLPF_184HZ = 1, 
  MPU6050_DLPF_94HZ = 2, 
  MPU6050_DLPF_44HZ = 3, 
  MPU6050_DLPF_21HZ = 4, 
  MPU6050_DLPF_10HZ = 5, 
  MPU6050_DLPF_5HZ = 6, 
  MPU6050_DLPF_OFF = 7
} mpu6050_dlpf_t;

// WAKEUP FREQUENCY (PWR_MGMT_2 0x6C Bit5)
typedef enum {
  MPU6050_WAKEUP_FREQ_1_25HZ = 0,
  MPU6050_WAKEUP_FREQ_5HZ = 1,
  MPU6050_WAKEUP_FREQ_20HZ = 2,
  MPU6050_WAKEUP_FREQ_40HZ = 3
} mpu6050_wakeup_freq_t;

// SOFT RESET SELECT (SIGNAL PATH RESET)
typedef enum {
  MPU6050_RESET_ALL = 0,
  MPU6050_RESET_ACCEL = 1,
  MPU6050_RESET_GYRO = 2,
  MPU6050_RESET_TEMP = 3,
  MPU6050_RESET_CUSTOM = 4 //Tu custom so duong tin hieu reset
} mpu6050_soft_reset_t;

//Interrrupt enable (INT_ENABLE) - Chon kieu ngat (INT_ENABLE 0x38)
typedef struct {
  bool data_ready_int; //Data ready interrupt
  bool i2c_master_int; //Motion detect
  bool fifo_overflow_int; //FIFO overflow (tran FIFO)
  bool disable_int;
} mpu6050_interrupt_enable_t;

//Interrupt configuration - Cau hinh tin hieu ngat (INT_PIN_CFG 0x37)
typedef struct {
  bool disable;
  bool int_level; //0 = active high, 1 = active low
  bool int_open; //0 = push-pull, 1 = open-drain
  bool latch_int_en; //0 = 50us pulse, 1 = giu cho den khi clear (Cach giu tin hieu ngat)
  bool int_rd_clear; //0 = clear khi doc INT_STATUS, 1 = clear khi doc bat ky thanh ghi nao (Cach clear co ngat)
  bool fsync_int_level; //0 = active high, 1 = active low
  bool fsync_int_en; //enable ngat FSYNC
  bool i2c_bypass_en; //enable bypass mode cho I2C aux
} mpu6050_interrupt_config_t;

//CHE DO DOC DATA
typedef enum {
  MPU6050_READ_FIFO, //Doc truc tiep tu FIFO
  MPU6050_READ_REGISTER //Doc truc tiep tu thanh ghi
} mpu6050_read_mode_t;

typedef struct {
  float accel_lsb_per_g;
  float gyro_lsb_per_dps;
} mpu6050_sensitivity_t;

//DATA STRUCT
typedef struct {
  int16_t accel_x;
  int16_t accel_y;
  int16_t accel_z;
  int16_t gyro_x;
  int16_t gyro_y;
  int16_t gyro_z;
  int16_t temp;
} mpu6050_raw_data_t;

typedef struct {
  float accel_x_g;
  float accel_y_g;
  float accel_z_g;
  float gyro_x_dps;
  float gyro_y_dps;
  float gyro_z_dps;
  float temp_c;
} mpu6050_scaled_data_t;

// MAIN CONFIG STRUCT
typedef struct {
  mpu6050_clock_src_t clock_source;
  mpu6050_sleep_mode_t sleep_mode;
  mpu6050_low_pwr_cycle_mode_t cycle_mode;
  mpu6050_standby_mode_t standby_mode;
  mpu6050_accel_range_t accel_range;
  mpu6050_gyro_range_t gyro_range;
  mpu6050_wakeup_freq_t wakeup_freq;
  mpu6050_dlpf_t dlfp_cfg;
  mpu6050_interrupt_enable_t interrupts;
  mpu6050_interrupt_config_t int_cfg;
  mpu6050_soft_reset_t reset;
  mpu6050_read_mode_t readMode;;
  mpu6050_sensitivity_t sens;
  mpu6050_raw_data_t raw_data;
  mpu6050_scaled_data_t scaled_data;
  
  uint8_t custom_mask; //Soft Reset custom mask
  bool temperature; //Bat/tat cam bien nhiet do
  bool temp_en, accel_en, gyro_en; //FIFO enable flags
  float freq_hz; //Sample rate
  bool fifo_enable; //FIFO enable flags
} mpu6050_config_t;

/*********************** FUNCTION ********************/

// ====================== KHOI TAO & CAU HINH ======================

/**
 * @brief Ham cau hinh I2C cho cam bien MPU6050 
 * 
 * @param dev Doi tuong I2C
 * @param port Cong dieu khien I2C
 * @param sda_pin Chan SDA
 * @param scl_pin Chan SCL
 * 
 * @note Su dung API cua I2C_dev.h
 */
esp_err_t mpu6050_I2C_dev_config(I2C_dev_init_t *dev, i2c_port_t port, uint8_t sda_pin, uint8_t scl_pin);

/**
 * @brief Ham cau hinh I2C cho cam bien MPU6050 
 * 
 * @param dev Doi tuong I2C
 * @param port Cong dieu khien I2C
 * @param sda_pin Chan SDA
 * @param scl_pin Chan SCL
 * 
 * @note Su dung API cua driver/I2C.h
 */
esp_err_t __attribute__((unused))mpu6050_I2C_driver_config(i2c_port_t port, uint8_t sda_pin, uint8_t scl_pin);

/**
 * @brief Ham doc thanh ghi WHO_AM_I de kiem tra ID thiet bi
 * @note Su dung thanh ghi WHO_AM_I 0x75
 * 
 * @param dev Doi tuong I2C
 * @return Sucess = ESP_OK
 */
esp_err_t mpu6050_verify_whoami(I2C_dev_init_t *dev);

/**
 * @brief Ham clear thanh ghi
 * @note Ghi gia tri 0x00 vao thanh ghi de clear (ngoai tru PWR_MGMT_1 va WHO_AM_I khong the clear duoc voi 0x00)
 * vi chung co gia tri mac dinh la PWR_MGMT_1 = 0x40 va WHO_AM_I = 0x68 (datasheet)
 * 
 * @param reg Thanh ghi can clear
 * @param dev Doi tuong I2C
 * @return Success = ESP_OK
 */
esp_err_t mpu6050_clear_reg(I2C_dev_init_t *dev, uint8_t reg);

/**
 * @brief Ham reset cac khoi du lieu cam bien (lam moi)
 * @note Su dung thanh ghi SIGNAL_PATH_RESET 0x68
 * 
 * @param dev Doi tuong I2C
 * @param reset_type Kieu reset
 * @param custom_mask Custom kieu reset bang bit mask (|) (Neu khong dung thi dien NULL hoac 0)
 * 
 * @return Success = ESP_OK
 */
esp_err_t mpu6050_softReset(I2C_dev_init_t *dev, mpu6050_soft_reset_t reset_type, uint8_t custom_mask);

/**
 * @brief Ham reset toan bo cam bien 
 * @note Ghi vao bit7 DEVICE_RESET(0x80) cua thanh ghi PWR_MGMT_1 0x6B
 * \note - Bit nay se tuong dong clear va 0 sau khi reset xong
 * 
 * @param dev Doi tuong I2C
 * @return Success = ESP_OK
 */
esp_err_t mpu6050_deviceReset(I2C_dev_init_t *dev);

/**
 * @brief Ham set che do Sleep Mode (tiet kiem nang luong)
 * @note Sau khi reset cam bien, MPU6050 thuong o che do `sleep`
 * \note - Ta can thay doi Bit6 `SLEEP` trong `PWR_MGMT_1 0x6B` de kich hoat Sleep Mode
 * \note - O che do nay, Accel, Gyro, Temo deu disable, chi co I2C van hoat dong de danh thuc cam bien
 * 
 * @param dev Doi tuong I2C
 * @param sleep Bat/tat sleep mode
 * @return Success = ESP_OK
 */
esp_err_t mpu6050_sleep_mode(I2C_dev_init_t *dev, mpu6050_sleep_mode_t sleep);

/**
 * @brief Ham set che do Low Power Cycle Mode 
 * @note Su dung thanh ghi PWR_MGMT_1 0x6B tai Bit5 CYCLE
 * \note - Khi Bit `CYCLE` set la 1 va Bit6 `SLEEP` disable, se kich hoat duoc che do nay
 * \note - Trong mode nay Gyro se tat, chi co Accel hoat dong dinh ky -> Lay mau 1 lan duy nhat, roi quay ve sleep
 * \note - Tan so wake-ups trong mode nay duoc cau hinh tai Bit[7:6] `LP_WAKE_CTRL` trong thanh ghi `PWR_MGMT_2` 0x6C
 * 
 * @param dev Doi tuong I2C
 * @param cycle_mode Bat/tat che do cycle mode
 * @return Success = ESP_OK
 */
esp_err_t mpu6050_low_pwr_cycle_mode(I2C_dev_init_t *dev, mpu6050_low_pwr_cycle_mode_t cycle_mode);

/**
 * @brief Ham cau hinh tan so wake-ups trong Low Power Cycle Mode
 * @note Su dung thanh ghi PWR_MGMT_2 tai Bit[7:6] `LP_WAKE_CTRL` de dieu khien
 * 
 * @param dev Doi tuong I2C
 * @param wakeup_freq Tan so mong muon
 * @return Success = ESP_OK
 */
esp_err_t mpu6050_set_wakeups_freq(I2C_dev_init_t *dev, mpu6050_wakeup_freq_t wakeup_freq);

/**
 * @brief Ham bat/tat che do do nhiet do ben trong
 * @note Su dung thanh ghi PWR_MGMT_1 o Bit3 TEMP_DIS
 * 
 * @param dev Doi tuong I2C
 * @param enable Mo hoac tat
 * @return Success = ESP_OK
 */
esp_err_t mpu6050_enable_temp(I2C_dev_init_t *dev, bool enable);

/**
 * @brief Ham cau hinh thang do cho Accel
 * @note Su dung thanh ghi MPU6050_ACCEL_CONFIG 0x1C
 * 
 * @param dev Doi tuong I2C
 * @param range Dai do truyen vao
 * @param sens Do nhay cua accel tuong ung voi `range` (bao nhieu LSB/g) - Gia tri duoc tinh trong ham va xuat ra con tro `config` chung
 * @return Success = ESP_OK
 */
esp_err_t mpu6050_set_accel_range(I2C_dev_init_t *dev, mpu6050_accel_range_t range, mpu6050_sensitivity_t *sens);

/**
 * @brief Ham cau hinh thang do cho Gyro
 * @note Su dung thanh ghi MPU6050_GYRO_CONFIG 0x1B
 * 
 * @param dev Doi tuong I2C
 * @param range Dai do truyen vao
 * @param sens Do nhay cua gyro tuong ung voi `range` (bao nhieu LSB/1°/s) - Gia tri duoc tinh trong ham va xuat ra con tro `config` chung
 * @return Success = ESP_OK
 */
esp_err_t mpu6050_set_gyro_range(I2C_dev_init_t *dev, mpu6050_gyro_range_t range, mpu6050_sensitivity_t *sens);

/**
 * @brief Ham set che do Standby Mode cho cac truc Accel/Gyro trong cam bien (neu khong muon dung)
 * @note Su dung thanh ghi PWR_MGMT_2 0x6C tu bit[5:0] de thao tac voi 3 truc x, y, z
 * \note - Standby mode nay la viec tat cac truc data -> tac dong vao phan cung -> Khong lay data, khong cap nhat
 * \note - Dung trong truong hop muon tiet kiem nang luong
 * 
 * @param dev Doi tuong I2C
 * @param standby_mode Con tro toi cac truc ban can disable
 * @return Success = ESP_OK
 */
esp_err_t mpu6050_standby_mode(I2C_dev_init_t *dev, mpu6050_standby_mode_t *standby_mode);

/**
 * @brief Ham cau hinh bang thong bo loc Digital Low Pass Filter cho cam bien
 * @note Su dung thanh ghi CONFIG[2:0] 0x1A de thay doi Bit2->Bit0 DLPF_CFG[2:0]
 * 
 * @param dev Doi tuong I2C
 * @param bandwidth Bang tan can loc
 * @return Success = ESP_OK
 */
esp_err_t mpu6050_config_dlpf(I2C_dev_init_t *dev, mpu6050_dlpf_t bandwidth);

/**
 * @brief Set so lan chia nho tan so goc (Fs_base) - duoc lay tu DLFP_CFG
 * @note Su dung thanh ghi SMPRT_DIV 0x19 
 * @note Vi la 1 thanh ghi 8-bit dung hoan toan cho divisor nen chi can ghi thang gia tri chia vao la xong,
 * khong can clear bit
 * 
 * @param dev Doi tuong I2C
 * @param smprt_div So lan chia (gioi han tu 0 - 255 lan)
 * @return Success = ESP_OK
 */
esp_err_t mpu6050_set_sampleRate_div(I2C_dev_init_t *dev, uint8_t smprt_div);

/**
 * @brief Ham doc gia tri raw cua Sample Rate Divider (sau khi chia tan)
 * @note Doc truc tiep tu thanh ghi SMPRT_DIV 0x19
 * 
 * @param dev Doi tuong I2C
 * @return Gia tri raw SMPRT_DIV (0 - 255)
 */
uint8_t mpu6050_get_sampleRate_div(I2C_dev_init_t *dev);

/**
 * @brief Ham set tan so lay mau (Hz)
 * @note Su dung thanh ghi CONFIG 0x1A -> doc du lieu DLPF_CFG de lay `Fs_base`
 * \note - Tu tan so mong muon (Hz) truyen vao -> Tinh ra so lan chia (smprt_div) roi chuyen vao thanh ghi SMPRT_DIV 0x19
 * \note - Tan so nay se la tan so lay mau chung cho ca Gyro va Accel 
 * \note - Dua tren cong thuc: `Fs_actual = Fs_base / (1 + SMPRT_DIV)`
 * \note - Voi Fs_base duoc tinh dua tren Bit2->Bit0 DLPF_CFG[2:0] tai thanh ghi CONFIG 0x1A
 * 
 * @param dev Doi tuong I2C
 * @param freq_hz Tan so mong muon 
 * @return Success = ESP_OK;
 */
esp_err_t mpu6050_set_sampleRate_Hz(I2C_dev_init_t *dev, float freq_hz);

/**
 * @brief Ham lay ve gia tri tan so lay mau thuc te (Hz)
 * @note Dua tren cong thuc: `Fs_actual = Fs_base / (1 + SMPRT_DIV)`
 * 
 * @param dev Doi tuong I2C
 * @return Tan so thuc te (Hz)
 */
float mpu6050_get_sampleRate_Hz(I2C_dev_init_t *dev);

/**
 * @brief Ham cau hinh Source Clock cho cam bien
 * @note Su dung thanh ghi `PWR_MGMT_1` 0x6B tai vi tri Bit2->Bit0 de cau hinh CLKSEL
 * \note - Mac dinh khi khoi dong cam bien dung internal 8MHz oscillator (Khong on dinh bang PLL)
 * \note - Datasheet khuyen nen chon PPL voi gyro (thuong la Z gyro) de co do on dinh tot hon 
 * \note - External 32MHz/19.2MHz dung khi can dong bo voi nguon clock ngoai 
 * 
 * @param dev Doi tuong I2C
 * @param src_clk Cau hinh che do source clock
 * @return Sucess = ESP_OK
 */
esp_err_t mpu6050_set_source_clock(I2C_dev_init_t *dev, mpu6050_clock_src_t src_clk);

/**
 * @brief Ham cau hinh cam bien MPU6050
 * @note Goi ham nay 1 lan de co the cau hinh toan bo cam bien
 * 
 * @param dev Doi tuong I2C
 * @param config Con tro toi struct chua toan bo cau hinh cam bien
 * @return Success = ESP_OK
 */
esp_err_t mpu6050_sensor_config(I2C_dev_init_t *dev, mpu6050_config_t *config);

// ==================== INTERRUPT ==================

/**
 * @brief Bat/tat ngat cam bien MPU6050
 * @note Su dung thanh ghi INT_ENABLE 0x38
 * 
 * @param dev Doi tuong I2C
 * @param enable Con tro cau hinh trang thai ngat theo Bitmask
 * @return Success = ESP_OK
 */
esp_err_t mpu6050_interrupt_enable(I2C_dev_init_t *dev, mpu6050_interrupt_enable_t *enable);

/**
 * @brief Doc trang thai ngat
 * @note Su dung thanh ghi INT_STATUS 0x3A
 * 
 * @param dev Doi tuong I2C
 * @param status Con tro de luu gia tri tra ve khi doc duoc tu thanh ghi trang thai
 * @return Success = ESP_OK
 */
esp_err_t mpu6050_interrupt_get_status(I2C_dev_init_t *dev, mpu6050_interrupt_enable_t *status);

/**
 * @brief Cau hinh che do ngat (cach xuat tin hieu ngat ra ngoai)
 * @note Su dung thanh ghi INT_PIN_CFG 0x37 
 * 
 * @param dev Doi tuong I2C
 * @param int_cfg Con tro den struc cau hinh ngat
 * @return Success = ESP_OK
 */
esp_err_t mpu6050_interrupt_config(I2C_dev_init_t *dev, mpu6050_interrupt_config_t *int_cfg);

// ==================== FIFO ==================

/**
 * @brief Ham cau hinh du lieu cho FIFO buffer
 * @note Su dung thanh ghi FIFO_CONFIG 0x23 de quyet dinh loai du lieu Accel, Gyro hay Temp duoc ghi vao
 * \note - Neu khong doc gia tri nao thi disable di de tiet kiem dung luong FIFO va de parse hon
 * @param dev Doi tuong I2C
 * @param temp True - nhan du lieu temp, False - khong nhan du lieu temp
 * @param accel True - nhan du lieu accel, False - Khong nhan du lieu accel
 * @param gyro True - nhan du lieu Gyro, False - khong nhan du lieu Gyro
 * @return Success = ESP_OK
 */
esp_err_t mpu6050_fifo_config(I2C_dev_init_t *dev, bool temp, bool accel, bool gyro);

/**
 * @brief Ham bat/tat FIFO buffer
 * @note Su dung thanh ghi USER_CTRL 0x6A tai Bit6 FIFO_EN de bat/tat FIFO
 * @param dev Doi tuong I2C
 * @param enable True = bat, False = tat
 * @return Success = ESP_OK
 */
esp_err_t mpu6050_fifo_enable(I2C_dev_init_t *dev, bool enable);

/**
 * @brief Ham kiem tra xem FIFO co bi overflow khong
 * @note Su dung thanh ghi INT_STATUS 0x3A de kiem tra tai Bit4 FIFO_OFLOW_INT
 * \note Bit nay se tu dong clear sau khi duoc doc 
 * 
 * @param dev Doi tuong I2C
 * @param is_overflow Con tro tra ve gia tri doc duoc tu thanh ghi, true = overflow, false = no overflow
 * @return Success = ESP_OK
 */
esp_err_t mpu6050_is_fifo_overflow(I2C_dev_init_t *dev, bool *is_overflow);

/**
 * @brief Ham clear/reset FIFO
 * @note Su dung thanh USER_CTRL 0x6A o Bit2 FIFO_RESET de reset FIFO buffer
 * \note - Tat Bit6 FIFO_EN = 0 (ngan ghi trong luc reset) -> Sau khi ghi FIFO_RESET = 1 -> Doi 5-10ms -> Clear FIFO_RESET
 * \note - Khi reset FIFO_COUNT_H/L se ve 0
 * \note - De dung duoc ham nay can enable FIFO truoc, neu khong se bao loi
 * 
 * @param dev Doi tuong I2C
 * @param restore_prev_state Neu true, khoi phuc Bit6 FIFO_EN ve gia tri cu sau khi reset.
 *                           Neu false, de Bit6 FIFO_EN = 0 (disable) (tat ghi vao FIFO buffer)
 * @return Success = ESP_OK
 */
esp_err_t mpu6050_clear_fifo(I2C_dev_init_t *dev, bool restore_prev_state);

// ====================== CHUC NANG ======================

/**
 * @brief Ham tra ve kich thuoc bytes can doc duoc tu FIFO buffer trong 1 packet
 * @note Su dung thanh ghi FIFO_CONFIG 0x23 de xem Accel, Gyro hay Temp dang duoc enable roi tinh ra kich thuoc bytes cua 1 packet can doc
 * \note - De doc duoc ham nay, can phai thiet lap gia tri can doc bang ham `mpu6050_fifo_config()` truoc
 * @param dev Doi tuong I2C
 * @return Kich thuoc (bytes) trong 1 packet
 */
size_t mpu6050_get_sample_size(I2C_dev_init_t *dev);

/**
 * @brief Ham doc du lieu raw tu FIFO
 * @note Su dung thanh ghi FIFO_R_W 0x74 de doc 8-bit gia tri
 * \note - Truoc tien doc dung luong trong FIFO tu FIFO_COUNT_H 0x72 (8-bit) va FIFO_COUNT_L 0x73 (8-bit)
 * \note - Sau do ghep chung lai thanh 1 so 16-bit
 * \note - Gia tri doc duoc la cac byte lien tiep nhau theo 1 packet 1 sample day du = 14 bytes, thu tu Temp -> Gyro -> Accel
 * \note - Toi uu cho tan so lay mau cao (>1kHz), neu MCU ban thi van duoc giu lai trong FIFO buffer -> Khong mat mau
 * 
 * @param dev Doi tuong I2C
 * @param raw_data Con tro den struct chua cac bien luu data raw
 * @param samples So sample muon doc
 * @return Success = ESP_OK
 */
esp_err_t mpu6050_fifo_read_raw(I2C_dev_init_t *dev, mpu6050_raw_data_t *raw_data, size_t sample);

/**
 * @brief Ham doc du lieu raw tu Register 
 * @note Su dung cac thanh ghi tu 59 (ACCEL_XOUT_H (0x3B))-> 72 (GYRO_ZOUT_L (0x48))
 * \note - Du lieu luon duoc cap nhat theo Sample Rate (thanh ghi SMPRT_DIV)
 * \note - Khac voi FIFO, thu tu doc o Register la Accel -> Temp -> Gyro
 * \note - FIFO config cho phep quyet dinh data nao duoc push vao nhung data o Register van duoc cap nhat
 * \note - Thao tac de hon so voi FIFO, phu hop voi tan so lay mau thap (50-200Hz)
 * \note - Neu MCU polling khong du nhanh de doc 1 sample dung chu ky - De mat mau
 * 
 * @param dev Doi tuong I2C
 * @param raw_data Con tro toi struct luu cac gia tri raw
 * @param samples So sample muon doc
 * 
 * @warning Do thanh ghi cap nhat cac gia tri moi theo sample rate (tuy config 1kHz hay 100Hz) trong khi code 
 * chay trong vong `for` nhanh hon rat nhieu nen se co rui ro in ra cac gia tri giong nhau lien tiep hoac thay 
 * doi rat it -> Vi thanh ghi chua kip update mau moi
 */
esp_err_t mpu6050_register_read_raw(I2C_dev_init_t *dev, mpu6050_raw_data_t *raw_data, size_t samples);

/**
 * @brief Ham doc 1 sample raw Accel/Gyro/Temp
 * @note Co 2 che do doc tu FIFO hoac tu Data Register
 * 
 * @param dev Doi tuong FIFO
 * @param raw_data Con tro den cac bien trong struct de luu du lieu raw
 * @param read_mode Switch 1 trong 2 che do de doc: FIFO hoac truc tiep tu Register
 * @return Success = ESP_OK
 */
esp_err_t mpu6050_read_raw_sample(I2C_dev_init_t *dev, mpu6050_raw_data_t *raw_data, 
                                  mpu6050_read_mode_t read_mode);

/**
 * @brief Ham doc 1 sample Accel/Gyro/Temp da duoc scaled
 * @note Co 2 che do doc tu FIFO hoac tu Data Register
 * 
 * @param dev Doi tuong FIFO
 * @param scaled_data Con tro den cac bien trong struct de luu du lieu da duoc scaled
 * @param read_mode Switch 1 trong 2 che do de doc: FIFO hoac truc tiep tu Register
 * @param sens Do nhay cua tung gia tri Accel/Gyro duoc set trong `range` de chuyen doi
 */
esp_err_t mpu6050_read_scaled_sample(I2C_dev_init_t *dev, mpu6050_scaled_data_t *scaled_data, 
                                    mpu6050_read_mode_t read_mode, mpu6050_sensitivity_t sens);

/**
 * @brief Ham doc nhieu sample raw Accel/Gyro/Temp
 * @note Co 2 che do doc tu FIFO hoac tu Data Register
 * \note - Doc nhieu sample 1 luc se giup giam overhead do I2C so voi viec doc tung sample
 * 
 * @param dev Doi tuong FIFO
 * @param raw_data Con tro den cac bien trong struct de luu du lieu raw
 * @param read_mode Switch 1 trong 2 che do de doc: FIFO hoac truc tiep tu Register
 * @param samples So sample mong muon doc 1 lan
 */
esp_err_t mpu6050_read_raw_multi_samples(I2C_dev_init_t *dev, mpu6050_raw_data_t *raw_data, 
                                        mpu6050_read_mode_t read_mode, size_t samples);

/**
 * @brief Ham doc nhieu sample Accel/Gyro/Temp da duoc scaled
 * @note Co 2 che do doc tu FIFO hoac tu Data Register
 * \note - Doc nhieu sample 1 luc se giup giam overhead do I2C so voi viec doc tung sample
 * 
 * @param dev Doi tuong FIFO
 * @param scaled_data Con tro den cac bien trong struct de luu du lieu da duoc scaled
 * @param read_mode Switch 1 trong 2 che do de doc: FIFO hoac truc tiep tu Register
 * @param sens Do nhay cua tung gia tri Accel/Gyro duoc set trong `range` de chuyen doi
 * @param samples So sample mong muon doc 1 lan
 */
esp_err_t mpu6050_read_scaled_multi_samples(I2C_dev_init_t *dev, mpu6050_scaled_data_t *scaled_data, 
                                          mpu6050_read_mode_t read_mode, mpu6050_sensitivity_t sens, 
                                          size_t samples);

/**
 * @brief 
 */
esp_err_t __attribute__((unused))mpu6050_selfTestGyro(I2C_dev_init_t *dev);

/**
 * @brief 
 */
esp_err_t __attribute__((unused))mpu6050_selfTestAccel(I2C_dev_init_t *dev);


#define IS_NULL_ARG(ARG) do{ \
  if((ARG) == NULL) { \
    return ESP_ERR_INVALID_ARG; \
  } \
} while(0)


#ifdef __cplusplus
}
#endif //__cplusplus

#endif //__INC_MPU6050_H