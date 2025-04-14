/*
* Brian R Taylor
* brian.taylor@bolderflight.com
*
* Copyright (c) 2022 Bolder Flight Systems Inc
*
* Permission is hereby granted, free of charge, to any person obtaining a copy
* of this software and associated documentation files (the “Software”), to
* deal in the Software without restriction, including without limitation the
* rights to use, copy, modify, merge, publish, distribute, sublicense, and/or
* sell copies of the Software, and to permit persons to whom the Software is
* furnished to do so, subject to the following conditions:
*
* The above copyright notice and this permission notice shall be included in
* all copies or substantial portions of the Software.
*
* THE SOFTWARE IS PROVIDED “AS IS”, WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
* IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
* FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
* AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
* LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
* FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS
* IN THE SOFTWARE.
*
*
* translated by anton stolz from arduino cpp to stm32 c
*/
#include "mpu6500.h"
#include <math.h>
#include <string.h>

#define PWR_MGMNT_1_         0x6B
#define CLKSEL_PLL_          0x01
#define WHOAMI_              0x75
#define WHOAMI_MPU6500_      0x70
#define ACCEL_CONFIG_        0x1C
#define GYRO_CONFIG_         0x1B
#define ACCEL_CONFIG2_       0x1D
#define CONFIG_              0x1A
#define SMPLRT_DIV_          0x19
#define INT_PIN_CFG_         0x37
#define INT_ENABLE_          0x38
#define INT_DISABLE_         0x00
#define INT_PULSE_50US_      0x00
#define INT_RAW_RDY_EN_      0x01
#define INT_STATUS_          0x3A
#define RAW_DATA_RDY_INT_    0x01

#define G_MPS2_              9.80665f
#define DEG2RAD_             (M_PI / 180.0f)
#define TEMP_SCALE_          333.87f

static bool WriteRegister(Mpu6500* dev, uint8_t reg, uint8_t data) {
  return HAL_I2C_Mem_Write(dev->hi2c, MPU6500_I2C_ADDR << 1, reg, I2C_MEMADD_SIZE_8BIT,
                           &data, 1, MPU6500_I2C_TIMEOUT) == HAL_OK;
}

static bool ReadRegisters(Mpu6500* dev, uint8_t reg, uint8_t count, uint8_t* data) {
  return HAL_I2C_Mem_Read(dev->hi2c, MPU6500_I2C_ADDR << 1, reg, I2C_MEMADD_SIZE_8BIT,
                          data, count, MPU6500_I2C_TIMEOUT) == HAL_OK;
}

bool Mpu6500_Init(Mpu6500* dev, I2C_HandleTypeDef* hi2c) {
  dev->hi2c = hi2c;

  if (!WriteRegister(dev, PWR_MGMNT_1_, CLKSEL_PLL_)) return false;

  if (!ReadRegisters(dev, WHOAMI_, 1, &dev->who_am_i)) return false;
  if (dev->who_am_i != WHOAMI_MPU6500_) return false;

  if (!Mpu6500_ConfigAccelRange(dev, ACCEL_RANGE_16G)) return false;
  if (!Mpu6500_ConfigGyroRange(dev, GYRO_RANGE_2000DPS)) return false;
  if (!Mpu6500_ConfigDlpfBandwidth(dev, DLPF_BANDWIDTH_184HZ)) return false;
  if (!Mpu6500_ConfigSrd(dev, 0)) return false;

  return true;
}

bool Mpu6500_EnableDrdyInt(Mpu6500* dev) {
  return WriteRegister(dev, INT_PIN_CFG_, INT_PULSE_50US_) &&
         WriteRegister(dev, INT_ENABLE_, INT_RAW_RDY_EN_);
}

bool Mpu6500_DisableDrdyInt(Mpu6500* dev) {
  return WriteRegister(dev, INT_ENABLE_, INT_DISABLE_);
}

bool Mpu6500_ConfigAccelRange(Mpu6500* dev, AccelRange range) {
  switch (range) {
    case ACCEL_RANGE_2G: dev->accel_scale = 2.0f / 32767.5f; break;
    case ACCEL_RANGE_4G: dev->accel_scale = 4.0f / 32767.5f; break;
    case ACCEL_RANGE_8G: dev->accel_scale = 8.0f / 32767.5f; break;
    case ACCEL_RANGE_16G: dev->accel_scale = 16.0f / 32767.5f; break;
    default: return false;
  }

  if (!WriteRegister(dev, ACCEL_CONFIG_, range)) return false;

  dev->accel_range = range;
  return true;
}

bool Mpu6500_ConfigGyroRange(Mpu6500* dev, GyroRange range) {
  switch (range) {
    case GYRO_RANGE_250DPS: dev->gyro_scale = 250.0f / 32767.5f; break;
    case GYRO_RANGE_500DPS: dev->gyro_scale = 500.0f / 32767.5f; break;
    case GYRO_RANGE_1000DPS: dev->gyro_scale = 1000.0f / 32767.5f; break;
    case GYRO_RANGE_2000DPS: dev->gyro_scale = 2000.0f / 32767.5f; break;
    default: return false;
  }

  if (!WriteRegister(dev, GYRO_CONFIG_, range)) return false;

  dev->gyro_range = range;
  return true;
}

bool Mpu6500_ConfigDlpfBandwidth(Mpu6500* dev, DlpfBandwidth dlpf) {
  if (!WriteRegister(dev, ACCEL_CONFIG2_, dlpf)) return false;
  if (!WriteRegister(dev, CONFIG_, dlpf)) return false;

  dev->dlpf_bandwidth = dlpf;
  return true;
}

bool Mpu6500_ConfigSrd(Mpu6500* dev, uint8_t srd) {
  if (!WriteRegister(dev, SMPLRT_DIV_, srd)) return false;
  dev->srd = srd;
  return true;
}

bool Mpu6500_Read(Mpu6500* dev) {
  dev->new_imu_data = false;

  if (!ReadRegisters(dev, INT_STATUS_, 15, dev->data_buf)) return false;

  dev->new_imu_data = dev->data_buf[0] & RAW_DATA_RDY_INT_;
  if (!dev->new_imu_data) return false;

  dev->accel_cnts[0] = (int16_t)(dev->data_buf[1] << 8 | dev->data_buf[2]);
  dev->accel_cnts[1] = (int16_t)(dev->data_buf[3] << 8 | dev->data_buf[4]);
  dev->accel_cnts[2] = (int16_t)(dev->data_buf[5] << 8 | dev->data_buf[6]);

  dev->temp_cnts = (int16_t)(dev->data_buf[7] << 8 | dev->data_buf[8]);

  dev->gyro_cnts[0] = (int16_t)(dev->data_buf[9] << 8 | dev->data_buf[10]);
  dev->gyro_cnts[1] = (int16_t)(dev->data_buf[11] << 8 | dev->data_buf[12]);
  dev->gyro_cnts[2] = (int16_t)(dev->data_buf[13] << 8 | dev->data_buf[14]);

  dev->accel[0] = dev->accel_cnts[1] * dev->accel_scale * G_MPS2_;
  dev->accel[1] = dev->accel_cnts[0] * dev->accel_scale * G_MPS2_;
  dev->accel[2] = dev->accel_cnts[2] * dev->accel_scale * -1.0f * G_MPS2_;

  dev->gyro[0] = dev->gyro_cnts[1] * dev->gyro_scale * DEG2RAD_;
  dev->gyro[1] = dev->gyro_cnts[0] * dev->gyro_scale * DEG2RAD_;
  dev->gyro[2] = dev->gyro_cnts[2] * dev->gyro_scale * -1.0f * DEG2RAD_;

  dev->temp = ((float)(dev->temp_cnts) - 21.0f) / TEMP_SCALE_ + 21.0f;

  return true;
}
