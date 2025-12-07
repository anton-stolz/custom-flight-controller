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
*
*
*
* translated by anton stolz from arduino cpp to stm32 c
*/
#ifndef MPU6500_H
#define MPU6500_H

#include "i2c.h"
#include <stdint.h>
#include <stdbool.h>

#define MPU6500_I2C_ADDR (0x68)
#define MPU6500_I2C_TIMEOUT 100

typedef enum {
  ACCEL_RANGE_2G = 0x00,
  ACCEL_RANGE_4G = 0x08,
  ACCEL_RANGE_8G = 0x10,
  ACCEL_RANGE_16G = 0x18
} AccelRange;

typedef enum {
  GYRO_RANGE_250DPS = 0x00,
  GYRO_RANGE_500DPS = 0x08,
  GYRO_RANGE_1000DPS = 0x10,
  GYRO_RANGE_2000DPS = 0x18
} GyroRange;

typedef enum {
  DLPF_BANDWIDTH_184HZ = 0x01,
  DLPF_BANDWIDTH_92HZ = 0x02,
  DLPF_BANDWIDTH_41HZ = 0x03,
  DLPF_BANDWIDTH_20HZ = 0x04,
  DLPF_BANDWIDTH_10HZ = 0x05,
  DLPF_BANDWIDTH_5HZ = 0x06
} DlpfBandwidth;

typedef struct {
  I2C_HandleTypeDef* hi2c;
  AccelRange accel_range;
  float accel_scale;
  GyroRange gyro_range;
  float gyro_scale;
  DlpfBandwidth dlpf_bandwidth;
  uint8_t srd;
  uint8_t who_am_i;
  bool new_imu_data;

  int16_t accel_cnts[3];
  int16_t gyro_cnts[3];
  int16_t temp_cnts;

  float accel[3];
  float gyro[3];
  float temp;
  uint8_t data_buf[15];
} Mpu6500;

bool Mpu6500_Init(Mpu6500* dev, I2C_HandleTypeDef* hi2c);
bool Mpu6500_EnableDrdyInt(Mpu6500* dev);
bool Mpu6500_DisableDrdyInt(Mpu6500* dev);
bool Mpu6500_ConfigAccelRange(Mpu6500* dev, AccelRange range);
bool Mpu6500_ConfigGyroRange(Mpu6500* dev, GyroRange range);
bool Mpu6500_ConfigDlpfBandwidth(Mpu6500* dev, DlpfBandwidth dlpf);
bool Mpu6500_ConfigSrd(Mpu6500* dev, uint8_t srd);
bool Mpu6500_Read(Mpu6500* dev);

#endif
