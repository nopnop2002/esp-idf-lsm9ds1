/*
  This file is part of the Arduino_LSM9DS1 library.
  Copyright (c) 2019 Arduino SA. All rights reserved.

  This library is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License as published by the Free Software Foundation; either
  version 2.1 of the License, or (at your option) any later version.

  This library is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
  Lesser General Public License for more details.

  You should have received a copy of the GNU Lesser General Public
  License along with this library; if not, write to the Free Software
  Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301  USA
*/

#include <cstring>
#include "esp_rom_sys.h"
#include "esp_log.h"
#include "driver/i2c_master.h"
#include "LSM9DS1.h"

#define I2C_NUM I2C_NUM_0
#define I2C_TICKS_TO_WAIT 100 // Maximum ticks to wait before issuing a timeout.

static i2c_master_bus_handle_t bus_handle;
static i2c_master_dev_handle_t dev_handle;
static i2c_master_dev_handle_t mag_handle;

// Arduino compatible macros
#define delay(ms) esp_rom_delay_us(ms*1000)

#define LSM9DS1_ADDRESS            0x6b

#define LSM9DS1_WHO_AM_I           0x0f
#define LSM9DS1_CTRL_REG1_G        0x10
#define LSM9DS1_STATUS_REG         0x17
#define LSM9DS1_OUT_X_G            0x18
#define LSM9DS1_CTRL_REG6_XL       0x20
#define LSM9DS1_CTRL_REG8          0x22
#define LSM9DS1_OUT_X_XL           0x28

// magnetometer
#define LSM9DS1_ADDRESS_M          0x1e

#define LSM9DS1_CTRL_REG1_M        0x20
#define LSM9DS1_CTRL_REG2_M        0x21
#define LSM9DS1_CTRL_REG3_M        0x22
#define LSM9DS1_STATUS_REG_M       0x27
#define LSM9DS1_OUT_X_L_M          0x28

LSM9DS1Class::LSM9DS1Class()
{
  continuousMode = false;
  devAddr = LSM9DS1_ADDRESS;
  magAddr = LSM9DS1_ADDRESS_M;
}

LSM9DS1Class::LSM9DS1Class(uint16_t devAddress, uint16_t magAddress)
{
  continuousMode = false;
  devAddr = devAddress;
  magAddr = magAddress;
}

LSM9DS1Class::~LSM9DS1Class()
{
}

int LSM9DS1Class::begin()
{
  // using I2C for communication
  i2c_master_bus_config_t i2c_mst_config = {};
  i2c_mst_config.clk_source = I2C_CLK_SRC_DEFAULT;
  i2c_mst_config.glitch_ignore_cnt = 7;
  i2c_mst_config.i2c_port = I2C_NUM;
  i2c_mst_config.scl_io_num = (gpio_num_t)CONFIG_GPIO_SCL;
  i2c_mst_config.sda_io_num = (gpio_num_t)CONFIG_GPIO_SDA;
  i2c_mst_config.flags.enable_internal_pullup = true;
  ESP_ERROR_CHECK(i2c_new_master_bus(&i2c_mst_config, &bus_handle));

  i2c_device_config_t dev_cfg = {};
  dev_cfg.dev_addr_length = I2C_ADDR_BIT_LEN_7;
  dev_cfg.device_address = devAddr;
  dev_cfg.scl_speed_hz = 400000;
  ESP_ERROR_CHECK(i2c_master_bus_add_device(bus_handle, &dev_cfg, &dev_handle));

  i2c_device_config_t mag_cfg = {};
  mag_cfg.dev_addr_length = I2C_ADDR_BIT_LEN_7;
  mag_cfg.device_address = magAddr;
  mag_cfg.scl_speed_hz = 400000;
  ESP_ERROR_CHECK(i2c_master_bus_add_device(bus_handle, &mag_cfg, &mag_handle));

  // reset
  writeRegister(LSM9DS1_ADDRESS, LSM9DS1_CTRL_REG8, 0x05);
  writeRegister(LSM9DS1_ADDRESS_M, LSM9DS1_CTRL_REG2_M, 0x0c);

  delay(10);

  if (readRegister(LSM9DS1_ADDRESS, LSM9DS1_WHO_AM_I) != 0x68) {
    end();

    return 0;
  }

  if (readRegister(LSM9DS1_ADDRESS_M, LSM9DS1_WHO_AM_I) != 0x3d) {
    end();

    return 0;
  }

  //writeRegister(LSM9DS1_ADDRESS, LSM9DS1_CTRL_REG1_G, 0x78); // 119 Hz, 2000 dps, 16 Hz BW
  writeRegister(LSM9DS1_ADDRESS, LSM9DS1_CTRL_REG1_G, 0x60); // 119 Hz, 245 dps, 16 Hz BW
  writeRegister(LSM9DS1_ADDRESS, LSM9DS1_CTRL_REG6_XL, 0x70); // 119 Hz, 4g

  writeRegister(LSM9DS1_ADDRESS_M, LSM9DS1_CTRL_REG1_M, 0xb4); // Temperature compensation enable, medium performance, 20 Hz
  writeRegister(LSM9DS1_ADDRESS_M, LSM9DS1_CTRL_REG2_M, 0x00); // 4 gauss
  writeRegister(LSM9DS1_ADDRESS_M, LSM9DS1_CTRL_REG3_M, 0x00); // Continuous conversion mode

  return 1;
}

void LSM9DS1Class::setContinuousMode() {
  // Enable FIFO (see docs https://www.st.com/resource/en/datasheet/DM00103319.pdf)
  writeRegister(LSM9DS1_ADDRESS, 0x23, 0x02);
  // Set continuous mode
  writeRegister(LSM9DS1_ADDRESS, 0x2E, 0xC0);

  continuousMode = true;
}

void LSM9DS1Class::setOneShotMode() {
  // Disable FIFO (see docs https://www.st.com/resource/en/datasheet/DM00103319.pdf)
  writeRegister(LSM9DS1_ADDRESS, 0x23, 0x00);
  // Disable continuous mode
  writeRegister(LSM9DS1_ADDRESS, 0x2E, 0x00);

  continuousMode = false;
}

void LSM9DS1Class::end()
{
  writeRegister(LSM9DS1_ADDRESS_M, LSM9DS1_CTRL_REG3_M, 0x03);
  writeRegister(LSM9DS1_ADDRESS, LSM9DS1_CTRL_REG1_G, 0x00);
  writeRegister(LSM9DS1_ADDRESS, LSM9DS1_CTRL_REG6_XL, 0x00);
}

void LSM9DS1Class::getBias(float *gyroBias, float* accelBias) {
  int16_t data[3];
  int32_t sum[6] = {0};
  int count = 0;

  while(1) {
    if (accelerationAvailable()) {
      if (readRegisters(LSM9DS1_ADDRESS, LSM9DS1_OUT_X_XL, (uint8_t*)data, sizeof(data))) {
        sum[3] += data[0]; // accel-x
        sum[4] += data[1]; // accel-z
        int accelZ = data[2] - 8192;
        sum[5] += accelZ; // accel-z
        //printf("accel %d %d %d %d\n", data[0], data[1], data[2], accelZ);
        //printf("sum=%ld %ld %ld\n", sum[3],sum[4],sum[5]);
        count++;
        if (count == 100) break;
        delay(10);
      }
    }
  }

  count = 0;
  while(1) {
    if (gyroscopeAvailable()) {
      if (readRegisters(LSM9DS1_ADDRESS, LSM9DS1_OUT_X_G, (uint8_t*)data, sizeof(data))) {
        sum[0] += data[0]; // gyro-x
        sum[1] += data[1]; // gyro-y
        sum[2] += data[2]; // gyro-z
        //printf("gyrol %d %d %d\n", data[0], data[1], data[2]);
        //printf("sum=%ld %ld %ld\n", sum[0],sum[1],sum[2]);
        count++;
        if (count == 100) break;
        delay(10);
      }
    }
  }

  for (int i = 0; i < 3; ++i) {
    gyroBias[i] = sum[i] / 100.0f;
    accelBias[i] = sum[i+3] / 100.0f;
  }
}

void LSM9DS1Class::setBias(float *gyroBias, float* accelBias) {
  for (int i = 0; i < 3; ++i) {
    _gyroBias[i] = gyroBias[i];
    _accelBias[i] = accelBias[i];
  }
}
int LSM9DS1Class::readAcceleration(float& x, float& y, float& z)
{
  int16_t data[3];

  if (!readRegisters(LSM9DS1_ADDRESS, LSM9DS1_OUT_X_XL, (uint8_t*)data, sizeof(data))) {
    return 0;
  }

#if 0
  x = data[0] * 4.0 / 32768.0;
  y = data[1] * 4.0 / 32768.0;
  z = data[2] * 4.0 / 32768.0;
#endif

  x = (data[0] - _accelBias[0]) / 8192.0;
  y = (data[1] - _accelBias[1]) / 8192.0;
  z = (data[2] - _accelBias[2]) / 8192.0;

  return 1;
}

int LSM9DS1Class::accelerationAvailable()
{
  if (continuousMode) {
    // Read FIFO_SRC. If any of the rightmost 8 bits have a value, there is data.
    if (readRegister(LSM9DS1_ADDRESS, 0x2F) & 63) {
      return 1;
    }
  } else {
    if (readRegister(LSM9DS1_ADDRESS, LSM9DS1_STATUS_REG) & 0x01) {
      return 1;
    }
  }

  return 0;
}

float LSM9DS1Class::accelerationSampleRate()
{
  return 119.0F;
}

int LSM9DS1Class::readGyroscope(float& x, float& y, float& z)
{
  int16_t data[3];

  if (!readRegisters(LSM9DS1_ADDRESS, LSM9DS1_OUT_X_G, (uint8_t*)data, sizeof(data))) {
    return 0;
  }

  // Gyroscope full-scale is 245 dps.
  float gyroScale = 32768.0 / 245.0;

#if 0
  x = data[0] * 2000.0 / 32768.0;
  y = data[1] * 2000.0 / 32768.0;
  z = data[2] * 2000.0 / 32768.0;
#endif

  x = (data[0] - _gyroBias[0]) / gyroScale;
  y = (data[1] - _gyroBias[1]) / gyroScale;
  z = (data[2] - _gyroBias[2]) / gyroScale;
  return 1;
}

int LSM9DS1Class::gyroscopeAvailable()
{
  if (readRegister(LSM9DS1_ADDRESS, LSM9DS1_STATUS_REG) & 0x02) {
    return 1;
  }

  return 0;
}

float LSM9DS1Class::gyroscopeSampleRate()
{
  return 119.0F;
}

int LSM9DS1Class::readMagneticField(float& x, float& y, float& z)
{
  int16_t data[3];

  if (!readRegisters(LSM9DS1_ADDRESS_M, LSM9DS1_OUT_X_L_M, (uint8_t*)data, sizeof(data))) {
    return 0;
  }

#if 0
  x = data[0] * 4.0 * 100.0 / 32768.0;
  y = data[1] * 4.0 * 100.0 / 32768.0;
  z = data[2] * 4.0 * 100.0 / 32768.0;
#endif

  x = (data[0] - _magBias[0]) / 81.92;
  y = (data[1] - _magBias[1]) / 81.92;
  z = (data[2] - _magBias[2]) / 81.92;

  return 1;
}

int LSM9DS1Class::magneticFieldAvailable()
{
  if (readRegister(LSM9DS1_ADDRESS_M, LSM9DS1_STATUS_REG_M) & 0x08) {
    return 1;
  }

  return 0;
}

float LSM9DS1Class::magneticFieldSampleRate()
{
  return 20.0;
}

int LSM9DS1Class::readRegister(uint8_t slaveAddress, uint8_t address)
{
  uint8_t out_buf[1];
  out_buf[0] = address;
  uint8_t in_buf[1];
  if (slaveAddress == LSM9DS1_ADDRESS) {
    ESP_ERROR_CHECK(i2c_master_transmit_receive(dev_handle, out_buf, 1, in_buf, 1, I2C_TICKS_TO_WAIT));
  } else {
    ESP_ERROR_CHECK(i2c_master_transmit_receive(mag_handle, out_buf, 1, in_buf, 1, I2C_TICKS_TO_WAIT));
  }
  return in_buf[0];
}

int LSM9DS1Class::readRegisters(uint8_t slaveAddress, uint8_t address, uint8_t* data, size_t length)
{
  uint8_t out_buf[1];
  out_buf[0] = address;
  if (slaveAddress == LSM9DS1_ADDRESS) {
    ESP_ERROR_CHECK(i2c_master_transmit_receive(dev_handle, out_buf, 1, data, length, I2C_TICKS_TO_WAIT));
  } else {
    ESP_ERROR_CHECK(i2c_master_transmit_receive(mag_handle, out_buf, 1, data, length, I2C_TICKS_TO_WAIT));
  }
  return 1;
}

int LSM9DS1Class::writeRegister(uint8_t slaveAddress, uint8_t address, uint8_t value)
{
  uint8_t out_buf[2];
  out_buf[0] = address;
  out_buf[1] = value;
  if (slaveAddress == LSM9DS1_ADDRESS) {
    ESP_ERROR_CHECK(i2c_master_transmit(dev_handle, out_buf, 2, I2C_TICKS_TO_WAIT));
  } else {
    ESP_ERROR_CHECK(i2c_master_transmit(mag_handle, out_buf, 2, I2C_TICKS_TO_WAIT));
  }
  return 1;
}
