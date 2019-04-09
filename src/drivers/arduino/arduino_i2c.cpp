/****************************************************************************
 *
 *   Copyright (c) 2019 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/**
 * @file arduino_i2c.cpp
 *
 * I2C interface for a custom Arduino data collection solution
 */

/* XXX trim includes */
#include <px4_config.h>
#include <px4_defines.h>

#include <assert.h>
#include <errno.h>
#include <stdbool.h>
#include <stdint.h>
#include <sys/types.h>
#include <unistd.h>

#include <arch/board/board.h>

#include <drivers/device/i2c.h>

#include "board_config.h"

#define ARDUINO_BUS PX4_I2C_BUS_EXPANSION
#define ARDUINO_ADDR0 0x72
#define ARDUINO_ADDR1 0x73

#define ARDUINO_IOCTL_RESET 0x0

#define ARDUINO_BASE_PATH "/dev/arduino"


class Arduino : public device::I2C {
public:
  Arduino(int address = ARDUINO_BASEADDR);
  virtual ~Arduino() = default;

  virtual int init();
  virtual ssize_t read(device::file_t *filp, char *buffer, size_t buflen);
  virtual int ioctl(device::file_t *filp, int cmd, unsigned long arg);

protected:
  virtual int probe();

private:
  orb_advert_t _orb_handle;

}

Arduino::Arduino(int address)
    : I2C("Arduino", ARDUINO_DEVICE_PATH, bus, address, 400000) {
}

int Arduino::init() { return I2C::init(); }

ssize_t read(device::file_t *filp, char *buffer, size_t buflen) {
  float vals[4] = {0.0, 0.0, 0.0, 0.0};

  ret = transfer(nullptr, 0, &val[0], 4*sizeof(float));

  if (ret < 0) {
    DEVICE_DEBUG("error reading from sensor: %d", ret);
    return ret;
  }

  struct sensor_arduino_s report;
  report.timestamp = hrt_absolute_time();
  report.rpm0 = vals[0];
  report.rpm1 = vals[1];
  report.temp0 = vals[2];
  report.temp1 = vals[3];

  if (_orb_handle != nullptr) {
    orb_publish(ORB_ID(sensor_arduino), _orb_handle, &report);
  }

  ret = OK;

  return ret;
}

int ioctl(device::file_t *filep, int cmd, unsigned long arg) {
  int ret = PX4_OK;
  switch (operation) {
  case ARDUINO_IOCTL_RESET:
    // ret = _reset();
    // TODO add reset command to send over the I2C line
    // something like
    // char cmd = ARDUINO_IOCTL_RESET;
    // ret = transfer(&cmd, 1, nullptr, 0)
    break;
  }
}

int probe() {
  int ret = PX4_OK;
  return ret;
}
