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

#include <px4_config.h>
#include <px4_defines.h>

#include <stdint.h>

#include <drivers/device/i2c.h>
#include <drivers/drv_hrt.h>

#include <uORB/topics/debug_key_value.h>
#include <uORB/uORB.h>

/* Imports used in other drivers that were unnecessary so far
#include <assert.h>
#include <errno.h>
#include <stdbool.h>
#include <sys/types.h>
#include <unistd.h>

#include <arch/board/board.h>

#include <drivers/drv_orb_dev.h>
#include <drivers/drv_sensor.h>
#include <sys/ioctl.h>

#include <uORB/topics/debug_value.h>
#include <uORB/topics/debug_vect.h>

#include "board_config.h"

// This include was in the drivers that used a ring buffer, but
// it's unclear if this driver needs a ring buffer.
#include <drivers/device/ringbuffer.h>
*/

// Arduino is on the I2C bus
#define ARDUINO_BUS PX4_I2C_BUS_EXPANSION
// Make a couple of valid address for Arduinos to be.  Unclear
// if this is the right way to go about that.
#define ARDUINO_ADDR0 0x72
#define ARDUINO_ADDR1 0x73

// Define IOCTL commands
#define ARDUINO_IOCTL_RESET 0x0
#define ARDUINO_IOCTL_SELF_CHECK 0x1

// Define a device path.  Do there need to be more?
#define ARDUINO_BASE_PATH "/dev/arduino"

/**
 * A driver to collect data from an Arduino connected over
 * I2C
 *
 * This is to allow custom sensor integration like PWM RPM
 * sensors that are trivial to read from an Arduino but
 * hard to read into a Pixhawk
 */
class Arduino : public device::I2C {
public:
  Arduino(int address = ARDUINO_ADDR0);
  virtual ~Arduino() = default;

  virtual int init();
  virtual ssize_t read(char *buffer, size_t buflen);
  virtual int ioctl(int cmd, unsigned long arg);

protected:
  virtual int probe();

private:
  orb_advert_t _orb_handle;

  int _reset();
  int _self_check();
};

extern "C" __EXPORT int arduino_main(int argc, char *argv[]);

/**
 * Constructor for the Arduino driver state object.
 *
 * All this really does is set the variables and call the I2C
 * constructor
 */
Arduino::Arduino(int address)
    : I2C("Arduino", ARDUINO_BASE_PATH, ARDUINO_BUS, address, 400000),
      _orb_handle(nullptr) {}

/**
 * This initializes the driver by advertising on uOrb and calling
 * I2C::init()
 *
 * This shouldn't be doing anything fancy for the time being.
 */
int Arduino::init() {
  if (_orb_handle == nullptr) {
    struct debug_key_value_s report = {};
    _orb_handle = orb_advertise(ORB_ID(debug_key_value), &report);
  }
  if (_orb_handle == nullptr) {
    return PX4_ERROR;
  }
  return I2C::init();
}

/**
 * Read implementation for the Arduino driver
 *
 * This reads a set of measurements from the Arduino
 * and reports these via debug messages.  It will probably
 * need to have some extra logic to prime the Arduino for reading
 * measurements as opposed to health check etc.
 */
ssize_t Arduino::read(char *buffer, size_t buflen) {
  uint8_t raw_vals[8] = {0};
  int ret = transfer(nullptr, 0, &raw_vals[0], 4 * sizeof(float));

  // This is a dirty hack to facilitate transferring floats over
  // an iterface that works on bytes.  The float array is just
  // cast to a byte array and set over the wire where it is
  // then cast back into a float array from a byte array.
  float *vals = (float *)raw_vals;

  if (ret < 0) {
    DEVICE_DEBUG("error reading from sensor: %d", ret);
    return ret;
  }

  uint64_t timestamp_us = hrt_absolute_time();
  uint32_t timestamp_ms = timestamp_us / 1000;

  debug_key_value_s report0 = {
      timestamp_us, // Timestamp in microseconds
      timestamp_ms, // Timestamp in milliseconds
      vals[0],      // Value
      "rpm0",       // Key
  };

  debug_key_value_s report1 = {
      timestamp_us, // Timestamp in microseconds
      timestamp_ms, // Timestamp in milliseconds
      vals[1],      // Value
      "rpm1",       // Key
  };

  debug_key_value_s report2 = {
      timestamp_us, // Timestamp in microseconds
      timestamp_ms, // Timestamp in milliseconds
      vals[2],      // Value
      "temp0",      // Key
  };

  debug_key_value_s report3 = {
      timestamp_us, // Timestamp in microseconds
      timestamp_ms, // Timestamp in milliseconds
      vals[3],      // Value
      "temp1",      // Key
  };

  // report.timestamp = hrt_absolute_time();

  if (_orb_handle != nullptr) {
    orb_publish(ORB_ID(debug_key_value), _orb_handle, &report0);
    orb_publish(ORB_ID(debug_key_value), _orb_handle, &report1);
    orb_publish(ORB_ID(debug_key_value), _orb_handle, &report2);
    orb_publish(ORB_ID(debug_key_value), _orb_handle, &report3);
  }

  ret = PX4_OK;

  return ret;
}

/**
 * IOCTL implementation for the Arduino driver
 *
 * This method mostly just matches on command inputs and
 * uses the helper functions to actually perform any of the
 * IOCTL functions.
 */
int Arduino::ioctl(int cmd, unsigned long arg) {
  int ret = PX4_OK;
  switch (cmd) {
  case ARDUINO_IOCTL_RESET:
    ret = _reset();
    break;
  case ARDUINO_IOCTL_SELF_CHECK:
    ret = _self_check();
    break;
  default:
    break;
  }
  return ret;
}

/**
 * Probe implementation for the Arduino driver (stub)
 *
 * Does a reset followed by a health check to check the
 * initial functionality of the Arduino.
 */
int Arduino::probe() {
  int result1 = ioctl(ARDUINO_IOCTL_RESET, 0);
  if (result1 != PX4_OK) {
    return result1;
  }
  int result2 = ioctl(ARDUINO_IOCTL_SELF_CHECK, 0);
  if (result2 != PX4_OK) {
    return result2;
  }
  return PX4_OK;
}

/**
 * Helper method to send a reset command to the Arduino (stub)
 *
 * This method should send a reset command and if it appears
 * to be successful, return PX4_OK.  If the reset doesn't
 * appear to have worked, return an error code.
 */
int Arduino::_reset() {
  int ret = PX4_OK;
  uint8_t wire_cmd = ARDUINO_IOCTL_RESET;
  int ret2 = transfer(&wire_cmd, 1, nullptr, 0);
  if (ret2 < 0) {
    ret = PX4_ERROR;
  }
  return ret;
}

/**
 * Helper method to do a health check of the Arduino (stub)
 *
 * This method should send a command over the wire instructing
 * the Arduino to send a health report (probably just a uint8_t)
 * back on the next request from the driver.  Then the driver
 * should read that health report and determine the status of the Arduino.
 */
int Arduino::_self_check() {
  int ret = PX4_OK;
  uint8_t wire_cmd = ARDUINO_IOCTL_SELF_CHECK;
  int ret2 = transfer(&wire_cmd, 1, nullptr, 0);
  if (ret2 < 0) {
    ret = PX4_ERROR;
  }
  return ret;
}

/**
 * Main method for the Arduino driver because apparently drivers
 * have main methods.
 */
int arduino_main(int argc, char *argv[]) { return 0; }
