/****************************************************************************
 *
 *   Copyright (c) 2016 PX4 Development Team. All rights reserved.
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
 * @file .h
 *
 * Shared defines for the mpu6000 driver.
 */

#pragma once

#include <px4_config.h>

#include <sys/types.h>
#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdlib.h>
#include <semaphore.h>
#include <string.h>
#include <fcntl.h>
#include <poll.h>
#include <errno.h>
#include <stdio.h>
#include <math.h>
#include <unistd.h>
#include <getopt.h>

#include <systemlib/perf_counter.h>
#include <systemlib/err.h>
#include <systemlib/conversions.h>
#include <systemlib/px4_macros.h>

#include <nuttx/arch.h>
#include <nuttx/wqueue.h>
#include <nuttx/clock.h>

#include <board_config.h>
#include <drivers/drv_hrt.h>

#include <drivers/device/spi.h>
#include <drivers/device/i2c.h>
#include <drivers/device/ringbuffer.h>
#include <drivers/device/integrator.h>
#include <drivers/drv_accel.h>
#include <drivers/drv_gyro.h>
#include <mathlib/math/filter/LowPassFilter2p.hpp>
#include <lib/conversion/rotation.h>

#include "mpu6000_registers.h"

#define MPU_DEVICE_PATH_ACCEL		"/dev/mpu6000_accel"
#define MPU_DEVICE_PATH_GYRO		"/dev/mpu6000_gyro"
#define MPU_DEVICE_PATH_ACCEL1		"/dev/mpu6000_accel1"
#define MPU_DEVICE_PATH_GYRO1		"/dev/mpu6000_gyro1"
#define MPU_DEVICE_PATH_ACCEL_EXT	"/dev/mpu6000_accel_ext"
#define MPU_DEVICE_PATH_GYRO_EXT	"/dev/mpu6000_gyro_ext"
#define MPU_DEVICE_PATH_ACCEL_EXT1	"/dev/mpu6000_accel_ext1"
#define MPU_DEVICE_PATH_GYRO_EXT1	"/dev/mpu6000_gyro_ext1"

#define ICM20602_DEVICE_PATH_ACCEL		"/dev/icm20602_accel"
#define ICM20602_DEVICE_PATH_GYRO		"/dev/icm20602_gyro"
#define ICM20602_DEVICE_PATH_ACCEL1		"/dev/icm20602_accel1"
#define ICM20602_DEVICE_PATH_GYRO1		"/dev/icm20602_gyro1"
#define ICM20602_DEVICE_PATH_ACCEL_EXT	"/dev/icm20602_accel_ext"
#define ICM20602_DEVICE_PATH_GYRO_EXT	"/dev/icm20602_gyro_ext"
#define ICM20602_DEVICE_PATH_ACCEL_EXT1	"/dev/icm20602_accel_ext1"
#define ICM20602_DEVICE_PATH_GYRO_EXT1	"/dev/icm20602_gyro_ext1"

#define ICM20608_DEVICE_PATH_ACCEL		"/dev/icm20608_accel"
#define ICM20608_DEVICE_PATH_GYRO		"/dev/icm20608_gyro"
#define ICM20608_DEVICE_PATH_ACCEL1		"/dev/icm20608_accel1"
#define ICM20608_DEVICE_PATH_GYRO1		"/dev/icm20608_gyro1"
#define ICM20608_DEVICE_PATH_ACCEL_EXT	"/dev/icm20608_accel_ext"
#define ICM20608_DEVICE_PATH_GYRO_EXT	"/dev/icm20608_gyro_ext"
#define ICM20608_DEVICE_PATH_ACCEL_EXT1	"/dev/icm20608_accel_ext1"
#define ICM20608_DEVICE_PATH_GYRO_EXT1	"/dev/icm20608_gyro_ext1"

#define ICM20689_DEVICE_PATH_ACCEL		"/dev/icm20689_accel"
#define ICM20689_DEVICE_PATH_GYRO		"/dev/icm20689_gyro"



#pragma pack(push, 1)
/**
 * Report conversation within the MPU6000, including command byte and
 * interrupt status.
 */
struct MPUReport {
	uint8_t		cmd;
	uint8_t		status;
	uint8_t		accel_x[2];
	uint8_t		accel_y[2];
	uint8_t		accel_z[2];
	uint8_t		temp[2];
	uint8_t		gyro_x[2];
	uint8_t		gyro_y[2];
	uint8_t		gyro_z[2];
};
#pragma pack(pop)



/* interface factories */
extern device::Device *MPU6000_SPI_interface(int bus, int device_type, bool external_bus);
extern device::Device *MPU6000_I2C_interface(int bus, int device_type, bool external_bus);
extern int MPU6000_probe(device::Device *dev, int device_type);

typedef device::Device *(*MPU6000_constructor)(int, int, bool);

enum MPU_DEVICE_TYPE {
    MPU_DEVICE_TYPE_MPU6000    = 6000,
    MPU_DEVICE_TYPE_ICM20602 = 20602,
    MPU_DEVICE_TYPE_ICM20608 = 20608,
    MPU_DEVICE_TYPE_ICM20689 = 20689
};

enum MPU6000_BUS {
    MPU6000_BUS_ALL = 0,
    MPU6000_BUS_I2C_INTERNAL,
    MPU6000_BUS_I2C_EXTERNAL,
    MPU6000_BUS_SPI_INTERNAL1,
    MPU6000_BUS_SPI_INTERNAL2,
    MPU6000_BUS_SPI_EXTERNAL1,
    MPU6000_BUS_SPI_EXTERNAL2
};

class MPU6000_gyro;

class MPU6000 : public device::CDev
{
public:
    MPU6000(device::Device *interface, const char *path_accel, const char *path_gyro, enum Rotation rotation,
            int device_type);
    virtual ~MPU6000();
    
    virtual int        init();
    
    virtual ssize_t        read(struct file *filp, char *buffer, size_t buflen);
    virtual int        ioctl(struct file *filp, int cmd, unsigned long arg);
    
    /**
     * Diagnostics - print some basic information about the driver.
     */
    void            print_info();
    
    void            print_registers();
    
    /**
     * Test behaviour against factory offsets
     *
     * @return 0 on success, 1 on failure
     */
    int             factory_self_test();
    
    // deliberately cause a sensor error
    void             test_error();
    
protected:
    Device            *_interface;
    
    virtual int        probe();
    
    friend class MPU6000_gyro;
    
    virtual ssize_t        gyro_read(struct file *filp, char *buffer, size_t buflen);
    virtual int        gyro_ioctl(struct file *filp, int cmd, unsigned long arg);
    
private:
    int             _device_type;
    MPU6000_gyro        *_gyro;
    uint8_t            _product;    /** product code */
    
#if defined(USE_I2C)
    /*
     * SPI bus based device use hrt
     * I2C bus needs to use work queue
     */
    work_s            _work;
#endif
    bool             _use_hrt;
    
    struct hrt_call        _call;
    unsigned        _call_interval;
    uint8_t         _gyro_downsample_ratio;
    
    ringbuffer::RingBuffer    *_accel_reports;
    
    struct accel_calibration_s    _accel_scale;
    float            _accel_range_scale;
    float            _accel_range_m_s2;
    orb_advert_t        _accel_topic;
    int            _accel_orb_class_instance;
    int            _accel_class_instance;
    
    ringbuffer::RingBuffer    *_gyro_reports;
    ringbuffer::RingBuffer  *_delta_angle_reports;
    
    struct gyro_calibration_s    _gyro_scale;
    float            _gyro_range_scale;
    float            _gyro_range_rad_s;
    
    unsigned        _sample_rate;
    unsigned int    _currentMaxSampleRate;
    
    perf_counter_t        _accel_reads;
    perf_counter_t        _gyro_reads;
    perf_counter_t        _sample_perf;
    perf_counter_t        _bad_transfers;
    perf_counter_t        _bad_registers;
    perf_counter_t        _good_transfers;
    perf_counter_t        _reset_retries;
    perf_counter_t        _duplicates;
    perf_counter_t        _controller_latency_perf;
    
    uint8_t            _register_wait;
    uint64_t        _reset_wait;
    
    math::LowPassFilter2p    _accel_filter_x;
    math::LowPassFilter2p    _accel_filter_y;
    math::LowPassFilter2p    _accel_filter_z;
    math::LowPassFilter2p    _gyro_filter_x;
    math::LowPassFilter2p    _gyro_filter_y;
    math::LowPassFilter2p    _gyro_filter_z;
    
    Integrator        _accel_int;
    Integrator        _gyro_int;
    
    enum Rotation        _rotation;
    
    // this is used to support runtime checking of key
    // configuration registers to detect SPI bus errors and sensor
    // reset
#define MPU6000_CHECKED_PRODUCT_ID_INDEX 0
#define MPU6000_NUM_CHECKED_REGISTERS 10
    static const uint8_t    _checked_registers[MPU6000_NUM_CHECKED_REGISTERS];
    uint8_t            _checked_values[MPU6000_NUM_CHECKED_REGISTERS];
    uint8_t            _checked_next;
    
    // use this to avoid processing measurements when in factory
    // self test
    volatile bool        _in_factory_test;
    
    // last temperature reading for print_info()
    float            _last_temperature;
    
    // keep last accel reading for duplicate detection
    uint16_t        _last_accel[3];
    bool            _got_duplicate;
    
    /**
     * Start automatic measurement.
     */
    void            start();
    
    /**
     * Stop automatic measurement.
     */
    void            stop();
    
    /**
     * Reset chip.
     *
     * Resets the chip and measurements ranges, but not scale and offset.
     */
    int            reset();
    
    /**
     * is_icm_device
     */
    bool         is_icm_device() { return !is_mpu_device(); }
    /**
     * is_mpu_device
     */
    bool         is_mpu_device() { return _device_type == MPU_DEVICE_TYPE_MPU6000; }
    
    
#if defined(USE_I2C)
    /**
     * When the I2C interfase is on
     * Perform a poll cycle; collect from the previous measurement
     * and start a new one.
     *
     * This is the heart of the measurement state machine.  This function
     * alternately starts a measurement, or collects the data from the
     * previous measurement.
     *
     * When the interval between measurements is greater than the minimum
     * measurement interval, a gap is inserted between collection
     * and measurement to provide the most recent measurement possible
     * at the next interval.
     */
    void            cycle();
    
    /**
     * Static trampoline from the workq context; because we don't have a
     * generic workq wrapper yet.
     *
     * @param arg        Instance pointer for the driver that is polling.
     */
    static void        cycle_trampoline(void *arg);
    
    void use_i2c(bool on_true) { _use_hrt = !on_true; }
    
#endif
    
    bool is_i2c(void) { return !_use_hrt; }
    
    
    /**
     * Static trampoline from the hrt_call context; because we don't have a
     * generic hrt wrapper yet.
     *
     * Called by the HRT in interrupt context at the specified rate if
     * automatic polling is enabled.
     *
     * @param arg        Instance pointer for the driver that is polling.
     */
    static void        measure_trampoline(void *arg);
    
    /**
     * Fetch measurements from the sensor and update the report buffers.
     */
    int            measure();
    
    /**
     * Read a register from the MPU6000
     *
     * @param        The register to read.
     * @return        The value that was read.
     */
    uint8_t            read_reg(unsigned reg, uint32_t speed = MPU6000_LOW_BUS_SPEED);
    uint16_t        read_reg16(unsigned reg);
    
    
    /**
     * Write a register in the MPU6000
     *
     * @param reg        The register to write.
     * @param value        The new value to write.
     */
    int                write_reg(unsigned reg, uint8_t value);
    
    /**
     * Modify a register in the MPU6000
     *
     * Bits are cleared before bits are set.
     *
     * @param reg        The register to modify.
     * @param clearbits    Bits in the register to clear.
     * @param setbits    Bits in the register to set.
     */
    void            modify_reg(unsigned reg, uint8_t clearbits, uint8_t setbits);
    
    /**
     * Write a register in the MPU6000, updating _checked_values
     *
     * @param reg        The register to write.
     * @param value        The new value to write.
     */
    void            write_checked_reg(unsigned reg, uint8_t value);
    
    /**
     * Set the MPU6000 measurement range.
     *
     * @param max_g        The maximum G value the range must support.
     * @return        OK if the value can be supported, -ERANGE otherwise.
     */
    int                set_accel_range(unsigned max_g);
    
    /**
     * Swap a 16-bit value read from the MPU6000 to native byte order.
     */
    uint16_t        swap16(uint16_t val) { return (val >> 8) | (val << 8);    }
    
    /**
     * Get the internal / external state
     *
     * @return true if the sensor is not on the main MCU board
     */
    bool            is_external()
    {
        unsigned dummy;
        return _interface->ioctl(ACCELIOCGEXTERNAL, dummy);
    }
    
    /**
     * Measurement self test
     *
     * @return 0 on success, 1 on failure
     */
    int             self_test();
    
    /**
     * Accel self test
     *
     * @return 0 on success, 1 on failure
     */
    int                accel_self_test();
    
    /**
     * Gyro self test
     *
     * @return 0 on success, 1 on failure
     */
    int                gyro_self_test();
    
    /*
     set low pass filter frequency
     */
    void             _set_dlpf_filter(uint16_t frequency_hz);
    void             _set_icm_acc_dlpf_filter(uint16_t frequency_hz);
    
    /*
     set sample rate (approximate) - 8kHz to 5Hz
     */
    void            _set_sample_rate(unsigned desired_sample_rate_hz);
    
    /*
     check that key registers still have the right value
     */
    void            check_registers(void);
    
    /* do not allow to copy this class due to pointer data members */
    MPU6000(const MPU6000 &);
    MPU6000 operator=(const MPU6000 &);
    
};





/**
 * Helper class implementing the gyro driver node.
 */
class MPU6000_gyro : public device::CDev
{
public:
    MPU6000_gyro(MPU6000 *parent, const char *path);
    ~MPU6000_gyro();
    
    virtual ssize_t        read(struct file *filp, char *buffer, size_t buflen);
    virtual int        ioctl(struct file *filp, int cmd, unsigned long arg);
    
    virtual int        init();
    
protected:
    friend class MPU6000;
    
    void            parent_poll_notify();
    
private:
    MPU6000            *_parent;
    orb_advert_t        _gyro_topic;
    orb_advert_t        _delta_angle_topic;
    int            _gyro_orb_class_instance;
    int            _gyro_class_instance;
    
    /* do not allow to copy this class due to pointer data members */
    MPU6000_gyro(const MPU6000_gyro &);
    MPU6000_gyro operator=(const MPU6000_gyro &);
};

