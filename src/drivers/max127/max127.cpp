/****************************************************************************
 *
 *   Copyright (c) 2013 PX4 Development Team. All rights reserved.
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
 * @file max127.cpp
 * @author Lukas Koepfli
 *
 * Driver for the MAX127 analog to digital converter connected via I2C.
 * This driver converts only the first two channels. It is used
 * to detect the relative angle between a unmanned vehicle and a paraglider.
 */

#include <nuttx/config.h>

#include <drivers/device/i2c.h>

#include <sys/types.h>
#include <stdint.h>
#include <stdlib.h>
#include <stdbool.h>
#include <semaphore.h>
#include <string.h>
#include <fcntl.h>
#include <poll.h>
#include <errno.h>
#include <stdio.h>
#include <math.h>
#include <unistd.h>

#include <nuttx/arch.h>
#include <nuttx/wqueue.h>
#include <nuttx/clock.h>

#include <systemlib/perf_counter.h>
#include <systemlib/err.h>

#include <drivers/drv_hrt.h>
#include <drivers/drv_rel_angle.h>
#include <drivers/device/ringbuffer.h>

#include <uORB/uORB.h>
#include <uORB/topics/subsystem_info.h>
#include <uORB/topics/vehicle_paraglider_angle.h>

#include <board_config.h>

/* Configuration Constants */
#define MAX127_BUS 				PX4_I2C_BUS_EXPANSION /* The I2C Bus where the max127 is wired */
#define MAX127_BASEADDR 		0x28 	/* 7-bit address. 8-bit address is 0x50 */

#define MAX127_RANGE 			1 		/* Defines the Input Range 1=0to+5V ; 2=0to+10V ; 3=-5to+5V ; 4=-10to+10V */
#define MAX127_USED_CHANNELS 	2 		/* Defines the used number of channels */

enum MODE {
	_0TO5V = 0, /* Input Range from 0 to 5V */
	_0TO10V = 1, /* Input Range from 0 to 10V */
	_5TO5V = 2, /* Input Range from -5 to 5V */
	_10TO10V = 3 /* Input Range from -10 to 10V */
};

# define MODE					0		/* Default Mode is 0 to 5V input Range */

/* Limits of the detectable raw values potentiometer for angle measurement between vehicle and paraglider */
#define MIN_RAW_LEFT (0.0) /* Defines the minimal raw value with the left Potentiometer */
#define MAX_RAW_LEFT (4095.0) /* Defines the maximum raw value with the left Potentiometer */
#define MIN_RAW_RIGHT (0.0) /* Defines the minimal raw value with the right Potentiometer */
#define MAX_RAW_RIGHT (4095.0) /* Defines the maximum raw value with the right Potentiometer */

/* Limits of the detectable angles between vehicle and paraglider */
#define MIN_ANGLE_LEFT (0.0) /* Defines the minimal detectable angle with the left Potentiometer */
#define MAX_ANGLE_LEFT (M_TWOPI) /* Defines the maximum detectable angle with the left Potentiometer */
#define MIN_ANGLE_RIGHT (0.0) /* Defines the minimal detectable angle with the right Potentiometer */
#define MAX_ANGLE_RIGHT (M_TWOPI) /* Defines the maximum detectable angle with the right Potentiometer */

#define MAX127_CONVERSION_INTERVAL 1000 /* Time until the IC has completed the conversion (it can be zero) 1ms*/

/* oddly, ERROR is not defined for c++ */
#ifdef ERROR
# undef ERROR
#endif
static const int ERROR = -1;

#ifndef CONFIG_SCHED_WORKQUEUE
# error This requires CONFIG_SCHED_WORKQUEUE.
#endif

class MAX127: public device::I2C {
public:
	MAX127(int bus = MAX127_BUS, int address = MAX127_BASEADDR);
	virtual ~MAX127();

	virtual int init();

	virtual ssize_t read(struct file *filp, char *buffer, size_t buflen);
	virtual int ioctl(struct file *filp, int cmd, unsigned long arg);

	/**
	 * Diagnostics - print some basic information about the driver.
	 */
	void print_info();

protected:
	virtual int probe();

private:
	float _min_raw_left; /* Defines the minimal raw value with the left Potentiometer */
	float _max_raw_left; /* Defines the maximum raw value with the left Potentiometer */
	float _min_raw_right; /* Defines the minimal raw value with the right Potentiometer */
	float _max_raw_right; /* Defines the maximum raw value with the right Potentiometer */
	float _min_angle_left; /* Defines the minimal detectable angle with the left Potentiometer */
	float _max_angle_left; /* Defines the maximum detectable angle with the left Potentiometer */
	float _min_angle_right; /* Defines the minimal detectable angle with the right Potentiometer */
	float _max_angle_right; /* Defines the maximum detectable angle with the right potentiometer */
	struct vehicle_paraglider_angle_s _report_rel_angle; /*  uORB topic for relative angle between vehicle and paraglider */
	work_s _work;
	RingBuffer *_reports;bool _sensor_ok;
	int _measure_ticks;bool _collect_phase;

	orb_advert_t _rel_angle_topic; /* uORB pub for relative angle */

	perf_counter_t _sample_perf;
	perf_counter_t _comms_errors;
	perf_counter_t _buffer_overflows;

	/**
	 * Test whether the device supported by the driver is present at a
	 * specific address.
	 *
	 * @param address	The I2C bus address to probe.
	 * @return		True if the device is present.
	 */
	int probe_address(uint8_t address);

	/**
	 * Initialise the automatic measurement state machine and start it.
	 *
	 * @note This function is called at open and error time.  It might make sense
	 *       to make it more aggressive about resetting the bus in case of errors.
	 */
	void start();

	/**
	 * Stop the automatic measurement state machine.
	 */
	void stop();

	/**
	 * Set the min and max angle and raw value thresholds if you want the end points of the sensors
	 * range to be brought in at all, otherwise it will use the defaults values
	 */
	void set_minimum_raw_left(float min);
	void set_maximum_raw_left(float max);
	void set_minimum_raw_right(float min);
	void set_maximum_raw_right(float max);
	void set_minimum_angle_left(float min);
	void set_maximum_angle_left(float max);
	void set_minimum_angle_right(float min);
	void set_maximum_angle_right(float max);

	float get_minimum_raw_left();
	float get_maximum_raw_left();
	float get_minimum_raw_right();
	float get_maximum_raw_right();
	float get_minimum_angle_left();
	float get_maximum_angle_left();
	float get_minimum_angle_right();
	float get_maximum_angle_right();

	/**
	 * Perform a poll cycle; collect from the previous measurement
	 * and start a new one.
	 */
	void cycle();
	int measure();
	int collect();
	/**
	 * Static trampoline from the workq context; because we don't have a
	 * generic workq wrapper yet.
	 *
	 * @param arg		Instance pointer for the driver that is polling.
	 */
	static void cycle_trampoline(void *arg);

};

/*
 * Driver 'main' command.
 */
extern "C" __EXPORT int max127_main(int argc, char *argv[]);

MAX127::MAX127(int bus, int address) :
		I2C("MAX127", REL_ANGLE_DEVICE_PATH, bus, address, 100000), _min_raw_left(
				MIN_RAW_LEFT), _max_raw_left(MAX_RAW_LEFT), _min_raw_right(
				MIN_RAW_RIGHT), _max_raw_right(MAX_RAW_RIGHT), _min_angle_left(
				MIN_ANGLE_LEFT), _max_angle_left(MAX_ANGLE_LEFT), _min_angle_right(
				MIN_ANGLE_RIGHT), _max_angle_right(MAX_ANGLE_RIGHT), _reports(
				nullptr), _sensor_ok(false), _measure_ticks(0), _collect_phase(
				false), _rel_angle_topic(-1), _sample_perf(
				perf_alloc(PC_ELAPSED, "max127_read")), _comms_errors(
				perf_alloc(PC_COUNT, "max127_comms_errors")), _buffer_overflows(
				perf_alloc(PC_COUNT, "max127_buffer_overflows")) {
	// enable debug() calls
	_debug_enabled = true;

	// work_cancel in the dtor will explode if we don't do this...
	memset(&_work, 0, sizeof(_work));
}

MAX127::~MAX127() {
	/* make sure we are truly inactive */
	stop();

	/* free any existing reports */
	if (_reports != nullptr)
		delete _reports;
}

int MAX127::init() {
	int ret = ERROR;

	/* do I2C init (and probe) first */
	if (I2C::init() != OK)
		goto out;

	/* allocate basic report buffers */
	_reports = new RingBuffer(2, sizeof(_report_rel_angle));

	if (_reports == nullptr)
		goto out;

	/* get a publish handle on the relative angle topic */
	struct _report_rel_angle zero_report;
	memset(&zero_report, 0, sizeof(zero_report));
	_rel_angle_topic = orb_advertise(ORB_ID(vehicle_paraglider_angle),
			&zero_report);

	if (_rel_angle_topic < 0)
		debug("failed to create rel_angle object. Did you start uOrb?");

	ret = OK;
	/* sensor is ok, but we don't really know if the values useful */
	_sensor_ok = true;
	out: return ret;
}

int MAX127::probe() {
	return measure();
}

void MAX127::set_minimum_raw_left(float min) {
	_min_raw_left = min;
}

void MAX127::set_maximum_raw_left(float max) {
	_max_raw_left = max;
}

void MAX127::set_minimum_raw_right(float min) {
	_min_raw_right = min;
}

void MAX127::set_maximum_raw_right(float max) {
	_max_raw_right = max;
}

void MAX127::set_minimum_angle_left(float min) {
	_min_angle_left = min;
}

void MAX127::set_maximum_angle_left(float max) {
	_max_angle_left = max;
}

void MAX127::set_minimum_angle_right(float min) {
	_min_angle_right = min;
}

void MAX127::set_maximum_angle_right(float max) {
	_max_angle_right = max;
}

float MAX127::get_minimum_raw_left() {
	return _min_raw_left;
}

float MAX127::get_maximum_raw_left() {
	return _max_raw_left;
}

float MAX127::get_minimum_raw_right() {
	return _min_raw_right;
}

float MAX127::get_maximum_raw_right() {
	return _max_raw_right;
}

float MAX127::get_minimum_angle_left() {
	return _min_angle_left;
}

float MAX127::get_maximum_angle_left() {
	return _max_angle_left;
}

float MAX127::get_minimum_angle_right() {
	return _min_angle_right;
}

float MAX127::get_maximum_angle_right() {
	return _max_angle_right;
}

int MAX127::ioctl(struct file *filp, int cmd, unsigned long arg) {
	switch (cmd) {

	case SENSORIOCSPOLLRATE: {
		switch (arg) {

		/* switching to manual polling */
		case SENSOR_POLLRATE_MANUAL:
			stop();
			_measure_ticks = 0;
			return OK;

			/* external signalling (DRDY) not supported */
		case SENSOR_POLLRATE_EXTERNAL:

			/* zero would be bad */
		case 0:
			return -EINVAL;

			/* set default/max polling rate */
		case SENSOR_POLLRATE_MAX:
		case SENSOR_POLLRATE_DEFAULT: {
			/* do we need to start internal polling? */
			bool want_start = (_measure_ticks == 0);

			/* set interval for next measurement to minimum legal value */
			_measure_ticks = USEC2TICK(MAX127_CONVERSION_INTERVAL);

			/* if we need to start the poll state machine, do it */
			if (want_start)
				start();

			return OK;
		}

			/* adjust to a legal polling interval in Hz */
		default: {
			/* do we need to start internal polling? */
			bool want_start = (_measure_ticks == 0);

			/* convert hz to tick interval via microseconds */
			unsigned ticks = USEC2TICK(1000000 / arg);

			/* check against maximum rate */
			if (ticks < USEC2TICK(MAX127_CONVERSION_INTERVAL))
				return -EINVAL;

			/* update interval for next measurement */
			_measure_ticks = ticks;

			/* if we need to start the poll state machine, do it */
			if (want_start)
				start();

			return OK;
		}
		}
	}

	case SENSORIOCGPOLLRATE:
		if (_measure_ticks == 0)
			return SENSOR_POLLRATE_MANUAL;

		return (1000 / _measure_ticks);

	case SENSORIOCSQUEUEDEPTH: {
		/* lower bound is mandatory, upper bound is a sanity check */
		if ((arg < 1) || (arg > 100))
			return -EINVAL;

		irqstate_t flags = irqsave();
		if (!_reports->resize(arg)) {
			irqrestore(flags);
			return -ENOMEM;
		}
		irqrestore(flags);

		return OK;
	}

	case SENSORIOCGQUEUEDEPTH:
		return _reports->size();

	case SENSORIOCRESET:
		/* XXX implement this */
		return -EINVAL;

		break;
	default:
		/* give it to the superclass */
		return I2C::ioctl(filp, cmd, arg);
	}
}

ssize_t MAX127::read(struct file *filp, char *buffer, size_t buflen) {
	unsigned count = buflen / sizeof(struct _report_rel_angle);
	struct _report_rel_angle *rbuf =
			reinterpret_cast<struct _report_rel_angle *>(buffer);
	int ret = 0;

	/* buffer must be large enough */
	if (count < 1)
		return -ENOSPC;

	/* if automatic measurement is enabled */
	if (_measure_ticks > 0) {

		/*
		 * While there is space in the caller's buffer, and reports, copy them.
		 * Note that we may be pre-empted by the workq thread while we are doing this;
		 * we are careful to avoid racing with them.
		 */
		while (count--) {
			if (_reports->get(rbuf)) {
				ret += sizeof(*rbuf);
				rbuf++;
			}
		}

		/* if there was no data, warn the caller */
		return ret ? ret : -EAGAIN;
	}

	/* manual measurement - run one conversion */
	do {
		_reports->flush();

		/* trigger a measurement */
		if (OK != measure()) {
			ret = -EIO;
			break;
		}

		/* wait for it to complete */
		usleep(MAX127_CONVERSION_INTERVAL);

		/* run the collection phase */
		if (OK != collect()) {
			ret = -EIO;
			break;
		}

		/* state machine will have generated a report, copy it out */
		if (_reports->get(rbuf)) {
			ret = sizeof(*rbuf);
		}

	} while (0);

	return ret;
}

int MAX127::measure(int channel, enum MODE) {
	int ret;
	uint8_t cmd;

	if (channel > 7) {
		ret = EINVAL_STR;
	} else {

		/*
		 *  Create the control-byte
		 */
		switch (MODE) {
		case _0TO5V:
			cmd = (0x80 | (channel << 4));
			break;
		case _0TO10V:
			cmd = (0x88 | (channel << 4));
			;
			break;
		case _5TO5V:
			cmd = (0x84 | (channel << 4));
			;
			break;
		case _10TO10V:
			cmd = (0x8c | (channel << 4));
			;
			break;
		default:
			cmd = (0x80 | (channel << 4));
			;
			break;
		}
	}

	/*
	 * Send the command to begin a measurement.
	 */
	uint8_t cmd = MAX127_TAKE_VALUE_CH0;
	ret = transfer(&cmd, 1, nullptr, 0);

	if (OK != ret) {
		perf_count(_comms_errors);
		log("i2c::transfer returned %d", ret);
		return ret;
	}
	ret = OK;

	return ret;
}

int MAX127::collect() {
	int ret = -EIO;

	/* read from the sensor */
	uint8_t val[2] = { 0, 0 };

	perf_begin(_sample_perf);

	ret = transfer(nullptr, 0, &val[0], 2);

	if (ret < 0) {
		log("error reading from sensor: %d", ret);
		perf_count(_comms_errors);
		perf_end(_sample_perf);
		return ret;
	}

	uint16_t value = val[0] << 8 | val[1]; /* Convert two data-bytes to one */
	value = value >> 4; /* shift the value 4 bits to the left side because the last 4 bits of the second data-byte are zero*/
	/* Convert the raw value from the adc into a physical unit [rad] */
	float line = (get_maximum_angle_left() - get_minimum_angle_left())
			/ (get_maximum_raw_left() - get_minimum_raw_left()); /* [rad / digital_value] */
	float si_units = line * value; /* [rad] */
	struct vehicle_paraglider_angle_s report;

	/* this should be fairly close to the end of the measurement, so the best approximation of the time */
	report.timestamp = hrt_absolute_time();
	report.error_count = perf_event_count(_comms_errors);
	report.raw_left = value;
	report.angle_left = si_units;
	report.valid =
			si_units > get_minimum_angle_left()
					&& si_units < get_maximum_angle_left() ? 1 : 0;

	/* publish it */
	orb_publish(ORB_ID(vehicle_paraglider_angle), _rel_angle_topic, &report);

	if (_reports->force(&report)) {
		perf_count(_buffer_overflows);
	}

	/* notify anyone waiting for data */
	poll_notify(POLLIN);

	ret = OK;

	perf_end(_sample_perf);
	return ret;
}

void MAX127::start() {
	/* reset the report ring and state machine */
	_collect_phase = false;
	_reports->flush();

	/* schedule a cycle to start things */
	work_queue(HPWORK, &_work, (worker_t) & MAX127::cycle_trampoline, this, 1);

	/* notify about state change */
	struct subsystem_info_s info = { true, true, true,
			SUBSYSTEM_TYPE_RANGEFINDER };
	static orb_advert_t pub = -1;

	if (pub > 0) {
		orb_publish(ORB_ID(subsystem_info), pub, &info);
	} else {
		pub = orb_advertise(ORB_ID(subsystem_info), &info);
	}
}

void MAX127::stop() {
	work_cancel(HPWORK, &_work);
}

void MAX127::cycle_trampoline(void *arg) {
	MAX127 *dev = (MAX127 *) arg;

	dev->cycle();
}

void MAX127::cycle() {
	/* collection phase? */
	if (_collect_phase) {

		/* perform collection */
		if (OK != collect()) {
			log("collection error");
			/* restart the measurement state machine */
			start();
			return;
		}

		/* next phase is measurement */
		_collect_phase = false;

		/*
		 * Is there a collect->measure gap?
		 */
		if (_measure_ticks > USEC2TICK(MAX127_CONVERSION_INTERVAL)) {

			/* schedule a fresh cycle call when we are ready to measure again */
			work_queue(HPWORK, &_work, (worker_t) & MAX127::cycle_trampoline,
					this,
					_measure_ticks - USEC2TICK(MAX127_CONVERSION_INTERVAL));

			return;
		}
	}

	/* measurement phase */
	if (OK != measure())
		log("measure error");

	/* next phase is collection */
	_collect_phase = true;

	/* schedule a fresh cycle call when the measurement is done */
	work_queue(HPWORK, &_work, (worker_t) & MAX127::cycle_trampoline, this,
			USEC2TICK(MAX127_CONVERSION_INTERVAL));
}

void MAX127::print_info() {
	perf_print_counter(_sample_perf);
	perf_print_counter(_comms_errors);
	perf_print_counter(_buffer_overflows);
	printf("poll interval:  %u ticks\n", _measure_ticks);
	_reports->print_info("report queue");
}

/**
 * Local functions in support of the shell command.
 */
namespace max127 {

/* oddly, ERROR is not defined for c++ */
#ifdef ERROR
# undef ERROR
#endif
const int ERROR = -1;

MAX127 *g_dev;

void start();
void stop();
void test();
void reset();
void info();

/**
 * Start the driver.
 */
void start() {
	int fd;

	if (g_dev != nullptr)
		errx(1, "already started");

	/* create the driver */
	g_dev = new MAX127(MAX127_BUS);

	if (g_dev == nullptr)
		goto fail;

	if (OK != g_dev->init())
		goto fail;

	/* set the poll rate to default, starts automatic data collection */
	fd = open(REL_ANGLE_DEVICE_PATH, O_RDONLY);

	if (fd < 0)
		goto fail;

	if (ioctl(fd, SENSORIOCSPOLLRATE, SENSOR_POLLRATE_DEFAULT) < 0)
		goto fail;

	exit(0);

	fail:

	if (g_dev != nullptr) {
		delete g_dev;
		g_dev = nullptr;
	}

	errx(1, "driver start failed");
}

/**
 * Stop the driver
 */
void stop() {
	if (g_dev != nullptr) {
		delete g_dev;
		g_dev = nullptr;
	} else {
		errx(1, "driver not running");
	}
	exit(0);
}

/**
 * Perform some basic functional tests on the driver;
 * make sure we can collect data from the sensor in polled
 * and automatic modes.
 */
void test() {
	struct vehicle_paraglider_angle_s report;
	ssize_t sz;
	int ret;

	int fd = open(REL_ANGLE_DEVICE_PATH, O_RDONLY);

	if (fd < 0)
		err(1,
				"%s open failed (try 'max127 start' if the driver is not running",
				REL_ANGLE_DEVICE_PATH);

	/* do a simple demand read */
	sz = read(fd, &report, sizeof(report));

	if (sz != sizeof(report))
		err(1, "immediate read failed");

	warnx("single read");
	warnx("measurement: %0.2f m", (double) report.distance); // XXX
	warnx("time:        %lld", report.timestamp);

	/* start the sensor polling at 2Hz */
	if (OK != ioctl(fd, SENSORIOCSPOLLRATE, 2))
		errx(1, "failed to set 2Hz poll rate");

	/* read the sensor 5x and report each value */
	for (unsigned i = 0; i < 5; i++) {
		struct pollfd fds;

		/* wait for data to be ready */
		fds.fd = fd;
		fds.events = POLLIN;
		ret = poll(&fds, 1, 2000);

		if (ret != 1)
			errx(1, "timed out waiting for sensor data");

		/* now go get it */
		sz = read(fd, &report, sizeof(report));

		if (sz != sizeof(report))
			err(1, "periodic read failed");

		warnx("periodic read %u", i);
		warnx("measurement: %0.3f", (double) report.distance); // XXX
		warnx("time:        %lld", report.timestamp);
	}

	errx(0, "PASS");
}

/**
 * Reset the driver.
 */
void reset() {
	int fd = open(REL_ANGLE_DEVICE_PATH, O_RDONLY);

	if (fd < 0)
		err(1, "failed ");

	if (ioctl(fd, SENSORIOCRESET, 0) < 0)
		err(1, "driver reset failed");

	if (ioctl(fd, SENSORIOCSPOLLRATE, SENSOR_POLLRATE_DEFAULT) < 0)
		err(1, "driver poll restart failed");

	exit(0);
}

/**
 * Print a little info about the driver.
 */
void info() {
	if (g_dev == nullptr)
		errx(1, "driver not running");

	printf("state @ %p\n", g_dev);
	g_dev->print_info();

	exit(0);
}

} // namespace

int max127_main(int argc, char *argv[]) {
	/*
	 * Start/load the driver.
	 */
	if (!strcmp(argv[1], "start"))
		max127::start();

	/*
	 * Stop the driver
	 */
	if (!strcmp(argv[1], "stop"))
		max127::stop();

	/*
	 * Test the driver/device.
	 */
	if (!strcmp(argv[1], "test"))
		max127::test();

	/*
	 * Reset the driver.
	 */
	if (!strcmp(argv[1], "reset"))
		max127::reset();

	/*
	 * Print driver information.
	 */
	if (!strcmp(argv[1], "info") || !strcmp(argv[1], "status"))
		max127::info();

	errx(1, "unrecognized command, try 'start', 'test', 'reset' or 'info'");
}
