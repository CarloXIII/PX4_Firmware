/**
 * @file rpm_arduino.cpp
 * @author Benedikt Imbach, 2013

 *
 * Driver for the arudino board pro mini on a serial port
 */

#include <nuttx/clock.h>
#include <sys/types.h>
#include <stdint.h>
#include <stdio.h>
#include <stdbool.h>
#include <stdlib.h>
#include <semaphore.h>
#include <string.h>
#include <fcntl.h>
#include <poll.h>
#include <errno.h>
#include <stdio.h>
#include <math.h>
#include <unistd.h>
#include <fcntl.h>
#include <nuttx/config.h>
#include <nuttx/arch.h>
#include <arch/board/board.h>
#include <drivers/drv_hrt.h>
#include <drivers/device/i2c.h>
#include <systemlib/perf_counter.h>
#include <systemlib/scheduling_priorities.h>
#include <systemlib/err.h>
#include <uORB/uORB.h>
#include "rpm_arduino_parser.h"
//#include "drivers/drv_rpm.h"

#define TIMEOUT_5HZ 500
#define RATE_MEASUREMENT_PERIOD 5000000

/* oddly, ERROR is not defined for c++ */
#ifdef ERROR
# undef ERROR
#endif
static const int ERROR = -1;

#ifndef CONFIG_SCHED_WORKQUEUE
# error This requires CONFIG_SCHED_WORKQUEUE.
#endif



class RPM_ARDUINO : public device::CDev
{
public:
	RPM_ARDUINO(const char* uart_path);
	virtual ~RPM_ARDUINO();

	virtual int			init();

	virtual int			ioctl(struct file *filp, int cmd, unsigned long arg);

	/**
	 * Diagnostics - print some basic information about the driver.
	 */
	void				print_info();
	void				print_status();

private:

	bool				_task_should_exit;				///< flag to make the main worker task exit
	int					_serial_fd;						///< serial interface to XSENS
	unsigned			_baudrate;						///< current baudrate
	char				_port[20];						///< device / serial port path
	volatile int		_task;							///< worker task
	bool				_healthy;						///< flag to signal if the XSENS is ok
	bool 				_baudrate_changed;				///< flag to signal that the baudrate with the XSENS has changed
	bool				_mode_changed;					///< flag that the XSENS mode has changed
	RPM_ARDUINO_Helper	*_Helper;						///< instance of XSENS parser
	struct rpm_report 	_report;						///< uORB topic for xsens gps position
	orb_advert_t		_report_pub;					///< uORB pub for gps position
	//orb_advert_t		_report_pub_sensor_combined;		///< uORB pub for gps position
	float				_rate;							///< position update rate


	/**
	 * Try to configure the XSENS, handle outgoing communication to the XSENS
	 */
	void				config();

	/**
	 * Trampoline to the worker task
	 */
	static void			task_main_trampoline(void *arg);


	/**
	 * Worker task: main XSENS thread that configures the XSENS and parses incoming data, always running
	 */
	void				task_main(void);

	/**
	 * Set the baudrate of the UART to the XSENS
	 */
	int				set_baudrate(unsigned baud);

	/**
	 * Send a reset command to the XSENS
	 */
	void				cmd_reset();

};


/*
 * Driver 'main' command.
 */
extern "C" __EXPORT int rpm_arduino_main(int argc, char *argv[]);

namespace
{

RPM_ARDUINO	*g_dev;

}


RPM_ARDUINO::RPM_ARDUINO(const char* uart_path) :
	CDev("rpm_arduino", RPM_ARDUINO_DEVICE_PATH),
	_task_should_exit(false),
	_healthy(false),
	_mode_changed(false),
	_Helper(nullptr),
	_report_pub(-1),
	//_report_pub_sensor_combined(-1),
	_rate(0.0f)
{
	/* store port name */
	strncpy(_port, uart_path, sizeof(_port));
	/* enforce null termination */
	_port[sizeof(_port) - 1] = '\0';

	/* we need this potentially before it could be set in task_main */
	g_dev = this;
	memset(&_report, 0, sizeof(_report));
	_debug_enabled = true;
}

RPM_ARDUINO::~RPM_ARDUINO()
{
	/* tell the task we want it to go away */
	_task_should_exit = true;

	/* spin waiting for the task to stop */
	for (unsigned i = 0; (i < 10) && (_task != -1); i++) {
		/* give it another 100ms */
		usleep(100000);
	}

	/* well, kill it anyway, though this will probably crash */
	if (_task != -1)
		task_delete(_task);
	g_dev = nullptr;

}

int
RPM_ARDUINO::init()
{
	int ret = ERROR;

	/* do regular cdev init */
	if (CDev::init() != OK)
		goto out;

	/* start the XSENS driver worker task */
	_task = task_create("rpm_arduino", SCHED_PRIORITY_SLOW_DRIVER, 2048, (main_t)&RPM_ARDUINO::task_main_trampoline, nullptr);

	if (_task < 0) {
		warnx("task start failed: %d", errno);
		return -errno;
	}

	ret = OK;
out:
	return ret;
}

int
RPM_ARDUINO::ioctl(struct file *filp, int cmd, unsigned long arg)
{
	lock();

	int ret = OK;

	switch (cmd) {
	case SENSORIOCRESET:
		cmd_reset();
		break;
	}

	unlock();

	return ret;
}

void
RPM_ARDUINO::task_main_trampoline(void *arg)
{
	g_dev->task_main();
}

void
RPM_ARDUINO::task_main()
{
	log("starting");

	/* open the serial port */
	_serial_fd = ::open(_port, O_RDWR);

	if (_serial_fd < 0) {
		log("failed to open serial port: %s err: %d", _port, errno);
		/* tell the dtor that we are exiting, set error code */
		_task = -1;
		_exit(1);
	}

	uint64_t last_rate_measurement = hrt_absolute_time();
	unsigned last_rate_count = 0;

	/* loop handling received serial bytes and also configuring in between */
	while (!_task_should_exit) {

		if (_Helper != nullptr) {
			delete(_Helper);
			/* set to zero to ensure parser is not used while not instantiated */
			_Helper = nullptr;
		}


		_Helper = new RPM_ARDUINO_PARSER(_serial_fd, &_report);

		//warnx("xsens: task main started");

		unlock();
		if (_Helper->configure(_baudrate) == 0) {
			unlock();

			// XSENS is obviously detected successfully, reset statistics
			_Helper->reset_update_rates();
			//warnx("xsens detected");

			while (_Helper->receive(TIMEOUT_5HZ) > 0 && !_task_should_exit) {
//				lock();
				//warnx("xsens: receive");
				/* opportunistic publishing - else invalid data would end up on the bus */
				if (_report_pub > 0) {
					if(_Helper->rpm_arduino_new_data == true){
						orb_publish(ORB_ID(sensor_rpm), _report_pub, &_report);
						_Helper->rpm_arduino_new_data = false;
					}
				} else {
					_report_pub = orb_advertise(ORB_ID(sensor_rpm), &_report);
				}


				last_rate_count++;

				/* measure update rate every 5 seconds */
				if (hrt_absolute_time() - last_rate_measurement > RATE_MEASUREMENT_PERIOD) {
					_rate = last_rate_count / ((float)((hrt_absolute_time() - last_rate_measurement)) / 1000000.0f);
					last_rate_measurement = hrt_absolute_time();
					last_rate_count = 0;
					_Helper->store_update_rates();
					_Helper->reset_update_rates();
				}

				if (!_healthy) {
					warnx("module found");
					_healthy = true;
				}
			}
			if (_healthy) {
				warnx("module lost");
				_healthy = false;
				_rate = 0.0f;
			}

			lock();
		}
		lock();

	}
	debug("exiting");

	::close(_serial_fd);

	/* tell the dtor that we are exiting */
	_task = -1;
	_exit(0);
}



void
RPM_ARDUINO::cmd_reset()
{
	//XXX add reset?
}

void
RPM_ARDUINO::print_info()
{
	warnx("port: %s, baudrate: %d, status: %s", _port, _baudrate, (_healthy) ? "OK" : "NOT OK");
	usleep(100000);
}

void
RPM_ARDUINO::print_status()
{
	printf("\n******\nRPM Driver data Arduino");
	printf("RPM packet:");
	printf("\nRPM: %.3f", _report.rpm);
	printf("\ntime since last update [s]: %.3f", ((float)((hrt_absolute_time() - _report.timestamp)) / 1000000.0f) );
	printf("\nrpm package rate: %.3f", _rate );
	//printf("\ngps data age (bGPS): %.3f", xsens_gps_pvt->bgps );

	//xsens::_
	//errx(0, "PASS");
	//errx(0, "PASS");
	//("port: %s, baudrate: %d, status: %s", _port, _baudrate, (_healthy) ? "OK" : "NOT OK");

	usleep(100000);
}

/**
 * Local functions in support of the shell command.
 */
namespace rpm_arduino
{

RPM_ARDUINO	*g_dev;

void	start(const char *uart_path);
void	stop();
void	test();
void	reset();
void	info();

/**
 * Start the driver.
 */
void
start(const char *uart_path)
{
	int fd;

	if (g_dev != nullptr)
		errx(1, "already started");

	/* create the driver */
	g_dev = new RPM_ARDUINO(uart_path);

	if (g_dev == nullptr)
		goto fail;

	if (OK != g_dev->init())
		goto fail;

	/* set the poll rate to default, starts automatic data collection */
	fd = open(RPM_ARDUINO_DEVICE_PATH, O_RDONLY);

	if (fd < 0) {
		errx(1, "Could not open device path: %s\n", RPM_ARDUINO_DEVICE_PATH);
		goto fail;
	}
	exit(0);

fail:

	if (g_dev != nullptr) {
		delete g_dev;
		g_dev = nullptr;
	}

	errx(1, "driver start failed");
}

/**
 * Stop the driver.
 */
void
stop()
{
	delete g_dev;
	g_dev = nullptr;

	exit(0);
}

/**
 * Perform some basic functional tests on the driver;
 * make sure we can collect data from the sensor in polled
 * and automatic modes.
 */
void
test()
{
	if (g_dev == nullptr)
		errx(1, "driver not running");

	g_dev->print_status();
	exit(0);
}

/**
 * Reset the driver.
 */
void
reset()
{
	int fd = open(RPM_ARDUINO_DEVICE_PATH, O_RDONLY);

	if (fd < 0)
		err(1, "failed ");

	if (ioctl(fd, SENSORIOCRESET, 0) < 0)
		err(1, "driver reset failed");

	exit(0);
}

/**
 * Print the status of the driver.
 */
void
info()
{
	if (g_dev == nullptr)
		errx(1, "driver not running");

	g_dev->print_info();

	exit(0);
}

} // namespace


int
rpm_arduino_main(int argc, char *argv[])
{

	/* set to default */
	char* device_name = RPM_ARDUINO_DEFAULT_UART_PORT;

	/*
	 * Start/load the driver.
	 */
	if (!strcmp(argv[1], "start")) {
		/* work around getopt unreliability */
		if (argc > 3) {
			if (!strcmp(argv[2], "-d")) {
				device_name = argv[3];
			} else {
				goto out;
			}
		}
		rpm_arduino::start(device_name);
	}

	if (!strcmp(argv[1], "stop"))
		rpm_arduino::stop();
	/*
	 * Test the driver/device.
	 */
	if (!strcmp(argv[1], "test"))
		rpm_arduino::test();

	/*
	 * Reset the driver.
	 */
	if (!strcmp(argv[1], "reset"))
		rpm_arduino::reset();

	/*
	 * Print driver status.
	 */
	if (!strcmp(argv[1], "status"))
		rpm_arduino::info();

out:
	errx(1, "unrecognized command, try 'start', 'stop', 'test', 'reset' or 'status' [-d /dev/ttyS3-n]");
}
