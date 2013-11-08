/**
 * @file current_sensor.cpp
 * @author Benedikt Imbach

 *
 *Current sensor driver based on the FHS 40-PSP600 current sensor
 *in combination with the ADC AD7997 with I2C interface

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
#include <drivers/drv_current_sensor.h>
#include <drivers/device/ringbuffer.h>

#include <uORB/uORB.h>
#include <uORB/topics/subsystem_info.h>

#include <board_config.h>

/* Configuration Constants */
#define MB12XX_BUS 			PX4_I2C_BUS_EXPANSION
#define MB12XX_BASEADDR 	0x20 /* 7-bit address. 8-bit address is 0xE0 	-> 0x20 = 0b010 0000

/* MB12xx Registers addresses */

#define MB12XX_TAKE_RANGE_REG	0x70		/* Measure range Register, use all channels */
#define MB12XX_SET_ADDRESS_1	0xAA		/* Change address 1 Register */
#define MB12XX_SET_ADDRESS_2	0xA5		/* Change address 2 Register */

/* Device limits */
#define MB12XX_MIN_DISTANCE (0.20f)
#define MB12XX_MAX_DISTANCE (7.65f)

#define MB12XX_CONVERSION_INTERVAL 60000 /* 60ms */

/* oddly, ERROR is not defined for c++ */
#ifdef ERROR
# undef ERROR
#endif
static const int ERROR = -1;

#ifndef CONFIG_SCHED_WORKQUEUE
# error This requires CONFIG_SCHED_WORKQUEUE.
#endif

class MB12XX : public device::I2C
{
public:
	MB12XX(int bus = MB12XX_BUS, int address = MB12XX_BASEADDR);
	virtual ~MB12XX();

	virtual int 		init();

	virtual ssize_t		read(struct file *filp, char *buffer, size_t buflen);
	virtual int			ioctl(struct file *filp, int cmd, unsigned long arg);

	/**
	* Diagnostics - print some basic information about the driver.
	*/
	void				print_info();

protected:
	virtual int			probe();

private:
	float				_min_distance;
	float				_max_distance;
	work_s				_work;
	RingBuffer		*_reports;
	bool				_sensor_ok;
	int					_measure_ticks;
	bool				_collect_phase;

	orb_advert_t		_range_finder_topic;

	perf_counter_t		_sample_perf;
	perf_counter_t		_comms_errors;
	perf_counter_t		_buffer_overflows;

	/**
	* Test whether the device supported by the driver is present at a
	* specific address.
	*
	* @param address	The I2C bus address to probe.
	* @return		True if the device is present.
	*/
	int					probe_address(uint8_t address);

	/**
	* Initialise the automatic measurement state machine and start it.
	*
	* @note This function is called at open and error time.  It might make sense
	*       to make it more aggressive about resetting the bus in case of errors.
	*/
	void				start();

	/**
	* Stop the automatic measurement state machine.
	*/
	void				stop();

	/**
	* Set the min and max distance thresholds if you want the end points of the sensors
	* range to be brought in at all, otherwise it will use the defaults MB12XX_MIN_DISTANCE
	* and MB12XX_MAX_DISTANCE
	*/
	void				set_minimum_distance(float min);
	void				set_maximum_distance(float max);
	float				get_minimum_distance();
	float				get_maximum_distance();

	/**
	* Perform a poll cycle; collect from the previous measurement
	* and start a new one.
	*/
	void				cycle();
	int					measure();
	int					collect();
	/**
	* Static trampoline from the workq context; because we don't have a
	* generic workq wrapper yet.
	*
	* @param arg		Instance pointer for the driver that is polling.
	*/
	static void		cycle_trampoline(void *arg);


};

/*
 * Driver 'main' command.
 */
extern "C" __EXPORT int mb12xx_main(int argc, char *argv[]);

MB12XX::MB12XX(int bus, int address) :
	I2C("MB12xx", CURRENT_SENSOR_DEVICE_PATH, bus, address, 100000),
	_min_distance(MB12XX_MIN_DISTANCE),
	_max_distance(MB12XX_MAX_DISTANCE),
	_reports(nullptr),
	_sensor_ok(false),
	_measure_ticks(0),
	_collect_phase(false),
	_range_finder_topic(-1),
	_sample_perf(perf_alloc(PC_ELAPSED, "mb12xx_read")),
	_comms_errors(perf_alloc(PC_COUNT, "mb12xx_comms_errors")),
	_buffer_overflows(perf_alloc(PC_COUNT, "mb12xx_buffer_overflows"))
{
	// enable debug() calls
	_debug_enabled = true;

	// work_cancel in the dtor will explode if we don't do this...
	memset(&_work, 0, sizeof(_work));
}

MB12XX::~MB12XX()
{
	/* make sure we are truly inactive */
	stop();

	/* free any existing reports */
	if (_reports != nullptr)
		delete _reports;
}

int
MB12XX::init()
{
	int ret = ERROR;

	/* do I2C init (and probe) first */
	if (I2C::init() != OK)
		goto out;

	/* allocate basic report buffers */
	_reports = new RingBuffer(2, sizeof(current_sensor_report));

	if (_reports == nullptr)
		goto out;

	/* get a publish handle on the range finder topic */
	struct current_sensor_report zero_report;
	memset(&zero_report, 0, sizeof(zero_report));
	_range_finder_topic = orb_advertise(ORB_ID(sensor_current_sensor), &zero_report);

	if (_range_finder_topic < 0)
		debug("failed to create sensor_current_sensor object. Did you start uOrb?");

	ret = OK;
	/* sensor is ok, but we don't really know if it is within range */
	_sensor_ok = true;
out:
	return ret;
}

int
MB12XX::probe()
{
	return measure();
}

void
MB12XX::set_minimum_distance(float min)
{
	_min_distance = min;
}

void
MB12XX::set_maximum_distance(float max)
{
	_max_distance = max;
}

float
MB12XX::get_minimum_distance()
{
	return _min_distance;
}

float
MB12XX::get_maximum_distance()
{
	return _max_distance;
}

int
MB12XX::ioctl(struct file *filp, int cmd, unsigned long arg)
{
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
					_measure_ticks = USEC2TICK(MB12XX_CONVERSION_INTERVAL);

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
					if (ticks < USEC2TICK(MB12XX_CONVERSION_INTERVAL))
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

	case RANGEFINDERIOCSETMINIUMDISTANCE:
	{
		set_minimum_distance(*(float *)arg);
		return 0;
	}
	break;
	case RANGEFINDERIOCSETMAXIUMDISTANCE:
	{
		set_maximum_distance(*(float *)arg);
		return 0;
	}
	break;
	default:
		/* give it to the superclass */
		return I2C::ioctl(filp, cmd, arg);
	}
}

ssize_t
MB12XX::read(struct file *filp, char *buffer, size_t buflen)
{
	unsigned count = buflen / sizeof(struct current_sensor_report);
	struct current_sensor_report *rbuf = reinterpret_cast<struct current_sensor_report *>(buffer);
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
		usleep(MB12XX_CONVERSION_INTERVAL);

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

int
MB12XX::measure()
{
	int ret;

	/*
	 * Send the command to begin a measurement.
	 */
	uint8_t cmd = MB12XX_TAKE_RANGE_REG;
	ret = transfer(&cmd, 1, nullptr, 0);

	if (OK != ret)
	{
		perf_count(_comms_errors);
		log("i2c::transfer returned %d", ret);
		return ret;
	}
	ret = OK;

	return ret;
}

int
MB12XX::collect()
{
	int	ret = -EIO;

	/* read from the sensor */
	uint8_t val[16];

	perf_begin(_sample_perf);
	uint8_t cmd = MB12XX_TAKE_RANGE_REG;
	ret = transfer(&cmd, 1, &val[0], 16);

	if (ret < 0)
	{
		log("error reading from sensor: %d", ret);
		perf_count(_comms_errors);
		perf_end(_sample_perf);
		return ret;
	}

	struct current_sensor_report report;
	/* this should be fairly close to the end of the measurement, so the best approximation of the time */
	report.timestamp = hrt_absolute_time();
        report.error_count = perf_event_count(_comms_errors);

	report.vin1 = ((((0x0F) & val[0]) << 8) | val[1]);	//Datasheet AD7998 p.20
	report.vin2 = ((((0x0F) & val[2]) << 8) | val[3]);
	report.vin3 = ((((0x0F) & val[4]) << 8) | val[5]);
	report.vin4 = ((((0x0F) & val[6]) << 8) | val[7]);
	report.vin5 = ((((0x0F) & val[8]) << 8) | val[9]);
	report.vin6 = ((((0x0F) & val[10]) << 8) | val[11]);
	report.vin7 = ((((0x0F) & val[12]) << 8) | val[13]);
	report.vin8 = ((((0x0F) & val[14]) << 8) | val[15]);



	// Check
	uint8_t cmdxx = {0x02};		//Configreg:  -> 0h0, 0b0010 -> 0h2,    ergibt 0h02
												//Configreg: 0bxxxx111111111000
	transfer(&cmdxx, 1, nullptr, 0);

	uint8_t readback [2] = {0,0};
	transfer(nullptr, 0, &readback[0], 2);
	report.valid = readback[0] << 8 | readback[1];


	//report.valid = si_units > get_minimum_distance() && si_units < get_maximum_distance() ? 1 : 0;

	/* publish it */
	orb_publish(ORB_ID(sensor_current_sensor), _range_finder_topic, &report);

	if (_reports->force(&report)) {
		perf_count(_buffer_overflows);
	}

	/* notify anyone waiting for data */
	poll_notify(POLLIN);

	ret = OK;

	perf_end(_sample_perf);
	return ret;
}

void
MB12XX::start()
{
	/* reset the report ring and state machine */
	_collect_phase = false;
	_reports->flush();


	// Set the Configuration Register and activate the channels to be converted
	int ret;

	uint8_t cmd [3] = {0x02, 0x0F, 0xF8};		//Configreg:  -> 0h0, 0b0010 -> 0h2,    ergibt 0h02
												//Configreg: 0bxxxx111111111000
	ret = transfer(&cmd[0], 3, nullptr, 0);

	if (OK != ret)
	{
		perf_count(_comms_errors);
		log("i2c::transfer returned %d", ret);
	}


	// Check
	uint8_t readback [2] = {0,0};
	ret = transfer(nullptr, 0, &readback[0], 2);
	if (readback[1] != cmd[2] || readback[0] != cmd[1]){
		log("Readback from Configuration Register is not the same: \t%d\n", readback[0] << 8 | readback[1]);
	}




	/* schedule a cycle to start things */
	work_queue(HPWORK, &_work, (worker_t)&MB12XX::cycle_trampoline, this, 1);

	/* notify about state change */
	struct subsystem_info_s info = {
		true,
		true,
		true,
		SUBSYSTEM_TYPE_RANGEFINDER};
	static orb_advert_t pub = -1;

	if (pub > 0) {
		orb_publish(ORB_ID(subsystem_info), pub, &info);
	} else {
		pub = orb_advertise(ORB_ID(subsystem_info), &info);
	}
}

void
MB12XX::stop()
{
	work_cancel(HPWORK, &_work);
}

void
MB12XX::cycle_trampoline(void *arg)
{
	MB12XX *dev = (MB12XX *)arg;

	dev->cycle();
}

void
MB12XX::cycle()
{
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
		if (_measure_ticks > USEC2TICK(MB12XX_CONVERSION_INTERVAL)) {

			/* schedule a fresh cycle call when we are ready to measure again */
			work_queue(HPWORK,
				   &_work,
				   (worker_t)&MB12XX::cycle_trampoline,
				   this,
				   _measure_ticks - USEC2TICK(MB12XX_CONVERSION_INTERVAL));

			return;
		}
	}

	/* measurement phase */
	if (OK != measure())
		log("measure error");

	/* next phase is collection */
	_collect_phase = true;

	/* schedule a fresh cycle call when the measurement is done */
	work_queue(HPWORK,
		   &_work,
		   (worker_t)&MB12XX::cycle_trampoline,
		   this,
		   USEC2TICK(MB12XX_CONVERSION_INTERVAL));
}

void
MB12XX::print_info()
{
	perf_print_counter(_sample_perf);
	perf_print_counter(_comms_errors);
	perf_print_counter(_buffer_overflows);
	printf("poll interval:  %u ticks\n", _measure_ticks);
	_reports->print_info("report queue");
}

/**
 * Local functions in support of the shell command.
 */
namespace mb12xx
{

/* oddly, ERROR is not defined for c++ */
#ifdef ERROR
# undef ERROR
#endif
const int ERROR = -1;

MB12XX	*g_dev;

void	start();
void	stop();
void	test();
void	reset();
void	info();

/**
 * Start the driver.
 */
void
start()
{
	int fd;

	if (g_dev != nullptr)
		errx(1, "already started");

	/* create the driver */
	g_dev = new MB12XX(MB12XX_BUS);

	if (g_dev == nullptr)
		goto fail;

	if (OK != g_dev->init())
		goto fail;

	/* set the poll rate to default, starts automatic data collection */
	fd = open(CURRENT_SENSOR_DEVICE_PATH, O_RDONLY);

	if (fd < 0)
		goto fail;

	if (ioctl(fd, SENSORIOCSPOLLRATE, SENSOR_POLLRATE_DEFAULT) < 0)
		goto fail;

	exit(0);

fail:

	if (g_dev != nullptr)
	{
		delete g_dev;
		g_dev = nullptr;
	}

	errx(1, "driver start failed");
}

/**
 * Stop the driver
 */
void stop()
{
	if (g_dev != nullptr)
	{
		delete g_dev;
		g_dev = nullptr;
	}
	else
	{
		errx(1, "driver not running");
	}
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
	struct current_sensor_report report;
	ssize_t sz;
	int ret;

	int fd = open(CURRENT_SENSOR_DEVICE_PATH, O_RDONLY);

	if (fd < 0)
		err(1, "%s open failed (try 'mb12xx start' if the driver is not running", CURRENT_SENSOR_DEVICE_PATH);

	/* do a simple demand read */
	sz = read(fd, &report, sizeof(report));

	if (sz != sizeof(report))
		err(1, "immediate read failed");

	warnx("single read");
	warnx("measurement: %0.2f m", (double)report.vin1);
	warnx("time:        %lld", report.timestamp);


/*

	// start the sensor polling at 2Hz
	if (OK != ioctl(fd, SENSORIOCSPOLLRATE, 2))
		errx(1, "failed to set 2Hz poll rate");

	// read the sensor 5x and report each value
	for (unsigned i = 0; i < 50; i++) {
		struct pollfd fds;

		// wait for data to be ready
		fds.fd = fd;
		fds.events = POLLIN;
		ret = poll(&fds, 1, 2000);

		if (ret != 1)
			errx(1, "timed out waiting for sensor data");

		// now go get it
		sz = read(fd, &report, sizeof(report));

		if (sz != sizeof(report))
			err(1, "periodic read failed");

		warnx("periodic read %u", i);
		warnx("measurement: %0.3f", (double)report.distance);
		warnx("time:        %lld", report.timestamp);
	}

	errx(0, "PASS");

*/
}

/**
 * Reset the driver.
 */
void
reset()
{
	int fd = open(CURRENT_SENSOR_DEVICE_PATH, O_RDONLY);

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
void
info()
{
	if (g_dev == nullptr)
		errx(1, "driver not running");

	printf("state @ %p\n", g_dev);
	g_dev->print_info();

	exit(0);
}

} // namespace

int
mb12xx_main(int argc, char *argv[])
{
	/*
	 * Start/load the driver.
	 */
	if (!strcmp(argv[1], "start"))
		mb12xx::start();

	 /*
	  * Stop the driver
	  */
	 if (!strcmp(argv[1], "stop"))
		 mb12xx::stop();

	/*
	 * Test the driver/device.
	 */
	if (!strcmp(argv[1], "test"))
		mb12xx::test();

	/*
	 * Reset the driver.
	 */
	if (!strcmp(argv[1], "reset"))
		mb12xx::reset();

	/*
	 * Print driver information.
	 */
	if (!strcmp(argv[1], "info") || !strcmp(argv[1], "status"))
		mb12xx::info();

	errx(1, "unrecognized command, try 'start', 'test', 'reset' or 'info'");
}
