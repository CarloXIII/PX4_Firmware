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

#include <board_config.h>

#include <systemlib/perf_counter.h>
#include <systemlib/err.h>

#include <drivers/drv_current_sensor.h>
#include <drivers/drv_hrt.h>
#include <drivers/device/ringbuffer.h>

#include <uORB/uORB.h>

#include <uORB/topics/subsystem_info.h>

#define CURRENT_SENSOR_BUS 			PX4_I2C_BUS_EXPANSION

/* I2C bus address */
#define I2C_ADDRESS_AD7998	0x51	/* 7-bit address. 8-bit address is 0xA2 */

/* AD7998 Register Addresses  */
#define CONVERSION_RESULT_REG	0x0	/* Conversion Result Register (Read)  */
#define ALERT_STATUS_REG	0x1	/* Alert Status Register (Read/Write)   */
#define CONFIGURATION_REG	0x2	/* Configuration Register (Read/Write)   */
#define CYCLE_TIMER_REG		0x3	/* Cycle Timer Register (Read/Write)   */
#define DATA_LOW_REG_CH1	0x4	/* DATA LOW  Reg CH1 (Read/Write)  */
#define DATA_HIGH_REG_CH1	0x5	/* DATA HIGH  Reg CH1 (Read/Write)  */
#define HYSTERESIS_REG_CH1	0x6	/* Hysteresis Reg CH1 (Read/Write)   */
#define DATA_LOW_REG_CH2	0x7	/* DATA LOW  Reg CH2 (Read/Write)  */
#define DATA_HIGH_REG_CH2	0x8	/* DATA HIGH  Reg CH2 (Read/Write)   */
#define HYSTERESIS_REG_CH2	0x9	/* Hysteresis Reg CH2 (Read/Write)  */
#define DATA_LOW_REG_CH3	0xA	/* DATA LOW  Reg CH3 (Read/Write)  */
#define DATA_HIGH_REG_CH3	0xB	/* DATA HIGH  Reg CH3 (Read/Write)  */
#define HYSTERESIS_REG_CH3	0xC	/* Hysteresis Reg CH3 (Read/Write)  */
#define DATA_LOW_REG_CH4	0xD	/* DATA LOW  Reg CH4 (Read/Write)   */
#define DATA_HIGH_REG_CH4	0xE	/* DATA HIGH  Reg CH4 (Read/Write)  */
#define HYSTERESIS_REG_CH4	0xF	/* Hysteresis Reg CH4 (Read/Write)  */

#define VIN1	0x8	/* Voltage on pin */
#define VIN2	0x9	/* Voltage on pin */
#define VIN3	0xA	/* Voltage on pin */
#define VIN4	0xB	/* Voltage on pin */
#define VIN5	0xC	/* Voltage on pin */
#define VIN6	0xD	/* Voltage on pin */
#define VIN7	0xE	/* Voltage on pin */
#define VIN8	0xF	/* Voltage on pin */


#define READ_RESULT_VIN(channel)	((channel<<4) + CONVERSION_RESULT_REG)

class CurrentSensor : public device::I2C
{
public:
	CurrentSensor(int bus, int address = I2C_ADDRESS_AD7998);
	virtual ~CurrentSensor();

	virtual int 		init();

	//virtual ssize_t		read(struct file *filp, char *buffer, size_t buflen);
	//virtual int			ioctl(struct file *filp, int cmd, unsigned long arg);

	/**
	* Diagnostics - print some basic information about the driver.
	*/
	void				print_info();


	/**
	* Perform a poll cycle; collect from the previous measurement
	* and start a new one.
	*/
	void	cycle();
	int	measure();
	int	collect();

	//XXX change
protected:

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
extern "C" __EXPORT int current_sensor_main(int argc, char *argv[]);

CurrentSensor::CurrentSensor(int bus, int address) :
	I2C("MB12xx", CURRENT_SENSOR_DEVICE_PATH, bus, address, 100000)
{

}



CurrentSensor::~CurrentSensor()
{

	//XXX
}




int
CurrentSensor::init()
{
	int ret = -1;

	/* do I2C init (and probe) first */
	if (I2C::init() != OK)
		goto out;


	ret = OK;
out:
	return ret;
}






int
CurrentSensor::measure()
{
	int ret;
	uint8_t readout_array [2] = {0, 0};
	int  readout = 0;
	/*
	 * Send the command to begin a measurement.
	 */
	uint8_t cmd = READ_RESULT_VIN(VIN1);	//read Vin1
	warnx("\nRead now....");
	ret = transfer(&cmd, 1, readout_array, 2);

	if (OK != ret) {
		//perf_count(_comms_errors);
		warnx("\nRead failed (transfer)");
		return ret;
	}


	readout = (int) (readout_array[1]>>8) + (int) (readout_array[0]);

	ret = OK;
	warnx("\ncurrent: %d ", readout);

	return readout;
}

int
CurrentSensor::collect()
{

	int ret = OK;

	return ret;
}

void
CurrentSensor::cycle()
{
	int ret = OK;

}


void
CurrentSensor::print_info()
{

}

/**
 * Local functions in support of the shell command.
 */
namespace current_sensor
{

/* oddly, ERROR is not defined for c++ */
#ifdef ERROR
# undef ERROR
#endif
const int ERROR = -1;

CurrentSensor	*g_dev = nullptr;

void	start(int i2c_bus);
void	stop();
void	test();
void	reset();
void	info();

/**
 * Start the driver.
 */
void
start(int i2c_bus)
{
	int fd;

	if (g_dev != nullptr)
		errx(1, "already started");

	/* create the driver */
	g_dev = new CurrentSensor(i2c_bus, I2C_ADDRESS_AD7998);

	/* check if the AD7998 was instantiated */
	if (g_dev == nullptr)
		goto fail;


	/* set the poll rate to default, starts automatic data collection */
	//fd = open(CURRENT_SENSOR_DEVICE_PATH, O_RDONLY);

	//if (fd < 0)
		//goto fail;

	//if (ioctl(fd, SENSORIOCSPOLLRATE, SENSOR_POLLRATE_DEFAULT) < 0)
		//goto fail;

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
void
stop()
{
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
void
test()
{
	// struct differential_pressure_s report;
	// ssize_t sz;
	int ret;


	warnx("single read");
	//warnx("current: %d ", report.differential_pressure_pa);
	if (g_dev == nullptr){
		errx(1, "driver not running");
		exit(0);
	}


	g_dev->measure();
	exit(0);

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

	//if (ioctl(fd, SENSORIOCRESET, 0) < 0)
		//err(1, "driver reset failed");

	//if (ioctl(fd, SENSORIOCSPOLLRATE, SENSOR_POLLRATE_DEFAULT) < 0)
		//err(1, "driver poll restart failed");

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


static void
current_sensor_usage()
{
	warnx("usage: current_sensor command [options]");
	warnx("options:");
	warnx("\t-b --bus i2cbus (%d)", CURRENT_SENSOR_BUS);
	warnx("command:");
	warnx("\tstart|stop|reset|test|info");
}

int
current_sensor_main(int argc, char *argv[])
{
	int i2c_bus = CURRENT_SENSOR_BUS;

	int i;

	for (i = 1; i < argc; i++) {
		if (strcmp(argv[i], "-b") == 0 || strcmp(argv[i], "--bus") == 0) {
			if (argc > i + 1) {
				i2c_bus = atoi(argv[i + 1]);
			}
		}
	}

	/*
	 * Start/load the driver.
	 */
	if (!strcmp(argv[1], "start"))
		current_sensor::start(i2c_bus);

	/*
	 * Stop the driver
	 */
	if (!strcmp(argv[1], "stop"))
		current_sensor::stop();

	/*
	 * Test the driver/device.
	 */
	if (!strcmp(argv[1], "test"))
		current_sensor::test();

	/*
	 * Reset the driver.
	 */
	if (!strcmp(argv[1], "reset"))
		current_sensor::reset();

	/*
	 * Print driver information.
	 */
	if (!strcmp(argv[1], "info") || !strcmp(argv[1], "status"))
		current_sensor::info();

	current_sensor_usage();
	exit(0);
}
