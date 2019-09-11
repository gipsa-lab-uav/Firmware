#include <px4_config.h>

#include <sys/types.h>
#include <stdint.h>
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
#include <px4_getopt.h>
#include <px4_log.h>

#include <nuttx/arch.h>
#include <nuttx/wqueue.h>
#include <nuttx/clock.h>

#include <arch/board/board.h>
#include <board_config.h>

#include <lib/cdev/CDev.hpp>
#include <drivers/drv_baro.h>
#include <drivers/drv_hrt.h>
#include <drivers/device/ringbuffer.h>

#include <perf/perf_counter.h>
#include <systemlib/err.h>

#ifndef CONFIG_SCHED_WORKQUEUE
# error This requires CONFIG_SCHED_WORKQUEUE.
#endif

class FakeBaro : public cdev::CDev
{
public:
	FakeBaro(const char *path);
	virtual ~FakeBaro();

	virtual int	init();
	
	virtual ssize_t	read(struct file *filp, char *buffer, size_t buflen);
	
	/**
	 * Diagnostics - print some basic information about the driver.
	 */
	void print_info();

private:
	bool _running;
	
	struct work_s		_work;
	unsigned			_period_ticks; //ticks period

	ringbuffer::RingBuffer	*_reports;
	
	sensor_baro_s _brp;
	
	orb_advert_t		_baro_topic;
	int					_orb_class_instance;
	int					_class_instance;
		
	/* periodic execution helpers */
	void			start_cycle();
	void			stop_cycle();
	void			cycle(); //main execution
	static void		cycle_trampoline(void *arg);
};

/*
 * Driver 'main' command.
 */
extern "C" __EXPORT int fakebaro_main(int argc, char *argv[]);

FakeBaro::FakeBaro(const char *path):
	CDev(path),
	_running(false),
	_reports(nullptr),
	_baro_topic(nullptr),
	_orb_class_instance(-1),
	_class_instance(-1)
{
	// work_cancel in stop_cycle called from the dtor will explode if we don't do this...
	memset(&_work, 0, sizeof(_work));
}

FakeBaro::~FakeBaro()
{
		/* make sure we are truly inactive */
	stop_cycle();

	if (_class_instance != -1) {
		unregister_class_devname(BARO_BASE_DEVICE_PATH, _class_instance);
	}

	/* free any existing reports */
	if (_reports != nullptr) {
		delete _reports;
	}

	if (_baro_topic != nullptr) {
		orb_unadvertise(_baro_topic);
	}
}

int FakeBaro::init()
{
	int ret = CDev::init();
	
  /* allocate basic report buffers */
	_reports = new ringbuffer::RingBuffer(2, sizeof(sensor_baro_s));

	if (_reports == nullptr) {
		PX4_ERR("can't get memory for reports");
		ret = -ENOMEM;
		return ret;
	}
	
  /* register alternate interfaces if we have to */
	_class_instance = register_class_devname(BARO_BASE_DEVICE_PATH);
	
	_brp.error_count = 0;
	_brp.device_id = 0;
	_brp.pressure = 1013;
	_brp.temperature = 25;
	
	_baro_topic = orb_advertise_multi(ORB_ID(sensor_baro), &_brp,
					  &_orb_class_instance, ORB_PRIO_HIGH);
	
	if (_baro_topic == nullptr) {
		PX4_WARN("failed to create sensor_baro publication");
		return -ENOMEM;
	}

	_period_ticks = USEC2TICK(50000);

	start_cycle();
	
	return OK;
}

ssize_t FakeBaro::read(struct file *filp, char *buffer, size_t buflen)
{
	unsigned count = buflen / sizeof(sensor_baro_s);
	sensor_baro_s *brp = reinterpret_cast<sensor_baro_s *>(buffer);
	int ret = 0;

	/* buffer must be large enough */
	if (count < 1) {
		return -ENOSPC;
	}

	_reports->flush();
	
	if (_reports->get(brp)) { //get new generated report
		ret = sizeof(*brp);
	}

	return ret;
}

void FakeBaro::start_cycle()
{
	_running = true;
	_reports->flush();
	/* schedule a cycle to start things */
	work_queue(HPWORK, &_work, (worker_t)&FakeBaro::cycle_trampoline, this, 1);
}

void FakeBaro::stop_cycle()
{
	_running = false;
	work_cancel(HPWORK, &_work);
}

void FakeBaro::cycle()
{
	_brp.timestamp = hrt_absolute_time();
	
	/* publish it */
	
	_brp.pressure = 1013 + 1e-3*(_brp.timestamp&0xff);

	orb_publish(ORB_ID(sensor_baro), _baro_topic, &_brp);

	_reports->force(&_brp);
	
	if (_running) {
		work_queue(HPWORK, &_work, (worker_t)&FakeBaro::cycle_trampoline, this, _period_ticks);
	}
}

void	FakeBaro::cycle_trampoline(void *arg)
{
	FakeBaro *dev = reinterpret_cast<FakeBaro *>(arg);
	dev->cycle();
}

void
FakeBaro::print_info()
{
	_reports->print_info("report queue");
	
	sensor_baro_s brp = {};
	_reports->get(&brp);
	print_message(brp);
}

namespace fakebaro
{
FakeBaro * _fakebaro_dev = NULL;

void	start();
void	test();
void	reset();
void	info();
void	usage();

void start()
{
	bool started = false;

	if(_fakebaro_dev == NULL)
	{
		_fakebaro_dev = new FakeBaro("/dev/fakebaro");
	}
	
	if(_fakebaro_dev == nullptr)
	{
		started = false;
	}else{
		if(OK!= _fakebaro_dev->init())
		{
			started = false;
			delete _fakebaro_dev;
			_fakebaro_dev = nullptr;
		}else
		{
			started = true;
		}
	}

	if (!started) {
		PX4_ERR("driver start failed");
		exit(1);
	}

	// one or more drivers started OK
	exit(0);
}
	
void test()
{
	PX4_ERR("Not implemented");
	exit(0);
}

void reset()
{
}

void info()
{
  if(_fakebaro_dev!=nullptr)
	{
		_fakebaro_dev->print_info();
	}
	exit(0);
}

void usage()
{
		PX4_WARN("missing command: try 'start', 'info', 'test', 'reset'");
}
} // namespace

int fakebaro_main(int argc, char *argv[])
{
	int myoptind = 1;
	int ch;
	const char *myoptarg = nullptr;
	
	while ((ch = px4_getopt(argc, argv, "", &myoptind, &myoptarg)) != EOF) {
		switch (ch) {
		default:
			fakebaro::usage();
			return 0;
		}
	}
	
	if (myoptind >= argc) {
		fakebaro::usage();
		return -1;
	}
	
	const char *verb = argv[myoptind];

	/*
	 * Start/load the driver.
	 */
	if (!strcmp(verb, "start")) {
		fakebaro::start();
	}

	/*
	 * Test the driver/device.
	 */
	if (!strcmp(verb, "test")) {
		fakebaro::test();
	}

	/*
	 * Reset the driver.
	 */
	if (!strcmp(verb, "reset")) {
		fakebaro::reset();
	}

	/*
	 * Print driver information.
	 */
	if (!strcmp(verb, "info")) {
		fakebaro::info();
	}

	PX4_ERR("unrecognized command, try 'start', 'test', 'reset' or 'info'");
	return -1;
}
