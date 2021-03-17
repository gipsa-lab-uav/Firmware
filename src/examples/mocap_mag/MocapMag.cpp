#include "MocapMag.hpp"


using namespace matrix;
using namespace time_literals;

MocapMag::MocapMag() :
	ModuleParams(nullptr),
	ScheduledWorkItem(MODULE_NAME, px4::wq_configurations::hp_default),
	_px4_mag(0) //_px4_mag(0, ROTATION_NONE)
{
	_px4_mag.set_device_type(DRV_MAG_DEVTYPE_MAGSIM);
	_px4_mag.set_external(false);
}

bool MocapMag::init()
{
	field = Vector3f(0.5f, 0.f, 0.f);
	ScheduleOnInterval(10_ms); // 100 Hz
	return true;
}

void MocapMag::Run()
{
	if (should_exit()) {
		ScheduleClear();
		exit_and_cleanup();
		return;
	}
	
	vehicle_odometry_s odometry;
	if(_vehicle_odometry_sub.update(&odometry)) {
		field =  Dcmf{Quatf{odometry.q}} .transpose() * Vector3f(0.5f, 0.f, 0.f);
	}
	_px4_mag.update(hrt_absolute_time(), field(0), field(1), field(2));
	
}

int MocapMag::task_spawn(int argc, char *argv[])
{
	MocapMag *instance = new MocapMag();

	if (instance) {
		_object.store(instance);
		_task_id = task_id_is_work_queue;

		if (instance->init()) {
			return PX4_OK;
		}

	} else {
		PX4_ERR("alloc failed");
	}

	delete instance;
	_object.store(nullptr);
	_task_id = -1;

	return PX4_ERROR;
}

int MocapMag::custom_command(int argc, char *argv[])
{
	return print_usage("unknown command");
}

int MocapMag::print_usage(const char *reason)
{
	if (reason) {
		PX4_WARN("%s\n", reason);
	}

	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description
Publish constant magnetometer value.
)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("mocap_mag", "driver");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();
	return 0;
}

extern "C" __EXPORT int mocap_mag_main(int argc, char *argv[])
{
	return MocapMag::main(argc, argv);
}
