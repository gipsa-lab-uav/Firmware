#pragma once

#include <px4_platform_common/defines.h>
#include <px4_platform_common/module.h>
#include <px4_platform_common/module_params.h>
#include <px4_platform_common/posix.h>
#include <px4_platform_common/px4_work_queue/ScheduledWorkItem.hpp>
#include <drivers/drv_sensor.h>
#include <lib/drivers/magnetometer/PX4Magnetometer.hpp>
#include <uORB/Subscription.hpp>
#include <uORB/topics/vehicle_odometry.h>

class MocapMag : public ModuleBase<MocapMag>, public ModuleParams, public px4::ScheduledWorkItem
{
public:
	MocapMag();
	~MocapMag() override = default;

	/** @see ModuleBase */
	static int task_spawn(int argc, char *argv[]);

	/** @see ModuleBase */
	static int custom_command(int argc, char *argv[]);

	/** @see ModuleBase */
	static int print_usage(const char *reason = nullptr);

	bool init();

private:
	void Run() override;
	bool _mag_available{false};
	PX4Magnetometer _px4_mag;
	uORB::Subscription _vehicle_odometry_sub{ORB_ID(vehicle_visual_odometry)};
	matrix::Vector3f field;
};
