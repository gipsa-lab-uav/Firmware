px4_add_module(
	MODULE examples__mocap_mag
	MAIN mocap_mag
	COMPILE_FLAGS
	SRCS
		MocapMag.cpp
		MocapMag.hpp
	DEPENDS
		drivers_magnetometer
		px4_work_queue
)
