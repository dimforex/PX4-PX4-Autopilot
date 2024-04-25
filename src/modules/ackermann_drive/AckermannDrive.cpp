/****************************************************************************
 *
 *   Copyright (c) 2024 PX4 Development Team. All rights reserved.
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

#include "AckermannDrive.hpp"

using namespace time_literals;
using namespace matrix;

AckermannDrive::AckermannDrive() :
	ModuleParams(nullptr),
	ScheduledWorkItem(MODULE_NAME, px4::wq_configurations::rate_ctrl)
{
	updateParams();
}

bool AckermannDrive::init()
{
	ScheduleOnInterval(10_ms); // 100 Hz
	return true;
}

void AckermannDrive::updateParams()
{
	ModuleParams::updateParams();
}

void AckermannDrive::Run()
{
	if (should_exit()) {
		ScheduleClear();
		exit_and_cleanup();
	}

	// uORB subscriber updates
	if (_parameter_update_sub.updated()) {
		updateParams();
	}

	_vehicle_status_sub.update(&_vehicle_status);

	// Navigation modes
	switch (_vehicle_status.nav_state) {
	case vehicle_status_s::NAVIGATION_STATE_MANUAL: // Manual mode
		if (_manual_control_setpoint_sub.updated()) {
			ackermann_drive_setpoint_s _ackermann_drive_setpoint{};
			manual_control_setpoint_s manual_control_setpoint{};

			if (_manual_control_setpoint_sub.copy(&manual_control_setpoint)) {
				_ackermann_drive_setpoint.timestamp = hrt_absolute_time();
				_ackermann_drive_setpoint.speed = manual_control_setpoint.throttle;
				_ackermann_drive_setpoint.steering = manual_control_setpoint.roll;
				_ackermann_drive_setpoint.manual = true;
				_ackermann_drive_setpoint_pub.publish(_ackermann_drive_setpoint);
			}
		}

		break;

	case vehicle_status_s::NAVIGATION_STATE_AUTO_MISSION: // Mission mode
		_ackermann_drive_guidance.purePursuit();
		break;

	default: // Unimplemented nav states will stop the rover
		ackermann_drive_setpoint_s _ackermann_drive_setpoint{};
		_ackermann_drive_setpoint.timestamp = hrt_absolute_time();
		_ackermann_drive_setpoint.speed = 0.f;
		_ackermann_drive_setpoint.steering = 0.f;
		_ackermann_drive_setpoint.manual = false;
		_ackermann_drive_setpoint_pub.publish(_ackermann_drive_setpoint);
		break;
	}

	_ackermann_drive_control.actuatorControl();
}

int AckermannDrive::task_spawn(int argc, char *argv[])
{
	AckermannDrive *instance = new AckermannDrive();

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

int AckermannDrive::custom_command(int argc, char *argv[])
{
	return print_usage("unknown command");
}

int AckermannDrive::print_usage(const char *reason)
{
	if (reason) {
		PX4_ERR("%s\n", reason);
	}

	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description
Rover state machine.
)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("ackermann_drive", "controller");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();
	return 0;
}

extern "C" __EXPORT int ackermann_drive_main(int argc, char *argv[])
{
	return AckermannDrive::main(argc, argv);
}
