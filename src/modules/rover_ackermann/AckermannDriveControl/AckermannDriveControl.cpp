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

#include "AckermannDriveControl.hpp"

using namespace time_literals;

AckermannDriveControl::AckermannDriveControl(ModuleParams *parent) : ModuleParams(parent)
{
	updateParams();
	pid_init(&_pid_speed, PID_MODE_DERIVATIV_NONE, 0.001f);
}

void AckermannDriveControl::updateParams()
{
	ModuleParams::updateParams();

	pid_set_parameters(&_pid_speed,
			   _param_ad_p_speed.get(), // Proportional gain
			   _param_ad_i_speed.get(), // Integral gain
			   0, // Derivative gain
			   2, // Integral limit
			   200); // Output limit
}

void AckermannDriveControl::actuatorControl()
{
	// uORB subscriber updates
	_ackermann_drive_setpoint_sub.update(&_ackermann_drive_setpoint);

	// Timestamps
	hrt_abstime now = hrt_absolute_time();
	const float dt = math::min((now - _time_stamp_last), 5000_ms) / 1e6f;
	_time_stamp_last = now;

	// Speed control
	if (fabsf(_ackermann_drive_setpoint.speed) < 0.01f) { // Stop
		_speed = 0.f;
		_steering = _ackermann_drive_setpoint.steering;

	} else if (_ackermann_drive_setpoint.manual) { // Manual mode
		_speed = _ackermann_drive_setpoint.speed;
		_steering = _ackermann_drive_setpoint.steering;

	} else { // Mission mode
		_speed = pid_calculate(&_pid_speed, _ackermann_drive_setpoint.speed, _ackermann_drive_setpoint.speed_actual, 0,
				       dt);
		_speed = math::constrain(_speed, 0.f, 1.f);
		_steering = math::interpolate<float>(_ackermann_drive_setpoint.steering, -_param_ad_max_steer_angle.get(),
						     _param_ad_max_steer_angle.get(),
						     -1.f, 1.f);
	}

	// Publish to wheel motors
	actuator_motors_s actuator_motors{};
	actuator_motors.reversible_flags = _param_r_rev.get();
	actuator_motors.control[0] = _speed;
	actuator_motors.control[1] = _speed;
	actuator_motors.timestamp = now;
	_actuator_motors_pub.publish(actuator_motors);

	// Publish to servo
	actuator_servos_s actuator_servos{};
	actuator_servos.control[0] = -_steering;
	actuator_servos.control[1] = -_steering;
	actuator_servos.timestamp = now;
	_actuator_servos_pub.publish(actuator_servos);
}
