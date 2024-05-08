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

#pragma once

// PX4 includes
#include <px4_platform_common/px4_config.h>
#include <px4_platform_common/defines.h>
#include <px4_platform_common/module.h>
#include <px4_platform_common/module_params.h>
#include <px4_platform_common/px4_work_queue/ScheduledWorkItem.hpp>

// uORB includes
#include <uORB/Publication.hpp>
#include <uORB/PublicationMulti.hpp>
#include <uORB/Subscription.hpp>
#include <uORB/topics/ackermann_drive_setpoint.h>
#include <uORB/topics/actuator_motors.h>
#include <uORB/topics/actuator_servos.h>
#include <uORB/topics/vehicle_local_position.h>


// Standard libraries
#include <math.h>
#include <matrix/matrix/math.hpp>
#include <matrix/math.hpp>
#include <mathlib/mathlib.h>
#include <lib/pid/pid.h>

using namespace matrix;

class AckermannDriveControl : public ModuleParams
{
public:
	/**
	 * @brief Constructor for AckermannDriveControl.
	 * @param parent The parent ModuleParams object.
	 */
	AckermannDriveControl(ModuleParams *parent);
	~AckermannDriveControl() = default;

	/**
	 * @brief Compute and publish actuator commands based on ackermann_drive_setpoint.
	 */
	void actuatorControl();

protected:
	void updateParams() override;

private:
	// uORB subscriptions
	uORB::Subscription _ackermann_drive_setpoint_sub{ORB_ID(ackermann_drive_setpoint)};
	ackermann_drive_setpoint_s _ackermann_drive_setpoint{};

	// uORB publications
	uORB::PublicationMulti<actuator_motors_s> _actuator_motors_pub{ORB_ID(actuator_motors)};
	uORB::Publication<actuator_servos_s> _actuator_servos_pub{ORB_ID(actuator_servos)};

	// Variables
	hrt_abstime _time_stamp_last{0};
	float _speed{0};
	float _steering{0};
	PID_t _pid_speed;

	// Parameters
	DEFINE_PARAMETERS(
		(ParamFloat<px4::params::AD_SPEED_P>) _param_ad_p_speed,
		(ParamFloat<px4::params::AD_SPEED_I>) _param_ad_i_speed,
		(ParamFloat<px4::params::AD_MAX_STR_ANG>) _param_ad_max_steer_angle,
		(ParamInt<px4::params::CA_R_REV>) _param_r_rev
	)
};
