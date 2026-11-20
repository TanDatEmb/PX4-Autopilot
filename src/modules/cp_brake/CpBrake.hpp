/****************************************************************************
 *
 *   Copyright (c) 2026 collision-prevention contributors. All rights reserved.
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

/**
 * @file CpBrake.hpp
 *
 * cp_brake: turns the obstacle_distance map of the companion computer into per-sector speed caps
 * and the operational state (collision-prevention ICD-CP-04, architecture section 4). It validates
 * the configuration and the input, applies the stopping model and the dwell and rise limiter, runs
 * the state machine of lib/cp_brake/StateMachine.hpp, and publishes cp_brake_constraints and
 * cp_brake_status. The evaluation runs on a fixed interval as well as on every input, so a stopped
 * input is noticed by its age. The caps are not enforced yet: enforcement_active stays false until
 * the limiter hook exists, and heading_valid stays false until the heading history exists. The
 * module only publishes those two topics; it never requests a mode (ADR 0007).
 */

#pragma once

#include <px4_platform_common/defines.h>
#include <px4_platform_common/module.h>
#include <px4_platform_common/module_params.h>
#include <px4_platform_common/px4_work_queue/ScheduledWorkItem.hpp>

#include <drivers/drv_hrt.h>

#include <uORB/Publication.hpp>
#include <uORB/Subscription.hpp>
#include <uORB/SubscriptionCallback.hpp>
#include <uORB/topics/cp_brake_constraints.h>
#include <uORB/topics/cp_brake_status.h>
#include <uORB/topics/obstacle_distance.h>
#include <uORB/topics/parameter_update.h>
#include <uORB/topics/takeoff_status.h>
#include <uORB/topics/vehicle_local_position.h>

#include "CapLimiter.hpp"
#include "ConfigCheck.hpp"
#include "StateMachine.hpp"
#include "StoppingModel.hpp"

class CpBrake : public ModuleBase<CpBrake>, public ModuleParams, public px4::ScheduledWorkItem
{
public:
	CpBrake();
	~CpBrake() override = default;

	/** @see ModuleBase */
	static int task_spawn(int argc, char *argv[]);

	/** @see ModuleBase */
	static int custom_command(int argc, char *argv[]);

	/** @see ModuleBase */
	static int print_usage(const char *reason = nullptr);

	bool init();

	/**
	 * Stop request that also wakes the work item, so "cp_brake stop" does not wait for the next
	 * evaluation tick or input.
	 */
	void request_stop() override
	{
		ModuleBase<CpBrake>::request_stop();
		ScheduleNow();
	}

	int print_status() override;

private:
	/**
	 * Interval of the periodic evaluation [us]. It bounds how long after the age of the last map
	 * reaches CPB_AGE_STOP the state machine notices it (a stopped input sends no message). An
	 * implementation value, not an approved one: the publish rate of the constraint is OPEN-032.
	 */
	static constexpr uint32_t kEvaluationIntervalUs{50000U};

	void Run() override;

	void updateConfig();
	void process(const obstacle_distance_s &input);
	void evaluate();

	uORB::Publication<cp_brake_constraints_s> _constraints_pub{ORB_ID(cp_brake_constraints)};
	uORB::Publication<cp_brake_status_s> _status_pub{ORB_ID(cp_brake_status)};

	uORB::SubscriptionCallbackWorkItem _obstacle_sub{this, ORB_ID(obstacle_distance)};
	uORB::Subscription _parameter_update_sub{ORB_ID(parameter_update)};
	uORB::Subscription _takeoff_status_sub{ORB_ID(takeoff_status)};
	uORB::Subscription _local_position_sub{ORB_ID(vehicle_local_position)};

	cp_brake::ConfigInputs _config{};
	uint32_t _config_faults{0U};

	cp_brake::CapLimiter _limiter{cp_brake::LimiterParams{-1.f, -1.f}};
	cp_brake::SectorCaps _raw_caps{};
	cp_brake::SectorFlags _sector_valid{};

	uint64_t _last_sample_timestamp{0U};   ///< acquisition time of the latest accepted map, 0 before the first
	uint32_t _input_flags{0U};             ///< input check result of the latest obstacle_distance
	hrt_abstime _last_constraint_timestamp{0U};
	uint32_t _input_count{0U};
	uint32_t _input_rejected_count{0U};

	uint8_t _takeoff_state{0U};            ///< takeoff_status; 0 (uninitialized) until one is published
	bool _px4_state_valid{false};          ///< estimator position, velocity and heading usable for control

	// Inputs of the state machine that no command sets yet: the MAVLink command that sets the
	// acknowledgement and the override is a separate issue (OPEN-020). The acknowledgement is an
	// event: evaluate() clears it after use.
	bool _pilot_ack{false};
	bool _pilot_override{false};

	bool _safe_stop_latched{false};
	uint32_t _reason_flags{0U};
	uint8_t _state{cp_brake_constraints_s::STATE_DISABLED};

	DEFINE_PARAMETERS(
		(ParamInt<px4::params::CPB_EN>) _param_cpb_en,
		(ParamFloat<px4::params::CPB_MAX_V>) _param_cpb_max_v,
		(ParamFloat<px4::params::CPB_A_BRK>) _param_cpb_a_brk,
		(ParamFloat<px4::params::CPB_JERK>) _param_cpb_jerk,
		(ParamFloat<px4::params::CPB_T_LAT>) _param_cpb_t_lat,
		(ParamFloat<px4::params::CPB_MARGIN>) _param_cpb_margin,
		(ParamFloat<px4::params::CPB_AGE_STOP>) _param_cpb_age_stop,
		(ParamFloat<px4::params::CPB_CLR_T>) _param_cpb_clr_t,
		(ParamFloat<px4::params::CPB_RISE>) _param_cpb_rise,
		(ParamFloat<px4::params::CPB_CSTR_AGE>) _param_cpb_cstr_age,
		(ParamFloat<px4::params::CP_DIST>) _param_cp_dist,
		(ParamFloat<px4::params::MPC_XY_VEL_MAX>) _param_mpc_xy_vel_max
	)
};
