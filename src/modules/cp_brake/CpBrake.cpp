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

#include "CpBrake.hpp"

#include <cstddef>
#include <cstdint>

#include <parameters/param.h>

#include "InputCheck.hpp"
#include "ReasonFlags.hpp"

// the library constants must equal the message constants
static_assert(cp_brake::kReasonInvalidConfig == cp_brake_constraints_s::REASON_INVALID_CONFIG, "reason flag");
static_assert(cp_brake::kReasonFrameInvalid == cp_brake_constraints_s::REASON_FRAME_INVALID, "reason flag");
static_assert(cp_brake::kReasonDataInvalid == cp_brake_constraints_s::REASON_DATA_INVALID, "reason flag");
static_assert(cp_brake::kSectorCount == 72, "sector count of obstacle_distance and cp_brake_constraints");

CpBrake::CpBrake() :
	ModuleParams(nullptr),
	WorkItem(MODULE_NAME, px4::wq_configurations::lp_default)
{
}

bool CpBrake::init()
{
	if (!_obstacle_sub.registerCallback()) {
		PX4_ERR("callback registration failed");
		return false;
	}

	updateConfig();
	return true;
}

void CpBrake::updateConfig()
{
	updateParams();

	// a second obstacle_distance publisher (the Gazebo bridge) makes the source ambiguous
	bool second_source = false;
	const param_t handle = param_find("SIM_GZ_EN_LIDAR");

	if (handle != PARAM_INVALID) {
		int32_t enabled = 0;
		second_source = (param_get(handle, &enabled) != PX4_OK) || (enabled != 0);
	}

	_config.stopping = cp_brake::StoppingParams{_param_cpb_a_brk.get(), _param_cpb_jerk.get(), _param_cpb_t_lat.get(),
			_param_cpb_margin.get(), _param_cpb_max_v.get()};
	_config.limiter = cp_brake::LimiterParams{_param_cpb_clr_t.get(), _param_cpb_rise.get()};
	_config.sample_age_stop_s = _param_cpb_age_stop.get();
	_config.constraint_age_limit_s = _param_cpb_cstr_age.get();
	_config.stock_cp_dist_m = _param_cp_dist.get();
	_config.mpc_xy_vel_max_m_s = _param_mpc_xy_vel_max.get();
	_config.second_obstacle_source = second_source;

	_config_faults = (_param_cpb_en.get() == 1) ? cp_brake::checkConfig(_config) : 0U;

	// what status reports before an input arrives; publish() recomputes both for every input
	_state = (_param_cpb_en.get() == 1) ? cp_brake_constraints_s::STATE_NOT_READY
		 : cp_brake_constraints_s::STATE_DISABLED;
	_reason_flags = (_config_faults != 0U) ? cp_brake::kReasonInvalidConfig : 0U;

	// a changed limiter parameter starts the limiter again from zero (fail closed)
	_limiter = cp_brake::CapLimiter(_config.limiter);
}

void CpBrake::Run()
{
	if (should_exit()) {
		_obstacle_sub.unregisterCallback();
		exit_and_cleanup();
		return;
	}

	if (_parameter_update_sub.updated()) {
		parameter_update_s param_update;
		_parameter_update_sub.copy(&param_update);
		updateConfig();
	}

	obstacle_distance_s input;

	while (_obstacle_sub.update(&input)) {
		process(input);
	}
}

void CpBrake::process(const obstacle_distance_s &input)
{
	++_input_count;

	const cp_brake::ObstacleInput view{input.frame, input.increment, input.angle_offset, input.min_distance,
					   input.max_distance, input.distances, cp_brake::kInputSectorCount};
	const uint32_t input_flags = cp_brake::checkObstacleInput(view);
	const bool input_ok = (input_flags == 0U);

	if (!input_ok) {
		++_input_rejected_count;
	}

	if (input_ok && _config_faults == 0U && _param_cpb_en.get() == 1) {
		for (std::size_t i = 0; i < cp_brake::kSectorCount; ++i) {
			const uint16_t wire = input.distances[i];
			_raw_caps[i] = cp_brake::sectorSpeedCap(wire, input.min_distance, input.max_distance, 0.f, _config.stopping);
			_sector_valid[i] = (wire != UINT16_MAX);
		}

		// a gap longer than the sample age limit gave no supporting data: no step, so the limiter
		// only tightens and its dwell starts again
		const float dt_s = cp_brake::sampleStep(_last_sample_timestamp, input.timestamp,
							_config.sample_age_stop_s);

		_limiter.update(_raw_caps, _sector_valid, dt_s);
	}

	if (input_ok) {
		_last_sample_timestamp = input.timestamp;
	}

	publish(input, input_flags, input_ok);
}

void CpBrake::publish(const obstacle_distance_s &input, uint32_t input_flags, bool input_ok)
{
	const hrt_abstime now = hrt_absolute_time();
	const bool enabled = (_param_cpb_en.get() == 1);
	const bool usable = enabled && (_config_faults == 0U) && input_ok;

	_reason_flags = input_flags | ((_config_faults != 0U) ? cp_brake::kReasonInvalidConfig : 0U);

	// provisional states of this step; the state machine is a separate step
	if (!enabled) {
		_state = cp_brake_constraints_s::STATE_DISABLED;

	} else if (!usable) {
		_state = cp_brake_constraints_s::STATE_NOT_READY;

	} else {
		_state = cp_brake_constraints_s::STATE_ACTIVE_CLEAR;
	}

	cp_brake_constraints_s constraints{};
	constraints.timestamp = now;
	constraints.sample_timestamp = input.timestamp;
	constraints.heading_rad = 0.f;
	constraints.heading_valid = false; // no heading history yet
	constraints.enforcement_active = false; // no hook yet
	constraints.valid = usable;
	constraints.state = _state;
	constraints.reason_flags = _reason_flags;

	for (std::size_t i = 0; i < cp_brake::kSectorCount; ++i) {
		constraints.caps_m_s[i] = usable ? _limiter.caps()[i] : 0.f;
	}

	_constraints_pub.publish(constraints);
	_last_constraint_timestamp = now;

	cp_brake_status_s status{};
	status.timestamp = now;
	status.sample_timestamp = _last_sample_timestamp;
	status.constraint_timestamp = _last_constraint_timestamp;
	// age of the last sample, saturated to the 32 bit field
	uint64_t sample_age_us = 0U;

	if (_last_sample_timestamp != 0U && now > _last_sample_timestamp) {
		sample_age_us = now - _last_sample_timestamp;
	}

	status.sample_age_us = (sample_age_us > UINT32_MAX) ? UINT32_MAX : static_cast<uint32_t>(sample_age_us);
	status.constraint_age_us = 0U;
	status.state = _state;
	status.reason_flags = _reason_flags;
	status.config_fault_flags = _config_faults;
	status.input_count = _input_count;
	status.input_rejected_count = _input_rejected_count;
	_status_pub.publish(status);
}

int CpBrake::task_spawn(int argc, char *argv[])
{
	CpBrake *instance = new CpBrake();

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

int CpBrake::print_status()
{
	PX4_INFO("enabled: %d, state: %u, reason flags: 0x%x, config faults: 0x%x",
		 static_cast<int>(_param_cpb_en.get()), static_cast<unsigned>(_state),
		 static_cast<unsigned>(_reason_flags), static_cast<unsigned>(_config_faults));
	PX4_INFO("inputs: %u, rejected: %u", static_cast<unsigned>(_input_count),
		 static_cast<unsigned>(_input_rejected_count));
	return 0;
}

int CpBrake::custom_command(int argc, char *argv[])
{
	return print_usage("unknown command");
}

int CpBrake::print_usage(const char *reason)
{
	if (reason) {
		PX4_WARN("%s\n", reason);
	}

	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description
Per-sector speed caps of the collision prevention (collision-prevention ICD-CP-04). Reads
obstacle_distance, checks the configuration and the input, applies the stopping model and the dwell
and rise limiter, and publishes cp_brake_constraints and cp_brake_status. It does not enforce the
caps: the limiter hook is separate. Started at boot when CPB_EN is 1.

)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("cp_brake", "controller");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();

	return 0;
}

extern "C" __EXPORT int cp_brake_main(int argc, char *argv[])
{
	return CpBrake::main(argc, argv);
}
