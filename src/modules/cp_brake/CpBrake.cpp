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
static_assert(cp_brake::kReasonSampleStale == cp_brake_constraints_s::REASON_SAMPLE_STALE, "reason flag");
static_assert(cp_brake::kReasonNoHeading == cp_brake_constraints_s::REASON_NO_HEADING, "reason flag");
static_assert(cp_brake::kReasonUnqualified == cp_brake_constraints_s::REASON_UNQUALIFIED, "reason flag");
static_assert(cp_brake::kReasonBlocked == cp_brake_constraints_s::REASON_BLOCKED, "reason flag");
static_assert(cp_brake::kReasonAckRequired == cp_brake_constraints_s::REASON_ACK_REQUIRED, "reason flag");
static_assert(cp_brake::kReasonOverride == cp_brake_constraints_s::REASON_OVERRIDE, "reason flag");
static_assert(cp_brake::kStateDisabled == cp_brake_constraints_s::STATE_DISABLED, "state");
static_assert(cp_brake::kStateObserverOnly == cp_brake_constraints_s::STATE_OBSERVER_ONLY, "state");
static_assert(cp_brake::kStateNotReady == cp_brake_constraints_s::STATE_NOT_READY, "state");
static_assert(cp_brake::kStateActiveClear == cp_brake_constraints_s::STATE_ACTIVE_CLEAR, "state");
static_assert(cp_brake::kStateBraking == cp_brake_constraints_s::STATE_BRAKING, "state");
static_assert(cp_brake::kStateSafeStop == cp_brake_constraints_s::STATE_SAFE_STOP, "state");
static_assert(cp_brake::kTakeoffStateFlight == takeoff_status_s::TAKEOFF_STATE_FLIGHT, "takeoff state");
static_assert(cp_brake::kSectorCount == 72, "sector count of obstacle_distance and cp_brake_constraints");

CpBrake::CpBrake() :
	ModuleParams(nullptr),
	ScheduledWorkItem(MODULE_NAME, px4::wq_configurations::lp_default)
{
}

bool CpBrake::init()
{
	if (!_obstacle_sub.registerCallback()) {
		PX4_ERR("callback registration failed");
		return false;
	}

	updateConfig();

	// a stopped obstacle_distance sends nothing, so its age is only noticed by a periodic evaluation
	ScheduleOnInterval(kEvaluationIntervalUs);
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

	// a changed limiter parameter starts the limiter again from zero (fail closed)
	_limiter = cp_brake::CapLimiter(_config.limiter);
}

void CpBrake::Run()
{
	if (should_exit()) {
		ScheduleClear();
		_obstacle_sub.unregisterCallback();
		exit_and_cleanup();
		return;
	}

	if (_parameter_update_sub.updated()) {
		parameter_update_s param_update;
		_parameter_update_sub.copy(&param_update);
		updateConfig();
	}

	takeoff_status_s takeoff_status;

	if (_takeoff_status_sub.update(&takeoff_status)) {
		_takeoff_state = takeoff_status.takeoff_state;
	}

	vehicle_local_position_s local_position;

	if (_local_position_sub.update(&local_position)) {
		_px4_state_valid = local_position.xy_valid && local_position.v_xy_valid
				   && local_position.heading_good_for_control;
	}

	obstacle_distance_s input;

	while (_obstacle_sub.update(&input)) {
		process(input);
	}

	// every run, whatever woke it: a new input or the evaluation interval
	evaluate();
}

void CpBrake::process(const obstacle_distance_s &input)
{
	++_input_count;

	const cp_brake::ObstacleInput view{input.frame, input.increment, input.angle_offset, input.min_distance,
					   input.max_distance, input.distances, cp_brake::kInputSectorCount};
	_input_flags = cp_brake::checkObstacleInput(view);
	const bool input_ok = (_input_flags == 0U);

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
}

void CpBrake::evaluate()
{
	const hrt_abstime now = hrt_absolute_time();
	const bool has_sample = (_last_sample_timestamp != 0U);

	// age of the latest accepted map; a map stamped after now gives a negative age and counts as stale
	float sample_age_s = 0.f;

	if (has_sample) {
		sample_age_s = static_cast<float>(static_cast<int64_t>(now) - static_cast<int64_t>(_last_sample_timestamp)) * 1e-6f;
	}

	// what the caps allow: some cap below the maximum speed limits, and no cap above zero means blocked
	bool limiting = false;
	bool any_motion = false;

	for (std::size_t i = 0; i < cp_brake::kSectorCount; ++i) {
		const float cap = _limiter.caps()[i];
		limiting = limiting || !(cap >= _config.stopping.max_speed_m_s);
		any_motion = any_motion || (cap > 0.f);
	}

	cp_brake::StateInputs inputs{};
	inputs.enabled = (_param_cpb_en.get() == 1);
	inputs.config_faults = _config_faults;
	inputs.input_flags = _input_flags;
	inputs.has_sample = has_sample;
	inputs.sample_age_s = sample_age_s;
	inputs.sample_age_stop_s = _config.sample_age_stop_s;
	inputs.heading_valid = false; // no heading history until issue #94
	inputs.px4_state_valid = _px4_state_valid;
	inputs.takeoff_state = _takeoff_state;
	inputs.limiting = limiting;
	inputs.blocked = !any_motion;
	inputs.pilot_ack = _pilot_ack;
	inputs.pilot_override = _pilot_override;
	inputs.safe_stop_latched = _safe_stop_latched;

	const cp_brake::StateResult result = cp_brake::evaluateState(inputs);

	_pilot_ack = false; // an acknowledgement is an event of one evaluation
	_safe_stop_latched = result.safe_stop_latched;
	_state = result.state;
	_reason_flags = result.reason_flags;

	cp_brake_constraints_s constraints{};
	constraints.timestamp = now;
	constraints.sample_timestamp = _last_sample_timestamp;
	constraints.heading_rad = 0.f;
	constraints.heading_valid = false; // no heading history yet
	constraints.enforcement_active = false; // no hook yet
	constraints.valid = result.constraint_valid;
	constraints.state = _state;
	constraints.reason_flags = _reason_flags;

	for (std::size_t i = 0; i < cp_brake::kSectorCount; ++i) {
		constraints.caps_m_s[i] = result.constraint_valid ? _limiter.caps()[i] : 0.f;
	}

	_constraints_pub.publish(constraints);
	_last_constraint_timestamp = now;

	cp_brake_status_s status{};
	status.timestamp = now;
	status.sample_timestamp = _last_sample_timestamp;
	status.constraint_timestamp = _last_constraint_timestamp;
	// age of the last sample, saturated to the 32 bit field
	uint64_t sample_age_us = 0U;

	if (has_sample && now > _last_sample_timestamp) {
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

	const hrt_abstime now = hrt_absolute_time();
	const double sample_age_s = (_last_sample_timestamp != 0U && now > _last_sample_timestamp)
				    ? static_cast<double>(now - _last_sample_timestamp) * 1e-6 : 0.0;
	PX4_INFO("takeoff state: %u, px4 state valid: %d, safe stop latched: %d, sample age: %.3f s",
		 static_cast<unsigned>(_takeoff_state), static_cast<int>(_px4_state_valid),
		 static_cast<int>(_safe_stop_latched), sample_age_s);
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
Per-sector speed caps and operational state of the collision prevention (collision-prevention
ICD-CP-04). Reads obstacle_distance, checks the configuration and the input, applies the stopping
model and the dwell and rise limiter, runs the state machine (DISABLED, OBSERVER_ONLY, NOT_READY,
ACTIVE_CLEAR, BRAKING, SAFE_STOP), and publishes cp_brake_constraints and cp_brake_status. The
evaluation also runs on a fixed interval, so a stopped input is noticed by its age. It does not
enforce the caps: the limiter hook is separate. It never requests a mode. Started at boot when
CPB_EN is 1.

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
