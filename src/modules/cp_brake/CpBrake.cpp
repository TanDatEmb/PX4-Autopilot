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

#include <cmath>
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
static_assert(cp_brake::kReasonTimeSync == cp_brake_constraints_s::REASON_TIME_SYNC, "reason flag");
static_assert(cp_brake::kReasonConstraintStale == cp_brake_constraints_s::REASON_CONSTRAINT_STALE, "reason flag");
static_assert(cp_brake::kReasonConstraintMalformed == cp_brake_constraints_s::REASON_CONSTRAINT_MALFORMED, "reason flag");
static_assert(cp_brake::kReasonNavState == cp_brake_constraints_s::REASON_NAV_STATE, "reason flag");
static_assert(cp_brake::kStateDisabled == cp_brake_constraints_s::STATE_DISABLED, "state");
static_assert(cp_brake::kStateObserverOnly == cp_brake_constraints_s::STATE_OBSERVER_ONLY, "state");
static_assert(cp_brake::kStateNotReady == cp_brake_constraints_s::STATE_NOT_READY, "state");
static_assert(cp_brake::kStateActiveClear == cp_brake_constraints_s::STATE_ACTIVE_CLEAR, "state");
static_assert(cp_brake::kStateBraking == cp_brake_constraints_s::STATE_BRAKING, "state");
static_assert(cp_brake::kStateSafeStop == cp_brake_constraints_s::STATE_SAFE_STOP, "state");
static_assert(cp_brake::kTakeoffStateFlight == takeoff_status_s::TAKEOFF_STATE_FLIGHT, "takeoff state");
static_assert(cp_brake::kSectorCount == 72, "sector count of obstacle_distance and cp_brake_constraints");

#define CP_BRAKE_SAME_VALUE(lib, msg) static_assert(static_cast<uint8_t>(lib) == (msg), "status constant")
CP_BRAKE_SAME_VALUE(cp_brake::HeadingLookup::kOk, cp_brake_status_s::HEADING_LOOKUP_OK);
CP_BRAKE_SAME_VALUE(cp_brake::HeadingLookup::kInvalidParams, cp_brake_status_s::HEADING_LOOKUP_INVALID_PARAMS);
CP_BRAKE_SAME_VALUE(cp_brake::HeadingLookup::kEmpty, cp_brake_status_s::HEADING_LOOKUP_EMPTY);
CP_BRAKE_SAME_VALUE(cp_brake::HeadingLookup::kBeforeFirst, cp_brake_status_s::HEADING_LOOKUP_BEFORE_FIRST);
CP_BRAKE_SAME_VALUE(cp_brake::HeadingLookup::kAfterLast, cp_brake_status_s::HEADING_LOOKUP_AFTER_LAST);
CP_BRAKE_SAME_VALUE(cp_brake::HeadingLookup::kReset, cp_brake_status_s::HEADING_LOOKUP_RESET);
CP_BRAKE_SAME_VALUE(cp_brake::HeadingLookup::kGap, cp_brake_status_s::HEADING_LOOKUP_GAP);
CP_BRAKE_SAME_VALUE(cp_brake::HeadingLookup::kNotGood, cp_brake_status_s::HEADING_LOOKUP_NOT_GOOD);
CP_BRAKE_SAME_VALUE(cp_brake::TimeSyncVerdict::kGood, cp_brake_status_s::TIME_SYNC_GOOD);
CP_BRAKE_SAME_VALUE(cp_brake::TimeSyncVerdict::kInvalidLimits, cp_brake_status_s::TIME_SYNC_INVALID_LIMITS);
CP_BRAKE_SAME_VALUE(cp_brake::TimeSyncVerdict::kNoStatus, cp_brake_status_s::TIME_SYNC_NO_STATUS);
CP_BRAKE_SAME_VALUE(cp_brake::TimeSyncVerdict::kStale, cp_brake_status_s::TIME_SYNC_STALE);
CP_BRAKE_SAME_VALUE(cp_brake::TimeSyncVerdict::kRoundTripTooLong, cp_brake_status_s::TIME_SYNC_ROUND_TRIP_TOO_LONG);
CP_BRAKE_SAME_VALUE(cp_brake::TimeSyncVerdict::kOffsetDeviation, cp_brake_status_s::TIME_SYNC_OFFSET_DEVIATION);
#undef CP_BRAKE_SAME_VALUE
static_assert(cp_brake_status_s::TIME_SYNC_NOT_JUDGED > cp_brake_status_s::TIME_SYNC_OFFSET_DEVIATION,
	      "the value of a gate that is off must not collide with a verdict");

namespace
{
// ADR 0003: hardware (NuttX) runs the uXRCE-DDS time synchronization; only simulation and replay do not
#if defined(__PX4_NUTTX)
constexpr bool kTimeSyncRequired{true};
#else
constexpr bool kTimeSyncRequired{false};
#endif
} // namespace

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

	if (!_local_position_sub.registerCallback()) {
		PX4_ERR("callback registration failed");
		_obstacle_sub.unregisterCallback();
		return false;
	}

	updateConfig();

	// a stopped obstacle_distance sends nothing, so its age is only noticed by a periodic evaluation
	ScheduleDelayed(kEvaluationIntervalUs);
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
	_config.decel_bound_m_s2 = _param_cpb_decel_max.get();

	// the history of the heading of the sample time: a changed depth or gap limit starts it again, an
	// unrelated parameter update does not (it would leave no heading for the next second)
	const cp_brake::HeadingHistoryParams history_params{_param_cpb_hdg_depth.get(), _param_cpb_hdg_gap.get()};

	if (!cp_brake::sameParams(history_params, _heading_history.params())) {
		_heading_history.reset(history_params); // not a temporary: it would not fit the NuttX stack frame
	}

	_config.heading_history = history_params;
	_config.time_sync_limits = cp_brake::TimeSyncLimits{_param_cpb_sync_rtt.get(), _param_cpb_sync_off.get(),
			_param_cpb_sync_age.get()};

	// UXRCE_DDS_SYNCT off is the simulation and replay profile (ADR 0003): there is no timesync_status
	// to judge. A missing parameter counts as on, so the gate fails closed.
	_time_sync_enabled = true;
	const param_t synct_handle = param_find("UXRCE_DDS_SYNCT");

	if (synct_handle != PARAM_INVALID) {
		int32_t synct = 1;
		_time_sync_enabled = (param_get(synct_handle, &synct) != PX4_OK) || (synct != 0);
	}

	_config.time_sync_enabled = _time_sync_enabled;
	_config.time_sync_required = kTimeSyncRequired;

	_config_faults = (_param_cpb_en.get() == 1) ? cp_brake::checkConfig(_config) : 0U;

	// a changed limiter parameter starts the limiter again from zero (fail closed)
	_limiter = cp_brake::CapLimiter(_config.limiter);
}

void CpBrake::Run()
{
	if (should_exit()) {
		ScheduleClear();
		_obstacle_sub.unregisterCallback();
		_local_position_sub.unregisterCallback();
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

	processHeadingAndTimeSync();

	obstacle_distance_s input;
	bool new_input = false;

	while (_obstacle_sub.update(&input)) {
		process(input);
		new_input = true;
	}

	// a new input, or the evaluation interval has run out; the heading samples wake the module much
	// more often than that, and only keep the history
	if (new_input || hrt_absolute_time() >= _next_evaluation) {
		evaluate();
	}
}

void CpBrake::processHeadingAndTimeSync()
{
	vehicle_local_position_s local_position;

	if (_local_position_sub.update(&local_position)) {
		_px4_state_valid = local_position.xy_valid && local_position.v_xy_valid
				   && local_position.heading_good_for_control;

		// keyed by the sample time of the estimate, not by the time it was published
		if (_heading_history.add(local_position.timestamp_sample, local_position.heading,
					 local_position.heading_reset_counter, local_position.heading_good_for_control)) {
			++_heading_sample_count;

		} else {
			++_heading_rejected_count;
		}
	}

	for (auto &subscription : _timesync_status_subs) {
		timesync_status_s status;

		if (subscription.update(&status) && status.source_protocol == timesync_status_s::SOURCE_PROTOCOL_DDS) {
			_time_sync_status = cp_brake::TimeSyncStatus{status.timestamp, status.observed_offset,
					    status.estimated_offset, status.round_trip_time};
		}
	}
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

	if (input_ok) {
		// the time alignment of this map, judged now: PX4 converted its timestamp with the offset of
		// the latest exchange. With UXRCE_DDS_SYNCT off (simulation) there is nothing to judge.
		if (_time_sync_enabled) {
			_time_sync_verdict = cp_brake::judgeTimeSync(_time_sync_status, hrt_absolute_time(),
					     _config.time_sync_limits);
			_time_sync_good = (_time_sync_verdict == cp_brake::TimeSyncVerdict::kGood);

		} else {
			_time_sync_good = true;
		}

		if (!_time_sync_good) {
			++_time_sync_rejected_count;
		}
	}

	// an unused map does not refresh the sample, so it ages and the stale rule applies; the step for
	// the limiter does not reach back over it
	float step_s = -1.f;

	if (input_ok) {
		step_s = _map_timeline.onMap(input.timestamp, _time_sync_good, _config.sample_age_stop_s);
	}

	if (input_ok && _config_faults == 0U && _param_cpb_en.get() == 1) {
		for (std::size_t i = 0; i < cp_brake::kSectorCount; ++i) {
			const uint16_t wire = input.distances[i];
			_raw_caps[i] = cp_brake::sectorSpeedCap(wire, input.min_distance, input.max_distance, 0.f, _config.stopping);
			_sector_valid[i] = (wire != UINT16_MAX);
		}

		if (!_time_sync_good) {
			// a map whose time alignment is not good is not used: every sector is unknown, never free
			_raw_caps.fill(0.f);
			_sector_valid.fill(false);
		}

		// a gap longer than the sample age limit, or an unused map before this one, gave no supporting
		// data: no step, so the limiter only tightens and its dwell starts again
		_limiter.update(_raw_caps, _sector_valid, step_s);
	}
}

void CpBrake::evaluate()
{
	const hrt_abstime now = hrt_absolute_time();
	const uint64_t sample_timestamp = _map_timeline.sampleTimestamp();
	const bool has_sample = (sample_timestamp != 0U);

	// age of the latest accepted map; a map stamped after now gives a negative age and counts as stale
	float sample_age_s = 0.f;

	if (has_sample) {
		sample_age_s = static_cast<float>(static_cast<int64_t>(now) - static_cast<int64_t>(sample_timestamp)) * 1e-6f;
	}

	// what the caps allow: some cap below the maximum speed limits, and no cap above zero means blocked
	bool limiting = false;
	bool any_motion = false;

	for (std::size_t i = 0; i < cp_brake::kSectorCount; ++i) {
		const float cap = _limiter.caps()[i];
		limiting = limiting || !(cap >= _config.stopping.max_speed_m_s);
		any_motion = any_motion || (cap > 0.f);
	}

	// the heading of the sample time, from the history; no map yet means no heading
	const cp_brake::HeadingResult heading = has_sample
			? _heading_history.at(sample_timestamp)
			: cp_brake::HeadingResult{cp_brake::HeadingLookup::kEmpty, 0.f};
	_heading_lookup = heading.lookup;

	cp_brake::StateInputs inputs{};
	inputs.enabled = (_param_cpb_en.get() == 1);
	inputs.config_faults = _config_faults;
	inputs.input_flags = _input_flags;
	inputs.has_sample = has_sample;
	inputs.sample_age_s = sample_age_s;
	inputs.sample_age_stop_s = _config.sample_age_stop_s;
	inputs.heading_valid = heading.valid();
	inputs.time_sync_good = _time_sync_good;
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
	constraints.sample_timestamp = sample_timestamp;
	constraints.heading_rad = heading.valid() ? heading.heading_rad : 0.f;
	constraints.heading_valid = heading.valid();
	// the limiter hook in mc_pos_control may act in these states only; it has the final say on
	// CPB_EN, the flight phase, the navigation state and the age of this constraint
	constraints.enforcement_active = cp_brake::isEnforcingState(_state);
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
	status.sample_timestamp = sample_timestamp;
	status.constraint_timestamp = _last_constraint_timestamp;
	// age of the last sample, saturated to the 32 bit field
	uint64_t sample_age_us = 0U;

	if (has_sample && now > sample_timestamp) {
		sample_age_us = now - sample_timestamp;
	}

	status.sample_age_us = (sample_age_us > UINT32_MAX) ? UINT32_MAX : static_cast<uint32_t>(sample_age_us);
	status.constraint_age_us = 0U;
	status.state = _state;
	status.reason_flags = _reason_flags;
	status.config_fault_flags = _config_faults;
	status.input_count = _input_count;
	status.input_rejected_count = _input_rejected_count;
	status.heading_lookup = static_cast<uint8_t>(_heading_lookup);
	status.time_sync_verdict = _time_sync_enabled ? static_cast<uint8_t>(_time_sync_verdict)
				   : cp_brake_status_s::TIME_SYNC_NOT_JUDGED;
	status.heading_history_size = static_cast<uint8_t>(_heading_history.size());
	status.heading_sample_count = _heading_sample_count;
	status.heading_rejected_count = _heading_rejected_count;
	status.time_sync_rejected_count = _time_sync_rejected_count;
	_status_pub.publish(status);

	_next_evaluation = now + kEvaluationIntervalUs;
	ScheduleDelayed(kEvaluationIntervalUs);
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
	const uint64_t sample_timestamp = _map_timeline.sampleTimestamp();
	const double sample_age_s = (sample_timestamp != 0U && now > sample_timestamp)
				    ? static_cast<double>(now - sample_timestamp) * 1e-6 : 0.0;
	PX4_INFO("takeoff state: %u, px4 state valid: %d, safe stop latched: %d, sample age: %.3f s",
		 static_cast<unsigned>(_takeoff_state), static_cast<int>(_px4_state_valid),
		 static_cast<int>(_safe_stop_latched), sample_age_s);
	PX4_INFO("heading history: %u samples (accepted %u, refused %u), depth %d, gap limit %.3f s, lookup: %u",
		 static_cast<unsigned>(_heading_history.size()), static_cast<unsigned>(_heading_sample_count),
		 static_cast<unsigned>(_heading_rejected_count), static_cast<int>(_heading_history.params().depth),
		 static_cast<double>(_heading_history.params().max_gap_s), static_cast<unsigned>(_heading_lookup));
	PX4_INFO("time sync gate: %s, verdict: %u, maps not used: %u, map time good: %d",
		 _time_sync_enabled ? "on" : "off (UXRCE_DDS_SYNCT=0)", static_cast<unsigned>(_time_sync_verdict),
		 static_cast<unsigned>(_time_sync_rejected_count), static_cast<int>(_time_sync_good));
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
ACTIVE_CLEAR, BRAKING, SAFE_STOP), and publishes cp_brake_constraints and cp_brake_status. It keeps
a timestamped history of the vehicle heading and interpolates it at the map timestamp, and it does
not use a map whose time alignment fails the gate on timesync_status. The evaluation also runs on a
fixed interval, so a stopped input is noticed by its age. It does not enforce the caps itself: the
limiter hook in mc_pos_control applies them to the horizontal setpoint, and acts only in the states
where enforcement_active is set. It never requests a mode. Started at boot when CPB_EN is 1.

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
