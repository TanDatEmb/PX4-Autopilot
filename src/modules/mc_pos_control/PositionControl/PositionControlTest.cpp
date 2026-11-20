/****************************************************************************
 *
 *   Copyright (C) 2019 PX4 Development Team. All rights reserved.
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

#include <gtest/gtest.h>
#include <PositionControl.hpp>
#include <px4_defines.h>

#include <cmath>
#include <cstdint>
#include <cstring>

#include <lib/cp_brake/ReasonFlags.hpp>
#include <lib/cp_brake/StateMachine.hpp>

using namespace matrix;

TEST(PositionControlTest, EmptySetpoint)
{
	PositionControl position_control;

	vehicle_local_position_setpoint_s output_setpoint{};
	position_control.getLocalPositionSetpoint(output_setpoint);
	EXPECT_FLOAT_EQ(output_setpoint.x, 0.f);
	EXPECT_FLOAT_EQ(output_setpoint.y, 0.f);
	EXPECT_FLOAT_EQ(output_setpoint.z, 0.f);
	EXPECT_FLOAT_EQ(output_setpoint.yaw, 0.f);
	EXPECT_FLOAT_EQ(output_setpoint.yawspeed, 0.f);
	EXPECT_FLOAT_EQ(output_setpoint.vx, 0.f);
	EXPECT_FLOAT_EQ(output_setpoint.vy, 0.f);
	EXPECT_FLOAT_EQ(output_setpoint.vz, 0.f);
	EXPECT_EQ(Vector3f(output_setpoint.acceleration), Vector3f(0.f, 0.f, 0.f));
	EXPECT_EQ(Vector3f(output_setpoint.thrust), Vector3f(0, 0, 0));

	vehicle_attitude_setpoint_s attitude{};
	position_control.getAttitudeSetpoint(attitude);
	Eulerf euler_att(Quatf(attitude.q_d));
	EXPECT_FLOAT_EQ(euler_att.phi(), 0.f);
	EXPECT_FLOAT_EQ(euler_att.theta(), 0.f);
	EXPECT_FLOAT_EQ(euler_att.psi(), 0.f);
	EXPECT_FLOAT_EQ(attitude.yaw_sp_move_rate, 0.f);
	EXPECT_EQ(Quatf(attitude.q_d), Quatf(1.f, 0.f, 0.f, 0.f));
	EXPECT_EQ(Vector3f(attitude.thrust_body), Vector3f(0.f, 0.f, 0.f));
}

class PositionControlBasicTest : public ::testing::Test
{
public:
	PositionControlBasicTest()
	{
		_position_control.setPositionGains(Vector3f(1.f, 1.f, 1.f));
		_position_control.setVelocityGains(Vector3f(20.f, 20.f, 20.f), Vector3f(20.f, 20.f, 20.f), Vector3f(20.f, 20.f, 20.f));
		_position_control.setVelocityLimits(1.f, 1.f, 1.f);
		_position_control.setThrustLimits(0.1f, MAXIMUM_THRUST);
		_position_control.setHorizontalThrustMargin(HORIZONTAL_THRUST_MARGIN);
		_position_control.setTiltLimit(1.f);
		_position_control.setHoverThrust(.5f);
	}

	bool runController()
	{
		_position_control.setInputSetpoint(_input_setpoint);
		const bool ret = _position_control.update(.1f);
		_position_control.getLocalPositionSetpoint(_output_setpoint);
		_position_control.getAttitudeSetpoint(_attitude);
		return ret;
	}

	PositionControl _position_control;
	trajectory_setpoint_s _input_setpoint{PositionControl::empty_trajectory_setpoint};
	vehicle_local_position_setpoint_s _output_setpoint{};
	vehicle_attitude_setpoint_s _attitude{};

	static constexpr float MAXIMUM_THRUST = 0.9f;
	static constexpr float HORIZONTAL_THRUST_MARGIN = 0.3f;
};

class PositionControlBasicDirectionTest : public PositionControlBasicTest
{
public:
	void checkDirection()
	{
		Vector3f thrust(_output_setpoint.thrust);
		EXPECT_GT(thrust(0), 0.f);
		EXPECT_GT(thrust(1), 0.f);
		EXPECT_LT(thrust(2), 0.f);

		Vector3f body_z = Quatf(_attitude.q_d).dcm_z();
		EXPECT_LT(body_z(0), 0.f);
		EXPECT_LT(body_z(1), 0.f);
		EXPECT_GT(body_z(2), 0.f);
	}
};

TEST_F(PositionControlBasicDirectionTest, PositionDirection)
{
	Vector3f(.1f, .1f, -.1f).copyTo(_input_setpoint.position);
	EXPECT_TRUE(runController());
	checkDirection();
}

TEST_F(PositionControlBasicDirectionTest, VelocityDirection)
{
	Vector3f(.1f, .1f, -.1f).copyTo(_input_setpoint.velocity);
	EXPECT_TRUE(runController());
	checkDirection();
}

TEST_F(PositionControlBasicTest, TiltLimit)
{
	Vector3f(10.f, 10.f, 0.f).copyTo(_input_setpoint.position);

	EXPECT_TRUE(runController());
	Vector3f body_z = Quatf(_attitude.q_d).dcm_z();
	float angle = acosf(body_z.dot(Vector3f(0.f, 0.f, 1.f)));
	EXPECT_GT(angle, 0.f);
	EXPECT_LE(angle, 1.f);

	_position_control.setTiltLimit(0.5f);
	EXPECT_TRUE(runController());
	body_z = Quatf(_attitude.q_d).dcm_z();
	angle = acosf(body_z.dot(Vector3f(0.f, 0.f, 1.f)));
	EXPECT_GT(angle, 0.f);
	EXPECT_LE(angle, .50001f);

	_position_control.setTiltLimit(1.f);  // restore original
}

TEST_F(PositionControlBasicTest, VelocityLimit)
{
	Vector3f(10.f, 10.f, -10.f).copyTo(_input_setpoint.position);

	EXPECT_TRUE(runController());
	Vector2f velocity_xy(_output_setpoint.vx, _output_setpoint.vy);
	EXPECT_LE(velocity_xy.norm(), 1.f);
	EXPECT_LE(abs(_output_setpoint.vz), 1.f);
}

TEST_F(PositionControlBasicTest, PositionControlMaxThrustLimit)
{
	// Given a setpoint that drives the controller into vertical and horizontal saturation
	Vector3f(10.f, 10.f, -10.f).copyTo(_input_setpoint.position);

	// When you run it for one iteration
	runController();
	Vector3f thrust(_output_setpoint.thrust);

	// Then the thrust vector length is limited by the maximum
	EXPECT_FLOAT_EQ(thrust.norm(), MAXIMUM_THRUST);

	// Then the horizontal thrust is limited by its margin
	EXPECT_FLOAT_EQ(thrust(0), HORIZONTAL_THRUST_MARGIN / sqrt(2.f));
	EXPECT_FLOAT_EQ(thrust(1), HORIZONTAL_THRUST_MARGIN / sqrt(2.f));
	EXPECT_FLOAT_EQ(thrust(2),
			-sqrt(MAXIMUM_THRUST * MAXIMUM_THRUST - HORIZONTAL_THRUST_MARGIN * HORIZONTAL_THRUST_MARGIN));
	thrust.print();

	// Then the collective thrust is limited by the maximum
	EXPECT_EQ(_attitude.thrust_body[0], 0.f);
	EXPECT_EQ(_attitude.thrust_body[1], 0.f);
	EXPECT_FLOAT_EQ(_attitude.thrust_body[2], -MAXIMUM_THRUST);

	// Then the horizontal margin results in a tilt with the ratio of: horizontal margin / maximum thrust
	Eulerf euler_att(Quatf(_attitude.q_d));
	EXPECT_FLOAT_EQ(euler_att.phi(), asin((HORIZONTAL_THRUST_MARGIN / sqrt(2.f)) / MAXIMUM_THRUST));
	// TODO: add this line back once attitude setpoint generation strategy does not align body yaw with heading all the time anymore
	// EXPECT_FLOAT_EQ(_attitude.pitch_body, -asin((HORIZONTAL_THRUST_MARGIN / sqrt(2.f)) / MAXIMUM_THRUST));
}

TEST_F(PositionControlBasicTest, PositionControlMinThrustLimit)
{
	Vector3f(10.f, 0.f, 10.f).copyTo(_input_setpoint.position);

	runController();
	Vector3f thrust(_output_setpoint.thrust);
	EXPECT_FLOAT_EQ(thrust.length(), 0.1f);

	EXPECT_FLOAT_EQ(_attitude.thrust_body[2], -0.1f);
	Eulerf euler_att(Quatf(_attitude.q_d));

	EXPECT_FLOAT_EQ(euler_att.phi(), 0.f);
	EXPECT_FLOAT_EQ(euler_att.theta(), -1.f);
}

TEST_F(PositionControlBasicTest, FailsafeInput)
{
	_input_setpoint.acceleration[0] = _input_setpoint.acceleration[1] = 0.f;
	_input_setpoint.velocity[2] = .1f;

	EXPECT_TRUE(runController());
	EXPECT_FLOAT_EQ(_attitude.thrust_body[0], 0.f);
	EXPECT_FLOAT_EQ(_attitude.thrust_body[1], 0.f);
	EXPECT_LT(_output_setpoint.thrust[2], -.1f);
	EXPECT_GT(_output_setpoint.thrust[2], -.5f);
	EXPECT_GT(_attitude.thrust_body[2], -.5f);
	EXPECT_LE(_attitude.thrust_body[2], -.1f);
}

TEST_F(PositionControlBasicTest, IdleThrustInput)
{
	// High downwards acceleration to make sure there's no thrust
	Vector3f(0.f, 0.f, 100.f).copyTo(_input_setpoint.acceleration);

	EXPECT_TRUE(runController());
	EXPECT_FLOAT_EQ(_output_setpoint.thrust[0], 0.f);
	EXPECT_FLOAT_EQ(_output_setpoint.thrust[1], 0.f);
	EXPECT_FLOAT_EQ(_output_setpoint.thrust[2], -.1f); // minimum thrust
}

TEST_F(PositionControlBasicTest, InputCombinationsPosition)
{
	Vector3f(.1f, .2f, .3f).copyTo(_input_setpoint.position);

	EXPECT_TRUE(runController());
	EXPECT_FLOAT_EQ(_output_setpoint.x, .1f);
	EXPECT_FLOAT_EQ(_output_setpoint.y, .2f);
	EXPECT_FLOAT_EQ(_output_setpoint.z, .3f);
	EXPECT_FALSE(isnan(_output_setpoint.vx));
	EXPECT_FALSE(isnan(_output_setpoint.vy));
	EXPECT_FALSE(isnan(_output_setpoint.vz));
	EXPECT_FALSE(isnan(_output_setpoint.thrust[0]));
	EXPECT_FALSE(isnan(_output_setpoint.thrust[1]));
	EXPECT_FALSE(isnan(_output_setpoint.thrust[2]));
}

TEST_F(PositionControlBasicTest, InputCombinationsPositionVelocity)
{
	_input_setpoint.velocity[0] = .1f;
	_input_setpoint.velocity[1] = .2f;
	_input_setpoint.position[2] = .3f; // altitude

	EXPECT_TRUE(runController());
	EXPECT_TRUE(isnan(_output_setpoint.x));
	EXPECT_TRUE(isnan(_output_setpoint.y));
	EXPECT_FLOAT_EQ(_output_setpoint.z, .3f);
	EXPECT_FLOAT_EQ(_output_setpoint.vx, .1f);
	EXPECT_FLOAT_EQ(_output_setpoint.vy, .2f);
	EXPECT_FALSE(isnan(_output_setpoint.vz));
	EXPECT_FALSE(isnan(_output_setpoint.thrust[0]));
	EXPECT_FALSE(isnan(_output_setpoint.thrust[1]));
	EXPECT_FALSE(isnan(_output_setpoint.thrust[2]));
}

TEST_F(PositionControlBasicTest, SetpointValiditySimple)
{
	EXPECT_FALSE(runController());
	_input_setpoint.position[0] = .1f;
	EXPECT_FALSE(runController());
	_input_setpoint.position[1] = .2f;
	EXPECT_FALSE(runController());
	_input_setpoint.acceleration[2] = .3f;
	EXPECT_TRUE(runController());
}

TEST_F(PositionControlBasicTest, SetpointValidityAllCombinations)
{
	// This test runs any combination of set and unset (NAN) setpoints and checks if it gets accepted or rejected correctly
	float *const setpoint_loop_access_map[] = {&_input_setpoint.position[0], &_input_setpoint.velocity[0], &_input_setpoint.acceleration[0],
						   &_input_setpoint.position[1], &_input_setpoint.velocity[1], &_input_setpoint.acceleration[1],
						   &_input_setpoint.position[2], &_input_setpoint.velocity[2], &_input_setpoint.acceleration[2]
						  };

	for (int combination = 0; combination < 512; combination++) {
		_input_setpoint = PositionControl::empty_trajectory_setpoint;

		for (int j = 0; j < 9; j++) {
			if (combination & (1 << j)) {
				// Set arbitrary finite value, some values clearly hit the limits to check these corner case combinations
				*(setpoint_loop_access_map[j]) = static_cast<float>(combination) / static_cast<float>(j + 1);
			}
		}

		// Expect at least one setpoint per axis
		const bool has_x_setpoint = ((combination & 7) != 0);
		const bool has_y_setpoint = (((combination >> 3) & 7) != 0);
		const bool has_z_setpoint = (((combination >> 6) & 7) != 0);
		// Expect xy setpoints to come in pairs
		const bool has_xy_pairs = (combination & 7) == ((combination >> 3) & 7);
		const bool expected_result = has_x_setpoint && has_y_setpoint && has_z_setpoint && has_xy_pairs;

		EXPECT_EQ(runController(), expected_result) << "combination " << combination << std::endl
				<< "input" << std::endl
				<< "position     " << _input_setpoint.position[0] << ", "
				<< _input_setpoint.position[1] << ", " << _input_setpoint.position[2] << std::endl
				<< "velocity     " << _input_setpoint.velocity[0] << ", "
				<< _input_setpoint.velocity[1] << ", " << _input_setpoint.velocity[2] << std::endl
				<< "acceleration " << _input_setpoint.acceleration[0] << ", "
				<< _input_setpoint.acceleration[1] << ", " << _input_setpoint.acceleration[2] << std::endl
				<< "output" << std::endl
				<< "position     " << _output_setpoint.x << ", " << _output_setpoint.y << ", " << _output_setpoint.z << std::endl
				<< "velocity     " << _output_setpoint.vx << ", " << _output_setpoint.vy << ", " << _output_setpoint.vz << std::endl
				<< "acceleration " << _output_setpoint.acceleration[0] << ", "
				<< _output_setpoint.acceleration[1] << ", " << _output_setpoint.acceleration[2] << std::endl;
	}
}

TEST_F(PositionControlBasicTest, PositionSetpointInvalidState)
{
	Vector3f(.1f, .2f, .3f).copyTo(_input_setpoint.position); // Position setpoint

	// All states valid - ok
	PositionControlStates states{};
	_position_control.setState(states);
	EXPECT_TRUE(runController());

	// Position invalid
	states.position(0) = NAN;
	_position_control.setState(states);
	EXPECT_FALSE(runController());

	// Position and velocity invalid
	states.velocity(0) = NAN;
	_position_control.setState(states);
	EXPECT_FALSE(runController());

	// Velocity invalid
	states.position(0) = 0.f;
	_position_control.setState(states);
	EXPECT_FALSE(runController());

	// Velocity derivative invalid
	states.velocity(0) = 0.f;
	states.acceleration(1) = NAN;
	_position_control.setState(states);
	EXPECT_FALSE(runController());
}

TEST_F(PositionControlBasicTest, VelocitySetpointInvalidState)
{
	Vector3f(.1f, .2f, .3f).copyTo(_input_setpoint.velocity); // Velocity setpoint

	// All states valid - ok
	PositionControlStates states{};
	_position_control.setState(states);
	EXPECT_TRUE(runController());

	// Position invalid - ok
	states.position(0) = NAN;
	_position_control.setState(states);
	EXPECT_TRUE(runController());

	// Position and velocity invalid
	states.velocity(0) = NAN;
	_position_control.setState(states);
	EXPECT_FALSE(runController());

	// Velocity invalid
	states.position(0) = 0.f;
	_position_control.setState(states);
	EXPECT_FALSE(runController());

	// Velocity derivative invalid
	states.velocity(0) = 0.f;
	states.acceleration(1) = NAN;
	_position_control.setState(states);
	EXPECT_FALSE(runController());
}


TEST_F(PositionControlBasicTest, UpdateHoverThrust)
{
	// GIVEN: some hover thrust and 0 velocity change
	const float hover_thrust = 0.6f;
	_position_control.setHoverThrust(hover_thrust);

	Vector3f(0.f, 0.f, 0.f).copyTo(_input_setpoint.velocity);

	// WHEN: we run the controller
	EXPECT_TRUE(runController());

	// THEN: the output thrust equals the hover thrust
	EXPECT_EQ(_output_setpoint.thrust[2], -hover_thrust);

	// HOWEVER WHEN: we set a new hover thrust through the update function
	const float hover_thrust_new = 0.7f;
	_position_control.updateHoverThrust(hover_thrust_new);
	EXPECT_TRUE(runController());

	// THEN: the integral is updated to avoid discontinuities and
	// the output is still the same
	EXPECT_EQ(_output_setpoint.thrust[2], -hover_thrust);
}

TEST_F(PositionControlBasicTest, IntegratorWindupWithInvalidSetpoint)
{
	// GIVEN: the controller was ran with an invalid setpoint containing some valid values
	_input_setpoint.position[0] = .1f;
	_input_setpoint.position[1] = .2f;
	// all z-axis setpoints stay NAN
	EXPECT_FALSE(runController());

	// WHEN: we run the controller with a valid setpoint
	_input_setpoint = PositionControl::empty_trajectory_setpoint;
	Vector3f(0.f, 0.f, 0.f).copyTo(_input_setpoint.velocity);
	EXPECT_TRUE(runController());

	// THEN: the integral did not wind up and produce unexpected deviation
	Eulerf euler_att(Quatf(_attitude.q_d));
	EXPECT_FLOAT_EQ(euler_att.phi(), 0.f);
	EXPECT_FLOAT_EQ(euler_att.theta(), 0.f);
}

// ---------------------------------------------------------------------------------------------
// Collision prevention limiter hook (issue #65, collision-prevention ICD-CP-04, OPEN-035)
//
// The numbers here are test fixtures. In particular the 0.5 s constraint age limit is not the
// unapproved CPB_CSTR_AGE, which stays a sentinel in the module.
// ---------------------------------------------------------------------------------------------

namespace
{

constexpr uint64_t kNowUs{10000000U};
constexpr float kAgeLimitS{0.5f};
constexpr float kFreeCap{12.f};

bool sameBits(float a, float b)
{
	return std::memcmp(&a, &b, sizeof(float)) == 0;
}

} // namespace

class CollisionLimiterTest : public ::testing::Test
{
public:
	CollisionLimiterTest()
	{
		configure(_position_control, 4.f);
		configure(_baseline, 4.f);
		resetInputs();
	}

	static void configure(PositionControl &control, const float velocity_gain)
	{
		const Vector3f p(velocity_gain, velocity_gain, 8.f);
		const Vector3f i(velocity_gain > 0.f ? 0.4f : 0.f, velocity_gain > 0.f ? 0.4f : 0.f, 0.f);
		const Vector3f d(0.f, 0.f, 0.f);
		control.setPositionGains(Vector3f(0.95f, 0.95f, 1.f));
		control.setVelocityGains(p, i, d);
		control.setVelocityLimits(12.f, 3.f, 1.5f);
		control.setThrustLimits(0.1f, 0.9f);
		control.setHorizontalThrustMargin(0.3f);
		control.setTiltLimit(1.f);
		control.setHoverThrust(.5f);
	}

	void resetInputs()
	{
		_input = PositionControl::empty_trajectory_setpoint;
		_input.position[2] = -10.f; // altitude hold, so the vertical axis has a setpoint
		_input.velocity[0] = 0.f;
		_input.velocity[1] = 0.f;
		_states = PositionControlStates{};
		_states.position = Vector3f(0.f, 0.f, -10.f);
		_states.velocity = Vector3f(0.f, 0.f, 0.f);
		_states.acceleration = Vector3f(0.f, 0.f, 0.f);
		_states.yaw = 0.f;

		_context = cp_brake::HookContext{true, true, true, kNowUs, kAgeLimitS};
		_constraint = cp_brake::HookConstraint{};
		_constraint.present = true;
		_constraint.timestamp_us = kNowUs - 20000U;
		_constraint.heading_rad = 0.f;
		_constraint.heading_valid = true;

		for (std::size_t sector = 0; sector < cp_brake::kSectorCount; ++sector) {
			_constraint.caps_m_s[sector] = kFreeCap;
		}

		_constraint.enforcement_active = true;
		_constraint.valid = true;
		_constraint.state = cp_brake::kStateActiveClear;
		_constraint.reason_flags = 0U;
	}

	void setVelocity(const float north, const float east)
	{
		_input.velocity[0] = north;
		_input.velocity[1] = east;
	}

	void setFeedForward(const float north, const float east)
	{
		_input.acceleration[0] = north;
		_input.acceleration[1] = east;
	}

	// one call of update() on the controller with the hook and one on the controller without it
	void run()
	{
		for (PositionControl *control : {&_position_control, &_baseline}) {
			control->setState(_states);
			control->setInputSetpoint(_input);
		}

		_position_control.setCollisionLimiter(_context, _constraint);
		_ok = _position_control.update(DT);
		_baseline_ok = _baseline.update(DT);
		_output = vehicle_local_position_setpoint_s{};
		_baseline_output = vehicle_local_position_setpoint_s{};
		_position_control.getLocalPositionSetpoint(_output);
		_baseline.getLocalPositionSetpoint(_baseline_output);
		_attitude = vehicle_attitude_setpoint_s{};
		_baseline_attitude = vehicle_attitude_setpoint_s{};
		_position_control.getAttitudeSetpoint(_attitude);
		_baseline.getAttitudeSetpoint(_baseline_attitude);
	}

	const CollisionLimiterRecord &record() const { return _position_control.collisionLimiterRecord(); }

	void expectSameAsBaseline() const
	{
		EXPECT_EQ(_ok, _baseline_ok);
		EXPECT_TRUE(sameBits(_output.x, _baseline_output.x));
		EXPECT_TRUE(sameBits(_output.y, _baseline_output.y));
		EXPECT_TRUE(sameBits(_output.z, _baseline_output.z));
		EXPECT_TRUE(sameBits(_output.yaw, _baseline_output.yaw));
		EXPECT_TRUE(sameBits(_output.yawspeed, _baseline_output.yawspeed));
		EXPECT_TRUE(sameBits(_output.vx, _baseline_output.vx));
		EXPECT_TRUE(sameBits(_output.vy, _baseline_output.vy));
		EXPECT_TRUE(sameBits(_output.vz, _baseline_output.vz));

		for (int axis = 0; axis < 3; ++axis) {
			EXPECT_TRUE(sameBits(_output.acceleration[axis], _baseline_output.acceleration[axis])) << axis;
			EXPECT_TRUE(sameBits(_output.thrust[axis], _baseline_output.thrust[axis])) << axis;
			EXPECT_TRUE(sameBits(_attitude.thrust_body[axis], _baseline_attitude.thrust_body[axis])) << axis;
		}

		for (int i = 0; i < 4; ++i) {
			EXPECT_TRUE(sameBits(_attitude.q_d[i], _baseline_attitude.q_d[i])) << i;
		}
	}

	// the vertical axis and the yaw are never touched by the hook
	void expectVerticalAndYawAsBaseline() const
	{
		EXPECT_TRUE(sameBits(_output.z, _baseline_output.z));
		EXPECT_TRUE(sameBits(_output.vz, _baseline_output.vz));
		EXPECT_TRUE(sameBits(_output.yaw, _baseline_output.yaw));
		EXPECT_TRUE(sameBits(_output.yawspeed, _baseline_output.yawspeed));
		EXPECT_TRUE(sameBits(_output.acceleration[2], _baseline_output.acceleration[2]));
	}

	void expectSafeStop(const uint32_t reasons_that_must_be_set) const
	{
		EXPECT_TRUE(_ok);
		EXPECT_EQ(record().result.action, cp_brake::HookAction::kSafeStop);
		EXPECT_EQ(record().result.reason_flags & reasons_that_must_be_set, reasons_that_must_be_set);
		EXPECT_EQ(_output.vx, 0.f);
		EXPECT_EQ(_output.vy, 0.f);
		EXPECT_EQ(record().vel_sp_out_m_s[0], 0.f);
		EXPECT_EQ(record().vel_sp_out_m_s[1], 0.f);
		EXPECT_EQ(record().acc_sp_out_m_s2[0], 0.f);
		EXPECT_EQ(record().acc_sp_out_m_s2[1], 0.f);
		expectVerticalAndYawAsBaseline();
	}

	void expectAllFinite() const
	{
		EXPECT_TRUE(std::isfinite(_output.vx));
		EXPECT_TRUE(std::isfinite(_output.vy));

		for (int axis = 0; axis < 3; ++axis) {
			EXPECT_TRUE(std::isfinite(_output.acceleration[axis])) << axis;
			EXPECT_TRUE(std::isfinite(_output.thrust[axis])) << axis;
		}
	}

	static constexpr float DT{0.01f};

	PositionControl _position_control;
	PositionControl _baseline; // never given a hook: the unchanged controller
	trajectory_setpoint_s _input{};
	PositionControlStates _states{};
	cp_brake::HookContext _context{};
	cp_brake::HookConstraint _constraint{};
	bool _ok{false};
	bool _baseline_ok{false};
	vehicle_local_position_setpoint_s _output{};
	vehicle_local_position_setpoint_s _baseline_output{};
	vehicle_attitude_setpoint_s _attitude{};
	vehicle_attitude_setpoint_s _baseline_attitude{};
};

// zero velocity gains: the output acceleration is the feed-forward as the hook left it
class CollisionLimiterFeedForwardTest : public CollisionLimiterTest
{
public:
	CollisionLimiterFeedForwardTest()
	{
		configure(_position_control, 0.f);
		configure(_baseline, 0.f);
	}
};

TEST_F(CollisionLimiterTest, AControllerThatWasNeverGivenAHookIsTransparent)
{
	setVelocity(8.f, 2.f);
	setFeedForward(1.f, 0.5f);
	run();
	EXPECT_EQ(record().result.action, cp_brake::HookAction::kTransparent);
	expectSameAsBaseline();
}

TEST_F(CollisionLimiterTest, CpbEnOffIsBitIdenticalToTheBaselineWhateverTheConstraintSays)
{
	// the most hostile constraint: safe stop, nothing valid, caps 0, no number as heading
	_context.enabled = false;
	_constraint.state = cp_brake::kStateSafeStop;
	_constraint.valid = false;
	_constraint.heading_rad = NAN;

	for (std::size_t sector = 0; sector < cp_brake::kSectorCount; ++sector) {
		_constraint.caps_m_s[sector] = 0.f;
	}

	setVelocity(8.f, 2.f);
	setFeedForward(1.f, 0.5f);
	run();
	EXPECT_EQ(record().result.action, cp_brake::HookAction::kTransparent);
	EXPECT_EQ(record().result.reason_flags, 0U);
	expectSameAsBaseline();
	EXPECT_TRUE(_ok);
	EXPECT_FLOAT_EQ(_output.vx, 8.f);
	EXPECT_FLOAT_EQ(_output.vy, 2.f);
}

TEST_F(CollisionLimiterTest, BeforeTakeoffStateFlightTheHookIsTransparentWhetherTheConstraintIsValidOrNot)
{
	_context.in_flight = false;
	setVelocity(8.f, 0.f);
	_constraint.caps_m_s[0] = 0.f;
	run();
	EXPECT_EQ(record().result.action, cp_brake::HookAction::kTransparent);
	expectSameAsBaseline();
	EXPECT_FLOAT_EQ(_output.vx, 8.f);

	// invalid constraint, stale, no heading: still the baseline on the ground
	_constraint.valid = false;
	_constraint.heading_valid = false;
	_constraint.timestamp_us = 0U;
	run();
	EXPECT_EQ(record().result.action, cp_brake::HookAction::kTransparent);
	EXPECT_FLOAT_EQ(_output.vx, 8.f);
}

TEST_F(CollisionLimiterTest, InAnyOtherNavigationStateTheHookIsTransparentAndSaysWhy)
{
	_context.nav_state_enforced = false;
	setVelocity(8.f, 0.f);
	setFeedForward(1.f, 0.f);
	_constraint.caps_m_s[0] = 0.f;
	run();
	EXPECT_EQ(record().result.action, cp_brake::HookAction::kTransparent);
	EXPECT_EQ(record().result.reason_flags, cp_brake::kReasonNavState);
	expectSameAsBaseline();
	EXPECT_FLOAT_EQ(_output.vx, 8.f);
}

TEST_F(CollisionLimiterTest, EachCardinalCapLimitsOnlyTheComponentTowardItself)
{
	const struct {
		int sector;
		Vector2f direction;
	} cases[] = {{0, Vector2f(1.f, 0.f)}, {18, Vector2f(0.f, 1.f)}, {36, Vector2f(-1.f, 0.f)}, {54, Vector2f(0.f, -1.f)}};

	for (const auto &c : cases) {
		SCOPED_TRACE(c.sector);
		resetInputs();
		const Vector2f perpendicular(-c.direction(1), c.direction(0));
		const Vector2f commanded = c.direction * 8.f + perpendicular * 2.f;
		setVelocity(commanded(0), commanded(1));
		_constraint.caps_m_s[c.sector] = 3.f;
		run();

		const Vector2f out(_output.vx, _output.vy);
		EXPECT_TRUE(_ok);
		EXPECT_EQ(record().result.action, cp_brake::HookAction::kLimited);
		EXPECT_EQ(record().result.active_sector, c.sector);
		EXPECT_NEAR(out.dot(c.direction), 3.f, 1e-4f); // toward the sector: at its cap
		EXPECT_NEAR(out.dot(perpendicular), 2.f, 1e-4f); // along it: untouched
		expectVerticalAndYawAsBaseline();
		expectAllFinite();
	}
}

TEST_F(CollisionLimiterTest, ACapLimitsNothingWhenTheSetpointIsWithinIt)
{
	setVelocity(2.f, 1.f);
	_constraint.caps_m_s[0] = 3.f;
	run();
	EXPECT_EQ(record().result.action, cp_brake::HookAction::kClear);
	expectSameAsBaseline();
}

TEST_F(CollisionLimiterTest, TheBrakingStateLimitsLikeActiveClear)
{
	_constraint.state = cp_brake::kStateBraking;
	setVelocity(8.f, 0.f);
	_constraint.caps_m_s[0] = 3.f;
	run();
	EXPECT_EQ(record().result.action, cp_brake::HookAction::kLimited);
	EXPECT_NEAR(_output.vx, 3.f, 1e-4f);
}

TEST_F(CollisionLimiterTest, SideAndRearObstaclesDoNotReduceForwardMotion)
{
	// ADR 0008: no global minimum range. Everything from the right side through the rear to the left
	// side is blocked completely; the forward motion is the baseline.
	for (int sector = 18; sector <= 54; ++sector) {
		_constraint.caps_m_s[sector] = 0.f;
	}

	setVelocity(8.f, 0.f);
	run();
	EXPECT_EQ(record().result.action, cp_brake::HookAction::kClear);
	expectSameAsBaseline();
	EXPECT_FLOAT_EQ(_output.vx, 8.f);
}

TEST_F(CollisionLimiterTest, AFrontObstacleDoesNotReduceRetreat)
{
	for (int sector = 0; sector < 18; ++sector) {
		_constraint.caps_m_s[sector] = 0.f;
		_constraint.caps_m_s[(72 - sector) % 72] = 0.f;
	}

	setVelocity(-8.f, 0.f);
	run();
	EXPECT_EQ(record().result.action, cp_brake::HookAction::kClear);
	expectSameAsBaseline();
	EXPECT_FLOAT_EQ(_output.vx, -8.f);
}

TEST_F(CollisionLimiterTest, TheCapsAreRotatedWithTheHeadingOfTheConstraintNotTheCurrentHeading)
{
	_constraint.heading_rad = M_PI_2_F; // the map was taken facing east
	_states.yaw = 0.f;                   // the vehicle now faces north
	_constraint.caps_m_s[0] = 3.f;       // an obstacle in the sector that pointed east at the sample time

	setVelocity(0.f, 8.f);
	run();
	EXPECT_NEAR(_output.vy, 3.f, 1e-4f);
	EXPECT_NEAR(_output.vx, 0.f, 1e-4f);

	resetInputs();
	_constraint.heading_rad = M_PI_2_F;
	_states.yaw = 0.f;
	_constraint.caps_m_s[0] = 3.f;
	setVelocity(8.f, 0.f); // north is not toward that sector
	run();
	EXPECT_EQ(record().result.action, cp_brake::HookAction::kClear);
	EXPECT_FLOAT_EQ(_output.vx, 8.f);
}

TEST_F(CollisionLimiterTest, APositionOnlyMissionSetpointIsLimitedToo)
{
	// no velocity setpoint at all: the position error term is what moves the vehicle
	_input.velocity[0] = NAN;
	_input.velocity[1] = NAN;
	_input.position[0] = 30.f;
	_input.position[1] = 0.f;
	_constraint.caps_m_s[0] = 4.f;
	run();
	EXPECT_TRUE(_ok);
	EXPECT_FLOAT_EQ(_baseline_output.vx, 12.f); // the baseline would fly at the limit
	EXPECT_NEAR(_output.vx, 4.f, 1e-4f);
	EXPECT_NEAR(_output.vy, 0.f, 1e-4f);
}

TEST_F(CollisionLimiterTest, TheRecordHoldsTheSetpointsBeforeAndAfter)
{
	setVelocity(8.f, 2.f);
	setFeedForward(1.f, 0.5f);
	_states.velocity = Vector3f(1.f, -1.f, 0.f);
	_constraint.caps_m_s[0] = 3.f;
	run();
	EXPECT_FLOAT_EQ(record().vel_sp_in_m_s[0], 8.f);
	EXPECT_FLOAT_EQ(record().vel_sp_in_m_s[1], 2.f);
	EXPECT_FLOAT_EQ(record().acc_sp_in_m_s2[0], 1.f);
	EXPECT_FLOAT_EQ(record().acc_sp_in_m_s2[1], 0.5f);
	EXPECT_FLOAT_EQ(record().vel_m_s[0], 1.f);
	EXPECT_FLOAT_EQ(record().vel_m_s[1], -1.f);
	EXPECT_NEAR(record().vel_sp_out_m_s[0], 3.f, 1e-4f);
	EXPECT_NEAR(record().vel_sp_out_m_s[1], 2.f, 1e-4f);
	EXPECT_FLOAT_EQ(record().vel_sp_out_m_s[0], _output.vx);
	EXPECT_FLOAT_EQ(record().vel_sp_out_m_s[1], _output.vy);
}

TEST_F(CollisionLimiterFeedForwardTest, TheFeedForwardTowardALimitingSectorFadesOutAsTheVehicleReachesTheCap)
{
	// cap 4 m/s ahead, the trajectory asks 8 m/s and accelerates at 2 m/s^2 (and 1.5 m/s^2 sideways)
	const struct {
		float speed_toward_m_s;
		float expected_ff_m_s2;
	} cases[] = {{-2.f, 2.f}, {0.f, 2.f}, {1.f, 1.5f}, {2.f, 1.f}, {3.f, 0.5f}, {4.f, 0.f}, {6.f, 0.f}};

	for (const auto &c : cases) {
		SCOPED_TRACE(c.speed_toward_m_s);
		resetInputs();
		setVelocity(8.f, 0.f);
		setFeedForward(2.f, 1.5f);
		_constraint.caps_m_s[0] = 4.f;
		_states.velocity = Vector3f(c.speed_toward_m_s, 0.f, 0.f);
		run();
		EXPECT_TRUE(_ok);
		EXPECT_NEAR(_output.acceleration[0], c.expected_ff_m_s2, 1e-5f);
		EXPECT_NEAR(_output.acceleration[1], 1.5f, 1e-5f); // along the obstacle: untouched
	}
}

TEST_F(CollisionLimiterFeedForwardTest, AZeroCapRemovesTheFeedForwardTowardItWhateverTheVehicleDoes)
{
	for (const float speed : {-2.f, 0.f, 2.f}) {
		SCOPED_TRACE(speed);
		resetInputs();
		setVelocity(8.f, 0.f);
		setFeedForward(2.f, 0.f);
		_constraint.caps_m_s[0] = 0.f;
		_states.velocity = Vector3f(speed, 0.f, 0.f);
		run();
		EXPECT_NEAR(_output.acceleration[0], 0.f, 1e-6f);
		EXPECT_NEAR(_output.vx, 0.f, 1e-6f);
	}
}

TEST_F(CollisionLimiterFeedForwardTest, FeedForwardAwayFromTheSectorIsKept)
{
	setVelocity(8.f, 0.f);
	setFeedForward(-2.f, 0.f);
	_constraint.caps_m_s[0] = 4.f;
	run();
	EXPECT_NEAR(_output.acceleration[0], -2.f, 1e-6f);
}

TEST_F(CollisionLimiterFeedForwardTest, AVehicleVelocityThatIsNotANumberRemovesTheFeedForwardTowardTheSector)
{
	// position-only setpoint, so the velocity state is not needed by the controller
	_input.velocity[0] = NAN;
	_input.velocity[1] = NAN;
	_input.position[0] = 30.f;
	_input.position[1] = 0.f;
	setFeedForward(2.f, 0.f);
	_states.velocity = Vector3f(NAN, NAN, 0.f);
	_constraint.caps_m_s[0] = 4.f;
	run();
	EXPECT_TRUE(_ok);
	EXPECT_NEAR(_output.vx, 4.f, 1e-4f);
	EXPECT_NEAR(_output.acceleration[0], 0.f, 1e-6f);
	expectAllFinite();
}

TEST_F(CollisionLimiterTest, AnUnsetFeedForwardStaysUnsetAndNothingNotANumberReachesTheOutput)
{
	setVelocity(8.f, 0.f);
	_constraint.caps_m_s[0] = 3.f;
	run();
	EXPECT_TRUE(std::isnan(record().acc_sp_in_m_s2[0]));
	EXPECT_TRUE(std::isnan(record().acc_sp_out_m_s2[0]));
	EXPECT_TRUE(std::isnan(record().acc_sp_out_m_s2[1]));
	EXPECT_TRUE(_ok);
	expectAllFinite();
}

TEST_F(CollisionLimiterTest, ACapThatIsNotAFiniteNumberGreaterOrEqualZeroStopsTheVelocityTowardTheSector)
{
	for (const float bad : {NAN, INFINITY, -INFINITY, -1.f, -0.5f}) {
		SCOPED_TRACE(bad);
		resetInputs();
		setVelocity(8.f, 2.f);
		_constraint.caps_m_s[0] = bad;
		run();
		EXPECT_TRUE(_ok);
		EXPECT_EQ(record().result.action, cp_brake::HookAction::kLimited);
		EXPECT_NEAR(_output.vx, 0.f, 1e-3f); // stopped toward the sector
		EXPECT_NEAR(_output.vy, 2.f, 1e-3f); // the rest of the motion is kept
		expectAllFinite();
	}
}

TEST_F(CollisionLimiterTest, WhenNoCapIsUsableTheHorizontalVelocityIsZero)
{
	for (std::size_t sector = 0; sector < cp_brake::kSectorCount; ++sector) {
		_constraint.caps_m_s[sector] = NAN;
	}

	setVelocity(8.f, 2.f);
	setFeedForward(1.f, 1.f);
	run();
	EXPECT_TRUE(_ok);
	EXPECT_NEAR(_output.vx, 0.f, 1e-3f);
	EXPECT_NEAR(_output.vy, 0.f, 1e-3f);
	expectAllFinite();
}

TEST_F(CollisionLimiterTest, AVelocitySetpointThatIsNotFiniteIsLeftAlone)
{
	setVelocity(INFINITY, 0.f);
	_constraint.caps_m_s[0] = 3.f;
	run();
	EXPECT_EQ(_ok, _baseline_ok); // the controller's own rejection, unchanged
	EXPECT_TRUE(sameBits(_output.vx, _baseline_output.vx));
	EXPECT_TRUE(sameBits(_output.vy, _baseline_output.vy));
}

TEST_F(CollisionLimiterTest, RepeatedCallsInOneCycleGiveTheSameResult)
{
	// PX4 calls update() up to three times per cycle with a setpoint it set each time
	setVelocity(8.f, 2.f);
	setFeedForward(1.f, 0.5f);
	_states.velocity = Vector3f(1.f, 0.f, 0.f);
	_constraint.caps_m_s[0] = 3.f;
	_constraint.caps_m_s[1] = 5.f;

	CollisionLimiterRecord first{};
	float first_vx = 0.f;
	float first_vy = 0.f;

	for (int call = 0; call < 3; ++call) {
		SCOPED_TRACE(call);
		_position_control.setState(_states);
		_position_control.setInputSetpoint(_input);
		_position_control.setCollisionLimiter(_context, _constraint);
		EXPECT_TRUE(_position_control.update(DT));
		vehicle_local_position_setpoint_s output{};
		_position_control.getLocalPositionSetpoint(output);
		const CollisionLimiterRecord &r = _position_control.collisionLimiterRecord();

		if (call == 0) {
			first = r;
			first_vx = output.vx;
			first_vy = output.vy;
			EXPECT_EQ(r.result.action, cp_brake::HookAction::kLimited);

		} else {
			EXPECT_EQ(r.result.action, first.result.action);
			EXPECT_EQ(r.result.reason_flags, first.result.reason_flags);
			EXPECT_EQ(r.result.active_sector, first.result.active_sector);
			EXPECT_TRUE(sameBits(output.vx, first_vx));
			EXPECT_TRUE(sameBits(output.vy, first_vy));

			for (int axis = 0; axis < 2; ++axis) {
				EXPECT_TRUE(sameBits(r.vel_sp_in_m_s[axis], first.vel_sp_in_m_s[axis]));
				EXPECT_TRUE(sameBits(r.vel_sp_out_m_s[axis], first.vel_sp_out_m_s[axis]));
				EXPECT_TRUE(sameBits(r.acc_sp_in_m_s2[axis], first.acc_sp_in_m_s2[axis]));
				EXPECT_TRUE(sameBits(r.acc_sp_out_m_s2[axis], first.acc_sp_out_m_s2[axis]));
			}
		}
	}
}

TEST_F(CollisionLimiterTest, NoConstraintAtAllInFlightIsASafeStop)
{
	_constraint = cp_brake::HookConstraint{};
	setVelocity(8.f, 2.f);
	setFeedForward(1.f, 0.5f);
	run();
	expectSafeStop(cp_brake::kReasonConstraintStale);
}

TEST_F(CollisionLimiterTest, AStaleConstraintIsASafeStop)
{
	setVelocity(8.f, 2.f);
	setFeedForward(1.f, 0.5f);

	// exactly at the limit is stale (the rule of the state machine for the sample age)
	_constraint.timestamp_us = kNowUs - 500000U;
	run();
	expectSafeStop(cp_brake::kReasonConstraintStale);

	_constraint.timestamp_us = kNowUs - 3000000U;
	run();
	expectSafeStop(cp_brake::kReasonConstraintStale);

	// a constraint stamped after now has a negative age: stale
	_constraint.timestamp_us = kNowUs + 1000U;
	run();
	expectSafeStop(cp_brake::kReasonConstraintStale);
}

TEST_F(CollisionLimiterTest, AConstraintJustInsideTheAgeLimitIsUsed)
{
	setVelocity(8.f, 0.f);
	_constraint.timestamp_us = kNowUs - 499999U;
	_constraint.caps_m_s[0] = 3.f;
	run();
	EXPECT_EQ(record().result.action, cp_brake::HookAction::kLimited);
	EXPECT_NEAR(_output.vx, 3.f, 1e-4f);
}

TEST_F(CollisionLimiterTest, AnAgeLimitThatIsNotSetIsASafeStop)
{
	// CPB_CSTR_AGE is a sentinel (-1) until the owner approves a value: fail safe, pick nothing
	for (const float limit : {-1.f, 0.f, NAN, INFINITY}) {
		SCOPED_TRACE(limit);
		resetInputs();
		_context.constraint_age_limit_s = limit;
		setVelocity(8.f, 2.f);
		setFeedForward(1.f, 0.5f);
		run();
		expectSafeStop(cp_brake::kReasonConstraintStale);
	}
}

TEST_F(CollisionLimiterTest, NoHeadingForTheSampleTimeIsASafeStopWhenThePx4StateIsValid)
{
	_constraint.heading_valid = false;
	_constraint.reason_flags = cp_brake::kReasonNoHeading; // what cp_brake reports
	setVelocity(8.f, 2.f);
	setFeedForward(1.f, 0.5f);
	run();
	expectSafeStop(cp_brake::kReasonNoHeading);

	// the hook says it too when the constraint did not
	_constraint.reason_flags = 0U;
	run();
	expectSafeStop(cp_brake::kReasonNoHeading);
}

TEST_F(CollisionLimiterTest, AHeadingThatIsNotANumberIsASafeStopAndMalformed)
{
	for (const float bad : {NAN, INFINITY}) {
		SCOPED_TRACE(bad);
		_constraint.heading_rad = bad;
		setVelocity(8.f, 2.f);
		setFeedForward(1.f, 0.5f);
		run();
		expectSafeStop(cp_brake::kReasonNoHeading | cp_brake::kReasonConstraintMalformed);
		expectAllFinite();
	}
}

TEST_F(CollisionLimiterTest, AnInvalidPx4StateMeansNoActionAtAll)
{
	// ADR 0007: UNQUALIFIED, CP adds no action, the PX4 failsafes act. cp_brake reports it as NOT_READY
	// without enforcement, and without a heading.
	_constraint.enforcement_active = false;
	_constraint.state = cp_brake::kStateNotReady;
	_constraint.heading_valid = false;
	_constraint.reason_flags = cp_brake::kReasonUnqualified | cp_brake::kReasonNoHeading;
	_constraint.caps_m_s[0] = 0.f;
	setVelocity(8.f, 2.f);
	setFeedForward(1.f, 0.5f);
	run();
	EXPECT_EQ(record().result.action, cp_brake::HookAction::kNoAction);
	EXPECT_EQ(record().result.reason_flags, cp_brake::kReasonUnqualified | cp_brake::kReasonNoHeading);
	expectSameAsBaseline();
	EXPECT_FLOAT_EQ(_output.vx, 8.f);
}

TEST_F(CollisionLimiterTest, AConstraintThatIsNotValidIsASafeStopWithItsOwnReasons)
{
	_constraint.valid = false;
	_constraint.state = cp_brake::kStateSafeStop;
	_constraint.reason_flags = cp_brake::kReasonInvalidConfig | cp_brake::kReasonAckRequired;
	setVelocity(8.f, 2.f);
	setFeedForward(1.f, 0.5f);
	run();
	expectSafeStop(cp_brake::kReasonInvalidConfig | cp_brake::kReasonAckRequired);
}

TEST_F(CollisionLimiterTest, ALatchedSafeStopStaysAtZeroEvenWithUsableCaps)
{
	_constraint.state = cp_brake::kStateSafeStop;
	_constraint.reason_flags = cp_brake::kReasonAckRequired;
	setVelocity(8.f, 2.f);
	setFeedForward(1.f, 0.5f);
	run();
	expectSafeStop(cp_brake::kReasonAckRequired);
}

TEST_F(CollisionLimiterTest, AnEnforcedStateTheHookCannotMapIsASafeStopAndMalformed)
{
	for (const uint8_t state : {cp_brake::kStateDisabled, cp_brake::kStateObserverOnly, cp_brake::kStateNotReady,
				    static_cast<uint8_t>(6), static_cast<uint8_t>(255)}) {
		SCOPED_TRACE(static_cast<int>(state));
		_constraint.state = state; // enforcement_active is still true
		setVelocity(8.f, 2.f);
		run();
		expectSafeStop(cp_brake::kReasonConstraintMalformed);
	}
}

TEST_F(CollisionLimiterTest, ASafeStopLeavesTheVerticalAxisAndTheYawAlone)
{
	_input.position[2] = -12.f; // climbing demand
	_input.yaw = 0.7f;
	_input.yawspeed = 0.1f;
	_input.velocity[2] = NAN;
	_constraint.state = cp_brake::kStateSafeStop;
	setVelocity(8.f, 2.f);
	run();
	expectSafeStop(0U);
	EXPECT_TRUE(sameBits(_output.vz, _baseline_output.vz));
	EXPECT_NE(_output.vz, 0.f);
}

TEST_F(CollisionLimiterTest, TheFeedForwardIsClearedInASafeStop)
{
	_constraint.state = cp_brake::kStateSafeStop;
	setVelocity(8.f, 2.f);
	setFeedForward(2.f, 1.f);
	run();
	expectSafeStop(0U);
	EXPECT_EQ(record().acc_sp_in_m_s2[0], 2.f);
	EXPECT_EQ(record().acc_sp_in_m_s2[1], 1.f);
}

TEST_F(CollisionLimiterTest, ASafeStopAlsoCoversAnAccelerationOnlySetpoint)
{
	// no velocity, no position: the vehicle would be driven by the acceleration alone
	_input.velocity[0] = NAN;
	_input.velocity[1] = NAN;
	setFeedForward(2.f, 1.f);
	_constraint.state = cp_brake::kStateSafeStop;
	run();
	expectSafeStop(0U);
	expectAllFinite();
}
