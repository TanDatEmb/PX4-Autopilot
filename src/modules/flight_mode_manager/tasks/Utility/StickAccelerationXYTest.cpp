/****************************************************************************
 *
 *   Copyright (C) 2026 PX4 Development Team. All rights reserved.
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
 * Functional tests of the cp_brake bound inside the Position stick generator (ADR 0026): the generator keeps its
 * velocity within the caps, so the hook downstream does not cut in normal flight. Bench values (fixtures only).
 */

#include <gtest/gtest.h>

#include <cmath>
#include <cstring>

#include <drivers/drv_hrt.h>
#include <lib/parameters/param.h>
#include <uORB/Publication.hpp>
#include <uORB/topics/cp_brake_constraints.h>
#include <uORB/topics/takeoff_status.h>
#include <uORB/topics/vehicle_local_position.h>
#include <uORB/topics/vehicle_status.h>

#include <GeneratorBound.hpp>
#include <StoppingModel.hpp>

#include "StickAccelerationXY.hpp"

using namespace matrix;

namespace
{

constexpr float kDt{0.02f};

void setFloat(const char *name, float value)
{
	param_set(param_find(name), &value);
}

void setInt(const char *name, int32_t value)
{
	param_set(param_find(name), &value);
}

class StickAccelerationXYTest : public ::testing::Test
{
protected:
	void SetUp() override
	{
		param_control_autosave(false);
		setInt("CPB_EN", 1);
		setFloat("CPB_CSTR_AGE", 0.5f);
		setFloat("CPB_A_BRK", 3.f);
		setFloat("CPB_A_SIDE", 1.2f);
		setFloat("CPB_JERK", 4.f);
		setFloat("CPB_T_LAT", 0.95f);
		setFloat("CPB_MARGIN", 2.f);
		setFloat("CPB_MAX_V", 12.f);
		setFloat("CPB_V_SIDE", 5.f);
		setFloat("CPB_REAR_ANG", 134.6f);
		setFloat("MPC_VEL_MANUAL", 10.f);
		setFloat("MPC_ACC_HOR", 3.f);
		setFloat("MPC_JERK_MAX", 8.f);

		takeoff_status_s takeoff{};
		takeoff.timestamp = hrt_absolute_time();
		takeoff.takeoff_state = takeoff_status_s::TAKEOFF_STATE_FLIGHT;
		takeoff.tilt_limit = 0.7854f;
		_takeoff_pub.publish(takeoff);

		vehicle_status_s status{};
		status.timestamp = hrt_absolute_time();
		status.nav_state = vehicle_status_s::NAVIGATION_STATE_POSCTL;
		_status_pub.publish(status);
	}

	// a constraint with every sector clear except one, published now (or age_s ago)
	void publishWall(int sector, float cap_m_s, float distance_m, float age_s = 0.f)
	{
		cp_brake_constraints_s c{};
		c.timestamp = hrt_absolute_time() - static_cast<hrt_abstime>(age_s * 1e6f);
		c.sample_timestamp = c.timestamp;
		c.heading_rad = 0.f;
		c.heading_valid = true;

		for (int i = 0; i < 72; ++i) {
			c.caps_m_s[i] = 12.f;
			c.distances_m[i] = 40.f;
		}

		if (sector >= 0) {
			c.caps_m_s[sector] = cap_m_s;
			c.distances_m[sector] = distance_m;
		}

		c.clearance_m = 2.f;
		c.enforcement_active = true;
		c.valid = true;
		c.state = cp_brake_constraints_s::STATE_ACTIVE_CLEAR;
		_constraints_pub.publish(c);
	}

	// one generator step with the limited setpoint fed back as the hook would pass it; returns the velocity setpoint
	Vector2f step(StickAccelerationXY &generator, const Vector2f &stick)
	{
		generator.generateSetpoints(stick, 0.f, 0.f, Vector3f(), _vel_sp, kDt);
		Vector3f pos_sp;
		Vector3f vel_sp;
		Vector3f acc_sp;
		generator.getSetpoints(pos_sp, vel_sp, acc_sp);
		_vel_sp = Vector2f(vel_sp);
		return _vel_sp;
	}

	Vector2f _vel_sp{};
	uORB::Publication<cp_brake_constraints_s> _constraints_pub{ORB_ID(cp_brake_constraints)};
	uORB::Publication<takeoff_status_s> _takeoff_pub{ORB_ID(takeoff_status)};
	uORB::Publication<vehicle_status_s> _status_pub{ORB_ID(vehicle_status)};
	uORB::Publication<vehicle_local_position_s> _local_pos_pub{ORB_ID(vehicle_local_position)};
};

const cp_brake::StoppingParams kForward{3.f, 4.f, 0.95f, 2.f, 12.f};

} // namespace

// Must run first: the uORB topic of the constraint persists across the tests of this process.
// With CP on and in flight but no constraint received yet, the hook safe-stops (stale), so the generator must not
// wind up behind it either (ADR 0026 decision 2).
TEST_F(StickAccelerationXYTest, NoConstraintYetHoldsLikeTheHook)
{
	StickAccelerationXY generator{nullptr};

	for (int k = 0; k < 100; ++k) {
		EXPECT_LE(step(generator, Vector2f(1.f, 0.f)).norm(), 1e-3f) << "step " << k;
	}
}

// SYS-CP-006: with CP off the generator is PX4's, and a clear constraint changes nothing either
TEST_F(StickAccelerationXYTest, CpOffAndClearConstraintAreUnbounded)
{
	setInt("CPB_EN", 0);
	publishWall(0, 0.f, 2.f); // ignored: CP off
	StickAccelerationXY off{nullptr};
	float off_speed[300];

	for (int k = 0; k < 300; ++k) {
		off_speed[k] = step(off, Vector2f(1.f, 0.f))(0);
	}

	setInt("CPB_EN", 1);
	publishWall(-1, 0.f, 0.f);
	_vel_sp.setZero();
	StickAccelerationXY clear{nullptr};

	for (int k = 0; k < 300; ++k) {
		EXPECT_FLOAT_EQ(step(clear, Vector2f(1.f, 0.f))(0), off_speed[k]) << "step " << k;
	}

	EXPECT_GT(off_speed[299], 9.f);
}

// a held cap ahead is never exceeded, and the generator settles at the bound, under it
TEST_F(StickAccelerationXYTest, HeldCapNeverExceeded)
{
	const float cap = 2.f;
	const float distance = cp_brake::stoppingDistance(cap, kForward);
	publishWall(0, cap, distance);
	StickAccelerationXY generator{nullptr};
	float top = 0.f;

	for (int k = 0; k < 500; ++k) {
		top = fmaxf(top, step(generator, Vector2f(1.f, 0.f))(0));
	}

	EXPECT_LE(top, cap + 1e-3f);
	EXPECT_GT(_vel_sp(0), 0.5f * cap); // it still moves toward the obstacle, slower
}

// at hover with a cap of 0 ahead the stick toward it does nothing, and the stick away moves at once
TEST_F(StickAccelerationXYTest, CapZeroNoCreepAtHover)
{
	publishWall(0, 0.f, 2.f);
	StickAccelerationXY generator{nullptr};

	for (int k = 0; k < 100; ++k) {
		EXPECT_LE(step(generator, Vector2f(1.f, 0.f)).norm(), 1e-3f) << "step " << k;
	}

	for (int k = 0; k < 50; ++k) {
		step(generator, Vector2f(-1.f, 0.f));
	}

	EXPECT_LT(_vel_sp(0), -1.f);
}

// a stale constraint gives a bound of 0: the generator brakes with the forward class model even with the stick held
TEST_F(StickAccelerationXYTest, StaleBrakesAtClassRate)
{
	publishWall(-1, 0.f, 0.f);
	StickAccelerationXY generator{nullptr};

	while (_vel_sp(0) < 5.f) {
		step(generator, Vector2f(1.f, 0.f));
	}

	publishWall(-1, 0.f, 0.f, 1.f); // one second old: stale against CPB_CSTR_AGE 0.5 s
	float previous = _vel_sp(0);

	for (int k = 0; k < 250; ++k) {
		const float v = step(generator, Vector2f(1.f, 0.f))(0);
		EXPECT_LE(previous - v, 3.f * kDt + 1e-3f) << "step " << k;
		previous = v;
	}

	EXPECT_LT(_vel_sp.norm(), 0.05f);
}

// The hook's second corridor ray is the vehicle's measured velocity: an obstacle off the setpoint corridor but on the
// vehicle's path must bound the generator as it limits the hook (same inputs as the hook).
TEST_F(StickAccelerationXYTest, VehicleVelocityRayCountsLikeTheHook)
{
	publishWall(4, 1.f, 20.f); // 20 deg right of the nose, 20 m away: 6 m off the north corridor, on a 20 deg path
	vehicle_local_position_s lpos{};
	lpos.timestamp = hrt_absolute_time();
	lpos.v_xy_valid = true;
	lpos.vx = 3.f * cosf(0.349066f);
	lpos.vy = 3.f * sinf(0.349066f);
	_local_pos_pub.publish(lpos);
	StickAccelerationXY generator{nullptr};
	float top = 0.f;

	for (int k = 0; k < 300; ++k) {
		top = fmaxf(top, step(generator, Vector2f(1.f, 0.f)).norm());
	}

	// along the sector the setpoint may carry at most the cap: |v| cos(20 deg) <= 1
	EXPECT_LE(top * cosf(0.349066f), 1.f + 1e-3f);
}
