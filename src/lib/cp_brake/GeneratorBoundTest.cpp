/****************************************************************************
 *
 *   Copyright (c) 2026 PX4 Development Team. All rights reserved.
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

#include <cmath>
#include <cstdint>
#include <cstdio>

#include "CapLimiter.hpp"
#include "StateMachine.hpp"
#include "GeneratorBound.hpp"
#include "SetpointLimiter.hpp"
#include "StoppingModel.hpp"

namespace
{

constexpr uint64_t kNowUs{10000000U};

// the X500 bench model (fixtures, not approved values): A_BRK 3, A_SIDE 1.2, JERK 4, T_LAT 0.95, MARGIN 2, MAX_V 12
const cp_brake::StoppingParams kForward{3.f, 4.f, 0.95f, 2.f, 12.f};
const cp_brake::StoppingParams kSide{1.2f, 4.f, 0.95f, 2.f, 12.f};
const float kHover[2] = {0.f, 0.f};

// lock and unclaimed bearings out of the way
cp_brake::HookContext openContext()
{
	return cp_brake::HookContext{true, true, true, kNowUs, 0.5f, 0.f, 1000.f, 180.f};
}

// the bench direction classes: lock 5 m/s, unclaimed beyond 134.6 deg
cp_brake::HookContext classContext()
{
	return cp_brake::HookContext{true, true, true, kNowUs, 0.5f, 0.f, 5.f, 134.6f};
}

// nothing near: every sector at the highest cap, 40 m away
cp_brake::HookConstraint clearConstraint()
{
	cp_brake::HookConstraint c{};
	c.present = true;
	c.timestamp_us = kNowUs - 10000U;
	c.heading_rad = 0.f;
	c.heading_valid = true;
	c.enforcement_active = true;
	c.valid = true;
	c.state = cp_brake::kStateActiveClear;

	for (std::size_t i = 0; i < cp_brake::kSectorCount; ++i) {
		c.caps_m_s[i] = 12.f;
		c.distances_m[i] = 40.f;
	}

	c.clearance_m = 2.f;
	return c;
}

cp_brake::HookConstraint wall(const std::size_t sector, const float cap_m_s, const float distance_m)
{
	cp_brake::HookConstraint c = clearConstraint();
	c.caps_m_s[sector] = cap_m_s;
	c.distances_m[sector] = distance_m;
	return c;
}

// slope of the stopping distance over the speed, the same finite difference the bound documents
float slope(const float speed_m_s, const cp_brake::StoppingParams &p)
{
	return (cp_brake::stoppingDistance(speed_m_s + 0.01f, p) - cp_brake::stoppingDistance(speed_m_s, p)) / 0.01f;
}

// the largest speed whose stopping distance fits in distance_m (the cap the model gives at that distance)
float capAt(const float distance_m, const cp_brake::StoppingParams &p)
{
	float lo = 0.f;
	float hi = p.max_speed_m_s;

	if (cp_brake::stoppingDistance(0.f, p) > distance_m) {
		return 0.f;
	}

	for (int i = 0; i < 60; ++i) {
		const float mid = 0.5f * (lo + hi);

		if (cp_brake::stoppingDistance(mid, p) <= distance_m) {
			lo = mid;

		} else {
			hi = mid;
		}
	}

	return lo;
}

} // namespace

TEST(GeneratorBound, UnenforcedRuleNotLimited)
{
	const float dir[2] = {1.f, 0.f};
	cp_brake::HookContext off = openContext();
	off.enabled = false;
	const cp_brake::GeneratorBound b = cp_brake::generatorSpeedBound(off, clearConstraint(), kHover, dir, 10.f, kForward,
					   kSide, 8.f);
	EXPECT_FALSE(b.limited);
	EXPECT_FLOAT_EQ(b.speed_m_s, 10.f);
}

// A cap at CPB_MAX_V can still fall: its margin is kept, or it appears as a step when the cap first drops (Codex).
TEST(GeneratorBound, SaturatedCapKeepsItsMargin)
{
	const float dir[2] = {1.f, 0.f};
	const cp_brake::GeneratorBound b = cp_brake::generatorSpeedBound(openContext(), clearConstraint(), kHover, dir, 12.f,
					   kForward, kSide, 8.f);
	ASSERT_TRUE(b.limited);
	const float rate = 12.f / slope(12.f, kForward);
	EXPECT_NEAR(b.speed_m_s, 12.f - rate * rate / 4.f, 1e-3f);
}

// The margin is sized with the jerk the generator actually follows with, here a PX4 jerk limit below CPB_JERK (Codex).
TEST(GeneratorBound, MarginUsesTheFollowerJerk)
{
	const float dir[2] = {1.f, 0.f};
	const float cap = 5.f;
	const cp_brake::GeneratorBound b = cp_brake::generatorSpeedBound(openContext(),
					   wall(0, cap, cp_brake::stoppingDistance(cap, kForward)), kHover, dir, 10.f, kForward, kSide, 2.f);
	ASSERT_TRUE(b.limited);
	EXPECT_FLOAT_EQ(b.jerk_m_s3, 2.f);
	const float rate = cap / slope(cap, kForward);
	EXPECT_NEAR(b.speed_m_s, cap - rate * rate / 2.f, 1e-3f);
}

TEST(GeneratorBound, StaleConstraintGivesZero)
{
	const float dir[2] = {1.f, 0.f};
	cp_brake::HookConstraint c = clearConstraint();
	c.timestamp_us = kNowUs - 600000U; // older than the 0.5 s age limit of the context
	const cp_brake::GeneratorBound b = cp_brake::generatorSpeedBound(openContext(), c, kHover, dir, 10.f, kForward, kSide, 8.f);
	EXPECT_TRUE(b.limited);
	EXPECT_FLOAT_EQ(b.speed_m_s, 0.f);
	EXPECT_FLOAT_EQ(b.brake_m_s2, 3.f);
}

TEST(GeneratorBound, WallAheadUsesForwardClassWithLagMargin)
{
	const float dir[2] = {1.f, 0.f};
	const float cap = 5.f;
	const cp_brake::GeneratorBound b = cp_brake::generatorSpeedBound(openContext(),
					   wall(0, cap, cp_brake::stoppingDistance(cap, kForward)), kHover, dir, 10.f, kForward, kSide, 8.f);
	ASSERT_TRUE(b.limited);
	EXPECT_FLOAT_EQ(b.brake_m_s2, 3.f);
	EXPECT_FLOAT_EQ(b.jerk_m_s3, 4.f);
	EXPECT_NEAR(b.fall_per_m, 1.f / slope(cap, kForward), 1e-4f);
	const float rate = cap / slope(cap, kForward);
	EXPECT_NEAR(b.speed_m_s, cap - rate * rate / 4.f, 1e-3f);
	EXPECT_GT(b.speed_m_s, 0.f);
}

TEST(GeneratorBound, WallBesideUsesSideClass)
{
	const float dir[2] = {0.f, 1.f}; // east, 90 degrees right of the nose (heading 0)
	const cp_brake::GeneratorBound b = cp_brake::generatorSpeedBound(openContext(),
					   wall(18, 2.f, cp_brake::stoppingDistance(2.f, kSide)), kHover, dir, 10.f, kForward, kSide, 8.f);
	ASSERT_TRUE(b.limited);
	EXPECT_FLOAT_EQ(b.brake_m_s2, 1.2f);
}

TEST(GeneratorBound, CapAboveTopSpeedIsStillReported)
{
	// a cap of 11 m/s is above the generator's 10 m/s, but the generator must see it coming while it accelerates
	const float dir[2] = {1.f, 0.f};
	const cp_brake::GeneratorBound b = cp_brake::generatorSpeedBound(openContext(),
					   wall(0, 11.f, cp_brake::stoppingDistance(11.f, kForward)), kHover, dir, 10.f, kForward, kSide, 8.f);
	EXPECT_TRUE(b.limited);
	EXPECT_LT(b.speed_m_s, 11.f);
}

TEST(GeneratorBound, SpeedLockHasNoLagMargin)
{
	const float dir[2] = {0.f, 1.f};
	const cp_brake::GeneratorBound b = cp_brake::generatorSpeedBound(classContext(), clearConstraint(), kHover, dir, 10.f,
					   kForward, kSide, 8.f);
	ASSERT_TRUE(b.limited);
	EXPECT_NEAR(b.speed_m_s, 5.f, 1e-4f);
	EXPECT_FLOAT_EQ(b.brake_m_s2, 1.2f);
}

// The lock binds first while an obstacle cap is still above it; the obstacle bound with its margin must already be
// there, or the bound steps down by the margin when the obstacle takes over (seen in SITL: the hook cut 1 m/s).
TEST(GeneratorBound, LockToObstacleHandOverIsContinuous)
{
	const float dir[2] = {0.f, 1.f};
	const float cap = 5.2f; // just above the 5 m/s lock
	const cp_brake::GeneratorBound b = cp_brake::generatorSpeedBound(classContext(),
					   wall(18, cap, cp_brake::stoppingDistance(cap, kSide)), kHover, dir, 10.f, kForward, kSide, 8.f);
	ASSERT_TRUE(b.limited);
	const float rate = cap / slope(cap, kSide);
	EXPECT_NEAR(b.speed_m_s, cap - rate * rate / 4.f, 1e-3f);
	EXPECT_LT(b.speed_m_s, 5.f);
	EXPECT_FLOAT_EQ(b.brake_m_s2, 1.2f);
}

TEST(GeneratorBound, AccelerationReachesTheBoundWithoutOvershoot)
{
	// below the bound the acceleration may only be as large as a jerk-limited ramp-down allows
	EXPECT_NEAR(cp_brake::boundAcceleration(3.f, 0.f, 5.f, 0.f, 4.f, 3.f, 0.02f), sqrtf(2.f * 4.f * 2.f), 1e-5f);
	EXPECT_FLOAT_EQ(cp_brake::boundAcceleration(5.f, 0.f, 5.f, 0.f, 4.f, 3.f, 0.02f), 0.f);
	// above it the vehicle brakes, never harder than the class allows
	EXPECT_NEAR(cp_brake::boundAcceleration(5.5f, 0.f, 5.f, 0.f, 4.f, 3.f, 0.02f), -2.f, 1e-5f);
	EXPECT_FLOAT_EQ(cp_brake::boundAcceleration(9.f, 0.f, 5.f, 0.f, 4.f, 3.f, 0.02f), -3.f);
	// one step never crosses the bound, even when the headroom is smaller than the continuous law assumes
	EXPECT_LE(4.999f + 0.02f * cp_brake::boundAcceleration(4.999f, 0.f, 5.f, 0.f, 4.f, 3.f, 0.02f), 5.f + 1e-6f);
}

// A bound that falls closes the headroom from both sides: the law must already match its rate when the speed
// meets it, or a generator still accelerating overshoots (found with PX4 jerk limits up to 5 m/s^3).
TEST(GeneratorBound, AccelerationAnticipatesAFallingBound)
{
	EXPECT_NEAR(cp_brake::boundAcceleration(3.f, 0.f, 5.f, 2.f / 3.f, 4.f, 3.f, 0.02f), -2.f + sqrtf(2.f * 4.f * 2.f), 1e-5f);
	EXPECT_NEAR(cp_brake::boundAcceleration(5.f, 0.f, 5.f, 0.4f, 4.f, 3.f, 0.02f), -2.f, 1e-5f); // on it: follow its fall
	EXPECT_NEAR(cp_brake::boundAcceleration(5.5f, 0.f, 5.f, 1.f / 5.5f, 4.f, 3.f, 0.02f), -3.f, 1e-5f); // above: never past the class
	EXPECT_NEAR(cp_brake::boundAcceleration(5.1f, 0.f, 5.f, 1.f / 5.1f, 4.f, 3.f, 0.02f), -1.f - sqrtf(2.f * 4.f * 0.1f), 1e-4f);
}

// While the generator still accelerates, the bound's fall rate grows with its speed: the jerk left to bring the
// closing acceleration to zero is J - fall_per_m * a (found with PX4 jerk limits of 3 and 4 m/s^3 at 7 to 8 m/s).
TEST(GeneratorBound, AccelerationLeavesJerkForAGrowingFall)
{
	const float accelerating = cp_brake::boundAcceleration(5.f, 3.f, 8.f, 0.25f, 4.f, 3.f, 0.02f);
	EXPECT_NEAR(accelerating, -5.f * 0.25f + sqrtf(2.f * (4.f - 0.25f * 3.f) * 3.f), 1e-4f);
	EXPECT_LT(accelerating, cp_brake::boundAcceleration(5.f, 0.f, 8.f, 0.25f, 4.f, 3.f, 0.02f));
}

// The reason for the lag margin: a generator that follows the bound with the stick acceleration and the PX4 jerk
// limit stays under the cap the hook computes at the same place, so the hook does not cut and clear the
// feed-forward in a normal approach. Caps are held for 60 ms as cp_brake publishes them. At a creep (below
// 0.5 m/s, a few cm from the stop) a new map may lower the cap a few mm/s under the speed; the hook then clears
// a feed-forward the vehicle cannot feel, which is what ADR 0026 measures.
static void straightApproach(const float jerk_limit)
{
	const float dt = 0.02f;
	const float wall_m = 40.f;
	const float stick_acc = 6.f; // full stick, 2 x MPC_ACC_HOR
	float x = 0.f;
	float v = 0.f;
	float a = 0.f;
	float held_cap = 12.f;
	float held_distance = wall_m;
	float worst = -1e9f;
	float worst_above_creep = -1e9f;
	float ff_lost = 0.f;

	for (int k = 0; k < 3000; ++k) {
		if (k % 3 == 0) { // a new map every 60 ms
			held_distance = wall_m - x;
			held_cap = capAt(held_distance, kForward);
		}

		const float dir[2] = {1.f, 0.f};
		const float vel[2] = {v, 0.f};
		const cp_brake::GeneratorBound b = cp_brake::generatorSpeedBound(openContext(), wall(0, held_cap, held_distance), vel,
						   dir, 10.f, kForward, kSide, jerk_limit);
		// the generator's own top speed, reached smoothly as the PX4 drag does
		float a_cmd = fminf(stick_acc, cp_brake::boundAcceleration(v, a, 10.f, 0.f, jerk_limit, 6.f, dt));

		if (b.limited) {
			a_cmd = fminf(a_cmd, cp_brake::boundAcceleration(v, a, b.speed_m_s, b.fall_per_m, b.jerk_m_s3, b.brake_m_s2, dt));
		}

		const float step = jerk_limit * dt;
		a = fmaxf(a - step, fminf(a + step, a_cmd));
		v = fmaxf(v + a * dt, 0.f);
		x += v * dt;

		// the hook would cut when the setpoint passes the cap it holds
		worst = fmaxf(worst, v - held_cap);

		if (v > 0.5f) {
			worst_above_creep = fmaxf(worst_above_creep, v - held_cap);
		}

		if (v - held_cap > 1e-4f) {
			ff_lost = fmaxf(ff_lost, fabsf(a));
		}
	}

	printf("JERK %.1f worst_above_creep %.3f worst %.3f ff_lost %.2f stop %.2f\n", (double)jerk_limit, (double)worst_above_creep, (double)worst,
	       (double)ff_lost, (double)(wall_m - x));
	EXPECT_LE(worst_above_creep, 1e-4f); // the hook does not cut while the vehicle moves
	EXPECT_LE(worst, 0.005f);
	EXPECT_LE(ff_lost, 0.2f); // what it clears at a creep is not felt
	EXPECT_GE(wall_m - x, kForward.margin_m);
	EXPECT_LT(v, 0.05f);
}

TEST(GeneratorBound, StraightApproachHookClearsNoFeltFeedForward)
{
	straightApproach(8.f); // MPC_JERK_MAX at bench
	straightApproach(2.f); // PX4 jerk limits below, at and above CPB_JERK (4)
	straightApproach(3.f);
	straightApproach(4.f);
	straightApproach(5.f);
	straightApproach(6.f);
}
