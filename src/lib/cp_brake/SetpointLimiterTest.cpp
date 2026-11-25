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
 * @file SetpointLimiterTest.cpp
 *
 * Unit tests of the limiter hook function (collision-prevention ICD-CP-04, ADR 0008, ADR 0025): the
 * geometry of every sector at several headings, the corridor, and the properties that must hold for
 * any input. The speeds, caps, distances and clearances are test fixtures, not approved values.
 */

#include <gtest/gtest.h>

#include <cmath>
#include <cstdint>
#include <utility>

#include "ReasonFlags.hpp"
#include "SetpointLimiter.hpp"
#include "StateMachine.hpp"

namespace
{

constexpr float kPi{3.14159265358979f};
constexpr float kSectorRad{2.f * kPi / 72.f};
constexpr uint64_t kNowUs{10000000U};
constexpr float kClearanceM{2.f};
const float kHover[2] = {0.f, 0.f}; // no current velocity: only the setpoint makes the corridor

// the speed lock and the unclaimed bearings are out of the way here (a lock no setpoint reaches, no
// unclaimed bearing); the tests of SYS-CP-031 set them
cp_brake::HookContext flightContext()
{
	return cp_brake::HookContext{true, true, true, kNowUs, 0.5f, 0.f, 1000.f, 180.f};
}

// the X500 bench values of the direction classes (fixtures here): lock 5 m/s, unclaimed beyond 134.6 deg
cp_brake::HookContext classContext(const float heading_rad)
{
	return cp_brake::HookContext{true, true, true, kNowUs, 0.5f, heading_rad, 5.f, 134.6f};
}

cp_brake::HookConstraint openConstraint(const float heading_rad)
{
	cp_brake::HookConstraint c{};
	c.present = true;
	c.timestamp_us = kNowUs - 10000U;
	c.heading_rad = heading_rad;
	c.heading_valid = true;
	c.enforcement_active = true;
	c.valid = true;
	c.state = cp_brake::kStateActiveClear;

	// every obstacle at distance 0: every sector the setpoint enters counts (the half-planes of ADR 0008)
	for (std::size_t i = 0; i < cp_brake::kSectorCount; ++i) {
		c.caps_m_s[i] = 12.f;
		c.distances_m[i] = 0.f;
	}

	c.clearance_m = kClearanceM;
	return c;
}

float dot(const float a[2], const float b[2])
{
	return a[0] * b[0] + a[1] * b[1];
}

float cross(const float a[2], const float b[2])
{
	return a[0] * b[1] - a[1] * b[0];
}

void direction(const int sector, const float heading_rad, float out[2])
{
	const double bearing = static_cast<double>(heading_rad) + static_cast<double>(sector) * static_cast<double>(kSectorRad);
	out[0] = static_cast<float>(std::cos(bearing));
	out[1] = static_cast<float>(std::sin(bearing));
}

// cap as the hook must read it: a finite number >= 0, anything else is 0
float usableCap(const float cap)
{
	return (std::isfinite(cap) && cap >= 0.f) ? cap : 0.f;
}

// deterministic pseudo-random numbers, so a failure can be reproduced
struct Lcg {
	uint32_t state;
	float next() // [0, 1)
	{
		state = state * 1664525U + 1013904223U;
		return static_cast<float>(state >> 8) / 16777216.f;
	}
};

} // namespace

TEST(SetpointLimiterTest, EverySectorScalesTheSetpointToItsCapAtEveryHeading)
{
	// ADR 0025: the setpoint keeps its direction and its speed is scaled until the component along the
	// sector meets the cap
	const float headings[] = {0.f, 0.7f, -2.5f, 6.f, kPi, -kPi, kPi / 2.f};

	for (const float heading : headings) {
		for (int sector = 0; sector < static_cast<int>(cp_brake::kSectorCount); ++sector) {
			SCOPED_TRACE(::testing::Message() << "heading " << heading << " sector " << sector);
			cp_brake::HookConstraint c = openConstraint(heading);
			c.caps_m_s[sector] = 3.f;

			float u[2];
			direction(sector, heading, u);
			const float perp[2] = {-u[1], u[0]};
			float vel[2] = {8.f * u[0] + 2.f * perp[0], 8.f * u[1] + 2.f * perp[1]};
			float acc[2] = {NAN, NAN};

			const cp_brake::HookResult r = cp_brake::limitHorizontalSetpoint(flightContext(), c, kHover, vel, acc);
			EXPECT_EQ(r.action, cp_brake::HookAction::kLimited);
			EXPECT_EQ(r.active_sector, sector);
			EXPECT_NEAR(vel[0] * u[0] + vel[1] * u[1], 3.f, 1e-3f);
			EXPECT_NEAR(vel[0] * perp[0] + vel[1] * perp[1], 2.f * 3.f / 8.f, 1e-3f);
		}
	}
}

TEST(SetpointLimiterTest, TheResultSatisfiesEverySectorForAnyInput)
{
	Lcg rng{12345U};
	const cp_brake::HookContext context = flightContext();

	for (int trial = 0; trial < 3000; ++trial) {
		const float heading = (rng.next() - 0.5f) * 14.f;
		cp_brake::HookConstraint c = openConstraint(heading);

		for (std::size_t i = 0; i < cp_brake::kSectorCount; ++i) {
			const float pick = rng.next();

			if (pick < 0.05f) {
				c.caps_m_s[i] = NAN;

			} else if (pick < 0.08f) {
				c.caps_m_s[i] = INFINITY;

			} else if (pick < 0.11f) {
				c.caps_m_s[i] = -rng.next();

			} else if (pick < 0.40f) {
				c.caps_m_s[i] = 0.f;

			} else {
				c.caps_m_s[i] = rng.next() * 12.f;
			}
		}

		const float speed = rng.next() * 12.f;
		const float angle = rng.next() * 2.f * kPi;
		const float in[2] = {speed * cosf(angle), speed * sinf(angle)};
		float vel[2] = {in[0], in[1]};
		float acc[2] = {(rng.next() - 0.5f) * 6.f, (rng.next() - 0.5f) * 6.f};

		const cp_brake::HookResult r = cp_brake::limitHorizontalSetpoint(context, c, kHover, vel, acc);
		SCOPED_TRACE(::testing::Message() << "trial " << trial);
		ASSERT_TRUE(std::isfinite(vel[0]) && std::isfinite(vel[1]) && std::isfinite(acc[0]) && std::isfinite(acc[1]));
		EXPECT_TRUE(r.action == cp_brake::HookAction::kClear || r.action == cp_brake::HookAction::kLimited);

		// never faster than asked, and never in another direction (ADR 0025: no sliding)
		EXPECT_LE(std::hypot(vel[0], vel[1]), speed + 1e-4f);
		EXPECT_NEAR(cross(in, vel), 0.f, 1e-3f);
		EXPECT_GE(dot(in, vel), -1e-4f);

		// every sector is respected (all distances are 0, so every sector the setpoint enters counts)
		for (int sector = 0; sector < static_cast<int>(cp_brake::kSectorCount); ++sector) {
			float u[2];
			direction(sector, heading, u);
			EXPECT_LE(vel[0] * u[0] + vel[1] * u[1], usableCap(c.caps_m_s[sector]) + 1e-3f) << "sector " << sector;
		}

		// a second application changes nothing: the hook is a pure function of the setpoint it is given
		float vel_again[2] = {vel[0], vel[1]};
		float acc_again[2] = {acc[0], acc[1]};
		cp_brake::limitHorizontalSetpoint(context, c, kHover, vel_again, acc_again);
		EXPECT_NEAR(vel_again[0], vel[0], 2e-3f);
		EXPECT_NEAR(vel_again[1], vel[1], 2e-3f);

		// the same inputs give the same bits
		float vel_first[2] = {in[0], in[1]};
		float vel_second[2] = {in[0], in[1]};
		float acc_first[2] = {acc[0], acc[1]};
		float acc_second[2] = {acc[0], acc[1]};
		cp_brake::limitHorizontalSetpoint(context, c, kHover, vel_first, acc_first);
		cp_brake::limitHorizontalSetpoint(context, c, kHover, vel_second, acc_second);
		EXPECT_EQ(vel_first[0], vel_second[0]);
		EXPECT_EQ(vel_first[1], vel_second[1]);
		EXPECT_EQ(acc_first[0], acc_second[0]);
		EXPECT_EQ(acc_first[1], acc_second[1]);
	}
}

TEST(SetpointLimiterTest, ASafeStopGivesExactZeroEvenWhenTheSetpointIsNotANumber)
{
	cp_brake::HookConstraint c = openConstraint(0.f);
	c.state = cp_brake::kStateSafeStop;
	float vel[2] = {NAN, INFINITY};
	float acc[2] = {NAN, 3.f};

	const cp_brake::HookResult r = cp_brake::limitHorizontalSetpoint(flightContext(), c, kHover, vel, acc);
	EXPECT_EQ(r.action, cp_brake::HookAction::kSafeStop);
	EXPECT_EQ(vel[0], 0.f);
	EXPECT_EQ(vel[1], 0.f);
	EXPECT_EQ(acc[0], 0.f);
	EXPECT_EQ(acc[1], 0.f);
}

TEST(SetpointLimiterTest, TransparentPathsDoNotReadOrChangeAnything)
{
	cp_brake::HookConstraint c = openConstraint(NAN);
	c.state = cp_brake::kStateSafeStop;
	c.valid = false;
	c.caps_m_s[0] = 0.f;

	cp_brake::HookContext context = flightContext();

	context.enabled = false;
	{
		float vel[2] = {8.f, 1.f};
		float acc[2] = {1.f, 1.f};
		const cp_brake::HookResult r = cp_brake::limitHorizontalSetpoint(context, c, kHover, vel, acc);
		EXPECT_EQ(r.action, cp_brake::HookAction::kTransparent);
		EXPECT_EQ(r.reason_flags, 0U);
		EXPECT_EQ(vel[0], 8.f);
		EXPECT_EQ(vel[1], 1.f);
		EXPECT_EQ(acc[0], 1.f);
		EXPECT_EQ(acc[1], 1.f);
	}

	context = flightContext();
	context.in_flight = false;
	{
		float vel[2] = {8.f, 1.f};
		float acc[2] = {1.f, 1.f};
		const cp_brake::HookResult r = cp_brake::limitHorizontalSetpoint(context, c, kHover, vel, acc);
		EXPECT_EQ(r.action, cp_brake::HookAction::kTransparent);
		EXPECT_EQ(vel[0], 8.f);
	}

	context = flightContext();
	context.nav_state_enforced = false;
	{
		float vel[2] = {8.f, 1.f};
		float acc[2] = {1.f, 1.f};
		const cp_brake::HookResult r = cp_brake::limitHorizontalSetpoint(context, c, kHover, vel, acc);
		EXPECT_EQ(r.action, cp_brake::HookAction::kTransparent);
		EXPECT_EQ(r.reason_flags, cp_brake::kReasonNavState);
		EXPECT_EQ(vel[0], 8.f);
	}
}

TEST(SetpointLimiterTest, TheNewReasonBitsAreDistinctAndOutsideTheConstraintFaultMask)
{
	const uint32_t bits[] = {cp_brake::kReasonConstraintStale, cp_brake::kReasonConstraintMalformed, cp_brake::kReasonNavState};
	uint32_t seen = cp_brake::kReasonInvalidConfig | cp_brake::kReasonFrameInvalid | cp_brake::kReasonDataInvalid
			| cp_brake::kReasonSampleStale | cp_brake::kReasonNoHeading | cp_brake::kReasonUnqualified
			| cp_brake::kReasonBlocked | cp_brake::kReasonAckRequired | cp_brake::kReasonOverride
			| cp_brake::kReasonTimeSync;

	for (const uint32_t bit : bits) {
		EXPECT_NE(bit, 0U);
		EXPECT_EQ(bit & (bit - 1U), 0U); // one bit
		EXPECT_EQ(seen & bit, 0U);
		EXPECT_EQ(cp_brake::kReasonConstraintFaults & bit, 0U); // hook reasons do not change the state machine
		seen |= bit;
	}
}

TEST(SetpointLimiterTest, AnInfiniteVelocitySetpointIsLeftAloneAndNothingNotANumberIsMade)
{
	cp_brake::HookConstraint c = openConstraint(0.f);
	c.caps_m_s[0] = 3.f;

	float vel[2] = {INFINITY, 0.f};
	float acc[2] = {1.f, 1.f};
	const cp_brake::HookResult r = cp_brake::limitHorizontalSetpoint(flightContext(), c, kHover, vel, acc);
	EXPECT_EQ(r.action, cp_brake::HookAction::kClear);
	EXPECT_TRUE(std::isinf(vel[0]));
	EXPECT_EQ(vel[1], 0.f);
	EXPECT_EQ(acc[0], 1.f);
	EXPECT_EQ(acc[1], 1.f);

	float vel_nan[2] = {NAN, 4.f};
	float acc_nan[2] = {1.f, 1.f};
	cp_brake::limitHorizontalSetpoint(flightContext(), c, kHover, vel_nan, acc_nan);
	EXPECT_TRUE(std::isnan(vel_nan[0]));
	EXPECT_EQ(vel_nan[1], 4.f);
}

TEST(SetpointLimiterTest, TwoCapsAtOnceTheSmallerRatioSetsTheScale)
{
	// toward an inside corner the direction is kept and the tighter sector sets the speed: 2 / 8
	cp_brake::HookConstraint c = openConstraint(0.f);
	c.caps_m_s[0] = 3.f;
	c.caps_m_s[18] = 2.f;
	float vel[2] = {8.f, 8.f};
	float acc[2] = {NAN, NAN};
	const cp_brake::HookResult r = cp_brake::limitHorizontalSetpoint(flightContext(), c, kHover, vel, acc);
	EXPECT_EQ(r.action, cp_brake::HookAction::kLimited);
	EXPECT_EQ(r.active_sector, 18);
	EXPECT_NEAR(vel[0], 2.f, 1e-3f);
	EXPECT_NEAR(vel[1], 2.f, 1e-3f);
}

TEST(SetpointLimiterTest, AnObstacleFartherThanTheClearanceFromThePathDoesNotLimit)
{
	// a wall 5 m to the right of a northward path: the sector at 45 degrees sees it at about 6.8 m, and
	// its nearest edge (42.5 degrees) lies 4.6 m from the path, more than the 2 m clearance
	cp_brake::HookConstraint c = openConstraint(0.f);
	c.caps_m_s[9] = 1.f;
	c.distances_m[9] = 6.8f;
	float vel[2] = {8.f, 0.f};
	float acc[2] = {1.f, 0.f};
	const cp_brake::HookResult r = cp_brake::limitHorizontalSetpoint(flightContext(), c, kHover, vel, acc);
	EXPECT_EQ(r.action, cp_brake::HookAction::kClear);
	EXPECT_EQ(r.active_sector, cp_brake::kNoSector);
	EXPECT_EQ(vel[0], 8.f);
	EXPECT_EQ(vel[1], 0.f);
	EXPECT_EQ(acc[0], 1.f);

	// the same sector at 2.5 m: its edge is 1.7 m from the path, so it counts and the speed is scaled
	c.distances_m[9] = 2.5f;
	float vel_near[2] = {8.f, 0.f};
	float acc_near[2] = {1.f, 0.f};
	const cp_brake::HookResult near = cp_brake::limitHorizontalSetpoint(flightContext(), c, kHover, vel_near, acc_near);
	EXPECT_EQ(near.action, cp_brake::HookAction::kLimited);
	EXPECT_EQ(near.active_sector, 9);
	EXPECT_NEAR(vel_near[0], 1.f / cosf(kPi / 4.f), 1e-3f);
	EXPECT_EQ(vel_near[1], 0.f);
}

TEST(SetpointLimiterTest, TheCurrentVelocityAlsoMakesTheCorridor)
{
	// setpoint north, sector at 45 degrees 4 m away: off the setpoint ray (edge 2.7 m from it), but on
	// the ray of a current velocity 30 degrees east of north (edge 0.87 m from it)
	cp_brake::HookConstraint c = openConstraint(0.f);
	c.caps_m_s[9] = 1.f;
	c.distances_m[9] = 4.f;

	float vel[2] = {8.f, 0.f};
	float acc[2] = {NAN, NAN};
	EXPECT_EQ(cp_brake::limitHorizontalSetpoint(flightContext(), c, kHover, vel, acc).action, cp_brake::HookAction::kClear);

	const float turning[2] = {6.f * cosf(kPi / 6.f), 6.f * sinf(kPi / 6.f)};
	const cp_brake::HookResult r = cp_brake::limitHorizontalSetpoint(flightContext(), c, turning, vel, acc);
	EXPECT_EQ(r.action, cp_brake::HookAction::kLimited);
	EXPECT_NEAR(vel[0], 1.f / cosf(kPi / 4.f), 1e-3f);
	EXPECT_EQ(vel[1], 0.f);

	// a current velocity that is not a number gives no direction
	const float unknown[2] = {NAN, 1.f};
	float vel_nan[2] = {8.f, 0.f};
	EXPECT_EQ(cp_brake::limitHorizontalSetpoint(flightContext(), c, unknown, vel_nan, acc).action, cp_brake::HookAction::kClear);
}

TEST(SetpointLimiterTest, ASectorAtTheMarginStopsEveryMotionThatEntersIt)
{
	// a cap of 0 comes with the margin as its distance: the edge of a sector at 60 degrees lies 1.7 m
	// from the path, inside a clearance equal to the margin, so the motion stops
	cp_brake::HookConstraint c = openConstraint(0.f);
	c.caps_m_s[12] = 0.f;
	c.distances_m[12] = kClearanceM;
	float vel[2] = {8.f, 0.f};
	float acc[2] = {1.f, 1.f};
	const cp_brake::HookResult r = cp_brake::limitHorizontalSetpoint(flightContext(), c, kHover, vel, acc);
	EXPECT_EQ(r.action, cp_brake::HookAction::kLimited);
	EXPECT_EQ(vel[0], 0.f);
	EXPECT_EQ(vel[1], 0.f);
	EXPECT_EQ(acc[0], 0.f);

	// moving away from it is not limited
	float away[2] = {-8.f, -1.f};
	float acc_away[2] = {1.f, 1.f};
	EXPECT_EQ(cp_brake::limitHorizontalSetpoint(flightContext(), c, kHover, away, acc_away).action, cp_brake::HookAction::kClear);
	EXPECT_EQ(away[0], -8.f);
}

TEST(SetpointLimiterTest, ADistanceOrClearanceThatIsNotUsableMakesTheSectorCount)
{
	const float bad_distances[] = {NAN, -1.f};
	const float bad_clearances[] = {NAN, INFINITY, -1.f};

	for (const float bad : bad_distances) {
		cp_brake::HookConstraint c = openConstraint(0.f);
		c.caps_m_s[9] = 1.f;
		c.distances_m[9] = bad;
		float vel[2] = {8.f, 0.f};
		float acc[2] = {NAN, NAN};
		EXPECT_EQ(cp_brake::limitHorizontalSetpoint(flightContext(), c, kHover, vel, acc).action,
			  cp_brake::HookAction::kLimited) << "distance " << bad;
	}

	for (const float bad : bad_clearances) {
		cp_brake::HookConstraint c = openConstraint(0.f);
		c.caps_m_s[9] = 1.f;
		c.distances_m[9] = 30.f;
		c.clearance_m = bad;
		float vel[2] = {8.f, 0.f};
		float acc[2] = {NAN, NAN};
		EXPECT_EQ(cp_brake::limitHorizontalSetpoint(flightContext(), c, kHover, vel, acc).action,
			  cp_brake::HookAction::kLimited) << "clearance " << bad;
	}
}

TEST(SetpointLimiterTest, ARandomMapNeverTurnsTheSetpointAndMeetsEverySectorThatCounts)
{
	Lcg rng{777U};

	for (int trial = 0; trial < 3000; ++trial) {
		const float heading = (rng.next() - 0.5f) * 14.f;
		cp_brake::HookConstraint c = openConstraint(heading);

		for (std::size_t i = 0; i < cp_brake::kSectorCount; ++i) {
			c.caps_m_s[i] = (rng.next() < 0.3f) ? rng.next() * 6.f : 12.f;
			c.distances_m[i] = 2.f + rng.next() * 38.f;
		}

		const float speed = rng.next() * 12.f;
		const float angle = rng.next() * 2.f * kPi;
		const float in[2] = {speed * cosf(angle), speed * sinf(angle)};
		const float now[2] = {(rng.next() - 0.5f) * 12.f, (rng.next() - 0.5f) * 12.f};
		float vel[2] = {in[0], in[1]};
		float acc[2] = {NAN, NAN};
		cp_brake::limitHorizontalSetpoint(flightContext(), c, now, vel, acc);
		SCOPED_TRACE(::testing::Message() << "trial " << trial);

		EXPECT_NEAR(cross(in, vel), 0.f, 1e-3f);
		EXPECT_GE(dot(in, vel), -1e-4f);
		EXPECT_LE(std::hypot(vel[0], vel[1]), speed + 1e-4f);

		// a sector whose obstacle may lie on the setpoint path within the clearance is met
		for (int sector = 0; sector < static_cast<int>(cp_brake::kSectorCount); ++sector) {
			float u[2];
			direction(sector, heading, u);
			const double s = static_cast<double>(speed);
			const double a = std::acos(std::fmax(-1.0, std::fmin(1.0, static_cast<double>(dot(in, u)) / (s + 1e-12))));
			const double edge = std::fmax(0.0, a - static_cast<double>(kSectorRad) / 2.0);
			const double offset = static_cast<double>(c.distances_m[sector]) * std::sin(edge);

			if (s > 1e-3 && a < static_cast<double>(kPi) / 2.0 && offset < static_cast<double>(kClearanceM) - 1e-3) {
				EXPECT_LE(dot(vel, u), c.caps_m_s[sector] + 1e-3f) << "sector " << sector;
			}
		}
	}
}

TEST(SetpointLimiterTest, AHalfPlaneAtExactlyNinetyDegreesLeavesTheMotionAlone)
{
	// ADR 0008, 0025: an obstacle exactly to the side does not reduce forward motion, even with a cap of 0
	// at distance 0 (the setpoint has no component toward it)
	cp_brake::HookConstraint c = openConstraint(0.f);
	c.caps_m_s[18] = 0.f;
	c.caps_m_s[54] = 0.f;
	float vel[2] = {8.f, 0.f};
	float acc[2] = {NAN, NAN};
	const cp_brake::HookResult r = cp_brake::limitHorizontalSetpoint(flightContext(), c, kHover, vel, acc);
	EXPECT_EQ(r.action, cp_brake::HookAction::kClear);
	EXPECT_EQ(vel[0], 8.f);
	EXPECT_EQ(vel[1], 0.f);
}

TEST(SetpointLimiterTest, ASetpointWellInsideEveryCapChangesNothingAndIsClear)
{
	cp_brake::HookConstraint c = openConstraint(0.f);
	c.caps_m_s[0] = 5.f;
	float vel[2] = {1.f, 0.5f};
	float acc[2] = {2.f, 1.f};

	const cp_brake::HookResult r = cp_brake::limitHorizontalSetpoint(flightContext(), c, kHover, vel, acc);
	EXPECT_EQ(r.action, cp_brake::HookAction::kClear);
	EXPECT_EQ(vel[0], 1.f);
	EXPECT_EQ(acc[0], 2.f);
	EXPECT_EQ(acc[1], 1.f);
}

TEST(SetpointLimiterTest, TheWholeFeedForwardIsClearedWhenTheVelocityIsLimited)
{
	// ADR 0020: when any sector limits the velocity setpoint, the horizontal feed-forward is cleared, the
	// part along the obstacle and the part away from it included
	cp_brake::HookConstraint c = openConstraint(0.f);
	c.caps_m_s[0] = 3.f;

	for (const auto &ff : {std::pair<float, float>{2.f, 1.f}, {2.f, 0.f}, {-2.f, 0.5f}, {0.f, -1.f}}) {
		float vel[2] = {8.f, 2.f};
		float acc[2] = {ff.first, ff.second};
		const cp_brake::HookResult r = cp_brake::limitHorizontalSetpoint(flightContext(), c, kHover, vel, acc);
		EXPECT_EQ(r.action, cp_brake::HookAction::kLimited);
		EXPECT_EQ(acc[0], 0.f);
		EXPECT_EQ(acc[1], 0.f);
	}
}

TEST(SetpointLimiterTest, TheFeedForwardIsUnchangedWhenNoSectorLimits)
{
	cp_brake::HookConstraint c = openConstraint(0.f);
	c.caps_m_s[0] = 5.f;

	// well inside every cap
	{
		float vel[2] = {1.f, 0.5f};
		float acc[2] = {2.f, 1.f};
		const cp_brake::HookResult r = cp_brake::limitHorizontalSetpoint(flightContext(), c, kHover, vel, acc);
		EXPECT_EQ(r.action, cp_brake::HookAction::kClear);
		EXPECT_EQ(acc[0], 2.f);
		EXPECT_EQ(acc[1], 1.f);
	}

	// exactly at a cap with nothing to project: no sector limits the setpoint, so nothing is cleared (the
	// rule of ADR 0020 has no gradation)
	{
		c.caps_m_s[0] = 1.f;
		float vel[2] = {1.f, 0.5f};
		float acc[2] = {2.f, 0.f};
		const cp_brake::HookResult r = cp_brake::limitHorizontalSetpoint(flightContext(), c, kHover, vel, acc);
		EXPECT_EQ(r.action, cp_brake::HookAction::kClear);
		EXPECT_EQ(acc[0], 2.f);
		EXPECT_EQ(acc[1], 0.f);
	}
}

TEST(SetpointLimiterTest, AFeedForwardThatIsNotANumberStaysNotANumberWhenTheVelocityIsLimited)
{
	cp_brake::HookConstraint c = openConstraint(0.f);
	c.caps_m_s[0] = 3.f;

	for (const auto &ff : {std::pair<float, float>{NAN, NAN}, {NAN, 1.f}, {1.f, NAN}, {INFINITY, 0.f}}) {
		float vel[2] = {8.f, 0.f};
		float acc[2] = {ff.first, ff.second};
		const cp_brake::HookResult r = cp_brake::limitHorizontalSetpoint(flightContext(), c, kHover, vel, acc);
		EXPECT_EQ(r.action, cp_brake::HookAction::kLimited);
		EXPECT_NEAR(vel[0], 3.f, 1e-3f);

		// not a number or not finite: not a feed-forward, left as it was
		EXPECT_TRUE(std::isnan(acc[0]) == std::isnan(ff.first));
		EXPECT_TRUE(std::isnan(acc[1]) == std::isnan(ff.second));
		EXPECT_TRUE(std::isinf(acc[0]) == std::isinf(ff.first));
	}
}

TEST(SetpointLimiterTest, ThreeCallsWithTheSameInputsGiveTheSameResult)
{
	// PX4 calls the position controller up to three times per cycle, each time with the setpoint it sets
	cp_brake::HookConstraint c = openConstraint(0.4f);
	c.caps_m_s[2] = 2.f;
	c.caps_m_s[20] = 3.f;
	float first_vel[2] = {0.f, 0.f};
	float first_acc[2] = {0.f, 0.f};

	for (int call = 0; call < 3; ++call) {
		float vel[2] = {7.f, 3.f};
		float acc[2] = {1.5f, -0.5f};
		const cp_brake::HookResult r = cp_brake::limitHorizontalSetpoint(flightContext(), c, kHover, vel, acc);
		EXPECT_EQ(r.action, cp_brake::HookAction::kLimited);

		if (call == 0) {
			first_vel[0] = vel[0];
			first_vel[1] = vel[1];
			first_acc[0] = acc[0];
			first_acc[1] = acc[1];

		} else {
			EXPECT_EQ(vel[0], first_vel[0]);
			EXPECT_EQ(vel[1], first_vel[1]);
			EXPECT_EQ(acc[0], first_acc[0]);
			EXPECT_EQ(acc[1], first_acc[1]);
		}
	}

	EXPECT_EQ(first_acc[0], 0.f);
	EXPECT_EQ(first_acc[1], 0.f);
}

TEST(SetpointLimiterTest, TheFeedForwardIsClearedExactlyWhenTheVelocityIsLimited)
{
	Lcg rng{4242U};
	const cp_brake::HookContext context = flightContext();

	for (int trial = 0; trial < 3000; ++trial) {
		const float heading = (rng.next() - 0.5f) * 14.f;
		cp_brake::HookConstraint c = openConstraint(heading);

		for (std::size_t i = 0; i < cp_brake::kSectorCount; ++i) {
			const float pick = rng.next();
			c.caps_m_s[i] = (pick < 0.3f) ? 0.f : ((pick < 0.6f) ? rng.next() * 12.f : 12.f);
		}

		const float speed = rng.next() * 12.f;
		const float angle = rng.next() * 2.f * kPi;
		float vel[2] = {speed * cosf(angle), speed * sinf(angle)};
		const float vel_in[2] = {vel[0], vel[1]};
		const float acc_in[2] = {(rng.next() - 0.5f) * 8.f, (rng.next() - 0.5f) * 8.f};
		float acc[2] = {acc_in[0], acc_in[1]};
		const cp_brake::HookResult r = cp_brake::limitHorizontalSetpoint(context, c, kHover, vel, acc);
		SCOPED_TRACE(::testing::Message() << "trial " << trial);

		if (r.action == cp_brake::HookAction::kLimited) {
			EXPECT_EQ(acc[0], 0.f);
			EXPECT_EQ(acc[1], 0.f);

		} else {
			EXPECT_EQ(r.action, cp_brake::HookAction::kClear);
			EXPECT_EQ(acc[0], acc_in[0]);
			EXPECT_EQ(acc[1], acc_in[1]);
			EXPECT_EQ(vel[0], vel_in[0]);
			EXPECT_EQ(vel[1], vel_in[1]);
		}
	}
}

// SYS-CP-031: the speed lock outside the forward class and the unclaimed directions

TEST(SetpointLimiterTest, TheLockScalesASideSetpointToTheLockAndKeepsItsDirection)
{
	const cp_brake::HookConstraint c = openConstraint(0.f);
	float vel[2] = {0.f, 8.f}; // east, 90 degrees from the nose
	float acc[2] = {1.f, 1.f};
	const cp_brake::HookResult r = cp_brake::limitHorizontalSetpoint(classContext(0.f), c, kHover, vel, acc);

	EXPECT_EQ(r.action, cp_brake::HookAction::kLimited);
	EXPECT_EQ(r.active_sector, cp_brake::kNoSector); // the lock, not a sector
	EXPECT_NEAR(vel[0], 0.f, 1e-6f);
	EXPECT_NEAR(vel[1], 5.f, 1e-5f);
	EXPECT_EQ(acc[0], 0.f); // limited: the feed-forward is cleared (ADR 0020)
	EXPECT_EQ(acc[1], 0.f);
	EXPECT_EQ(r.reason_flags & cp_brake::kReasonUnclaimedDirection, 0U);
}

TEST(SetpointLimiterTest, TheForwardClassIsNotLockedAndItsBoundaryIsForward)
{
	const cp_brake::HookConstraint c = openConstraint(0.f);

	for (const float deg : {0.f, 30.f, 45.f, -45.f}) {
		const float rad = deg * kPi / 180.f;
		float vel[2] = {12.f * cosf(rad), 12.f * sinf(rad)};
		float acc[2] = {NAN, NAN};
		const cp_brake::HookResult r = cp_brake::limitHorizontalSetpoint(classContext(0.f), c, kHover, vel, acc);
		EXPECT_EQ(r.action, cp_brake::HookAction::kClear) << deg;
		EXPECT_NEAR(vel[0] * vel[0] + vel[1] * vel[1], 144.f, 1e-3f) << deg;
	}

	for (const float deg : {46.f, -46.f, 90.f, 134.f, 180.f}) {
		const float rad = deg * kPi / 180.f;
		float vel[2] = {12.f * cosf(rad), 12.f * sinf(rad)};
		float acc[2] = {NAN, NAN};
		const cp_brake::HookResult r = cp_brake::limitHorizontalSetpoint(classContext(0.f), c, kHover, vel, acc);
		EXPECT_EQ(r.action, cp_brake::HookAction::kLimited) << deg;
		EXPECT_NEAR(sqrtf(vel[0] * vel[0] + vel[1] * vel[1]), 5.f, 1e-4f) << deg;
		EXPECT_NEAR(atan2f(vel[1], vel[0]), atan2f(sinf(rad), cosf(rad)), 1e-5f) << deg; // direction kept
	}
}

TEST(SetpointLimiterTest, TheLockUsesTheCurrentHeadingNotTheOneOfTheSample)
{
	// the caps rotate with the heading of the sample (0 here); the lock follows the nose now (east)
	const cp_brake::HookConstraint c = openConstraint(0.f);
	const float east = kPi / 2.f;

	float along_nose[2] = {0.f, 8.f};
	float acc[2] = {NAN, NAN};
	EXPECT_EQ(cp_brake::limitHorizontalSetpoint(classContext(east), c, kHover, along_nose, acc).action,
		  cp_brake::HookAction::kClear);
	EXPECT_NEAR(along_nose[1], 8.f, 1e-6f);

	float sideways[2] = {8.f, 0.f}; // north is 90 degrees from a nose pointing east
	EXPECT_EQ(cp_brake::limitHorizontalSetpoint(classContext(east), c, kHover, sideways, acc).action,
		  cp_brake::HookAction::kLimited);
	EXPECT_NEAR(sideways[0], 5.f, 1e-5f);
}

TEST(SetpointLimiterTest, TheCurrentVelocityLocksOnlyWhileItIsFasterThanTheLock)
{
	const cp_brake::HookConstraint c = openConstraint(0.f);
	float acc[2] = {NAN, NAN};

	// the setpoint is forward; the vehicle still slides east at 4 m/s, inside the lock: not locked
	const float slow_slide[2] = {0.f, 4.f};
	float vel[2] = {10.f, 0.f};
	EXPECT_EQ(cp_brake::limitHorizontalSetpoint(classContext(0.f), c, slow_slide, vel, acc).action,
		  cp_brake::HookAction::kClear);
	EXPECT_NEAR(vel[0], 10.f, 1e-6f);

	// at 6 m/s east the vehicle is beyond the lock sideways: the setpoint is scaled to the lock
	const float fast_slide[2] = {0.f, 6.f};
	vel[0] = 10.f;
	vel[1] = 0.f;
	EXPECT_EQ(cp_brake::limitHorizontalSetpoint(classContext(0.f), c, fast_slide, vel, acc).action,
		  cp_brake::HookAction::kLimited);
	EXPECT_NEAR(vel[0], 5.f, 1e-5f);
}

TEST(SetpointLimiterTest, ASectorThatLimitsMoreThanTheLockKeepsItsSector)
{
	cp_brake::HookConstraint c = openConstraint(0.f);
	c.caps_m_s[18] = 2.f; // east
	float vel[2] = {0.f, 8.f};
	float acc[2] = {NAN, NAN};
	const cp_brake::HookResult r = cp_brake::limitHorizontalSetpoint(classContext(0.f), c, kHover, vel, acc);

	EXPECT_EQ(r.action, cp_brake::HookAction::kLimited);
	EXPECT_EQ(r.active_sector, 18);
	EXPECT_NEAR(vel[1], 2.f, 1e-5f);
}

TEST(SetpointLimiterTest, AnUnclaimedDirectionIsReportedAndLimitsNothingMore)
{
	const cp_brake::HookConstraint c = openConstraint(0.f);

	for (const float deg : {140.f, 180.f, -140.f}) {
		const float rad = deg * kPi / 180.f;
		float vel[2] = {3.f * cosf(rad), 3.f * sinf(rad)}; // below the lock
		float acc[2] = {NAN, NAN};
		const cp_brake::HookResult r = cp_brake::limitHorizontalSetpoint(classContext(0.f), c, kHover, vel, acc);
		EXPECT_EQ(r.action, cp_brake::HookAction::kClear) << deg;
		EXPECT_NE(r.reason_flags & cp_brake::kReasonUnclaimedDirection, 0U) << deg;
		EXPECT_NEAR(sqrtf(vel[0] * vel[0] + vel[1] * vel[1]), 3.f, 1e-5f) << deg;
	}

	for (const float deg : {0.f, 90.f, 130.f, -130.f}) {
		const float rad = deg * kPi / 180.f;
		float vel[2] = {3.f * cosf(rad), 3.f * sinf(rad)};
		float acc[2] = {NAN, NAN};
		const cp_brake::HookResult r = cp_brake::limitHorizontalSetpoint(classContext(0.f), c, kHover, vel, acc);
		EXPECT_EQ(r.reason_flags & cp_brake::kReasonUnclaimedDirection, 0U) << deg;
	}

	// at or below 0.2 m/s the setpoint direction is hover noise and is not reported; above it, it is
	for (const float speed : {0.02f, 0.19f, 0.21f, 1.f}) {
		float slow[2] = {-speed, 0.f};
		float acc[2] = {NAN, NAN};
		const cp_brake::HookResult r = cp_brake::limitHorizontalSetpoint(classContext(0.f), c, kHover, slow, acc);
		EXPECT_EQ((r.reason_flags & cp_brake::kReasonUnclaimedDirection) != 0U, speed > 0.2f) << speed;
		EXPECT_NEAR(slow[0], -speed, 1e-6f) << speed; // the report limits nothing
	}

	// a forward setpoint while the vehicle still moves backward faster than the lock: reported and locked
	const float backward[2] = {-6.f, 0.f};
	float vel[2] = {8.f, 0.f};
	float acc[2] = {NAN, NAN};
	const cp_brake::HookResult r = cp_brake::limitHorizontalSetpoint(classContext(0.f), c, backward, vel, acc);
	EXPECT_NE(r.reason_flags & cp_brake::kReasonUnclaimedDirection, 0U);
	EXPECT_EQ(r.action, cp_brake::HookAction::kLimited);
	EXPECT_NEAR(vel[0], 5.f, 1e-5f);
}

TEST(SetpointLimiterTest, UnusableLockInputsFailClosed)
{
	const cp_brake::HookConstraint c = openConstraint(0.f);
	float acc[2] = {NAN, NAN};

	// no usable heading: every direction is locked, forward included
	float forward[2] = {8.f, 0.f};
	EXPECT_EQ(cp_brake::limitHorizontalSetpoint(classContext(NAN), c, kHover, forward, acc).action,
		  cp_brake::HookAction::kLimited);
	EXPECT_NEAR(forward[0], 5.f, 1e-5f);

	// no usable lock speed: the locked directions get 0, the forward class is unchanged
	for (const float lock : {NAN, -1.f, 0.f}) {
		cp_brake::HookContext context = classContext(0.f);
		context.side_speed_m_s = lock;
		float side[2] = {0.f, 3.f};
		EXPECT_EQ(cp_brake::limitHorizontalSetpoint(context, c, kHover, side, acc).action, cp_brake::HookAction::kLimited)
				<< lock;
		EXPECT_NEAR(side[1], 0.f, 1e-6f) << lock;

		float ahead[2] = {3.f, 0.f};
		EXPECT_EQ(cp_brake::limitHorizontalSetpoint(context, c, kHover, ahead, acc).action, cp_brake::HookAction::kClear)
				<< lock;
	}

	// no usable start of the unclaimed bearings: every direction outside the forward class is reported
	for (const float angle : {NAN, -1.f, 45.f, 200.f}) {
		cp_brake::HookContext context = classContext(0.f);
		context.unclaimed_start_deg = angle;
		float side[2] = {0.f, 3.f};
		EXPECT_NE(cp_brake::limitHorizontalSetpoint(context, c, kHover, side, acc).reason_flags
			  & cp_brake::kReasonUnclaimedDirection, 0U) << angle;
	}
}

TEST(SetpointLimiterTest, TheLockHoldsForAnyInputAndNeverTurnsOrLengthensTheSetpoint)
{
	Lcg rng{777U};

	for (int trial = 0; trial < 3000; ++trial) {
		const float heading = (rng.next() - 0.5f) * 14.f;
		const float nose_heading = heading + (rng.next() - 0.5f) * 0.6f;
		cp_brake::HookConstraint c = openConstraint(heading);

		for (std::size_t i = 0; i < cp_brake::kSectorCount; ++i) {
			c.caps_m_s[i] = (rng.next() < 0.2f) ? rng.next() * 12.f : 12.f;
		}

		const float speed = rng.next() * 12.f;
		const float angle = rng.next() * 2.f * kPi;
		float vel[2] = {speed * cosf(angle), speed * sinf(angle)};
		const float vel_in[2] = {vel[0], vel[1]};
		float acc[2] = {NAN, NAN};
		cp_brake::limitHorizontalSetpoint(classContext(nose_heading), c, kHover, vel, acc);
		SCOPED_TRACE(::testing::Message() << "trial " << trial);

		const float out = sqrtf(vel[0] * vel[0] + vel[1] * vel[1]);
		EXPECT_LE(out, speed + 1e-4f);
		EXPECT_GE(dot(vel, vel_in), -1e-4f);           // never reversed
		EXPECT_NEAR(cross(vel, vel_in), 0.f, 1e-3f);   // never turned

		const float nose[2] = {cosf(nose_heading), sinf(nose_heading)};
		const float cos_angle = (speed > 0.f) ? dot(vel_in, nose) / speed : 1.f;

		if (cos_angle < cosf(45.5f * kPi / 180.f)) {
			EXPECT_LE(out, 5.f + 1e-3f); // outside the forward class: at most the lock
		}
	}
}
