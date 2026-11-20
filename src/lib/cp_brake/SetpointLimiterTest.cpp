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
 * Unit tests of the limiter hook function (collision-prevention ICD-CP-04, ADR 0008): the geometry
 * of every sector at several headings, and the properties that must hold for any input. The speeds
 * and caps are test fixtures, not approved values.
 */

#include <gtest/gtest.h>

#include <cmath>
#include <cstdint>

#include "ReasonFlags.hpp"
#include "SetpointLimiter.hpp"
#include "StateMachine.hpp"

namespace
{

constexpr float kPi{3.14159265358979f};
constexpr float kSectorRad{2.f * kPi / 72.f};
constexpr uint64_t kNowUs{10000000U};

cp_brake::HookContext flightContext()
{
	return cp_brake::HookContext{true, true, true, kNowUs, 0.5f};
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

	for (std::size_t i = 0; i < cp_brake::kSectorCount; ++i) {
		c.caps_m_s[i] = 12.f;
	}

	return c;
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

TEST(SetpointLimiterTest, EverySectorLimitsTheVelocityAlongItsOwnDirectionAtEveryHeading)
{
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
			const float state[2] = {0.f, 0.f};

			const cp_brake::HookResult r = cp_brake::limitHorizontalSetpoint(flightContext(), c, state, vel, acc);
			EXPECT_EQ(r.action, cp_brake::HookAction::kLimited);
			EXPECT_EQ(r.active_sector, sector);
			EXPECT_NEAR(vel[0] * u[0] + vel[1] * u[1], 3.f, 1e-3f);
			EXPECT_NEAR(vel[0] * perp[0] + vel[1] * perp[1], 2.f, 1e-3f);
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
		const float state[2] = {rng.next() * 6.f, rng.next() * 6.f};

		const cp_brake::HookResult r = cp_brake::limitHorizontalSetpoint(context, c, state, vel, acc);
		SCOPED_TRACE(::testing::Message() << "trial " << trial);
		ASSERT_TRUE(std::isfinite(vel[0]) && std::isfinite(vel[1]) && std::isfinite(acc[0]) && std::isfinite(acc[1]));
		EXPECT_TRUE(r.action == cp_brake::HookAction::kClear || r.action == cp_brake::HookAction::kLimited);

		// never faster than asked
		EXPECT_LE(std::hypot(vel[0], vel[1]), speed + 1e-4f);

		// every sector is respected
		for (int sector = 0; sector < static_cast<int>(cp_brake::kSectorCount); ++sector) {
			float u[2];
			direction(sector, heading, u);
			EXPECT_LE(vel[0] * u[0] + vel[1] * u[1], usableCap(c.caps_m_s[sector]) + 1e-3f) << "sector " << sector;
		}

		// a second application changes nothing: the hook is a pure function of the setpoint it is given
		float vel_again[2] = {vel[0], vel[1]};
		float acc_again[2] = {acc[0], acc[1]};
		cp_brake::limitHorizontalSetpoint(context, c, state, vel_again, acc_again);
		EXPECT_NEAR(vel_again[0], vel[0], 2e-3f);
		EXPECT_NEAR(vel_again[1], vel[1], 2e-3f);

		// the same inputs give the same bits
		float vel_first[2] = {in[0], in[1]};
		float vel_second[2] = {in[0], in[1]};
		float acc_first[2] = {acc[0], acc[1]};
		float acc_second[2] = {acc[0], acc[1]};
		cp_brake::limitHorizontalSetpoint(context, c, state, vel_first, acc_first);
		cp_brake::limitHorizontalSetpoint(context, c, state, vel_second, acc_second);
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
	const float state[2] = {NAN, NAN};

	const cp_brake::HookResult r = cp_brake::limitHorizontalSetpoint(flightContext(), c, state, vel, acc);
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
	const float state[2] = {0.f, 0.f};

	context.enabled = false;
	{
		float vel[2] = {8.f, 1.f};
		float acc[2] = {1.f, 1.f};
		const cp_brake::HookResult r = cp_brake::limitHorizontalSetpoint(context, c, state, vel, acc);
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
		const cp_brake::HookResult r = cp_brake::limitHorizontalSetpoint(context, c, state, vel, acc);
		EXPECT_EQ(r.action, cp_brake::HookAction::kTransparent);
		EXPECT_EQ(vel[0], 8.f);
	}

	context = flightContext();
	context.nav_state_enforced = false;
	{
		float vel[2] = {8.f, 1.f};
		float acc[2] = {1.f, 1.f};
		const cp_brake::HookResult r = cp_brake::limitHorizontalSetpoint(context, c, state, vel, acc);
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
