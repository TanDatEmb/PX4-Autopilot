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
 * @file CapLimiterTest.cpp
 *
 * Unit tests of the cp_brake dwell and rise limiter. The parameters used here are test fixtures
 * (0.3 s and 2.0 m/s^2 are the design baselines of the architecture, PERF-05).
 */

#include <gtest/gtest.h>

#include <cmath>
#include <limits>

#include "CapLimiter.hpp"

using namespace cp_brake;

namespace
{
constexpr float kDt{0.1f};

LimiterParams fixture()
{
	return LimiterParams{0.3f, 2.f};
}

SectorCaps rawAll(float value)
{
	SectorCaps caps;
	caps.fill(value);
	return caps;
}

SectorFlags validAll(bool value = true)
{
	SectorFlags flags;
	flags.fill(value);
	return flags;
}
} // namespace

TEST(CapLimiterTest, EverySectorStartsAtZero)
{
	CapLimiter limiter(fixture());

	for (std::size_t i = 0; i < kSectorCount; ++i) {
		EXPECT_EQ(limiter.caps()[i], 0.f);
	}
}

TEST(CapLimiterTest, ARiseNeedsTheDwellAndIsThenBoundedByTheRate)
{
	CapLimiter limiter(fixture());
	const SectorCaps raw = rawAll(1.f);

	// the dwell is 0.3 s: the first two steps of 0.1 s change nothing
	EXPECT_EQ(limiter.update(raw, validAll(), kDt)[0], 0.f);
	EXPECT_EQ(limiter.update(raw, validAll(), kDt)[0], 0.f);

	// at the third step the dwell is reached and the cap rises by 2 m/s^2 * 0.1 s
	EXPECT_NEAR(limiter.update(raw, validAll(), kDt)[0], 0.2f, 1e-6f);

	float previous = limiter.caps()[0];

	for (int i = 0; i < 20; ++i) {
		const float cap = limiter.update(raw, validAll(), kDt)[0];
		EXPECT_LE(cap - previous, 0.2f + 1e-6f);
		EXPECT_LE(cap, 1.f);
		previous = cap;
	}

	EXPECT_NEAR(previous, 1.f, 1e-6f);
}

TEST(CapLimiterTest, ANewObstacleTightensTheCapAtOnce)
{
	CapLimiter limiter(fixture());

	for (int i = 0; i < 30; ++i) {
		limiter.update(rawAll(5.f), validAll(), kDt);
	}

	EXPECT_NEAR(limiter.caps()[0], 5.f, 1e-5f);

	EXPECT_EQ(limiter.update(rawAll(1.5f), validAll(), kDt)[0], 1.5f);
	EXPECT_EQ(limiter.update(rawAll(0.f), validAll(), kDt)[0], 0.f);
}

TEST(CapLimiterTest, InvalidDataResetsTheDwell)
{
	CapLimiter limiter(fixture());
	const SectorCaps raw = rawAll(1.f);

	limiter.update(raw, validAll(), kDt);
	limiter.update(raw, validAll(), kDt);
	limiter.update(raw, validAll(false), kDt); // unknown or stale data: dwell starts again

	EXPECT_EQ(limiter.update(raw, validAll(), kDt)[0], 0.f);
	EXPECT_EQ(limiter.update(raw, validAll(), kDt)[0], 0.f);
	EXPECT_NEAR(limiter.update(raw, validAll(), kDt)[0], 0.2f, 1e-6f);
}

TEST(CapLimiterTest, ATighteningResetsTheDwellOfThatSector)
{
	CapLimiter limiter(fixture());

	for (int i = 0; i < 30; ++i) {
		limiter.update(rawAll(2.f), validAll(), kDt);
	}

	limiter.update(rawAll(1.f), validAll(), kDt);                 // tightens to 1.0 m/s
	EXPECT_EQ(limiter.update(rawAll(2.f), validAll(), kDt)[0], 1.f); // dwell 0.1 s
	EXPECT_EQ(limiter.update(rawAll(2.f), validAll(), kDt)[0], 1.f); // dwell 0.2 s
	EXPECT_NEAR(limiter.update(rawAll(2.f), validAll(), kDt)[0], 1.2f, 1e-6f);
}

TEST(CapLimiterTest, SectorsAreIndependent)
{
	CapLimiter limiter(fixture());
	SectorCaps raw = rawAll(1.f);
	SectorFlags valid = validAll();
	valid[10] = false;
	raw[11] = 0.f;

	for (int i = 0; i < 5; ++i) {
		limiter.update(raw, valid, kDt);
	}

	EXPECT_GT(limiter.caps()[0], 0.f);
	EXPECT_EQ(limiter.caps()[10], 0.f);
	EXPECT_EQ(limiter.caps()[11], 0.f);
}

TEST(CapLimiterTest, NeverAboveTheRawCapAndNeverNegative)
{
	CapLimiter limiter(fixture());

	for (int i = 0; i < 100; ++i) {
		const SectorCaps raw = rawAll(0.37f * static_cast<float>(i % 7));
		const SectorCaps &caps = limiter.update(raw, validAll(), kDt);

		for (std::size_t s = 0; s < kSectorCount; ++s) {
			EXPECT_LE(caps[s], raw[s]);
			EXPECT_GE(caps[s], 0.f);
		}
	}
}

TEST(CapLimiterTest, InvalidInputsCanOnlyTighten)
{
	const float nan = std::numeric_limits<float>::quiet_NaN();
	const SectorCaps raw = rawAll(1.f);

	{
		CapLimiter limiter(fixture());

		for (const float bad_dt : {0.f, -0.1f, nan}) {
			for (int i = 0; i < 10; ++i) {
				EXPECT_EQ(limiter.update(raw, validAll(), bad_dt)[0], 0.f);
			}
		}
	}

	for (const LimiterParams bad : {LimiterParams{nan, 2.f}, LimiterParams{0.3f, 0.f}, LimiterParams{-1.f, 2.f}}) {
		EXPECT_FALSE(isValid(bad));
		CapLimiter limiter(bad);

		for (int i = 0; i < 10; ++i) {
			EXPECT_EQ(limiter.update(raw, validAll(), kDt)[0], 0.f);
		}
	}

	CapLimiter limiter(fixture());

	for (int i = 0; i < 30; ++i) {
		limiter.update(raw, validAll(), kDt);
	}

	EXPECT_EQ(limiter.update(rawAll(nan), validAll(), kDt)[0], 0.f);
}

TEST(CapLimiterTest, SampleStepIsTheTimeBetweenSamplesWithinTheAgeLimit)
{
	EXPECT_FLOAT_EQ(sampleStep(1000000U, 1050000U, 0.5f), 0.05f);
	EXPECT_FLOAT_EQ(sampleStep(1000000U, 1400000U, 0.5f), 0.4f);
}

TEST(CapLimiterTest, SampleStepHasNoStepWithoutAUsableInterval)
{
	const float nan = std::numeric_limits<float>::quiet_NaN();

	EXPECT_LT(sampleStep(0U, 1050000U, 0.5f), 0.f);        // no previous sample
	EXPECT_LT(sampleStep(1050000U, 1050000U, 0.5f), 0.f);  // time did not advance
	EXPECT_LT(sampleStep(1050000U, 1000000U, 0.5f), 0.f);  // time went back
	EXPECT_LT(sampleStep(1000000U, 1600000U, 0.5f), 0.f);  // longer than the age limit
	EXPECT_LT(sampleStep(1000000U, 4000000U, 0.5f), 0.f);

	for (const float bad_limit : {0.f, -0.5f, nan}) {
		EXPECT_LT(sampleStep(1000000U, 1050000U, bad_limit), 0.f);
	}
}

TEST(CapLimiterTest, AGapLongerThanTheAgeLimitDoesNotRaiseACap)
{
	CapLimiter limiter(fixture());
	const SectorCaps raw = rawAll(10.f);
	uint64_t now_us = 1000000U;

	for (int i = 0; i < 30; ++i) { // 1.5 s of data at 20 Hz
		const uint64_t previous_us = now_us;
		now_us += 50000U;
		limiter.update(raw, validAll(), sampleStep(previous_us, now_us, 0.5f));
	}

	const float before = limiter.caps()[0];
	EXPECT_GT(before, 0.f);
	EXPECT_LT(before, 10.f);

	// no data for 3 s, then a clear map: the gap gave no supporting data, so nothing rises
	const uint64_t previous_us = now_us;
	now_us += 3000000U;
	EXPECT_EQ(limiter.update(raw, validAll(), sampleStep(previous_us, now_us, 0.5f))[0], before);

	// the dwell starts again: no rise for 0.2 s of data, then the normal rate
	EXPECT_EQ(limiter.update(raw, validAll(), kDt)[0], before);
	EXPECT_EQ(limiter.update(raw, validAll(), kDt)[0], before);
	EXPECT_NEAR(limiter.update(raw, validAll(), kDt)[0], before + 0.2f, 1e-5f);
}

TEST(CapLimiterTest, TightenLowersACapAndRestartsItsDwell)
{
	CapLimiter limiter(fixture());
	const SectorCaps raw = rawAll(1.f);

	// reach a cap of 0.2 after the dwell, then tighten to 0.1
	limiter.update(raw, validAll(), kDt);
	limiter.update(raw, validAll(), kDt);
	limiter.update(raw, validAll(), kDt);
	ASSERT_NEAR(limiter.caps()[0], 0.2f, 1e-6f);

	limiter.tighten(rawAll(0.1f));
	EXPECT_NEAR(limiter.caps()[0], 0.1f, 1e-6f);

	// the dwell started again: two steps of 0.1 s change nothing, the third rises
	EXPECT_NEAR(limiter.update(raw, validAll(), kDt)[0], 0.1f, 1e-6f);
	EXPECT_NEAR(limiter.update(raw, validAll(), kDt)[0], 0.1f, 1e-6f);
	EXPECT_NEAR(limiter.update(raw, validAll(), kDt)[0], 0.3f, 1e-6f);
}

TEST(CapLimiterTest, TightenNeverRaisesACapAndKeepsTheDwellOfAnUntouchedSector)
{
	CapLimiter limiter(fixture());
	const SectorCaps raw = rawAll(1.f);

	// two steps of dwell, then a tighten call that would only allow more: nothing changes
	limiter.update(raw, validAll(), kDt);
	limiter.update(raw, validAll(), kDt);
	limiter.tighten(rawAll(5.f));
	EXPECT_EQ(limiter.caps()[0], 0.f);

	// the dwell was kept: the next step reaches it and the cap rises
	EXPECT_NEAR(limiter.update(raw, validAll(), kDt)[0], 0.2f, 1e-6f);
}

TEST(CapLimiterTest, TightenReadsAnInvalidRawCapAsZeroAndSectorsAreIndependent)
{
	CapLimiter limiter(fixture());
	SectorCaps raw = rawAll(1.f);

	for (int i = 0; i < 6; ++i) {
		limiter.update(raw, validAll(), kDt);
	}

	ASSERT_GT(limiter.caps()[0], 0.f);
	ASSERT_GT(limiter.caps()[1], 0.f);
	raw[0] = std::numeric_limits<float>::quiet_NaN();
	raw[1] = -1.f;
	raw[2] = 100.f;
	const float untouched = limiter.caps()[2];
	limiter.tighten(raw);
	EXPECT_EQ(limiter.caps()[0], 0.f);
	EXPECT_EQ(limiter.caps()[1], 0.f);
	EXPECT_EQ(limiter.caps()[2], untouched);
}
