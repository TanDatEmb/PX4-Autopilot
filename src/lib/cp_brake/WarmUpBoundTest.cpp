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

#include <gtest/gtest.h>

#include <limits>

#include "WarmUpBound.hpp"

using cp_brake::WarmUpBound;

namespace
{
constexpr uint64_t kT0_us{10'000'000U};
constexpr float kLimit_s{0.5f};
}

TEST(WarmUpBoundTest, NotWarmingGivesNoExemption)
{
	WarmUpBound bound;
	EXPECT_FALSE(bound.update(false, kT0_us, kLimit_s));
}

TEST(WarmUpBoundTest, TheExemptionHoldsUpToTheLimitFromTheFirstWarmingEvaluation)
{
	WarmUpBound bound;
	EXPECT_TRUE(bound.update(true, kT0_us, kLimit_s));
	EXPECT_TRUE(bound.update(true, kT0_us + 250'000U, kLimit_s));
	EXPECT_TRUE(bound.update(true, kT0_us + 500'000U, kLimit_s));
	EXPECT_FALSE(bound.update(true, kT0_us + 500'001U, kLimit_s));
	EXPECT_FALSE(bound.update(true, kT0_us + 5'000'000U, kLimit_s)); // and it stays over while warming
}

TEST(WarmUpBoundTest, ARefreshedMapDoesNotRestartTheClock)
{
	// the caller passes only the evaluation time, never the map time: a continuous warm-up always ends
	WarmUpBound bound;
	bool allowed = true;
	uint64_t t = kT0_us;

	for (int i = 0; i < 1000 && allowed; ++i) {
		allowed = bound.update(true, t, kLimit_s);
		t += 50'000U;
	}

	EXPECT_FALSE(allowed);
}

TEST(WarmUpBoundTest, ANotWarmingEvaluationEndsTheEpisodeAndTheNextWarmUpStartsANewOne)
{
	WarmUpBound bound;
	EXPECT_TRUE(bound.update(true, kT0_us, kLimit_s));
	EXPECT_FALSE(bound.update(true, kT0_us + 600'000U, kLimit_s));
	EXPECT_FALSE(bound.update(false, kT0_us + 700'000U, kLimit_s));
	EXPECT_TRUE(bound.update(true, kT0_us + 800'000U, kLimit_s));
	EXPECT_TRUE(bound.update(true, kT0_us + 1'300'000U, kLimit_s));
	EXPECT_FALSE(bound.update(true, kT0_us + 1'300'001U, kLimit_s));
}

TEST(WarmUpBoundTest, AnUnusableLimitGivesNoExemption)
{
	WarmUpBound bound;
	EXPECT_FALSE(bound.update(true, kT0_us, -1.f));
	EXPECT_FALSE(bound.update(true, kT0_us, 0.f));
	EXPECT_FALSE(bound.update(true, kT0_us, std::numeric_limits<float>::quiet_NaN()));
	EXPECT_FALSE(bound.update(true, kT0_us, std::numeric_limits<float>::infinity()));
	EXPECT_TRUE(bound.update(true, kT0_us, kLimit_s));
}

TEST(WarmUpBoundTest, ABackwardsClockNeverExtendsTheExemption)
{
	WarmUpBound bound;
	EXPECT_TRUE(bound.update(true, kT0_us, kLimit_s));
	EXPECT_FALSE(bound.update(true, kT0_us - 1U, kLimit_s));
}

TEST(WarmUpBoundTest, ResetEndsTheEpisode)
{
	WarmUpBound bound;
	EXPECT_TRUE(bound.update(true, kT0_us, kLimit_s));
	EXPECT_FALSE(bound.update(true, kT0_us + 600'000U, kLimit_s));
	bound.reset();
	EXPECT_TRUE(bound.update(true, kT0_us + 700'000U, kLimit_s));
}
