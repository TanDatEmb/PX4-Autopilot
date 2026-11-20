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
 * @file MapTimelineTest.cpp
 *
 * Unit tests of the map timeline of cp_brake: which map is the current sample, and which time step
 * the limiter may credit (issue #94, Codex review of fork PR #7). The 0.5 s age limit is the
 * approved baseline CPB_AGE_STOP (PERF-04); the dwell and the rise rate are test fixtures.
 */

#include <gtest/gtest.h>

#include <cstdint>

#include "CapLimiter.hpp"
#include "MapTimeline.hpp"

using namespace cp_brake;

namespace
{
constexpr float kAgeLimit_s{0.5f};
constexpr uint64_t kT_us{1000000U};         // 1.0 s
constexpr uint64_t kMap_us{100000U};        // maps at 10 Hz
} // namespace

TEST(MapTimelineTest, TheFirstMapHasNoStepAndBecomesTheSample)
{
	MapTimeline timeline;

	EXPECT_EQ(timeline.sampleTimestamp(), 0U);
	EXPECT_FLOAT_EQ(timeline.onMap(kT_us, true, kAgeLimit_s), -1.f);
	EXPECT_EQ(timeline.sampleTimestamp(), kT_us);
}

TEST(MapTimelineTest, TwoUsedMapsGiveTheTimeBetweenThem)
{
	MapTimeline timeline;
	timeline.onMap(kT_us, true, kAgeLimit_s);

	EXPECT_NEAR(timeline.onMap(kT_us + kMap_us, true, kAgeLimit_s), 0.1f, 1e-6f);
	EXPECT_NEAR(timeline.onMap(kT_us + 2U * kMap_us, true, kAgeLimit_s), 0.1f, 1e-6f);
	EXPECT_EQ(timeline.sampleTimestamp(), kT_us + 2U * kMap_us);
}

TEST(MapTimelineTest, AGapLongerThanTheAgeLimitGivesNoStep)
{
	MapTimeline timeline;
	timeline.onMap(kT_us, true, kAgeLimit_s);

	EXPECT_FLOAT_EQ(timeline.onMap(kT_us + 600000U, true, kAgeLimit_s), -1.f);
}

TEST(MapTimelineTest, AMapThatIsNotUsedDoesNotRefreshTheSample)
{
	MapTimeline timeline;
	timeline.onMap(kT_us, true, kAgeLimit_s);
	timeline.onMap(kT_us + 1U * kMap_us, false, kAgeLimit_s);
	timeline.onMap(kT_us + 2U * kMap_us, false, kAgeLimit_s);

	EXPECT_EQ(timeline.sampleTimestamp(), kT_us);   // it keeps ageing, so the stale rule applies
}

TEST(MapTimelineTest, TheFirstUsedMapAfterUnusedOnesHasNoStep)
{
	MapTimeline timeline;
	timeline.onMap(kT_us, true, kAgeLimit_s);
	timeline.onMap(kT_us + 1U * kMap_us, false, kAgeLimit_s);
	timeline.onMap(kT_us + 2U * kMap_us, false, kAgeLimit_s);
	timeline.onMap(kT_us + 3U * kMap_us, false, kAgeLimit_s);

	// the outage is not supporting data: the step must not reach back over it
	EXPECT_FLOAT_EQ(timeline.onMap(kT_us + 4U * kMap_us, true, kAgeLimit_s), -1.f);
	EXPECT_EQ(timeline.sampleTimestamp(), kT_us + 4U * kMap_us);

	// and the next one counts from the first map that was used again
	EXPECT_NEAR(timeline.onMap(kT_us + 5U * kMap_us, true, kAgeLimit_s), 0.1f, 1e-6f);
}

TEST(MapTimelineTest, ASingleUnusedMapBreaksTheStepToo)
{
	MapTimeline timeline;
	timeline.onMap(kT_us, true, kAgeLimit_s);
	timeline.onMap(kT_us + 1U * kMap_us, false, kAgeLimit_s);

	EXPECT_FLOAT_EQ(timeline.onMap(kT_us + 2U * kMap_us, true, kAgeLimit_s), -1.f);
}

TEST(MapTimelineTest, AnUnusableAgeLimitGivesNoStep)
{
	MapTimeline timeline;
	timeline.onMap(kT_us, true, 0.f);

	EXPECT_FLOAT_EQ(timeline.onMap(kT_us + kMap_us, true, 0.f), -1.f);
	EXPECT_FLOAT_EQ(timeline.onMap(kT_us + 2U * kMap_us, true, -1.f), -1.f);
}

TEST(MapTimelineTest, ACapDoesNotRiseUntilTheFullDwellHasRunAgainAfterAnOutage)
{
	// the case of the review: a good map, three maps not used, then the time sync recovers
	const LimiterParams params{0.3f, 2.f};   // dwell 0.3 s, rise 2 m/s^2: test fixtures
	CapLimiter limiter(params);
	MapTimeline timeline;
	SectorCaps raw{};
	SectorFlags valid{};
	raw.fill(0.f);
	valid.fill(false);
	raw[0] = 3.f;
	valid[0] = true;

	auto feed = [&](uint64_t t_us, bool used) {
		SectorCaps given = raw;
		SectorFlags known = valid;

		if (!used) {
			given.fill(0.f);       // an unused map: every sector unknown, never free
			known.fill(false);
		}

		const float dt_s = timeline.onMap(t_us, used, kAgeLimit_s);
		return limiter.update(given, known, dt_s)[0];
	};

	// before the outage the cap has risen: first map 0, then the dwell of 0.3 s, then the rise
	EXPECT_FLOAT_EQ(feed(kT_us, true), 0.f);
	feed(kT_us + 1U * kMap_us, true);
	feed(kT_us + 2U * kMap_us, true);
	EXPECT_GT(feed(kT_us + 3U * kMap_us, true), 0.f);

	// the outage: three maps not used, the cap is 0 at once
	EXPECT_FLOAT_EQ(feed(kT_us + 4U * kMap_us, false), 0.f);
	EXPECT_FLOAT_EQ(feed(kT_us + 5U * kMap_us, false), 0.f);
	EXPECT_FLOAT_EQ(feed(kT_us + 6U * kMap_us, false), 0.f);

	// recovery: the dwell starts again, 0.3 s of used maps are needed before any rise
	EXPECT_FLOAT_EQ(feed(kT_us + 7U * kMap_us, true), 0.f);   // no step
	EXPECT_FLOAT_EQ(feed(kT_us + 8U * kMap_us, true), 0.f);   // dwell 0.1 s
	EXPECT_FLOAT_EQ(feed(kT_us + 9U * kMap_us, true), 0.f);   // dwell 0.2 s
	EXPECT_GT(feed(kT_us + 10U * kMap_us, true), 0.f);        // dwell 0.3 s: the cap rises
}
