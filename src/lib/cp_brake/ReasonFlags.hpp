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
 * @file ReasonFlags.hpp
 *
 * Reason and fault bits of cp_brake. The REASON_* values equal the constants of the
 * CpBrakeConstraints message.
 */

#pragma once

#include <cstdint>

namespace cp_brake
{

constexpr uint32_t kReasonInvalidConfig{1U << 0};
constexpr uint32_t kReasonFrameInvalid{1U << 1};
constexpr uint32_t kReasonDataInvalid{1U << 2};
/** No accepted map yet, or its age reached CPB_AGE_STOP, or the age or its limit is unusable. */
constexpr uint32_t kReasonSampleStale{1U << 3};
/** No heading exists for the sample time (the heading history has no value for it). */
constexpr uint32_t kReasonNoHeading{1U << 4};
/** The PX4 estimator or heading is invalid: the claim is unqualified and CP adds no action (ADR 0007). */
constexpr uint32_t kReasonUnqualified{1U << 5};
/** The caps leave no horizontal motion at all (blockage report). */
constexpr uint32_t kReasonBlocked{1U << 6};
/** A safe stop is latched and waits for the pilot acknowledgement and a revalidation. */
constexpr uint32_t kReasonAckRequired{1U << 7};
/** A pilot override is active: the constraint is not enforced (logged). */
constexpr uint32_t kReasonOverride{1U << 8};
/** The time alignment of the latest map is not good: the map is not used, its sectors are unknown. */
constexpr uint32_t kReasonTimeSync{1U << 9};
/** The hook found no constraint, one older than CPB_CSTR_AGE, or no usable age limit: no evidence of free space. */
constexpr uint32_t kReasonConstraintStale{1U << 10};
/** The hook found a malformed constraint: a heading that is not a number, or an enforced state it cannot map. */
constexpr uint32_t kReasonConstraintMalformed{1U << 11};
/** The hook is transparent because the navigation state is neither Position nor Mission. */
constexpr uint32_t kReasonNavState{1U << 12};
/** The map was sampled before the first heading sample (warm-up after the PX4 state became valid): NOT_READY, no latch (ADR 0020 D5). */
constexpr uint32_t kReasonHeadingWarmUp{1U << 13};

/** The reasons that make the caps unusable as a whole (no evidence of free space). */
constexpr uint32_t kReasonConstraintFaults{kReasonInvalidConfig | kReasonFrameInvalid | kReasonDataInvalid
		| kReasonSampleStale};

} // namespace cp_brake
