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
 * Collision prevention brake enable
 *
 * Starts the cp_brake module at boot (reboot to apply). The module publishes the per-sector speed
 * caps; the limiter hook in mc_pos_control applies them, in flight in Position and Mission, and reads
 * this parameter too. Do not enable together with the stock collision prevention (CP_DIST > 0).
 *
 * @boolean
 * @group Collision Prevention Brake
 */
PARAM_DEFINE_INT32(CPB_EN, 0);

/**
 * Maximum speed of the caps
 *
 * Upper bound of every cap and of the stopping model. Design baseline of the architecture (PERF-01),
 * not a qualified limit. MPC_XY_VEL_MAX must not exceed it.
 *
 * @unit m/s
 * @min 0.1
 * @max 30
 * @decimal 1
 * @group Collision Prevention Brake
 */
PARAM_DEFINE_FLOAT(CPB_MAX_V, 12.f);

/**
 * Braking acceleration of the stopping model
 *
 * Reference used by the model, not a limit on the setpoint rate. Design baseline (PERF-02).
 *
 * @unit m/s^2
 * @min 0.1
 * @max 20
 * @decimal 1
 * @group Collision Prevention Brake
 */
PARAM_DEFINE_FLOAT(CPB_A_BRK, 3.f);

/**
 * Braking jerk of the stopping model
 *
 * Reference used by the model. Design baseline (PERF-02).
 *
 * @unit m/s^3
 * @min 0.1
 * @max 50
 * @decimal 1
 * @group Collision Prevention Brake
 */
PARAM_DEFINE_FLOAT(CPB_JERK, 4.f);

/**
 * Lag from the hook output to the vehicle response
 *
 * No approved value yet (OPEN-004): -1 means not set, and cp_brake reports an invalid
 * configuration instead of running.
 *
 * @unit s
 * @min -1
 * @max 5
 * @decimal 2
 * @group Collision Prevention Brake
 */
PARAM_DEFINE_FLOAT(CPB_T_LAT, -1.f);

/**
 * Clearance margin of the stopping model
 *
 * Design baseline (SAFE-01), not a qualified limit.
 *
 * @unit m
 * @min 0
 * @max 20
 * @decimal 1
 * @group Collision Prevention Brake
 */
PARAM_DEFINE_FLOAT(CPB_MARGIN, 2.f);

/**
 * Sample age that triggers the safe stop
 *
 * Design baseline (PERF-04), not a qualified limit.
 *
 * @unit s
 * @min 0.05
 * @max 5
 * @decimal 2
 * @group Collision Prevention Brake
 */
PARAM_DEFINE_FLOAT(CPB_AGE_STOP, 0.5f);

/**
 * Clear dwell before a cap may rise
 *
 * Design baseline (PERF-05), not a qualified limit.
 *
 * @unit s
 * @min 0
 * @max 5
 * @decimal 2
 * @group Collision Prevention Brake
 */
PARAM_DEFINE_FLOAT(CPB_CLR_T, 0.3f);

/**
 * Highest rate of rise of a cap
 *
 * Design baseline (PERF-05), not a qualified limit.
 *
 * @unit m/s^2
 * @min 0.1
 * @max 20
 * @decimal 1
 * @group Collision Prevention Brake
 */
PARAM_DEFINE_FLOAT(CPB_RISE, 2.f);

/**
 * Staleness limit of the constraint
 *
 * No approved value yet (OPEN-032): -1 means not set, and cp_brake reports an invalid
 * configuration instead of running.
 *
 * @unit s
 * @min -1
 * @max 5
 * @decimal 2
 * @group Collision Prevention Brake
 */
PARAM_DEFINE_FLOAT(CPB_CSTR_AGE, -1.f);

/**
 * Samples kept in the heading history
 *
 * No approved value yet (OPEN-032): -1 means not set, and cp_brake reports an invalid
 * configuration instead of running. At least 2, at most the storage of the module (128).
 *
 * @min -1
 * @max 128
 * @group Collision Prevention Brake
 */
PARAM_DEFINE_INT32(CPB_HDG_DEPTH, -1);

/**
 * Longest span between two heading samples that may be interpolated
 *
 * No approved value yet (OPEN-032): -1 means not set, and cp_brake reports an invalid
 * configuration instead of running.
 *
 * @unit s
 * @min -1
 * @max 5
 * @decimal 3
 * @group Collision Prevention Brake
 */
PARAM_DEFINE_FLOAT(CPB_HDG_GAP, -1.f);

/**
 * Longest round-trip time of a good time synchronization
 *
 * Limit of the time-alignment gate, read from timesync_status. No approved value yet (OPEN-033):
 * -1 means not set, and cp_brake reports an invalid configuration instead of running.
 *
 * @unit s
 * @min -1
 * @max 5
 * @decimal 4
 * @group Collision Prevention Brake
 */
PARAM_DEFINE_FLOAT(CPB_SYNC_RTT, -1.f);

/**
 * Largest deviation of the observed time offset from its estimate
 *
 * Limit of the time-alignment gate, read from timesync_status. No approved value yet (OPEN-033):
 * -1 means not set, and cp_brake reports an invalid configuration instead of running.
 *
 * @unit s
 * @min -1
 * @max 5
 * @decimal 4
 * @group Collision Prevention Brake
 */
PARAM_DEFINE_FLOAT(CPB_SYNC_OFF, -1.f);

/**
 * Oldest timesync_status still trusted
 *
 * Limit of the time-alignment gate. No approved value yet (OPEN-033): -1 means not set, and
 * cp_brake reports an invalid configuration instead of running.
 *
 * @unit s
 * @min -1
 * @max 5
 * @decimal 2
 * @group Collision Prevention Brake
 */
PARAM_DEFINE_FLOAT(CPB_SYNC_AGE, -1.f);
