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
 * Braking acceleration of the side class
 *
 * Stopping model of every sector whose centre is more than 45 degrees from the nose (SYS-CP-028);
 * CPB_A_BRK is the forward class. It comes from the sensor mount: g tan of the angle between the
 * horizon and the lower edge of the field of view toward the side, 1.2 m/s^2 for the X500 bench mount
 * pitched 10 degrees down (approved 2026-11-22). Platform value: -1 means not set, and cp_brake reports
 * an invalid configuration instead of running.
 *
 * @unit m/s^2
 * @min -1
 * @max 20
 * @decimal 1
 * @group Collision Prevention Brake
 */
PARAM_DEFINE_FLOAT(CPB_A_SIDE, -1.f);

/**
 * Speed lock outside the forward class
 *
 * While the velocity setpoint, or the velocity while faster than this, points more than 45 degrees from
 * the nose, the horizontal speed is at most this value (SYS-CP-031 a). It limits the velocity vector,
 * never a sector cap. Also the cap of an unclaimed sector with no evidence (CPB_REAR_ANG). 5 m/s for the
 * X500 bench (PERF-01 Mission tier, approved 2026-11-22). Not above CPB_MAX_V. Platform value: -1 means
 * not set, and cp_brake reports an invalid configuration instead of running.
 *
 * @unit m/s
 * @min -1
 * @max 30
 * @decimal 1
 * @group Collision Prevention Brake
 */
PARAM_DEFINE_FLOAT(CPB_V_SIDE, -1.f);

/**
 * Start of the unclaimed bearings
 *
 * Angle from the nose beyond which the horizon is never in the field of view at level, so obstacles
 * below the sensor are not seen and the direction is outside the claim (SYS-CP-031 b): a sector there
 * with no evidence has the cap CPB_V_SIDE instead of 0, and the hook reports the motion
 * (REASON_UNCLAIMED_DIRECTION). It must match the rear class of the obstacle map (rear_start_rad).
 * 134.6 degrees for the X500 bench mount pitched 10 degrees down (approved 2026-11-22); 180 leaves no
 * unclaimed bearing. Above 45. Platform value: -1 means not set, and cp_brake reports an invalid
 * configuration instead of running.
 *
 * @unit deg
 * @min -1
 * @max 180
 * @decimal 1
 * @group Collision Prevention Brake
 */
PARAM_DEFINE_FLOAT(CPB_REAR_ANG, -1.f);

/**
 * Upper bound of the horizontal deceleration
 *
 * Used to allow for a vehicle that slowed down between the sample time of a map and its use (the
 * distance moved since the sample, ADR 0020 D4). No smaller than the braking acceleration of the
 * model (CPB_A_BRK). No approved value yet: -1 means not set, and cp_brake reports an invalid
 * configuration instead of running.
 *
 * @unit m/s^2
 * @min -1
 * @max 20
 * @decimal 1
 * @group Collision Prevention Brake
 */
PARAM_DEFINE_FLOAT(CPB_DECEL_MAX, -1.f);

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
 * Design baseline (SAFE-01), not a qualified limit. It is also the half-width of the corridor of
 * ADR 0025: a sector limits the motion only when its obstacle may lie within this distance of the
 * path, so it must cover the vehicle half-span, the lateral tracking error and the lateral drift over
 * the map age as well as the stopping margin.
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
