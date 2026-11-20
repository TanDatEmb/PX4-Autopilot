############################################################################
#
#   Copyright (c) 2026 collision-prevention contributors. All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
# 1. Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
# 2. Redistributions in binary form must reproduce the above copyright
#    notice, this list of conditions and the following disclaimer in
#    the documentation and/or other materials provided with the
#    distribution.
# 3. Neither the name PX4 nor the names of its contributors may be
#    used to endorse or promote products derived from this software
#    without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
# OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
# AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#
############################################################################

px4_add_library(cp_brake_model
	CapLimiter.cpp
	ConfigCheck.cpp
	HeadingHistory.cpp
	InputCheck.cpp
	MapTimeline.cpp
	SetpointLimiter.cpp
	StateMachine.cpp
	StoppingModel.cpp
	TimeSync.cpp
)
target_include_directories(cp_brake_model PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})

px4_add_unit_gtest(SRC CapLimiterTest.cpp LINKLIBS cp_brake_model)
px4_add_unit_gtest(SRC ChecksTest.cpp LINKLIBS cp_brake_model)
px4_add_unit_gtest(SRC HeadingHistoryTest.cpp LINKLIBS cp_brake_model)
px4_add_unit_gtest(SRC MapTimelineTest.cpp LINKLIBS cp_brake_model)
px4_add_unit_gtest(SRC ModeRequestGuardTest.cpp LINKLIBS cp_brake_model)
if(TARGET unit-ModeRequestGuard)
	# the guard reads the sources of the library and of the module
	target_compile_definitions(unit-ModeRequestGuard PRIVATE
		CP_BRAKE_LIB_DIR="${CMAKE_CURRENT_SOURCE_DIR}"
		CP_BRAKE_MODULE_DIR="${CMAKE_CURRENT_SOURCE_DIR}/../../modules/cp_brake")
endif()
px4_add_unit_gtest(SRC SetpointLimiterTest.cpp LINKLIBS cp_brake_model)
px4_add_unit_gtest(SRC StateMachineTest.cpp LINKLIBS cp_brake_model)
px4_add_unit_gtest(SRC StoppingModelTest.cpp LINKLIBS cp_brake_model)
px4_add_unit_gtest(SRC TimeSyncTest.cpp LINKLIBS cp_brake_model)
