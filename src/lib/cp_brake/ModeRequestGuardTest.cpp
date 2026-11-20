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
 * @file ModeRequestGuardTest.cpp
 *
 * Source guard for ADR 0007: cp_brake never publishes a vehicle command and never requests Hold,
 * RTL or another mode. This is a check of the sources, not a proof of behaviour: it fails when the
 * module or the library names a command or mode-request topic, or when the module declares a
 * publication other than its own two topics. The module cannot be run on the host.
 */

#include <gtest/gtest.h>

#include <filesystem>
#include <fstream>
#include <iterator>
#include <regex>
#include <set>
#include <sstream>
#include <string>
#include <vector>

namespace fs = std::filesystem;

namespace
{

std::string readFile(const fs::path &path)
{
	std::ifstream in(path);
	EXPECT_TRUE(in.is_open()) << path;
	return std::string(std::istreambuf_iterator<char>(in), std::istreambuf_iterator<char>());
}

// the production sources of the module and of its library: no test file, which names the tokens
std::vector<fs::path> productionSources()
{
	std::vector<fs::path> files;
	const fs::path lib{CP_BRAKE_LIB_DIR};
	const fs::path module{CP_BRAKE_MODULE_DIR};

	for (const fs::path &dir : {lib, module}) {
		for (const auto &entry : fs::directory_iterator(dir)) {
			const fs::path &path = entry.path();
			const std::string name = path.filename().string();
			const bool is_source = path.extension() == ".cpp" || path.extension() == ".hpp"
					       || path.extension() == ".c" || name == "CMakeLists.txt" || name == "Kconfig";
			const bool is_test = name.size() > 8 && name.compare(name.size() - 8, 8, "Test.cpp") == 0;

			if (is_source && !is_test) {
				files.push_back(path);
			}
		}
	}

	return files;
}

} // namespace

TEST(ModeRequestGuardTest, TheSourcesNameNoCommandOrModeRequest)
{
	const std::vector<fs::path> files = productionSources();
	ASSERT_GE(files.size(), 10U) << "the guard found too few sources to mean anything";

	const char *const forbidden[] = {"vehicle_command", "VehicleCommand", "VEHICLE_CMD", "action_request",
					 "ActionRequest", "commander", "set_nav_state", "nav_state_user_intention"
					};

	for (const fs::path &path : files) {
		const std::string text = readFile(path);

		for (const char *token : forbidden) {
			EXPECT_EQ(text.find(token), std::string::npos) << path << " names " << token;
		}
	}
}

TEST(ModeRequestGuardTest, TheModulePublishesOnlyItsTwoTopics)
{
	const std::set<std::string> allowed = {"cp_brake_constraints_s", "cp_brake_status_s"};
	std::set<std::string> found;
	const std::regex publication(R"(Publication\w*\s*<\s*(\w+)\s*>)");
	const std::regex raw_publish(R"(orb_(advertise|publish)\w*\s*\()");

	for (const fs::path &path : productionSources()) {
		const std::string text = readFile(path);

		for (auto it = std::sregex_iterator(text.begin(), text.end(), publication); it != std::sregex_iterator(); ++it) {
			found.insert((*it)[1]);
			EXPECT_EQ(allowed.count((*it)[1]), 1U) << path << " publishes " << (*it)[1];
		}

		EXPECT_FALSE(std::regex_search(text, raw_publish)) << path << " publishes with a raw orb call";
	}

	// not vacuous: the pattern really sees the module's own publications
	EXPECT_EQ(found, allowed);
}
