// Copyright (c) 2026, Team SOBITS
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
// * Redistributions of source code must retain the above copyright notice, this
//   list of conditions and the following disclaimer.
//
// * Redistributions in binary form must reproduce the above copyright notice,
//   this list of conditions and the following disclaimer in the documentation
//   and/or other materials provided with the distribution.
//
// * Neither the name of the copyright holder nor the names of its
//   contributors may be used to endorse or promote products derived from this
//   software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
// DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
// FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
// DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
// SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
// CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
// OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
// OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

#include <gtest/gtest.h>

#include <cstdlib>
#include <string>
#include <vector>

#include "sobits_robot_descriptor/loader.hpp"

namespace srd = sobits_robot_descriptor;

static const std::string DATA = TEST_DATA_DIR;
static const std::string SOBIT_HOME = DATA + "/sobit_home.robot.yaml";

TEST(Loader, ParsesSobitHome)
{
  const auto d = srd::load_file(SOBIT_HOME);
  EXPECT_EQ(d.robot_id, "sobit_home");
  EXPECT_EQ(d.namespace_, "sobit_home");
  EXPECT_EQ(d.description_package, "sobit_home_description");
  EXPECT_EQ(d.args.at("head_cam_type"), "orbbec");
  ASSERT_EQ(d.groups.size(), 6u);
  EXPECT_EQ(d.joints().size(), 31u);
  EXPECT_EQ(d.joints(true).size(), 33u);
  EXPECT_EQ(d.group("hand_left").uncommanded_joints.at(0), "hand_left_finger_r_mcp_joint");
  ASSERT_TRUE(d.mobile_base);
  EXPECT_TRUE(d.mobile_base->has_vel_y);
  EXPECT_FALSE(d.mobile_base->has_vel_z);
  EXPECT_EQ(d.all_excluded_joints().size(), 8u);
  EXPECT_EQ(d.end_effector("left").control->command_frame, "left_target_link");
  EXPECT_EQ(d.lidar("lidar_merged").merged_from.size(), 2u);
}

TEST(Loader, DerivedTopics)
{
  const auto d = srd::load_file(SOBIT_HOME);
  const auto & head = d.group("head");
  EXPECT_EQ(head.command_topic, "head_position_controller/joint_trajectory");
  EXPECT_EQ(head.command_action, "head_position_controller/follow_joint_trajectory");
  EXPECT_EQ(head.state_topic, "head_position_controller/controller_state");
  EXPECT_EQ(d.mobile_base->controllers.at(1).command_topic,
    "wheel_drive_velocity_controller/commands");
}

TEST(Loader, DiffDrive)
{
  const auto d = srd::load_file(DATA + "/diffbot.robot.yaml");
  ASSERT_TRUE(d.mobile_base);
  const auto & c = d.mobile_base->controllers.at(0);
  EXPECT_EQ(c.interface, "diff_drive");
  EXPECT_EQ(c.kind, "velocity");
  EXPECT_EQ(c.command_topic, "cmd_vel");
  EXPECT_EQ(d.topic(c.command_topic), "/diffbot/cmd_vel");
  EXPECT_EQ(c.joints, (std::vector<std::string>{"wheel_left_joint", "wheel_right_joint"}));
  EXPECT_DOUBLE_EQ(c.wheel_radius, 0.05);
  EXPECT_DOUBLE_EQ(c.wheel_separation, 0.3);
  EXPECT_EQ(d.all_excluded_joints().size(), 2u);
  const auto h = srd::load_file(SOBIT_HOME);
  EXPECT_DOUBLE_EQ(h.mobile_base->controllers.at(0).wheel_radius, 0.0);
}

TEST(Loader, TopicAndFramePrefix)
{
  const auto d = srd::load_file(SOBIT_HOME, "sobit_home/");
  const auto & cam = d.camera("head_camera");
  ASSERT_TRUE(cam.color && cam.depth);
  EXPECT_EQ(d.topic(cam.color->raw_topic), "/sobit_home/head_camera/color/image_raw");
  EXPECT_EQ(d.topic("/abs"), "/abs");
  EXPECT_EQ(d.frame(d.base_frame), "sobit_home/base_footprint");
  EXPECT_EQ(cam.streams().size(), 2u);
  ASSERT_TRUE(cam.depth->range_m);
  EXPECT_DOUBLE_EQ(cam.depth->range_m->second, 10.0);
  EXPECT_EQ(cam.color->width, 640);
  EXPECT_EQ(d.camera("hand_left_camera").streams().size(), 1u);
  EXPECT_EQ(srd::load_file(DATA + "/minimal.robot.yaml").topic("joint_states"), "/joint_states");
}

TEST(Loader, RequiresFiltering)
{
  EXPECT_EQ(srd::load_file(SOBIT_HOME).imus.size(), 1u);
  const auto rs = srd::load_file(SOBIT_HOME, "", {{"head_cam_type", "realsense"}});
  EXPECT_TRUE(rs.imus.empty());
  EXPECT_EQ(rs.cameras.size(), 3u);
  EXPECT_THROW(rs.imu("head_camera_imu"), std::runtime_error);
}

TEST(Loader, RequiresCaseInsensitive)
{
  const auto up = srd::load_file(SOBIT_HOME, "", {{"head_cam_type", "ORBBEC"}});
  EXPECT_EQ(up.imus.size(), 1u);
  EXPECT_EQ(up.args.at("head_cam_type"), "ORBBEC");
  EXPECT_TRUE(srd::load_file(SOBIT_HOME, "", {{"head_cam_type", "Realsense"}}).imus.empty());
}

TEST(Loader, UnknownNameListsKnown)
{
  const auto d = srd::load_file(SOBIT_HOME);
  try {
    d.group("arm");
    FAIL();
  } catch (const std::runtime_error & e) {
    EXPECT_NE(std::string(e.what()).find("'arm_left'"), std::string::npos);
  }
}

TEST(Resolve, EnvAndSearchDirs)
{
  unsetenv("AMENT_PREFIX_PATH");
  unsetenv("COLCON_PREFIX_PATH");
  setenv(srd::ENV_PATH, ("/nonexistent:" + DATA).c_str(), 1);
  EXPECT_EQ(srd::resolve_path("sobit_home"), SOBIT_HOME);
  EXPECT_EQ(srd::load("minimal").robot_id, "minimal");
  unsetenv(srd::ENV_PATH);
  EXPECT_EQ(srd::resolve_path("minimal", {DATA}), DATA + "/minimal.robot.yaml");
  EXPECT_EQ(srd::resolve_path(SOBIT_HOME), SOBIT_HOME);
  EXPECT_THROW(srd::resolve_path("no_such_robot"), std::runtime_error);
}

TEST(Loader, RejectsSchemaV1)
{
  const std::string p = ::testing::TempDir() + "v1.robot.yaml";
  FILE * f = std::fopen(p.c_str(), "w");
  ASSERT_NE(f, nullptr);
  std::fputs("schema_version: 1\nrobot_id: x\n", f);
  std::fclose(f);
  try {
    srd::load_file(p);
    FAIL();
  } catch (const std::runtime_error & e) {
    EXPECT_NE(std::string(e.what()).find("schema_version must be 2, got 1"), std::string::npos);
  }
}
