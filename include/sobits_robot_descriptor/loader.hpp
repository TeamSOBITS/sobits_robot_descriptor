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

#pragma once

// Header-only C++17 mirror of the python loader (schema v2, yaml-cpp only).
// Full validation lives in python (`ros2 run sobits_robot_descriptor validate`).

#include <yaml-cpp/yaml.h>

#include <algorithm>
#include <cctype>
#include <cstdlib>
#include <filesystem>
#include <map>
#include <optional>
#include <sstream>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

namespace sobits_robot_descriptor
{

constexpr int SCHEMA_VERSION = 2;
constexpr const char * ENV_PATH = "SOBITS_ROBOT_DESCRIPTOR_PATH";

using Args = std::map<std::string, std::string>;

struct StreamSpec
{
  std::string kind;  // "color" | "depth"
  std::string frame;  // optical frame
  std::string raw_topic;
  std::string info_topic;
  std::string compressed_topic;  // empty when absent
  std::string points_topic;
  std::string encoding;
  int width = 0;  // 0 when absent
  int height = 0;
  double fps = 0.0;
  std::optional<std::pair<double, double>> range_m;
};

struct CameraSpec
{
  std::string name;
  std::string frame;  // mount link
  std::string model;
  std::optional<StreamSpec> color;
  std::optional<StreamSpec> depth;
  Args requires_;  // `requires` is a C++20 keyword

  std::vector<StreamSpec> streams() const
  {
    std::vector<StreamSpec> out;
    if (color) {out.push_back(*color);}
    if (depth) {out.push_back(*depth);}
    return out;
  }
};

struct LidarSpec
{
  std::string name;
  std::string frame;
  std::string scan_topic;
  std::string points_topic;
  std::vector<std::string> merged_from;
  Args requires_;  // `requires` is a C++20 keyword
};

struct ImuSpec
{
  std::string name;
  std::string frame;
  std::string topic;
  Args requires_;  // `requires` is a C++20 keyword
};

struct GroupSpec
{
  std::string name;
  std::string controller;
  std::vector<std::string> joints;
  std::string interface = "trajectory";
  std::string kind = "position";
  std::vector<std::string> uncommanded_joints;
  std::string command_topic;
  std::string command_action;  // empty for interface "group"
  std::string state_topic;
};

struct BaseControllerSpec
{
  std::string name;
  std::string controller;
  std::vector<std::string> joints;
  std::string interface = "group";
  std::string kind = "position";
  std::string command_topic;
};

struct MobileBaseSpec
{
  std::string command_topic;
  std::string odom_topic;
  bool has_vel_x = false;
  bool has_vel_y = false;
  bool has_vel_z = false;
  bool has_vel_theta = false;
  std::vector<BaseControllerSpec> controllers;
};

struct EEControlSpec
{
  std::string group;
  std::string command_frame;
  std::string enable_topic;
};

struct EESpec
{
  std::string name;
  std::string ee_link;
  std::string reference_frame;
  std::optional<EEControlSpec> control;
};

struct UrdfSpec
{
  std::string xacro;
  std::string urdf;
  Args args;
};

namespace detail
{

template<typename T>
const T & by_name(const std::vector<T> & items, const std::string & kind, const std::string & name)
{
  for (const auto & i : items) {
    if (i.name == name) {return i;}
  }
  std::vector<std::string> known;
  for (const auto & i : items) {known.push_back(i.name);}
  std::sort(known.begin(), known.end());
  std::ostringstream os;
  os << "Unknown " << kind << " '" << name << "'. Known: [";
  for (size_t k = 0; k < known.size(); ++k) {
    os << (k ? ", " : "") << "'" << known[k] << "'";
  }
  os << "]";
  throw std::runtime_error(os.str());
}

}  // namespace detail

struct RobotDescriptor
{
  int schema_version = SCHEMA_VERSION;
  std::string robot_id;
  std::string version = "1.0.0";
  std::string namespace_;  // `namespace` is a keyword
  std::string description_package;
  UrdfSpec urdf;
  std::string base_frame;
  std::string odom_frame = "odom";
  std::string joint_states_topic;
  std::vector<GroupSpec> groups;
  std::optional<MobileBaseSpec> mobile_base;
  std::vector<CameraSpec> cameras;
  std::vector<LidarSpec> lidars;
  std::vector<ImuSpec> imus;
  std::vector<EESpec> ee;
  std::vector<std::string> excluded_joints;
  std::string prefix;
  Args args;  // effective xacro args (urdf.args + overrides)
  std::string path;

  std::string topic(const std::string & rel) const
  {
    if (!rel.empty() && rel.front() == '/') {return rel;}
    std::string ns = namespace_;
    while (!ns.empty() && ns.front() == '/') {ns.erase(ns.begin());}
    while (!ns.empty() && ns.back() == '/') {ns.pop_back();}
    return ns.empty() ? "/" + rel : "/" + ns + "/" + rel;
  }

  std::string frame(const std::string & name) const {return prefix + name;}

  const GroupSpec & group(const std::string & n) const
  {
    return detail::by_name(groups, "group", n);
  }
  const CameraSpec & camera(const std::string & n) const
  {
    return detail::by_name(cameras, "camera", n);
  }
  const LidarSpec & lidar(const std::string & n) const
  {
    return detail::by_name(lidars, "lidar", n);
  }
  const ImuSpec & imu(const std::string & n) const {return detail::by_name(imus, "imu", n);}
  const EESpec & end_effector(const std::string & n) const
  {
    return detail::by_name(ee, "end effector", n);
  }

  std::vector<std::string> joints(bool include_uncommanded = false) const
  {
    std::vector<std::string> out;
    for (const auto & g : groups) {
      out.insert(out.end(), g.joints.begin(), g.joints.end());
      if (include_uncommanded) {
        out.insert(out.end(), g.uncommanded_joints.begin(), g.uncommanded_joints.end());
      }
    }
    return out;
  }

  std::vector<std::string> all_excluded_joints() const
  {
    std::vector<std::string> out = excluded_joints;
    if (mobile_base) {
      for (const auto & c : mobile_base->controllers) {
        for (const auto & j : c.joints) {
          if (std::find(out.begin(), out.end(), j) == out.end()) {out.push_back(j);}
        }
      }
    }
    return out;
  }
};

namespace detail
{

inline std::string norm(std::string s)
{
  // Match python: lower-case; true/yes/on and false/no/off compare as "true"/"false".
  std::transform(s.begin(), s.end(), s.begin(), [](unsigned char c) {return std::tolower(c);});
  if (s == "true" || s == "yes" || s == "on") {return "true";}
  if (s == "false" || s == "no" || s == "off") {return "false";}
  return s;
}

inline const YAML::Node req(const YAML::Node & n, const std::string & key, const std::string & where)
{
  const YAML::Node v = n[key];
  if (!v || v.IsNull()) {
    throw std::runtime_error("Malformed descriptor in " + where + ": missing key '" + key + "'");
  }
  return v;
}

inline std::string str(const YAML::Node & n, const std::string & key, const std::string & dflt = "")
{
  const YAML::Node v = n[key];
  return (v && !v.IsNull()) ? v.as<std::string>() : dflt;
}

inline std::vector<std::string> strs(const YAML::Node & n, const std::string & key)
{
  const YAML::Node v = n[key];
  return (v && v.IsSequence()) ? v.as<std::vector<std::string>>() : std::vector<std::string>{};
}

inline YAML::Node seq(const YAML::Node & n, const std::string & key)
{
  if (n && n.IsMap() && n[key] && n[key].IsSequence()) {return n[key];}
  return YAML::Node(YAML::NodeType::Sequence);
}

inline Args args_of(const YAML::Node & n)
{
  Args out;
  if (n && n.IsMap()) {
    for (const auto & kv : n) {out[kv.first.as<std::string>()] = kv.second.as<std::string>();}
  }
  return out;
}

inline bool present(const Args & req, const Args & eff)
{
  for (const auto & [k, v] : req) {
    auto it = eff.find(k);
    if (it == eff.end() || detail::norm(it->second) != detail::norm(v)) {return false;}
  }
  return true;
}

inline std::optional<StreamSpec> stream(const std::string & kind, const YAML::Node & n,
  const std::string & where)
{
  if (!n || n.IsNull()) {return std::nullopt;}
  StreamSpec s;
  s.kind = kind;
  s.frame = req(n, "frame", where).as<std::string>();
  s.raw_topic = str(n, "raw_topic");
  s.info_topic = str(n, "info_topic");
  s.compressed_topic = str(n, "compressed_topic");
  s.points_topic = str(n, "points_topic");
  s.encoding = str(n, "encoding");
  if (n["width"]) {s.width = n["width"].as<int>();}
  if (n["height"]) {s.height = n["height"].as<int>();}
  if (n["fps"]) {s.fps = n["fps"].as<double>();}
  if (n["range_m"] && n["range_m"].IsSequence() && n["range_m"].size() == 2) {
    s.range_m = std::make_pair(n["range_m"][0].as<double>(), n["range_m"][1].as<double>());
  }
  return s;
}

inline std::vector<std::string> split_env(const char * var)
{
  std::vector<std::string> out;
  const char * v = std::getenv(var);
  if (!v) {return out;}
  std::stringstream ss(v);
  std::string item;
  while (std::getline(ss, item, ':')) {
    if (!item.empty()) {out.push_back(item);}
  }
  return out;
}

}  // namespace detail

// Same order as python, minus the ament index (the prefix scan covers it).
inline std::string resolve_path(
  const std::string & robot_id, const std::vector<std::string> & search_dirs = {})
{
  namespace fs = std::filesystem;
  std::error_code ec;
  if (fs::is_regular_file(robot_id, ec)) {return robot_id;}
  const std::string file = robot_id + ".robot.yaml";
  const std::string pkg = robot_id + "_description";
  std::vector<std::string> tried;
  auto probe = [&](const fs::path & p) {
      tried.push_back(p.string());
      return fs::is_regular_file(p, ec);
    };

  for (const auto & e : detail::split_env(ENV_PATH)) {
    const fs::path p(e);
    if (fs::is_regular_file(p, ec)) {
      tried.push_back(p.string());
      if (p.filename() == file) {return p.string();}
      continue;
    }
    if (probe(p / file)) {return (p / file).string();}
    if (probe(p / robot_id / file)) {return (p / robot_id / file).string();}
  }
  const fs::path rel = fs::path("share") / pkg / "config" / file;
  for (const char * var : {"AMENT_PREFIX_PATH", "COLCON_PREFIX_PATH"}) {
    for (const auto & e : detail::split_env(var)) {
      if (probe(fs::path(e) / rel)) {return (fs::path(e) / rel).string();}
      if (probe(fs::path(e) / pkg / rel)) {return (fs::path(e) / pkg / rel).string();}
    }
  }
  for (const auto & d : search_dirs) {
    if (probe(fs::path(d) / file)) {return (fs::path(d) / file).string();}
    if (probe(fs::path(d) / robot_id / file)) {return (fs::path(d) / robot_id / file).string();}
  }
  std::string msg = "Robot descriptor '" + robot_id + "' not found. Set " + ENV_PATH +
    " or install " + pkg + ". Tried:";
  for (const auto & t : tried) {msg += "\n  " + t;}
  if (tried.empty()) {msg += "\n  (none)";}
  throw std::runtime_error(msg);
}

inline RobotDescriptor load_file(
  const std::string & path, const std::string & prefix = "", const Args & args = {})
{
  YAML::Node d;
  try {
    d = YAML::LoadFile(path);
  } catch (const YAML::Exception & e) {
    throw std::runtime_error("Cannot read descriptor " + path + ": " + e.what());
  }
  if (!d.IsMap()) {throw std::runtime_error("Descriptor " + path + " is not a YAML mapping");}
  const int ver = d["schema_version"] ? d["schema_version"].as<int>() : 0;
  if (ver != SCHEMA_VERSION) {
    throw std::runtime_error(
            "Invalid descriptor " + path + ":\n  schema_version must be 2, got " +
            (d["schema_version"] ? d["schema_version"].as<std::string>() : std::string("None")));
  }

  using namespace detail;
  RobotDescriptor r;
  try {
    r.path = path;
    r.prefix = prefix;
    r.robot_id = req(d, "robot_id", path).as<std::string>();
    r.version = str(d, "version", "1.0.0");
    r.namespace_ = str(d, "namespace");
    r.description_package = str(d, "description_package", r.robot_id + "_description");
    const YAML::Node u = req(d, "urdf", path);
    r.urdf.xacro = req(u, "xacro", path).as<std::string>();
    r.urdf.urdf = str(u, "urdf");
    r.urdf.args = args_of(u["args"]);
    r.args = r.urdf.args;
    for (const auto & [k, v] : args) {r.args[k] = v;}
    r.base_frame = req(d, "base_frame", path).as<std::string>();
    r.odom_frame = str(d, "odom_frame", "odom");
    r.joint_states_topic = req(d, "joint_states_topic", path).as<std::string>();

    for (const auto & g : seq(d, "groups")) {
      GroupSpec s;
      s.name = req(g, "name", path).as<std::string>();
      s.controller = req(g, "controller", path).as<std::string>();
      s.joints = req(g, "joints", path).as<std::vector<std::string>>();
      s.interface = str(g, "interface", "trajectory");
      s.kind = str(g, "kind", "position");
      s.uncommanded_joints = strs(g, "uncommanded_joints");
      const bool grp = s.interface == "group";
      s.command_topic = str(g, "command_topic",
          s.controller + (grp ? "/commands" : "/joint_trajectory"));
      s.command_action = str(g, "command_action",
          grp ? "" : s.controller + "/follow_joint_trajectory");
      s.state_topic = str(g, "state_topic", grp ? "" : s.controller + "/controller_state");
      r.groups.push_back(std::move(s));
    }

    if (const YAML::Node mb = d["mobile_base"]; mb && mb.IsMap()) {
      MobileBaseSpec m;
      m.command_topic = req(mb, "command_topic", path).as<std::string>();
      m.odom_topic = req(mb, "odom_topic", path).as<std::string>();
      m.has_vel_x = mb["has_vel_x"] && mb["has_vel_x"].as<bool>();
      m.has_vel_y = mb["has_vel_y"] && mb["has_vel_y"].as<bool>();
      m.has_vel_z = mb["has_vel_z"] && mb["has_vel_z"].as<bool>();
      m.has_vel_theta = mb["has_vel_theta"] && mb["has_vel_theta"].as<bool>();
      for (const auto & c : seq(mb, "controllers")) {
        BaseControllerSpec s;
        s.name = req(c, "name", path).as<std::string>();
        s.controller = req(c, "controller", path).as<std::string>();
        s.joints = req(c, "joints", path).as<std::vector<std::string>>();
        s.interface = str(c, "interface", "group");
        s.kind = str(c, "kind", "position");
        s.command_topic = str(c, "command_topic",
            s.controller + (s.interface == "group" ? "/commands" : "/joint_trajectory"));
        m.controllers.push_back(std::move(s));
      }
      r.mobile_base = std::move(m);
    }

    const YAML::Node sensors = d["sensors"];
    for (const auto & c : seq(sensors, "cameras")) {
      CameraSpec s;
      s.requires_ = args_of(c["requires"]);
      if (!present(s.requires_, r.args)) {continue;}
      s.name = req(c, "name", path).as<std::string>();
      s.frame = req(c, "frame", path).as<std::string>();
      s.model = str(c, "model");
      s.color = stream("color", c["color"], path);
      s.depth = stream("depth", c["depth"], path);
      r.cameras.push_back(std::move(s));
    }
    for (const auto & c : seq(sensors, "lidars")) {
      LidarSpec s;
      s.requires_ = args_of(c["requires"]);
      if (!present(s.requires_, r.args)) {continue;}
      s.name = req(c, "name", path).as<std::string>();
      s.frame = req(c, "frame", path).as<std::string>();
      s.scan_topic = req(c, "scan_topic", path).as<std::string>();
      s.points_topic = str(c, "points_topic");
      s.merged_from = strs(c, "merged_from");
      r.lidars.push_back(std::move(s));
    }
    for (const auto & c : seq(sensors, "imus")) {
      ImuSpec s;
      s.requires_ = args_of(c["requires"]);
      if (!present(s.requires_, r.args)) {continue;}
      s.name = req(c, "name", path).as<std::string>();
      s.frame = req(c, "frame", path).as<std::string>();
      s.topic = req(c, "topic", path).as<std::string>();
      r.imus.push_back(std::move(s));
    }

    for (const auto & e : seq(d, "ee")) {
      EESpec s;
      s.name = req(e, "name", path).as<std::string>();
      s.ee_link = req(e, "ee_link", path).as<std::string>();
      s.reference_frame = req(e, "reference_frame", path).as<std::string>();
      if (const YAML::Node c = e["control"]; c && c.IsMap()) {
        s.control = EEControlSpec{
          req(c, "group", path).as<std::string>(),
          req(c, "command_frame", path).as<std::string>(),
          req(c, "enable_topic", path).as<std::string>()};
      }
      r.ee.push_back(std::move(s));
    }
    r.excluded_joints = strs(d, "excluded_joints");
  } catch (const YAML::Exception & e) {
    throw std::runtime_error("Malformed descriptor in " + path + ": " + e.what());
  }
  return r;
}

inline RobotDescriptor load(
  const std::string & id_or_path, const std::string & prefix = "", const Args & args = {},
  const std::vector<std::string> & search_dirs = {})
{
  return load_file(resolve_path(id_or_path, search_dirs), prefix, args);
}

}  // namespace sobits_robot_descriptor
