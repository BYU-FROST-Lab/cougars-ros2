// Copyright (c) 2026 BYU FROST Lab
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

/**
 * @file coug_waypoint_manager.hpp
 * @brief MapViz plugin helper, manages waypoint storage and file I/O.
 * @author Nelson Durrant
 * @date Jan 2026
 */

#pragma once

#include <geometry_msgs/msg/pose.hpp>
#include <map>
#include <optional>
#include <string>
#include <vector>

namespace coug_mapviz {

// ---------------------------------------------------------------------------
// Per-waypoint data
// ---------------------------------------------------------------------------

struct CougWaypoint {
  geometry_msgs::msg::Pose pose;  // position: x=lon, y=lat, z=depth_value

  // Required per waypoint
  std::string depth_ref = "surface";  // "surface" or "bottom"
  bool park = false;

  // Optional per-waypoint overrides (absent = use MissionDefaults)
  std::optional<double> speed;
  std::optional<double> slip_radius;
  std::optional<double> capture_radius;
};

// ---------------------------------------------------------------------------
// Mission-level defaults (apply to all waypoints without per-wp overrides)
// ---------------------------------------------------------------------------

struct MissionDefaults {
  double speed = 50.0;
  double slip_radius = 2.0;
  double capture_radius = 10.0;
};

// ---------------------------------------------------------------------------
// Manager
// ---------------------------------------------------------------------------

class CougWaypointManager {
 public:
  CougWaypointManager() = default;
  ~CougWaypointManager() = default;

  // Waypoint CRUD
  void addWaypoint(const std::string& topic, const CougWaypoint& waypoint);
  void setWaypoints(const std::string& topic, const std::vector<CougWaypoint>& waypoints);
  std::vector<CougWaypoint> getWaypoints(const std::string& topic) const;
  const std::map<std::string, std::vector<CougWaypoint>>& getAllWaypoints() const;
  void removeTopic(const std::string& topic);
  void clearWaypoints(const std::string& topic);
  void clearAllWaypoints();

  // Mission defaults
  MissionDefaults getDefaults(const std::string& topic) const;
  void setDefaults(const std::string& topic, const MissionDefaults& defaults);

  /**
   * @brief Saves all (or one) topic to a JSON file.
   *        JSON format: { "/topic": { "defaults": {...}, "waypoints": [...] } }
   *        Old array-only format is still readable via loadFromFile.
   */
  bool saveToFile(const std::string& filename, const std::string& topic = "") const;

  /**
   * @brief Loads waypoints (and optional defaults) from a JSON file.
   *        Handles both the new object format and the legacy array format.
   */
  bool loadFromFile(const std::string& filename, const std::string& topic = "");

 private:
  std::map<std::string, std::vector<CougWaypoint>> waypoint_map_;
  std::map<std::string, MissionDefaults> defaults_map_;
};

}  // namespace coug_mapviz
