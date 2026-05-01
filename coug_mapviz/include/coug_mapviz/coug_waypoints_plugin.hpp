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
 * @file coug_waypoints_plugin.hpp
 * @brief MapViz plugin for multi-agent waypoint mission planning.
 * @author Nelson Durrant
 * @date Jan 2026
 */

#pragma once

#include <mapviz/map_canvas.h>
#include <mapviz/mapviz_plugin.h>
#include <swri_transform_util/transform.h>
#include <ui_coug_waypoints_config.h>

#include <QGLWidget>
#include <QMouseEvent>
#include <QObject>
#include <QPainter>
#include <QTimer>
#include <QWidget>
#include <coug_mapviz/coug_waypoint_manager.hpp>
#include <geographic_msgs/msg/route_network.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <map>
#include <rclcpp/rclcpp.hpp>
#include <string>
#include <vector>

namespace coug_mapviz {

class CougWaypointsPlugin : public mapviz::MapvizPlugin {
  Q_OBJECT

 public:
  CougWaypointsPlugin();
  ~CougWaypointsPlugin() override;

  bool Initialize(QGLWidget* canvas) override;
  void Shutdown() override {}
  void Draw(double x, double y, double scale) override;
  void Paint(QPainter* painter, double x, double y, double scale) override;
  void Transform() override {}
  void LoadConfig(const YAML::Node& node, const std::string& path) override;
  void SaveConfig(YAML::Emitter& emitter, const std::string& path) override;
  QWidget* GetConfigWidget(QWidget* parent) override;
  bool SupportsPainting() override { return true; }

 protected:
  void PrintError(const std::string& message) override;
  void PrintInfo(const std::string& message) override;
  void PrintWarning(const std::string& message) override;

  bool eventFilter(QObject* object, QEvent* event) override;
  bool handleMousePress(QMouseEvent* event);
  bool handleMouseRelease(QMouseEvent* event);
  bool handleMouseMove(QMouseEvent* event);

 protected Q_SLOTS:
  void PublishWaypoints();
  void Stop();
  void Clear();
  void SaveWaypoints();
  void LoadWaypoints();
  void TopicChanged(const QString& text);
  void DiscoverTopics();
  void PublishAll();
  void StopAll();
  void VisibilityChanged(bool visible);

  // Per-waypoint property slots (enabled only when a point is selected)
  void DepthChanged(double value);
  void DepthRefChanged(int index);
  void ParkChanged(int state);
  void SpeedOverrideToggled(int state);
  void SpeedChanged(double value);
  void SlipRadiusOverrideToggled(int state);
  void SlipRadiusChanged(double value);
  void CaptureRadiusOverrideToggled(int state);
  void CaptureRadiusChanged(double value);

  // Mission-level default slots (always enabled)
  void DefaultMissionIdChanged(int value);
  void DefaultSpeedChanged(double value);
  void DefaultSlipRadiusChanged(double value);
  void DefaultCaptureRadiusChanged(double value);

 private:
  Ui::coug_waypoints_config ui_;
  QWidget* config_widget_;
  mapviz::MapCanvas* map_canvas_;

  std::map<std::string, rclcpp::Publisher<geographic_msgs::msg::RouteNetwork>::SharedPtr>
      publishers_;
  CougWaypointManager manager_;
  std::string current_topic_;

  int selected_point_;
  int dragged_point_;
  bool is_mouse_down_;
  QPointF mouse_down_pos_;
  qint64 mouse_down_time_;

  QTimer* discovery_timer_;

  void PublishTopic(const std::string& topic, const std::vector<CougWaypoint>& wps);
  bool IsTopicAvailable(const std::string& topic);
  int GetClosestPoint(const QPointF& point, double& distance);

  // Drawing helpers
  void DrawWaypointCircles(const std::vector<CougWaypoint>& wps,
                           const swri_transform_util::Transform& transform,
                           const MissionDefaults& defaults);
  void DrawPath(const std::vector<CougWaypoint>& wps, const QColor& color,
                const swri_transform_util::Transform& transform, int selected_index = -1);
  void PaintLabels(QPainter* painter, const std::vector<CougWaypoint>& wps,
                   const swri_transform_util::Transform& transform, const QColor& color);
  void PaintPath(QPainter* painter, const std::vector<CougWaypoint>& wps, const QColor& color,
                 const swri_transform_util::Transform& transform, int selected_index = -1);

  // UI sync helpers
  void SetWaypointEditorsEnabled(bool enabled);
  void UpdateEditorsFromWaypoint(const CougWaypoint& wp);
  void UpdateMissionDefaultsUI(const MissionDefaults& defaults);
};

}  // namespace coug_mapviz
