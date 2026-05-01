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
 * @file coug_waypoints_plugin.cpp
 * @brief Implementation of the CougWaypointsPlugin.
 * @author Nelson Durrant
 * @date Jan 2026
 */

#include <swri_transform_util/frames.h>

#include <QDateTime>
#include <QDir>
#include <QFileDialog>
#include <QMouseEvent>
#include <QPainter>
#include <cmath>
#include <coug_mapviz/coug_waypoints_plugin.hpp>
#include <cstdlib>
#include <pluginlib/class_list_macros.hpp>
#include <string>
#include <vector>

PLUGINLIB_EXPORT_CLASS(coug_mapviz::CougWaypointsPlugin, mapviz::MapvizPlugin)

namespace coug_mapviz {

// ---------------------------------------------------------------------------
// Static OpenGL circle helpers
// ---------------------------------------------------------------------------

static void DrawFilledCircle(double cx, double cy, double r, float red, float grn, float blu,
                              float alpha) {
  glColor4f(red, grn, blu, alpha);
  glBegin(GL_TRIANGLE_FAN);
  glVertex2d(cx, cy);
  const int N = 64;
  for (int i = 0; i <= N; i++) {
    double angle = 2.0 * M_PI * i / N;
    glVertex2d(cx + r * std::cos(angle), cy + r * std::sin(angle));
  }
  glEnd();
}

static void DrawCircleOutline(double cx, double cy, double r, float red, float grn, float blu,
                               float alpha) {
  glColor4f(red, grn, blu, alpha);
  glBegin(GL_LINE_LOOP);
  const int N = 64;
  for (int i = 0; i < N; i++) {
    double angle = 2.0 * M_PI * i / N;
    glVertex2d(cx + r * std::cos(angle), cy + r * std::sin(angle));
  }
  glEnd();
}

// ---------------------------------------------------------------------------
// Constructor / destructor
// ---------------------------------------------------------------------------

CougWaypointsPlugin::CougWaypointsPlugin()
    : MapvizPlugin(),
      ui_(),
      config_widget_(new QWidget()),
      map_canvas_(nullptr),
      selected_point_(-1),
      dragged_point_(-1),
      is_mouse_down_(false),
      mouse_down_time_(0) {
  ui_.setupUi(config_widget_);

  QPalette p(config_widget_->palette());
  p.setColor(QPalette::Window, Qt::white);
  config_widget_->setPalette(p);
  QPalette p3(ui_.status->palette());
  p3.setColor(QPalette::Text, Qt::darkGreen);
  ui_.status->setPalette(p3);

  discovery_timer_ = new QTimer(this);
  QObject::connect(discovery_timer_, SIGNAL(timeout()), this, SLOT(DiscoverTopics()));

  QObject::connect(ui_.topic_selector, SIGNAL(currentTextChanged(const QString&)), this,
                   SLOT(TopicChanged(const QString&)));
  QObject::connect(ui_.publish, SIGNAL(clicked()), this, SLOT(PublishWaypoints()));
  QObject::connect(ui_.stop, SIGNAL(clicked()), this, SLOT(Stop()));
  QObject::connect(ui_.clear, SIGNAL(clicked()), this, SLOT(Clear()));
  QObject::connect(ui_.save, SIGNAL(clicked()), this, SLOT(SaveWaypoints()));
  QObject::connect(ui_.load, SIGNAL(clicked()), this, SLOT(LoadWaypoints()));
  QObject::connect(this, SIGNAL(VisibleChanged(bool)), this, SLOT(VisibilityChanged(bool)));

  // Mission default signals (always connected — defaults are always editable)
  QObject::connect(ui_.mission_id_editor, SIGNAL(valueChanged(int)), this,
                   SLOT(DefaultMissionIdChanged(int)));
  QObject::connect(ui_.default_speed_editor, SIGNAL(valueChanged(double)), this,
                   SLOT(DefaultSpeedChanged(double)));
  QObject::connect(ui_.default_slip_radius_editor, SIGNAL(valueChanged(double)), this,
                   SLOT(DefaultSlipRadiusChanged(double)));
  QObject::connect(ui_.default_capture_radius_editor, SIGNAL(valueChanged(double)), this,
                   SLOT(DefaultCaptureRadiusChanged(double)));

  // Per-waypoint property signals
  QObject::connect(ui_.depth_editor, SIGNAL(valueChanged(double)), this,
                   SLOT(DepthChanged(double)));
  QObject::connect(ui_.depth_ref_selector, SIGNAL(currentIndexChanged(int)), this,
                   SLOT(DepthRefChanged(int)));
  QObject::connect(ui_.park_checkbox, SIGNAL(stateChanged(int)), this, SLOT(ParkChanged(int)));
  QObject::connect(ui_.speed_override_checkbox, SIGNAL(stateChanged(int)), this,
                   SLOT(SpeedOverrideToggled(int)));
  QObject::connect(ui_.speed_editor, SIGNAL(valueChanged(double)), this,
                   SLOT(SpeedChanged(double)));
  QObject::connect(ui_.slip_radius_override_checkbox, SIGNAL(stateChanged(int)), this,
                   SLOT(SlipRadiusOverrideToggled(int)));
  QObject::connect(ui_.slip_radius_editor, SIGNAL(valueChanged(double)), this,
                   SLOT(SlipRadiusChanged(double)));
  QObject::connect(ui_.capture_radius_override_checkbox, SIGNAL(stateChanged(int)), this,
                   SLOT(CaptureRadiusOverrideToggled(int)));
  QObject::connect(ui_.capture_radius_editor, SIGNAL(valueChanged(double)), this,
                   SLOT(CaptureRadiusChanged(double)));
}

CougWaypointsPlugin::~CougWaypointsPlugin() {
  if (map_canvas_) {
    map_canvas_->removeEventFilter(this);
  }
}

bool CougWaypointsPlugin::Initialize(QGLWidget* canvas) {
  map_canvas_ = dynamic_cast<mapviz::MapCanvas*>(canvas);
  map_canvas_->installEventFilter(this);

  discovery_timer_->start(1000);
  DiscoverTopics();

  initialized_ = true;
  return true;
}

void CougWaypointsPlugin::VisibilityChanged(bool visible) {
  if (visible) {
    map_canvas_->installEventFilter(this);
  } else {
    map_canvas_->removeEventFilter(this);
  }
}

// ---------------------------------------------------------------------------
// Topic discovery
// ---------------------------------------------------------------------------

void CougWaypointsPlugin::DiscoverTopics() {
  auto topics_and_types = node_->get_topic_names_and_types();
  for (const auto& [topic, types] : topics_and_types) {
    for (const auto& type : types) {
      if (type == "geographic_msgs/msg/RouteNetwork") {
        if (ui_.topic_selector->findText(QString::fromStdString(topic)) == -1) {
          ui_.topic_selector->addItem(QString::fromStdString(topic));
        }
        break;
      }
    }
  }
}

void CougWaypointsPlugin::TopicChanged(const QString& text) {
  current_topic_ = text.toStdString();
  selected_point_ = -1;
  SetWaypointEditorsEnabled(false);

  // Populate mission defaults UI for this topic
  UpdateMissionDefaultsUI(manager_.getDefaults(current_topic_));

  auto wps = manager_.getWaypoints(current_topic_);
  if (wps.empty()) {
    PrintInfo("Click to add waypoints");
  } else {
    PrintInfo(current_topic_ + " (" + std::to_string(wps.size()) + " waypoints)");
  }
}

// ---------------------------------------------------------------------------
// Publishing
// ---------------------------------------------------------------------------

static geographic_msgs::msg::WayPoint makeGeoWaypoint(uint32_t index, const CougWaypoint& wp) {
  geographic_msgs::msg::WayPoint gp;
  gp.id.uuid.fill(0);
  gp.id.uuid[12] = (index >> 24) & 0xFF;
  gp.id.uuid[13] = (index >> 16) & 0xFF;
  gp.id.uuid[14] = (index >> 8) & 0xFF;
  gp.id.uuid[15] = index & 0xFF;

  gp.position.latitude = wp.pose.position.y;
  gp.position.longitude = wp.pose.position.x;
  gp.position.altitude = wp.pose.position.z;

  // Required props
  { geographic_msgs::msg::KeyValue kv; kv.key = "depth_ref"; kv.value = wp.depth_ref;
    gp.props.push_back(kv); }
  { geographic_msgs::msg::KeyValue kv; kv.key = "park";
    kv.value = wp.park ? "true" : "false"; gp.props.push_back(kv); }

  // Optional per-waypoint override props
  if (wp.speed.has_value()) {
    geographic_msgs::msg::KeyValue kv;
    kv.key = "speed";
    kv.value = std::to_string(*wp.speed);
    gp.props.push_back(kv);
  }
  if (wp.slip_radius.has_value()) {
    geographic_msgs::msg::KeyValue kv;
    kv.key = "slip_radius";
    kv.value = std::to_string(*wp.slip_radius);
    gp.props.push_back(kv);
  }
  if (wp.capture_radius.has_value()) {
    geographic_msgs::msg::KeyValue kv;
    kv.key = "capture_radius";
    kv.value = std::to_string(*wp.capture_radius);
    gp.props.push_back(kv);
  }

  return gp;
}

void CougWaypointsPlugin::PublishTopic(const std::string& topic,
                                       const std::vector<CougWaypoint>& wps) {
  if (publishers_.find(topic) == publishers_.end()) {
    publishers_[topic] = node_->create_publisher<geographic_msgs::msg::RouteNetwork>(
        topic, rclcpp::SystemDefaultsQoS());
  }

  auto msg = std::make_unique<geographic_msgs::msg::RouteNetwork>();
  msg->header.frame_id = "wgs84";
  msg->header.stamp = node_->now();

  // Mission ID goes into the network's id field
  const MissionDefaults& d = manager_.getDefaults(topic);
  msg->id.uuid.fill(0);
  msg->id.uuid[12] = (static_cast<uint32_t>(d.mission_id) >> 24) & 0xFF;
  msg->id.uuid[13] = (static_cast<uint32_t>(d.mission_id) >> 16) & 0xFF;
  msg->id.uuid[14] = (static_cast<uint32_t>(d.mission_id) >> 8) & 0xFF;
  msg->id.uuid[15] = static_cast<uint32_t>(d.mission_id) & 0xFF;

  // Mission-level defaults go in the network's own props
  auto addProp = [&](const std::string& k, const std::string& v) {
    geographic_msgs::msg::KeyValue kv;
    kv.key = k;
    kv.value = v;
    msg->props.push_back(kv);
  };
  addProp("speed", std::to_string(d.speed));
  addProp("slip_radius", std::to_string(d.slip_radius));
  addProp("capture_radius", std::to_string(d.capture_radius));

  // Waypoints
  for (size_t i = 0; i < wps.size(); i++) {
    msg->points.push_back(makeGeoWaypoint(static_cast<uint32_t>(i), wps[i]));
  }

  publishers_[topic]->publish(*msg);
}

void CougWaypointsPlugin::PublishWaypoints() {
  if (ui_.apply_all->isChecked()) {
    PublishAll();
  } else if (!current_topic_.empty()) {
    auto wps = manager_.getWaypoints(current_topic_);
    PublishTopic(current_topic_, wps);
    if (wps.empty()) {
      PrintWarning("Stopped");
    } else {
      PrintInfo("Published " + std::to_string(wps.size()) + " waypoints");
    }
  } else {
    PrintError("No topic selected");
  }
}

void CougWaypointsPlugin::Stop() {
  if (ui_.apply_all->isChecked()) {
    StopAll();
  } else if (!current_topic_.empty()) {
    PublishTopic(current_topic_, {});
    PrintWarning("Stopped");
  }
}

void CougWaypointsPlugin::PublishAll() {
  int count = 0;
  for (const auto& [topic, wps] : manager_.getAllWaypoints()) {
    PublishTopic(topic, wps);
    count++;
  }
  PrintInfo("Published all (" + std::to_string(count) + " topics)");
}

void CougWaypointsPlugin::StopAll() {
  for (const auto& [topic, wps] : manager_.getAllWaypoints()) {
    (void)wps;
    PublishTopic(topic, {});
  }
  PrintWarning("Stopped all (" + std::to_string(manager_.getAllWaypoints().size()) + " topics)");
}

bool CougWaypointsPlugin::IsTopicAvailable(const std::string& topic) {
  return ui_.topic_selector->findText(QString::fromStdString(topic)) != -1;
}

// ---------------------------------------------------------------------------
// File I/O
// ---------------------------------------------------------------------------

void CougWaypointsPlugin::SaveWaypoints() {
  const char* overlay_ws = std::getenv("OVERLAY_WS");
  QString path = QString::fromUtf8(overlay_ws) + "/src/coug_mapviz/missions";
  QDir dir(path);
  if (!dir.exists()) dir.mkpath(".");

  QString filename =
      QFileDialog::getSaveFileName(config_widget_, "Save Mission", path, "JSON Files (*.json)");
  if (filename.isEmpty()) return;
  if (!filename.endsWith(".json", Qt::CaseInsensitive)) filename += ".json";

  std::string topic_to_save;
  if (!ui_.apply_all->isChecked()) {
    if (current_topic_.empty()) { PrintError("No topic selected to save"); return; }
    topic_to_save = current_topic_;
  }

  if (manager_.saveToFile(filename.toStdString(), topic_to_save)) {
    if (topic_to_save.empty()) {
      PrintInfo("Saved all (" + std::to_string(manager_.getAllWaypoints().size()) + " topics)");
    } else {
      PrintInfo("Saved topic: " + topic_to_save);
    }
  } else {
    PrintError("Failed to save file");
  }
}

void CougWaypointsPlugin::LoadWaypoints() {
  const char* overlay_ws = std::getenv("OVERLAY_WS");
  QString path = QString::fromUtf8(overlay_ws) + "/src/coug_mapviz/missions";
  QString filename =
      QFileDialog::getOpenFileName(config_widget_, "Load Mission", path, "JSON Files (*.json)");
  if (filename.isEmpty()) return;

  std::string topic_to_load;
  if (!ui_.apply_all->isChecked()) {
    if (current_topic_.empty()) { PrintError("No topic selected to load"); return; }
    topic_to_load = current_topic_;
  }

  if (manager_.loadFromFile(filename.toStdString(), topic_to_load)) {
    std::vector<std::string> unavailable;
    for (const auto& [topic, wps] : manager_.getAllWaypoints()) {
      (void)wps;
      if (!IsTopicAvailable(topic)) unavailable.push_back(topic);
    }
    for (const auto& t : unavailable) manager_.removeTopic(t);

    TopicChanged(QString::fromStdString(current_topic_));
    if (topic_to_load.empty()) {
      PrintInfo("Loaded all topics (" + std::to_string(manager_.getAllWaypoints().size()) +
                " total)");
    } else {
      auto wps = manager_.getWaypoints(topic_to_load);
      PrintInfo("Loaded: " + topic_to_load + " (" + std::to_string(wps.size()) + " waypoints)");
    }
  } else {
    PrintError("Failed to load file");
  }
}

// ---------------------------------------------------------------------------
// Clear
// ---------------------------------------------------------------------------

void CougWaypointsPlugin::Clear() {
  if (ui_.apply_all->isChecked()) {
    manager_.clearAllWaypoints();
  } else {
    manager_.clearWaypoints(current_topic_);
  }

  ui_.depth_editor->setValue(0.0);
  ui_.speed_override_checkbox->blockSignals(true);
  ui_.speed_override_checkbox->setChecked(false);
  ui_.speed_override_checkbox->blockSignals(false);
  ui_.slip_radius_override_checkbox->blockSignals(true);
  ui_.slip_radius_override_checkbox->setChecked(false);
  ui_.slip_radius_override_checkbox->blockSignals(false);
  ui_.capture_radius_override_checkbox->blockSignals(true);
  ui_.capture_radius_override_checkbox->setChecked(false);
  ui_.capture_radius_override_checkbox->blockSignals(false);

  selected_point_ = -1;
  dragged_point_ = -1;
  SetWaypointEditorsEnabled(false);
  PrintInfo("Cleared waypoints");
}

// ---------------------------------------------------------------------------
// Drawing — OpenGL (Draw) and QPainter (Paint)
// ---------------------------------------------------------------------------

void CougWaypointsPlugin::DrawWaypointCircles(const std::vector<CougWaypoint>& wps,
                                              const swri_transform_util::Transform& transform,
                                              const MissionDefaults& defaults) {
  for (const auto& wp : wps) {
    tf2::Vector3 center(wp.pose.position.x, wp.pose.position.y, 0);
    center = transform * center;
    double cx = center.x(), cy = center.y();

    double cap_r = wp.capture_radius.has_value() ? *wp.capture_radius : defaults.capture_radius;
    double slip_r = wp.slip_radius.has_value() ? *wp.slip_radius : defaults.slip_radius;

    // Capture radius — light blue, very transparent fill + solid outline
    DrawFilledCircle(cx, cy, cap_r, 0.2f, 0.6f, 1.0f, 0.12f);
    DrawCircleOutline(cx, cy, cap_r, 0.2f, 0.6f, 1.0f, 0.65f);

    // Slip radius — orange, slightly more opaque fill + solid outline
    DrawFilledCircle(cx, cy, slip_r, 1.0f, 0.55f, 0.0f, 0.18f);
    DrawCircleOutline(cx, cy, slip_r, 1.0f, 0.55f, 0.0f, 0.75f);
  }
}

void CougWaypointsPlugin::Draw(double x, double y, double scale) {
  (void)x;
  (void)y;
  (void)scale;

  swri_transform_util::Transform transform;
  if (!tf_manager_->GetTransform(target_frame_, swri_transform_util::_wgs84_frame, transform)) {
    return;
  }

  // Draw radius circles first (behind everything)
  glEnable(GL_BLEND);
  glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
  for (const auto& [topic, wps] : manager_.getAllWaypoints()) {
    DrawWaypointCircles(wps, transform, manager_.getDefaults(topic));
  }
  glDisable(GL_BLEND);

  // Draw path lines for other topics
  glLineWidth(2);
  for (const auto& [topic, wps] : manager_.getAllWaypoints()) {
    if (topic != current_topic_ && IsTopicAvailable(topic)) {
      DrawPath(wps, QColor(Qt::white), transform, -1);
    }
  }
}

void CougWaypointsPlugin::DrawPath(const std::vector<CougWaypoint>& wps, const QColor& color,
                                   const swri_transform_util::Transform& transform,
                                   int selected_index) {
  if (color == Qt::blue) {
    glColor4f(0.0, 0.0, 1.0, 1.0);
  } else {
    glColor4f(1.0, 1.0, 1.0, 1.0);
  }

  glBegin(GL_LINE_STRIP);
  for (const auto& wp : wps) {
    tf2::Vector3 point(wp.pose.position.x, wp.pose.position.y, 0);
    point = transform * point;
    glVertex2d(point.x(), point.y());
  }
  glEnd();

  glPointSize(20);
  glBegin(GL_POINTS);
  for (size_t i = 0; i < wps.size(); ++i) {
    if (static_cast<int>(i) == selected_index) {
      glColor4f(1.0, 1.0, 0.0, 1.0);
    } else if (color == Qt::blue) {
      glColor4f(0.0, 1.0, 1.0, 1.0);
    } else {
      glColor4f(0.5, 0.5, 0.5, 1.0);
    }
    tf2::Vector3 point(wps[i].pose.position.x, wps[i].pose.position.y, 0);
    point = transform * point;
    glVertex2d(point.x(), point.y());
  }
  glEnd();
}

void CougWaypointsPlugin::Paint(QPainter* painter, double x, double y, double scale) {
  (void)x;
  (void)y;
  (void)scale;

  swri_transform_util::Transform transform;
  if (!tf_manager_->GetTransform(target_frame_, swri_transform_util::_wgs84_frame, transform)) {
    return;
  }

  painter->save();
  painter->resetTransform();
  painter->setFont(QFont("DejaVu Sans Mono", 10, QFont::Bold));

  for (const auto& [topic, wps] : manager_.getAllWaypoints()) {
    if (topic != current_topic_ && IsTopicAvailable(topic)) {
      PaintLabels(painter, wps, transform, QColor(255, 255, 255, 200));
    }
  }

  {
    auto wps = manager_.getWaypoints(current_topic_);
    if (!wps.empty()) {
      PaintPath(painter, wps, QColor(Qt::blue), transform, selected_point_);
      PaintLabels(painter, wps, transform, Qt::white);
    }
  }

  painter->restore();
}

void CougWaypointsPlugin::PaintPath(QPainter* painter, const std::vector<CougWaypoint>& wps,
                                    const QColor& color,
                                    const swri_transform_util::Transform& transform,
                                    int selected_index) {
  QVector<QPointF> points;
  for (const auto& wp : wps) {
    tf2::Vector3 point(wp.pose.position.x, wp.pose.position.y, 0);
    point = transform * point;
    points.push_back(map_canvas_->FixedFrameToMapGlCoord(QPointF(point.x(), point.y())));
  }

  painter->setPen(QPen(color, 2));
  painter->drawPolyline(points);

  for (int i = 0; i < points.size(); ++i) {
    if (i == selected_index) {
      painter->setPen(QPen(Qt::yellow, 20, Qt::SolidLine, Qt::RoundCap));
    } else if (color == Qt::blue) {
      painter->setPen(QPen(Qt::cyan, 20, Qt::SolidLine, Qt::RoundCap));
    } else {
      painter->setPen(QPen(Qt::gray, 20, Qt::SolidLine, Qt::RoundCap));
    }
    painter->drawPoint(points[i]);
  }
}

void CougWaypointsPlugin::PaintLabels(QPainter* painter, const std::vector<CougWaypoint>& wps,
                                      const swri_transform_util::Transform& transform,
                                      const QColor& color) {
  for (size_t i = 0; i < wps.size(); i++) {
    tf2::Vector3 point(wps[i].pose.position.x, wps[i].pose.position.y, 0);
    point = transform * point;
    QPointF gl = map_canvas_->FixedFrameToMapGlCoord(QPointF(point.x(), point.y()));

    painter->setPen(QPen(color));
    // "S:-10.0m" or "B:5.0m"
    QString prefix = (wps[i].depth_ref == "bottom") ? "B:" : "S:";
    QString depth_text = prefix + QString::number(wps[i].pose.position.z, 'f', 1) + "m";
    painter->drawText(QRectF(QPointF(gl.x() - 50, gl.y() + 15), QSizeF(100, 20)),
                      Qt::AlignHCenter | Qt::AlignTop, depth_text);

    // Waypoint index, "P" suffix if park
    painter->setPen(QPen(color == Qt::white ? Qt::black : color));
    QString num_text = QString::number(i + 1);
    if (wps[i].park) num_text += "P";
    painter->drawText(QRectF(QPointF(gl.x() - 20, gl.y() - 20), QSizeF(40, 40)),
                      Qt::AlignHCenter | Qt::AlignVCenter, num_text);
  }
}

// ---------------------------------------------------------------------------
// UI sync helpers
// ---------------------------------------------------------------------------

void CougWaypointsPlugin::SetWaypointEditorsEnabled(bool enabled) {
  ui_.depth_editor->setEnabled(enabled);
  ui_.depth_ref_selector->setEnabled(enabled);
  ui_.park_checkbox->setEnabled(enabled);
  ui_.speed_override_checkbox->setEnabled(enabled);
  ui_.speed_editor->setEnabled(enabled && ui_.speed_override_checkbox->isChecked());
  ui_.slip_radius_override_checkbox->setEnabled(enabled);
  ui_.slip_radius_editor->setEnabled(enabled && ui_.slip_radius_override_checkbox->isChecked());
  ui_.capture_radius_override_checkbox->setEnabled(enabled);
  ui_.capture_radius_editor->setEnabled(enabled &&
                                        ui_.capture_radius_override_checkbox->isChecked());
}

void CougWaypointsPlugin::UpdateEditorsFromWaypoint(const CougWaypoint& wp) {
  ui_.depth_editor->blockSignals(true);
  ui_.depth_editor->setValue(wp.pose.position.z);
  ui_.depth_editor->blockSignals(false);

  ui_.depth_ref_selector->blockSignals(true);
  ui_.depth_ref_selector->setCurrentText(QString::fromStdString(wp.depth_ref));
  ui_.depth_ref_selector->blockSignals(false);

  ui_.park_checkbox->blockSignals(true);
  ui_.park_checkbox->setChecked(wp.park);
  ui_.park_checkbox->blockSignals(false);

  bool has_speed = wp.speed.has_value();
  ui_.speed_override_checkbox->blockSignals(true);
  ui_.speed_override_checkbox->setChecked(has_speed);
  ui_.speed_override_checkbox->blockSignals(false);
  ui_.speed_editor->setEnabled(has_speed);
  if (has_speed) {
    ui_.speed_editor->blockSignals(true);
    ui_.speed_editor->setValue(*wp.speed);
    ui_.speed_editor->blockSignals(false);
  }

  bool has_slip = wp.slip_radius.has_value();
  ui_.slip_radius_override_checkbox->blockSignals(true);
  ui_.slip_radius_override_checkbox->setChecked(has_slip);
  ui_.slip_radius_override_checkbox->blockSignals(false);
  ui_.slip_radius_editor->setEnabled(has_slip);
  if (has_slip) {
    ui_.slip_radius_editor->blockSignals(true);
    ui_.slip_radius_editor->setValue(*wp.slip_radius);
    ui_.slip_radius_editor->blockSignals(false);
  }

  bool has_cap = wp.capture_radius.has_value();
  ui_.capture_radius_override_checkbox->blockSignals(true);
  ui_.capture_radius_override_checkbox->setChecked(has_cap);
  ui_.capture_radius_override_checkbox->blockSignals(false);
  ui_.capture_radius_editor->setEnabled(has_cap);
  if (has_cap) {
    ui_.capture_radius_editor->blockSignals(true);
    ui_.capture_radius_editor->setValue(*wp.capture_radius);
    ui_.capture_radius_editor->blockSignals(false);
  }
}

void CougWaypointsPlugin::UpdateMissionDefaultsUI(const MissionDefaults& d) {
  ui_.mission_id_editor->blockSignals(true);
  ui_.mission_id_editor->setValue(d.mission_id);
  ui_.mission_id_editor->blockSignals(false);

  ui_.default_speed_editor->blockSignals(true);
  ui_.default_speed_editor->setValue(d.speed);
  ui_.default_speed_editor->blockSignals(false);

  ui_.default_slip_radius_editor->blockSignals(true);
  ui_.default_slip_radius_editor->setValue(d.slip_radius);
  ui_.default_slip_radius_editor->blockSignals(false);

  ui_.default_capture_radius_editor->blockSignals(true);
  ui_.default_capture_radius_editor->setValue(d.capture_radius);
  ui_.default_capture_radius_editor->blockSignals(false);
}

// ---------------------------------------------------------------------------
// Mission-level default slots
// ---------------------------------------------------------------------------

void CougWaypointsPlugin::DefaultMissionIdChanged(int value) {
  auto d = manager_.getDefaults(current_topic_);
  d.mission_id = value;
  manager_.setDefaults(current_topic_, d);
}

void CougWaypointsPlugin::DefaultSpeedChanged(double value) {
  auto d = manager_.getDefaults(current_topic_);
  d.speed = value;
  manager_.setDefaults(current_topic_, d);
}

void CougWaypointsPlugin::DefaultSlipRadiusChanged(double value) {
  auto d = manager_.getDefaults(current_topic_);
  d.slip_radius = value;
  manager_.setDefaults(current_topic_, d);
}

void CougWaypointsPlugin::DefaultCaptureRadiusChanged(double value) {
  auto d = manager_.getDefaults(current_topic_);
  d.capture_radius = value;
  manager_.setDefaults(current_topic_, d);
}

// ---------------------------------------------------------------------------
// Per-waypoint property slots
// ---------------------------------------------------------------------------

void CougWaypointsPlugin::DepthChanged(double value) {
  if (selected_point_ < 0) return;
  auto wps = manager_.getWaypoints(current_topic_);
  if (static_cast<size_t>(selected_point_) >= wps.size()) return;
  wps[selected_point_].pose.position.z = value;
  manager_.setWaypoints(current_topic_, wps);
}

void CougWaypointsPlugin::DepthRefChanged(int /*index*/) {
  if (selected_point_ < 0) return;
  auto wps = manager_.getWaypoints(current_topic_);
  if (static_cast<size_t>(selected_point_) >= wps.size()) return;
  wps[selected_point_].depth_ref = ui_.depth_ref_selector->currentText().toStdString();
  manager_.setWaypoints(current_topic_, wps);
}

void CougWaypointsPlugin::ParkChanged(int /*state*/) {
  if (selected_point_ < 0) return;
  auto wps = manager_.getWaypoints(current_topic_);
  if (static_cast<size_t>(selected_point_) >= wps.size()) return;
  wps[selected_point_].park = ui_.park_checkbox->isChecked();
  manager_.setWaypoints(current_topic_, wps);
}

void CougWaypointsPlugin::SpeedOverrideToggled(int state) {
  bool enabled = (state == Qt::Checked);
  ui_.speed_editor->setEnabled(enabled);
  if (selected_point_ < 0) return;
  auto wps = manager_.getWaypoints(current_topic_);
  if (static_cast<size_t>(selected_point_) >= wps.size()) return;
  wps[selected_point_].speed =
      enabled ? std::optional<double>(ui_.speed_editor->value()) : std::nullopt;
  manager_.setWaypoints(current_topic_, wps);
}

void CougWaypointsPlugin::SpeedChanged(double value) {
  if (selected_point_ < 0) return;
  auto wps = manager_.getWaypoints(current_topic_);
  if (static_cast<size_t>(selected_point_) >= wps.size()) return;
  if (wps[selected_point_].speed.has_value()) {
    wps[selected_point_].speed = value;
    manager_.setWaypoints(current_topic_, wps);
  }
}

void CougWaypointsPlugin::SlipRadiusOverrideToggled(int state) {
  bool enabled = (state == Qt::Checked);
  ui_.slip_radius_editor->setEnabled(enabled);
  if (selected_point_ < 0) return;
  auto wps = manager_.getWaypoints(current_topic_);
  if (static_cast<size_t>(selected_point_) >= wps.size()) return;
  wps[selected_point_].slip_radius =
      enabled ? std::optional<double>(ui_.slip_radius_editor->value()) : std::nullopt;
  manager_.setWaypoints(current_topic_, wps);
}

void CougWaypointsPlugin::SlipRadiusChanged(double value) {
  if (selected_point_ < 0) return;
  auto wps = manager_.getWaypoints(current_topic_);
  if (static_cast<size_t>(selected_point_) >= wps.size()) return;
  if (wps[selected_point_].slip_radius.has_value()) {
    wps[selected_point_].slip_radius = value;
    manager_.setWaypoints(current_topic_, wps);
  }
}

void CougWaypointsPlugin::CaptureRadiusOverrideToggled(int state) {
  bool enabled = (state == Qt::Checked);
  ui_.capture_radius_editor->setEnabled(enabled);
  if (selected_point_ < 0) return;
  auto wps = manager_.getWaypoints(current_topic_);
  if (static_cast<size_t>(selected_point_) >= wps.size()) return;
  wps[selected_point_].capture_radius =
      enabled ? std::optional<double>(ui_.capture_radius_editor->value()) : std::nullopt;
  manager_.setWaypoints(current_topic_, wps);
}

void CougWaypointsPlugin::CaptureRadiusChanged(double value) {
  if (selected_point_ < 0) return;
  auto wps = manager_.getWaypoints(current_topic_);
  if (static_cast<size_t>(selected_point_) >= wps.size()) return;
  if (wps[selected_point_].capture_radius.has_value()) {
    wps[selected_point_].capture_radius = value;
    manager_.setWaypoints(current_topic_, wps);
  }
}

// ---------------------------------------------------------------------------
// Mouse interaction
// ---------------------------------------------------------------------------

bool CougWaypointsPlugin::eventFilter(QObject* object, QEvent* event) {
  (void)object;
  switch (event->type()) {
    case QEvent::MouseButtonPress:
      return handleMousePress(dynamic_cast<QMouseEvent*>(event));
    case QEvent::MouseButtonRelease:
      return handleMouseRelease(dynamic_cast<QMouseEvent*>(event));
    case QEvent::MouseMove:
      return handleMouseMove(dynamic_cast<QMouseEvent*>(event));
    default:
      return false;
  }
}

int CougWaypointsPlugin::GetClosestPoint(const QPointF& point, double& distance) {
  swri_transform_util::Transform transform;
  if (!tf_manager_->GetTransform(target_frame_, swri_transform_util::_wgs84_frame, transform)) {
    return -1;
  }

  int closest = -1;
  distance = std::numeric_limits<double>::max();
  auto wps = manager_.getWaypoints(current_topic_);

  for (size_t i = 0; i < wps.size(); i++) {
    tf2::Vector3 wp(wps[i].pose.position.x, wps[i].pose.position.y, 0);
    wp = transform * wp;
    QPointF transformed = map_canvas_->FixedFrameToMapGlCoord(QPointF(wp.x(), wp.y()));
    double d = QLineF(transformed, point).length();
    if (d < distance) { distance = d; closest = static_cast<int>(i); }
  }
  return closest;
}

bool CougWaypointsPlugin::handleMousePress(QMouseEvent* event) {
  dragged_point_ = -1;
  double distance = 0.0;
  int closest_point = GetClosestPoint(event->localPos(), distance);

  if (event->button() == Qt::LeftButton) {
    is_mouse_down_ = true;
    mouse_down_pos_ = event->localPos();
    mouse_down_time_ = QDateTime::currentMSecsSinceEpoch();
    if (distance < 15) { dragged_point_ = closest_point; return true; }
  } else if (event->button() == Qt::RightButton) {
    if (distance < 15) {
      auto wps = manager_.getWaypoints(current_topic_);
      wps.erase(wps.begin() + closest_point);
      manager_.setWaypoints(current_topic_, wps);
      if (selected_point_ == closest_point) {
        selected_point_ = -1;
        SetWaypointEditorsEnabled(false);
      }
      return true;
    }
  }
  return false;
}

bool CougWaypointsPlugin::handleMouseRelease(QMouseEvent* event) {
  qreal dist = QLineF(mouse_down_pos_, event->localPos()).length();
  qint64 ms = QDateTime::currentMSecsSinceEpoch() - mouse_down_time_;

  if (dragged_point_ != -1) {
    if (dist <= 5.0 && ms < 500) {
      // Short click on existing point → select it
      selected_point_ = dragged_point_;
      auto wps = manager_.getWaypoints(current_topic_);
      SetWaypointEditorsEnabled(true);
      UpdateEditorsFromWaypoint(wps[selected_point_]);
    }
    dragged_point_ = -1;
    return true;
  }

  if (is_mouse_down_ && ms < 500 && dist <= 5.0) {
    // Short click on empty space → place new waypoint
    QPointF transformed = map_canvas_->MapGlCoordToFixedFrame(event->localPos());
    swri_transform_util::Transform transform;
    if (tf_manager_->GetTransform(swri_transform_util::_wgs84_frame, target_frame_, transform)) {
      tf2::Vector3 position(transformed.x(), transformed.y(), 0.0);
      position = transform * position;

      CougWaypoint wp;
      wp.pose.position.x = position.x();
      wp.pose.position.y = position.y();
      wp.pose.position.z = ui_.depth_editor->value();
      wp.depth_ref = ui_.depth_ref_selector->currentText().toStdString();
      wp.park = ui_.park_checkbox->isChecked();
      if (ui_.speed_override_checkbox->isChecked())
        wp.speed = ui_.speed_editor->value();
      if (ui_.slip_radius_override_checkbox->isChecked())
        wp.slip_radius = ui_.slip_radius_editor->value();
      if (ui_.capture_radius_override_checkbox->isChecked())
        wp.capture_radius = ui_.capture_radius_editor->value();

      manager_.addWaypoint(current_topic_, wp);
    }
  }

  is_mouse_down_ = false;
  dragged_point_ = -1;
  return false;
}

bool CougWaypointsPlugin::handleMouseMove(QMouseEvent* event) {
  if (dragged_point_ >= 0) {
    if (selected_point_ != -1) {
      selected_point_ = -1;
      SetWaypointEditorsEnabled(false);
    }

    swri_transform_util::Transform transform;
    if (tf_manager_->GetTransform(swri_transform_util::_wgs84_frame, target_frame_, transform)) {
      QPointF transformed = map_canvas_->MapGlCoordToFixedFrame(event->localPos());
      tf2::Vector3 position(transformed.x(), transformed.y(), 0.0);
      position = transform * position;

      auto wps = manager_.getWaypoints(current_topic_);
      wps[dragged_point_].pose.position.x = position.x();
      wps[dragged_point_].pose.position.y = position.y();
      manager_.setWaypoints(current_topic_, wps);
    }
    return true;
  }
  return false;
}

// ---------------------------------------------------------------------------
// Config
// ---------------------------------------------------------------------------

void CougWaypointsPlugin::LoadConfig(const YAML::Node& node, const std::string& path) {
  (void)node;
  (void)path;
}

void CougWaypointsPlugin::SaveConfig(YAML::Emitter& emitter, const std::string& path) {
  (void)emitter;
  (void)path;
}

QWidget* CougWaypointsPlugin::GetConfigWidget(QWidget* parent) {
  config_widget_->setParent(parent);
  return config_widget_;
}

void CougWaypointsPlugin::PrintError(const std::string& message) {
  PrintErrorHelper(ui_.status, message, 1.0);
}
void CougWaypointsPlugin::PrintInfo(const std::string& message) {
  PrintInfoHelper(ui_.status, message, 1.0);
}
void CougWaypointsPlugin::PrintWarning(const std::string& message) {
  PrintWarningHelper(ui_.status, message, 1.0);
}

}  // namespace coug_mapviz
