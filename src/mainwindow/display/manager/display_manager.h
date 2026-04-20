#ifndef DISPLAY_MANAGER_H
#define MAINWINDOW_DISPLAY_MANAGER_H

#include <any>
#include <map>
#include <string>
#include <vector>

#include <QObject>
#include <QPointF>
#include <QTimer>

#include "display/display_cost_map.h"
#include "display/display_demo.h"
#include "display/display_occ_map.h"
#include "display/display_path.h"
#include "display/laser_points.h"
#include "display/manager/display_factory.h"
#include "display/point_shape.h"
#include "display/robot_shape.h"
#include "display/virtual_display.h"
#include "display/manager/view_manager.h"
#include "display/manager/scene_manager.h"
#include "widgets/set_pose_widget.h"
#include "occupancy_map.h"
#include "topology_map.h"
#include "core/framework/framework.h"

namespace Display {

class DisplayManager : public QObject {
  Q_OBJECT
 public:
  DisplayManager();
  ~DisplayManager();
  bool SetDisplayConfig(const std::string &config_name, const std::any &data);
  VirtualDisplay *GetDisplay(const std::string &name);
  void UpdateRobotPose(const RobotPose &pose);
  void SetRelocMode(bool is_start);
  void StartReloc();
  void SetEditMapMode(MapEditMode mode);
  void SetToolRange(double range);
  double GetEraserRange() const;
  double GetPenRange() const;
  void AddOneNavPoint();
  void AddPointAtRobotPosition();
  OccupancyMap &GetMap();
  OccupancyMap GetOccupancyMap();
  void UpdateOCCMap(const OccupancyMap &map);
  TopologyMap GetTopologyMap();
  void UpdateTopologyMap(const TopologyMap &topology_map);
  void SetScaleBig();
  void SetScaleSmall();
  QWidget *GetViewPtr() { return graphics_view_ptr_; }
  void FocusDisplay(const std::string &display_name);
  RobotPose GetRobotPose() const { return robot_pose_; }
  RobotPose wordPose2Scene(const RobotPose &point);
  QPointF wordPose2Scene(const QPointF &point);
  RobotPose wordPose2Map(const RobotPose &pose);
  QPointF wordPose2Map(const QPointF &pose);
  RobotPose mapPose2Word(const RobotPose &pose);
  RobotPose scenePoseToWord(const RobotPose &pose);
  RobotPose scenePoseToMap(const RobotPose &pose);

 signals:
  void signalPub2DPose(const RobotPose &pose);
  void signalPub2DGoal(const RobotPose &pose);
  void signalTopologyMapUpdate(const TopologyMap &topology_map);
  void signalCurrentSelectPointChanged(const TopologyMap::PointInfo &point);
  void signalEditMapModeChanged(MapEditMode mode);
  void signalCursorPose(QPointF pos);

 public slots:
  void slotRobotScenePoseChanged(const RobotPose &pose);
  void slotSetRobotPose(const RobotPose &pose);
  void updateScaled(double value);

 private:
  void InitUi();
  std::vector<Point> transLaserPoint(const std::vector<Point> &point);

  ViewManager *graphics_view_ptr_;
  SceneManager *scene_manager_ptr_;
  RobotPose robot_pose_;
  OccupancyMap map_data_;
  bool is_reloc_mode_ = false;
  SetPoseWidget *set_reloc_pose_widget_;

  std::map<std::string, std::vector<Framework::MessageBus::CallbackId>> subscription_ids_;
  void AddSubscription(const std::string& topic, Framework::MessageBus::CallbackId id) {
    subscription_ids_[topic].push_back(id);
  }
  void ClearSubscriptions() {
    for (auto& [topic, ids] : subscription_ids_) {
      for (auto id : ids) {
        UNSUBSCRIBE(topic, id);
      }
    }
    subscription_ids_.clear();
  }
};
}  // namespace Display
#endif
