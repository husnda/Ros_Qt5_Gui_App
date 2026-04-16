/*
 * @Author: chengyang chengyangkj@outlook.com
 * @Date: 2023-07-27 14:47:24
 * @LastEditors: chengyangkj chengyangkj@qq.com
 * @LastEditTime: 2023-10-15 09:14:17
 * @FilePath: /ros_qt5_gui_app/src/channel/ros2/rclcomm.cpp
 * @Description: ros2通讯类
 */
#include "rclcomm.h"
#include <fstream>
#include <chrono>
#include <tf2/LinearMath/Quaternion.h>
#include "config/config_manager.h"
#include "logger/logger.h"
#include "core/framework/framework.h"
#include "msg/msg_info.h"

rclcomm::rclcomm() {
  SET_DEFAULT_TOPIC_NAME(DISPLAY_GOAL, "/goal_pose")
  SET_DEFAULT_TOPIC_NAME(MSG_ID_SET_RELOC_POSE, "/initialpose")
  SET_DEFAULT_TOPIC_NAME(DISPLAY_MAP, "/map")
  SET_DEFAULT_TOPIC_NAME(DISPLAY_LOCAL_COST_MAP, "/local_costmap/costmap")
  SET_DEFAULT_TOPIC_NAME(DISPLAY_GLOBAL_COST_MAP, "/global_costmap/costmap")
  SET_DEFAULT_TOPIC_NAME(DISPLAY_LASER, "/scan")
  SET_DEFAULT_TOPIC_NAME(DISPLAY_GLOBAL_PATH, "/plan")
  SET_DEFAULT_TOPIC_NAME(DISPLAY_LOCAL_PATH, "/local_plan")
  SET_DEFAULT_TOPIC_NAME(DISPLAY_ROBOT, "/odom")
  SET_DEFAULT_TOPIC_NAME(MSG_ID_SET_ROBOT_SPEED, "/cmd_vel")
  SET_DEFAULT_TOPIC_NAME(MSG_ID_BATTERY_STATE, "/battery")
  SET_DEFAULT_TOPIC_NAME(DISPLAY_ROBOT_FOOTPRINT, "/local_costmap/published_footprint")
  SET_DEFAULT_TOPIC_NAME(DISPLAY_TOPOLOGY_MAP, "/map/topology")
  SET_DEFAULT_TOPIC_NAME(MSG_ID_TOPOLOGY_MAP_UPDATE, "/map/topology/update")
  SET_DEFAULT_KEY_VALUE("BaseFrameId", "base_link")
  if (Config::ConfigManager::Instance()->GetRootConfig().images.empty()) {
    Config::ConfigManager::Instance()->GetRootConfig().images.push_back(
        Config::ImageDisplayConfig{.location = "front",
                                   .topic = "/camera/front/image_raw",
                                   .enable = true});
  }
  Config::ConfigManager::Instance()->StoreConfig();
}

rclcomm::~rclcomm() {
  Stop();
}

bool rclcomm::Start() {
  std::lock_guard<std::mutex> lock(executor_mutex_);
  
  if (!rclcpp::ok()) {
    try {
      rclcpp::init(0, nullptr);
    } catch (const std::exception& e) {
      LOG_ERROR("rclcpp::init failed: " << e.what());
      return false;
    }
  }
  
  try {
    m_executor = std::make_unique<rclcpp::executors::MultiThreadedExecutor>();

    std::string node_name = "ros_qt5_gui_app_" + std::to_string(std::chrono::system_clock::now().time_since_epoch().count() % 1000000);
    node = rclcpp::Node::make_shared(node_name);
    m_executor->add_node(node);
    
    callback_group_laser =
        node->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    callback_group_other =
        node->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

    auto sub1_obt = rclcpp::SubscriptionOptions();
    sub1_obt.callback_group = callback_group_other;
    auto sub_laser_obt = rclcpp::SubscriptionOptions();
    sub_laser_obt.callback_group = callback_group_laser;

    nav_goal_publisher_ = node->create_publisher<geometry_msgs::msg::PoseStamped>(
        GET_TOPIC_NAME(DISPLAY_GOAL), 10);
    reloc_pose_publisher_ =
        node->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(
            GET_TOPIC_NAME(MSG_ID_SET_RELOC_POSE), 10);
    speed_publisher_ = node->create_publisher<geometry_msgs::msg::Twist>(
        GET_TOPIC_NAME(MSG_ID_SET_ROBOT_SPEED), 10);

    // Explicitly create subscriptions to ensure type safety and correct API usage
    try {
        map_subscriber_ = node->create_subscription<nav_msgs::msg::OccupancyGrid>(
            GET_TOPIC_NAME(DISPLAY_MAP), rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local(),
            std::bind(&rclcomm::map_callback, this, std::placeholders::_1), sub1_obt);
        
        local_cost_map_subscriber_ = node->create_subscription<nav_msgs::msg::OccupancyGrid>(
            GET_TOPIC_NAME(DISPLAY_LOCAL_COST_MAP), rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local(),
            std::bind(&rclcomm::localCostMapCallback, this, std::placeholders::_1), sub1_obt);
            
        global_cost_map_subscriber_ = node->create_subscription<nav_msgs::msg::OccupancyGrid>(
            GET_TOPIC_NAME(DISPLAY_GLOBAL_COST_MAP), rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local(),
            std::bind(&rclcomm::globalCostMapCallback, this, std::placeholders::_1), sub1_obt);

        laser_scan_subscriber_ = node->create_subscription<sensor_msgs::msg::LaserScan>(
            GET_TOPIC_NAME(DISPLAY_LASER), 20,
            std::bind(&rclcomm::laser_callback, this, std::placeholders::_1), sub_laser_obt);

        battery_state_subscriber_ = node->create_subscription<sensor_msgs::msg::BatteryState>(
            GET_TOPIC_NAME(MSG_ID_BATTERY_STATE), 1,
            std::bind(&rclcomm::BatteryCallback, this, std::placeholders::_1), sub1_obt);

        global_path_subscriber_ = node->create_subscription<nav_msgs::msg::Path>(
            GET_TOPIC_NAME(DISPLAY_GLOBAL_PATH), 20,
            std::bind(&rclcomm::path_callback, this, std::placeholders::_1), sub1_obt);

        local_path_subscriber_ = node->create_subscription<nav_msgs::msg::Path>(
            GET_TOPIC_NAME(DISPLAY_LOCAL_PATH), 20,
            std::bind(&rclcomm::local_path_callback, this, std::placeholders::_1), sub1_obt);

        odometry_subscriber_ = node->create_subscription<nav_msgs::msg::Odometry>(
            GET_TOPIC_NAME(DISPLAY_ROBOT), 20,
            std::bind(&rclcomm::odom_callback, this, std::placeholders::_1), sub1_obt);

        robot_footprint_subscriber_ = node->create_subscription<geometry_msgs::msg::PolygonStamped>(
            GET_TOPIC_NAME(DISPLAY_ROBOT_FOOTPRINT), 20,
            std::bind(&rclcomm::robotFootprintCallback, this, std::placeholders::_1), sub1_obt);
            
        topology_map_subscriber_ = node->create_subscription<topology_msgs::msg::TopologyMap>(
            GET_TOPIC_NAME(DISPLAY_TOPOLOGY_MAP), rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local(),
            std::bind(&rclcomm::topologyMapCallback, this, std::placeholders::_1), sub1_obt);
            
        topology_map_update_publisher_ = node->create_publisher<topology_msgs::msg::TopologyMap>(
            GET_TOPIC_NAME(MSG_ID_TOPOLOGY_MAP_UPDATE), rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local());

    } catch (const std::exception& e) {
        LOG_ERROR("Failed to create subscriptions: " << e.what());
    }

    for (auto one_image_display : Config::ConfigManager::Instance()->GetRootConfig().images) {
      if (one_image_display.topic.empty()) continue;
      try {
        image_subscriber_list_.emplace_back(
            node->create_subscription<sensor_msgs::msg::Image>(
                one_image_display.topic, 1, [this, one_image_display](const sensor_msgs::msg::Image::SharedPtr msg) {
                    if (!msg) return;
                    cv::Mat conversion_mat_;
                    try {
                        cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::RGB8);
                        if (cv_ptr) conversion_mat_ = cv_ptr->image;
                    } catch (...) {
                        try {
                            cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg);
                            if (cv_ptr) {
                                if (msg->encoding == "CV_8UC3") conversion_mat_ = cv_ptr->image;
                                else if (msg->encoding == "8UC1") cv::cvtColor(cv_ptr->image, conversion_mat_, CV_GRAY2RGB);
                            }
                        } catch (...) { return; }
                    }
                    if (!conversion_mat_.empty()) {
                        auto image_payload = std::make_pair(one_image_display.location, std::make_shared<cv::Mat>(conversion_mat_));
                        PUBLISH(MSG_ID_IMAGE, image_payload);
                    }
                }));
      } catch (...) {}
    }

    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(node->get_clock(), std::chrono::seconds(10));
    // Pass node to TransformListener to ensure correct initialization in ROS2 Humble
    transform_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_, node, false);
    
  } catch (const std::exception& e) {
    LOG_ERROR("ROS2 initialization failed: " << e.what());
    return false;
  }

  SUBSCRIBE(MSG_ID_SET_NAV_GOAL_POSE, [this](const basic::RobotPose& pose) { PubNavGoal(pose); });
  SUBSCRIBE(MSG_ID_SET_RELOC_POSE, [this](const basic::RobotPose& pose) { PubRelocPose(pose); });
  SUBSCRIBE(MSG_ID_SET_ROBOT_SPEED, [this](const basic::RobotSpeed& speed) { PubRobotSpeed(speed); });
  SUBSCRIBE(MSG_ID_TOPOLOGY_MAP_UPDATE, [this](const TopologyMap& topology_map) {
    topology_msgs::msg::TopologyMap ros_msg = ConvertToRosMsg(topology_map);
    if (topology_map_update_publisher_) topology_map_update_publisher_->publish(ros_msg);
  });

  init_flag_ = true;
  // Start background spin thread to ensure timely processing of TF and callbacks
  std::thread([this]() {
      try {
          m_executor->spin();
      } catch (...) {}
  }).detach();
  
  return true;
}

bool rclcomm::Stop() {
  std::lock_guard<std::mutex> lock(executor_mutex_);
  if (init_flag_) {
    init_flag_ = false;
    map_subscriber_.reset();
    local_cost_map_subscriber_.reset();
    global_cost_map_subscriber_.reset();
    laser_scan_subscriber_.reset();
    battery_state_subscriber_.reset();
    global_path_subscriber_.reset();
    local_path_subscriber_.reset();
    odometry_subscriber_.reset();
    robot_footprint_subscriber_.reset();
    topology_map_subscriber_.reset();
    image_subscriber_list_.clear();
    nav_goal_publisher_.reset();
    reloc_pose_publisher_.reset();
    speed_publisher_.reset();
    topology_map_update_publisher_.reset();
    transform_listener_.reset();
    tf_buffer_.reset();
    if (m_executor && node) m_executor->remove_node(node);
    node.reset();
    m_executor.reset();
    if (rclcpp::ok()) rclcpp::shutdown();
  }
  return true;
}

void rclcomm::BatteryCallback(const sensor_msgs::msg::BatteryState::SharedPtr msg) {
  std::map<std::string, std::string> map;
  map["percent"] = std::to_string(msg->percentage);
  map["voltage"] = std::to_string(msg->voltage);
  PUBLISH(MSG_ID_BATTERY_STATE, map);
}

void rclcomm::getRobotPose() {
  std::string base_frame = Config::ConfigManager::Instance()->GetConfigValue("BaseFrameId", "base_link");
  auto pose = getTransform(base_frame, "map");
  PUBLISH(MSG_ID_ROBOT_POSE, pose);
}

basic::RobotPose rclcomm::getTransform(std::string from, std::string to) {
  basic::RobotPose ret;
  try {
    if (!tf_buffer_ || !tf_buffer_->canTransform(to, from, tf2::TimePointZero, std::chrono::milliseconds(500))) return ret;
    geometry_msgs::msg::TransformStamped transform = tf_buffer_->lookupTransform(to, from, tf2::TimePointZero, std::chrono::milliseconds(500));
    tf2::Quaternion q;
    tf2::fromMsg(transform.transform.rotation, q);
    double roll, pitch, yaw;
    tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);
    ret.x = transform.transform.translation.x;
    ret.y = transform.transform.translation.y;
    ret.theta = yaw;
  } catch (...) {}
  return ret;
}

void rclcomm::odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg) {
  basic::RobotState state;
  state.vx = msg->twist.twist.linear.x;
  state.vy = msg->twist.twist.linear.y;
  state.w = msg->twist.twist.angular.z;
  state.x = msg->pose.pose.position.x;
  state.y = msg->pose.pose.position.y;
  tf2::Quaternion q;
  tf2::fromMsg(msg->pose.pose.orientation, q);
  double roll, pitch, yaw;
  tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);
  state.theta = yaw;
  PUBLISH(MSG_ID_ODOM_POSE, state);
}

void rclcomm::local_path_callback(const nav_msgs::msg::Path::SharedPtr msg) {
  try {
    if (!tf_buffer_ || !tf_buffer_->canTransform("map", msg->header.frame_id, tf2::TimePointZero, std::chrono::milliseconds(100))) return;
    basic::RobotPath path;
    for (const auto& p : msg->poses) {
      geometry_msgs::msg::PointStamped pt_in, pt_out;
      pt_in.header = msg->header;
      pt_in.point = p.pose.position;
      tf_buffer_->transform(pt_in, pt_out, "map", std::chrono::milliseconds(100));
      path.push_back({pt_out.point.x, pt_out.point.y});
    }
    PUBLISH(MSG_ID_LOCAL_PATH, path);
  } catch (...) {}
}

void rclcomm::Process() {
  std::lock_guard<std::mutex> lock(executor_mutex_);
  if (init_flag_ && rclcpp::ok()) getRobotPose();
}

void rclcomm::path_callback(const nav_msgs::msg::Path::SharedPtr msg) {
  try {
    if (!tf_buffer_ || !tf_buffer_->canTransform("map", msg->header.frame_id, tf2::TimePointZero, std::chrono::milliseconds(100))) return;
    basic::RobotPath path;
    for (const auto& p : msg->poses) {
      geometry_msgs::msg::PointStamped pt_in, pt_out;
      pt_in.header = msg->header;
      pt_in.point = p.pose.position;
      tf_buffer_->transform(pt_in, pt_out, "map", std::chrono::milliseconds(100));
      path.push_back({pt_out.point.x, pt_out.point.y});
    }
    PUBLISH(MSG_ID_GLOBAL_PATH, path);
  } catch (...) {}
}

void rclcomm::laser_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
  try {
    if (!tf_buffer_) return;
    basic::LaserScan laser_points;
    std::string base_frame = Config::ConfigManager::Instance()->GetConfigValue("BaseFrameId", "base_link");
    for (size_t i = 0; i < msg->ranges.size(); ++i) {
      float dist = msg->ranges[i];
      if (std::isinf(dist) || std::isnan(dist)) continue;
      double angle = msg->angle_min + i * msg->angle_increment;
      geometry_msgs::msg::PointStamped pt_in, pt_out;
      pt_in.header = msg->header;
      pt_in.point.x = dist * cos(angle);
      pt_in.point.y = dist * sin(angle);
      tf_buffer_->transform(pt_in, pt_out, base_frame);
      laser_points.push_back({pt_out.point.x, pt_out.point.y});
    }
    laser_points.id = 0;
    PUBLISH(MSG_ID_LASER_SCAN, laser_points);
  } catch (...) {}
}

void rclcomm::robotFootprintCallback(const geometry_msgs::msg::PolygonStamped::SharedPtr msg) {
  basic::RobotPath footprint;
  for (auto point : msg->polygon.points) footprint.push_back({point.x, point.y});
  PUBLISH(MSG_ID_ROBOT_FOOTPRINT, footprint);
}

void rclcomm::topologyMapCallback(const topology_msgs::msg::TopologyMap::SharedPtr msg) {
  PUBLISH(MSG_ID_TOPOLOGY_MAP, ConvertFromRosMsg(msg));
}

void rclcomm::PubRelocPose(const basic::RobotPose &pose) {
  if (!reloc_pose_publisher_) return;
  geometry_msgs::msg::PoseWithCovarianceStamped msg;
  msg.header.frame_id = "map";
  msg.header.stamp = node->get_clock()->now();
  msg.pose.pose.position.x = pose.x;
  msg.pose.pose.position.y = pose.y;
  tf2::Quaternion q;
  q.setRPY(0, 0, pose.theta);
  msg.pose.pose.orientation = tf2::toMsg(q);
  reloc_pose_publisher_->publish(msg);
}

void rclcomm::PubNavGoal(const basic::RobotPose &pose) {
  if (!nav_goal_publisher_) return;
  geometry_msgs::msg::PoseStamped msg;
  msg.header.frame_id = "map";
  msg.header.stamp = node->get_clock()->now();
  msg.pose.position.x = pose.x;
  msg.pose.position.y = pose.y;
  tf2::Quaternion q;
  q.setRPY(0, 0, pose.theta);
  msg.pose.orientation = tf2::toMsg(q);
  nav_goal_publisher_->publish(msg);
}

void rclcomm::PubRobotSpeed(const basic::RobotSpeed &speed) {
  if (!speed_publisher_) return;
  geometry_msgs::msg::Twist msg;
  msg.linear.x = speed.vx;
  msg.linear.y = speed.vy;
  msg.angular.z = speed.w;
  speed_publisher_->publish(msg);
}

TopologyMap rclcomm::ConvertFromRosMsg(const topology_msgs::msg::TopologyMap::SharedPtr msg) {
  TopologyMap map;
  map.map_name = msg->map_name;
  for (const auto& c : msg->map_property.support_controllers) map.map_property.support_controllers.push_back(c);
  for (const auto& g : msg->map_property.support_goal_checkers) map.map_property.support_goal_checkers.push_back(g);
  for (const auto& p : msg->points) {
      TopologyMap::PointInfo pi;
      pi.name = p.name; pi.x = p.x; pi.y = p.y; pi.theta = p.theta; pi.type = (PointType)p.type;
      map.points.push_back(pi);
  }
  for (const auto& r : msg->routes) {
      TopologyMap::RouteInfo ri;
      ri.controller = r.route_info.controller; ri.speed_limit = r.route_info.speed_limit; ri.goal_checker = r.route_info.goal_checker;
      map.routes[r.from_point][r.to_point] = ri;
  }
  return map;
}

topology_msgs::msg::TopologyMap rclcomm::ConvertToRosMsg(const TopologyMap& topology_map) {
  topology_msgs::msg::TopologyMap msg;
  msg.map_name = topology_map.map_name;
  for (const auto& c : topology_map.map_property.support_controllers) msg.map_property.support_controllers.push_back(c);
  for (const auto& g : topology_map.map_property.support_goal_checkers) msg.map_property.support_goal_checkers.push_back(g);
  for (const auto& p : topology_map.points) {
      topology_msgs::msg::TopologyMapPointInfo pt;
      pt.name = p.name; pt.x = p.x; pt.y = p.y; pt.theta = p.theta; pt.type = (int)p.type;
      msg.points.push_back(pt);
  }
  for (auto const& [from, to_map] : topology_map.routes) {
    for (auto const& [to, info] : to_map) {
      topology_msgs::msg::RouteConnection route;
      route.from_point = from; route.to_point = to;
      route.route_info.controller = info.controller;
      route.route_info.speed_limit = info.speed_limit;
      route.route_info.goal_checker = info.goal_checker;
      msg.routes.push_back(route);
    }
  }
  return msg;
}

void rclcomm::localCostMapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg) {
  basic::OccupancyMap cost_map(msg->info.height, msg->info.width, Eigen::Vector3d(msg->info.origin.position.x, msg->info.origin.position.y, 0), msg->info.resolution);
  for (size_t i = 0; i < msg->data.size(); ++i) cost_map(i / msg->info.width, i % msg->info.width) = msg->data[i];
  cost_map.SetFlip();
  PUBLISH(MSG_ID_LOCAL_COST_MAP, cost_map);
}

void rclcomm::globalCostMapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg) {
  basic::OccupancyMap cost_map(msg->info.height, msg->info.width, Eigen::Vector3d(msg->info.origin.position.x, msg->info.origin.position.y, 0), msg->info.resolution);
  for (size_t i = 0; i < msg->data.size(); ++i) cost_map(i / msg->info.width, i % msg->info.width) = msg->data[i];
  cost_map.SetFlip();
  PUBLISH(MSG_ID_GLOBAL_COST_MAP, cost_map);
}

void rclcomm::map_callback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg) {
  basic::OccupancyMap new_map(msg->info.height, msg->info.width, Eigen::Vector3d(msg->info.origin.position.x, msg->info.origin.position.y, 0), msg->info.resolution);
  for (size_t i = 0; i < msg->data.size(); ++i) new_map(i / msg->info.width, i % msg->info.width) = msg->data.at(i);
  new_map.SetFlip();
  occ_map_ = new_map;
  PUBLISH(MSG_ID_OCCUPANCY_MAP, new_map);
}
