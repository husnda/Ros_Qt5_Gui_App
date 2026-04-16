/*
 * @Author: chengyang chengyangkj@outlook.com
 * @Date: 2023-07-27 14:47:24
 * @LastEditors: chengyangkj chengyangkj@qq.com
 * @LastEditTime: 2023-10-15 09:14:17
 * @FilePath: /ros_qt5_gui_app/src/channel/ros1/rosnode.cpp
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

    // 使用唯一节点名防止冲突
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

    // 核心话题订阅
    auto create_safe_sub = [&](const std::string& name, auto& sub, auto func, const rclcpp::QoS& qos, const rclcpp::SubscriptionOptions& obt) {
        try {
            sub = node->create_subscription<typename std::remove_reference_t<decltype(sub)>::element_type::MessageT>(
                GET_TOPIC_NAME(name), qos, std::bind(func, this, std::placeholders::_1), obt);
            LOG_INFO("Successfully subscribed to " << name);
        } catch (const std::exception& e) {
            LOG_ERROR("Failed to subscribe to " << name << ": " << e.what());
        }
    };

    create_safe_sub(DISPLAY_MAP, map_subscriber_, &rclcomm::map_callback, rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local(), sub1_obt);
    create_safe_sub(DISPLAY_LOCAL_COST_MAP, local_cost_map_subscriber_, &rclcomm::localCostMapCallback, rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local(), sub1_obt);
    create_safe_sub(DISPLAY_GLOBAL_COST_MAP, global_cost_map_subscriber_, &rclcomm::globalCostMapCallback, rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local(), sub1_obt);
    create_safe_sub(DISPLAY_LASER, laser_scan_subscriber_, &rclcomm::laser_callback, rclcpp::QoS(20), sub_laser_obt);
    create_safe_sub(MSG_ID_BATTERY_STATE, battery_state_subscriber_, &rclcomm::BatteryCallback, rclcpp::QoS(1), sub1_obt);
    create_safe_sub(DISPLAY_GLOBAL_PATH, global_path_subscriber_, &rclcomm::path_callback, rclcpp::QoS(20), sub1_obt);
    create_safe_sub(DISPLAY_LOCAL_PATH, local_path_subscriber_, &rclcomm::local_path_callback, rclcpp::QoS(20), sub1_obt);
    create_safe_sub(DISPLAY_ROBOT, odometry_subscriber_, &rclcomm::odom_callback, rclcpp::QoS(20), sub1_obt);
    create_safe_sub(DISPLAY_ROBOT_FOOTPRINT, robot_footprint_subscriber_, &rclcomm::robotFootprintCallback, rclcpp::QoS(20), sub1_obt);
    
    try {
        topology_map_subscriber_ = node->create_subscription<topology_msgs::msg::TopologyMap>(
            GET_TOPIC_NAME(DISPLAY_TOPOLOGY_MAP), 
            rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local(),
            std::bind(&rclcomm::topologyMapCallback, this, std::placeholders::_1),
            sub1_obt);
        LOG_INFO("Successfully subscribed to topology map");
    } catch (const std::exception& e) {
        LOG_ERROR("Failed to subscribe to topology map: " << e.what());
    }

    try {
        topology_map_update_publisher_ = node->create_publisher<topology_msgs::msg::TopologyMap>(
            GET_TOPIC_NAME(MSG_ID_TOPOLOGY_MAP_UPDATE), 
            rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local());
    } catch (const std::exception& e) {
        LOG_ERROR("Failed to create topology map publisher: " << e.what());
    }
        
    for (auto one_image_display : Config::ConfigManager::Instance()->GetRootConfig().images) {
      if (one_image_display.topic.empty()) continue;
      LOG_INFO("image location:" << one_image_display.location << "topic:" << one_image_display.topic);
      try {
        image_subscriber_list_.emplace_back(
            node->create_subscription<sensor_msgs::msg::Image>(
                one_image_display.topic, 1, [this, one_image_display](const sensor_msgs::msg::Image::SharedPtr msg) {
                    if (!msg) return;
                    cv::Mat conversion_mat_;
                    try {
                    // 必须使用 toCvCopy 进行深拷贝，防止原始消息被销毁导致内存越界
                    cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(
                        msg, sensor_msgs::image_encodings::RGB8);
                    if (cv_ptr) {
                        conversion_mat_ = cv_ptr->image;
                    }
                    } catch (cv_bridge::Exception &e) {
                    try {
                        cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg);
                        if (cv_ptr) {
                            if (msg->encoding == "CV_8UC3") {
                            conversion_mat_ = cv_ptr->image;
                            } else if (msg->encoding == "8UC1") {
                            cv::cvtColor(cv_ptr->image, conversion_mat_, CV_GRAY2RGB);
                            } else if (msg->encoding == "16UC1" || msg->encoding == "32FC1") {
                            double min = 0;
                            double max = 10;
                            if (msg->encoding == "16UC1") max *= 1000;
                            cv::Mat img_scaled_8u;
                            cv::Mat(cv_ptr->image - min).convertTo(img_scaled_8u, CV_8UC1, 255. / (max - min));
                            cv::cvtColor(img_scaled_8u, conversion_mat_, CV_GRAY2RGB);
                            }
                        }
                    } catch (...) { return; }
                    }
                    if (!conversion_mat_.empty()) {
                        auto image_payload = std::make_pair(one_image_display.location, std::make_shared<cv::Mat>(conversion_mat_));
                        PUBLISH(MSG_ID_IMAGE, image_payload);
                    }
                }));
      } catch (const std::exception& e) {
        LOG_ERROR("Failed to create image subscription for " << one_image_display.topic << ": " << e.what());
      }
    }

    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(node->get_clock(), std::chrono::seconds(10));
    transform_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
    
  } catch (const std::exception& e) {
    LOG_ERROR("ROS2 Node/Sub initialization failed: " << e.what());
    return false;
  }

  SUBSCRIBE(MSG_ID_SET_NAV_GOAL_POSE, [this](const basic::RobotPose& pose) {
    PubNavGoal(pose);
  });
  SUBSCRIBE(MSG_ID_SET_RELOC_POSE, [this](const basic::RobotPose& pose) {
    PubRelocPose(pose);
  });
  SUBSCRIBE(MSG_ID_SET_ROBOT_SPEED, [this](const basic::RobotSpeed& speed) {
    PubRobotSpeed(speed);
  });
  SUBSCRIBE(MSG_ID_TOPOLOGY_MAP_UPDATE, [this](const TopologyMap& topology_map) {
    topology_msgs::msg::TopologyMap ros_msg = ConvertToRosMsg(topology_map);
    topology_map_update_publisher_->publish(ros_msg);
  });

  init_flag_ = true;
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
    
    if (m_executor && node) {
        m_executor->remove_node(node);
    }
    node.reset();
    m_executor.reset();
    
    if (rclcpp::ok()) {
        rclcpp::shutdown();
    }
  }
  return true;
}

void rclcomm::BatteryCallback(
    const sensor_msgs::msg::BatteryState::SharedPtr msg) {
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
/**
 * @description: 获取坐标变化
 * @param {string} from 要变换的坐标系
 * @param {string} to 基坐标系
 * @return {basic::RobotPose}from变换到to坐标系下，需要变换的坐标
 */
basic::RobotPose rclcomm::getTransform(std::string from, std::string to) {
  basic::RobotPose ret;
  try {
    if (!tf_buffer_ || !tf_buffer_->canTransform(to, from, tf2::TimePointZero, std::chrono::milliseconds(100))) {
      return ret;
    }
    geometry_msgs::msg::TransformStamped transform =
        tf_buffer_->lookupTransform(to, from, tf2::TimePointZero, std::chrono::milliseconds(100));
    geometry_msgs::msg::Quaternion msg_quat = transform.transform.rotation;
    // 转换类型
    tf2::Quaternion q;
    tf2::fromMsg(msg_quat, q);
    tf2::Matrix3x3 mat(q);
    double roll, pitch, yaw;
    mat.getRPY(roll, pitch, yaw);
    // x y
    double x = transform.transform.translation.x;
    double y = transform.transform.translation.y;

    ret.x = x;
    ret.y = y;
    ret.theta = yaw;

  } catch (tf2::TransformException &ex) {
    // LOG_ERROR("getTransform error from:" << from << " to:" << to
    //                                      << " error:" << ex.what());
  }
  return ret;
}
void rclcomm::odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg) {
  basic::RobotState state;
  state.vx = (double)msg->twist.twist.linear.x;
  state.vy = (double)msg->twist.twist.linear.y;
  state.w = (double)msg->twist.twist.angular.z;
  state.x = (double)msg->pose.pose.position.x;
  state.y = (double)msg->pose.pose.position.y;

  geometry_msgs::msg::Quaternion msg_quat = msg->pose.pose.orientation;
  // 转换类型
  tf2::Quaternion q;
  tf2::fromMsg(msg_quat, q);
  tf2::Matrix3x3 mat(q);
  double roll, pitch, yaw;
  mat.getRPY(roll, pitch, yaw);
  state.theta = yaw;
  PUBLISH(MSG_ID_ODOM_POSE, state);
}
void rclcomm::local_path_callback(const nav_msgs::msg::Path::SharedPtr msg) {
  try {
    if (!tf_buffer_ || !tf_buffer_->canTransform("map", msg->header.frame_id, tf2::TimePointZero, std::chrono::milliseconds(100))) {
      return;
    }
    geometry_msgs::msg::PointStamped point_map_frame;
    geometry_msgs::msg::PointStamped point_odom_frame;
    basic::RobotPath path;
    for (int i = 0; i < msg->poses.size(); i++) {
      double x = msg->poses.at(i).pose.position.x;
      double y = msg->poses.at(i).pose.position.y;
      point_odom_frame.point.x = x;
      point_odom_frame.point.y = y;
      point_odom_frame.header.frame_id = msg->header.frame_id;
      point_odom_frame.header.stamp = msg->header.stamp;
      tf_buffer_->transform(point_odom_frame, point_map_frame, "map", std::chrono::milliseconds(100));
      basic::Point point;
      point.x = point_map_frame.point.x;
      point.y = point_map_frame.point.y;
      path.push_back(point);
    }
    PUBLISH(MSG_ID_LOCAL_PATH, path);
  } catch (tf2::TransformException &ex) {
  }
}

/// @brief loop for rate
void rclcomm::Process() {
  std::lock_guard<std::mutex> lock(executor_mutex_);
  if (init_flag_ && rclcpp::ok() && m_executor) {
    m_executor->spin_some();
    getRobotPose();
  }
}

void rclcomm::path_callback(const nav_msgs::msg::Path::SharedPtr msg) {
  try {
    if (!tf_buffer_ || !tf_buffer_->canTransform("map", msg->header.frame_id, tf2::TimePointZero, std::chrono::milliseconds(100))) {
      return;
    }
    geometry_msgs::msg::PointStamped point_map_frame;
    geometry_msgs::msg::PointStamped point_odom_frame;
    basic::RobotPath path;
    for (int i = 0; i < msg->poses.size(); i++) {
      double x = msg->poses.at(i).pose.position.x;
      double y = msg->poses.at(i).pose.position.y;
      point_odom_frame.point.x = x;
      point_odom_frame.point.y = y;
      point_odom_frame.header.frame_id = msg->header.frame_id;
      point_odom_frame.header.stamp = msg->header.stamp;
      tf_buffer_->transform(point_odom_frame, point_map_frame, "map", std::chrono::milliseconds(100));
      basic::Point point;
      point.x = point_map_frame.point.x;
      point.y = point_map_frame.point.y;
      path.push_back(point);
    }
    PUBLISH(MSG_ID_GLOBAL_PATH, path);
  } catch (tf2::TransformException &ex) {
  }
}

void rclcomm::laser_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
  double angle_min = msg->angle_min;
  double angle_increment = msg->angle_increment;
  try {
    if (!tf_buffer_) return;
    geometry_msgs::msg::PointStamped point_base_frame;
    geometry_msgs::msg::PointStamped point_laser_frame;
    basic::LaserScan laser_points;
    for (int i = 0; i < msg->ranges.size(); i++) {
      double angle = angle_min + i * angle_increment;
      double dist = msg->ranges[i];
      if (std::isinf(dist))
        continue;
      double x = dist * cos(angle);
      double y = dist * sin(angle);
      point_laser_frame.point.x = x;
      point_laser_frame.point.y = y;
      point_laser_frame.header.frame_id = msg->header.frame_id;
      std::string base_frame = Config::ConfigManager::Instance()->GetConfigValue("BaseFrameId", "base_link");
      tf_buffer_->transform(point_laser_frame, point_base_frame, base_frame);
      basic::Point p;
      p.x = point_base_frame.point.x;
      p.y = point_base_frame.point.y;
      laser_points.push_back(p);
    }
    laser_points.id = 0;
    PUBLISH(MSG_ID_LASER_SCAN, laser_points);
  } catch (tf2::TransformException &ex) {}
}
void rclcomm::robotFootprintCallback(const geometry_msgs::msg::PolygonStamped::SharedPtr msg) {
  basic::RobotPath footprint;
  for (auto point : msg->polygon.points) {
    basic::Point p;
    p.x = point.x;
    p.y = point.y;
    footprint.push_back(p);
  }
  PUBLISH(MSG_ID_ROBOT_FOOTPRINT, footprint);
}
void rclcomm::topologyMapCallback(const topology_msgs::msg::TopologyMap::SharedPtr msg) {
  PUBLISH(MSG_ID_TOPOLOGY_MAP, ConvertFromRosMsg(msg));
}
void rclcomm::PubRelocPose(const basic::RobotPose &pose) {
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
  geometry_msgs::msg::Twist msg;
  msg.linear.x = speed.vx;
  msg.linear.y = speed.vy;
  msg.angular.z = speed.w;
  speed_publisher_->publish(msg);
}
TopologyMap rclcomm::ConvertFromRosMsg(const topology_msgs::msg::TopologyMap::SharedPtr msg) {
  TopologyMap map;
  map.map_name = msg->map_name;
  for (auto controller : msg->map_property.support_controllers) {
    map.map_property.support_controllers.push_back(controller);
  }
  for (auto goal_checker : msg->map_property.support_goal_checkers) {
    map.map_property.support_goal_checkers.push_back(goal_checker);
  }
  for (auto point : msg->points) {
    TopologyMap::PointInfo p;
    p.name = point.name;
    p.x = point.x;
    p.y = point.y;
    p.theta = point.theta;
    p.type = (PointType)point.type;
    map.points.push_back(p);
  }
  for (auto route : msg->routes) {
    TopologyMap::RouteInfo r;
    r.controller = route.route_info.controller;
    r.speed_limit = route.route_info.speed_limit;
    r.goal_checker = route.route_info.goal_checker;
    map.routes[route.from_point][route.to_point] = r;
  }
  return map;
}
topology_msgs::msg::TopologyMap rclcomm::ConvertToRosMsg(const TopologyMap& topology_map) {
  topology_msgs::msg::TopologyMap msg;
  msg.map_name = topology_map.map_name;
  for (auto controller : topology_map.map_property.support_controllers) {
    msg.map_property.support_controllers.push_back(controller);
  }
  for (auto goal_checker : topology_map.map_property.support_goal_checkers) {
    msg.map_property.support_goal_checkers.push_back(goal_checker);
  }
  for (auto point : topology_map.points) {
    topology_msgs::msg::Point point_msg;
    point_msg.name = point.name;
    point_msg.x = point.x;
    point_msg.y = point.y;
    point_msg.theta = point.theta;
    point_msg.type = point.type;
    msg.points.push_back(point_msg);
  }
  for (auto const& [from, to_map] : topology_map.routes) {
    for (auto const& [to, info] : to_map) {
      topology_msgs::msg::Route route_msg;
      route_msg.from_point = from;
      route_msg.to_point = to;
      route_msg.route_info.controller = info.controller;
      route_msg.route_info.speed_limit = info.speed_limit;
      route_msg.route_info.goal_checker = info.goal_checker;
      msg.routes.push_back(route_msg);
    }
  }
  return msg;
}
void rclcomm::localCostMapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg) {
  basic::OccupancyMap cost_map(msg->info.height, msg->info.width, Eigen::Vector3d(msg->info.origin.position.x, msg->info.origin.position.y, 0), msg->info.resolution);
  for (int i = 0; i < msg->data.size(); i++) {
    int x = i / msg->info.width;
    int y = i % msg->info.width;
    cost_map(x, y) = msg->data[i];
  }
  cost_map.SetFlip();
  PUBLISH(MSG_ID_LOCAL_COST_MAP, cost_map);
}
void rclcomm::globalCostMapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg) {
  basic::OccupancyMap cost_map(msg->info.height, msg->info.width, Eigen::Vector3d(msg->info.origin.position.x, msg->info.origin.position.y, 0), msg->info.resolution);
  for (int i = 0; i < msg->data.size(); i++) {
    int x = i / msg->info.width;
    int y = i % msg->info.width;
    cost_map(x, y) = msg->data[i];
  }
  cost_map.SetFlip();
  PUBLISH(MSG_ID_GLOBAL_COST_MAP, cost_map);
}
void rclcomm::map_callback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg) {
  basic::OccupancyMap new_map(msg->info.height, msg->info.width, Eigen::Vector3d(msg->info.origin.position.x, msg->info.origin.position.y, 0), msg->info.resolution);
  for (int i = 0; i < msg->data.size(); i++) {
    int x = i / msg->info.width;
    int y = i % msg->info.width;
    new_map(x, y) = msg->data.at(i);
  }
  new_map.SetFlip();
  occ_map_ = new_map;
  PUBLISH(MSG_ID_OCCUPANCY_MAP, new_map);
}
