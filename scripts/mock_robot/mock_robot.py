#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid, Odometry, Path
from sensor_msgs.msg import LaserScan, BatteryState, Image
from geometry_msgs.msg import Twist, PoseStamped, PoseWithCovarianceStamped, Point, Quaternion, TransformStamped
from std_msgs.msg import Header
from tf2_ros import TransformBroadcaster, StaticTransformBroadcaster
import numpy as np
import math
import time

class MockRobot(Node):
    def __init__(self):
        super().__init__('mock_robot')
        
        # Parameters
        self.map_size = 200
        self.resolution = 0.05
        
        # QoS for Map/Costmap (matches rclcomm.cpp: transient_local)
        map_qos = rclpy.qos.QoSProfile(
            depth=1,
            durability=rclpy.qos.DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=rclpy.qos.ReliabilityPolicy.RELIABLE
        )
        
        # Publishers
        self.map_pub = self.create_publisher(OccupancyGrid, '/map', qos_profile=map_qos)
        self.odom_pub = self.create_publisher(Odometry, '/odom', 10)
        self.scan_pub = self.create_publisher(LaserScan, '/scan', 10)
        self.battery_pub = self.create_publisher(BatteryState, '/battery', 10)
        self.image_pub = self.create_publisher(Image, '/camera/front/image_raw', 10)
        self.global_path_pub = self.create_publisher(Path, '/plan', qos_profile=map_qos)
        self.local_path_pub = self.create_publisher(Path, '/local_plan', qos_profile=map_qos)
        self.global_costmap_pub = self.create_publisher(OccupancyGrid, '/global_costmap/costmap', qos_profile=map_qos)
        self.local_costmap_pub = self.create_publisher(OccupancyGrid, '/local_costmap/costmap', qos_profile=map_qos)

        # TF Broadcasters
        self.tf_broadcaster = TransformBroadcaster(self)
        self.static_tf_broadcaster = StaticTransformBroadcaster(self)

        # Subscribers
        self.cmd_vel_sub = self.create_subscription(Twist, '/cmd_vel', self.cmd_vel_callback, 10)
        self.goal_sub = self.create_subscription(PoseStamped, '/goal_pose', self.goal_callback, 10)
        self.initial_pose_sub = self.create_subscription(PoseWithCovarianceStamped, '/initialpose', self.initial_pose_callback, 10)

        # Robot State
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.vx = 0.0
        self.vw = 0.0
        self.battery_percent = 1.0
        
        # Pre-generate map data
        self.map_data = np.zeros(self.map_size * self.map_size, dtype=np.int8)
        grid = self.map_data.reshape((self.map_size, self.map_size))
        grid[0, :] = 100
        grid[-1, :] = 100
        grid[:, 0] = 100
        grid[:, -1] = 100
        grid[40:60, 40:60] = 100
        grid[120:140, 80:100] = 100

        # Broadcast Static TF: map -> odom (REMOVED - Use dynamic only for better reliability in simulation)
        # self.publish_static_tf()

        # Timers
        self.create_timer(0.05, self.publish_odom_and_tf) # Increased to 20Hz
        self.create_timer(0.2, self.publish_scan)
        self.create_timer(1.0, self.publish_map) # Increased to 1Hz
        self.create_timer(2.0, self.publish_battery)
        self.create_timer(0.1, self.publish_image)
        self.create_timer(0.02, self.update_pose) # 50Hz pose update

        # Initial publications
        self.publish_map()

        self.get_logger().info("Mock Robot Started. map -> odom -> base_link tree active.")

    def safe_publish(self, publisher, msg):
        try:
            if rclpy.ok():
                publisher.publish(msg)
        except Exception:
            pass

    # publish_static_tf REMOVED

    def cmd_vel_callback(self, msg):
        self.vx = msg.linear.x
        self.vw = msg.angular.z

    def goal_callback(self, msg):
        self.get_logger().info(f"Received Nav Goal: {msg.pose.position.x}, {msg.pose.position.y}")
        path = Path()
        path.header.frame_id = 'map'
        path.header.stamp = self.get_clock().now().to_msg()
        steps = 50
        for i in range(steps):
            pose = PoseStamped()
            pose.header.frame_id = 'map'
            pose.pose.position.x = self.x + (msg.pose.position.x - self.x) * i / steps
            pose.pose.position.y = self.y + (msg.pose.position.y - self.y) * i / steps
            path.poses.append(pose)
        self.safe_publish(self.global_path_pub, path)
        self.safe_publish(self.local_path_pub, path)

    def initial_pose_callback(self, msg):
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
        self.theta = math.atan2(siny_cosp, cosy_cosp)
        self.get_logger().info(f"Relocalized to: {self.x}, {self.y}, theta: {self.theta}")

    def update_pose(self):
        dt = 0.02
        self.theta += self.vw * dt
        self.x += self.vx * math.cos(self.theta) * dt
        self.y += self.vx * math.sin(self.theta) * dt

    def publish_odom_and_tf(self):
        now = self.get_clock().now().to_msg()
        q = self.euler_to_quaternion(0, 0, self.theta)

        # 1. Publish Odom Message
        odom = Odometry()
        odom.header.stamp = now
        odom.header.frame_id = 'odom'
        odom.child_frame_id = 'base_link'
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.orientation.x = q[0]
        odom.pose.pose.orientation.y = q[1]
        odom.pose.pose.orientation.z = q[2]
        odom.pose.pose.orientation.w = q[3]
        odom.twist.twist.linear.x = self.vx
        odom.twist.twist.angular.z = self.vw
        self.safe_publish(self.odom_pub, odom)

        # 2. Broadcast TF: map -> odom -> base_link
        t_map_odom = TransformStamped()
        t_map_odom.header.stamp = now
        t_map_odom.header.frame_id = 'map'
        t_map_odom.child_frame_id = 'odom'
        t_map_odom.transform.translation.x = 0.0
        t_map_odom.transform.translation.y = 0.0
        t_map_odom.transform.translation.z = 0.0
        t_map_odom.transform.rotation.w = 1.0

        t_odom_base = TransformStamped()
        t_odom_base.header.stamp = now
        t_odom_base.header.frame_id = 'odom'
        t_odom_base.child_frame_id = 'base_link'
        t_odom_base.transform.translation.x = self.x
        t_odom_base.transform.translation.y = self.y
        t_odom_base.transform.translation.z = 0.0
        t_odom_base.transform.rotation.x = q[0]
        t_odom_base.transform.rotation.y = q[1]
        t_odom_base.transform.rotation.z = q[2]
        t_odom_base.transform.rotation.w = q[3]

        try:
            self.tf_broadcaster.sendTransform([t_map_odom, t_odom_base])
        except:
            pass

    def publish_map(self):
        msg = OccupancyGrid()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'map'
        msg.info.map_load_time = msg.header.stamp
        msg.info.resolution = self.resolution
        msg.info.width = self.map_size
        msg.info.height = self.map_size
        msg.info.origin.position.x = -float(self.map_size) * self.resolution / 2.0
        msg.info.origin.position.y = -float(self.map_size) * self.resolution / 2.0
        msg.info.origin.orientation.w = 1.0
        msg.data = self.map_data.tolist()
        self.safe_publish(self.map_pub, msg)
        self.safe_publish(self.global_costmap_pub, msg)
        self.safe_publish(self.local_costmap_pub, msg)

    def publish_scan(self):
        msg = LaserScan()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'base_link'
        msg.angle_min = -math.pi
        msg.angle_max = math.pi
        msg.angle_increment = math.pi / 180.0
        msg.time_increment = 0.0
        msg.scan_time = 0.1
        msg.range_min = 0.1
        msg.range_max = 10.0
        num_readings = 360
        msg.ranges = [3.0 + 0.1 * np.random.randn() for _ in range(num_readings)]
        self.safe_publish(self.scan_pub, msg)

    def publish_battery(self):
        msg = BatteryState()
        msg.header.stamp = self.get_clock().now().to_msg()
        self.battery_percent -= 0.0001
        if self.battery_percent < 0: self.battery_percent = 1.0
        msg.percentage = float(self.battery_percent)
        msg.voltage = 12.0 * self.battery_percent
        self.safe_publish(self.battery_pub, msg)

    def publish_image(self):
        msg = Image()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'camera_frame'
        msg.height = 480
        msg.width = 640
        msg.encoding = 'rgb8'
        msg.is_bigendian = 0
        msg.step = 640 * 3
        t = time.time()
        data = np.zeros((480, 640, 3), dtype=np.uint8)
        r = int(127 + 127 * math.sin(t))
        g = int(127 + 127 * math.sin(t + 2))
        b = int(127 + 127 * math.sin(t + 4))
        data[:, :, 0] = r
        data[:, :, 1] = g
        data[:, :, 2] = b
        msg.data = data.tobytes()
        self.safe_publish(self.image_pub, msg)

    def euler_to_quaternion(self, roll, pitch, yaw):
        qx = np.sin(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) - np.cos(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
        qy = np.cos(roll/2) * np.sin(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.cos(pitch/2) * np.sin(yaw/2)
        qz = np.cos(roll/2) * np.cos(pitch/2) * np.sin(yaw/2) - np.sin(roll/2) * np.sin(pitch/2) * np.cos(yaw/2)
        qw = np.cos(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
        return [float(qx), float(qy), float(qz), float(qw)]

def main(args=None):
    try:
        rclpy.init(args=args)
        node = MockRobot()
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    except Exception as e:
        print(f"Mock Robot Error: {e}")
    finally:
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    from rclpy.executors import ExternalShutdownException
    main()
