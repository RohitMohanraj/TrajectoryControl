#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
import math

class TestScenarios(Node):
    def __init__(self):
        super().__init__('test_scenarios')
        self.waypoint_pub = self.create_publisher(Path, '/waypoints', 10)
        self.published_once = False
        self.declare_parameter('scenario', 'straight_line')
        self.scenario = self.get_parameter('scenario').value
        self.scenarios = {
            'straight_line': self.generate_straight_line,
            'zigzag': self.generate_zigzag,
            's_curve': self.generate_s_curve,
        }
        self.timer = self.create_timer(2.0, self.publish_waypoints)
        self.get_logger().info(f'Started test_scenarios with scenario: {self.scenario}')
    
    def generate_straight_line(self):
        waypoints = []
        length = 5.0
        n_points = 20
        for i in range(n_points):
            x = length * i / (n_points - 1)
            y = 0.0
            waypoints.append([x, y])
        return waypoints
    
    def generate_zigzag(self):
        waypoints = []
        n_zigs = 5
        zig_length = 1.0
        for i in range(n_zigs * 2 + 1):
            x = i * zig_length
            y = zig_length if i % 2 == 0 else -zig_length
            waypoints.append([x, y])
        return waypoints
    
    def generate_s_curve(self):
        waypoints = []
        for x in range(21):
            y = 0.5 * math.sin(0.25 * x)
            waypoints.append([x * 0.3, y])
        return waypoints
    
    def publish_waypoints(self):
        if self.published_once:
            return
        if self.scenario not in self.scenarios:
            self.get_logger().error(f'Unknown scenario: {self.scenario}')
            return
        try:
            waypoints = self.scenarios[self.scenario]()
        except Exception as e:
            self.get_logger().error(f'Failed to generate waypoints: {e}')
            return
        
        path_msg = Path()
        path_msg.header.stamp = self.get_clock().now().to_msg()
        path_msg.header.frame_id = 'odom'
        for wp in waypoints:
            pose = PoseStamped()
            pose.header = path_msg.header
            pose.pose.position.x = float(wp[0])
            pose.pose.position.y = float(wp[1])
            pose.pose.position.z = 0.0
            pose.pose.orientation.w = 1.0
            path_msg.poses.append(pose)
        
        self.waypoint_pub.publish(path_msg)
        self.get_logger().info(f'Published {len(waypoints)} waypoints for {self.scenario}')
        self.published_once = True

def main(args=None):
    rclpy.init(args=args)
    node = TestScenarios()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__=='__main__':
    main()
