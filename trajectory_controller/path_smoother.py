#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
import numpy as np
from scipy.interpolate import CubicSpline

class PathSmoother(Node):
    def __init__(self):
        super().__init__('path_smoother')
        self.smooth_path_pub = self.create_publisher(Path, '/smooth_path', 10)
        self.waypoint_sub = self.create_subscription(Path, '/waypoints', self.smooth_path, 10)
        self.get_logger().info('Path Smoother Node started')

    def smooth_path(self, msg):
        if len(msg.poses) < 3:
            self.get_logger().warning('Need at least 3 waypoints for cubic spline smoothing')
            return
        try:
            waypoints = np.array([[p.pose.position.x, p.pose.position.y] for p in msg.poses])
            distances = [0]
            for i in range(1, len(waypoints)):
                distances.append(distances[-1] + np.linalg.norm(waypoints[i] - waypoints[i - 1]))
            distances = np.array(distances)
            cs_x = CubicSpline(distances, waypoints[:, 0])
            cs_y = CubicSpline(distances, waypoints[:, 1])
            smooth_dist = np.linspace(0, distances[-1], num=100)
            smooth_points = np.vstack((cs_x(smooth_dist), cs_y(smooth_dist))).T
        except Exception as e:
            self.get_logger().error(f'Error in smoothing path: {e}')
            return
        path_msg = Path()
        path_msg.header = msg.header
        path_msg.header.stamp = self.get_clock().now().to_msg()
        for p in smooth_points:
            pose = PoseStamped()
            pose.header = path_msg.header
            pose.pose.position.x = float(p[0])
            pose.pose.position.y = float(p[1])
            pose.pose.position.z = 0.0
            pose.pose.orientation.w = 1.0
            path_msg.poses.append(pose)
        self.smooth_path_pub.publish(path_msg)
        self.get_logger().info(f'Published smoothed path with {len(smooth_points)} points')

def main(args=None):
    rclpy.init(args=args)
    node = PathSmoother()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
