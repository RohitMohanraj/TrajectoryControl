#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped

class TrajectoryGenerator(Node):
    def __init__(self):
        super().__init__('trajectory_generator')
        self.trajectory_pub = self.create_publisher(Path, '/trajectory', 10)
        self.path_sub = self.create_subscription(Path, '/smooth_path', self.generate_trajectory, 10)
        self.get_logger().info('Trajectory Generator Node started')

    def generate_trajectory(self, msg):
        if len(msg.poses) < 1:
            return
        traj_msg = Path()
        traj_msg.header = msg.header
        t = 0.0
        dt = 0.1
        for pose in msg.poses:
            new_pose = PoseStamped()
            new_pose.header = msg.header
            new_pose.pose = pose.pose
            new_pose.header.stamp.sec = int(t)
            new_pose.header.stamp.nanosec = int((t - int(t)) * 1e9)
            traj_msg.poses.append(new_pose)
            t += dt
        self.trajectory_pub.publish(traj_msg)
        self.get_logger().info(f'Published trajectory with {len(traj_msg.poses)} points')

def main(args=None):
    rclpy.init(args=args)
    node = TrajectoryGenerator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
