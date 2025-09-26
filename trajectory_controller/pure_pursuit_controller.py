#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Path, Odometry
from geometry_msgs.msg import Twist
import math
import numpy as np


class PurePursuitController(Node):
    def __init__(self):
        super().__init__('pure_pursuit_controller')
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.traj_sub = self.create_subscription(Path, '/trajectory', self.set_trajectory, 10)
        self.odom_sub = self.create_subscription(Odometry, '/odom', self.update_pose, 10)

        self.trajectory = None
        self.pose = [float('nan')] * 3  # Start with no pose
        self.lookahead = 0.6
        self.max_lin_vel = 0.4
        self.max_ang_vel = 1.0
        self.goal_tolerance = 0.8

        self.target_idx = 0
        self.goal_reached = False

        self.timer = self.create_timer(0.1, self.control_loop)
        self.get_logger().info('Pure Pursuit Controller started')

    def set_trajectory(self, msg):
        self.trajectory = msg
        self.target_idx = 0
        self.goal_reached = False
        self.get_logger().info(f'Received trajectory with {len(msg.poses)} points')

    def update_pose(self, msg):
        self.pose[0] = msg.pose.pose.position.x
        self.pose[1] = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        self.pose[2] = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))

    def control_loop(self):
        # wait for pose to be valid
        if any(math.isnan(val) for val in self.pose):
            return
        
        if not self.trajectory or len(self.trajectory.poses) == 0:
            self.stop()
            return
        
        if self.goal_reached:
            self.stop()
            return

        curr_pos = np.array(self.pose[:2])

        # The goal is set as the midpoint of last quarter of trajectory to reduce oscillations
        last_quarter_index = len(self.trajectory.poses) - len(self.trajectory.poses) // 4
        goal_point = self.trajectory.poses[last_quarter_index].pose.position
        goal_pos = np.array([goal_point.x, goal_point.y])

        dist_to_goal = np.linalg.norm(goal_pos - curr_pos)
        if dist_to_goal < self.goal_tolerance:
            self.get_logger().info('Goal reached, stopping robot.')
            self.goal_reached = True
            self.stop()
            return

        target_point = self.find_target_point()
        if target_point is None:
            self.stop()
            return
        
        lin_vel, ang_vel = self.calculate_control(target_point)
        self.publish_cmd(lin_vel, ang_vel)

    def find_target_point(self):
        curr_pos = np.array(self.pose[:2])
        for i in range(self.target_idx, len(self.trajectory.poses)):
            pt = self.trajectory.poses[i].pose.position
            pt_arr = np.array([pt.x, pt.y])
            if np.linalg.norm(pt_arr - curr_pos) >= self.lookahead:
                self.target_idx = i
                return pt_arr
        # fallback to last point
        if len(self.trajectory.poses) > 0:
            pt = self.trajectory.poses[-1].pose.position
            return np.array([pt.x, pt.y])
        return None

    def calculate_control(self, target_point):
        dx = target_point[0] - self.pose[0]
        dy = target_point[1] - self.pose[1]
        target_yaw = math.atan2(dy, dx)
        angle_diff = target_yaw - self.pose[2]

        while angle_diff > math.pi:
            angle_diff -= 2 * math.pi
        while angle_diff < -math.pi:
            angle_diff += 2 * math.pi

        distance = np.linalg.norm(target_point - self.pose[:2])
        curvature = 0
        if distance > 0:
            curvature = 2 * math.sin(angle_diff) / distance

        lin_vel = self.max_lin_vel * (1 - abs(angle_diff) / math.pi)
        lin_vel = max(0.1, min(lin_vel, self.max_lin_vel))
        ang_vel = curvature * lin_vel
        ang_vel = max(-self.max_ang_vel, min(ang_vel, self.max_ang_vel))

        if distance < 1.0:
            lin_vel *= 0.5

        return lin_vel, ang_vel

    def stop(self):
        cmd = Twist()
        cmd.linear.x = 0.0
        cmd.angular.z = 0.0
        self.cmd_pub.publish(cmd)

    def publish_cmd(self, lin_vel, ang_vel):
        cmd = Twist()
        cmd.linear.x = lin_vel
        cmd.angular.z = ang_vel
        self.cmd_pub.publish(cmd)

def main(args=None):
    rclpy.init(args=args)
    node = PurePursuitController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
