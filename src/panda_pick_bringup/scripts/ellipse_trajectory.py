#!/usr/bin/env python3
"""
Move the FR3 end-effector through a 3D ellipse path using MoveIt2's Cartesian path service.

Ellipse defaults:
  center  = (0.4, 0.0, 0.5)  in fr3_link0 frame
  radii   = (0.15, 0.10) m   (X-axis, Y-axis)
  plane   = horizontal (XY-plane at the given Z)
  EE      = pointing straight down
"""
import math
import sys

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient

from geometry_msgs.msg import Pose, PoseStamped
from moveit_msgs.action import ExecuteTrajectory
from moveit_msgs.msg import RobotState
from moveit_msgs.srv import GetCartesianPath, GetPositionIK
from sensor_msgs.msg import JointState


CENTER = (0.4, 0.0, 0.5)
RADII = (0.15, 0.10)
NUM_WAYPOINTS = 60
EEF_LINK = 'fr3_hand_tcp'
GROUP_NAME = 'fr3_arm'
PLANNING_FRAME = 'fr3_link0'


def make_ee_down_pose(x, y, z):
    """EE pose at (x,y,z) with the tool pointing straight down (-Z in base frame).
       For fr3_hand_tcp: identity-ish orientation pointing down is quaternion
       (x=1, y=0, z=0, w=0) — a 180° rotation about X flips Z to -Z."""
    p = Pose()
    p.position.x = x
    p.position.y = y
    p.position.z = z
    p.orientation.x = 1.0
    p.orientation.y = 0.0
    p.orientation.z = 0.0
    p.orientation.w = 0.0
    return p


def ellipse_waypoints(center, radii, n):
    """Generate n Pose() waypoints along a horizontal ellipse."""
    cx, cy, cz = center
    rx, ry = radii
    poses = []
    for i in range(n + 1):
        t = 2.0 * math.pi * i / n
        x = cx + rx * math.cos(t)
        y = cy + ry * math.sin(t)
        z = cz
        poses.append(make_ee_down_pose(x, y, z))
    return poses


class EllipseRunner(Node):
    def __init__(self):
        super().__init__('ellipse_runner')

        self.cart_client = self.create_client(
            GetCartesianPath, '/compute_cartesian_path'
        )
        self.exec_client = ActionClient(
            self, ExecuteTrajectory, '/execute_trajectory'
        )

        self.get_logger().info('Waiting for MoveIt services...')
        self.cart_client.wait_for_service()
        self.exec_client.wait_for_server()
        self.get_logger().info('MoveIt ready.')

        # Subscribe once to /joint_states to seed the current state
        self.current_joint_state = None
        self.create_subscription(JointState, '/joint_states',
                                 self._js_cb, 10)

    def _js_cb(self, msg):
        self.current_joint_state = msg

    def wait_for_joint_state(self):
        while rclpy.ok() and self.current_joint_state is None:
            rclpy.spin_once(self, timeout_sec=0.1)

    def run(self):
        self.wait_for_joint_state()

        waypoints = ellipse_waypoints(CENTER, RADII, NUM_WAYPOINTS)
        self.get_logger().info(f'Generated {len(waypoints)} ellipse waypoints')

        # Build the cartesian path request
        req = GetCartesianPath.Request()
        req.header.frame_id = PLANNING_FRAME
        req.header.stamp = self.get_clock().now().to_msg()
        req.group_name = GROUP_NAME
        req.link_name = EEF_LINK
        req.waypoints = waypoints
        req.max_step = 0.01            # 1cm interpolation step
        req.jump_threshold = 0.0       # disabled
        req.avoid_collisions = True

        # Use current robot state as start
        start_state = RobotState()
        start_state.joint_state = self.current_joint_state
        req.start_state = start_state

        self.get_logger().info('Computing Cartesian path...')
        future = self.cart_client.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        resp = future.result()

        if resp.fraction < 0.95:
            self.get_logger().error(
                f'Cartesian planning achieved only {resp.fraction*100:.1f}% of path'
            )
            return False

        self.get_logger().info(
            f'Cartesian path computed: {resp.fraction*100:.1f}% achieved, '
            f'{len(resp.solution.joint_trajectory.points)} trajectory points'
        )

        # Execute
        goal = ExecuteTrajectory.Goal()
        goal.trajectory = resp.solution

        self.get_logger().info('Executing trajectory...')
        send_future = self.exec_client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, send_future)
        goal_handle = send_future.result()

        if not goal_handle.accepted:
            self.get_logger().error('Trajectory goal rejected')
            return False

        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)
        self.get_logger().info('Done.')
        return True


def main():
    rclpy.init()
    node = EllipseRunner()
    try:
        node.run()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
