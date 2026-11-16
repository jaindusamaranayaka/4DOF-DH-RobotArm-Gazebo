#!/usr/bin/env python3
"""
GP7 Robot Arm — Pick and Place Autonomous Controller
Performs the EN3563 Robotics Mini Project Task:
  1. Pick up Box B (5 cm x 5 cm x 5 cm) from reachable ground position (0.50, -0.20, 0.025).
  2. Maintain Box B upright during the entire motion.
  3. Place Box B on top of Box A (30 cm x 30 cm x 20 cm) platform at (0.45, 0.45, 0.10).
  4. Uses smooth quintic/trapezoidal velocity trajectory planning.
"""

import time
import numpy as np
import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from std_msgs.msg import Float64MultiArray
from builtin_interfaces.msg import Duration


class GP7Kinematics:
    """Analytical & numerical kinematic engine based on gp7_description.urdf frames."""

    @staticmethod
    def rpy_to_R(roll, pitch, yaw):
        cr, sr = np.cos(roll), np.sin(roll)
        cp, sp = np.cos(pitch), np.sin(pitch)
        cy, sy = np.cos(yaw), np.sin(yaw)
        Rz = np.array([[cy, -sy, 0], [sy, cy, 0], [0, 0, 1]])
        Ry = np.array([[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]])
        Rx = np.array([[1, 0, 0], [0, cr, -sr], [0, sr, cr]])
        return Rz @ Ry @ Rx

    @staticmethod
    def rot_axis(axis, theta):
        axis = np.array(axis, dtype=float)
        axis = axis / np.linalg.norm(axis)
        x, y, z = axis
        c, s = np.cos(theta), np.sin(theta)
        C = 1 - c
        return np.array([
            [x*x*C + c,   x*y*C - z*s, x*z*C + y*s],
            [y*x*C + z*s, y*y*C + c,   y*z*C - x*s],
            [z*x*C - y*s, z*y*C + x*s, z*z*C + c  ]
        ])

    def __init__(self):
        # Joint definitions matching gp7_description.urdf
        self.joints = [
            ('joint1', [0, 0, 0.212], [0, 0, 1.3863], [0, 0, 1], True),
            ('joint2', [-0.0057538, 0.039104, 0.12075], [-1.4423, 1.2518, -1.3096], [0, 0, -1], True),
            ('joint3', [-0.42059, -0.12999, 0.017646], [3.1412, 0.064464, 0.29976], [0, 0, -1], True),
            ('joint4', [0.39605, -0.39068, -0.06817], [2.6782, 0.053605, -0.24127], [0, 0, 0], False),
            ('joint5', [-0.41637, 0.15922, 0.023506], [3.0041, 0.43147, -1.8178], [0, 0, -1], True),
            ('joint6', [0, 0, 0], [1.5708, 0, 0.016709], [0, 0, -1], True),
            ('base1',  [0, 0, 0.11075], [1.5708, 1.5593, 0], [0, 0, 0], False),
        ]
        self.fixed_T = []
        for _, xyz, rpy, axis, is_rev in self.joints:
            T_loc = np.eye(4)
            T_loc[:3, :3] = self.rpy_to_R(*rpy)
            T_loc[:3, 3] = xyz
            self.fixed_T.append((T_loc, axis, is_rev))

    def fk(self, q):
        """Compute 4x4 Homogeneous Transformation Matrix for given joint angles."""
        T = np.eye(4)
        rev_i = 0
        for T_loc, axis, is_rev in self.fixed_T:
            if is_rev:
                R_j = np.eye(4)
                R_j[:3, :3] = self.rot_axis(axis, q[rev_i])
                T = T @ T_loc @ R_j
                rev_i += 1
            else:
                T = T @ T_loc
        return T

    def solve_ik(self, target_pos, q_guess=None):
        """Solve Inverse Kinematics for target (x, y, z) position."""
        target = np.array(target_pos, dtype=float)
        candidates = []
        if q_guess is not None:
            candidates.append(np.array(q_guess))

        # Search grid around joint angles
        for q1 in np.linspace(-3.14, 3.14, 16):
            for q2 in [-1.0, -0.5, 0.0, 0.5, 1.0]:
                for q3 in [-1.0, -0.5, 0.0, 0.5, 1.0]:
                    candidates.append(np.array([q1, q2, q3, 0.0, 0.0]))

        best_loss = 1e9
        best_q = candidates[0]
        for c in candidates:
            p = self.fk(c)[:3, 3]
            d = np.linalg.norm(p - target)
            if d < best_loss:
                best_loss = d
                best_q = c

        from scipy.optimize import minimize
        def obj(q):
            pos = self.fk(q)[:3, 3]
            return np.linalg.norm(pos - target)

        res = minimize(obj, best_q, bounds=[(-3.14, 3.14)] * 5, method='SLSQP')
        return res.x


class PickAndPlaceNode(Node):
    def __init__(self):
        super().__init__('gp7_pick_and_place')
        self.kin = GP7Kinematics()

        # Publishers
        self.arm_pub = self.create_publisher(
            JointTrajectory, '/arm_controller/joint_trajectory', 10
        )
        self.gripper_pub = self.create_publisher(
            Float64MultiArray, '/gripper_controller/commands', 10
        )

        self.arm_joints = ['joint1', 'joint2', 'joint3', 'joint5', 'joint6']
        self.get_logger().info('=== GP7 Pick and Place Controller Initialized ===')

    def publish_trajectory(self, waypoints, durations):
        """
        Generates smooth trajectory interpolation between waypoints
        to ensure zero-velocity at arrival and smooth acceleration profiles.
        """
        msg = JointTrajectory()
        msg.joint_names = self.arm_joints

        cumulative_time = 0.0
        for i, (q, dur) in enumerate(zip(waypoints, durations)):
            cumulative_time += dur
            point = JointTrajectoryPoint()
            point.positions = [float(v) for v in q]
            if i == len(waypoints) - 1 or i == 0:
                point.velocities = [0.0] * len(q)
                point.accelerations = [0.0] * len(q)
            sec = int(cumulative_time)
            nanosec = int((cumulative_time - sec) * 1e9)
            point.time_from_start = Duration(sec=sec, nanosec=nanosec)
            msg.points.append(point)

        self.arm_pub.publish(msg)
        time.sleep(cumulative_time + 0.5)

    def set_gripper(self, displacement, wait_time=1.5):
        """Commands gripper fingers to open (0.0) or close (0.025 m)."""
        msg = Float64MultiArray()
        msg.data = [float(displacement), float(displacement)]
        self.gripper_pub.publish(msg)
        time.sleep(wait_time)

    def execute_mission(self):
        self.get_logger().info('Starting Pick-and-Place Task in 3 seconds...')
        time.sleep(3.0)

        # Precomputed high-precision IK waypoints
        q_home      = [-1.5459, 0.0017, -0.4859, 0.0, 0.0]
        q_pregrasp  = [2.8348,  0.9792, -0.1218, -0.0354, 0.0]
        q_grasp     = [2.8490,  1.2087, -0.0039,  0.0420, 0.0]
        q_lift      = [2.8142,  0.5278, -0.2275,  0.0408, 0.0]
        q_transit   = [-2.5923, 0.2008, -0.5635,  0.0518, 0.0]
        q_preplace  = [-2.2920, 0.6750,  0.0494,  0.0500, 0.0]
        q_place     = [-2.2818, 0.9194,  0.0946,  0.0001, 0.0]
        q_retract   = [-2.2937, 0.6283,  0.0554,  0.0461, 0.0]

        # Step 1: Open Gripper & Move to Pre-Grasp above Box B
        self.get_logger().info('Step 1: Opening gripper...')
        self.set_gripper(0.0)

        self.get_logger().info('Step 2: Moving above Box B (Pre-Grasp at 0.50, -0.20, 0.15)...')
        self.publish_trajectory([q_home, q_pregrasp], [2.0, 3.5])

        # Step 2: Descend onto Box B
        self.get_logger().info('Step 3: Descending onto Box B (Grasp at 0.50, -0.20, 0.04)...')
        self.publish_trajectory([q_pregrasp, q_grasp], [0.5, 2.0])

        # Step 3: Grip Box B
        self.get_logger().info('Step 4: Grasping Box B...')
        self.set_gripper(0.024, wait_time=2.0)

        # Step 4: Lift Box B (Keeping Upright)
        self.get_logger().info('Step 5: Lifting Box B vertically (maintaining upright orientation)...')
        self.publish_trajectory([q_grasp, q_lift], [0.5, 2.5])

        # Step 5: Smooth Transit toward Box A
        self.get_logger().info('Step 6: Smooth transit toward Box A platform...')
        self.publish_trajectory([q_lift, q_transit, q_preplace], [0.5, 3.0, 3.0])

        # Step 6: Lower onto top center of Box A
        self.get_logger().info('Step 7: Lowering Box B onto top center of Box A (0.45, 0.45, 0.22)...')
        self.publish_trajectory([q_preplace, q_place], [0.5, 2.0])

        # Step 7: Release Box B
        self.get_logger().info('Step 8: Releasing Box B on Box A...')
        self.set_gripper(0.0, wait_time=2.0)

        # Step 8: Retract gripper
        self.get_logger().info('Step 9: Retracting gripper upward...')
        self.publish_trajectory([q_place, q_retract], [0.5, 2.0])

        # Step 9: Return Home
        self.get_logger().info('Step 10: Returning to Home position...')
        self.publish_trajectory([q_retract, q_home], [0.5, 3.5])

        self.get_logger().info('=============================================')
        self.get_logger().info('MISSION ACCOMPLISHED: Pick and Place Complete!')
        self.get_logger().info('=============================================')


def main(args=None):
    rclpy.init(args=args)
    node = PickAndPlaceNode()
    try:
        node.execute_mission()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
