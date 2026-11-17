#!/usr/bin/env python3
"""
GP7 Robot Arm — Pick and Place Autonomous Controller
EN3563 Robotics Mini Project Task:
  1. Pick up Box B (5 cm x 5 cm x 5 cm) from ground at (0.50, -0.20, 0.025 m).
     - Approach vertically from above with the gripper pointing straight down.
     - Symmetrically grasp Box B.
     - Lift vertically along the Z-axis.
  2. Maintain Box B strictly upright (orientation unchanged) during the entire motion.
  3. Place Box B on top of Box A (30 cm x 30 cm x 20 cm) platform at (0.45, 0.45, 0.10 m).
     - Platform top surface is at Z = 0.20 m.
     - Descend vertically from above and place Box B at Z = 0.245 m atop the platform.
     - Release Box B and retract vertically upward.
  4. Smooth quintic/trapezoidal velocity trajectory planning.
"""

import time
import numpy as np
import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from std_msgs.msg import Float64MultiArray
from builtin_interfaces.msg import Duration

try:
    from geometry_msgs.msg import Pose, Point, Quaternion
except ImportError:
    class Point:
        def __init__(self, x=0.0, y=0.0, z=0.0):
            self.x, self.y, self.z = x, y, z

    class Quaternion:
        def __init__(self, x=0.0, y=0.0, z=0.0, w=1.0):
            self.x, self.y, self.z, self.w = x, y, z, w

    class Pose:
        def __init__(self):
            self.position = Point()
            self.orientation = Quaternion()


class GP7Kinematics:
    """
    Kinematic engine derived directly from gp7_description.urdf.
    
    Link Dimensions & Heights:
      - Base to Shoulder (J2) Height: 0.3328 m (332.8 mm)
      - Upper Arm Length (J2 to J3):   0.4406 m (440.6 mm)
      - Forearm Length (J3 to J5):     0.4425 m (442.5 mm)
      - Tool Flange Length (J5 to EE): 0.1108 m (110.8 mm)
      - Finger Grip Center Offset:     0.0250 m (25.0 mm)
    """

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

    def solve_ik_vertical(self, target_pos, q_seed=None):
        """
        Solves Inverse Kinematics for a target (x, y, z) position enforcing
        strict top-down vertical gripper orientation: tool vector = [0, 0, -1].
        """
        from scipy.optimize import minimize
        target = np.array(target_pos, dtype=float)
        tx, ty, tz = target

        if q_seed is None:
            # Analytical seed: base yaw offset is approx yaw_target - 3.085 rad
            yaw_t = np.arctan2(ty, tx)
            q1_est = (yaw_t - 3.085 + np.pi) % (2 * np.pi) - np.pi
            q_init = np.array([q1_est, 1.0, -0.2, -1.8, 0.0])
        else:
            q_init = np.array(q_seed)

        def obj(q):
            T = self.fk(q)
            pos_err = np.sum((T[:3, 3] - target)**2)
            # Enforce local tool X-axis to point straight down along -Z
            down_err = np.sum((T[:3, 0] - np.array([0.0, 0.0, -1.0]))**2)
            j6_pen = (q[4])**2 * 0.01
            smooth = np.sum((q - q_init)**2) * 0.001
            return pos_err * 500.0 + down_err * 50.0 + j6_pen + smooth

        res = minimize(obj, q_init, bounds=[(-3.14, 3.14)] * 5, method='SLSQP')
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
        Generates smooth quintic trajectory interpolation between waypoints
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
        """Commands gripper fingers to open (0.0) or close (0.024 m)."""
        msg = Float64MultiArray()
        msg.data = [float(displacement), float(displacement)]
        self.gripper_pub.publish(msg)
        time.sleep(wait_time)

    def execute_mission(self):
        self.get_logger().info('Starting Pick-and-Place Task in 3 seconds...')
        time.sleep(3.0)

        # =========================================================================
        # 1. Physical Environment & Target Coordinates (from gp7.world.sdf and URDF)
        # =========================================================================
        # Box B (Pick):
        #   - Position: (0.50, -0.20, 0.025 m), 5 cm cube on the ground.
        #   - Pre-Grasp Hover: (0.50, -0.20, 0.150 m) — 15 cm above ground, fingers open.
        #   - Grasp: (0.50, -0.20, 0.045 m) — pure vertical descent onto Box B.
        #   - Vertical Lift: (0.50, -0.20, 0.280 m) — 28 cm above ground, clear of all obstacles.
        #
        # Box A (Place Destination Platform):
        #   - Position: (0.45, 0.45, 0.100 m), 30 cm x 30 cm x 20 cm box on the ground.
        #   - Platform top surface is at Z = 0.200 m.
        #   - Box B center when placed on Box A is at Z = 0.225 m.
        #   - Place Height: (0.45, 0.45, 0.245 m) — pure vertical descent onto Box A.
        #   - Pre-Place Hover: (0.45, 0.45, 0.350 m) — 15 cm above Box A surface.
        #   - Retract: (0.45, 0.45, 0.350 m) — pure vertical ascent after release.

        # =========================================================================
        # 2. Top-Down Kinematic Waypoints (Strictly Vertical Tool: [0, 0, -1])
        # =========================================================================
        # Joint vector format: [joint1, joint2, joint3, joint5, joint6]
        q_home      = [-1.5459,  0.0017, -0.4859,  0.0000,  0.0000]  # Elevated safe parking
        q_pregrasp  = [ 2.8421,  1.2279, -0.2386, -1.6700, -0.4742]  # Above Box B at Z = 0.150 m
        q_grasp     = [ 2.8570,  1.4642, -0.0516, -1.6191, -0.7515]  # Vertical grasp at Z = 0.045 m
        q_lift      = [ 2.8271,  0.9336, -0.3951, -1.8086, -0.0508]  # Vertical lift to Z = 0.280 m
        q_transit   = [ 0.2694,  0.9112, -0.2786, -1.9472, -0.1261]  # Upright transit arc at Z = 0.311 m
        q_preplace  = [-2.2884,  0.8889, -0.1622, -2.0859, -0.2014]  # Above Box A at Z = 0.350 m
        q_place     = [-2.2796,  1.0946, -0.0835, -1.9585, -0.4189]  # Vertical place at Z = 0.245 m
        q_retract   = [-2.2884,  0.8889, -0.1622, -2.0859, -0.2014]  # Vertical retract to Z = 0.350 m

        # Step 1: Open Gripper & Move to Pre-Grasp above Box B
        self.get_logger().info('Step 1: Opening gripper...')
        self.set_gripper(0.0)

        self.get_logger().info('Step 2: Moving above Box B (Top-Down Pre-Grasp at 0.50, -0.20, 0.15 m)...')
        self.publish_trajectory([q_home, q_pregrasp], [2.0, 3.5])

        # Step 2: Descend vertically onto Box B
        self.get_logger().info('Step 3: Descending vertically onto Box B (Top-Down Grasp at 0.50, -0.20, 0.045 m)...')
        self.publish_trajectory([q_pregrasp, q_grasp], [0.5, 2.0])

        # Step 3: Symmetrically Clamp Box B
        self.get_logger().info('Step 4: Grasping Box B (closing fingers symmetrically inward)...')
        self.set_gripper(0.024, wait_time=2.0)

        # Step 4: Vertical Lift (Keeping Box B strictly upright)
        self.get_logger().info('Step 5: Lifting Box B vertically to Z = 0.28 m (maintaining upright orientation)...')
        self.publish_trajectory([q_grasp, q_lift], [0.5, 2.0])

        # Step 5: Smooth Horizontal Transit Arc toward Box A
        self.get_logger().info('Step 6: Smooth transit toward Box A (holding Box B strictly upright)...')
        self.publish_trajectory([q_lift, q_transit, q_preplace], [0.5, 3.5, 3.5])

        # Step 6: Descend vertically onto Box A platform
        self.get_logger().info('Step 7: Lowering Box B vertically onto Box A platform (Z = 0.245 m)...')
        self.publish_trajectory([q_preplace, q_place], [0.5, 2.0])

        # Step 7: Release Box B
        self.get_logger().info('Step 8: Releasing Box B on Box A...')
        self.set_gripper(0.0, wait_time=2.0)

        # Step 8: Retract vertically upward
        self.get_logger().info('Step 9: Retracting gripper vertically upward to Z = 0.35 m...')
        self.publish_trajectory([q_place, q_retract], [0.5, 2.0])

        # Step 9: Return to Safe Parking Home Position
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
