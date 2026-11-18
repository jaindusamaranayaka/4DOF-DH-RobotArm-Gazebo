#!/usr/bin/env python3
"""
GP7 Robot Arm — Pick and Place Autonomous Controller (with DetachableJoint)
"""

import subprocess
import time
import threading

import rclpy
from rclpy.node import Node

from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from std_msgs.msg import Float64MultiArray
from builtin_interfaces.msg import Duration


class PickAndPlaceNode(Node):

    def __init__(self):
        super().__init__('gp7_pick_and_place')

        self.arm_pub = self.create_publisher(
            JointTrajectory,
            '/arm_controller/joint_trajectory',
            10
        )

        self.gripper_pub = self.create_publisher(
            Float64MultiArray,
            '/gripper_controller/commands',
            10
        )

        self.arm_joints = ['joint1', 'joint2', 'joint3', 'joint5', 'joint6']

        self._target_gripper = None
        self._gripper_running = True
        self._gripper_lock = threading.Lock()

        self._gripper_thread = threading.Thread(
            target=self._gripper_loop,
            daemon=True
        )
        self._gripper_thread.start()

        self.get_logger().info('GP7 Pick-and-Place Controller Initialized')

    def wait_for_controllers(self, timeout=15.0):
        self.get_logger().info('Waiting for arm and gripper controllers...')
        start_time = time.monotonic()

        while time.monotonic() - start_time < timeout:
            arm_ready = self.arm_pub.get_subscription_count() > 0
            gripper_ready = self.gripper_pub.get_subscription_count() > 0

            if arm_ready and gripper_ready:
                self.get_logger().info('Arm and gripper controllers connected.')
                return True

            time.sleep(0.1)

        self.get_logger().error('Controller connection timeout!')
        return False

    def _gripper_loop(self):
        publish_interval = 0.05  # 20 Hz

        while self._gripper_running:
            with self._gripper_lock:
                target = self._target_gripper

            if target is not None:
                msg = Float64MultiArray()
                msg.data = [float(target), float(target)]
                self.gripper_pub.publish(msg)

            time.sleep(publish_interval)

    def set_gripper(self, displacement, wait_time=1.0):
        with self._gripper_lock:
            self._target_gripper = float(displacement)

        self.get_logger().info(f'Gripper target: {displacement:.4f} m')
        time.sleep(wait_time)

    def attach_box(self):
        subprocess.run(
            ['ign', 'topic', '-t', '/box_b/attach', '-m', 'ignition.msgs.Empty', '-p', ' '],
            check=False
        )
        self.get_logger().info('Attached box_b to gripper via DetachableJoint.')

    def detach_box(self):
        subprocess.run(
            ['ign', 'topic', '-t', '/box_b/detach', '-m', 'ignition.msgs.Empty', '-p', ' '],
            check=False
        )
        self.get_logger().info('Detached box_b from gripper via DetachableJoint.')

    def publish_trajectory(self, waypoints, durations):
        if len(waypoints) != len(durations):
            raise ValueError('Number of waypoints and durations must match.')
        if len(waypoints) < 2:
            raise ValueError('Trajectory must contain at least two waypoints.')

        msg = JointTrajectory()
        msg.joint_names = self.arm_joints

        # 1. Reach the first waypoint cleanly at durations[0]
        cumulative_time = float(durations[0])
        first_pt = JointTrajectoryPoint()
        first_pt.positions = [float(v) for v in waypoints[0]]
        first_pt.velocities = [0.0] * len(self.arm_joints)
        sec = int(cumulative_time)
        nanosec = int((cumulative_time - sec) * 1e9)
        first_pt.time_from_start = Duration(sec=sec, nanosec=nanosec)
        msg.points.append(first_pt)

        # 2. Smooth quintic minimum-jerk interpolation for all remaining segments
        steps_per_sec = 20
        for seg in range(len(waypoints) - 1):
            q_start = waypoints[seg]
            q_end = waypoints[seg + 1]
            T = float(durations[seg + 1])

            if len(q_start) != len(self.arm_joints) or len(q_end) != len(self.arm_joints):
                raise ValueError('Each waypoint must contain five joint positions.')
            if T <= 0:
                raise ValueError('Trajectory durations must be positive.')

            n_steps = max(2, int(T * steps_per_sec))

            for k in range(1, n_steps + 1):
                t = (k / n_steps) * T
                tau = t / T

                # Quintic minimum-jerk profile
                s = 10 * tau**3 - 15 * tau**4 + 6 * tau**5
                s_dot = (30 * tau**2 - 60 * tau**3 + 30 * tau**4) / T

                point = JointTrajectoryPoint()
                point.positions = [
                    float(q_start[j] + s * (q_end[j] - q_start[j]))
                    for j in range(len(self.arm_joints))
                ]
                point.velocities = [
                    float(s_dot * (q_end[j] - q_start[j]))
                    for j in range(len(self.arm_joints))
                ]

                t_total = cumulative_time + t
                sec = int(t_total)
                nanosec = int((t_total - sec) * 1e9)
                point.time_from_start = Duration(sec=sec, nanosec=nanosec)
                msg.points.append(point)

            cumulative_time += T

        self.arm_pub.publish(msg)
        self.get_logger().info(
            f'Published smooth trajectory ({len(msg.points)} points, {cumulative_time:.1f} s).'
        )
        time.sleep(cumulative_time + 0.3)

    def destroy_node(self):
        self._gripper_running = False
        if self._gripper_thread is not None:
            self._gripper_thread.join(timeout=1.0)
        super().destroy_node()

    def execute_mission(self):
        self.get_logger().info('Starting GP7 Pick-and-Place Mission...')
        time.sleep(1.0)

        # Joint configurations: [joint1, joint2, joint3, joint5, joint6]
        q_home     = [-1.5459,  0.0017, -0.4859,  0.0000,  0.0000]
        q_pregrasp = [ 2.4316,  0.9367,  0.0586,  0.8941,  0.7854]
        q_grasp    = [ 2.4411,  1.1800,  0.1209,  1.0158,  0.7854]
        q_lift     = [ 2.4239,  0.7568,  0.0576,  0.7153,  0.7853]

        # Straight-line Cartesian transit waypoints (X = 0.50 m, 90° short path)
        q_t1       = [ 2.7247,  0.4688, -0.3911,  0.8761,  0.7853]
        q_t2       = [ 3.1781,  0.3328, -0.5408,  0.8897,  0.7854]
        q_t3       = [ 3.6502,  0.4050, -0.3701,  0.7913,  0.7854]

        # Place & retract above Box A (end_base Z = 0.225 m -> Box B bottom ~8.8 mm above Box A)
        q_preplace = [ 3.9916,  0.6602,  0.0988,  0.5777,  0.7852]
        q_place    = [ 3.9978,  0.8348,  0.0492,  0.8017,  0.7854]
        q_retract  = [ 3.9916,  0.6601,  0.0987,  0.5777,  0.7854]
        q_home_end = [ 4.7373,  0.0017, -0.4859,  0.0000,  0.0000]

        # STEP 1: Detach box_b first (plugin starts attached on spawn) & open gripper
        self.get_logger().info('STEP 1: Ensuring box_b is detached and opening gripper...')
        self.detach_box()
        self.set_gripper(displacement=0.030, wait_time=1.5)

        # STEP 2: Move above Box B
        self.get_logger().info('STEP 2: Moving to pre-grasp position...')
        self.publish_trajectory([q_home, q_pregrasp], [2.0, 3.5])

        # STEP 3: Descend to Box B
        self.get_logger().info('STEP 3: Descending to grasp position...')
        self.publish_trajectory([q_pregrasp, q_grasp], [0.5, 2.0])

        # STEP 4: Close fingers lightly (no overlap pressure) & lock box_b
        self.get_logger().info('STEP 4: Closing gripper and locking Box B...')
        self.set_gripper(displacement=-0.011, wait_time=1.0)
        self.attach_box()
        time.sleep(0.5)

        # STEP 5: Lift Box B
        self.get_logger().info('STEP 5: Lifting Box B...')
        self.publish_trajectory([q_grasp, q_lift], [0.5, 3.0])

        # STEP 6: Transport Box B
        self.get_logger().info('STEP 6: Transporting Box B toward Box A...')
        self.publish_trajectory(
            [q_lift, q_t1, q_t2, q_t3, q_preplace],
            [0.5, 3.0, 3.0, 3.0, 3.0]
        )

        # STEP 7: Lower Box B onto Box A
        self.get_logger().info('STEP 7: Lowering Box B onto Box A...')
        self.publish_trajectory([q_preplace, q_place], [0.5, 3.5])

        # STEP 8: Detach Box B & open gripper
        self.get_logger().info('STEP 8: Detaching and releasing Box B...')
        self.detach_box()
        time.sleep(0.2)
        self.set_gripper(displacement=0.030, wait_time=1.2)

        # STEP 9: Retract
        self.get_logger().info('STEP 9: Retracting the gripper...')
        self.publish_trajectory([q_place, q_retract], [0.5, 2.0])

        # STEP 10: Return home
        self.get_logger().info('STEP 10: Returning to home position...')
        self.publish_trajectory([q_retract, q_home_end], [0.5, 3.5])

        self.get_logger().info('=============================================')
        self.get_logger().info('PICK AND PLACE MISSION COMPLETED!')
        self.get_logger().info('=============================================')


def main(args=None):
    rclpy.init(args=args)
    node = PickAndPlaceNode()

    try:
        if not node.wait_for_controllers(timeout=15.0):
            return
        node.execute_mission()

    except KeyboardInterrupt:
        node.get_logger().info('Mission interrupted by user.')

    except Exception as error:
        node.get_logger().error(f'Mission failed: {error}')
        raise

    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
