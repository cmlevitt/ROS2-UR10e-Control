#!/usr/bin/env python3
"""
Pick and place routine executor for UR10e.
Sends goals directly to move_group /move_action — no MoveItPy needed.
"""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
import yaml
import os
import time
from ament_index_python.packages import get_package_share_directory

from geometry_msgs.msg import Pose, Point, Quaternion, PoseStamped
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import (
    MotionPlanRequest,
    WorkspaceParameters,
    Constraints,
    PositionConstraint,
    OrientationConstraint,
    BoundingVolume,
)
from shape_msgs.msg import SolidPrimitive
from std_msgs.msg import Header
from trajectory_msgs.msg import JointTrajectory
from control_msgs.msg import JointTolerance



class PnPNode(Node):
    def __init__(self):
        super().__init__('pnp_node')

        # ── Load YAML ──────────────────────────────────────────────
        pkg_share = get_package_share_directory('UR_pick_n_place')
        yaml_path = os.path.join(pkg_share, 'config', 'waypoints.yaml')
        with open(yaml_path, 'r') as f:
            config = yaml.safe_load(f)

        self.waypoints = config['waypoints']
        self.sequence  = config['sequence']
        self.settings  = config['settings']

        # ── Action client ───────────────────────────────────────────
        self._action_client = ActionClient(self, MoveGroup, '/move_action')
        self.get_logger().info("Waiting for /move_action server...")
        self._action_client.wait_for_server()
        self.get_logger().info("Connected to /move_action.")

        self.run_sequence()

    # ── Helpers ────────────────────────────────────────────────────

    def yaml_to_pose(self, name: str) -> Pose:
        wp = self.waypoints[name]
        pose = Pose()
        pose.position    = Point(**wp['position'])
        pose.orientation = Quaternion(**wp['orientation'])
        return pose

    def move_to_pose(self, pose: Pose) -> bool:
        s = self.settings

        # Build position constraint
        pos_constraint = PositionConstraint()
        pos_constraint.header = Header(frame_id=s['planning_frame'])
        pos_constraint.link_name = s['end_effector_link']
        pos_constraint.target_point_offset.x = 0.0
        pos_constraint.target_point_offset.y = 0.0
        pos_constraint.target_point_offset.z = 0.0

        bounding_box = SolidPrimitive()
        bounding_box.type = SolidPrimitive.BOX
        bounding_box.dimensions = [0.01, 0.01, 0.01]  # 1cm tolerance box

        pos_constraint.constraint_region = BoundingVolume()
        pos_constraint.constraint_region.primitives.append(bounding_box)

        primitive_pose = Pose()
        primitive_pose.position    = pose.position
        primitive_pose.orientation = pose.orientation
        pos_constraint.constraint_region.primitive_poses.append(primitive_pose)
        pos_constraint.weight = 1.0

        # Build orientation constraint
        ori_constraint = OrientationConstraint()
        ori_constraint.header = Header(frame_id=s['planning_frame'])
        ori_constraint.link_name = s['end_effector_link']
        ori_constraint.orientation = pose.orientation
        ori_constraint.absolute_x_axis_tolerance = 0.1
        ori_constraint.absolute_y_axis_tolerance = 0.1
        ori_constraint.absolute_z_axis_tolerance = 0.1
        ori_constraint.weight = 1.0

        # Combine into goal constraints
        goal_constraints = Constraints()
        goal_constraints.position_constraints.append(pos_constraint)
        goal_constraints.orientation_constraints.append(ori_constraint)

        # Build motion plan request
        request = MotionPlanRequest()
        request.group_name = 'ur_manipulator'
        request.num_planning_attempts = 5
        request.allowed_planning_time = 5.0
        request.max_velocity_scaling_factor     = s['velocity_scaling']
        request.max_acceleration_scaling_factor = s['acceleration_scaling']
        request.goal_constraints.append(goal_constraints)


        # Send goal
        goal_msg = MoveGroup.Goal()
        goal_msg.request = request
        goal_msg.planning_options.plan_only = False
        goal_msg.planning_options.replan = True
        goal_msg.planning_options.replan_attempts = 3

        self.get_logger().info("Sending goal to /move_action...")
        future = self._action_client.send_goal_async(goal_msg)
        rclpy.spin_until_future_complete(self, future)

        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error("Goal rejected.")
            return False

        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)

        result = result_future.result().result
        error_code = result.error_code.val

        if error_code == 1:  # SUCCESS
            return True
        elif error_code == -6:  # PREEMPTED — already at goal
            self.get_logger().warn("Motion preempted (already at goal), continuing.")
            return True
        elif error_code == -4:  # CONTROL_FAILED — often PREEMPTED when already at goal
            self.get_logger().warn("Control failed (possibly already at goal), continuing.")
            return True
        else:
            self.get_logger().error(f"Motion failed with error code: {error_code}")
            return False

    # ── Main sequence ──────────────────────────────────────────────

    def run_sequence(self):
        dip_names = {'dip_into_bed', 'dip_into_tank'}
        dwell     = self.settings['dip_dwell_seconds']

        # # DRY RUN - confirm waypoints load correctly, no motion
        # self.get_logger().info("Loaded waypoints:")
        # for name in self.sequence:
        #     pose = self.yaml_to_pose(name)
        #     self.get_logger().info(f"  {name}: {pose.position}")
        # return  # remove this line after dry run confirms OK

        for i, name in enumerate(self.sequence):
            self.get_logger().info(
                f"[{i+1}/{len(self.sequence)}] Moving to: {name}")
            pose    = self.yaml_to_pose(name)
            success = self.move_to_pose(pose)

            if not success:
                self.get_logger().error(
                    f"Failed at waypoint '{name}'. Halting.")
                return
            #return to execute just one

            if name in dip_names:
                self.get_logger().info(
                    f"  Dwelling for {dwell}s...")
                time.sleep(dwell)

        self.get_logger().info("Sequence complete.")


def main(args=None):
    rclpy.init(args=args)
    node = PnPNode()
    rclpy.shutdown()


if __name__ == '__main__':
    main()