#!/usr/bin/env python3

from threading import Thread

import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.node import Node

from pymoveit2 import MoveIt2
from pymoveit2.robots import ur5

import time

def main():
    rclpy.init()

    # Create node for this example
    node = Node("task1b_moveit")

    # Declare a list of goal poses
    goal_poses = [
        {
            "position": [0.35, 0.1, 0.68],
            "quat_xyzw": [0.5, 0.5, 0.5, 0.5],
            "cartesian": False
        },
        {
            "position": [-0.37, 0.12, 0.397],
            "quat_xyzw": [0.5, 0.5, 0.5, 0.5],
            "cartesian": False
        },
        {
            "position": [0.194, -0.43, 0.701],
            "quat_xyzw": [0.5, 0.5, 0.5, 0.5],
            "cartesian": False
        },
        {
            "position": [-0.37, 0.12, 0.397],
            "quat_xyzw": [0.5, 0.5, 0.5, 0.5],
            "cartesian": False
        }
    ]

    # Create callback group that allows execution of callbacks in parallel without restrictions
    callback_group = ReentrantCallbackGroup()

    # Create MoveIt 2 interface
    moveit2 = MoveIt2(
        node=node,
        joint_names=ur5.joint_names(),  # You need to define 'ur5' or use the actual robot's information.
        base_link_name=ur5.base_link_name(),
        end_effector_name=ur5.end_effector_name(),
        group_name=ur5.MOVE_GROUP_ARM,
        callback_group=callback_group,
    )

    # Spin the node in background thread(s)
    executor = rclpy.executors.MultiThreadedExecutor(2)
    executor.add_node(node)
    executor_thread = Thread(target=executor.spin, daemon=True, args=())
    executor_thread.start()

    for goal_pose in goal_poses:
        
        position = goal_pose["position"]
        quat_xyzw = goal_pose["quat_xyzw"]
        cartesian = goal_pose["cartesian"]

        # Move to pose
        node.get_logger().info(
            f"Moving to {{position: {list(position)}, quat_xyzw: {list(quat_xyzw)}}}"
        )
        moveit2.move_to_pose(position=position, quat_xyzw=quat_xyzw, cartesian=cartesian)
        moveit2.wait_until_executed()
        
        time.sleep(2)

    rclpy.shutdown()
    exit(0)

if __name__ == "__main__":
    main()
