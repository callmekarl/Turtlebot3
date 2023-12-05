#! /usr/bin/env python3
# Copyright 2021 Samsung Research America
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import time
from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
import rclpy
from rclpy.duration import Duration

def main():
    rclpy.init()
    navigator = BasicNavigator()

    # Set initial pose
    initial_pose = PoseStamped()
    initial_pose.header.frame_id = 'map'
    initial_pose.header.stamp = navigator.get_clock().now().to_msg()
    initial_pose.pose.position.x = -0.06309102475643158
    initial_pose.pose.position.y = 0.028479816392064095
    initial_pose.pose.orientation.z = -0.0029002584525454657
    initial_pose.pose.orientation.w = 0.99999579424161
    navigator.setInitialPose(initial_pose)

    # Wait for navigation to fully activate
    navigator.waitUntilNav2Active()

    # Define the goal pose 1
    goal_pose = PoseStamped()
    goal_pose.header.frame_id = 'map'
    goal_pose.header.stamp = navigator.get_clock().now().to_msg()
    goal_pose.pose.position.x = 2.3828070163726807
    goal_pose.pose.position.y = 0.028348933905363083
    goal_pose.pose.orientation.z = -0.002900254461821974
    goal_pose.pose.orientation.w = 0.9999957942531842

    # Main loop
    while True:
        # Navigate to the goal pose
        navigator.goToPose(goal_pose)
        while not navigator.isTaskComplete():
            pass

        # Wait for 10 seconds at the goal pose
        time.sleep(5)

        # Check the result after reaching the goal
        result = navigator.getResult()
        if result == TaskResult.SUCCEEDED:
            print('Goal succeeded!')
        elif result == TaskResult.CANCELED:
            print('Goal was canceled!')
        elif result == TaskResult.FAILED:
            print('Goal failed!')
        else:
            print('Goal has an invalid return status!')

        # Go back to the initial pose
        initial_pose.header.stamp = navigator.get_clock().now().to_msg()
        navigator.goToPose(initial_pose)
        while not navigator.isTaskComplete():
            pass

        # Wait for 10 seconds at the initial pose
        time.sleep(5)

if __name__ == '__main__':
    main()
