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

    # Define goal poses
    goal_poses = [PoseStamped() for _ in range(3)]
    goal_poses[1].header.frame_id = 'map'
    goal_poses[1].pose.position.x = 3.026761054992676
    goal_poses[1].pose.position.y = 1.992578387260437
    goal_poses[1].pose.orientation.z = 0.960445090193172
    goal_poses[1].pose.orientation.w = 0.2784694394791456

    goal_poses[2].header.frame_id = 'map'
    goal_poses[2].pose.position.x = -1.5473958253860474
    goal_poses[2].pose.position.y = 3.438857078552246
    goal_poses[2].pose.orientation.z = -0.012702573579062883
    goal_poses[2].pose.orientation.w = 0.999919319057527

    # Main navigation loop
    while True:
        for goal_pose in goal_poses[1:]:
            goal_pose.header.stamp = navigator.get_clock().now().to_msg()
            navigator.goToPose(goal_pose)
            while not navigator.isTaskComplete():
                pass

            # Wait for 10 seconds at the pose
            time.sleep(5)

        # Return to initial pose
        initial_pose.header.stamp = navigator.get_clock().now().to_msg()
        navigator.goToPose(initial_pose)
        while not navigator.isTaskComplete():
            pass

        # Wait for 10 seconds at the initial pose
        time.sleep(5)

if __name__ == '__main__':
    main()

