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
#
# Modified by AutomaticAddison.com
 
import time  # Time library
 
from geometry_msgs.msg import PoseStamped # Pose with ref frame and timestamp
from rclpy.duration import Duration # Handles time for ROS 2
import rclpy # Python client library for ROS 2
 
from nav2_simple_commander.robot_navigator import BasicNavigator
from tf_transformations import quaternion_from_euler

'''
Navigates a robot from an initial pose to a goal pose.
'''

def createPose(x:float, y:float, yaw:float=None)->PoseStamped:
    pose = PoseStamped()
    pose.header.frame_id = "map"
    pose.pose.position.x = x
    pose.pose.position.y = y
    pose.pose.position.z = 0.0
    if (yaw is not None):
      q = quaternion_from_euler(0, 0, yaw)
      pose.pose.orientation.x = q[0]
      pose.pose.orientation.y = q[1]
      pose.pose.orientation.z = q[2]
      pose.pose.orientation.w = q[3]
    return pose

def main():
 
  # Start the ROS 2 Python Client Library
  rclpy.init()
 
  # Launch the ROS 2 Navigation Stack
  navigator = BasicNavigator()
 
  # Wait for navigation to fully activate. Use this line if autostart is set to true.
  navigator.waitUntilNav2Active()

  pose1 = createPose(1.0, -2.0)
  pose2 = createPose(2.0, -2.0)
  pose3 = createPose(3.0, -1.5)
  pose4 = createPose(3.0, -1.0)
  pose5 = createPose(2.0, -0.5)
  pose6 = createPose(1.0, -0.5)
  pose7 = createPose(1.0, -1.0)
  poses = [pose1, pose2, pose3, pose4, pose5, pose6, pose7]

  navigator.goThroughPoses(poses)
  while not navigator.isTaskComplete():
    time.sleep(0.5)
 
if __name__ == '__main__':
    #get the x, y, yaw from user input
    # # is the value
    main()

    
