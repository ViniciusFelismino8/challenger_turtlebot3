#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import PoseStamped
from actionlib_msgs.msg import GoalID

class Robot:
  
  def __init__(self, frame_id):
    rospy.loginfo("Init Turtlebot3")
    self.pub_move_to_goal = rospy.Publisher("/move_base_simple/goal", PoseStamped, queue_size=1)
    self.frame_id = frame_id

  def move_to_goal(self, x=0, y=0, z=0, row=0, pitch=0, yaw=0, w=1):
    msg_move_to_goal = PoseStamped()
    msg_move_to_goal.pose.position.x = x
    msg_move_to_goal.pose.position.y = y
    msg_move_to_goal.pose.position.z = z
    msg_move_to_goal.pose.orientation.x = row
    msg_move_to_goal.pose.orientation.y = pitch
    msg_move_to_goal.pose.orientation.z = yaw
    msg_move_to_goal.pose.orientation.w = w
    msg_move_to_goal.header.frame_id = self.frame_id
    self.pub_move_to_goal.publish(msg_move_to_goal)

  def stop_move(self):
    rospy.Publisher("/Explore/cancel", GoalID, queue_size=1).publish(GoalID())