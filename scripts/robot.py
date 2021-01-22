#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import PoseStamped
from actionlib_msgs.msg import GoalID, GoalStatusArray
import tf


class Robot:
  move_status = 0

  def __init__(self, frame_id):
    rospy.loginfo("Init Turtlebot3")
    self.pub_move_to_goal = rospy.Publisher("/move_base_simple/goal", PoseStamped, queue_size=1)
    self.frame_id = frame_id
    rospy.Subscriber("/move_base/status", GoalStatusArray, self.callback_move_status)
    # self.tag0 = False
    # self.tag1 = False
    # self.tag2 = False
    self.listener = tf.TransformListener()

  def callback_move_status(self, data):
    self.move_status = data.status_list[0].status if data.status_list else 0

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

  def check_tag_id(self):
    try:
      trans, rot = self.listener.lookupTransform('/base_link', '/tag_130', rospy.Time(0))
      return 130
    except:
      try:
        trans, rot = self.listener.lookupTransform('/base_link', '/tag_132', rospy.Time(0))
        return 132
      except:
        try:
          trans, rot = self.listener.lookupTransform('/base_link', '/tag_134', rospy.Time(0))
          return 134
        except:
          return -1

  def get_tag_pose(self, id):
    try:
      trans, rot = self.listener.lookupTransform('/base_link', '/near_tag_'+str(id), rospy.Time(0))
      return trans, rot
    except:
      return None

  def get_tag_child(self, id):
    try:
      trans, rot = self.listener.lookupTransform('/base_link', '/pose'+str(id), rospy.Time(0))
      return trans, rot
    except:
      return None

  def get_tag_dist(self, id):
    try:
      trans, rot = self.listener.lookupTransform('/base_link', '/near_tag_'+str(id), rospy.Time(0))
      return trans[0]
    except:
      return -1

  def stop_move(self):
    rospy.Publisher("/move_base_simple/goal", PoseStamped, queue_size=1).publish(PoseStamped())