#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import PoseStamped
from actionlib_msgs.msg import GoalStatusArray
import tf

class Robot:
  mo
 None

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
      trans, rot = self.listener.lookupTransform('/camera', '/tag_0', rospy.Time(0))
      return 0
    except:
      try:
        trans, rot = self.listener.lookupTransform('/camera', '/tag_1', rospy.Time(0))
        return 1
      except:
        try:
          trans, rot = self.listener.lookupTransform('/camera', '/tag_2', rospy.Time(0))
          return 2
        except:
          return None

  def get_tag_pose(self, id):
    try:
      trans, rot = self.listener.lookupTransform('/camera', '/tag_'+id, rospy.Time(0))
      return trans, rot
    except:
      return None

  def get_tag_child(self, id):
    if id == 2:
      id = 0
    else: 
      id = id + 1
    
    try:
      trans, rot = self.listener.lookupTransform('/camera', '/pose'+id, rospy.Time(0))
      return trans, rot
    except:
      return None
