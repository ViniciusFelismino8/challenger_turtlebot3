#!/usr/bin/env python3
import os
import rospy
from geometry_msgs.msg import PoseStamped
from robot import Robot
from actionlib_msgs.msg import GoalID, GoalStatusArray
from std_msgs.msg import Int32
from tf2_msgs.msg import TFMessage
from signal import signal, SIGINT
from sys import exit

def handler(signal_received, frame):
  # Handle any cleanup here
  print('SIGINT or CTRL-C detected. Exiting gracefully')
  exit(0)
class StateMachine:
  state = 0
  case = []
  tag_id = None
  count = 0

  def __init__(self):
    rospy.init_node("machine", anonymous=True)
    self.robot = Robot('map')
    rospy.timer.sleep(1)

  def run(self):
    while True:
      rospy.sleep(1)

      if self.state == 0:
        rospy.loginfo_once("STATE 0")
        self.tag_id = self.robot.check_tag_id()
        print(self.tag_id)
        if self.tag_id >= 0:
          os.system("rosnode kill /explore")
          self.robot.stop_move()
          self.state = 1
        # self.state = 0
        rospy.timer.sleep(2)

      if self.state == 1:
        rospy.loginfo_once("STATE 1")
        trans, rot = self.robot.get_tag_pose(self.tag_id)
        self.tag_id = None
        self.robot.move_to_goal(x= trans[0], y=trans[1], yaw=rot[2])
        self.count = self.count+1
        self.state = 2

      if self.state == 2:
        rospy.loginfo("STATE 2")
        print(self.robot.move_status)
        if self.robot.move_status == 2:
          if self.count == 3:
            return True
          self.state = 3
        else:  
          self.state = 2
          # delay

      if self.state == 3:
        rospy.loginfo("STATE 3")
        tag_id = self.robot.check_tag_id()
        trans, rot = self.robot.get_tag_child(tag_id)
        self.tag_id=None
        self.robot.move_to_goal(x= trans[0], y=trans[1], yaw=rot[2])
        self.count = self.count+1
        self.state = 2


if __name__ == "__main__":
  signal(SIGINT, handler)
  state_machine = StateMachine()
  
  while not rospy.is_shutdown():
      
    state_machine.run()
    rospy.spin()  