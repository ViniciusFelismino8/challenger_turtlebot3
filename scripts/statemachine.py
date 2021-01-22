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
    self.robot = Robot('base_link')
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
        rospy.loginfo_once("STATE 1 - Aproximacao")
        trans, rot = self.robot.get_tag_pose(self.tag_id)
        self.robot.move_to_goal(x= trans[0], y=trans[1], yaw=rot[2], w=rot[3])
        self.count = self.count+1
        self.state = 2

      if self.state == 2:
        rospy.loginfo("STATE 2")
        print(self.robot.move_status)
        dist_tag=self.robot.get_tag_dist(self.tag_id)
        print(dist_tag)
        if self.robot.move_status == 3 or (dist_tag != -1 and dist_tag <= 0.4):
          self.robot.stop_move()
          if self.count == 3:
            self.state = 5
          self.state = 3
        else:  
          self.state = 2

        if self.robot.move_status >= 4 and self.robot.check_tag_id() == -1:
          self.state = 4
          # delay

      if self.state == 3:
        rospy.loginfo_once("STATE 3")
        self.tag_id = self.robot.check_tag_id()
        if self.tag_id == 134:
          self.tag_id = 130
        else: 
          self.tag_id = self.tag_id + 2
        trans, rot = self.robot.get_tag_child(self.tag_id)
        self.robot.move_to_goal(x= trans[0], y=trans[1], yaw=rot[2])
        self.count = self.count+1
        self.state = 2

      if self.state == 4:
        rospy.loginfo_once("STATE 4 - LOST")
        rospy.loginfo_once("REBOOT SYSTEM")
        os.system("roslaunch explore_lite explore.launch")
        self.state = 0


      if self.state == 5:
        rospy.loginfo_once("STATE 5 - FINISH")
        return True



if __name__ == "__main__":
  signal(SIGINT, handler)
  state_machine = StateMachine()
  
  while not rospy.is_shutdown():
      
    state_machine.run()
    rospy.spin()  