#!/usr/bin/env python3
import os
import rospy
from geometry_msgs.msg import PoseStamped
from robot import Robot
from actionlib_msgs.msg import GoalID, GoalStatusArray
from std_msgs.msg import Int32
from tf2_msgs.msg import TFMessage

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
        tag_id = self.robot.check_tag_id()
        print(tag_id)
        if tag_id:
          os.system("rosnode kill /explore")
          os.system('rostopic pub /move_base_simple/goal geometry_msgs/PoseStamped {}')
          # self.robot.stop_move()
          print('Parei')
          self.state = 1
        self.state = 0

      if self.state == 1:
        rospy.loginfo("STATE 1")
        trans, rot = self.robot.get_tag_pose(self.tag_id)
        self.tag_id = None
        self.robot.move_to_goal(x= trans[0], y=trans[1], yaw=rot[2])
        self.count = self.count+1
        self.state = 2

      if self.state == 2:
        rospy.loginfo("STATE 2")
        if self.robot.move_status != 1:
          if self.count == 3:
            return True
          self.state = 3
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
    state_machine = StateMachine()

    while not rospy.is_shutdown():
        
      state_machine.run()
      rospy.spin()  