#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import PoseStamped
from poseinicial import Robot
from actionlib_msgs.msg import GoalID, GoalStatusArray
from std_msgs.msg import Int32
from tf2_msgs.msg import TFMessage


class StateMachine:
  state = 0
  move_base_info = None
  apriltag_id = None
  case = []


  def __init__(self):
    rospy.init_node("machine", anonymous=True)
    rospy.Subscriber("/tf", TFMessage, self.callback_apriltag, queue_size=1)
    self.robot = Robot('map')
    # self.case.append({'x': 6.6, 'y': 0.9, 'yaw': 0})
    # self.case.append({'x': 3.4, 'y': 6.6, 'yaw': 1.0})
    # self.case.append({'x': -6.8, 'y': 6.8, 'yaw': -3.14})
    rospy.timer.sleep(1)

  def callback_main(self, data):
    input_ = data.status_list[0].status if data.status_list else 2
    self.FSM(input_)

  def callback_apriltag(self, data):
    self.apriltag_id = data.transforms[0].child_frame_id
    # self.dist_apriltag=data.transforms[0].transform.translation.z
 
  def ED0(self, input_):
    case = self.case[0]
    #aqui inicia a exploracao ate achar o aruco
    self.state = 2

  def ED1(self, input_):
    print("Case 2: ", input_)
    if (time.time() - self.time_old) > 10 and input_ == 3:
      self.state = 2 

  def ED2(self, input_):
    if input_ == 3:
      self.robot.stop_move()
      self.robot.move_to_goal(x=self.case[self.apriltag_id]['x'], y=self.case[self.apriltag_id]['y'], yaw=self.case[self.apriltag_id]['yaw'])
      self.state = 1
      self.time_old = time.time()
    
    # caso de parada
    # if self.apriltag_id == 5:
    #   self.state = 3
    
  def ED3(self, input_):
    if input_ != 1:
      rospy.loginfo("Finish my jorney!")

  def FSM(self, input_):
    switch = {
      0 : self.ED0,
      1 : self.ED1,
      2 : self.ED2,
      3 : self.ED3,
    }
    func = switch.get(self.state, lambda: None)
    func(input_)

  def run(self):
    rospy.Subscriber("/move_base/status", GoalStatusArray, self.callback_main)

if __name__ == "__main__":
  rospy.loginfo_once("Start Machine")
  state_machine = StateMachine()
  state_machine.run()
  while not rospy.is_shutdown():
    rospy.spin() 