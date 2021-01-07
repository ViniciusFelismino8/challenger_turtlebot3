#!/usr/bin/env python3

import rospy
import numpy as np
import cv2
import argparse
import yaml
import io
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image, CameraInfo
from std_msgs.msg import String, Int32
from geometry_msgs.msg import Point



class Vision:
 
  def __init__(self):
    #Publicar os topicos necessarios para utilizar no pacote
    self.image_pub = rospy.Publisher('camera/image', Image, queue_size=10)
    self.info_pub = rospy.Publisher('camera/camera_info', CameraInfo, queue_size=10)

    # Iniciar o ros_node
    rospy.init_node('apriltag_detector', anonymous=True)

  # Obter os parametros de calibracao da camera e envia como mensagem
  def get_camera_parameters(self,yaml_fname):
    with open(yaml_fname, "r") as file_handle:
      calib_data = yaml.load(file_handle, Loader=yaml.FullLoader)
    # Parse
    camera_info_msg = CameraInfo()
    camera_info_msg.width = calib_data["image_width"]
    camera_info_msg.height = calib_data["image_height"]
    camera_info_msg.K = calib_data["camera_matrix"]["data"]
    camera_info_msg.D = calib_data["distortion_coefficients"]["data"]
    camera_info_msg.R = calib_data["rectification_matrix"]["data"]
    camera_info_msg.P = calib_data["projection_matrix"]["data"]
    camera_info_msg.distortion_model = calib_data["distortion_model"]
    return camera_info_msg

  # Obter os parametros de calibracao da camera e envia como mensagem
  def pub_camera_info(self):
    # arg_parser = argparse.ArgumentParser()
    # arg_parser.add_argument('filename', help='Path to yaml file containing ' +\
    #                                         'camera calibration data')
    # args = arg_parser.parse_args()
    
    param = "/home/ubuntu/catkin_ws/src/challenge_turtlebot3/camera_info/turtlebot3_rpicamera_calibrated.yaml"

    # Parse yaml file
    camera_info_msg = self.get_camera_parameters(param)
  
    # while not rospy.is_shutdown():
    self.info_pub.publish(camera_info_msg)
    rospy.loginfo_once('CameraInfo message is being published under /camera/camera_info')


  # Publishes acquired images from the RaspiCam
  def pub_image(self, Event=None):
    # # taxa de publicacao
    rate = rospy.Rate(10)

    #Criar o objeto VideoCapture
    cap = cv2.VideoCapture(0, cv2.CAP_V4L2)

    # converter imagens ROS e OpenCV
    self.br = CvBridge()

    # Width and height capture size (in pixels)
    cap.set(3, 320)
    cap.set(4, 240)
    
    # While ROS is still running.
    while not rospy.is_shutdown():
        
        self.pub_camera_info()
      
        # Capture frame-by-frame
        # This method returns True/False as well
        # as the video frame.
        ret, frame = cap.read()
          
        if ret == True:
          # Print debugging information to the terminal
          rospy.loginfo_once('publishing video frame')
              
          # Publish the image.
          # The 'cv2_to_imgmsg' method converts an OpenCV
          # image to a ROS image message
          self.image_pub.publish(self.br.cv2_to_imgmsg(frame,'bgr8'))
              
        # Sleep just enough to maintain the desired rate
        rate.sleep()

  # def run(self):
  #   self.pub_image()


# Funcao main
if __name__ == '__main__':
  # Instantiate class
  sight = Vision()

  # Run publisher
  # try:
  #   while not rospy.is_shutdown():
  #     sight.run()
  # except rospy.ROSInterruptException:
  #   rospy.is_shutdown()

if __name__ == '__main__':
  # Sets different rates for each publisher from the node
  # rospy.Timer(rospy.Duration(1.0/20.0), sight.pub_camera_info)
  rospy.Timer(rospy.Duration(1.0/20.0), sight.pub_image)
  
  # Keep node running
  rospy.spin()