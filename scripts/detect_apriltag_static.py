# import the necessary packages
import apriltag
import argparse
import cv2

# construct the argument parser and parse the arguments
ap = argparse.ArgumentParser()
ap.add_argument("-i", "--image", required=True,
	help="path to input image containing AprilTag")
args = vars(ap.parse_args())

# load the input image and convert it to grayscale
print("[INFO] loading image...")
image = cv2.imread(args["image"])
gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

# define the AprilTags detector options and then detect the AprilTags
# in the input image
print("[INFO] detecting AprilTags...")
options = apriltag.DetectorOptions(families="tag36h11")
detector = apriltag.Detector(options)
results = detector.detect(gray)
print("[INFO] {} total AprilTags detected".format(len(results)))

# loop over the AprilTag detection results
for r in results:
	# extract the bounding box (x, y)-coordinates for the AprilTag
	# and convert each of the (x, y)-coordinate pairs to integers
	(ptA, ptB, ptC, ptD) = r.corners
	ptB = (int(ptB[0]), int(ptB[1]))
	ptC = (int(ptC[0]), int(ptC[1]))
	ptD = (int(ptD[0]), int(ptD[1]))
	ptA = (int(ptA[0]), int(ptA[1]))

	# draw the bounding box of the AprilTag detection
	cv2.line(image, ptA, ptB, (0, 255, 0), 2)
	cv2.line(image, ptB, ptC, (0, 255, 0), 2)
	cv2.line(image, ptC, ptD, (0, 255, 0), 2)
	cv2.line(image, ptD, ptA, (0, 255, 0), 2)

	# draw the center (x, y)-coordinates of the AprilTag
	(cX, cY) = (int(r.center[0]), int(r.center[1]))
	cv2.circle(image, (cX, cY), 5, (0, 0, 255), -1)

	# draw the tag family on the image
	tagFamily = r.tag_family.decode("utf-8")
	cv2.putText(image, tagFamily, (ptA[0], ptA[1] - 15),
		cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
	print("[INFO] tag family: {}".format(tagFamily))

# show the output image after AprilTag detection
cv2.imshow("Image", image)
cv2.waitKey(0)



















# import rospy
# import numpy as np
# import cv2
# from cv_bridge import CvBridge, CvBridgeError
# from sensor_msgs.msg import Image
# from std_msgs.msg import Float64
# from geometry_msgs.msg import Point
# import cv2.aruco as aruco
# import yaml
# import io



# class Vision:
  
#     def __init__(self):
#         # Initialize the CvBridge class
#         self.bridge = CvBridge()
#         # Initialize the ROS Node named 'aruco_detector'
#         rospy.init_node("aruco_detector")
#         # Get camera image
#         self.image_pub = rospy.Publisher("/camera/boitata_camera/image_raw", Image, queue_size=10)
#         # Aruco ID
#         self.pub_cnt_trk = rospy.Publisher("wbm_lsd-slam/aruco_id", Point, queue_size=1)   
#         # Aruco centroid
#         self.pub_dist_lines = rospy.Publisher("wbm_lsd-slam/aruco_centroid", Float64, queue_size=1)   

				
             
#     def send_image(self, img):
#         # Convert img to ros and pub image in a topic
#         img_pub = self.bridge.cv2_to_imgmsg(img, "bgr8")
#         self.image_pub.publish(img_pub)

#     def get_id(self, image):
#         # Set the region of interest
#         gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
#         aruco_dict = cv2.aruco.Dictionary_get(aruco.DICT_4X4_100)
#         arucoParameters = cv2.aruco.DetectorParameters_create()
        
#         corners, ids, rejectedImgPoints = cv2.aruco.detectMarkers(gray, aruco_dict, parameters=arucoParameters)

        
#         if np.all(ids != None):
#             image = aruco.drawDetectedMarkers(image, corners, ids, borderColor=(0, 0, 255))
#             display = aruco.drawDetectedMarkers(image, corners)
#             x1 = (corners[0][0][0][0], corners[0][0][0][1])
#             x2 = (corners[0][0][1][0], corners[0][0][1][1])
#             x3 = (corners[0][0][2][0], corners[0][0][2][1])
#             x4 = (corners[0][0][3][0], corners[0][0][3][1])
#             center = int((x1[0]+x3[0])/2), int((x1[1]+x3[1])/2)
            
#             image = cv2.circle(image, center, 2, (0, 0, 255))

        
#             for i, corner in zip(ids, corners):
#                 print('ID: {}; Corners: {}'.format(i, center))       
#         return image, ids

    
#     def callback(self,data): 
        
#         try:
#             cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
#         except CvBridgeError as e:
#             rospy.logerr("CvBridge Error: {0}".format(e))
          
#         # Get image perspective
#         # image_corners, ids = self.get_id(cv_image)
        
#         # self.send_image(cv_image)

#     def listener(self):
#         # Subscribe to a topic
#         # rospy.Subscriber('/camera/boitata_camera/image_raw', Image, self.callback)  
#         rospy.Subscriber('/my_camera/image', Image, self.callback) 
        
#         # Simply keeps python from exiting until this node is stopped
#         while not rospy.is_shutdown():
#             continue


# if __name__	== '__main__':
#   try:
#     cam_print = Vision()  
#     cam_print.listener()  
#   except rospy.ROSInterruptException:
#     pass