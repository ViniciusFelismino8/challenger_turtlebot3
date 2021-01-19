#!/usr/bin/env python3 
import rospy

import re
import tf_conversions

import tf2_ros
import geometry_msgs.msg
from apriltag_ros.msg import AprilTagDetectionArray


def handle_turtle_pose(msg):
    br = tf2_ros.TransformBroadcaster()
    t = geometry_msgs.msg.TransformStamped()

    t.header.stamp = rospy.Time.now()
    t.header.frame_id = "camera"

    if len(msg.detections) > 0:
        teste = str(msg.detections[0].id)
        teste2 = re.findall('\d+', teste )[0]
        t.child_frame_id = "tag_"+ str(teste2)
        t.transform.translation.x = msg.detections[0].pose.pose.pose.position.x
        t.transform.translation.y = msg.detections[0].pose.pose.pose.position.y
        t.transform.translation.z = msg.detections[0].pose.pose.pose.position.z
        t.transform.rotation.x = msg.detections[0].pose.pose.pose.orientation.x
        t.transform.rotation.y = msg.detections[0].pose.pose.pose.orientation.y
        t.transform.rotation.z = msg.detections[0].pose.pose.pose.orientation.z
        t.transform.rotation.w = msg.detections[0].pose.pose.pose.orientation.w
        br.sendTransform(t)

if __name__ == '__main__':
    rospy.init_node('tf2_turtle_broadcaster')
    rospy.Subscriber('/tag_detections',
                     AprilTagDetectionArray,
                     handle_turtle_pose)
    rospy.spin()