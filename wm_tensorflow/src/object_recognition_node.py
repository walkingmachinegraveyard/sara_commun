#!/usr/bin/env python
import roslib
roslib.load_manifest('wm_tensorflow')
import subprocess
import sys
import rospy
import cv2
import os
from std_msgs.msg import String, Bool
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError




class image_converter:

  def __init__(self):
    self.start = False
    #self.image_pub = rospy.Publisher("image_topic_2",Image)
    self.tts_pub = rospy.Publisher("sara_tts", String)
    self.bridge = CvBridge()
    camera_topic = rospy.get_param('object_recognition_node/camera_topic')
    self.image_sub = rospy.Subscriber(camera_topic, Image,self.callback)
    self.start_recognition = rospy.Subscriber('object_recognition_node/start_recognition', Bool, self.start_callback)
   

  def callback(self,data):
    if self.start:
	  try:
	    cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
	  except CvBridgeError as e:
            self.start = False
	    print(e)

	  try:
	    #self.image_pub.publish(self.bridge.cv2_to_imgmsg(cv_image, "bgr8"))
            print("Recognition started...")
	    cv2.imwrite( "/home/jeffrey/images/temp.jpg", cv_image )
            cv2.namedWindow('dst_rt', cv2.WINDOW_NORMAL)
            cv2.imshow( "dst_rt", cv_image )
            cv2.waitKey(1)
	    os.chdir("/home/jeffrey/tensorflow")
            out = subprocess.check_output(["bazel-bin/tensorflow/examples/label_image/label_image", "--image=/home/jeffrey/images/temp.jpg"])
            self.tts_pub.publish("This is a " + out)
            self.start = False

	  except CvBridgeError as e:
            self.start
	    print(e)

  def start_callback(self,data):
    self.start = True

def main(args):
  ic = image_converter()
  rospy.init_node('object_recognition_node', anonymous=True)
  ic.tts_pub.publish("Show me something, I will try to recognize it !")
  print("Waiting for start command...")
  try:
    rospy.spin()
  except KeyboardInterrupt:
    print("Shutting down")
  cv2.destroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)



