#!/usr/bin/env python

import roslib
roslib.load_manifest('anaglyph')
import sys
import rospy
import cv
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

class image_converter:

    def __init__(self, publish_topic, left_image_topic, right_image_topic):
        self.image_pub = rospy.Publisher(publish_topic,Image)

        #cv.NamedWindow("Image window", 1)
        self.bridge = CvBridge()
        self.left_sub = rospy.Subscriber(left_image_topic, Image, self.left_callback)
	self.right_sub = rospy.Subscriber(right_image_topic, Image, self.right_callback)
	self.left = False
	self.right = False
	self.out = False

    def left_callback(self,data):
        try:
            self.left = self.bridge.imgmsg_to_cv(data, "bgr8")
	    self.process()
        except CvBridgeError, e:
            print e
	
    def right_callback(self, data):
	try:
	    self.right = self.bridge.imgmsg_to_cv(data, "bgr8")
	    self.process()
	except CvBridgeError, e:
	    print e

    def process(self):
	if self.left!=False and self.right!=False:
	    channels = []
	    for i in range(6):
		channels.append(cv.cvCreateImage(cv.getSize(self.left),8,1)
	    cv.cvSplit(self.left, channels[0], channels[1], channels[2], None);
  	    cv.cvSplit(self.right, channels[3], channels[4], channels[5], None);
	    merge = cv.cvCreateImage(cv.cvGetSize(self.left, 8, 3)
   	    cv.cvMerge(channels[3],channels[4],channels[2], None, merge);
            #cv.ShowImage("Image window", merge)
            #cv.WaitKey(3)
            try:
                self.image_pub.publish(self.bridge.cv_to_imgmsg(merge, "bgr8"))
            except CvBridgeError, e:
                print e
	    self.left = False
	    self.right = False

def main(args):
    publish_topic = rospy.get_param('~publish_topic', "anaglyph_images")
    left_image_topic = rospy.get_param('~left_image_topic', "/left/image_raw")
    right_image_topic = rospy.get_param('~right_image_topic', "/right/image_raw")
    ic = image_converter(publish_topic, left_image_topic, right_image_topic)
    rospy.init_node('image_converter', anonymous=True)
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print "Shutting down"
    cv.DestroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)
