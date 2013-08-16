#!/usr/bin/env python

import roslib
roslib.load_manifest('raven_pose_estimator')
import rospy

import Util

import cv

from sensor_msgs.msg import Image, CameraInfo
from stereo_msgs.msg import DisparityImage
import cv_bridge

RED_LOWER_HSV = cv.Scalar(0, 120, 50)
RED_UPPER_HSV = cv.Scalar(8, 255, 255)
LOWERHSV = RED_LOWER_HSV
UPPERHSV = RED_UPPER_HSV

class DisparitySegmenter():
    def __init__(self, left_camera, right_camera, disparity):
        self.image = {'l': None, 'r': None}
        self.info = {}
        self.bridge = cv_bridge.CvBridge()
        self.calculating = False
        self.foundCentroidInLeft = False

        self.height = 50
        self.width = 200
        self.disparity_left = cv.CreateMat(self.height * 2, self.width * 2, cv.CV_16S)
        self.disparity_right = cv.CreateMat(self.height * 2, self.width * 2, cv.CV_16S)
        self.hsvImg = cv.CreateImage((1280, 960), 8, 3)
        self.threshImg = cv.CreateImage((1280, 960), 8, 1)

        rospy.Subscriber(left_camera + "/camera_info", CameraInfo, self.leftInfoCallback)
        rospy.Subscriber(right_camera + "/camera_info", CameraInfo, self.rightInfoCallback)
        rospy.Subscriber(disparity, DisparityImage, self.disparityCallback)

    def leftInfoCallback(self, msg):
        self.info['l'] = msg
        rospy.Subscriber(left_camera + "/image_rect_color", Image, self.leftImageCallback, queue_size = 1)

    def rightInfoCallback(self, msg):
        self.info['r'] = msg
        rospy.Subscriber(right_camera + "/image_rect_color", Image, self.rightImageCallback, queue_size = 1)

    def leftImageCallback(self, msg):
        self.image['l'] = self.process(msg, 'l')
        Util.showImage(self.image['l'], "left")
        self.handleBoth()

    def rightImageCallback(self, msg):
        self.image['r'] = self.process(msg, 'r')
        Util.showImage(self.image['r'], "right")
        self.handleBoth()

    def handleBoth(self):
        if self.image['l'] and self.image['r'] and not self.calculating:
            self.calculating = True

            left = self.image['l']
            right = self.image['r']

            # data structure initialization
            state = cv.CreateStereoGCState(16,2)
            # running the graph-cut algorithm
            print 'starting graph cut'
            cv.FindStereoCorrespondenceGC(left,right,
                                      self.disparity_left,self.disparity_right,state)
            if rospy.is_shutdown():
                return
            print 'finished graph cut'

            disp_left_visual = cv.CreateMat(left.height, left.width, cv.CV_8U)
            cv.ConvertScale( self.disparity_left, disp_left_visual, -16 );
            #cv.ConvertScale( self.disparity_right, disp_left_visual, 16 );
            cv.Save( "disparity.pgm", self.disparity_left )

            Util.showImage(disp_left_visual, "Disparity left")
            self.calculating = False

    def process(self, msg, arm):
        image = self.bridge.imgmsg_to_cv(msg, "bgr8")

        if arm == 'l':
            threshImg = Util.threshold(image, self.hsvImg, self.threshImg, LOWERHSV, UPPERHSV)
            self.foundCentroidInLeft, self.xClose, self.yClose, xCentroid, yCentroid = Util.find_centroid(threshImg)

        if self.foundCentroidInLeft:
            pixel = (self.xClose - 100, self.yClose)
            image = Util.getROI(image, pixel, self.width, self.height)
            if arm == 'r':
                cv.Smooth(image, image, cv.CV_GAUSSIAN, 3, 0)
            gray_image = cv.CreateMat(image.rows, image.cols, cv.CV_8U)
            cv.CvtColor(image, gray_image, cv.CV_BGR2GRAY)
            return gray_image

def test():
    segmenter = DisparitySegmenter('left', 'right')
    
if __name__ == '__main__':
    rospy.init_node('disparity_segmenter')
    left_camera = 'left'
    right_camera = 'right'
    segmenter = DisparitySegmenter(left_camera, right_camera)
    rospy.spin()
