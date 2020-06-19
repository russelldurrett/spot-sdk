#!/usr/bin/env python3
# Copyright (c) 2020 Boston Dynamics, Inc.  All rights reserved.
#
# Downloading, reproducing, distributing or otherwise using the SDK Software
# is subject to the terms and conditions of the Boston Dynamics Software
# Development Kit License (20191101-BDSDK-SL).

"""Simple image capture tutorial."""

import argparse
import sys

from bosdyn.api import image_pb2
import bosdyn.client
import bosdyn.client.util
from bosdyn.client.image import ImageClient
from bosdyn.api import image_pb2
import cv2
import numpy as np

import roslib
import sys
import rospy

import cv2
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError


import tf



class image_converter:

    def __init__(self):
        self.fl_image_pub = rospy.Publisher("/frontleft_image",Image,queue_size=10)
        self.fr_image_pub = rospy.Publisher("/frontright_image",Image,queue_size=10)
        self.b_image_pub = rospy.Publisher("/back_image",Image,queue_size=10)
        self.l_image_pub = rospy.Publisher("/left_image",Image,queue_size=10)
        self.r_image_pub = rospy.Publisher("/right_image",Image,queue_size=10)
        self.bridge = CvBridge()
        # self.image_sub = rospy.Subscriber("/wide_stereo/left/image_rect_throttle",Image,self.callback)
    
    def publish_images(self,image):
        # print("source name", image.shot.frame_name_image_sensor)
        # print("----------------------------------")

        if image.shot.image.pixel_format == image_pb2.Image.PIXEL_FORMAT_DEPTH_U16:
            dtype = np.uint16
            extension = ".png"
        else:
            dtype = np.uint8
            extension = ".jpg"

        img = np.fromstring(image.shot.image.data, dtype=dtype)
        if image.shot.image.format == image_pb2.Image.FORMAT_RAW:
            img = img.reshape(image.shot.image.rows, image.shot.image.cols)
        else:
            img = cv2.imdecode(img, -1)

        if image.shot.frame_name_image_sensor=="frontleft_fisheye":
            self.fl_image_pub.publish(self.bridge.cv2_to_imgmsg(img, "mono8"))
        elif image.shot.frame_name_image_sensor=="frontright_fisheye":
            self.fr_image_pub.publish(self.bridge.cv2_to_imgmsg(img, "mono8"))
        elif image.shot.frame_name_image_sensor=="back_fisheye":
            self.b_image_pub.publish(self.bridge.cv2_to_imgmsg(img, "mono8"))
        elif image.shot.frame_name_image_sensor=="left_fisheye":
            self.l_image_pub.publish(self.bridge.cv2_to_imgmsg(img, "mono8"))
        elif image.shot.frame_name_image_sensor=="right_fisheye":
            self.r_image_pub.publish(self.bridge.cv2_to_imgmsg(img, "mono8"))
            


    def callback(self,data):
        print("callback")
        try:
                cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
                print(e)

        (rows,cols,channels) = cv_image.shape
        if cols > 60 and rows > 60 :
                cv2.circle(cv_image, (150,50), 20, 255)

        cv2.imshow("Image window", cv_image)
        cv2.waitKey(3)

        try:
                self.image_pub.publish(self.bridge.cv2_to_imgmsg(cv_image, "bgr8"))
        except CvBridgeError as e:
                print(e)




def main(argv):
    # Parse args
    rospy.init_node('image_converter', anonymous=True)
    ic = image_converter()
    print("ros node initialized")
    # input()

    parser = argparse.ArgumentParser()
    bosdyn.client.util.add_common_arguments(parser)
    parser.add_argument('--list', help='list image sources', action='store_true')
    parser.add_argument('--image-sources', help='Get image from source(s)', action='append')
    options = parser.parse_args(argv)

    # Create robot object with an image client.
    sdk = bosdyn.client.create_standard_sdk('image_capture')
    sdk.load_app_token(options.app_token)
    robot = sdk.create_robot(options.hostname)
    robot.authenticate(options.username, options.password)
    image_client = robot.ensure_client(ImageClient.default_service_name)

    # try:
        # rospy.spin()
    # except KeyboardInterrupt:
        # print("Shutting down")


    # Optionally list image sources on robot.
    while True:
        if options.list:
            image_sources = image_client.list_image_sources()
            print(image_sources)
            # input()
            print("Image sources:")
            for source in image_sources:
                print("\t" + source.name)

        # Optionally capture one or more images.
        if options.image_sources:
            # Capture and save images to disk
            image_responses = image_client.get_image_from_sources(options.image_sources)

            for image in image_responses:
                # if image.shot.image.pixel_format == image_pb2.Image.PIXEL_FORMAT_DEPTH_U16:
                    # dtype = np.uint16
                    # extension = ".png"
                # else:
                    # dtype = np.uint8
                    # extension = ".jpg"

                # img = np.fromstring(image.shot.image.data, dtype=dtype)
                # if image.shot.image.format == image_pb2.Image.FORMAT_RAW:
                    # img = img.reshape(image.shot.image.rows, image.shot.image.cols)
                # else:
                    # img = cv2.imdecode(img, -1)

                ic.publish_images(image)
                # ic.image_pub.publish(ic.bridge.cv2_to_imgmsg(img, "mono8"))
                # cv2.imwrite(image.source.name + extension, img)
    return True


if __name__ == "__main__":
    if not main(sys.argv[1:]):
        sys.exit(1)
