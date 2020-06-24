# Copyright (c) 2020 Boston Dynamics, Inc.  All rights reserved.
#
# Downloading, reproducing, distributing or otherwise using the SDK Software
# is subject to the terms and conditions of the Boston Dynamics Software
# Development Kit License (20191101-BDSDK-SL).

"""Simple image capture tutorial."""

import argparse
import sys

import bosdyn.client
import bosdyn.client.util
from bosdyn.client.image import ImageClient

import cv2
import numpy as np

import roslib
import sys
import rospy

from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError



class image_converter:

    def __init__(self):
        self.rgb_image_pub = rospy.Publisher("/frontleft_rgb_image",Image,queue_size=10)
        self.depth_image_pub = rospy.Publisher("/frontleft_depth_image",Image,queue_size=10)
        self.bridge = CvBridge()
    
    def publish_visual_images(self,img):
        self.rgb_image_pub.publish(self.bridge.cv2_to_imgmsg(img, "bgr8"))

    def publish_depth_images(self,img):
        self.depth_image_pub.publish(self.bridge.cv2_to_imgmsg(img, "bgr8"))

    def publish_images(self,image):
        # print("source name", image.shot.frame_name_image_sensor)
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

    rospy.init_node('image_sender', anonymous=True)
    ic = image_converter()
    print("ros node initialized")
    # Parse args
    parser = argparse.ArgumentParser()
    bosdyn.client.util.add_common_arguments(parser)
    parser.add_argument('--camera', help='Camera to aquire image from.', default='frontleft',\
    choices=['frontleft', 'frontright', 'left', 'right', 'back'])
    options = parser.parse_args(argv)

    sources = [options.camera + '_depth_in_visual_frame', options.camera + '_fisheye_image']

    # Create robot object with an image client.
    sdk = bosdyn.client.create_standard_sdk('image_depth_plus_visual')
    sdk.load_app_token(options.app_token)
    robot = sdk.create_robot(options.hostname)
    robot.authenticate(options.username, options.password)
    image_client = robot.ensure_client(ImageClient.default_service_name)




    while True:
        # Capture and save images to disk
        image_responses = image_client.get_image_from_sources(sources)

        # Image responses are in the same order as the requests.
        # Convert to opencv images.

        # Depth is a raw bytestream
        cv_depth = np.fromstring(image_responses[0].shot.image.data, dtype=np.uint16)
        cv_depth = cv_depth.reshape(image_responses[0].shot.image.rows,
                                    image_responses[0].shot.image.cols)

        # Visual is a JPEG
        cv_visual = cv2.imdecode(np.fromstring(image_responses[1].shot.image.data, dtype=np.uint8), -1)

        # Convert the visual image from a single channel to RGB so we can add color
        visual_rgb = cv2.cvtColor(cv_visual, cv2.COLOR_GRAY2RGB)

        # Map depth ranges to color

        # cv2.applyColorMap() only supports 8-bit; convert from 16-bit to 8-bit and do scaling
        min_val = np.min(cv_depth)
        max_val = np.max(cv_depth)
        depth_range = max_val - min_val
        depth8 = (255.0 / depth_range * cv_depth - min_val).astype('uint8')
        depth8_rgb = cv2.cvtColor(depth8, cv2.COLOR_GRAY2RGB)
        # depth_color = cv2.applyColorMap(depth8_rgb, cv2.COLORMAP_JET)

        # Add the two images together.
        # out = cv2.addWeighted(visual_rgb, 0.5, depth_color, 0.5, 0)
        # out = cv2.addWeighted(depth8_rgb, 0.5, depth_color, 0.5, 0)

        
        ic.publish_visual_images(visual_rgb)
        ic.publish_depth_images(depth8_rgb)

    # Write the image out.
    # print("filename", filename)
    # filename = options.camera + ".jpg"
    # print("filename", filename)
    # cv2.imwrite(filename, out)
    # print("here")

    # return True


if __name__ == "__main__":
    if not main(sys.argv[1:]):
        sys.exit(1)
