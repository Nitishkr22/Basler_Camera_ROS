#!/usr/bin/env python2
# -*- coding: utf-8 -*-

# Copyright 2016 Massachusetts Institute of Technology

"""Extract images from a rosbag.
"""

import os
import argparse

import cv2
import sys
import rosbag
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from datetime import datetime
import numpy as np


def imgmsg_to_cv2(img_msg):
    if img_msg.encoding != "bgr8":
        rospy.logerr("This Coral detect node has been hardcoded to the 'bgr8' encoding.  Come change the code if you're actually trying to implement a new camera")
    dtype = np.dtype("uint8") # Hardcode to 8 bits...
    dtype = dtype.newbyteorder('>' if img_msg.is_bigendian else '<')
    image_opencv = np.ndarray(shape=(img_msg.height, img_msg.width, 3), # and three channels of data. Since OpenCV works with bgr natively, we don't need to reorder the channels.
                    dtype=dtype, buffer=img_msg.data)
    # If the byt order is different between the message and the system.
    if img_msg.is_bigendian == (sys.byteorder == 'little'):
        image_opencv = image_opencv.byteswap().newbyteorder()
    return image_opencv

def main():
    """Extract a folder of images from a rosbag.
    """
    #parser = argparse.ArgumentParser(description="Extract images from a ROS bag.")
    # #parser.add_argument("bag_file", help="Input ROS bag.")
    # parser.add_argument("output_dir", help="Output directory.")
    # parser.add_argument("image_topic", help="Image topic.")

    #args = parser.parse_args()
    bag_file = "/mnt/dsu0/ros_cam_rad/cam_rec_20_06/2023-06-20-11-02-11.bag"
    image_topics = ["/camera1_images","/camera2_images","/camera3_images"]
    output_dir = "/mnt/dsu0/ros_cam_rad/cam_rec_20_06/"


    #print ("Extract images from %s on topic %s into %s" % (bag_file, output_dir))

    bag = rosbag.Bag(bag_file, "r")
    bridge = CvBridge()
    count = 0
    for count,image_topic in enumerate(image_topics):
        folder_path = os.path.join(output_dir, f"folder{count}") 
        if not os.path.exists(folder_path):
            os.makedirs(folder_path)
            print(f"Created folder: {folder_path}")
        else:
            print(f"Folder already exists: {folder_path}")
        for topic, msg, t in bag.read_messages(topics=[image_topic]):
            cv_img = imgmsg_to_cv2(msg)

            timestamp = t.to_sec() # Convert to seconds

            # Convert timestamp to datetime object
            dt = datetime.fromtimestamp(timestamp)

            # Format the datetime object as 'yy/mm/dd/hh/mm/ss.ssss'
            formatted_timestamp = dt.strftime('%Y-%m-%d-%H-%M-%S.%f')
            #print(type(formatted_timestamp))
            filename = f'image_{formatted_timestamp}.jpg'
            cv2.imwrite(folder_path+"/"+filename,cv_img)
            #cv2.imwrite(os.path.join(output_dir, "frame%06i.png" % count), cv_img)
            # cv2.imwrite(os.path.join(output_dir, "./{0}_{1}.png".format(formatted_timestamp,count)), cv_img)
            # cv2.imwrite(os.path.join(output_dir, f"image_{formatted_timestamp}.png"), cv_img)
            print ("Wrote image %i" % count)

            count += 1

    bag.close()

    return

if __name__ == '__main__':
    main()
