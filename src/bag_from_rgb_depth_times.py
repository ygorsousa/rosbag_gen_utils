#! /usr/bin/env python

import rospy
import rospkg
import cv2
import sys
import rosbag
import os
from cv_bridge import CvBridge

"""

@author: Ygor Sousa (ycns@cin.ufpe.br)
"""

def get_files_from_dir(path):
    print ("Getting images from directory "+ str(path))
	
    files = []
	
    sorted_files = sorted(os.listdir(path))
    for image_file in sorted_files:
        if image_file.endswith(".jpeg") or image_file.endswith(".jpg") or image_file.endswith(".png"):
            files.append(image_file)
    
    return files

def add_data_images(bag, bridge, rostime, path, image_files, topic, multiplier, encoding_data):
    print ("- Converting and writing {} image files".format(len(image_files)))
    i = 0
    while i < len(image_files) and not rospy.is_shutdown():
        image_file = image_files[i]

        img_cv = cv2.imread(path + image_file, cv2.IMREAD_UNCHANGED)

        image_message = bridge.cv2_to_imgmsg(img_cv, encoding=encoding_data)
		
        stamp = (image_file.split('-')[1]).split('.')[0]
        d = rospy.Duration.from_sec(float(stamp[:6] + "." + stamp[6:]))
        t = rostime + (d * multiplier)
        image_message.header.stamp.secs = t.secs
        image_message.header.stamp.nsecs = t.nsecs
        image_message.header.seq = i

        print (image_message.header.seq, image_message.header.stamp, image_file, image_message.height, image_message.width, image_message.encoding, image_message.is_bigendian, image_message.step)

        bag.write(topic, image_message, image_message.header.stamp)
        i += 1

def create_bag(rgb_path, depth_path, rgb_files, depth_files, rgb_topic, depth_topic, bag_name, multiplier):
    bridge = CvBridge()
    rospack = rospkg.RosPack()
    bag = rosbag.Bag(rospack.get_path('rosbag_gen_utils') + "/" + bag_name + ".bag", 'w')
    print ("Starting bag generation")
    
    rostime = rospy.get_rostime()
    
    add_data_images(bag, bridge, rostime, rgb_path, rgb_files, rgb_topic, multiplier, "bgr8")
    add_data_images(bag, bridge, rostime, depth_path, depth_files, depth_topic, multiplier, "passthrough")

    bag.close()
    print ("Bag successfully generated")
	

def main(args):
    rospy.init_node('bag_from_rgb_depth_times', anonymous=True)
    if rospy.has_param('~rgb_images_path') and rospy.has_param('~depth_images_path') and rospy.has_param('~rgb_topic') and rospy.has_param('~depth_topic') and rospy.has_param('~bag_name'):
        rgb_images_path = rospy.get_param('~rgb_images_path')
        depth_images_path = rospy.get_param('~depth_images_path')
        rgb_topic = rospy.get_param('~rgb_topic')
        depth_topic = rospy.get_param('~depth_topic')
        bag_name = rospy.get_param('~bag_name')
        multiplier = 1
        if rospy.has_param('~multiplier'):
            multiplier = rospy.get_param('~multiplier')
        rgb_files = get_files_from_dir(rgb_images_path)
        depth_files = get_files_from_dir(depth_images_path)
        create_bag(rgb_images_path, depth_images_path, rgb_files, depth_files, rgb_topic, depth_topic, bag_name, multiplier)
    else:
        print ("Error: parameter not specified")


if __name__ == '__main__':
    main(sys.argv)
