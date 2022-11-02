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

def create_bag(path, image_files, image_topic, frequency, bag_name, enc, out_path):
    bridge = CvBridge()
    bag = rosbag.Bag(os.path.join(out_path, bag_name + ".bag"), 'w')
    print ("Generating bag from {} image files".format(len(image_files)))
    time = rospy.get_rostime()
    d = rospy.Duration.from_sec(1.0/float(frequency))
    i = 0
    while i < len(image_files) and not rospy.is_shutdown():
        image_file = image_files[i]

        img_cv = cv2.imread(path + image_file, cv2.IMREAD_UNCHANGED)
        
        image_message = bridge.cv2_to_imgmsg(img_cv, encoding=enc)

        image_message.header.stamp.secs = time.secs
        image_message.header.stamp.nsecs = time.nsecs
        image_message.header.seq = i

        print (image_message.header.seq, image_message.header.stamp, image_file, image_message.height, image_message.width, image_message.encoding, image_message.is_bigendian, image_message.step)

        bag.write(image_topic, image_message, image_message.header.stamp)
        time += d
        i += 1

    bag.close()
    print ("Bag successfully generated")
	

def main(args):
    rospy.init_node('bag_from_images_hz', anonymous=True)
    if rospy.has_param('~images_path') and rospy.has_param('~frequency') and rospy.has_param('~topic') and rospy.has_param('~bag_name') and rospy.has_param('~encoding'):
        images_path = rospy.get_param('~images_path')
        frequency = rospy.get_param('~frequency')
        topic = rospy.get_param('~topic')
        bag_name = rospy.get_param('~bag_name')
        encoding = rospy.get_param('~encoding')
        
        rospack = rospkg.RosPack()
        out_path = rospack.get_path('rosbag_gen_utils') 
        if rospy.has_param('~out_path'):
            out_path = rospy.get_param('~out_path')
        
        image_files = get_files_from_dir(images_path)
        create_bag(images_path, image_files, topic, frequency, bag_name, encoding, out_path)
    else:
        print ("Error: parameter not specified")


if __name__ == '__main__':
    main(sys.argv)
