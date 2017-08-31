import rosbag
import roslib
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import sys
import getopt
from math import cos, sin, floor, acos, pi
import numpy as np
import math
import copy
import cv2
import jsonpickle
import time
import os
import os.path
import shutil
import tempfile
import tf
import numpy
# def Euler_to_Quat(roll, pitch, yaw):
#     
#     t0 = cos(yaw * 0.5);
#     t1 = sin(yaw * 0.5);
#     t2 = cos(roll * 0.5);
#     t3 = sin(roll * 0.5);
#     t4 = cos(pitch * 0.5);
#     t5 = sin(pitch * 0.5);
#  
#     w = t0 * t2 * t4 + t1 * t3 * t5;
#     x = t0 * t3 * t4 - t1 * t2 * t5;
#     y = t0 * t2 * t5 + t1 * t3 * t4;
#     z = t1 * t2 * t4 - t0 * t3 * t5;
#  
#     return w, x, y, z 


def main(argv):
    bagDirectory = ''
    bagFiles = []
    outputFile = ''

    try:
        opts, args = getopt.getopt(
            argv, "hd:b:o:", ["bags_dir=", "bag_file=", "output_file="])
    except getopt.GetoptError:
        print ('getBagData.py -d <bags_dir> -b <bag_file> -o <output_file> ')
        sys.exit(2)
    for opt, arg in opts:
        if opt == '-h':
            print ('getBagData.py -d <bags_dir> -b <bag_file> -o <output_file> ')
            sys.exit()
        elif opt in ("-d", "--bags_dir"):
            bagDirectory = arg
        elif opt in ("-b", "--bag_file"):
            bagFiles.append(arg)
        elif opt in ("-o", "--output_file"):
            outputFile = arg
    if len(bagFiles) < 1:
        print ('getBagData.py -d <bags_dir> -b <bag_file> -o <output_file> ')
        sys.exit()
    if bagDirectory == '':
        print ('getBagData.py -d <bags_dir> -b <bag_file> -o <output_file> ')
        sys.exit()
    if outputFile == '':
        print ('getBagData.py -d <bags_dir> -b <bag_file> -o <output_file> ')
        sys.exit()

    print ('bag_directory: {}'.format(bagDirectory))
    print ('bagFiles: {}'.format(bagFiles))
    print ('output_file: {}'.format(outputFile))
    bagLocation = bagDirectory + "/" + bagFiles[0]
    outputFile = bagDirectory + "/" + outputFile
    parseBag(bagLocation, outputFile)

    
def parseBag(bagLocation, outputFile):
    bag = rosbag.Bag(bagLocation)
    fileOut = open(outputFile, 'w')
    old_ticks_scan =0
    old_ticks_odom =0 
    count_scan =0
    count_odom =0
    
    try:
        for topic, msg, t in bag.read_messages():
                if topic == "/scan": 
                    count_scan = count_scan +1 
                    ticks_scan = (msg.header.stamp.secs + numpy.int64(719162*24*60*60)) * numpy.int64(10000000) + (msg.header.stamp.nsecs + 50) / 100
                    if (old_ticks_scan <= ticks_scan):
                        print >> fileOut, "ticks: ", ticks_scan
                        print >> fileOut, "type: scan"
                        print >> fileOut, msg, "\n"
                        old_ticks_scan = ticks_scan
                    else:
                        print "Data scan[%d] not sorted" %count_scan  
                if topic == "buddy/Odometry":
                    count_odom  = count_odom  +1 
                    ticks_odom = (msg.header.stamp.secs + numpy.int64(719162*24*60*60)) * numpy.int64(10000000) + (msg.header.stamp.nsecs + 50) / 100
                    if (old_ticks_odom <= ticks_odom):
                        print >> fileOut, "ticks: ", ticks_odom
                        print >> fileOut, "type: odom"
                        print >> fileOut, msg, "\n"
                        old_ticks_odom = ticks_odom
                    else:
                        print "Data odom[%d] not sorted" %count_odom 
    finally:
        bag.close()
        fileOut.close()

if __name__ == "__main__":
    main(sys.argv[1:])