#!/usr/bin/env python
import rosbag
import rospy
from geometry_msgs.msg import PoseStamped
import time, sys, os
import argparse
import numpy as np

def getGroundTruthFromCsvFile():
    bag = rosbag.Bag( parsed.bag_name, 'w' )
    for file_name in os.listdir( parsed.dir ):
        topic_name = file_name[:]
        data = np.loadtxt( open( os.path.join( parsed.dir, file_name ), "rb" ), delimiter=parsed.delimiter )
        for row in data:
            timestamp_nsecs = str(row[0])
            timestamp = rospy.Time( int(timestamp_nsecs[0:-9]), int(timestamp_nsecs[-9:]) )

            pose = PoseStamped()
            pose.header.stamp = timestamp
            pose.header.frame_id = 'world'
            pose.pose.position.x = float(row[1])
            pose.pose.position.y = float(row[2])
            pose.pose.position.z = float(row[3])

            pose.pose.orientation.w = float(row[4])
            pose.pose.orientation.x = float(row[5])
            pose.pose.orientation.y = float(row[6])
            pose.pose.orientation.z = float(row[7])
            bag.write( "/{0}/pose".format( topic_name ), pose, timestamp )
    bag.close()


parser = argparse.ArgumentParser(description='Create a ROS bag using the other topic and the ground truth')
parser.add_argument('--dir', metavar='dir', nargs='?', help='ground truth data folder')
parser.add_argument('--bag_name', metavar='bag_name', default="data.bag", help='bag name')
parser.add_argument('--delimiter', metavar='delimiter', default=" ", help='csv delimiter')
# print help if no argument is specified
if len(sys.argv) < 3:
    parser.print_help()
    sys.exit(0)

# parse the args
parsed = parser.parse_args()
getGroundTruthFromCsvFile()