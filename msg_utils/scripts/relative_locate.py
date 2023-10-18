#!/usr/bin/env python
import rosbag
import argparse
import os
import math
import random
import numpy as np
from msg_utils.msg import uwb

def parse_args():
    parser = argparse.ArgumentParser(
        prog = 'calc.py',
        description='calculate the location')
    parser.add_argument('bagfile', type=str, help='path to a bagfile')
    parser.add_argument('x0', type=int, help='path to a bagfile')
    parser.add_argument('y0', type=int, help='path to a bagfile')
    parser.add_argument('x1', type=int, help='path to a bagfile')
    parser.add_argument('y1', type=int, help='path to a bagfile')
    parser.add_argument('x2', type=int, help='path to a bagfile')
    parser.add_argument('y2', type=int, help='path to a bagfile')
    args = parser.parse_args()
    return args

def calculate( bagfile, x1: int, y1: int, x2: int, y2: int, x3: int, y3: int ):
    x, y, cnt = 0.0, 0.0, 0.0
    for topic, msg, t in rosbag.Bag(bagfile).read_messages():
        if topic[1:4] == 'uwb':
            dest_id = msg.dest_id
            dist = msg.dist
            if len( dist ) == 3:
                r1, r2, r3 = dist[0], dist[1], dist[2]
                # r1, r2, r3 = 1.0, math.sqrt(2.0), 1.0
                A = np.array([[2*x1 - 2*x3, 2*y1 - 2*y3],[2*x2 - 2*x3, 2*y2 - 2*y3]])
                B = np.array([[x1**2-x3**2+y1**2-y3**2+r3**2-r1**2],[x2**2-x3**2+y2**2-y3**2+r3**2-r2**2]])
                C = np.linalg.inv( A ) @ B
                x = x + C[0, 0]
                y = y + C[1, 0]
                cnt = cnt + 1
    print( x / cnt, y / cnt )
            # break

            
        # if topic[1:6] == 'vicon':
        #     drone_id = int(topic[-1])
        #     if drone_id == 0:
        #         num0 += 1
        #     if drone_id == 1:
        #         num1 += 1
        #     msg_time = msg.header.stamp
        #     msg_time_list[drone_id].append( msg_time )
        #     header_list[drone_id].append(msg.header)
        #     x, y, z = msg.pose.position.x, msg.pose.position.y, msg.pose.position.z
        #     x_list[drone_id].append(x)
        #     y_list[drone_id].append(y)
        #     z_list[drone_id].append(z)
        #     time_list[drone_id].append(t)

if __name__ == "__main__":
    args = parse_args()
    calculate( args.bagfile, args.x0, args.y0, args.x1, args.y1, args.x2, args.y2 )