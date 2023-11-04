#!/usr/bin/env python3
import math 
import numpy as np 
import rospy
from std_msgs.msg import std_msgs
import csv
from matplotlib.animation import FuncAnimation
import os
from math import sin,cos,radians
import argparse
import sys
import utm

a = 6378137. #长轴
f = 1 / 298.257223563 #扁率
def gps_to_cartesion(lat,lon,alt):
    lat = radians(lat)
    lon = radians(lon)
    b = a * (1 - f)
    e = (a ** 2 - b ** 2) ** 0.5 / a
    N = a / (1 - e ** 2 * sin(lat) ** 2) ** 0.5

    X = (N + alt) * cos(lat) * cos(lon)
    Y = (N + alt) * cos(lat) * sin(lon)
    Z = (N * (1 - e ** 2) + alt) * sin(lat)

    return X,Y,Z

def to_xyz_3(M_lat, M_lon, M_alt, O_lat, O_lon, O_alt):
    Ea = 6378137
    Eb = 6356725
    M_lat = math.radians(M_lat)
    M_lon = math.radians(M_lon)
    O_lat = math.radians(O_lat)
    O_lon = math.radians(O_lon)
    Ec = Ea * (1 - (Ea - Eb) / Ea * ((math.sin(M_lat)) ** 2)) + M_alt
    Ed = Ec * math.cos(M_lat)
    d_lat = M_lat - O_lat
    d_lon = M_lon - O_lon
    x = d_lat * Ec
    y = d_lon * Ed
    z = M_alt - O_alt
    return x, y, z



def transfer():
    for file_name in os.listdir( parsed.dir ):
        data = np.loadtxt( open( os.path.join( parsed.dir, file_name ), "rb" ), delimiter = ' ' )
        O_lat, O_lon = utm.to_latlon( data[0,1], data[0,2],  49, 'N' )
        O_alt = data[0,3]
        data_new = []
        for row in data:
            # timestamp_nsecs = str(row[0])
            # timestamp = rospy.Time( int(timestamp_nsecs[0:-9]), int(timestamp_nsecs[-9:]) )
            
            lat, lon = utm.to_latlon( float(row[1]), float(row[2]), 49, 'N' )
            alt = float(row[3])
            print( lat, lon, alt )
            orientation_x, orientation_y, orientation_z, orientation_w = float(row[4]), float(row[5]), float(row[6]), float(row[7])
            x, y, z = to_xyz_3(lat, lon, alt, O_lat, O_lon, O_alt)

            row_new = [ row[0], x, y, z, orientation_x, orientation_y, orientation_z, orientation_w ]

            data_new.append( row_new )
        file_name_new = 'tum_' + file_name
        with open(os.path.join(parsed.dir, file_name_new), "w") as csvfile:
            writer = csv.writer( csvfile )
            writer.writerows(data_new)

parser = argparse.ArgumentParser(description='transfer gps data format to position data format')
parser.add_argument('--dir', metavar='dir', help='data folder need to be transfer')
if len(sys.argv) < 1:
    parser.print_help()
    sys.exit(0)

parsed = parser.parse_args()
transfer()