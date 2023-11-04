#!/usr/bin/env python3
import rospy
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Image
import socket
import struct
  
if __name__ == '__main__':
    tcp_client_socket1 = socket.socket( socket.AF_INET, socket.SOCK_STREAM )
    tcp_client_socket2 = socket.socket( socket.AF_INET, socket.SOCK_STREAM )
    tcp_client_socket3 = socket.socket( socket.AF_INET, socket.SOCK_STREAM )
    rospy.init_node('sender')
    flag = rospy.get_param('~flag')
    ip1, port1 = rospy.get_param('~ardrone1_ip'), rospy.get_param('~ardrone1_port')
    ip2, port2 = rospy.get_param('~ardrone2_ip'), rospy.get_param('~ardrone2_port')
    ip3, port3 = rospy.get_param('~ardrone3_ip'), rospy.get_param('~ardrone3_port')
    # print( type( ip1 ), type( port1 ) )
    # print(ip1, port1)
    tcp_client_socket1.connect( ( ip1, port1 ) )
    tcp_client_socket2.connect( ( ip2, port2 ) )
    tcp_client_socket3.connect( ( ip3, port3 ) )

    flag1 = struct.pack( b'i', 1 )
    flag2 = struct.pack( b'i', 2 )
    flag3 = struct.pack( b'i', 3 )

    if flag == 1:
        tcp_client_socket1.send( flag1 )
        tcp_client_socket2.send( flag1 )
        tcp_client_socket3.send( flag1 )
    elif flag == 2:
        tcp_client_socket1.send( flag2 )
        tcp_client_socket2.send( flag2 )
        tcp_client_socket3.send( flag2 )
    elif flag == 3:
        tcp_client_socket1.send( flag3 )
        tcp_client_socket2.send( flag3 )
        tcp_client_socket3.send( flag3 )
    
