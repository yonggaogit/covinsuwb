#!/usr/bin/env python3
# license removed for brevity
import rospy
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Image
import socket
import struct


def talker():
    rospy.init_node('image_odom_recevier', anonymous=True)


    ip, port = rospy.get_param('~ardrone1_ip'), rospy.get_param('~ardrone1_port')


    tcp_server_socket = socket.socket( socket.AF_INET, socket.SOCK_STREAM )
    address = (ip, port)
    tcp_server_socket.bind(address)
    tcp_server_socket.listen( 128 )
    client_socket, clientAddr = tcp_server_socket.accept()

    image_pub = rospy.Publisher('/ardrone2/image', Image, queue_size=10)
    odom_pub = rospy.Publisher('/ardrone2/odom', Odometry, queue_size=10)


    rate = rospy.Rate(40)
    odom_msg = Odometry()
    image_msg = Image()
    recv_data_whole = bytes()
    
    while True:
        recv_data = client_socket.recv(struct.calcsize('614400siiiiiifffffff')+1000)
        if len(recv_data) == 0 :
            client_socket.close()
            tcp_server_socket.close()
            print('finish')
            break
        else:
            recv_data_whole += recv_data

            if recv_data_whole.__len__() == struct.calcsize('614400siiiiiifffffff'):
                frame, iseq, isecs, insecs, oseq, osecs, onsecs, px, py, pz, ox, oy, oz, ow = struct.unpack('614400siiiiiifffffff',recv_data_whole)
                image_msg.data = frame
                image_msg.header.seq = iseq
                image_msg.header.stamp.secs = isecs
                image_msg.header.stamp.nsecs = insecs
                image_msg.encoding = '16UC1'
                image_msg.height = 480
                image_msg.width = 640
                image_msg.step = 1280
                image_msg.is_bigendian = 0
                image_msg.header.frame_id = 'camera_depth_optical_frame'
                
                odom_msg.header.seq = oseq
                odom_msg.header.stamp.secs = osecs
                odom_msg.header.stamp.nsecs = onsecs
                odom_msg.pose.pose.position.x = px
                odom_msg.pose.pose.position.y = py
                odom_msg.pose.pose.position.z = pz
                odom_msg.pose.pose.orientation.x = ox
                odom_msg.pose.pose.orientation.y = oy
                odom_msg.pose.pose.orientation.z = oz
                odom_msg.pose.pose.orientation.w = ow
                odom_msg.header.frame_id = 'world'

                image_pub.publish(image_msg)
                odom_pub.publish(odom_msg)
                client_socket.send("ok".encode('utf-8'))
                recv_data_whole = bytes()
                rate.sleep()
                
 
if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass