#!/usr/bin/env python3
# license removed for brevity
import rospy
from sensor_msgs.msg import Image
from geometry_msgs.msg import PoseStamped
import socket
import struct


def talker():
    rospy.init_node('fly_recevier', anonymous=True)


    ip, port = rospy.get_param('~ardrone_ip'), rospy.get_param('~ardrone_port')
    x1, y1 = rospy.get_param('~x1'), rospy.get_param('~y1')
    x2, y2 = rospy.get_param('~x2'), rospy.get_param('~y2')
    x3, y3 = rospy.get_param('~x3'), rospy.get_param('~y3')


    tcp_server_socket = socket.socket( socket.AF_INET, socket.SOCK_STREAM )
    address = (ip, port)
    tcp_server_socket.bind(address)
    tcp_server_socket.listen(128)
    client_socket, clientAddr = tcp_server_socket.accept()

    pub = rospy.Publisher('/move_base_simple/goal', PoseStamped, queue_size=10)


    rate = rospy.Rate(40)
    goal_msg = PoseStamped()
    recv_data_whole = bytes()
    
    while True:
        recv_data = client_socket.recv(struct.calcsize('i')+10)
        if len(recv_data) == 0 :
            client_socket.close()
            tcp_server_socket.close()
            print('finish')
            break
        else:
            recv_data_whole += recv_data

            if recv_data_whole.__len__() == struct.calcsize('i'):
                flag = struct.unpack('i',recv_data_whole)
                if flag == 1:
                    goal_msg.pose.position.x = x1
                    goal_msg.pose.position.y = y1
                    goal_msg.pose.position.z = 1.0
                elif flag == 2:
                    goal_msg.pose.position.x = x2
                    goal_msg.pose.position.y = y2
                    goal_msg.pose.position.z = 1.0
                elif flag == 3:
                    goal_msg.pose.position.x = x3
                    goal_msg.pose.position.y = y3
                    goal_msg.pose.position.z = 0.0

                goal_msg.header.frame_id = 'world'

                pub.publish( goal_msg )
                client_socket.send("ok".encode('utf-8'))
                recv_data_whole = bytes()
                rate.sleep()
                
 
if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass