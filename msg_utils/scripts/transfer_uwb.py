import rospy
from msg_utils.msg import uwb
from nlink_parser.msg import LinktrackNodeframe2
from nlink_parser.msg import LinktrackNode2
from std_msgs.msg import Header


global transfer_uwb_pub

def callback( uwb_msg: LinktrackNodeframe2 ):
    transfer_uwb_msg = uwb()
    node_list: list[LinktrackNode2] = uwb_msg.nodes
    local_time = uwb_msg.local_time
    system_time = uwb_msg.system_time

    msg_header = Header()

    msg_header.stamp = rospy.Time.now()
    transfer_uwb_msg.header = msg_header

    dest_id_list: list[int] = []
    dist_list: list[float] = []
    for i in range( len( node_list ) ):
        node = node_list[i]
        dest_id_list.append( node.id )
        dist_list.append( node.dis )
    transfer_uwb_msg.dest_id = dest_id_list
    transfer_uwb_msg.dist = dist_list

    transfer_uwb_pub.publish( transfer_uwb_msg )



def listener():
    rospy.init_node('uwb_listener', anonymous=True)

    rospy.Subscriber("/nlink_linktrack_nodeframe2", LinktrackNodeframe2, callback)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == "__main__":
    ag_n = rospy.get_param('/uwb_listener/ag_n')
    transfer_uwb_pub = rospy.Publisher('/uwb' + str(ag_n), uwb, queue_size=1)
    listener()