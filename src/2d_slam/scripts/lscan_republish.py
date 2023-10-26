#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import LaserScan

def laserscan_callback(msg, new_frame_id):
    # Modify the frame ID
    msg.header.frame_id = new_frame_id

    # Publish the modified LaserScan message
    pub.publish(msg)

if __name__ == '__main__':
    rospy.init_node('modify_laserscan_frame_id_node')

    # Input and Output topics
    input_topic = rospy.get_param('~input_topic', '/input_scan')
    output_topic = rospy.get_param('~output_topic', '/output_scan')

    # Parameter for the desired new frame ID
    new_frame_id = rospy.get_param('~new_frame_id', 'new_frame_id')

    # Initialize a publisher
    pub = rospy.Publisher(output_topic, LaserScan, queue_size=10)

    # Subscribe to the original LaserScan topic
    rospy.Subscriber(input_topic, LaserScan, laserscan_callback, new_frame_id)

    rospy.spin()
