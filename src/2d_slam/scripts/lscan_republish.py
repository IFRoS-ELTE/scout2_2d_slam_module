#!/usr/bin/env python3

# Import ROS Python library
import rospy

# Import LaserScan message type
from sensor_msgs.msg import LaserScan

# Callback function to modify the frame ID of LaserScan message
def laserscan_callback(msg, new_frame_id):
    # Modify the frame ID
    msg.header.frame_id = new_frame_id

    # Publish the modified LaserScan message
    pub.publish(msg)

# Entry point of the script
if __name__ == '__main__':
    # Initialize the ROS node
    rospy.init_node('modify_laserscan_frame_id_node')

    # Input and Output topics
    input_topic = rospy.get_param('~input_topic', '/input_scan')
    output_topic = rospy.get_param('~output_topic', '/output_scan')

    # Parameter for the desired new frame ID
    new_frame_id = rospy.get_param('~new_frame_id', 'new_frame_id')

    # Initialize a publisher with a queue size of 10
    pub = rospy.Publisher(output_topic, LaserScan, queue_size=10)

    # Subscribe to the original LaserScan topic and set the callback function
    rospy.Subscriber(input_topic, LaserScan, laserscan_callback, new_frame_id)

    # Keep the node running
    rospy.spin()
