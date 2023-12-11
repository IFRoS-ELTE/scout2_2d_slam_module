#!/usr/bin/env python3

# Import necessary ROS and TF2 libraries
import rospy
import tf2_ros
from geometry_msgs.msg import TransformStamped
from tf2_msgs.msg import TFMessage

# Define a class for the ZDriftFixer node
class ZDriftFixer:
    def __init__(self):
        # Initialize the ROS node with the name 'z_drift_fixer'
        rospy.init_node('z_drift_fixer_node')

        # Create a TF (Transform) broadcaster
        self.tf_broadcaster = tf2_ros.TransformBroadcaster()

        # Get parameters from the ROS parameter server
        self.original_frame_id = rospy.get_param("~original_frame_id")
        self.corrected_frame_id = rospy.get_param("~corrected_frame_id")
        self.z_translation = rospy.get_param("~z_translation")

        # Subscribe to the '/tf' topic and set the callback function
        rospy.Subscriber("/tf", TFMessage, self.transform_callback)

    # Callback function to handle incoming transform messages
    def transform_callback(self, transform_msg):
        # Loop through each transform in the TFMessage
        for transform in transform_msg.transforms:
            # Check if the child frame ID matches the specified original frame ID
            if transform.child_frame_id == self.original_frame_id:
                # Create a new TransformStamped message
                new_transform = TransformStamped()

                # Copy header and child frame ID from the original transform
                new_transform.header = transform.header
                new_transform.child_frame_id = self.corrected_frame_id

                # Copy the original transform data
                new_transform.transform = transform.transform

                # Modify the z-translation component of the transform
                new_transform.transform.translation.z = self.z_translation

                # Broadcast the corrected transform
                self.tf_broadcaster.sendTransform(new_transform)

    # Method to keep the node running
    def run(self):
        rospy.spin()

# Entry point of the script
if __name__ == '__main__':
    # Create an instance of the ZDriftFixer class
    node = ZDriftFixer()

    # Run the node
    node.run()
