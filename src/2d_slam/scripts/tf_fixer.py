#!/usr/bin/env python3
import rospy
import tf2_ros
from geometry_msgs.msg import TransformStamped
from tf2_msgs.msg import TFMessage

class ZDriftFixer:
    def __init__(self):
        rospy.init_node('z_drift_fixer')
        self.tf_broadcaster = tf2_ros.TransformBroadcaster()
        self.original_frame_id = rospy.get_param("~original_frame_id")
        self.corrected_frame_id = rospy.get_param("~corrected_frame_id")
        self.z_translation = rospy.get_param("~z_translation")
        rospy.Subscriber("/tf", TFMessage, self.transform_callback)

    def transform_callback(self, transform_msg):
        for transform in transform_msg.transforms:
            if transform.child_frame_id == self.original_frame_id:
                new_transform = TransformStamped()
                new_transform.header = transform.header
                new_transform.child_frame_id = self.corrected_frame_id
                new_transform.transform = transform.transform
                new_transform.transform.translation.z = self.z_translation
                self.tf_broadcaster.sendTransform(new_transform)

    def run(self):
        rospy.spin()

if __name__ == '__main__':
    node = ZDriftFixer()
    node.run()