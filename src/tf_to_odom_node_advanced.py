#!/usr/bin/env python3  

"""
Author: Burak Aksoy
Node: tf_to_odom_node
Description:
    get the transform between two specified TF frames A and B.
    Then also apply an additional pose offset from B to C (defined via param `tf_b_to_c_pose`).
    Publish the final pose (A to C) as a nav_msgs::Odometry message.
    
Parameters:
    - ~odom_topic_name_out (string)
    - ~tf_a_frame_name (string)
    - ~tf_b_frame_name (string)
    - ~tf_b_to_c_pose (dict with 'position' and 'orientation')
    - ~pub_rate (float)

Subscribes to:
    - tf2
Publishes to:
    - nav_msgs::Odometry

Broadcasts to:
    - NONE
"""

import rospy

import numpy as np
import tf2_ros
import tf2_geometry_msgs
import geometry_msgs.msg 
import nav_msgs.msg

# Because of transformations
import tf_conversions
import tf.transformations 


class Tf2Odom():
    def __init__(self):
        rospy.init_node('tf_to_odom', anonymous=True)

        # Topic name to publish
        self.odom_topic_name_out = rospy.get_param("~odom_topic_name_out", "odom_out")

        # Specified tf frame names
        self.tf_a_frame_name = rospy.get_param("~tf_a_frame_name", "tf_a_link")
        self.tf_b_frame_name = rospy.get_param("~tf_b_frame_name", "tf_b_link")

        # Additional offset from frame B to frame C (transform B->C)
        # If not provided, it defaults to identity transform.
        self.tf_b_to_c_pose = rospy.get_param(
            "~tf_b_to_c_pose", 
            {
                "position": {"x": 0.0, "y": 0.0, "z": 0.0},
                "orientation": {"x": 0.0, "y": 0.0, "z": 0.0, "w": 1.0}
            }
        )
        self.b2c_pos = np.array([
            self.tf_b_to_c_pose["position"]["x"],
            self.tf_b_to_c_pose["position"]["y"],
            self.tf_b_to_c_pose["position"]["z"]
        ])
        self.b2c_quat = [
            self.tf_b_to_c_pose["orientation"]["x"],
            self.tf_b_to_c_pose["orientation"]["y"],
            self.tf_b_to_c_pose["orientation"]["z"],
            self.tf_b_to_c_pose["orientation"]["w"]
        ]

        # Publisher
        self.pub_odom = rospy.Publisher(self.odom_topic_name_out, nav_msgs.msg.Odometry, queue_size=1)

        # TF2 listener
        self.tfBuffer = tf2_ros.Buffer()
        self.tfListener = tf2_ros.TransformListener(self.tfBuffer)

        # Transform variables
        self.T_a2b = None
        self.is_ok_tf = False

        # Publish rate
        self.pub_rate = rospy.get_param("~pub_rate", 100.0)
        self.rate = rospy.Rate(self.pub_rate)
        self.expected_duration = 1.00/self.pub_rate # seconds per cycle

        # Start control
        rospy.Timer(rospy.Duration(self.expected_duration), self.transformer)


    def transformer(self, event=None):
        # Find the transform between the specified frames A and B
        self.is_ok_tf = self.look_tfs()

        if not self.is_ok_tf:
            # If we cannot find the transformation, do not publish
            rospy.logdebug("No valid TF found from %s to %s yet." 
                           % (self.tf_a_frame_name, self.tf_b_frame_name))
        else:
            # Apply the extra offset from B->C and publish
            self.transform_and_publish()

    def look_tfs(self):
        try:
            # Returns geometry_msgs/TransformStamped
            self.T_a2b = self.tfBuffer.lookup_transform(
                self.tf_a_frame_name,
                self.tf_b_frame_name,
                rospy.Time()
            ) 
            return True
        except (tf2_ros.LookupException, 
                tf2_ros.ConnectivityException, 
                tf2_ros.ExtrapolationException):
            rospy.logwarn_once(
                'odom_transformer: Waiting to find the transformation from %s to %s' 
                % (self.tf_a_frame_name, self.tf_b_frame_name)
            ) 
            return False

    def transform_and_publish(self):
        # Extract the translation from A->B
        P_a2b_in_a_x = self.T_a2b.transform.translation.x
        P_a2b_in_a_y = self.T_a2b.transform.translation.y
        P_a2b_in_a_z = self.T_a2b.transform.translation.z
        P_a2b_in_a = np.array([P_a2b_in_a_x, P_a2b_in_a_y, P_a2b_in_a_z])

        # Extract the quaternion from A->B
        qw_cur = self.T_a2b.transform.rotation.w
        qx_cur = self.T_a2b.transform.rotation.x
        qy_cur = self.T_a2b.transform.rotation.y
        qz_cur = self.T_a2b.transform.rotation.z
        q_a2b = [qx_cur, qy_cur, qz_cur, qw_cur]

        # Build the 4x4 homogeneous transform for A->B
        T_a2b_mat = tf.transformations.quaternion_matrix(q_a2b)
        T_a2b_mat[0:3, 3] = P_a2b_in_a

        # Build the 4x4 homogeneous transform for B->C from param
        T_b2c_mat = tf.transformations.quaternion_matrix(self.b2c_quat)
        T_b2c_mat[0:3, 3] = self.b2c_pos

        # Compute the final transform A->C
        # T_a2c = T_a2b * T_b2c
        T_a2c_mat = np.dot(T_a2b_mat, T_b2c_mat)

        # Extract the position and orientation from A->C
        P_a2c_in_a = T_a2c_mat[0:3, 3]
        q_a2c = tf.transformations.quaternion_from_matrix(T_a2c_mat)

        # Finally publish the odometry from A->C
        self.publish_odom(P_a2c_in_a, q_a2c)

    def publish_odom(self, p_b, q_b):
        """
        Inputs:
        - p_b: final position vector [3x1] of frame C in A
        - q_b: final orientation quaternion [4x1] of frame C in A
        """
        msg = nav_msgs.msg.Odometry()

        # We'll treat 'frame_id' as the A frame
        msg.header.frame_id = self.tf_a_frame_name
        msg.header.stamp = rospy.Time.now()
        # We can denote child_frame_id as the 'C' frame
        # for clarity that it's the final offset pose
        msg.child_frame_id = self.tf_b_frame_name + "_offset"

        msg.pose.pose.position.x = p_b[0]
        msg.pose.pose.position.y = p_b[1]
        msg.pose.pose.position.z = p_b[2]
        msg.pose.pose.orientation.x = q_b[0]
        msg.pose.pose.orientation.y = q_b[1]
        msg.pose.pose.orientation.z = q_b[2]
        msg.pose.pose.orientation.w = q_b[3]

        # velocities 0 for example
        msg.twist.twist.linear.x = 0.0
        msg.twist.twist.linear.y = 0.0
        msg.twist.twist.linear.z = 0.0
        msg.twist.twist.angular.x = 0.0
        msg.twist.twist.angular.y = 0.0
        msg.twist.twist.angular.z = 0.0

        self.pub_odom.publish(msg)


if __name__ == '__main__':
    tf2Odom = Tf2Odom()
    rospy.spin()