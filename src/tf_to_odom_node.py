#!/usr/bin/env python3  

"""
Author: Burak Aksoy
Node: tf_to_odom_node
Description:
    get the transform btw. two specified tf frames A and B.
    Publish the pose from A to B as a nav_msgs::Odometry message.
    
Parameters:
    - TODO
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

# # Because of transformations
import tf_conversions
#  tf_conversions.transformations.euler_from_quaternion(Q_eg)
import tf.transformations 

class Tf2Odom():
    def __init__(self):
        rospy.init_node('tf_to_odom', anonymous=True)

        # Topic name to publish
        self.odom_topic_name_out = rospy.get_param("~odom_topic_name_out", "odom_out")

        # Specified tf frame names
        self.tf_a_frame_name = rospy.get_param("~tf_a_frame_name", "tf_a_link")
        self.tf_b_frame_name = rospy.get_param("~tf_b_frame_name", "tf_b_link")

        # Publisher
        self.pub_odom = rospy.Publisher(self.odom_topic_name_out, nav_msgs.msg.Odometry, queue_size=1)

        # TF2 listener
        self.tfBuffer = tf2_ros.Buffer()
        self.tfListener = tf2_ros.TransformListener(self.tfBuffer)

        self.T_a2b = None
        self.is_ok_tf = False

        # To store the odometry msg on frame a
        self.odom_a = nav_msgs.msg.Odometry()

        # Publish rate
        self.pub_rate = rospy.get_param("~pub_rate", 100.0)
        self.rate = rospy.Rate(self.pub_rate)
        self.expected_duration = 1.00/self.pub_rate # seconds per cycle

        # Start control
        rospy.Timer(rospy.Duration(self.expected_duration), self.transformer)


    def transformer(self, event=None):
        # Find the transform between the specified joint and the end effector
        self.is_ok_tf = self.look_tfs()

        if not self.is_ok_tf:
            # Do not publish odom since the transformation could not found
            pass
            # self.publish_odom([0.,0.,0.],[0.,0.,0.,1.],[0.,0.,0.],[0.,0.,0.])
        else:
            # Publish the command to move the end effector to the body joint
            self.transform_and_publish()

    def look_tfs(self):
        try:
            # returns type geometry_msgs.msg.TransformStamped
            self.T_a2b = self.tfBuffer.lookup_transform(self.tf_a_frame_name, self.tf_b_frame_name,  rospy.Time()) # in base frame 
        
            return True
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            # Put a warning which says that the transformation could not found
            rospy.logwarn_once('odom_transformer: Waiting to find the transformation from %s to %s' 
                            % (self.tf_a_frame_name, self.tf_b_frame_name)) 
            return False

    def transform_and_publish(self):
        # Get P_a2b_in_a and R_a2b (and R_b2a) from TF msgs.
        P_a2b_in_a_x = self.T_a2b.transform.translation.x
        P_a2b_in_a_y = self.T_a2b.transform.translation.y
        P_a2b_in_a_z = self.T_a2b.transform.translation.z
        P_a2b_in_a = np.array([P_a2b_in_a_x,P_a2b_in_a_y,P_a2b_in_a_z])

        qw_cur = self.T_a2b.transform.rotation.w # Scalar part of quaternion
        qx_cur = self.T_a2b.transform.rotation.x
        qy_cur = self.T_a2b.transform.rotation.y
        qz_cur = self.T_a2b.transform.rotation.z
        q_a2b = [qx_cur,qy_cur,qz_cur, qw_cur]

        # R_a2b = tf.transformations.quaternion_matrix(q_a2b) # 4x4
        # R_a2b = R_a2b[:3,:3]
        # R_b2a = R_a2b.T 
        

        self.publish_odom(P_a2b_in_a,q_a2b)

    def publish_odom(self, p_b, q_b):
        """
        Inputs:
        - p_b: position vector of frame b [3x1]
        - q_b: orientation quaternion vector of frame b  [4x1]
        """
        msg = nav_msgs.msg.Odometry()

        msg.header.frame_id = self.tf_a_frame_name
        msg.header.stamp = rospy.Time.now()
        msg.child_frame_id = self.tf_b_frame_name

        msg.pose.pose.position.x = p_b[0]
        msg.pose.pose.position.y = p_b[1]
        msg.pose.pose.position.z = p_b[2]
        msg.pose.pose.orientation.x = q_b[0]
        msg.pose.pose.orientation.y = q_b[1]
        msg.pose.pose.orientation.z = q_b[2]
        msg.pose.pose.orientation.w = q_b[3]

        # msg.pose.covariance = 

        msg.twist.twist.linear.x = 0.0
        msg.twist.twist.linear.y = 0.0
        msg.twist.twist.linear.z = 0.0
        msg.twist.twist.angular.x = 0.0
        msg.twist.twist.angular.y = 0.0
        msg.twist.twist.angular.z = 0.0

        # msg.twist.covariance = 

        self.pub_odom.publish(msg)


if __name__ == '__main__':
    tf2Odom = Tf2Odom()
    rospy.spin()