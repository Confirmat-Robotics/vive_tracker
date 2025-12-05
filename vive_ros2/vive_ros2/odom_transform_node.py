"""ROS 2 node to apply coordinate transform to odometry messages.

This node corrects odometry messages from older vive tracker recordings
that didn't have the OpenVR to ROS coordinate transform applied.

OpenVR tracking space (right, up, -forward) to ROS FLU (forward, left, up)
OpenVR basis: ex = +X (right), ey = +Y (up), ez = +Z (backward) so forward = -Z.
Desired ROS basis in OpenVR coords:
  x_ros (forward) = -Z_ovr -> [0, 0, -1]
  y_ros (left)    = -X_ovr -> [-1, 0, 0]
  z_ros (up)      =  Y_ovr -> [0, 1, 0]
"""

import numpy as np
import rclpy
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from scipy.spatial.transform import Rotation
from tf2_ros import TransformBroadcaster


# OpenVR to ROS coordinate transform
ROT_OVR_TO_ROS = Rotation.from_matrix([
    [0, 0, -1],
    [-1, 0, 0],
    [0, 1, 0],
])


class OdomTransformNode(Node):
    def __init__(self) -> None:
        super().__init__("odom_transform_node")
        
        # Declare parameters
        self.declare_parameter("input_topic", "/input/odom")
        self.declare_parameter("output_topic", "/output/odom")
        self.declare_parameter("publish_tf", True)
        
        # Get parameters
        input_topic = self.get_parameter("input_topic").get_parameter_value().string_value
        output_topic = self.get_parameter("output_topic").get_parameter_value().string_value
        publish_tf = self.get_parameter("publish_tf").get_parameter_value().bool_value
        
        # Create subscriber
        self.odom_sub = self.create_subscription(
            Odometry,
            input_topic,
            self.odom_callback,
            qos_profile_sensor_data
        )
        
        # Create publisher
        self.odom_pub = self.create_publisher(
            Odometry,
            output_topic,
            qos_profile_sensor_data
        )
        
        # Create TF broadcaster if enabled
        self.tf_broadcaster = None
        if publish_tf:
            self.tf_broadcaster = TransformBroadcaster(self)
        
        self.get_logger().info(f"Odometry transform node started")
        self.get_logger().info(f"  Input topic: {input_topic}")
        self.get_logger().info(f"  Output topic: {output_topic}")
        self.get_logger().info(f"  Publish TF: {publish_tf}")
    
    def odom_callback(self, msg: Odometry) -> None:
        """Transform odometry message from OpenVR to ROS coordinates."""
        
        # Extract position from input (in OpenVR coordinates)
        x_ovr = msg.pose.pose.position.x
        y_ovr = msg.pose.pose.position.y
        z_ovr = msg.pose.pose.position.z
        
        # Extract orientation from input (in OpenVR coordinates)
        qx_ovr = msg.pose.pose.orientation.x
        qy_ovr = msg.pose.pose.orientation.y
        qz_ovr = msg.pose.pose.orientation.z
        qw_ovr = msg.pose.pose.orientation.w
        
        # Transform position to ROS coordinates
        pos_ovr = np.array([x_ovr, y_ovr, z_ovr])
        pos_ros = ROT_OVR_TO_ROS.apply(pos_ovr)
        x_ros, y_ros, z_ros = pos_ros
        
        # Transform orientation to ROS coordinates
        rot_ovr = Rotation.from_quat([qx_ovr, qy_ovr, qz_ovr, qw_ovr])
        rot_ros = ROT_OVR_TO_ROS * rot_ovr
        qx_ros, qy_ros, qz_ros, qw_ros = rot_ros.as_quat()
        
        # Transform linear velocity to ROS coordinates
        vel_x_ovr = msg.twist.twist.linear.x
        vel_y_ovr = msg.twist.twist.linear.y
        vel_z_ovr = msg.twist.twist.linear.z
        vel_ovr = np.array([vel_x_ovr, vel_y_ovr, vel_z_ovr])
        vel_ros = ROT_OVR_TO_ROS.apply(vel_ovr)
        vel_x_ros, vel_y_ros, vel_z_ros = vel_ros
        
        # Transform angular velocity to ROS coordinates
        ang_x_ovr = msg.twist.twist.angular.x
        ang_y_ovr = msg.twist.twist.angular.y
        ang_z_ovr = msg.twist.twist.angular.z
        ang_ovr = np.array([ang_x_ovr, ang_y_ovr, ang_z_ovr])
        ang_ros = ROT_OVR_TO_ROS.apply(ang_ovr)
        ang_x_ros, ang_y_ros, ang_z_ros = ang_ros
        
        # Create output message
        out_msg = Odometry()
        out_msg.header = msg.header
        out_msg.child_frame_id = msg.child_frame_id
        
        # Set transformed position
        out_msg.pose.pose.position.x = x_ros
        out_msg.pose.pose.position.y = y_ros
        out_msg.pose.pose.position.z = z_ros
        
        # Set transformed orientation
        out_msg.pose.pose.orientation.x = qx_ros
        out_msg.pose.pose.orientation.y = qy_ros
        out_msg.pose.pose.orientation.z = qz_ros
        out_msg.pose.pose.orientation.w = qw_ros
        
        # Set transformed velocities
        out_msg.twist.twist.linear.x = vel_x_ros
        out_msg.twist.twist.linear.y = vel_y_ros
        out_msg.twist.twist.linear.z = vel_z_ros
        
        out_msg.twist.twist.angular.x = ang_x_ros
        out_msg.twist.twist.angular.y = ang_y_ros
        out_msg.twist.twist.angular.z = ang_z_ros
        
        # Copy covariances (these are frame-dependent, but for simplicity we copy them)
        out_msg.pose.covariance = msg.pose.covariance
        out_msg.twist.covariance = msg.twist.covariance
        
        # Publish transformed odometry
        self.odom_pub.publish(out_msg)
        
        # Publish TF if enabled
        if self.tf_broadcaster is not None:
            t = TransformStamped()
            t.header = msg.header
            t.child_frame_id = msg.child_frame_id
            
            t.transform.translation.x = x_ros
            t.transform.translation.y = y_ros
            t.transform.translation.z = z_ros
            
            t.transform.rotation.x = qx_ros
            t.transform.rotation.y = qy_ros
            t.transform.rotation.z = qz_ros
            t.transform.rotation.w = qw_ros
            
            self.tf_broadcaster.sendTransform(t)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = OdomTransformNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
