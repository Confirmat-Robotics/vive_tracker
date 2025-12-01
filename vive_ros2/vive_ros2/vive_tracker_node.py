"""ROS 2 node that republishes Vive tracker data as Odometry."""

from queue import Queue
from threading import Event, Thread

import rclpy
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from tf2_ros import TransformBroadcaster

from .vive_tracker_client import ViveTrackerClient


class ViveTrackerNode(Node):
    def __init__(self) -> None:
        super().__init__("vive_tracker_node")
        self.declare_parameter("host_ip", "192.168.50.171")
        self.declare_parameter("host_port", 8000)
        self.declare_parameter("tracker_name", "tracker_1")
        self.declare_parameter("topic", "")
        self.declare_parameter("link_name", "odom")
        self.declare_parameter("child_link_name", "tracker_link")
        self.declare_parameter("publish_tf", True)

        (
            self.host_ip,
            self.host_port,
            self.tracker_name,
            self.link_name,
            self.child_link_name,
            self.topic,
            self.publish_tf,
        ) = self.get_parameters(
            ["host_ip", "host_port", "tracker_name", "link_name", "child_link_name", "topic", "publish_tf"]
        )

        topic = self.topic.get_parameter_value().string_value
        topic_name = (
            f"{self.tracker_name.get_parameter_value().string_value}/odom" if topic == "" else topic
        )
        self.odom_pub = self.create_publisher(Odometry, topic_name, qos_profile=qos_profile_sensor_data)
        
        # Create TF broadcaster if enabled
        self.tf_broadcaster = None
        if self.publish_tf.get_parameter_value().bool_value:
            self.tf_broadcaster = TransformBroadcaster(self)

        client = ViveTrackerClient(
            host=self.host_ip.get_parameter_value().string_value,
            port=self.host_port.get_parameter_value().integer_value,
            tracker_name=self.tracker_name.get_parameter_value().string_value,
            should_record=False,
        )

        self.message_queue = Queue()
        self.kill_thread = Event()
        self.client_thread = Thread(target=client.run_threaded, args=(self.message_queue, self.kill_thread))

        try:
            self.client_thread.start()

            while rclpy.ok():
                msg = self.message_queue.get()

                timestamp = self.get_clock().now().to_msg()
                frame_id = self.link_name.get_parameter_value().string_value
                child_frame_id = self.child_link_name.get_parameter_value().string_value

                # Publish TF transform if enabled
                if self.tf_broadcaster is not None:
                    t = TransformStamped()
                    t.header.stamp = timestamp
                    t.header.frame_id = frame_id
                    t.child_frame_id = child_frame_id
                    
                    t.transform.translation.x = msg.x
                    t.transform.translation.y = msg.y
                    t.transform.translation.z = msg.z
                    
                    t.transform.rotation.x = msg.qx
                    t.transform.rotation.y = msg.qy
                    t.transform.rotation.z = msg.qz
                    t.transform.rotation.w = msg.qw
                    
                    self.tf_broadcaster.sendTransform(t)

                # Publish Odometry message
                odom_msg = Odometry()
                odom_msg.header.stamp = timestamp
                odom_msg.header.frame_id = frame_id

                odom_msg.child_frame_id = child_frame_id

                odom_msg.pose.pose.position.x = msg.x
                odom_msg.pose.pose.position.y = msg.y
                odom_msg.pose.pose.position.z = msg.z

                odom_msg.pose.pose.orientation.x = msg.qx
                odom_msg.pose.pose.orientation.y = msg.qy
                odom_msg.pose.pose.orientation.z = msg.qz
                odom_msg.pose.pose.orientation.w = msg.qw

                odom_msg.twist.twist.linear.x = msg.vel_x
                odom_msg.twist.twist.linear.y = msg.vel_y
                odom_msg.twist.twist.linear.z = msg.vel_z

                odom_msg.twist.twist.angular.x = msg.p
                odom_msg.twist.twist.angular.y = msg.q
                odom_msg.twist.twist.angular.z = msg.r

                self.odom_pub.publish(odom_msg)

        finally:
            self.kill_thread.set()
            self.client_thread.join()


def main(args=None) -> None:
    rclpy.init(args=args)
    vive_tracker_node = ViveTrackerNode()
    rclpy.spin(vive_tracker_node)
    vive_tracker_node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
