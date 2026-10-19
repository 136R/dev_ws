#!/usr/bin/env python3
"""建图回放用：/odometry/filtered → TF odom→base_footprint（录包时 EKF transform_time_offset=0，两者同一时刻同一值）。由 map_group.sh 起。"""
import rclpy
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from rclpy.parameter import Parameter
from tf2_ros import TransformBroadcaster
rclpy.init()
n = rclpy.create_node('odom_tf_relay', parameter_overrides=[Parameter('use_sim_time', value=True)])
br = TransformBroadcaster(n)
def cb(m):
    t = TransformStamped()
    t.header = m.header
    t.child_frame_id = m.child_frame_id
    t.transform.translation.x = m.pose.pose.position.x
    t.transform.translation.y = m.pose.pose.position.y
    t.transform.translation.z = m.pose.pose.position.z
    t.transform.rotation = m.pose.pose.orientation
    br.sendTransform(t)
n.create_subscription(Odometry, '/odometry/filtered', cb, 50)
rclpy.spin(n)
