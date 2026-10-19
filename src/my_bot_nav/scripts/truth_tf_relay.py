#!/usr/bin/env python3
"""建图回放对照用：真值 /gz/model/my_bot/pose 当 odom→base_footprint（odom = world），让 slam 用完美位姿画图。由 map_group.sh（RELAY=truth）起。"""
import rclpy
from geometry_msgs.msg import PoseStamped, TransformStamped
from rclpy.parameter import Parameter
from tf2_ros import TransformBroadcaster
rclpy.init()
n = rclpy.create_node('truth_tf_relay', parameter_overrides=[Parameter('use_sim_time', value=True)])
br = TransformBroadcaster(n)
def cb(m):
    t = TransformStamped()
    t.header.stamp = m.header.stamp
    t.header.frame_id = 'odom'
    t.child_frame_id = 'base_footprint'
    t.transform.translation.x = m.pose.position.x
    t.transform.translation.y = m.pose.position.y
    t.transform.rotation = m.pose.orientation
    br.sendTransform(t)
n.create_subscription(PoseStamped, '/gz/model/my_bot/pose', cb, 50)
rclpy.spin(n)
