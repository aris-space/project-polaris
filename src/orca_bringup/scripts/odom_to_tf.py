#!/usr/bin/env python3

import json
import time
from urllib import request

import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped


class OdomToTfNode(Node):
    def __init__(self) -> None:
        super().__init__('odom_to_tf')
        self.declare_parameter('odom_topic', '/odom')
        self.declare_parameter('parent_frame_id', 'odom')
        self.declare_parameter('child_frame_id', 'base_link')

        odom_topic = self.get_parameter('odom_topic').get_parameter_value().string_value
        self.parent_frame_id = self.get_parameter('parent_frame_id').get_parameter_value().string_value
        self.child_frame_id = self.get_parameter('child_frame_id').get_parameter_value().string_value

        self._odom_count = 0
        self._last_odom_stamp_sec = 0.0
        self._health_timer = self.create_timer(2.0, self._health_log)
        self.tf_broadcaster = TransformBroadcaster(self)
        # Match ros_gz_bridge / Gazebo-style odometry (best-effort); default QoS is reliable and will not connect.
        self.odom_sub = self.create_subscription(
            Odometry,
            odom_topic,
            self._on_odom,
            qos_profile_sensor_data,
        )
        # #region agent log
        self._debug_log(
            run_id='run-initial',
            hypothesis_id='H2',
            location='orca_bringup/scripts/odom_to_tf.py:__init__',
            message='odom_to_tf initialized',
            data={
                'odom_topic': odom_topic,
                'parent_frame_id': self.parent_frame_id,
                'child_frame_id': self.child_frame_id,
            },
        )
        # #endregion

    def _on_odom(self, msg: Odometry) -> None:
        self._odom_count += 1
        self._last_odom_stamp_sec = float(msg.header.stamp.sec) + float(msg.header.stamp.nanosec) * 1e-9
        if self._odom_count <= 5 or self._odom_count % 100 == 0:
            # #region agent log
            self._debug_log(
                run_id='run-initial',
                hypothesis_id='H1',
                location='orca_bringup/scripts/odom_to_tf.py:_on_odom',
                message='received odom sample',
                data={
                    'count': self._odom_count,
                    'msg_frame_id': msg.header.frame_id,
                    'configured_parent': self.parent_frame_id,
                    'configured_child': self.child_frame_id,
                    'x': msg.pose.pose.position.x,
                    'y': msg.pose.pose.position.y,
                    'z': msg.pose.pose.position.z,
                },
            )
            # #endregion
        tf_msg = TransformStamped()
        tf_msg.header.stamp = msg.header.stamp
        tf_msg.header.frame_id = self.parent_frame_id
        tf_msg.child_frame_id = self.child_frame_id

        tf_msg.transform.translation.x = msg.pose.pose.position.x
        tf_msg.transform.translation.y = msg.pose.pose.position.y
        tf_msg.transform.translation.z = msg.pose.pose.position.z
        tf_msg.transform.rotation = msg.pose.pose.orientation

        self.tf_broadcaster.sendTransform(tf_msg)

    def _health_log(self) -> None:
        now_sec = self.get_clock().now().nanoseconds / 1e9
        age = None
        if self._last_odom_stamp_sec > 0.0:
            age = now_sec - self._last_odom_stamp_sec
        # #region agent log
        self._debug_log(
            run_id='run-initial',
            hypothesis_id='H3',
            location='orca_bringup/scripts/odom_to_tf.py:_health_log',
            message='odom_to_tf health',
            data={
                'odom_count': self._odom_count,
                'last_odom_age_sec': age,
            },
        )
        # #endregion

    def _debug_log(self, run_id: str, hypothesis_id: str, location: str, message: str, data: dict) -> None:
        try:
            payload = {
                'sessionId': '04c242',
                'runId': run_id,
                'hypothesisId': hypothesis_id,
                'location': location,
                'message': message,
                'data': data,
                'timestamp': int(time.time() * 1000),
            }
            body = json.dumps(payload, separators=(',', ':')).encode('utf-8')
            req = request.Request(
                'http://127.0.0.1:7854/ingest/8ee5efed-e8d8-4d93-a980-c2f0ded3775f',
                data=body,
                headers={
                    'Content-Type': 'application/json',
                    'X-Debug-Session-Id': '04c242',
                },
                method='POST',
            )
            request.urlopen(req, timeout=0.5).read()
        except Exception:
            pass


def main() -> None:
    rclpy.init()
    node = OdomToTfNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
