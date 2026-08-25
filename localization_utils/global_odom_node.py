#!/usr/bin/env python3
"""Global Odometry Node: compose map→camera_init (TF) with camera_init→body (odometry).

Subscribes to /Odometry (camera_init→body, ~10 Hz) and looks up the
map→camera_init transform from the TF tree.  Publishes /global_odom
(map→body) at the same rate as the input odometry.

T_map_body = T_map_camera_init @ T_camera_init_body
"""
from __future__ import annotations

import numpy as np
import rclpy
from rclpy.time import Time
from rclpy.node import Node
from geometry_msgs.msg import Quaternion
from nav_msgs.msg import Odometry
import tf2_ros


def quat_to_rotmat(q):
    """Convert a quaternion to a 3x3 rotation matrix.

    Accepts either a ROS ``Quaternion`` message (with ``x/y/z/w`` attributes)
    or any 4-element sequence ``(x, y, z, w)``.
    """
    if hasattr(q, 'x'):
        x, y, z, w = q.x, q.y, q.z, q.w
    else:
        x, y, z, w = q
    return np.array([
        [1 - 2*y*y - 2*z*z,     2*x*y - 2*z*w,     2*x*z + 2*y*w],
        [    2*x*y + 2*z*w, 1 - 2*x*x - 2*z*z,     2*y*z - 2*x*w],
        [    2*x*z - 2*y*w,     2*y*z + 2*x*w, 1 - 2*x*x - 2*y*y],
    ])


def rotmat_to_quat(R):
    """Convert a 3x3 rotation matrix to a quaternion (x, y, z, w)."""
    R = np.asarray(R, dtype=np.float64)
    tr = R[0, 0] + R[1, 1] + R[2, 2]
    if tr > 0.0:
        s = np.sqrt(tr + 1.0) * 2.0
        w = 0.25 * s
        x = (R[2, 1] - R[1, 2]) / s
        y = (R[0, 2] - R[2, 0]) / s
        z = (R[1, 0] - R[0, 1]) / s
    elif R[0, 0] > R[1, 1] and R[0, 0] > R[2, 2]:
        s = np.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2]) * 2.0
        w = (R[2, 1] - R[1, 2]) / s
        x = 0.25 * s
        y = (R[0, 1] + R[1, 0]) / s
        z = (R[0, 2] + R[2, 0]) / s
    elif R[1, 1] > R[2, 2]:
        s = np.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2]) * 2.0
        w = (R[0, 2] - R[2, 0]) / s
        x = (R[0, 1] + R[1, 0]) / s
        y = 0.25 * s
        z = (R[1, 2] + R[2, 1]) / s
    else:
        s = np.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1]) * 2.0
        w = (R[1, 0] - R[0, 1]) / s
        x = (R[0, 2] + R[2, 0]) / s
        y = (R[1, 2] + R[2, 1]) / s
        z = 0.25 * s
    return (float(x), float(y), float(z), float(w))


def transform_to_matrix(transform):
    """Convert a geometry_msgs/Transform (or TransformStamped) to a 4x4 matrix."""
    t = transform.transform if hasattr(transform, 'transform') else transform
    tr = t.translation
    T = np.eye(4, dtype=np.float64)
    T[:3, :3] = quat_to_rotmat(t.rotation)
    T[:3, 3] = [tr.x, tr.y, tr.z]
    return T


class GlobalOdomNode(Node):
    """Compose map→camera_init (TF) with camera_init→body (odometry) → map→body."""

    def __init__(self):
        super().__init__('global_odom_node')

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.sub = self.create_subscription(
            Odometry, '/Odometry', self._odom_callback, 10
        )

        self.pub = self.create_publisher(
            Odometry, '/global_odom', 10
        )

        self.get_logger().info(
            'global_odom_node started: /Odometry (camera_init→body) + TF '
            '(map→camera_init) → /global_odom (map→body)'
        )

    def _odom_callback(self, msg: Odometry):
        """Compose the global odometry and publish."""
        try:
            t_stamped = self.tf_buffer.lookup_transform(
                'map',
                'camera_init',
                Time()
            )
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException,
                tf2_ros.ExtrapolationException) as e:
            self.get_logger().warn(f'TF lookup failed: {e}')
            return

        # T_map_camera_init from TF
        T_map_ci = transform_to_matrix(t_stamped)

        # T_camera_init_body from odometry pose
        T_ci_body = np.eye(4, dtype=np.float64)
        T_ci_body[:3, :3] = quat_to_rotmat(msg.pose.pose.orientation)
        T_ci_body[:3, 3] = [
            msg.pose.pose.position.x,
            msg.pose.pose.position.y,
            msg.pose.pose.position.z,
        ]

        # T_map_body = T_map_camera_init @ T_camera_init_body
        T_map_body = T_map_ci @ T_ci_body

        # Build output message
        out = Odometry()
        out.header.stamp = msg.header.stamp
        out.header.frame_id = 'map'
        out.child_frame_id = 'body'

        out.pose.pose.position.x = T_map_body[0, 3]
        out.pose.pose.position.y = T_map_body[1, 3]
        out.pose.pose.position.z = T_map_body[2, 3]

        qx, qy, qz, qw = rotmat_to_quat(T_map_body[:3, :3])
        out.pose.pose.orientation = Quaternion(x=qx, y=qy, z=qz, w=qw)

        # Pass through twist (also transformed would be ideal, but spec says
        # "generate using latest transform from TF is enough")
        out.twist = msg.twist

        self.pub.publish(out)


def main(args=None):
    rclpy.init(args=args)
    node = GlobalOdomNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
