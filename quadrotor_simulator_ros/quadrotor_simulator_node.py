import os

import rclpy
from rclpy.node import Node
from rclpy.time import Time, Duration
from nav_msgs.msg import Odometry
from tf2_msgs.msg import TFMessage
from geometry_msgs.msg import TransformStamped, Pose, Vector3, Transform
import ament_index_python

import numpy as np
from cprint import cprint

from quadrotor_simulator_py.quadrotor_model import QuadrotorModel
from quadrotor_msgs.msg import RPMCommand
import ros2_numpy as rnp

CONVERSION_CONSTANT = 10 ** 9


class QuadrotorSimulatorNode(Node):
    _sim_time = 0.0
    _sim_dt = 0.001

    _fixed_frame_id = 'world'
    _base_frame_id = 'body'

    def __init__(self):
        super().__init__('quadrotor_simulator_node')

        self.declare_parameter('rate/sim', 300.0)
        self._sim_dt = 1.0 / \
            (self.get_parameter('rate/sim').get_parameter_value().double_value)
        cprint.info('using sim dt: %f' % (self._sim_dt))

        self.declare_parameter('rate/odom', 200.0)

        self.declare_parameter('frame_id/fixed', 'world')
        self._fixed_frame_id = self.get_parameter(
            'frame_id/fixed').get_parameter_value().string_value

        self.declare_parameter('frame_id/base', 'body')
        self._base_frame_id = self.get_parameter(
            'frame_id/base').get_parameter_value().string_value

        self.model = QuadrotorModel()
        model_config_file = os.path.join(ament_index_python.get_package_share_directory(
            'quadrotor_simulator_py'), 'config/Model.yaml')
        self.model.initialize(model_config_file)

        # register callbacks
        # create the model wall timer at the specified simulation rate
        self.model_timer = self.create_timer(
            self._sim_dt, self.model_timer_callback)

        # create odom and transform publishers
        odom_r = self.get_parameter(
            'rate/odom').get_parameter_value().double_value
        self.odom_timer = self.create_timer(
            1.0/odom_r, self.odom_timer_callback)
        self.odom_pub = self.create_publisher(Odometry, "odom", 10)
        self.tf_pub = self.create_publisher(TFMessage, "/tf", 10)

        # subscriber to rpm command
        self.rpm_sub = self.create_subscription(
            RPMCommand, 'rpm_cmd', self.rpm_callback, 10)

    def model_timer_callback(self):
        self._sim_time += self._sim_dt

        self.model.update(self._sim_time)

        Twb = self.model.get_pose()
        ts = TransformStamped()
        ts.transform = rnp.msgify(Transform, Twb.se3.as_matrix())
        ts.header.stamp = self.get_clock().now().to_msg()
        ts.header.frame_id = self._fixed_frame_id
        ts.child_frame_id = self._base_frame_id

        tf_msg = TFMessage(transforms=[ts])
        self.tf_pub.publish(tf_msg)

    def odom_timer_callback(self):
        Twb = self.model.get_pose()
        vw = self.model.get_world_linear_velocity()
        wb = self.model.get_body_angular_velocity()

        odom_msg = Odometry()
        odom_msg.header.frame_id = self._fixed_frame_id
        odom_msg.child_frame_id = self._base_frame_id
        odom_msg.header.stamp = (Time() +
                                 Duration(nanoseconds=self.model.get_time() *
                                 CONVERSION_CONSTANT)).to_msg()
        odom_msg.pose.pose = rnp.msgify(Pose, Twb.se3.as_matrix())
        odom_msg.twist.twist.linear = rnp.msgify(Vector3, vw)
        odom_msg.twist.twist.angular = rnp.msgify(Vector3, wb)

        self.odom_pub.publish(odom_msg)

    def rpm_callback(self, msg):
        self.model.apply_command(msg.motor_rpm[:4])


def main(args=None):
    rclpy.init(args=args)

    qsimnode = QuadrotorSimulatorNode()

    rclpy.spin(qsimnode)


if __name__ == '__main__':
    main()
