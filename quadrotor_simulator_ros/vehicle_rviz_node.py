import os

import rclpy
from rclpy.node import Node

from ament_index_python import get_package_share_directory

from visualization_msgs.msg import Marker
from std_msgs.msg import ColorRGBA


class ColorPalette:
    # color palette is a private dictionary
    _palette = {}

    def __init__(self):
        # some default colors for convenience
        self._palette["red"] = ColorRGBA(r=0.8, g=0.1, b=0.1, a=1.0)
        self._palette["green"] = ColorRGBA(r=0.1, g=0.8, b=0.1, a=1.0)
        self._palette["grey"] = ColorRGBA(r=0.9, g=0.9, b=0.9, a=1.0)
        self._palette["dark_grey"] = ColorRGBA(r=0.6, g=0.6, b=0.6, a=1.0)
        self._palette["white"] = ColorRGBA(r=1.0, g=1.0, b=1.0, a=1.0)
        self._palette["orange"] = ColorRGBA(r=1.0, g=0.5, b=0.0, a=1.0)
        self._palette["translucent_light"] = ColorRGBA(
            r=0.1, g=0.1, b=0.1, a=0.1)
        self._palette["translucent"] = ColorRGBA(r=0.1, g=0.1, b=0.1, a=0.25)
        self._palette["translucent_dark"] = ColorRGBA(
            r=0.1, g=0.1, b=0.1, a=0.5)
        self._palette["black"] = ColorRGBA(r=0.0, g=0.0, b=0.0, a=1.0)
        self._palette["yellow"] = ColorRGBA(r=1.0, g=1.0, b=0.0, a=1.0)
        self._palette["brown"] = ColorRGBA(r=0.597, g=0.296, b=0.0, a=1.0)
        self._palette["pink"] = ColorRGBA(r=1.0, g=0.4, b=1.0, a=1.0)
        self._palette["lime_green"] = ColorRGBA(r=0.6, g=1.0, b=0.2, a=1.0)
        self._palette["purple"] = ColorRGBA(r=0.597, g=0.0, b=0.597, a=1.0)
        self._palette["cyan"] = ColorRGBA(r=0.0, g=1.0, b=1.0, a=1.0)
        self._palette["magenta"] = ColorRGBA(r=1.0, g=0.0, b=1.0, a=1.0)
        self._palette["blue"] = ColorRGBA(r=0.1, g=0.1, b=0.8, a=1.0)

    def set_color(self, string, r, g, b, a):
        self._palette[string] = ColorRGBA(r, g, b, a)

    def get_color(self, string):
        try:
            return self._palette[string]
        except KeyError:
            print('[ColorPalette] color %s not found. using red.' % (string))
            return self._palette["red"]

    def get_color_in_range(self, min, max, val):
        ratio = 2.0 * (val - min) / (max - min)
        r = max(0.0, 255.0 * (1.0 - ratio)) / 255.0
        b = max(0.0, 255.0 * (ratio - 1.0)) / 255.0
        g = max(0.0, 1.0 - b - r)

        return ColorRGBA(r, g, b, 1.0)


class VehicleRVizNode(Node):
    def __init__(self, color):
        super().__init__('vehicle_rviz_node')

        self.filename = 'package://quadrotor_simulator_py/data/vehicle.dae'
        print(self.filename)

        self.publisher = self.create_publisher(Marker, 'vehicle_mesh', 10)

        self.marker = Marker()
        self.marker.header.frame_id = "body"

        self.marker.ns = "vehicle_mesh_surface"
        self.marker.id = 0
        self.marker.type = self.marker.MESH_RESOURCE
        self.marker.action = self.marker.ADD
        self.marker.pose.position.x = 0.0
        self.marker.pose.position.y = 0.0
        self.marker.pose.position.z = 0.0
        self.marker.pose.orientation.x = 0.5
        self.marker.pose.orientation.y = 0.5
        self.marker.pose.orientation.z = 0.5
        self.marker.pose.orientation.w = 0.5
        self.marker.scale.x = 1.0
        self.marker.scale.y = 1.0
        self.marker.scale.z = 1.0
        self.marker.color = color
        self.marker.frame_locked = True
        self.marker.mesh_resource = self.filename

        self.vehicle_viz_timer = self.create_timer(1.0/30.0, self.vehicle_viz_callback)

    def vehicle_viz_callback(self):
        current_time = self.get_clock().now()
        self.marker.header.stamp = current_time.to_msg()
        self.publisher.publish(self.marker)


def main(args=None):
    rclpy.init(args=args)

    colors = ColorPalette()
    node = VehicleRVizNode(colors.get_color("red"))

    rclpy.spin(node)


if __name__ == "__main__":
    main()
