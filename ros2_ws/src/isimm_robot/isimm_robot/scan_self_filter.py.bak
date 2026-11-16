#!/usr/bin/env python3

import math

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan


class ScanSelfFilter(Node):

    def __init__(self):
        super().__init__('scan_self_filter')

        self.declare_parameter('input_topic', 'scan_raw')
        self.declare_parameter('output_topic', 'scan')
        self.declare_parameter('laser_x', 0.0)
        self.declare_parameter('laser_y', 0.0)
        self.declare_parameter('box_half_length', 0.285)
        self.declare_parameter('box_half_width', 0.20)
        self.declare_parameter('margin', 0.02)
        self.declare_parameter('min_range', 0.15)

        self.input_topic = self.get_parameter(
            'input_topic').value
        self.output_topic = self.get_parameter(
            'output_topic').value

        self.laser_x = self.get_parameter('laser_x').value
        self.laser_y = self.get_parameter('laser_y').value
        self.half_length = (
            self.get_parameter('box_half_length').value
            + self.get_parameter('margin').value
        )
        self.half_width = (
            self.get_parameter('box_half_width').value
            + self.get_parameter('margin').value
        )
        self.min_range = self.get_parameter('min_range').value

        self.publisher = self.create_publisher(
            LaserScan, self.output_topic, 10)

        self.subscription = self.create_subscription(
            LaserScan,
            self.input_topic,
            self.scan_callback,
            rclpy.qos.qos_profile_sensor_data
        )

        self.get_logger().info(
            f'Self filter: {self.input_topic} -> {self.output_topic}'
        )

    def scan_callback(self, msg):

        output = LaserScan()
        output.header = msg.header
        output.angle_min = msg.angle_min
        output.angle_max = msg.angle_max
        output.angle_increment = msg.angle_increment
        output.time_increment = msg.time_increment
        output.scan_time = msg.scan_time
        output.range_min = msg.range_min
        output.range_max = msg.range_max
        output.intensities = list(msg.intensities)

        output.ranges = list(msg.ranges)

        for i, distance in enumerate(msg.ranges):

            if not math.isfinite(distance):
                continue

            if distance < self.min_range:
                output.ranges[i] = float('inf')
                continue

            angle = (
                msg.angle_min + i * msg.angle_increment
            )

            x = self.laser_x + distance * math.cos(angle)
            y = self.laser_y + distance * math.sin(angle)

            if (
                abs(x) <= self.half_length
                and abs(y) <= self.half_width
            ):
                output.ranges[i] = float('inf')

        self.publisher.publish(output)


def main(args=None):

    rclpy.init(args=args)
    node = ScanSelfFilter()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
