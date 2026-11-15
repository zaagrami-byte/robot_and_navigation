
#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from tf2_ros import Buffer, TransformListener
from tf2_ros import TransformException
import math


class TFValidator(Node):

    def __init__(self):
        super().__init__('imu_tf_validator')

        self.buffer = Buffer()
        self.listener = TransformListener(self.buffer, self)

        self.timer = self.create_timer(1.0, self.check)

        self.success = False

    def check(self):

        try:
            transform = self.buffer.lookup_transform(
                'base_link',
                'imu_link',
                rclpy.time.Time()
            )

            t = transform.transform.translation
            q = transform.transform.rotation

            roll = math.atan2(
                2 * (q.w*q.x + q.y*q.z),
                1 - 2 * (q.x*q.x + q.y*q.y)
            )

            sinp = 2 * (q.w*q.y - q.z*q.x)
            pitch = math.asin(max(-1, min(1, sinp)))

            yaw = math.atan2(
                2 * (q.w*q.z + q.x*q.y),
                1 - 2 * (q.y*q.y + q.z*q.z)
            )

            print("\n===== IMU TF VALIDATION =====")

            print(
                f"Translation (m): "
                f"X={t.x:.4f}, Y={t.y:.4f}, Z={t.z:.4f}"
            )

            print(
                f"Rotation (degrees): "
                f"Roll={math.degrees(roll):.2f}, "
                f"Pitch={math.degrees(pitch):.2f}, "
                f"Yaw={math.degrees(yaw):.2f}"
            )

            print("TF STATUS: AVAILABLE")

            self.success = True

        except TransformException as e:
            print(f"TF NOT AVAILABLE: {e}")


def main():

    rclpy.init()

    node = TFValidator()

    try:
        rclpy.spin(node)

    except KeyboardInterrupt:
        pass

    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
