import board
import neopixel
import rclpy
from rclpy.node import Node
from std_msgs.msg import UInt8MultiArray


class RgbLedsHandlerNode(Node):
    def __init__(self):
        super().__init__('camera_handler')
        self.no_of_leds = 4
        self.leds = neopixel.NeoPixel(board.D18, self.no_of_leds)
        self.rgb_leds_subscriber = self.create_subscription(
            UInt8MultiArray,
            '/bpc_prp_robot/rgb_leds',
            self.rgb_leds_callback, 1)

        for i in range(self.no_of_leds):
            self.leds[i] = (1, 1, 1)

    def rgb_leds_callback(self, msg: UInt8MultiArray):
        self.get_logger().info(f"Received: {msg.data}")
        for i in range(min(msg.data.__len__(), self.no_of_leds * 3) // 3):
            self.leds[i] = (msg.data[i * 3], msg.data[i * 3 + 1], msg.data[i * 3 + 2])


def main(args=None):
    rclpy.init(args=args)

    rgb_leds_handler_node = RgbLedsHandlerNode()

    try:
        rclpy.spin(rgb_leds_handler_node)
    except KeyboardInterrupt:
        pass
    finally:
        rgb_leds_handler_node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
