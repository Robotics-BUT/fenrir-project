import rclpy
from rclpy.node import Node
from std_msgs.msg import UInt8
import RPi.GPIO as GPIO

GPIO.cleanup()
GPIO.setmode(GPIO.BCM)

class ButtonsHandlerNode(Node):
    def __init__(self):
        super().__init__('buttons_handler')
        self.buttons_publisher_ = self.create_publisher(UInt8, '/bpc_prp_robot/buttons', 1)
        self.pins = [5, 6, 13]
        self.pin_map = {
            5: 0,
            6: 1,
            13: 2,
        }

        self.btn_states = {}
        for pin in self.pins:
            GPIO.setup(pin, GPIO.IN, pull_up_down=GPIO.PUD_UP)
            self.btn_states[pin] = False

        self.timer = self.create_timer(0.1, self.timer_callback)


    def timer_callback(self):
        for pin in self.pins:
            state = GPIO.input(pin)
            if state == False and self.btn_states[pin] == True:
                self.edge_detected(pin)
            self.btn_states[pin] = state


    def edge_detected(self, pin):
        print(f"Button {self.pin_map[pin]} pressed")
        msg = UInt8()
        msg.data = self.pin_map[pin]
        self.buttons_publisher_.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    GPIO.setmode(GPIO.BCM)

    buttons_handler_node = ButtonsHandlerNode()

    try:
        rclpy.spin(buttons_handler_node)
    except KeyboardInterrupt:
        pass
    finally:
        buttons_handler_node.destroy_node()
        GPIO.cleanup()  # Clean up GPIO pins
        rclpy.shutdown()


if __name__ == '__main__':
    main()
