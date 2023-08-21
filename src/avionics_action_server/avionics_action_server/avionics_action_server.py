#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from avionics_interfaces.srv import RotateServo
from avionics_interfaces.msg import ServoResponse

class RotateServoServer(Node):

    def __init__(self):
        super().__init__('rotate_servo_server')

        self.srv = self.create_service(RotateServo, 'rotate_servo', self.rotate_servo_callback)

        self.response_received = False
        self.response_msg = None

        # Create a subscriber to the /servo_response topic
        self.servo_response_sub = self.create_subscription(
            ServoResponse, '/servo_response', self.response_callback, 10)  # Adjust QoSProfile as needed

        # Create a timer to check for the response periodically
        self.timer = self.create_timer(1.0, self.timer_callback)  # Adjust the timer period as needed (1.0 second here)

        self.get_logger().info("Rotate servo service server has been initialized.")

    def rotate_servo_callback(self, request, response):
        self.get_logger().info('Received service request :)')

        # Simulate servo rotation (replace with your actual servo control code)
        self.get_logger().info('Requested angle: {0}'.format(request.angle))
        self.get_logger().info('Requested channel: {0}'.format(request.channel))
        # Perform the servo rotation here and update the response accordingly
        response.success = True  # Set to True if the rotation was successful
        response.id = 1  # Set an appropriate ID
        response.angle = request.angle  # Set the actual angle

        self.get_logger().info('Waiting for response from /servo_response...')

        # Start the timer to periodically check for the response
        self.timer.reset()

        while not self.response_received:
            rclpy.spin_once(self)

        self.timer.cancel()  # Cancel the timer once the response is received

        self.get_logger().info('Returning result: SUCCESS (based on /servo_response)')

        return response

    def timer_callback(self):
        # This function is called by the timer to periodically check for the response
        if self.response_received:
            self.timer.cancel()  # Cancel the timer if the response is received

    def response_callback(self, msg):
        # Handle the response from /servo_response
        self.get_logger().info('Received response from /servo_response')
        self.response_msg = msg
        self.response_received = True

def main(args=None):
    rclpy.init(args=args)
    avionics_service_server = RotateServoServer()
    rclpy.spin(avionics_service_server)
    avionics_service_server.destroy()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
