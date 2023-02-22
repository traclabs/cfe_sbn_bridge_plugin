#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rosidl_runtime_py import get_message_interfaces
from ament_index_python.packages import get_package_share_directory


class CFEConsoleTest(Node):

    def __init__(self):
        super().__init__('cfe_console_test')

        self.get_logger().warn("================================================================")
        self.get_logger().warn("CFEConsoleTest")
        self.get_logger().warn("================================================================")

        self._info_period = 1.0  # seconds
        self._warn_period = 2.0  # seconds
        self._error_period = 3.0  # seconds
        self._debug_period = 0.5  # seconds
        self._fatal_period = 10.0  # seconds

        self._info_timer = self.create_timer(self._info_period, self.info_callback)
        self._warn_timer = self.create_timer(self._warn_period, self.warn_callback)
        self._error_timer = self.create_timer(self._error_period, self.error_callback)
        self._debug_timer = self.create_timer(self._debug_period, self.debug_callback)
        # self._fatal_timer = self.create_timer(self._fatal_period, self.fatal_callback)


    def info_callback(self):
        self.get_logger().info("CFEConsoleTest() -- INFO tick")

    def warn_callback(self):
        self.get_logger().warn("CFEConsoleTest() -- WARN tick")

    def error_callback(self):
        self.get_logger().error("CFEConsoleTest() -- ERROR tick")

    def debug_callback(self):
        self.get_logger().debug("CFEConsoleTest() -- DEBUG tick")

    def fatal_callback(self):
        self.get_logger().fatal("CFEConsoleTest() -- FATAL tick")



def main(args=None):
    rclpy.init(args=args)

    test = CFEConsoleTest()
    rclpy.spin(test)

    test.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
