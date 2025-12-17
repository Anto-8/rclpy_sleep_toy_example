#!/usr/bin/env python3

import time
import rclpy
import asyncio
from std_msgs.msg import String
from rclpy.executors import MultiThreadedExecutor
from rclpy_sleep_toy_example.testing_node import AsyncSleepTestNode, SyncExclusiveSleepTestNode



def main(args=None):
    rclpy.init(args=args)
    node = SyncExclusiveSleepTestNode()

    executor = MultiThreadedExecutor()
    executor.add_node(node)
    node.run()

    try:
        executor.spin()
    except KeyboardInterrupt:
        node.node.get_logger().info('Shutting down...')
    finally:
        executor.remove_node(node)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()