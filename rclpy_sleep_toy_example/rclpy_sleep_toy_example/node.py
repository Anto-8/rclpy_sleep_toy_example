#!/usr/bin/env python3

import time
import rclpy
import asyncio
from std_msgs.msg import String
from rclpy.executors import MultiThreadedExecutor
from rclpy_sleep_toy_example.testing_node import *


async def spin_node(executor):
    while rclpy.ok():
        # Check for ROS events, but don't block
        executor.spin_once(timeout_sec=0)
        # Yield control to asyncio so 'await' can work
        await asyncio.sleep(0.01)


def main(args=None):
    rclpy.init(args=args)
    # node = SyncExclusiveSleepTestNode(group_mode="same")
    # node = SyncReentrantSleepTestNode()
    # node = FloodedReentrantSleepTestNode()
    loop = asyncio.get_event_loop()
    node = AsyncFloodedNode(loop)

    executor = MultiThreadedExecutor(num_threads=1)
    executor.add_node(node)
    # node.run()

    loop = asyncio.get_event_loop()
    try:
        # executor.spin()
        loop.run_until_complete(spin_node(executor))
    except KeyboardInterrupt:
        node.get_logger().info('Shutting down...')
    finally:
        executor.remove_node(node)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()