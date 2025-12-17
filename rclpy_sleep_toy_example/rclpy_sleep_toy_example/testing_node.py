#!/usr/bin/env python3

import time
import rclpy
import asyncio
from rclpy.node import Node
from std_msgs.msg import String
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup



class SyncExclusiveSleepTestNode(Node):
    def __init__(self, group_mode: String):
        super().__init__("synch_exclusive_sleep_test_node")

        self.regular_callback_group = MutuallyExclusiveCallbackGroup()

        self.regular_count_1 = 0
        self.timer_1 = self.create_timer(5.0, self.regular_callback, callback_group=self.regular_callback_group)

        if group_mode == "different":
            self.regular_callback_group_2 = MutuallyExclusiveCallbackGroup()
        else:
            self.regular_callback_group_2 = self.regular_callback_group # test putting both callbacks in the same group

        self.regular_count_2 = 0
        self.timer_2 = self.create_timer(1.0, self.regular_callback_2, callback_group=self.regular_callback_group_2)

        self.get_logger().info("Sleep node started. Watch the logs to see parallel execution!")


    def regular_callback(self):
        self.regular_count_1 += 1
        self.get_logger().warn(f'|1 starting sleep| Count: {self.regular_count_1:2d} | Time: {time.time():.2f} |')
        time.sleep(3.0) 
        self.get_logger().warn(f'|1 ending sleep| Count: {self.regular_count_1:2d} | Time: {time.time():.2f} |')


    def regular_callback_2(self):
        self.regular_count_2 += 1
        self.get_logger().info(f'| TICK 2| Count: {self.regular_count_2:2d} | Time: {time.time():.2f} |')


    def run(self):
        self.get_logger().info("Executor is now spinning...")



class SyncReentrantSleepTestNode(Node):
    def __init__(self):
        super().__init__("synch_exclusive_sleep_test_node")

        self.regular_callback_group = ReentrantCallbackGroup()

        self.regular_count_1 = 0
        self.timer_1 = self.create_timer(5.0, self.regular_callback, callback_group=self.regular_callback_group)

        self.regular_count_2 = 0
        self.timer_2 = self.create_timer(1.0, self.regular_callback_2, callback_group=self.regular_callback_group)

        self.get_logger().info("Sleep node started. Watch the logs to see parallel execution!")


    def regular_callback(self):
        self.regular_count_1 += 1
        self.get_logger().warn(f'|1 starting sleep| Count: {self.regular_count_1:2d} | Time: {time.time():.2f} |')
        time.sleep(3.0) 
        self.get_logger().warn(f'|1 ending sleep| Count: {self.regular_count_1:2d} | Time: {time.time():.2f} |')


    def regular_callback_2(self):
        self.regular_count_2 += 1
        self.get_logger().info(f'| TICK 2| Count: {self.regular_count_2:2d} | Time: {time.time():.2f} |')


    def run(self):
        self.get_logger().info("Executor is now spinning...")



class AsyncSleepTestNode(Node):
    def __init__(self):
        super().__init__("sleep_test_node")
        self.regular_timer = self.create_timer(1.0, self.regular_callback)
        self.regular_count = 0

        self.async_timer = self.create_timer(3.0, self.async_callback)
        self.async_count = 0

        self.get_logger().info("Sleep node started...")

    def regular_callback(self):
        self.regular_count += 1
        self.get_logger().info(f'| REGULAR | Count: {self.regular_count:2d} | Time: {time.time():.2f} |')
    
    async def async_callback(self):
        self.async_count += 1
        start_time = time.time()

        self.get_logger().warn(f'|ASYNC| Starting sleep| Count: {self.async_count:2d}, Time: {start_time:.2f} |'
        )

        await asyncio.sleep(2.0)

        end_time = time.time()
        self.get_logger().warn(f'|ASYNC| Finished sleep| Duration: {end_time - start_time:.2f}s')



    def callback_2(self):
        pass

    def run(self):
        self.get_logger().info("Sleep node running...")