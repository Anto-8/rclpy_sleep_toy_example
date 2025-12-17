#!/usr/bin/env python3

import time
import rclpy
import asyncio
import threading
from rclpy.node import Node
from std_msgs.msg import String
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup


class FloodedReentrantSleepTestNode(Node):
    def __init__(self):
        super().__init__("flooded_sleep_test_node")
        
        self.group = ReentrantCallbackGroup()
        
        self.active_calls = 0
        self.lock = threading.Lock()

        for i in range(100):
            cb_lambda = lambda idx=i: self.callback(idx)
            self.create_timer(1.0, cb_lambda, callback_group=self.group)

        self.get_logger().info("Sleep node started. Watch the logs to see parallel execution!")


    def callback(self, idx):
        with self.lock:
            self.active_calls += 1
            current_active = self.active_calls
        
        self.get_logger().info(f"TICK ON CALLBACK {idx}")
        # self.get_logger().warn(f"Starting sleep on callback {idx}")
        time.sleep(10.0)
        self.get_logger().warn(f"Ending sleep on callback {idx}")
        
        with self.lock:
            self.active_calls -= 1

        self.get_logger().info(f"Active Threads: {current_active}")


    def run(self):
        self.get_logger().info("Executor is now spinning...")




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
        super().__init__("synch_reentrant_sleep_test_node")

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



class AsyncFloodedNode(Node):
    def __init__(self, loop):
        super().__init__("async_flooded_node")
        # Store the loop main 
        self._loop = loop 
        self.group = MutuallyExclusiveCallbackGroup()

        self.create_timer(5.0, lambda: self.timer_wrapper("LONGER  ", 5.0), callback_group=self.group)
        self.create_timer(2.0, lambda: self.timer_wrapper("MID     ", 1.0), callback_group=self.group)
        self.create_timer(1.0, lambda: self.timer_wrapper("HI-FREQ ", 0.1), callback_group=self.group)

        self.get_logger().info("Async Node Initialized. Loop captured.")

    def timer_wrapper(self, name, duration):
        # schedule the coroutine on the main thread's loop
        self._loop.call_soon_threadsafe(
            lambda: asyncio.create_task(self.async_callback(name, duration))
        )

    async def async_callback(self, name, sleep_duration):
        self.get_logger().info(f"[{name}] Starting sleep ({sleep_duration}s)")
        await asyncio.sleep(sleep_duration)
        self.get_logger().warn(f"[{name}] Woke up!")