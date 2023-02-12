from threading import Thread

import rclpy
from rclpy.node import Node

from .ron_viz_window import RonVizWindow

def main():
    rclpy.init()
    node = Node("ron_viz")
    ros_thread = Thread(target=rclpy.spin, args=(node,))
    ros_thread.start()

    app = RonVizWindow()
    app.run()

    if rclpy.ok():
        rclpy.shutdown()
    ros_thread.join()
