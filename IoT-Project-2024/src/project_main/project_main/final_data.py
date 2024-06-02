import sys
import time
import math
import threading

import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer
from rclpy.action.server import ServerGoalHandle
from rclpy.executors import MultiThreadedExecutor

from std_msgs.msg import String, Int64, Float64
from geometry_msgs.msg import Point, Vector3, Twist
from nav_msgs.msg import Odometry

from rosgraph_msgs.msg import Clock

from collections import deque
import dearpygui.dearpygui as dpg


NUMBER_OF_BALLOONS = int(sys.argv[1])
NUMBER_OF_SENSORS = int(sys.argv[2])
MIN_ALTITUDE_TO_PERFORM_PATROL = 15
SIZE = 10
WORLD_NAME = "iot_project_world"

class BaseController(Node):

    def __init__(self):
        super().__init__("data_visual")


        self.current_time = 0
        self.ids = 0

        self.cache = 0
        self.cache1 = 0
        self.cache2 = 0


        self.clock_topic = self.create_subscription(
            Clock,
            f'/world/{WORLD_NAME}/clock',
            self.get_time,
            10
        )

        self.balloons_tx = {}




        for i in range(NUMBER_OF_SENSORS):
            self.create_subscription(
                String,
                f'Balloon_{i}/tx_data',
                lambda msg, id = i : self.tx_callback(id, msg),
                10
            )



            

    def get_time(self, clock: Clock):
        self.current_time = clock.clock.sec + clock.clock.nanosec * 10**(-9)

    def tx_callback(self, id, msg: String):
        self.ids = id

        if id == 0:
            self.cache = msg.data
        elif id == 1:
            self.cache1 = msg.data
        elif id == 2:
            self.cache2 = msg.data




    def update_table(self):

        if self.ids == 0:
            for row in range(1):
                #for col in range(3):
                if self.cache:
                    dpg.set_value(f"cell_{row}_0", str(self.cache))
                else:
                    dpg.set_value(f"cell_{row}_0", "")

        elif self.ids == 1:
            for row in range(1):
                #for col in range(3):
                if self.cache1:
                    dpg.set_value(f"cell_{row}_1", str(self.cache1))
                else:
                    dpg.set_value(f"cell_{row}_1", "")
        elif self.ids == 2:
            for row in range(1):
                #for col in range(3):
                if self.cache2:
                    dpg.set_value(f"cell_{row}_2", str(self.cache2))
                else:
                    dpg.set_value(f"cell_{row}_2", "")
        
        

def ros_spin(executor):
    executor.spin()

def gui_update(drone_controller):
    while dpg.is_dearpygui_running():
        with dpg.mutex():
            drone_controller.update_table()
        time.sleep(1)

def main():
    rclpy.init()
    executor = MultiThreadedExecutor()
    drone_controller = BaseController()
    executor.add_node(drone_controller)

    ros_thread = threading.Thread(target=ros_spin, args=(executor,))
    ros_thread.start()

    dpg.create_context()
    with dpg.window(label="Table of Values", width=400, height=300):
        with dpg.table(header_row=True, row_background=True, borders_innerH=True, borders_outerH=True, borders_innerV=True, borders_outerV=True):
            #for i in range(3):
            dpg.add_table_column(label="Balloon_0")
            dpg.add_table_column(label="Balloon_1")
            dpg.add_table_column(label="Balloon_2")
            for row in range(1):
                with dpg.table_row():
                    for col in range(3):
                        dpg.add_text("", tag=f"cell_{row}_{col}")

    dpg.create_viewport(title='Dear PyGui - Table of Values', width=600, height=400)
    dpg.setup_dearpygui()
    dpg.show_viewport()

    gui_thread = threading.Thread(target=gui_update, args=(drone_controller,))
    gui_thread.start()

    dpg.start_dearpygui()
    dpg.destroy_context()

    drone_controller.destroy_node()
    executor.shutdown()
    rclpy.shutdown()

    ros_thread.join()
    gui_thread.join()

if __name__ == '__main__':
    main()
