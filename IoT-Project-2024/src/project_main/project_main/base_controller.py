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
import math_utils



NUMBER_OF_BALLOONS = int(sys.argv[1])
NUMBER_OF_SENSORS = int(sys.argv[2])
MIN_ALTITUDE_TO_PERFORM_PATROL = 15
SIZE = 10
WORLD_NAME = "iot_project_world"

class BaseController(Node):

    def __init__(self):
        super().__init__("drone_controller")

        self.cache = deque(maxlen=SIZE)  # K is the maximum storage capacity
        self.cache1 = deque(maxlen=SIZE)
        self.cache2 = deque(maxlen=SIZE)
        self.cache_size = SIZE
        self.current_time = 0
        self.ids = 0
        self.sens_num = 0
        self.dist = 0

        self.position = Point(x=0.0, y=0.0, z=0.0)
        self.yaw = 0

        self.total_lost_data = 0
        self.total_packet_delay = 0.0
        self.published_packets = 0

        self.clock_topic = self.create_subscription(
            Clock,
            f'/world/{WORLD_NAME}/clock',
            self.get_time,
            10
        )

        self.balloons_tx = {}
        self.sensor_positions = {}
        self.balloon_positions = {}

        for i in range(NUMBER_OF_SENSORS):
            self.create_subscription(
                String,
                f'Balloon_{i}/rx_data',
                lambda msg, id = i : self.rx_callback(id, msg),
                10
            )

            self.balloons_tx[i] = self.create_publisher(
                String,
                f'Balloon_{i}/tx_data',
                10
            )

            self.create_subscription(
                Odometry,
                f'ActiveSensor_{i}/odometry',
                lambda odometry_msg, sensor_id = i: self.store_sensor_position(sensor_id, odometry_msg),
                10
                #self.store_sensor_position
            )

            self.create_subscription(
                Odometry,
                f'Balloon_{i}/odometry',
                lambda odometry_msg, balloon_id = i: self.store_balloon_position(balloon_id, odometry_msg),
                10
                #self.store_sensor_position
            )

    def store_sensor_position(self, sensor_id, position : Odometry):

        self.sensor_positions[sensor_id] = position.pose.pose.position


    def store_balloon_position(self, balloon_id, position : Odometry):

        self.balloon_positions[balloon_id] = position.pose.pose.position


    def get_time(self, clock: Clock):
        self.current_time = clock.clock.sec + clock.clock.nanosec * 10**(-9)

    def rx_callback(self, id, msg: String):
        self.ids = id

        #self.get_logger().info(str(id))

        if id == 0:
            data = self.convert_to(msg.data)
            self.cache_data(data, self.current_time)
            self.offload_data()
        elif id == 1:
            data = self.convert_to(msg.data)
            self.cache_data1(data, self.current_time)
            self.offload_data()
        elif id == 2:
            data = self.convert_to(msg.data)
            self.cache_data2(data, self.current_time)
            self.offload_data()
        
        self.dist = math_utils.point_distance(self.sensor_positions[self.sens_num], self.balloon_positions[self.ids])
        

    def cache_data(self, data, timestamp):
        DELTA_T = 5
        # Remove expired data and count lost data
        self.total_lost_data += len([entry for entry in self.cache if entry['timestamp'] <= timestamp - DELTA_T])
        self.cache = deque([entry for entry in self.cache if entry['timestamp'] > timestamp - DELTA_T])
        # Add new data
        self.cache.append({'data': data, 'timestamp': timestamp})
        #self.get_logger().info(str(len(self.cache)))

    def cache_data1(self, data, timestamp):
        DELTA_T = 5
        # Remove expired data and count lost data
        self.total_lost_data += len([entry for entry in self.cache1 if entry['timestamp'] <= timestamp - DELTA_T])
        self.cache1 = deque([entry for entry in self.cache1 if entry['timestamp'] > timestamp - DELTA_T])
        # Add new data
        self.cache1.append({'data': data, 'timestamp': timestamp})
        #self.get_logger().info(str(len(self.cache1)))

    def cache_data2(self, data, timestamp):
        DELTA_T = 5
        # Remove expired data and count lost data
        self.total_lost_data += len([entry for entry in self.cache2 if entry['timestamp'] <= timestamp - DELTA_T])
        self.cache2 = deque([entry for entry in self.cache2 if entry['timestamp'] > timestamp - DELTA_T])
        # Add new data
        self.cache2.append({'data': data, 'timestamp': timestamp})
        #self.get_logger().info(str(len(self.cache2)))

    def offload_data(self):
        msg = String()
        if self.cache:
            msg.data = self.cache[0]['data']
            self.total_packet_delay += self.current_time - self.cache[0]['timestamp']
            self.published_packets += 1
            self.balloons_tx[0].publish(msg)
            #self.cache.popleft()

        if self.cache1:
            msg.data = self.cache1[0]['data']
            self.total_packet_delay += self.current_time - self.cache1[0]['timestamp']
            self.published_packets += 1
            self.balloons_tx[1].publish(msg)
            #self.cache1.popleft()

        if self.cache2:
            msg.data = self.cache2[0]['data']
            self.total_packet_delay += self.current_time - self.cache2[0]['timestamp']
            self.published_packets += 1
            self.balloons_tx[2].publish(msg)
            #self.cache2.popleft()

    def convert_to(self, string_value):
        cleaned_string = string_value.replace("Sensor data: ", "").replace("!", "")
        val,n = cleaned_string.split("_")
        self.sen_num = int(val)
        try:
            return str(cleaned_string)
        except ValueError:
            return None

    def update_table(self):
        if self.ids == 0:
            value = 1
            for row in range(5):
                if value <= len(self.cache):
                    dpg.set_value(f"cell_{row}_0", str(self.cache[value - 1]['data']))
                else:
                    dpg.set_value(f"cell_{row}_0", "")
                value += 1
        elif self.ids == 1:
            value = 1
            for row in range(5):
                if value <= len(self.cache1):
                    dpg.set_value(f"cell_{row}_1", str(self.cache1[value - 1]['data']))
                else:
                    dpg.set_value(f"cell_{row}_1", "")
                value += 1
        elif self.ids == 2:
            value = 1
            for row in range(5):
                if value <= len(self.cache2):
                    dpg.set_value(f"cell_{row}_2", str(self.cache2[value - 1]['data']))
                else:
                    dpg.set_value(f"cell_{row}_2", "")
                value += 1

        # Update statistics
        if self.published_packets > 0:
            avg_delay = self.total_packet_delay / self.published_packets
        else:
            avg_delay = 0.0

        dpg.set_value("lost_data", f"Lost Data: {self.total_lost_data}")
        dpg.set_value("avg_delay", f"Avg Packet Delay: {avg_delay:.2f}s")
        dpg.set_value("distance", f"distance: {self.dist:.2f}m")

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
            dpg.add_table_column(label="Balloon_0")
            dpg.add_table_column(label="Balloon_1")
            dpg.add_table_column(label="Balloon_2")
            for row in range(5):
                with dpg.table_row():
                    for col in range(3):
                        dpg.add_text("", tag=f"cell_{row}_{col}")

        # Add statistics display
        dpg.add_text("", tag="lost_data")
        dpg.add_text("", tag="avg_delay")
        dpg.add_text("", tag="distance")

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
