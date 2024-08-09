import sys

from robot_interfaces.srv import Heartbeat
from robot_interfaces.msg import Status

import rclpy
import time
import vlc
import os
from rclpy.node import Node
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor

class HeartbeatTransmitter(Node):

    def __init__(self):

        super().__init__('heartbeat_transmitter')

        # The value to be transmitted (used for testing purposes)
        self.declare_parameter('isup', False)

        client_group = MutuallyExclusiveCallbackGroup()
        timer_group = MutuallyExclusiveCallbackGroup()
        self.heartbeat_client = self.create_client(Heartbeat, 'heartbeat', callback_group=client_group)
        self.status_msg = self.create_publisher(Status, 'transmitter_communication', 10)
        self.timeout = 5

        # While the service is not up
        while not self.heartbeat_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Heartbeat Reciever not started, trying again')

            # Send a message saying communications are down
            statusMsg = Status()
            statusMsg.isup = False
            self.status_msg.publish(statusMsg)

        # Set the request message type
        self.req = Heartbeat.Request()

        # Create the timer after the service is up
        self.timer = self.create_timer(1, self.timer_callback, callback_group=timer_group)

        # Create the error message
        # absolute_path = os.path.dirname(__file__)
        # relative_path = "src/lib"
        # full_path = os.path.join(absolute_path, relative_path)
        self.error = vlc.MediaPlayer("src/control_packages/comms_heartbeat_pkg/sounds/Emergency.mp3")
        self.errorFlag = False

    # Send a consistent heartbeat
    def timer_callback(self):
        self.beat(self.get_parameter('isup').get_parameter_value().bool_value)     

    # Send a heartbeat
    def beat(self, pulse : bool):

        # Reset the sound
        if self.error.get_state() == vlc.State.Ended:
            self.error.stop()

        # Set the service outgoing boolean
        self.req.beat = pulse

        # Call the service and save the time
        self.future = self.heartbeat_client.call_async(self.req)
        self.last_beat = time.time()

        # Create a new message to show if communications are up
        statusMsg = Status()

        while rclpy.ok():

            if self.future.done() and self.future.result():
                
                

                # Everything is currently operational
                statusMsg.isup = True

                # Print to the console
                self.get_logger().info('\t ﮩ٨ـﮩﮩ٨ـ')

                self.status_msg.publish(statusMsg)
                return self.future.result()
            
            cur_time = time.time()

            # Check to see if a timeout has occured
            if ((self.last_beat + self.timeout) < cur_time):

                # Remove the pending request to avoid edge cases
                self.heartbeat_client.remove_pending_request

                # A timeout for response has occured
                statusMsg.isup = False

                # Print to the console
                self.get_logger().info('\033[38;2;255;128;0m\t _______\033[38;2;255;255;255m')
                self.status_msg.publish(statusMsg)

                # Check to see if the service is up at all 
                while not self.heartbeat_client.wait_for_service(timeout_sec=1.0):

                    # If this is the first time play the sound
                    if not self.errorFlag:
                        self.error.play()
                        self.errorFlag = True

                    # A timeout for response has occured
                    statusMsg.isup = False

                    # Print to the console
                    self.get_logger().info('\033[38;2;255;0;0m\t _______\033[38;2;255;255;255m')
                    self.status_msg.publish(statusMsg)

                # Change the flag
                self.errorFlag = False

                # Return nothing
                return None

        return None

def main():
    rclpy.init()
    heartbeat_transmitter = HeartbeatTransmitter()

    executor = MultiThreadedExecutor()
    executor.add_node(heartbeat_transmitter)
    executor.spin()

    heartbeat_transmitter.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
        