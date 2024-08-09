import rclpy
import time
from rclpy.node import Node

from robot_interfaces.srv import Heartbeat
from robot_interfaces.msg import Status

class HeartbeatReciever(Node):
    def __init__(self):
        super().__init__('heartbeat_reciever')

        heartbeat_group = None
        timer_group = None

        self.heartbeat_srv = self.create_service(Heartbeat, 'heartbeat', self.reciever_callback, callback_group=heartbeat_group)
        self.status_msg = self.create_publisher(Status, 'reciever_communication', 10)

        self.call_timer = self.create_timer(1, self.heartbeatTimer, callback_group=timer_group)

        self.timeout = 5 # Time in seconds to wait if heartbeat dies
        self.last_beat = time.time() # Time the last hearbeat was recieved

    def reciever_callback(self, request, response):
        # Make a new status message
        statusMsg = Status()

        # If the heartbeat says to stay up
        if (request.beat):
            statusMsg.isup = True
            
            # Print out a message
            self.get_logger().info('\tHeartbeat recieved: \033[38;2;0;255;0mUP\033[38;2;255;255;255m')
            self.status_msg.publish(statusMsg)

        # If it says to stay down
        else:
            statusMsg.isup = False

            # Print out a message
            self.get_logger().info('\tHeartbeat recieved: \033[38;2;255;0;0mDown\033[38;2;255;255;255m')
            self.status_msg.publish(statusMsg)

        # No matter what the message was heard
        response.echo = True
        self.last_beat = time.time()

        return response
    
    def heartbeatTimer(self):
        self.heartbeatFail()
    
    def heartbeatFail(self):

        # Get the current time
        cur_time = time.time()

        # If a timeout has occured
        if ((self.last_beat + self.timeout) < cur_time):

            # Make a new message
            statusMsg = Status()
            statusMsg.isup = False

            # Print out a message
            self.get_logger().info('\t\033[38;2;255;0;0mHEARTBEAT FAILED: STATUS DOWN\033[38;2;255;255;255m')
            self.status_msg.publish(statusMsg)

def main():
    rclpy.init()
    
    heartbeat_reciever = HeartbeatReciever()

    heartbeat_reciever.heartbeatFail()
    rclpy.spin(heartbeat_reciever)

    rclpy.shutdown()

if __name__ == '__main__':
    main()