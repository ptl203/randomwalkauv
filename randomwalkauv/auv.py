import rclpy
from rclpy.node import Node
import numpy as np
import time

#Import Interfaces
from rw_interfaces.msg import Action, State

class Auv(Node):

    def __init__(self):
        # Register as AUV node
        super().__init__('auv')
        #Register as Publisher to Action
        self.publisher_ = self.create_publisher(Action, 'action', 10)
        #Register as Subscriber to State
        self.subscription = self.create_subscription(
            State, 
            'state', 
            self.state_callback, 
            10)
        self.subscription # prevent unused variable warning
        self.action_space = [-5.0, 0.0, 5.0]

    #Define Callback for State
    def state_callback(self, msg):
        #Process Message
        relativeBearing = msg.bearing
        range = msg.range
        #Choose Action
        chosenAction = np.random.choice(self.action_space)
        #Publish Action
        action = Action()
        action.action = chosenAction

        # Log before publishing action
        self.get_logger().info('Chosen Action is to turn %d degrees' % (chosenAction))
        self.publisher_.publish(action)

def main(args=None):
    rclpy.init(args=args)

    auv = Auv()

    rclpy.spin(auv)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    auv.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()