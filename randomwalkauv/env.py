import rclpy
from rclpy.node import Node
import numpy as np
import time

#Import Interfaces
from rw_interfaces.msg import Action, State

class Env(Node):

    def __init__(self):
        # Register as AUV node
        super().__init__('env')

        #Register as Publisher to State
        self.publisher_state = self.create_publisher(State, 'state', 10)
        #Register as Subscriber to Action
        self.subscription = self.create_subscription(
            Action, 
            'action', 
            self.action_callback, 
            10)
        self.subscription # prevent unused variable warning

        #Set Up Variables
        self.bearing = 0.0
        self.auv_heading = 0.0
        self.auv_pos = np.array((0.0 ,0.0))
        self.goal_pos = np.array((1500.0, 0.0))
        self.range = np.linalg.norm(self.goal_pos - self.auv_pos)
        self.interval = 5.0
        self.done = False
        self.reward = 0.0
        self.step = 0

        #Publish State for first time
        time.sleep(5)
        state = State()
        state.range = self.range
        state.bearing = self.bearing
        state.step = self.step
        state.reward = 0.0
        
        #Log before publishing action
        self.log_state()
        self.publisher_state.publish(state)

    def log_state(self):
        #Log AUV Pos
        self.get_logger().info('AUV is at X: %d, Y: %d, Heading: %d' % (self.auv_pos[0], self.auv_pos[1],self.auv_heading))
        #Log Goal Pos
        self.get_logger().info('Goal is at X: %d, Y: %d' % (self.goal_pos[0], self.goal_pos[1]))
        #Log State
        self.get_logger().info('State is Range: %d, Bearing: %d, Reward: %d, Step: %d' % (self.range, self.bearing, self.reward, self.step))

    #Define Callback for action
    def action_callback(self, msg):
        
        # Decompose Message
        action = msg.action

        #Update AUV Pos
        self.auv_heading += action
        self.auv_pos[0] = self.auv_pos[0] + np.cos(self.auv_heading*np.pi/180.0) * self.interval
        self.auv_pos[1] += np.sin(self.auv_heading*np.pi/180.0) * self.interval
        
        #Calc Range and Bearing
        self.range = np.linalg.norm(self.goal_pos - self.auv_pos)
        temp = self.goal_pos - self.auv_pos
        self.bearing = (np.arctan(temp[1] / temp[0]) * 180.0 / np.pi) - self.auv_heading
        

        #Update Step
        self.step += 1

        # Calc Reward and decide if Done
        self.reward = 0.0

        if abs(self.auv_heading) > 90 or self.range < 10.0 or self.step > 1000:
            self.get_logger().info('Range is: %d, Heading is: %d, Step is: %d' % (self.range, self.auv_heading, self.step))
            self.get_logger().info('State Will Be Reset!')

            self.bearing = 0.0
            self.auv_heading = 0.0
            self.auv_pos = np.array((0.0 ,0.0))
            self.goal_pos = np.array((1500.0, 0.0))
            self.range = np.linalg.norm(self.goal_pos - self.auv_pos)
            self.step = 0
            self.reward = 0.0
        
        state = State()
        state.range = self.range
        state.bearing = self.bearing
        state.reward = self.reward
        state.step = self.step

        #Log before Publishing
        self.log_state()
        self.publisher_state.publish(state)

def main(args=None):
    rclpy.init(args=args)

    env = Env()

    rclpy.spin(env)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    env.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()