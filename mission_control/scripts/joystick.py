#!/usr/bin/env python3
import rclpy
import math
from rclpy.node import Node
from rclpy.parameter import Parameter

from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist

class JoystickHandler:
    """
    ## JoystickHandler Class
    
    Handles joystick messages.
    
    ### Attributes
    
    - axes (dict): A dictionary containing the joystick axes.  
        * `left_x`: left stick x-axis (left/right)
        * `left_y`: left stick y-axis (forward/backward)
        * `right_x`: right stick x-axis (left/right)
        * `right_y`: right stick y-axis (forward/backward)
        * `right_trigger`: right trigger (RT)
        * `left_trigger`: left trigger (LT)
        
    - buttons (dict): A dictionary containing the joystick buttons.  
        * `A`: button A
        * `B`: button B
        * `X`: button X
        * `Y`: button Y
        * `LB`: button LB
        * `RB`: button RB
        * `back`: button back
        * `start`: button start
        * `power`: button power
    """
    def __init__(self, msg):
        self.axes = {
            'left_x': msg.axes[0],
            'left_y': msg.axes[1],
            'right_x': msg.axes[2],
            'right_y': msg.axes[3], 
            'right_trigger': msg.axes[5],
            'left_trigger': msg.axes[4] 
        }
        self.buttons = {
            'A': msg.buttons[0],
            'B': msg.buttons[1], 
            'X': msg.buttons[2],
            'Y': msg.buttons[3], 
            'LB': msg.buttons[4], 
            'RB': msg.buttons[5], 
            'back': msg.buttons[6], 
            'start': msg.buttons[7], 
            'power': msg.buttons[8] 
        }


class JoystickController(Node):

    def __init__(self):
        super().__init__('joystick_controller')

        # Initialize parameter with a default value
        self.declare_parameter("velocity_scaling_factor", 1.0)
        self.scaling_factor = self.get_parameter("velocity_scaling_factor").value
        
        # Define subscriber to 'joy/joy' topic
        self.subscription = self.create_subscription(Joy, 'joy/joy', self.joystick_callback, 10)
        
        # Define publisher to output velocities
        self.publisher = self.create_publisher(Twist, 'cmd_vel', 10)

        self.subscription  # prevent unused variable warning

    def joystick_callback(self, msg):
        twist = Twist()
        
        joystick: JoystickHandler = JoystickHandler(msg)

        lin_x, ang_z = self.map_circle_to_square(joystick.axes['left_x'], joystick.axes['left_y'])


        # Assuming that axes[0] corresponds to x-axis (forward/backward) and axes[1] to y-axis (left/right)
        twist.linear.x =  lin_x * self.scaling_factor  # scaling factor for linear velocity
        twist.angular.z = ang_z * self.scaling_factor  # scaling factor for angular velocity
        
        self.publisher.publish(twist)

    def map_circle_to_square(x, y):
        """
        Map (x, y) coordinates from a circle to a square.

        #### Parameters:
        - x (float): x-coordinate in circular range [-1, 1]
        - y (float): y-coordinate in circular range [-1, 1]

        #### Returns:
        - tuple (float, float): (x, y) coordinates in square range
        """
        # Calculate the radius and angle (theta) from the input coordinates
        radius = math.sqrt(x ** 2 + y ** 2)
        theta = math.atan2(y, x)

        # Perform the squircular mapping
        square_x = radius * math.cos(theta) / max(abs(math.cos(theta)), abs(math.sin(theta)))
        square_y = radius * math.sin(theta) / max(abs(math.cos(theta)), abs(math.sin(theta)))

        return square_x, square_y

def main(args=None):
    rclpy.init(args=args)
    joystick_velocity_converter = JoystickController()
    rclpy.spin(joystick_velocity_converter)
    
    # Destroy the node explicitly (optional)
    joystick_velocity_converter.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()