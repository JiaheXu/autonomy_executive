#!/usr/bin/env python
"""
Inputs a Joy messages and produces controls
that match the control_interface
"""
import rclpy
from rclpy.node import Node

import time
import math
from geometry_msgs.msg import PointStamped, TwistStamped
from std_msgs.msg import UInt8, Bool, String
from sensor_msgs.msg import Joy
from airlab_msgs.msg import AIRLABModes
from std_msgs.msg import String, Float32, Int8, UInt8, Bool, UInt32MultiArray, Int32


class JoystickCommandSource(Node):
    def __init__(self):

        super().__init__('joystick_command_source')
        # self.last_log_time = self.get_clock().now()

        # Axes for control
        # self.get_parameter('my_parameter').get_parameter_value().string_value

        # self.declare_parameter('my_parameter', 'world')

        self.left_joystick_x = 0
        self.left_joystick_y = 1
        self.left_trigger = 2
        self.right_joystick_x = 3
        self.right_joystick_y = 4
        self.right_trigger = 5
        self.dpad_left_right = 6
        self.dpad_up_down = 7

        self.max_idx = max(
            self.left_trigger,
            self.left_joystick_x,
            self.left_joystick_y,
            self.dpad_up_down,
            self.right_joystick_x,
            self.right_joystick_y,
            self.right_trigger,
        )

        # Buttons for control
        self.a_button = 0
        self.b_button = 1
        self.x_button = 2
        self.y_button = 3
        self.lb_button = 4
        self.rb_button = 5
        self.back_button = 6
        self.start_button = 7
        self.xbox_button = 8
        self.left_joystick_button = 9
        self.right_joystick_button = 10
        
        self.max_button = max(
            self.b_button,
            self.y_button,
            self.x_button,
            self.a_button,
            self.rb_button,
            self.back_button,
            self.start_button,
            self.left_joystick_button,
            self.right_joystick_button,
            self.lb_button
        )

        self.authority_name = self.declare_parameter(
          'authority_name', 'local_joystick').get_parameter_value().string_value

        # print("self.authority_name: ", self.authority_name)
        # self.robot_iteration_deadband = Node.declare_parameter("~robot_iteration_deadband", 0.5)
        # self.exploration_trigger_debounce = Node.declare_parameter("~exploration_trigger_debounce", 5.)
        self.exploration_trigger_debounce = 5
        # print("self.exploration_trigger_debounce: ", )
        self.iterationCallback = None
        self.robot_iteration_triggered = False

        # Arming
        self.arm_button_pressed = False
        self.system_arm_state_at_press = False
        self.system_armed = False

        # Exploration
        self.exploration_button_pressed = False
        self.exploration_button_press_time = 0.0

        # Convoy
        self.convoy_mode_change = 0

        # System Feedback
        # self.arm_state_sub = rospy.Subscriber("executive/armed", Bool, self.armedCallback)
       # General
        self.active_robot = None
        self.target_speed = Float32()
        self.target_speed.data = 2.0
        self.speed_update_time = 0.0
        self.speed_update_gap = 0.5
        self.forming_up = False

        self.joystick_x2_update = False
        
        # Joystick Context Mode and related variables to switch
        self.JoyContextMode = 1 # 1 - Regular control mode, 0 - Mouse Mode
        self.xbox_button_pressed = False
        self.waypoint_plugin_active = False
        
        self.start_key_pressed = False
        self.back_key_pressed = False
        self.rb_pressed = False
        self.lb_pressed = False
        
        self.a_btn_pressed = False
        self.b_btn_pressed = False
        self.x_btn_pressed = False
        self.last_x_btn_time = None
        self.restored_view = False
        self.y_btn_pressed = False
        self.right_js_pressed = False
        self.left_js_pressed = False

        # # Input
        
        
        self.joystick_sub = self.create_subscription(Joy, "joy", self.joyCallback, 1)
        # System Feedback
        self.arm_state_sub = self.create_subscription(Bool, "low_level_control/arm_status", self.armedCallback, 1)

        # # Control Interface
        # self.authority_request_pub = self.create_publisher(String, "command/request_authority", 1)
        # self.joystick_pub = self.create_publisher(Joy, "%s/command/joy" % self.authority_name, 1)
        # self.state_pub = self.create_publisher(UInt8, "%s/command/mode" % self.authority_name, 1)
        # self.arm_pub = self.create_publisher(Bool, "%s/command/arm" % self.authority_name, 1)
        # self.explore_pub = self.create_publisher(Bool, "%s/command/enable_exploration" % self.authority_name, 1)
        self.authority_request_pub = self.create_publisher(String, "command/request_authority", 1)
        self.joystick_pub = self.create_publisher(Joy, "command/joy", 1)
        self.twist_pub = self.create_publisher(TwistStamped, "low_level_control/manual_override", 1)

        self.state_pub = self.create_publisher(UInt8, "command/mode", 1)
        self.arm_pub = self.create_publisher(Bool, "command/arm", 1)
        self.explore_pub = self.create_publisher(Bool, "command/enable_exploration", 1)

        self.get_logger().info('end of init')

    def armedCallback(self, msg):
        self.system_armed = msg.data

    def joyCallback(self, msg):
        # print("callback")
        # self.get_logger().info('My log message')
        bad = False
        if self.max_idx >= len(msg.axes):
            self.get_logger().info("Joystick message does not have enough axes")
            bad = True
        if self.max_button >= len(msg.buttons):
            self.get_logger().info( "Joystick message does not have enough buttons")
            bad = True

        if bad:
            return

        smart_joystick = msg.buttons[self.a_button]
        exploration = msg.buttons[self.x_button]
        waypoint = msg.buttons[self.y_button]
        manual = msg.buttons[self.b_button]

        arm_disarm = msg.buttons[self.rb_button]

        
        control_trigger_a = msg.buttons[self.lb_button]
        control_trigger_b = msg.buttons[self.rb_button]
        
        joystick_x = msg.axes[self.left_joystick_x]
        joystick_y = msg.axes[self.right_joystick_y]
        joystick_lateral = msg.axes[self.right_joystick_x]

        joystick_msg = Joy()
        joystick_msg.axes = [joystick_x, joystick_y, joystick_lateral]
        self.joystick_pub.publish(joystick_msg)
        
        twist_msg = TwistStamped()
        twist_msg.header.stamp = self.get_clock().now().to_msg()
        twist_msg.twist.linear.x = joystick_y
        twist_msg.twist.angular.z = joystick_x        
        self.twist_pub.publish(twist_msg)
        
        # Handle the control triggered (to request authority)
        if control_trigger_a and control_trigger_b:
            self.get_logger().info( "Requesting control authority.")
            authority_request_msg = String()
            authority_request_msg.data = self.authority_name
            self.authority_request_pub.publish(authority_request_msg)

        # # Handle triggering exploration
        # if exploration_trigger:
        #     if not self.exploration_button_pressed:
        #         self.get_logger().info("Starting exploration trigger.")
        #         self.exploration_button_press_time = time.time()
        #     if math.fabs(time.time() - self.exploration_button_press_time) > self.exploration_trigger_debounce:
        #         self.get_logger().info("Triggering exploration")
        #         exploration_trigger_msg = Bool()
        #         exploration_trigger_msg.data = True
        #         self.explore_pub.publish(exploration_trigger_msg)
        #     self.exploration_button_pressed = True
        # else:
        #     self.exploration_button_pressed = False

        # Handle arming / disarming
        if arm_disarm:
            if not self.arm_button_pressed:
                self.system_arm_state_at_press = self.system_armed

            self.get_logger().info( "Attempting to %s"%("arm" if not self.system_arm_state_at_press else "disarm"))
            arm_msg = Bool()
            arm_msg.data = not self.system_arm_state_at_press
            self.arm_pub.publish(arm_msg)

            self.arm_button_pressed = True
        else:
            self.arm_button_pressed = False

        # Request State changing
        mode_request_msg = UInt8()
        valid_request = True
        if manual:
            self.get_logger().info( "Requesting manual mode.")
            mode_request_msg.data = AIRLABModes.MANUAL_MODE
        elif smart_joystick:
            self.get_logger().info("Requesting joystick mode.")
            mode_request_msg.data = AIRLABModes.JOYSTICK_MODE
        elif exploration:
            self.get_logger().info( "Requesting exploration mode.")
            mode_request_msg.data = AIRLABModes.EXPLORATION_MODE
        elif waypoint:
            self.get_logger().info("Requesting waypoint mode.")
            mode_request_msg.data = AIRLABModes.WAYPOINT_MODE
        else:
            valid_request = False
        if valid_request:
            self.state_pub.publish(mode_request_msg)




def main(args=None):
    print("init!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!")
    rclpy.init(args=args)
    node = JoystickCommandSource()
    # try:
    #     rclpy.spin(node)
    # except KeyboardInterrupt:
    #     node.destroy_node()

    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
