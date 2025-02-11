#!/usr/bin/env python3
import tkinter as tk
import tkinter.font as tkFont
import tkinter.messagebox

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64


class SimControllerNode(Node):
    """This node is a simple gui based controller for the simulation."""

    def __init__(self, root):
        super().__init__('simulation_controller')
        self.get_logger().info('Simulation Controller Node Started')
        root.title('Simulation Controller')
        width = 640
        height = 480
        screenwidth = root.winfo_screenwidth()
        screenheight = root.winfo_screenheight()
        allignstr = '%dx%d+%d+%d' % (width, height, (screenwidth - width)/2,
                                     (screenheight - height)/2)
        root.geometry(allignstr)
        root.resizable(False, False)
        self.build_control_elements(root)
        self.build_settings_elements(root)
        self.speed = 0
        self.steering = 0
        self.setup_complete = False

    def build_control_elements(self, root):
        # create a label for the speed
        self.label_speed = tk.Label(root, text='Speed:')
        self.label_speed['font'] = tkFont.Font(family='Times', size=34)
        self.label_speed['fg'] = '#333333'
        self.label_speed['justify'] = 'center'
        self.label_speed.place(x=50, y=310, width=170, height=60)

        # create a steering label below the speed label
        self.label_steering = tk.Label(root, text='Steering:')
        self.label_steering['font'] = tkFont.Font(family='Times', size=34)
        self.label_steering['fg'] = '#333333'
        self.label_steering['justify'] = 'center'
        self.label_steering.place(x=50, y=370, width=170, height=60)

        # create an empty display for the speed next to the speed label
        self.display_speed = tk.Label(root, text='0')
        self.display_speed['font'] = tkFont.Font(family='Times', size=34)
        self.display_speed['fg'] = '#333333'
        self.display_speed['justify'] = 'center'
        self.display_speed.place(x=230, y=310, width=170, height=60)

        # create an empty display for the steering next to the steering label
        self.display_steering = tk.Label(root, text='0')
        self.display_steering['font'] = tkFont.Font(family='Times', size=34)
        self.display_steering['fg'] = '#333333'
        self.display_steering['justify'] = 'center'
        self.display_steering.place(x=230, y=370, width=170, height=60)

    def build_settings_elements(self, root):
        # create a label for the speed topic name
        self.label_speed_topic = tk.Label(root, text='Speed Topic:')
        self.label_speed_topic['font'] = tkFont.Font(family='Times', size=20)
        self.label_speed_topic['fg'] = '#333333'
        self.label_speed_topic['justify'] = 'center'
        self.label_speed_topic.place(x=50, y=100, width=170, height=60)

        # create a label for the steering topic name
        self.label_steering_topic = tk.Label(root, text='Steering Topic:')
        self.label_steering_topic['font'] = tkFont.Font(family='Times', size=20)
        self.label_steering_topic['fg'] = '#333333'
        self.label_steering_topic['justify'] = 'center'
        self.label_steering_topic.place(x=50, y=160, width=170, height=60)

        # create a line edit for the speed topic name next to the speed topic label
        self.edit_speed_topic = tk.Entry(root)
        self.edit_speed_topic['font'] = tkFont.Font(family='Times', size=20)
        self.edit_speed_topic['fg'] = '#333333'
        self.edit_speed_topic.place(x=230, y=100, width=380, height=60)
        self.edit_speed_topic.insert(0, '/set_motor_level_msg')

        # create a line edit for the steering topic name next to the steering topic label
        self.edit_steering_topic = tk.Entry(root)
        self.edit_steering_topic['font'] = tkFont.Font(family='Times', size=20)
        self.edit_steering_topic['fg'] = '#333333'
        self.edit_steering_topic.place(x=230, y=160, width=380, height=60)
        self.edit_steering_topic.insert(0, '/set_steering_level_msg')

        # create a button 'OK' below the steering topic name in the center
        self.button_ok = tk.Button(root, text='OK', command=self.on_ok)
        self.button_ok['font'] = tkFont.Font(family='Times', size=20)
        self.button_ok['fg'] = '#333333'
        self.button_ok.place(x=230, y=220, width=170, height=60)

    def on_ok(self):
        self.speed_topic = self.edit_speed_topic.get()
        self.steering_topic = self.edit_steering_topic.get()
        self.get_logger().info('Speed Topic: ' + self.speed_topic)
        self.get_logger().info('Steering Topic: ' + self.steering_topic)

        if self.speed_topic == '' or self.steering_topic == '':
            # display a message box saying that the speed and/or steering topic name is empty
            tkinter.messagebox.showinfo(
                'Error', 'Speed and/or Steering topic name is empty\nPlease enter the topic names')
        else:
            # create a publisher for the speed topic
            self.speed_publisher_ = self.create_publisher(Float64, self.speed_topic, 10)
            # create a publisher for the steering topic
            self.steering_publisher_ = self.create_publisher(Float64, self.steering_topic, 10)

            # make the edit boxes read only
            self.edit_speed_topic['state'] = 'disabled'
            self.edit_steering_topic['state'] = 'disabled'

            self.setup_complete = True

    # Create a keyboard listener
    def keyboard_listener(self, event):

        if self.setup_complete:
            if event.char == 'w' or event.keysym == 'Up':
                self.speed = self.speed + 20
                self.display_speed.configure(text=str(self.speed))
            elif event.char == 's' or event.keysym == 'Down':
                self.speed = self.speed - 20
                self.display_speed.configure(text=str(self.speed))
            elif event.char == 'a' or event.keysym == 'Left':
                self.steering = self.steering + 15
                self.display_steering.configure(text=str(self.steering))
            elif event.char == 'd' or event.keysym == 'Right':
                self.steering = self.steering - 15
                self.display_steering.configure(text=str(self.steering))
            elif event.char == ' ':
                self.speed = 0
                self.steering = 0
                self.display_speed.configure(text=str(self.speed))
                self.display_steering.configure(text=str(self.steering))

            speed_msg = Float64()
            # cast speed to float
            speed_msg.data = float(self.speed)
            self.speed_publisher_.publish(speed_msg)

            steering_msg = Float64()
            steering_msg.data = float(self.steering)
            self.steering_publisher_.publish(steering_msg)


def main(args=None):
    rclpy.init(args=args)
    root = tk.Tk()
    node = SimControllerNode(root)
    root.bind('<Key>', node.keyboard_listener)

    root.mainloop()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
