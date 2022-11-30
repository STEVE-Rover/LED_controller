#!/usr/bin/env python3

import rospy
from animations import Animations
from std_msgs.msg import Int8

# Red (state 0): autonomous navigation in progress
# Blue: teleoperation
# Flashing green (state 1): goal reached

COLORS = (0xFF0000, 0x00FF00, 0x0000FF)

class LEDController:
    def __init__(self):
        rospy.init_node("led_controller")

        self.led_state = 0  #0: off, 1: solid red, 2: flashing green, 3: solid blue 
        self.prev_led_state = 0
        self.animations = Animations()
        self.rate = rospy.get_param("~rate", 5)

        rospy.loginfo("led_controller ready")
        rospy.Subscriber("/goal_manager/state", Int8, self.goal_manager_state_cb)

    def goal_manager_state_cb(self, msg):
        self.led_state = msg.data

    def run(self):
        r = rospy.Rate(self.rate)
        while not rospy.is_shutdown():
            if self.prev_led_state != self.led_state:
                # LED state has changed.
                if self.prev_led_state == 2:
                    self.animations.stop_flashing = True
                self.prev_led_state = self.led_state
                if self.led_state == 0:
                    self.animations.solid(0)
                elif self.led_state == 1:
                    self.animations.solid(COLORS[0])
                elif self.led_state == 2:
                    self.animations.flashing(COLORS[1], 2)
                elif self.led_state == 3:
                    self.animations.solid(COLORS[2])
            r.sleep()


if __name__ == '__main__':
    try:
        rospy.loginfo("led_controller ready")
        ledcontroller = LEDController()
        ledcontroller.run()
    except rospy.ROSInterruptException:
        pass