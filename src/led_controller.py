#!/usr/bin/env python

import rospy
from std_msgs.msg import Int8
from geometry_msgs.msg import Twist

# Red (state 0): autonomous navigation in progress
# Blue: teleoperation
# Flashing green (state 1): goal reached

class LEDController:
    def __init__(self):
        rospy.init_node("led_controller")

        self.time_thresh = rospy.get_param("~time_thresh", 0.5)
        self.rate = rospy.get_param("~rate", 5)
        self.latest_goal_manager_state = (None, rospy.Time(0))  # (Int, Time())
        self.latest_gamepad_cmd_vel = rospy.Time(0)  # Time()

        rospy.loginfo("led_controller ready")
        
        rospy.Subscriber("/goal_manager/state", Int8, self.goal_manager_state_cb)
        rospy.Subscriber("/gamepad_cmd_vel", Twist, self.gamepad_cmd_vel_cb)

    def goal_manager_state_cb(self, msg):
        self.latest_goal_manager_state = (msg.data, rospy.Time.now())

    def gamepad_cmd_vel_cb(self, msg):
        self.latest_gamepad_cmd_vel = rospy.Time.now()

    def run(self):
        r = rospy.Rate(self.rate)
        while not rospy.is_shutdown():
            time_since_latest_state = (rospy.Time.now() - self.latest_goal_manager_state[1]).to_sec()
            time_since_latest_cmd_vel = (rospy.Time.now() -  self.latest_gamepad_cmd_vel).to_sec()
            if  time_since_latest_cmd_vel < self.time_thresh:
                # Teleoperation
                print("Blue light")
            elif time_since_latest_state < self.time_thresh:
                if self.latest_goal_manager_state[0] == 0:
                    # Autonomous navigation in progress
                    print("Red light")
                elif self.latest_goal_manager_state[0] == 1:
                    # Goal reached
                    print("Flashing green light")
            else:
                print("No light")
            r.sleep()


if __name__ == '__main__':
    try:
        ledcontroller = LEDController()
        ledcontroller.run()
    except rospy.ROSInterruptException:
        pass