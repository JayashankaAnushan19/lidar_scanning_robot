#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
import sys, select, termios, tty

settings = termios.tcgetattr(sys.stdin)

class TeleopRobot:
    def __init__(self):
        self.pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.sub = rospy.Subscriber('/scan', LaserScan, self.scan_callback)
        self.speed = 0.5
        self.max_speed = 1.0
        self.move_cmd = Twist()
        self.min_distance = 0.01  # Stop if obstacle closer than 0.3m
        self.obstacle_detected = False

    def scan_callback(self, data):
        # Check front 60 degrees (30 degrees left and right)
        front_ranges = data.ranges[len(data.ranges)//4:3*len(data.ranges)//4]
        valid_ranges = [r for r in front_ranges if not (r == float('inf') or r == float('nan'))]
        if valid_ranges and min(valid_ranges) < self.min_distance:
            self.obstacle_detected = True
        else:
            self.obstacle_detected = False

    def getKey(self):
        tty.setraw(sys.stdin.fileno())
        key = None
        try:
            if select.select([sys.stdin], [], [], 0.1)[0]:
                key = sys.stdin.read(1)
                if key == '\x1b':
                    key += sys.stdin.read(2)
        except Exception as e:
            print(f"Error reading key: {e}")
        finally:
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
        return key

    def run(self):
        rospy.init_node('my_teleop_cmds')
        print("")
        print("--- Control Your Bot! ---")
        print("Use 'WASD' or Arrow keys to move the bot")
        print("Press 'Q' or 'P' to quit to main menu.")
        print("Press 'R' to reset speed.")
        print("Press '+' to increase speed, '-' to decrease speed.")
        print("")

        def shutdown_hook():
            self.move_cmd.linear.x = 0
            self.move_cmd.angular.z = 0
            self.pub.publish(self.move_cmd)
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
            print("")
            print(">>> Bot Shutting down !!!.")
            print("")

        rospy.on_shutdown(shutdown_hook)

        try:
            while not rospy.is_shutdown():
                key = self.getKey()
                self.move_cmd.linear.x = 0
                self.move_cmd.angular.z = 0

                if key:
                    if key in ['w', '\x1b[A'] and not self.obstacle_detected:  # Up
                        self.move_cmd.linear.x = self.speed
                        key_display = 'Up   '
                    elif key in ['s', '\x1b[B']:  # Down
                        self.move_cmd.linear.x = -self.speed
                        key_display = 'Down '
                    elif key in ['a', '\x1b[D']:  # Left
                        self.move_cmd.angular.z = self.speed
                        key_display = 'Left '
                    elif key in ['d', '\x1b[C']:  # Right
                        self.move_cmd.angular.z = -self.speed
                        key_display = 'Right'
                    elif key == '+':  # Increase speed
                        self.speed = min(self.max_speed, self.speed + 0.1)
                        print(f"Speed increased to {self.speed:.2f}")
                        self.pub.publish(self.move_cmd)
                        continue
                    elif key == '-':  # Decrease speed
                        self.speed = max(0, self.speed - 0.1)
                        print(f"Speed decreased to {self.speed:.2f}")
                        self.pub.publish(self.move_cmd)
                        continue
                    elif key == 'r':  # Reset speed
                        self.move_cmd.linear.x = 0
                        self.move_cmd.angular.z = 0
                        self.speed = 0.5
                        self.pub.publish(self.move_cmd)
                        print("Robot resetted. !!!!")
                        continue
                    elif key in ['q', 'p']:  # Quit
                        print("")
                        print("Stop Command !!!.")
                        print("Program Exiting...")
                        break
                    else:
                        print("Unrecognized command. Bot stopped!")
                        self.pub.publish(self.move_cmd)
                        continue

                    print(f"Key: {key_display} | Linear Velocity: {self.move_cmd.linear.x:.2f} | Angular Velocity: {self.move_cmd.angular.z:.2f}")
                else:
                    key_display = 'None '

                self.pub.publish(self.move_cmd)

        except Exception as e:
            print(e)

        finally:
            self.move_cmd.linear.x = 0
            self.move_cmd.angular.z = 0
            self.pub.publish(self.move_cmd)
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)

if __name__ == '__main__':
    teleop = TeleopRobot()
    teleop.run()