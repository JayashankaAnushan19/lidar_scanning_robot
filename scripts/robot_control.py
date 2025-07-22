#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist
import sys, select, termios, tty

# Save the existing terminal settings
settings = termios.tcgetattr(sys.stdin)

def getKey():
    """Get keyboard input."""
    tty.setraw(sys.stdin.fileno())  # Set terminal to raw mode
    key = None
    try:
        # Take input every 0.1 seconds
        if select.select([sys.stdin], [], [], 0.1)[0]:
            key = sys.stdin.read(1)  # Read one character
            if key == '\x1b':  # Check for arrow key sequence
                key += sys.stdin.read(2)  # Read additional two characters for arrow keys
    except Exception as e:
        print(f"Error reading key: {e}")
    finally:
        # Restore terminal settings
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key

def main():
    rospy.init_node('my_teleop_cmds')
    pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
    
    # Define initial speed and turn speed
    speed = 1.0
    max_speed = 6.0
    move_cmd = Twist()

    print("")
    print("--- Control Your Bot! ---")
    print("Use 'WASD' or Arrow keys to move the bot")
    print("Press 'Q' or 'P' to quit to main menu.")
    print("Press 'R' to reset speed.")
    print("Press '+' to increase speed, '-' to decrease speed.")
    print("")
    print("")
 
    # Shutdown hook to clean up
    def shutdown_hook():
        move_cmd.linear.x = 0
        move_cmd.angular.z = 0
        pub.publish(move_cmd)
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
        print("")
        print(">>> Bot Shutting down !!!.")
        print("")

    rospy.on_shutdown(shutdown_hook)

    try:
        while not rospy.is_shutdown():
            key = getKey()
            move_cmd.linear.x = 0  # Default to stop
            move_cmd.angular.z = 0  # Default to stop

            if key:
                if key in ['w', '\x1b[A']:  # Up (W or Up Arrow)
                    move_cmd.linear.x = speed
                    key_display = 'Up   '
                elif key in ['s', '\x1b[B']:  # Down (S or Down Arrow)
                    move_cmd.linear.x = -speed
                    key_display = 'Down '
                elif key in ['a', '\x1b[D']:  # Left (A or Left Arrow)
                    move_cmd.angular.z = speed
                    key_display = 'Left '
                elif key in ['d', '\x1b[C']:  # Right (D or Right Arrow)
                    move_cmd.angular.z = -speed
                    key_display = 'Right'
                elif key == '+':  # Increase speed
                    speed = min(max_speed, speed + 0.2)
                    print(f"Speed increased to {speed:.2f}")
                    pub.publish(move_cmd)  # Publish stop command
                    continue
                elif key == '-':  # Decrease speed
                    speed = max(0, speed - 0.2)
                    print(f"Speed decreased to {speed:.2f}")
                    pub.publish(move_cmd)  # Publish stop command
                    continue
                elif key == 'r':  # Reset speed
                    move_cmd.linear.x = 0
                    move_cmd.angular.z = 0
                    speed = 1.0
                    pub.publish(move_cmd)
                    
                    print("Robot resetted. !!!!")
                    continue
                elif key in ['q', 'p']:  # Quit
                    print("")
                    print("Stop Command !!!.")
                    print("Program Exiting...")
                    break
                else:
                    print("Unrecognized command. Bot stopped!")
                    pub.publish(move_cmd)  # Publish stop command
                    continue

                # Display movement and speed details for clarity
                print(f"Key: {key_display} | Linear Velocity: {move_cmd.linear.x:.2f} | Angular Velocity: {move_cmd.angular.z:.2f}")
            else:
                # No key pressed, stop the robot
                key_display = 'None '
                # print(f"Key: {key_display} | Linear Velocity: {move_cmd.linear.x:.2f} | Angular Velocity: {move_cmd.angular.z:.2f}")

            # Publish the movement command
            pub.publish(move_cmd)

    except Exception as e:
        print(e)

    finally:
        # Stop the turtle when exiting
        move_cmd.linear.x = 0
        move_cmd.angular.z = 0
        pub.publish(move_cmd)
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)

if __name__ == '__main__':
    main()