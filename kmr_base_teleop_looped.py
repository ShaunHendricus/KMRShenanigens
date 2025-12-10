#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Twist
import sys, select, termios, tty
import time

# --- User Settings ---
msg = """
Control Your KMR Mobile Base!
---------------------------
Moving around:
        w    
   a    s    d

w/s : Forward / Backward (Linear X)
a/d : Left / Right (Linear Y - Strafe)
q/e : Rotate Left / Right (Angular Theta)

x   : Force Stop (spacebar also works)
CTRL-C to quit
"""

# Map keys to (x, y, theta, speed_multiplier)
moveBindings = {
    'w': (1, 0, 0, 1),   # +X
    's': (-1, 0, 0, 1),  # -X
    'a': (0, 1, 0, 1),   # +Y (Strafe Left)
    'd': (0, -1, 0, 1),  # -Y (Strafe Right)
    'q': (0, 0, 1, 1),   # +Theta (Rotate Left)
    'e': (0, 0, -1, 1),  # -Theta (Rotate Right)
}

# Speed settings
SPEED_LINEAR = 0.1  # m/s
SPEED_ANGULAR = 0.3 # rad/s


def getKey():
    # This function captures key presses without needing 'Enter'
    tty.setraw(sys.stdin.fileno())
    select.select([sys.stdin], [], [], 0)
    key = sys.stdin.read(1)
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key




counter = 0
print("Im here")



if __name__=="__main__":
    settings = termios.tcgetattr(sys.stdin)
    
    rospy.init_node('kmr_teleop_keyboard')
    pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)

    x = 0
    y = 0
    th = 0
    status = 0

    while counter <= 30:
        #Buying me 20 seconds to get back to desk. Constantly sends zero commands to robot, so it wont kick me out safety.

        x = 0
        y = 0
        th = 0
        status = 0

        twist = Twist()
        twist.linear.x = 0
        twist.linear.y = 0
        twist.linear.z = 0
        twist.angular.x = 0
        twist.angular.y = 0
        twist.angular.z = 0

        pub.publish(twist)

        print("helo")

        counter = counter + 1
        
        time.sleep(1)

    try:
        print(msg)
        while(1):
            key = getKey()
            
            # Identify key press
            if key in moveBindings.keys():
                x = moveBindings[key][0]
                y = moveBindings[key][1]
                th = moveBindings[key][2]
                print(f"Sending: X={x} Y={y} Th={th}")
            elif key == ' ' or key == 'x':
                x = 0
                y = 0
                th = 0
                print("STOP")
            elif (key == '\x03'): # CTRL-C
                break
            else:
                x = 0
                y = 0
                th = 0

            # Create Twist message
            twist = Twist()
            twist.linear.x = x * SPEED_LINEAR
            twist.linear.y = y * SPEED_LINEAR
            twist.linear.z = 0
            twist.angular.x = 0
            twist.angular.y = 0
            twist.angular.z = th * SPEED_ANGULAR

            # Publish the command
            pub.publish(twist)

    except Exception as e:
        print(e)

    finally:
        # Publish a stop command before exiting
        twist = Twist()
        twist.linear.x = 0; twist.linear.y = 0; twist.linear.z = 0
        twist.angular.x = 0; twist.angular.y = 0; twist.angular.z = 0
        pub.publish(twist)

        # Restore terminal settings
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
