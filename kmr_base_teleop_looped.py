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
    # Modified to be non-blocking so the loop keeps running
    tty.setraw(sys.stdin.fileno())
    # Timeout of 0 makes it check instantly and not block
    rlist, _, _ = select.select([sys.stdin], [], [], 0)
    if rlist:
        key = sys.stdin.read(1)
    else:
        key = ''
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

    # FIX 1: Faster Startup Loop (10Hz)
    # We loop 200 times with 0.1s sleep = 20 seconds duration
    while counter <= 200:
        # Buying me 20 seconds to get back to desk. 
        # Sends zero commands constantly so safety wont kick me out.

        twist = Twist()
        twist.linear.x = 0
        twist.linear.y = 0
        twist.linear.z = 0
        twist.angular.x = 0
        twist.angular.y = 0
        twist.angular.z = 0

        pub.publish(twist)

        # print("helo") # Commented out to prevent console spam
        
        counter = counter + 1
        time.sleep(0.1) # Sleep for 0.1s (10Hz) instead of 1.0s

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
            elif key != '': 
                # Only reset if a key WAS pressed but is unknown
                # If key is empty (no input), we keep the previous command (Toggle Mode)
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
            
            # FIX 2: Main Loop Frequency
            # Sleep 0.1s to maintain 10Hz heartbeat during operation
            time.sleep(0.1)

    except Exception as e:
        print(e)

    finally:
        # FIX 3: Robust Shutdown
        # Send a burst of stop commands to ensure the robot gets it
        print("\nStopping robot...")
        twist = Twist()
        twist.linear.x = 0; twist.linear.y = 0; twist.linear.z = 0
        twist.angular.x = 0; twist.angular.y = 0; twist.angular.z = 0
        
        for i in range(10): # Send 10 times over 1 second
            pub.publish(twist)
            time.sleep(0.1)

        # Restore terminal settings
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
        print("Done.")
