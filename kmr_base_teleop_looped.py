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
    rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
    if rlist:
        key = sys.stdin.read(1)
    else:
        key = ''
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key

if __name__=="__main__":
    settings = termios.tcgetattr(sys.stdin)
    
    rospy.init_node('kmr_teleop_keyboard')
    pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)

    # Use a ROS Rate to ensure consistent timing (10 Hz is standard/safe)
    rate = rospy.Rate(10) 

    x = 0
    y = 0
    th = 0
    status = 0

    print("Initialising... Holding connection for 20 seconds.")
    
    # --- FIX 1: Faster 'Heartbeat' Loop ---
    # Instead of counting 30 seconds with sleep(1), we count iterations at 10Hz.
    # 200 iterations * 0.1s = 20 seconds.
    start_time = rospy.Time.now().to_sec()
    while (rospy.Time.now().to_sec() - start_time) < 20.0: 
        # Buying me 20 seconds to get back to desk. 
        # Sends zero commands at 10Hz so safety doesn't trigger.

        twist = Twist()
        twist.linear.x = 0
        twist.linear.y = 0
        twist.linear.z = 0
        twist.angular.x = 0
        twist.angular.y = 0
        twist.angular.z = 0

        pub.publish(twist)
        
        # Removed print to avoid spamming console 10 times a second
        # print("helo") 
        
        rate.sleep() # Sleep for 0.1 seconds (10Hz)

    try:
        print(msg)
        while not rospy.is_shutdown():
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
                # If no key pressed, holding last command or resetting to 0?
                # Standard teleop logic holds state until changed, or resets on release.
                # Assuming hold-until-stop behavior based on your snippet, 
                # but adding a check to avoid stuck keys is safer.
                if key == '':
                    pass # Keep sending last command
                else:
                    x = 0; y = 0; th = 0

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
            
            # Small sleep to prevent CPU hogging in the main loop
            time.sleep(0.1)

    except Exception as e:
        print(e)

    finally:
        # --- FIX 2: Robust Shutdown Sequence ---
        # Instead of sending one stop command, we send a burst for 1 second.
        # This ensures the robot receives the '0' and stops safely before the script dies.
        print("\nSending Stop Command...")
        twist = Twist()
        twist.linear.x = 0; twist.linear.y = 0; twist.linear.z = 0
        twist.angular.x = 0; twist.angular.y = 0; twist.angular.z = 0
        
        for i in range(10): # Send 10 stop messages over 1 second
            pub.publish(twist)
            time.sleep(0.1)

        # Restore terminal settings
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
        print("Script exited cleanly.")
