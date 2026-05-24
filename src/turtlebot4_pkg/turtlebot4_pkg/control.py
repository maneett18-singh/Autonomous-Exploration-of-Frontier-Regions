# Reference is taken from teleop keyboard twist 
#github repo - https://github.com/rohbotics/ros2_teleop_keyboard/blob/master/teleop_twist_keyboard.py#L1-L138
#Group - D
# Team members 1. Rishabh Umashankar 2. Maneet Singh


import sys
import threading

import geometry_msgs.msg
import rclpy

if sys.platform == 'win32':
    import msvcrt
else:
    import termios
    import tty


msg = """
This is a custom built package and node which takes keypresses from the keyboard and publishes them as Twist/TwistStamped messages.
Reference : teleop_keyboard_twist package

Navigation keys:

Forward 	- w
Recerse 	- s
Left spin 	- a
Right spin 	- d
Right turn 	- e
Left turn 	- q


z to quit
"""

moveBindings = {
    'w': (1, 0, 0, 0),
    's': (-1, 0, 0, 0),
    'e': (1, 0, 0, -1),
    'a': (0, 0, 0, 1),
    'd': (0, 0, 0, -1),
    'q': (1, 0, 0, 1),
}

def getKey(settings):
    if sys.platform == 'win32':
        key = msvcrt.getwch()
    else:
        tty.setraw(sys.stdin.fileno())
        key = sys.stdin.read(1)
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key


def saveTerminalSettings():
    if sys.platform == 'win32':
        return None
    return termios.tcgetattr(sys.stdin)


def restoreTerminalSettings(old_settings):
    if sys.platform == 'win32':
        return
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_settings)


def main():
    settings = saveTerminalSettings()

    rclpy.init()

    node = rclpy.create_node('teleop_twist_keyboard')

    # parameters
    stamped = node.declare_parameter('stamped', False).value
    frame_id = node.declare_parameter('frame_id', '').value
    if not stamped and frame_id:
        raise Exception("'frame_id' can only be set when 'stamped' is True")

    if stamped:
        TwistMsg = geometry_msgs.msg.TwistStamped
    else:
        TwistMsg = geometry_msgs.msg.Twist

    pub = node.create_publisher(TwistMsg, 'cmd_vel', 10)

    spinner = threading.Thread(target=rclpy.spin, args=(node,))
    spinner.start()

    speed = 0.5
    turn = 1.0
    x = 0.0
    y = 0.0
    z = 0.0
    th = 0.0
    status = 0.0

    twist_msg = TwistMsg()

    if stamped:
        twist = twist_msg.twist
        twist_msg.header.stamp = node.get_clock().now().to_msg()
        twist_msg.header.frame_id = frame_id
    else:
        twist = twist_msg

    try:
        print(msg)
        while True:
            key = getKey(settings)
            if key == 'z':
                print("Exiting the program")
                break
            if key in moveBindings.keys():
                x, y, z, th = moveBindings[key]
            else:
                x, y, z, th = 0.0, 0.0, 0.0, 0.0
                if key == '\x03':
                    break

            if stamped:
                twist_msg.header.stamp = node.get_clock().now().to_msg()

            twist.linear.x = x * speed
            twist.linear.y = y * speed
            twist.linear.z = z * speed
            twist.angular.x = 0.0
            twist.angular.y = 0.0
            twist.angular.z = th * turn
            pub.publish(twist_msg)

    except Exception as e:
        print(e)

    finally:
        if stamped:
            twist_msg.header.stamp = node.get_clock().now().to_msg()

        twist.linear.x = 0.0
        twist.linear.y = 0.0
        twist.linear.z = 0.0
        twist.angular.x = 0.0
        twist.angular.y = 0.0
        twist.angular.z = 0.0
        pub.publish(twist_msg)
        rclpy.shutdown()
        spinner.join()

        restoreTerminalSettings(settings)


if __name__ == '__main__':
    main()
