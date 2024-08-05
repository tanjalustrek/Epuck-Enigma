#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import sys, select, termios, tty
import threading

msg = """
Control Your E-pucks!
---------------------------
Moving around:
   u    i    o
   j    k    l
   m    ,    .

q/z : increase/decrease max speeds by 10%
w/x : increase/decrease only linear speed by 10%
e/c : increase/decrease only angular speed by 10%
space key, k : force stop
1 : switch to E-puck 1
2 : switch to E-puck 2
anything else : stop smoothly

Press 1 to choose Epuck1
or 2 for Epuck2

CTRL-C to quit
"""

moveBindings = {
    'i':(1,0),
    'o':(1,-1),
    'j':(0,1),
    'l':(0,-1),
    'u':(1,1),
    ',':(-1,0),
    '.':(-1,1),
    'm':(-1,-1),
}

speedBindings={
    'q':(1.1,1.1),
    'z':(.9,.9),
    'w':(1.1,1),
    'x':(.9,1),
    'e':(1,1.1),
    'c':(1,.9),
}

def getKey(settings):
    tty.setraw(sys.stdin.fileno())
    rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
    if rlist:
        key = sys.stdin.read(1)
    else:
        key = ''
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key

class TeleopEpuck(Node):

    def __init__(self, robot_name):
        super().__init__(f'teleop_epuck_{robot_name}')
        self.robot_name = robot_name
        self.publisher_ = self.create_publisher(Twist, f'/{robot_name}/cmd_vel', 10)
        self.speed = 0.2
        self.turn = 1.0
        self.x = 0.0
        self.th = 0.0
        self.status = 0

    def process_key(self, key):
        if key in moveBindings.keys():
            self.x = moveBindings[key][0]
            self.th = moveBindings[key][1]
        elif key in speedBindings.keys():
            self.speed = self.speed * speedBindings[key][0]
            self.turn = self.turn * speedBindings[key][1]
            print(f"Speed change: speed={self.speed:.2f}, turn={self.turn:.2f}")
        elif key == ' ' or key == 'k' :
            self.x = 0.0
            self.th = 0.0
        else:
            self.x = 0.0
            self.th = 0.0
        return False

    def publish_twist(self):
        twist = Twist()
        twist.linear.x = self.speed * self.x
        twist.angular.z = self.turn * self.th
        self.publisher_.publish(twist)

    def publish_stop_twist(self):
        twist = Twist()
        self.publisher_.publish(twist)
def main(args=None):
    rclpy.init(args=args)
    epuck1 = TeleopEpuck("epuck1")
    epuck2 = TeleopEpuck("epuck2")
    
    current_epuck = epuck1
    print(msg)
    
    settings = termios.tcgetattr(sys.stdin)
    
    try:
        while True:
            key = getKey(settings)
            if key == '\x03':
                break
            elif key == '1':
                current_epuck = epuck1
                print("Switched to E-puck 1")
            elif key == '2':
                current_epuck = epuck2
                print("Switched to E-puck 2")
            else:
                current_epuck.process_key(key)
            current_epuck.publish_twist()
            
    except Exception as e:
        print(f"Error occurred: {e}")
    
    finally:
        epuck1.publish_stop_twist()
        epuck2.publish_stop_twist()
        epuck1.destroy_node()
        epuck2.destroy_node()
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
        rclpy.shutdown()

if __name__ == '__main__':
    main()