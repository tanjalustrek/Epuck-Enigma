#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import sys, select, termios, tty
import threading

msg = """
Control Your E-puck!
---------------------------
Moving around:
   u    i    o
   j    k    l
   m    ,    .

q/z : increase/decrease max speeds by 10%
w/x : increase/decrease only linear speed by 10%
e/c : increase/decrease only angular speed by 10%
space key, k : force stop
anything else : stop smoothly

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

    def __init__(self):
        super().__init__('teleop_epuck')
        self.publisher_ = self.create_publisher(Twist, '/epuck/cmd_vel', 10)
        self.speed = 0.2
        self.turn = 1.0
        self.x = 0.0
        self.th = 0.0
        self.status = 0
        self.interactive_mode = sys.stdin.isatty()

    def run(self):
        if self.interactive_mode:
            self._run_interactive()
        else:
            self._run_non_interactive()

    def _run_interactive(self):
        settings = termios.tcgetattr(sys.stdin)
        print(msg)
        
        try:
            while True:
                key = getKey(settings)
                if self._process_key(key):
                    break
                self._publish_twist()
        finally:
            self._publish_stop_twist()
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)

    def _run_non_interactive(self):
        def input_thread():
            while rclpy.ok():
                key = input("Enter command (i/j/k/l/q): ")
                self._process_key(key)

        threading.Thread(target=input_thread, daemon=True).start()

        try:
            while rclpy.ok():
                rclpy.spin_once(self, timeout_sec=0.1)
                self._publish_twist()
        finally:
            self._publish_stop_twist()

    def _process_key(self, key):
        if key in moveBindings.keys():
            self.x = moveBindings[key][0]
            self.th = moveBindings[key][1]
        elif key in speedBindings.keys():
            self.speed = self.speed * speedBindings[key][0]
            self.turn = self.turn * speedBindings[key][1]
            print(f"currently:\tspeed {self.speed:.2f}\tturn {self.turn:.2f} ")
            if (self.status == 14):
                print(msg)
            self.status = (self.status + 1) % 15
        elif key == ' ' or key == 'k' :
            self.x = 0.0
            self.th = 0.0
        elif key == '\x03':
            return True
        else:
            self.x = 0.0
            self.th = 0.0
        return False

    def _publish_twist(self):
        twist = Twist()
        twist.linear.x = self.speed * self.x
        twist.angular.z = self.turn * self.th
        self.publisher_.publish(twist)

    def _publish_stop_twist(self):
        twist = Twist()
        self.publisher_.publish(twist)

def main(args=None):
    rclpy.init(args=args)
    teleop_epuck = TeleopEpuck()
    try:
        teleop_epuck.run()
    except Exception as e:
        print(e)
    finally:
        teleop_epuck.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()