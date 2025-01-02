import time
from threading import Thread
import math
import random

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
from turtlesim.srv import Spawn, Kill

from .config import *

class Position:
    def __init__(self, x: float = 0.0, y: float = 0.0, theta: float = 0.0):
        self.x: float = x
        self.y: float = y
        self.theta: float = theta

    def __sub__(self, other: 'Position') -> 'Position':
        return Position(
            self.x - other.x,
            self.y - self.y,
            self.theta - self.theta
        )
    
    def get_c(self) -> float:
        return math.sqrt(self.x ** 2 + self.y ** 2)

    def calc_diff(self, other: 'Position') -> float:
        sub: 'Position' = other - self
        return sub.get_c()

class Turtle:
    def __init__(self, name: str, posi: Position):
        self.name: str = name
        self.posi: Position = posi

class Velocity:
    def __init__(self, linear: float = 0.0, angular: float = 0.0):
        self.linear: float = linear
        self.angular: float = angular

class OdevNode(Node):
    def __init__(self):
        super().__init__("odev_node")

        self.posi: Position = Position()
        self.vel : Velocity = Velocity()
        self.name_counter: int = 0
        self.alive_list: list[Turtle] = [] 
        self.target: Turtle = None

        self.create_subscription(Pose, "turtle1/pose", self.pose_cb, 10)
        self.cmd_vel_pubber = self.create_publisher(Twist, "turtle1/cmd_vel", 10)
        self.spawn_client = self.create_client(Spawn, "spawn")
        self.kill_client = self.create_client(Kill, "kill")

        self.wait_for_services()
        self.create_timer(SPAWN_RATE, self.timer_cb)

    def wait_for_services(self):
        while not self.spawn_client.wait_for_service(timeout_sec = 0.1):
            self.get_logger().info("waiting for spawn client")

        while not self.kill_client.wait_for_service(timeout_sec = 0.1):
            self.get_logger().info("waiting for kill client")

    def calc_set_vel(self, dangle: float, diff: float):
        sign_ang = -1 if dangle < 0.0 else 1
        self.vel.angular = (abs(dangle * ANG_VEL_FACT) + MIN_ANG_VEL) * sign_ang

        # Bizde geri vides yoq
        self.vel.linear = diff * LIN_VEL_FACT + MIN_LIN_VEL 

    def hunt(self):
        if self.target == None: return

        dangle: float = self.calc_dangle()
        diff: float = self.calc_diff()
        self.calc_set_vel(dangle, diff)
        self.pub_cmd_vel()

    def kill_control(self):
        for t in self.alive_list:    
            if abs(self.calc_diff(t)) < ERR_LIN:
                self.kill(t.name)
                if self.target != None:
                    if t.name == self.target.name:
                        self.target = None

                self.vel.linear = 0.0
                self.vel.angular = 0.0
                self.pub_cmd_vel()

    def spin(self):
        while True:
            rclpy.spin_once(self)
            self.kill_control()
            self.find_target()
            if self.target == None: time.sleep(ZZZ_TIME)
            self.hunt()

    def find_target(self):
        if self.target != None: return
        target: Turtle = None
        tdiff: float = 1000
        
        for t in self.alive_list:
            diff = self.posi.calc_diff(t.posi)
        
            if diff < tdiff:
                diff = tdiff
                target = t
        
        self.target = target

    def calc_dangle(self) -> float:
        # Hedef ve avcı kaplumbağanın x, y farkları
        delta_y: float = self.target.posi.y - self.posi.y
        delta_x: float = self.target.posi.x - self.posi.x

        # atan2 ile hedefin açısı hesaplanıyor
        target_angle: float = math.atan2(delta_y, delta_x)

        # Hedefe dönmek için gereken açı farkı
        dangle: float = target_angle - self.posi.theta

        # Açı farkını -PI ile PI arasına normalize ediyoruz
        while dangle > math.pi:
            dangle -= 2 * math.pi
        while dangle < -math.pi:
            dangle += 2 * math.pi

        return dangle
    
    def calc_diff(self, t: Turtle = None) -> float:
        if t == None: t = self.target 
        diff_x: float = t.posi.x - self.posi.x
        diff_y: float = t.posi.y - self.posi.y
        return math.sqrt(diff_x ** 2 + diff_y ** 2)

    def timer_cb(self):
        if len(self.alive_list) == MAX_ALIVE: return

        self.spawn(
            x = float(random.random() * 10 + 1),
            y = float(random.random() * 10 + 1),
        )  

    def pr0tect(self, func):
        try: return func()
        except ZeroDivisionError: return 0.0

    #
    # ROS Messaging Methods
    #

    def pose_cb(self, msg: Pose):
        self.posi.x = msg.x
        self.posi.y = msg.y
        self.posi.theta = msg.theta

    def pub_cmd_vel(self):
        msg = Twist()
        msg.linear.x = self.vel.linear
        msg.angular.z = self.vel.angular
        self.cmd_vel_pubber.publish(msg)

    def spawn(self, x: float, y: float):
        name: str = NAME_PREFIX + str(self.name_counter)
        
        self.spawn_client.call_async(
            Spawn.Request(
                x = x,
                y = y,
                name = name
            )
        )

        self.alive_list.append(
            Turtle(
                name = name,
                posi = Position(
                    x = x,
                    y = y
                )
            )
        )

        self.name_counter += 1

    def kill(self, name: str = None):
        if name == None: name = self.target.name
        self.kill_client.call_async(
            Kill.Request(
                name = name
            )
        )

        for turtle in self.alive_list:
            if turtle.name == name:
                self.alive_list.remove(turtle)
                break

def main(*args):
    rclpy.init()
    node = OdevNode()
    
    try: node.spin()
    except KeyboardInterrupt: pass
    
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()