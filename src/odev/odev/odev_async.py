import time
from threading import Thread
import math
import random

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
from turtlesim.srv import Spawn, Kill

PI: float = 3.141592

ZZZ_TIME: float = 0.05

MAX_ALIVE: int = 3
SPAWN_RATE: float = 2.0
NAME_PREFIX: str = "t"

ERR_ANG: float = 0.001
ERR_LIN: float = 0.5

ANG_VEL_FACT: float = 2.0
MIN_ANG_VEL: float = 0.3
LIN_VEL_FACT: float = 0.5
MIN_LIN_VEL: float = 1.0    

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
        Thread(target=self.hunting_loop).start()

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
        
        # print("hunt start")
        # print("angle start")
        # print(f"target is: {self.target.name}")
        
        
        dangle: float = self.calc_dangle()
        diff: float = self.calc_diff()

        while abs(diff) > ERR_LIN:
            self.calc_set_vel(dangle, diff)
            self.pub_cmd_vel()
            time.sleep(ZZZ_TIME)
            
            dangle: float = self.calc_dangle()
            diff: float = self.calc_diff()

        self.kill()

        self.vel.linear = 0.0
        self.vel.angular = 0.0
        self.pub_cmd_vel()
        
    def hunting_loop(self):
        while True:
            self.find_target()
            self.hunt()
            print("done")
            time.sleep(ZZZ_TIME)

    def find_target(self):
        print("finding target")
        target: Turtle = None
        tdiff: float = 1000
        
        for t in self.alive_list:
            diff = self.posi.calc_diff(t.posi)
        
            if diff < tdiff:
                diff = tdiff
                target = t
        
        self.target = target

    def calc_dangle(self) -> float:
        # diff_angle: dönülmek istenen açı
        # alpha: avcının x ekseni ile olan açısı
        # beta: kurbanın x ekseni ile olan açısı
        # self.posi.theta: avcının kendi açısı (alpha ile aralarındaki fark yön tayini)
        # diff_y: kurbanın ile avcının arasındaki y eksenince fark
        
        delta_y: float = self.target.posi.y - self.posi.y
        delta_x: float = self.target.posi.x - self.posi.x 

        def calc_beta() -> float:
            beta: float = self.pr0tect(
                lambda: math.atan(delta_y / delta_x)
            )
            beta = beta + (1 * PI if beta < 0.0 else 0.0)
            return beta

        def calc_alpha() -> float:
            return self.posi.theta + (PI if self.posi.theta < 0.0 else 0.0)

        beta: float = calc_beta() 
        alpha: float = calc_alpha()

        def is_front() -> bool:
            _theta = self.posi.theta + beta - alpha
            if _theta > PI: _theta -= 2 * PI

            if delta_y == 0.0:
                if _theta == 0.0:
                    return delta_x > 0.0
                else:
                    self.get_logger().fatal("hay at bitti")
                    return None
            else:
                return delta_y * _theta > 0.0

        if is_front():
            # print("önümde")
            return beta - alpha
        
        else:
            # print("arkamda")
            if beta - alpha < 0.0:
                return beta - alpha + PI
            else:
                return beta - alpha - PI

    def calc_diff(self) -> float:
        diff_x: float = self.target.posi.x - self.posi.x
        diff_y: float = self.target.posi.y - self.posi.y
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
    
    try: rclpy.spin(node)
    except KeyboardInterrupt: pass
    
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()