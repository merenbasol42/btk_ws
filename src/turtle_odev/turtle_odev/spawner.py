#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from turtlesim.srv import Spawn
from example_interfaces.msg import String
from my_interfaces.srv import BaitTurtle
import random

class SpawnerNode(Node):
    def __init__(self):
        super().__init__("spawner")
        self.namer_count_ = 2
        self.count_ = 0
        
        # self.declare_parameter("x",1.0)
        # self.declare_parameter("y",1.0)

        self.turtle_created_client_ = self.create_client(BaitTurtle, "turtle_created")
        self.turtle_created_client_.wait_for_service()

        self.spawn_client_ = self.create_client(Spawn, "spawn")
        self.spawn_client_.wait_for_service()

        self.create_subscription(String, "turtle_killed", self.turtle_killed, 10)

        self.create_timer(0.5,self.spawn_random)

    def convert_to_bait_turtle(self, request:Spawn.Request):
        msg = BaitTurtle.Request()
        msg.name = request.name
        msg.x = request.x
        msg.y = request.y
        return msg

    def turtle_killed(self,msg):
        self.count_ -= 1

    def spawn_random(self):
        if self.count_ >= 5:
            return
        request = Spawn.Request()
        request.name = "turtle" + str(self.namer_count_)
        request.theta = random.uniform(-3,3)
        request.x = random.uniform(1,10) #self.get_parameter("x").value 
        request.y = random.uniform(1,10) #self.get_parameter("x").value 

        self.spawn_client_.call_async(request)

        # Senkron olarak aramasını istiyorum
        self.turtle_created_client_.call_async(self.convert_to_bait_turtle(request))

        self.namer_count_ += 1
        self.count_ += 1

        print(request.name +" ismi ile spawn oldu")


def main(args = None):
    rclpy.init(args=args)

    node = SpawnerNode()
    rclpy.spin(node) # Kenks bu satır olmazsa nasıl çalışsın
    
    rclpy.shutdown()

if __name__ == "__main__":
    main()