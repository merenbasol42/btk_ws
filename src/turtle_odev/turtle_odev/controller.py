#!/usr/bin/env python3

import rclpy
import math
from rclpy.node import Node
from my_interfaces.srv import BaitTurtle 
from turtlesim.msg import Pose
from turtlesim.srv import Kill
from geometry_msgs.msg import Twist
from example_interfaces.msg import String


class ControllerNode(Node):
    def __init__(self):
        super().__init__("deneme")
        self.turtles_ = []
        self.target_ = BaitTurtle.Request()
        self.target_.name = "none"
        self.pose_ = Pose()
        self.PI = 3.1415
        
        self.kill_service_ = self.create_client(Kill,"kill")
        # self.create_subscription(BaitTurtle, "turtle_created", self.turtle_created, 10)
        self.create_service(BaitTurtle, "turtle_created", self.turtle_created)
        self.create_subscription(Pose, "turtle1/pose", self.pose_refresh, 10)
        self.turtle_killed_pubber_ = self.create_publisher(String,"turtle_killed",10)
        self.cmd_vel_pubber_ = self.create_publisher(Twist, "turtle1/cmd_vel", 10)

        self.create_timer(0.05, self.move)
        self.create_timer(0.05, self.control_and_kill_target)

    def pose_refresh(self, pose:Pose):
        self.pose_ = pose

    def control_and_kill_target(self):

        if self.target_.name == "none":
            if len(self.turtles_) != 0:
                self.choose_target()
            return
        distance = self.calculate_target_distance(self.target_)

        if distance < 1.0:
            request = Kill.Request()
            request.name = self.target_.name
            self.kill_service_.call_async(request)
            self.turtle_killed_pubber_.publish(String())

            index = 0
            for turtle in self.turtles_:
                if turtle.name == self.target_.name:
                    self.turtles_.pop(index)
                    self.target_.name = "none"
                    self.choose_target()
                    break
                index += 1
        else:
            pass

    def turtle_created(self, request:BaitTurtle.Request, response):
        self.turtles_.append(request)
        return response

    def calculate_target_distance(self,turtle:BaitTurtle.Request):
        akare = pow(abs(self.pose_.x - turtle.x),2)
        bkare = pow(abs(self.pose_.y - turtle.y),2)
        ckare = akare + bkare
        return ckare

    def choose_target(self):
        if len(self.turtles_) == 0:
            self.target_.name = "none"
            return

        distances = []
        for turtle in self.turtles_: #uzaklıklar hesaplanıyor
            distance = self.calculate_target_distance(turtle)
            distances.append(distance)

        min_value = min(distances) #en küçük değeri buluyoz
        index = distances.index(min_value) #en küçük değerdeki değerin indexini öğreniyoz
        self.target_ = self.turtles_[index] #target'a indezteki turtle'ı atıyoruz

    def get_name_list_turtles(self):
        name_list = []
        for turtle in self.turtles_:
            name_list.append(turtle.name)

        return name_list

    def move(self):
        move_twist = Twist()
        # print(self.get_name_list_turtles())
        if len(self.turtles_) == 0:
            move_twist.angular.z = 0.0
            move_twist.linear.x = 0.0
            
        else:
            self.choose_target()

            alpha = self.calculate_alpha() 
            angular_velocity = alpha

            if abs(alpha) < 0.2:
                linear_velocity = 3.0

            elif abs(alpha) < 0.5:
                linear_velocity = 2.0

            elif abs(alpha) < 1.25:
                angular_velocity *= 1.5
                linear_velocity = 1.5

            elif abs(alpha) < 1.75:
                angular_velocity *= 2.0
                linear_velocity = 1.0

            else:
                angular_velocity *= 2.0
                linear_velocity = 0.8

            move_twist.angular.z = angular_velocity
            move_twist.linear.x = linear_velocity

        self.cmd_vel_pubber_.publish(move_twist)

    def calculate_alpha(self):
        x_fark = self.pose_.x - self.target_.x
        y_fark = self.pose_.y - self.target_.y

        # yfark / xfark = eğim = tan(beta) 
        # print("yfark = {}".format(yfark))
        # print("xfark = {}".format(xfark))
        # print("y/x = {}".format(yfark/xfark))

        beta = math.atan(y_fark/x_fark)

        if beta < 0: # math.tan()'dan -90 ile +90 arası çıkıyor ama bizim hesabımız 0 - 180 arası ile hesaplanacak
            beta += 3.1415 # 180 derece eklendi

        theta = self.pose_.theta
        alpha = 0.0


        if theta < 0:
            p_theta = theta + self.PI # 180 derece eklendi
        else:
            p_theta = theta     


        avci_egimi = math.tan(p_theta) # Buradan 90 derce hatası çıkabilir handle'lamayı unutma
        avci_sabiti = self.pose_.y - avci_egimi*self.pose_.x 
        esik_y_degeri = avci_egimi*self.target_.x  + avci_sabiti # hedefin x'inde, avcimizin y degeri

        egim_y_fark = esik_y_degeri- self.target_.y

        # print("avci_egimi: " + str(avci_egimi))
        # print("esik_y_degeri: " + str(esik_y_degeri))

        sagdaMi = False

        if theta > -self.PI/2 and theta < self.PI/2:
            if egim_y_fark > 0:
                sagdaMi = True
            else:
                sagdaMi = False
        #teta > 90 ve theta < 180 )veya( teta > -180 ve teta < -90
        elif (theta > self.PI/2 and theta < self.PI) or (theta > -self.PI and theta < -self.PI/2):
            if egim_y_fark < 0:
                sagdaMi = True
            else:
                sagdaMi = False
            

        alpha = beta - p_theta    

        if alpha < 0 and not sagdaMi:
            alpha += self.PI
        elif alpha > 0 and sagdaMi:
            alpha -= self.PI


        #print("onceki beta = {}".format(beta))
        # print(str(sagdaMi))
        # print("beta = {}".format(beta))
        # print("theta = {}".format(p_theta))
        # print("alpha = {}".format(alpha))
        
        #- si + sı ayarlı merak etme
        return alpha



def main(args = None):
    rclpy.init(args=args)

    node = ControllerNode()
    rclpy.spin(node) # Kenks bu satır olmazsa nasıl çalışsın
    
    rclpy.shutdown()

if __name__ == "__main__":
    main()