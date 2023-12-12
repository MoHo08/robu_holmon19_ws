#Datein importieren fürs senden und empfangen von Ros Datein.................................................
import math
import rclpy                                                        #Ros schnittstelle für Python
from rclpy.node import Node
from std_msgs.msg import String                                                
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan

from rclpy.qos import qos_profile_sensor_data

import numpy as np
from enum import IntEnum
#.............................................................................................................


#Globale Variablen............................................................................................
ROBOT_DIRECTION_FRONT_INDEX = 0
ROBOT_DIRECTION_LEFT_FRONT_INDEX = 60
ROBOT_DIRECTION_LEFT_INDEX = 90
ROBOT_DIRECTION_LEFT_REAR_INDEX = 120
ROBOT_DIRECTION_REAR_INDEX = 180
ROBOT_DIRECTION_RIGHT_REAR_INDEX = 240
ROBOT_DIRECTION_RIGHT_INDEX = 270
ROBOT_DIRECTION_RIGHT_FRONT_INDEX = 300
#.............................................................................................................


#Zustände definieren..........................................................................................
class WallFollowerStates(IntEnum):
    WF_STATE_INVALID = -1,
    WF_STATE_DETECTWALL = 0,
    WF_STATE_DRIVE2WALL = 1,
    WF_STATE_ROTATE2WALL = 2,
    WF_STATE_FOLLOWWALL = 3
#.............................................................................................................


#Node.........................................................................................................
class WallFollower(Node):

    #+++++ Konstruktor +++++
    def __init__ (self):                                            
        super().__init__('Wallfollower')
        self.scan_subscriber = self.create_subscription(LaserScan, "/scan", self.scan_callback, qos_profile_sensor_data)
        self.cmd_vel_publisher = self.create_publisher(Twist, "/cmd_vel", 10)

        #+++++ Variablen +++++
        self.left_dist = 999999.9 # Left
        self.leftfront_dist = 999999.9 # Left-front
        self.front_dist = 999999.9 # Front
        self.rightfront_dist = 999999.9 # Right-front
        self.right_dist = 999999.9 # Right
        self.rightrear_dist = 999999.9
        self.rear_dist = 999999.9 # Rear
        self.leftrear_dist = 999999.9

        self.distances_history = []
        self.distances_histroy_size = 1

        self.wallfollower_state = WallFollowerStates.WF_STATE_INVALID

        self.forward_speed_wf_slow = 0.05 #m/s
        self.forward_speed_wf_fast = 0.1  #m/s

        self.turning_speed_wf_slow = 0.1  #rad/s
        self.turning_speed_wf_fast = 1.0  #rad/s

        self.dist_thresh_wf = 0.3         #m
        self.dist_hysteresis_wf = 0.02    #m
        self.dist_laser_offset = 0.03     #m
        self.minimum_distance_laser = 0.1

        self.timer = self.create_timer(0.2, self.timer_callback)

    #Timer Callback 
    def timer_callback(self):
        if len(self.distances_history) > 0:
            self.follow_wall()

    #Distances History
    def get_dist_avg_history(self, pos):
        sum = 0.0
        number_of_valid_values = 0

        for distances in self.distances_history:
            if (distances[pos] > self.minimum_distance_laser):
                sum = sum + distances[pos]
                number_of_valid_values = number_of_valid_values +1
        
        if (number_of_valid_values > 0):
            return sum / number_of_valid_values
        else:
            return -1


    #Regler, State-Maschine
    def follow_wall(self):

        #neue Nachricht anlegen
        msg = Twist()
        #Geschwindigkeiten null setzen
        msg.linear.x = 0.0
        msg.linear.y = 0.0
        msg.linear.z = 0.0

        msg.angular.x = 0.0
        msg.angular.y = 0.0
        msg.angular.z = 0.0


        #State Invalid
        if self.wallfollower_state == WallFollowerStates.WF_STATE_INVALID:
            print("WF_STATE_DETECTWALL")
            self.wallfollower_state = WallFollowerStates.WF_STATE_DETECTWALL


        #State Detectwall
        elif self.wallfollower_state == WallFollowerStates.WF_STATE_DETECTWALL:
            print("\nDetect Wall\n")
            dist_min = min(self.distances_history[-1])
            #links drehen bis zum kleinsten Abstand zu einer Wand
            if (self.front_dist) > (dist_min):
                #wenn er knapp bei der wand ist dreh langsam
                if abs((self.front_dist - dist_min)) < 0.2:
                    msg.angular.z = self.turning_speed_wf_slow
                #sonst schnell
                else:
                    msg.angular.z = self.turning_speed_wf_fast
            else:
                print("WF_STATE_DRIVE2WALL")
                self.wallfollower_state = WallFollowerStates.WF_STATE_DRIVE2WALL


        #State Drive2Wall
        elif self.wallfollower_state == WallFollowerStates.WF_STATE_DRIVE2WALL:
            print("\nDrive to Wall\n")
            fd_thresh = self.dist_thresh_wf + self.dist_laser_offset
            forward_speed_wf = self.calc_linear_speed()
            if self.front_dist > (fd_thresh + self.dist_hysteresis_wf):
                msg.linear.x = forward_speed_wf
            elif self.front_dist < (fd_thresh - self.dist_hysteresis_wf):
                msg.linear.x = -forward_speed_wf
            else:
                turn_direction = self.align_front()
                msg.angular.z = self.turning_speed_wf_slow * turn_direction
                if turn_direction == 0:
                    print("WF_STATE_ROTATE2WALL")
                    #safe the current distances as input for the state ROTATE2WALL
                    self.wallfollower_state_input_dist = self.self.distances_history[-1]
                    self.wallfollower_state = WallFollowerStates.WF_STATE_ROTATE2WALL


        #State Rotate2Wall
        elif self.wallfollower_state == WallFollowerStates.WF_STATE_ROTATE2WALL:
            print("\nRotate to Wall\n")
            sr = self.wallfollower_state_input_dist[ROBOT_DIRECTION_RIGHT_INDEX]
            if((sr != np.inf) and (abs(self.front_dist - self.dist_laser_offset - sr) > 0.05)) or ((self.front_dist !=  np.inf) and (sr ==np.inf)):
                msg.angular.z = -self.turning_speed_wf_fast
            else:
                turn_direction = self.align_left()
                msg.angular.z = self.turning_speed_wf_slow * turn_direction
                if turn_direction == 0:
                    print("WF_STATE_FOLLOWWALL")
                    self.wallfollower_state = WallFollowerStates.WF_STATE_FOLLOWWALL

        #State FollowWall
        elif self.wallfollower_state == WallFollowerStates.WF_STATE_FOLLOWWALL:
            print("\nFollow Wall\n")
            fd_thresh = self.dist_thresh_wf + self.dist_laser_offset
            rd_thresh = self.dist_thresh_wf

            lf = self.leftfront_dist
            lr = self.leftrear_dist

            forward_speed_wf = self.calc_linear_speed()

            if (self.front_dist > (fd_thresh + self.dist_hysteresis_wf)):
                #Nach Links lenken -> Abstand wurde zu groß
                if self.left_dist > (rd_thresh + self.dist_hysteresis_wf):
                    #Verhindert ein Aufschwingen, 
                    #Vergleichswert self.dist_hysteresis_wf ist nicht optimal
                    #-> eigenen Parameter erstellen und Winkel vergleichen!
                    if (lr - lf) < self.dist_hysteresis_wf:
                        msg.angular.z = self.turning_speed_wf_slow
                    msg.linear.x = forward_speed_wf
                #Nach Rechts lenken -> Abstand wurde unterschritten
                elif self.left_dist < (rd_thresh - self.dist_hysteresis_wf):
                    if (lf - lr) < self.dist_hysteresis_wf:
                        msg.angular.z = -self.turning_speed_wf_slow
                    msg.linear.x = forward_speed_wf
                #Geradeaus fahren
                else:
                    msg.linear.x = forward_speed_wf
            else: #Wand oder Ecke erreicht!
                turn_direction = self.align_left()
                msg.angular.z = self.turning_speed_wf_slow * turn_direction
                if turn_direction == 0:
                    print("WF_STATE_ROTATE2WALL")
                    self.wallfollower_state_input_dist = self.distances_history[-1]
                    self.wallfollower_state = WallFollowerStates.WF_STATE_ROTATE2WALL
        else:
            self.wallfollower_state = WallFollowerStates.WF_STATE_DETECTWALL




        #Geschwindigkeiten rausschicken
        print(msg)
        self.cmd_vel_publisher.publish(msg)


    #Methode um Geschwindigkeit (langsam oder schnell) zu berechnen
    def calc_linear_speed(self):
        fd_thresh = self.dist_thresh_wf + self.dist_laser_offset
        if self.front_dist > (1.2 * fd_thresh):
            forward_speed_wf = self.forward_speed_wf_fast
        else:
            forward_speed_wf = self.forward_speed_wf_slow

        return forward_speed_wf
    

    #Methode um gerade zu Wand ausrichten
    def align_front(self):
        fl = self.distances_history[-1][ROBOT_DIRECTION_LEFT_FRONT_INDEX]
        fr = self.distances_history[-1][ROBOT_DIRECTION_RIGHT_FRONT_INDEX]
        if ( (fr - fl) > self.dist_hysteresis_wf ):
            return 1    #turning left
        elif ( (fl - fr) > self.dist_hysteresis_wf ):
            return -1   #truning right
        else:
            return 0    #aligned
        
    #Methode um paralell zu Wand ausrichten
    def align_left(self):
        lf = self.distances_history[-1][ROBOT_DIRECTION_LEFT_FRONT_INDEX]
        lr = self.distances_history[-1][ROBOT_DIRECTION_LEFT_REAR_INDEX]
        if (lf - lr) > self.dist_hysteresis_wf:
            return 1    #turning left
        elif (lr - lf) > self.dist_hysteresis_wf:
            return -1   #truning right
        else:
            return 0    #aligned

    #Methode um den Winkel zur Wand auszurechnen    
    def calc_left_wall_angle(self):

        LEFT_ANGLE_DEG = 10
        LEFT_ANGLE_RAD =   (math.pi/180.0) * LEFT_ANGLE_DEG
        
        left_front_dist = self.get_dist_avg_history(ROBOT_DIRECTION_LEFT_INDEX-LEFT_ANGLE_DEG)
        left_rear_dist = self.get_dist_avg_history(ROBOT_DIRECTION_LEFT_INDEX+LEFT_ANGLE_DEG)

        dx = math.sin(LEFT_ANGLE_RAD) * (left_front_dist - left_rear_dist)
        dy = math.cos(LEFT_ANGLE_RAD) * (left_front_dist + left_rear_dist)

        left_wall_angle = math.atan2(dy, dx)
        #if (left_wall_angle > 0):
        #    left_wall_angle = ((math.pi/180) * 90) - left_wall_angle


    #Scan Callback, Lidar Daten holen
    def scan_callback(self, msg):

        self.distances_history.append(msg.ranges)
        if (len(self.distances_history) > self.distances_histroy_size):
            self.distances_history = self.distances_history[1:]
        
        self.left_dist = self.get_dist_avg_history(ROBOT_DIRECTION_LEFT_INDEX)
        self.leftfront_dist = self.get_dist_avg_history(ROBOT_DIRECTION_LEFT_FRONT_INDEX)
        self.front_dist = self.get_dist_avg_history(ROBOT_DIRECTION_FRONT_INDEX)
        self.rightfront_dist = self.get_dist_avg_history(ROBOT_DIRECTION_RIGHT_FRONT_INDEX)
        self.right_dist = self.get_dist_avg_history(ROBOT_DIRECTION_RIGHT_INDEX)
        self.rightrear_dist = self.get_dist_avg_history(ROBOT_DIRECTION_RIGHT_REAR_INDEX)
        self.rear_dist = self.get_dist_avg_history(ROBOT_DIRECTION_REAR_INDEX)
        self.leftrear_dist = self.get_dist_avg_history(ROBOT_DIRECTION_LEFT_REAR_INDEX)

        #print("left: %.2f m\n" %self.left_dist,
        #      "left front: %.2f m\n" %self.leftfront_dist,
        #      "front: %.2f m\n" %self.front_dist,
        #      "right front: %.2f m\n" %self.rightfront_dist,
        #      "r: %.2f m\n" %self.right_dist,
        #      "rear: %.2f m\n" %self.rear_dist,
        #      "\n")
    
#.............................................................................................................


#Main.........................................................................................................
def main(args=None):
    rclpy.init(args=args)
    wallfollower = WallFollower()

    rclpy.spin(wallfollower)

    wallfollower.destroy_node
    rclpy.shutdown
#..............................................................................................................

if __name__ == '__main__':
    main()