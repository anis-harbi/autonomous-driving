from gopigo import *
from collections import namedtuple
from itertools import product
import math
import time
import numpy as np
import matplotlib.pyplot as plt
import matplotlib as mlp
import math
import time
from contextlib import contextmanager
import sys

# helper parameters
Location = namedtuple("Location", "x y theta")

# task parameters
TARGET = 180
APPROACH = 7
START = Location(0, 0, 0)

# robot parameters
SPEED = 55
ENCODER_PER_CM = 18 / 20.42
CORRECTION = 1.29
FORWARD_CORRECTION = 0.65
DPR = 360.0 / 32
LOCATION_ARRAY = []
HITPOINTS = []



# Bug 2
class Bug(object):

    def __init__(self):
        self.servo_position = 104
        self._global_location = START
        set_speed(SPEED)

    def find_endpoint(self, x, y, rotation):
        """
        This function should return a global location given the
        distance and rotation traveled by the Mark 38
        curLocation = [[math.cos(theta), -math.sin(theta), x], [math.sin(theata), math.cos(theta), y], [0, 0, 1]]

        in rotate matrix and transit matrix, the angular rotated
        and distance moved are measured wrt robot coordinate
        """
        rotation = math.radians(rotation)
        theta = math.radians(self.global_location.theta)
        curlocation = [[math.cos(theta), -math.sin(theta), self.global_location.x],
                       [math.sin(theta), math.cos(theta), self.global_location.y],
                       [0, 0, 1]]

        rotateMatrix = [[math.cos(rotation), -math.sin(rotation), 0],
                        [math.sin(rotation), math.cos(rotation), 0],
                            [0, 0, 1]]
        transitMatrix = [[1, 0, x], [0, 1, y], [0, 0, 1]]
        #newLocation = matrix_multiply(matrix_multiply(curlocation, rotateMatrix), transitMatrix)
        newLocation = np.dot(np.dot(curlocation, rotateMatrix), transitMatrix)
        endpoint = Location(newLocation[0][2],
                            newLocation[1][2],
                            self.global_location.theta + math.degrees(rotation))
        # print(endpoint)
        return(endpoint)

    def measure_dist(self):
        DISTANCE_CORRECTION_FACTOR = 1.29
        return round(us_dist(15)/DISTANCE_CORRECTION_FACTOR)

    def move_forward(self, distance):
        revolutions = int(round(distance * ENCODER_PER_CM)) # minus error
        enc_tgt(1, 1, revolutions)
        fwd()
        realDistance = distance - FORWARD_CORRECTION
        self.global_location = self.find_endpoint(realDistance, 0, 0)

    @property
    def global_location(self):
        return self._global_location

    @global_location.setter
    def global_location(self, location):
        LOCATION_ARRAY.append(location)
        if location.theta >= 360:
            self._global_location = Location(location.x, location.y,
                                            location.theta-360)
        elif location.theta < 0:
            self._global_location = Location(location.x, location.y,
                                            location.theta+360)
        else:
            self._global_location = location



    def is_on_mline(self):
        """
        determine whether robot is on mline, if yes, return true, else false
        :return:
        """
        if self.global_location.y <= 2.3 and self.global_location.y >=  -2.3:
            return True
        else:
            return False

    def right_rotate(self, pulse):
        enc_tgt(1, 1, pulse)
        right_rot()

    def left_rotate(self, pulse):
        enc_tgt(1, 1, pulse)
        left_rot()

    def follow_boundary_adjustment(self, too_far, too_close):
        """
        make an adjustment in direction while following the boundary
        :return: nothing
        """
 
        counter_right = 0
        counter_left = 0
        while self.measure_dist() > too_far:
            self.right_rotate(2)
            time.sleep(1)
            counter_right += 1

        while self.measure_dist() < too_close:
            self.left_rotate(1)
            time.sleep(1)
            counter_left += 1
        
        real_left = counter_left * 11.25
        real_right = counter_right * 22.5
        return -real_right  + real_left  


    def facing_goal(self):
        """
        this function will adjust the orientation to make the robot face the goal
        """
        theta = self.gloabal_location.theta
        turn_right(theta)


    def follow_boundary(self, too_far, too_close, advance):
        """
        stop signs:
        self.global_location == goal_location   reach goal
        self.global_location is in hit_list  hit point re encountered
        is_on_mline      if on mline, and distance is shorter, stop
        """

        ###### FIND TRACK
        # turn left until the object cannot be detected, 
        # this can guaranttee no obstacle is ahead when following the boundary
        counter = 0
        while (self.measure_dist() < 20):
            self.left_rotate(1)
            counter += 1
            time.sleep(1)
            
        servo(15)
        time.sleep(1)

        while (self.measure_dist() > too_far or self.measure_dist() < too_close):
            self.left_rotate(1)
            time.sleep(1)
            counter += 1


        real_degree_adjusted = counter * 11.25

        self.global_location = self.find_endpoint(0, 0, real_degree_adjusted)

        ###### ENTER TRACK AND FOLLOW BOUNDARY
        while True:
            self.move_forward(advance)

            time.sleep(1)
            degree_adjusted = self.follow_boundary_adjustment(too_far, too_close)
            self.global_location = self.find_endpoint(0, 0, degree_adjusted)
            #print(self.global_location)


            ###### LEAVE TRACK
            if self.is_on_mline() and (TARGET - self.global_location.x) < (TARGET - HITPOINTS[-1].x) :

                # self.left_rotate(int((360 - self.global_location.theta) // DPR))
                self.left_rotate(int(round((360 - self.global_location.theta) // 11.25)))
                time.sleep(2)
                self.global_location = self.find_endpoint(0, 0, 360 - self.global_location.theta)

                self.is_trapped()

                break
                #self.left_rotate(2)
                #time.sleep(2)
                #print('in is_on_mline', self.global_location)

            if self.is_at_goal():
                print("REACH GOAL")
                # stop the algorithm



    def have_obstacle(self):
        """
        return true if there is an obstacle ahead
        :param self:
        :return:
        """
        if self.measure_dist() <= (TARGET - self.global_location.x):
            return True
        else:
            return False

    def approach_object(self):

        while self.measure_dist() > APPROACH:
            self.move_forward(5)
            time.sleep(1)
        HITPOINTS.append(self.global_location)

    def is_trapped(self):
        if not HITPOINTS:
            return False

        for point in HITPOINTS:
            if math.sqrt((point.x - self.global_location.x)**2 + (point.y - self.global_location.y)**2) <= 10:
                print('Trapped')
                plt.figure()
                plt.plot([location.x for location in LOCATION_ARRAY], [location.y for location in LOCATION_ARRAY])
                plt.show()
                plt.savefig("routine.jpg")
                sys.exit(0)#should end the algorithm
                return True
        return False


    def is_at_goal(self):
        """
        return True if the robot reaches the goal
        """
        if math.sqrt((TARGET - self.global_location.x) ** 2 + self.global_location.y ** 2) >= 5:
            return False
        else:
            print("Reach Destination")
            plt.figure()
            plt.plot([location.x for location in LOCATION_ARRAY], [location.y for location in LOCATION_ARRAY])
            plt.show()
            plt.savefig("routine.jpg")
            sys.exit(0)
            # stop the algorithm

    def run(self):
        #plt.ioff()
        plt.show()
        while True:
            servo(104)
            time.sleep(1)

            if self.have_obstacle():
                self.approach_object()
                self.follow_boundary(15, 7, 5)

                """
                if self.global_location.y < -1:
                    self.left_rotate(4)
                    time.sleep(1)
                    self.global_location = self.find_endpoint(0,0,45)
                    self.move_forward(5)
                    time.sleep(1)
                    self.right_rotate(4)
                    time.sleep(1)
                    self.global_location = self.find_endpoint(0, 0, -45)
                """

            elif self.is_at_goal():
                print('Reach Destination')
                stop()
                time.sleep(1)
                break

            else:
                self.move_forward(5)
                time.sleep(1)

if __name__ == "__main__":
    robot = Bug()
    robot.run()

    plt.figure()
    plt.plot([location.x for location in LOCATION_ARRAY], [location.y for location in LOCATION_ARRAY])
    plt.show()
    plt.savefig("routine.jpg")
