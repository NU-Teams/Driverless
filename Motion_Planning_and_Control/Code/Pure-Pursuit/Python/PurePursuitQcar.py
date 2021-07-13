# -*- coding: utf-8 -*-
"""
Created on Mon Mar 29 22:31:35 2021

@author: user
"""

from Quanser.product_QCar import QCar
from Quanser.q_ui import gamepadViaTarget
from Quanser.q_misc import Calculus
from Quanser.q_interpretation import basic_speed_estimation
import os
import time
import struct
import numpy as np 
import struct
import math
import pandas as pd

class State:

    def __init__(self, x=0.0, y=0.0, yaw=0.0, v=0.0):
        self.x = x
        self.y = y
        self.yaw = yaw
        self.v = v
        self.rear_x = self.x - ((WB / 2) * math.cos(self.yaw))
        self.rear_y = self.y - ((WB / 2) * math.sin(self.yaw))

    def update(self, a, delta):
        beta = math.atan(l_r/(l_f + l_r)*math.tan(delta))
        self.x += self.v * math.cos(self.yaw + beta) * dt
        self.y += self.v * math.sin(self.yaw + beta) * dt
        self.yaw += self.v / l_r * math.tan(beta) * dt
        self.v += a / m * dt
        self.rear_x = self.x - ((WB / 2) * math.cos(self.yaw))
        self.rear_y = self.y - ((WB / 2) * math.sin(self.yaw))

    def calc_distance(self, point_x, point_y):
        dx = self.rear_x - point_x
        dy = self.rear_y - point_y
        return math.hypot(dx, dy)


class States:

    def __init__(self):
        self.x = []
        self.y = []
        self.yaw = []
        self.v = []
        self.t = []

    def append(self, t, state):
        self.x.append(state.x)
        self.y.append(state.y)
        self.yaw.append(state.yaw)
        self.v.append(state.v)
        self.t.append(t)


def proportional_control(target, current):
    a = Kp * (target - current)  / dt

    return a


class TargetCourse:

    def __init__(self, cx, cy):
        self.cx = cx
        self.cy = cy
        self.old_nearest_point_index = None

    def search_target_index(self, state):

        # To speed up nearest point search, doing it at only first time.
        if self.old_nearest_point_index is None:
            # search nearest point index
            dx = [state.rear_x - icx for icx in self.cx]
            dy = [state.rear_y - icy for icy in self.cy]
            d = np.hypot(dx, dy)
            ind = np.argmin(d)
            self.old_nearest_point_index = ind
        else:
            ind = self.old_nearest_point_index
            distance_this_index = state.calc_distance(self.cx[ind],
                                                      self.cy[ind])
            while True:
                distance_next_index = state.calc_distance(self.cx[ind + 1],
                                                          self.cy[ind + 1])
                if distance_this_index < distance_next_index:
                    break
                ind = ind + 1 if (ind + 1) < len(self.cx) else ind
                distance_this_index = distance_next_index
            self.old_nearest_point_index = ind

        Lf = k * state.v + Lfc  # update look ahead distance

        # search look ahead target point index
        while Lf > state.calc_distance(self.cx[ind], self.cy[ind]):
            if (ind + 1) >= len(self.cx):
                break  # not exceed goal
            ind += 1

        return ind, Lf


def pure_pursuit_steer_control(state, trajectory, pind):
    ind, Lf = trajectory.search_target_index(state)

    if pind >= ind:
        ind = pind

    if ind < len(trajectory.cx):
        tx = trajectory.cx[ind]
        ty = trajectory.cy[ind]
    else:  # toward goal
        tx = trajectory.cx[-1]
        ty = trajectory.cy[-1]
        ind = len(trajectory.cx) - 1

    alpha = math.atan2(ty - state.rear_y, tx - state.rear_x) - state.yaw

    delta = math.atan2(2.0 * WB * math.sin(alpha) / Lf, 1.0)

    return delta, ind

def norm(data, a, b):
    output = a + ((data + 1) * (b - a) / (1 + 1))
    return output
# -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- 
## Timing Parameters and methods 
startTime = time.time()
def elapsed_time():
    return time.time() - startTime

sampleRate = 50
sampleTime = 1/sampleRate
simulationTime = 60.0
print('Sample Time: ', sampleTime)

# Additional parameters
counter = 0

# Initialize motor command array
mtr_cmd = np.array([0,0])

# Set up a differentiator to get encoderSpeed from encoderCounts
diff = Calculus().differentiator_variable(sampleTime)
_ = next(diff)
timeStep = sampleTime

# -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- 
# QCar Initialization
myCar = QCar()




# -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- 
# Reset startTime before Main Loop
startTime = time.time()

# Parameters
k = 0.1  # look forward gain
Lfc = 1.0  # [m] look-ahead distance Higher = bigger overshoot less than 2 leads to ocilation
Kp = 0.1  # speed proportional gain
dt = 1  # [s] time tick
WB = 3  # [m] wheel base of vehicle
l_r = 1.5
l_f = 1.5
m = 255
flag = 0
#custom track
#mx = pd.read_csv('x.csv')
#my = pd.read_csv('y.csv')
#cx = np.array(mx)
#cy = np.array(my)
#Handling track
mxy = pd.read_csv('handling_track.csv')
carray = np.array(mxy)
cx = np.concatenate((carray[:,0], carray[:,0].T))
cy = np.concatenate((carray[:,1], carray[:,1].T))

# initial state
state = State(x=-0.0, y=-3.0, yaw=90.0, v=0.0)

lastIndex = len(cx) - 1
target_speed = 2.0 / 3.6  # [km/h] to [m/s]
states = States()
states.append(time, state)
target_course = TargetCourse(cx, cy)
target_ind, _ = target_course.search_target_index(state)
fo = open('PurePursuit.txt','a')

## Main Loop
try:
    while elapsed_time() < simulationTime:
        # Start timing this iteration
        start = elapsed_time()

         # Calc control input
        ai = proportional_control(target_speed, state.v)
        di, target_ind = pure_pursuit_steer_control(
            state, target_course, target_ind)

        state.update(ai, di)  # Control vehicle


        # Perform I/O

        mtr_cmd = np.array([0.05, -norm(di, 0.5, -0.5)]) #Implement control max [0.30,+- 0.50]
            
        LEDs = np.array([0, 0, 0, 0, 0, 0, 1, 1])

        current, batteryVoltage, encoderCounts = myCar.read_write_std(mtr_cmd, LEDs)     
        
              
        
        
        
        # Differentiate encoder counts and then estimate linear speed in m/s
        encoderSpeed = diff.send((encoderCounts, timeStep))
        linearSpeed = basic_speed_estimation(encoderSpeed)

        # End timing this iteration
        end = elapsed_time()

        # Calculate computation time, and the time that the thread should pause/sleep for
        computation_time = end - start
        sleep_time = sampleTime - computation_time%sampleTime

        # Pause/sleep and print out the current timestamp
        time.sleep(sleep_time)
	
	
        fo.write(str(time.time()) + "\t")

        fo.write("{0:4.4f}\t{1:4.4f}\t{2:4.4f}\t\n" .format(linearSpeed, mtr_cmd[0], mtr_cmd[1]))
        print("Car Speed:\t\t\t{0:1.2f}\tm/s\nRemaining battery capacity:\t{1:4.2f}\t%\nMotor throttle:\t\t\t{2:4.2f}\t% PWM\nSteering:\t\t\t{3:3.2f}\trad"
                                                            .format(linearSpeed, 100 - (batteryVoltage - 10.5)*100/(12.6 - 10.5), mtr_cmd[0], mtr_cmd[1]))
        timeAfterSleep = elapsed_time()
        timeStep = timeAfterSleep - start
        counter += 1

except KeyboardInterrupt:
    print("User interrupted!")

finally:    
    myCar.terminate()
# -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- 
