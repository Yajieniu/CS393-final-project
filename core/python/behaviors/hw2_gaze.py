"""hw2 behavior."""

from __future__ import print_function
from __future__ import division
from __future__ import absolute_import
from math import pi

import memory
import mem_objects
import core
import commands
import cfgstiff
from task import Task
from state_machine import Node, S, C, T, LoopingStateMachine
import UTdebug

DELAY = 0.4

# Top camera range (0, 1278) x (0, 958)
X_RANGE = 320
Y_RANGE = 240
X_THETA = 40 * core.DEG_T_RAD
Y_THETA = 30

x_diff = 0
y_diff = 0

def clip_x(x_diff):
	x_diff = min(x_diff, 160)
	return x_diff


class Gazer(Node):
    def run(self):
        global x_diff
        global y_diff
        ball = mem_objects.world_objects[core.WO_BALL]
        if ball.seen:
            x = ball.imageCenterX
            y = ball.imageCenterY
            print (x_diff, y_diff)



            x_diff = -((x - X_RANGE/2)/X_RANGE)*X_THETA
            x_diff = clip_x(x_diff)
            y_diff = -21 - ((y - Y_RANGE/2)/Y_RANGE)*Y_THETA


        else:
            choice = "no_ball"
            print ("No ball detected")

            x_diff = 0
            # y_diff = -21
        
        self.finish()

class On(Node):
    def run(self):
        commands.setStiffness()
        self.finish()


class Gaze(Node):
    def run(self):
        commands.setHeadPanTilt(pan=x_diff, tilt=y_diff, time=DELAY, isChange=True)
        if self.getTime() > DELAY:
            self.finish()


class Playing(LoopingStateMachine):
    def setup(self):
        gazer = Gazer()
        gaze = Gaze()
        on = On()
        self.add_transition(on, C, gazer)
        self.add_transition(gazer, C, gaze, C, gazer)
