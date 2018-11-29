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

class Controller(object):
    def __init__(self, p, i, d, T=30):
        self.Kp = p
        self.Ki = i
        self.Kd = d

        self.last_error = 0.
        self.pterm = 0.
        self.iterm = 0.
        self.dterm = 0.
        self.i = 0

        self.T = T

    def update(self, error):
        self.pterm = error
        self.iterm += error
        self.dterm = (error - self.last_error) / DELAY
        self.i += 1
        if self.i >= self.T:
            self.clear()

        return self.pterm * self.Kp + self.iterm * self.Ki + self.dterm * self.Kd

    def clear(self):
        self.pterm = 0.0
        self.iterm = 0.0
        self.dterm = 0.0
        self.last_error = 0.0
        self.i = 0


CENTER_THRESHOLD = 100
DELAY = 0.05

# Top camera range (0, 1278) x (0, 958)
X_RANGE = 320
Y_RANGE = 240
X_THETA = 40 * core.DEG_T_RAD
Y_THETA = 30

x_diff = 0
y_diff = 0


# distance of center = 90. breadth = 100

class Stand(Node):
    def run(self):
        commands.stand()
        if self.getTime() > 3.0:
            # memory.speech.say("playing stand complete")
            self.finish()


class On(Node):
    def run(self):
        commands.setStiffness()
        self.finish()

class GazeCenter(Node):
    def run(self):
        self.finish()

class Gaze(Node):
    def run(self):
        commands.setHeadPanTilt(pan=x_diff, tilt=y_diff, time=0.5, isChange=True)
        # if self.getTime() > DELAY:
        #     self.finish()

class Gazer(Node):
    def __init__(self, p=1.2, i=0.0, d=0.0):
        super(Gazer, self).__init__()
        self.controller = Controller(p, i, d)

    def run(self):
        global x_diff
        global y_diff
        ball = mem_objects.world_objects[core.WO_BALL]
        if ball.seen:
            x = ball.imageCenterX
            y = ball.imageCenterY
            print ("Detected ball centroid: ", x, y)

            x_error = -((x - X_RANGE/2)/X_RANGE)*X_THETA
            x_diff = self.controller.update(x_error)
            y_diff = -21 - ((y - Y_RANGE/2)/Y_RANGE)*Y_THETA


        else:
            choice = "no_ball"
            print ("No ball detected")

            x_diff = 0
            # y_diff = -21
        
        self.finish()



class Playing(LoopingStateMachine):
    def setup(self):
        gazer = Gazer()
        gaze = Gaze()
        on = On()
        self.add_transition(on, C, gazer)
        self.add_transition(gazer, C, gaze, T(DELAY), gazer)

        # self.add_transition(gazer, S('no_ball'), gazer)
        # stand = Stand()
        # gaze1 = GazeLeft()
        # gaze2 = GazeLeft()
        # self.trans(stand, C, gaze1, T(3.0), gaze2, T(3.0))
