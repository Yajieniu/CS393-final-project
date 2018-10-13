"""hw1 behavior."""

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
    def __init__(self, p, i, d):
        self.Kp = p
        self.Ki = i
        self.kD = d

        self.last_error = 0.
        self.pterm = 0.
        self.iterm = 0.
        self.dterm = 0.

    def update(self, error):
        self.pterm = error
        self.iterm += error
        self.dterm = (error - self.last_error) / DELAY

        return self.pterm * self.Kp + self.iterm * self.Ki + self.dterm * self.Kd


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
        commands.setHeadPanTilt(pan=0, tilt=-23, time=0.5)
        # self.finish()



class Playing(LoopingStateMachine):
    def setup(self):
        # gazer = Gazer()
        # gaze = Gaze()
        on = On()
        self.add_transition(on, T(0.5), on)

        # self.add_transition(gazer, S('no_ball'), gazer)
        # stand = Stand()
        # gaze1 = GazeLeft()
        # gaze2 = GazeLeft()
        # self.trans(stand, C, gaze1, T(3.0), gaze2, T(3.0))
