"""hw3 behavior."""

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

DELAY = 0.05
DIST_THRESHOLD = 170
THETA_THRESHOLD = 0.05

RIGHT_FOOT_OFFSET = -0.20

vtheta = 0.
vx = 0.


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

    def __call__(self, error):
        self.pterm = error
        self.iterm += error
        self.dterm = (error - self.last_error) / DELAY

        print ("p=%.8f\t"%(self.pterm * self.Kp), "i=%.8f"%(self.iterm * self.Kd), "d=%.8f"%(self.dterm * self.Kd))

        self.i += 1
        if self.i >= self.T:
            self.clear()

        self.last_error = error

        return self.pterm * self.Kp + self.iterm * self.Ki + self.dterm * self.Kd

    def clear(self):
        self.pterm = 0.0
        self.iterm = 0.0
        self.dterm = 0.0
        self.last_error = 0.0
        self.i = 0

class Stand(Node):
    def run(self):
        commands.stand()
        if self.getTime() > 3.0:
            memory.speech.say("stand up")
            self.finish()


class FollowBall(Node):
    def run(self):
        commands.setWalkVelocity(vx, 0, vtheta)



class FindBall(Node):
    def __init__(self):
        super(FindBall, self).__init__()
        self.vtheta_controller = Controller(p=1.0, i=0.0, d=0.0)
        self.vx_controller = Controller(p=4e-3, i=5e-4, d=1e-5)

    def run(self):
        ball = mem_objects.world_objects[core.WO_BALL]
        ball_distance = ball.visionDistance
        ball_bearing = ball.visionBearing

        if not ball.seen or (not ball.fromTopCamera and ball_distance <= DIST_THRESHOLD and abs(ball_bearing - RIGHT_FOOT_OFFSET) <= THETA_THRESHOLD):
            print ("\n\n\n\t\tball not detected or too close \n\n\n")
            vx = 0.0
            vtheta = 0.0
        else:
            global vtheta
            global vx

            vx = self.vx_controller(ball_distance - DIST_THRESHOLD)
            # vx = 0.0
            vtheta = ball_bearing - RIGHT_FOOT_OFFSET
            # vtheta = self.vtheta_controller(ball.visionBearing)

            print ("\n\n\n\t\tvtheta = %.3f"%vtheta, "vx=%.3f\n\n\n"%vx)

            print (("\n\n\n\t\tball distance:%.3f \n\n\n" % ball_distance))

        self.finish()

class Playing(LoopingStateMachine):
    def setup(self):
        findball = FindBall()
        followball = FollowBall()
        stand = Stand()
        self.add_transition(stand, C, findball)
        self.add_transition(findball, C, followball, T(DELAY), findball)
