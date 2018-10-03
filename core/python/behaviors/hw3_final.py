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

THETA_THRESHOLD = 0.05
RIGHT_FOOT_OFFSET = -0.20

DELAY = 0.05
BALL_MIN_PHASE1 = 250
BALL_MIN_PHASE2 = 200
GOAL_MIN = 500

VX_MIN = 0.2
VX_MAX = 1
VY_MIN = 0.05
VTHETA_MIN = 0.05
VTHETA_MAX = 0.25

FACTOR = 0.001
SCALE = 0.005

VX_OFFSET = 0.1

P = 1#4e-3/SCALE
I = 1e-4/SCALE
D = 2e-4/SCALE

vx = 0.
vy = 0.
vtheta = 0.

goal_side = 1
ball_side = 1
play_mode = 1
dribble = 0.

left_offset = 0.


class Controller(object):
    def __init__(self, p, i, d, T=10):
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

        # print ("\n\n\tp=%.8f\t"%(self.pterm * self.Kp), "", "d=%.8f"%(self.dterm * self.Kd))

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
        commands.setHeadPanTilt(pan=0, tilt=0, time=0.5, isChange=True)
        if self.getTime() > 3.0:
            memory.speech.say("stand up")
            self.finish()


class FollowBall(Node):
    def run(self):
        if vx > VX_MIN:
            print ("\n\n\t\t vx=%.3f, vy=%.3f, vtheta=%.3f" % (vx, vy, vtheta))
        commands.setWalkVelocity(vx, vy, vtheta)



class FindBall(Node):
    def __init__(self):
        super(FindBall, self).__init__()
        # self.vtheta_controller = Controller(p=1.0, i=0.0, d=0.0)
        self.controller = Controller(p=P, i=I, d=D)

    def run(self):
        global vx
        global vy
        global vtheta
        global goal_side
        global ball_side
        global dribble
        global left_offset
        global play_mode
        global FACTOR

        vx0, vy0, vtheta0 = vx, vy, vtheta

        ball = mem_objects.world_objects[core.WO_BALL]
        goal = mem_objects.world_objects[core.WO_UNKNOWN_GOAL]

        ball_theta = ball.visionBearing
        ball_x = ball.visionDistance

        goal_theta = goal.visionBearing
        goal_x = goal.visionDistance

        # Goal seen earlier code
        if goal.seen:
            goal_side = goal_theta / abs(goal_theta)
        
        if ball.seen:
            ball_side = ball_theta / abs(ball_theta)
        
        if play_mode != 4:
            play_mode = 3
        # Changing speed
        if play_mode == 1:
            FACTOR = 0.01
            if not ball.seen:
                print ('\n\n\n\nBall not seen.\n\n\n\n')
                vx = 0.0
                vy = 0.0
                vtheta = ball_side* 0.5
            elif ball.fromTopCamera or ball_x > BALL_MIN_PHASE1:
                print ('\n\n\n\nBall seen far away.\n\n\n\n')
                vx = self.controller( (ball_x - BALL_MIN_PHASE1) * SCALE)
                vtheta = ball_theta
            else:
                play_mode = 2

        elif play_mode == 2:
            if ball.seen and (ball.fromTopCamera or ball_x > 2*BALL_MIN_PHASE1):
                play_mode = 1

            vx = 0.
            if goal.seen:
                print ('\n\n\n\nGoal visible.\n\n\n\n')
                vy = -ball_theta*0.3
                vtheta = (ball_theta-goal_theta)
            else:
                print ('\n\n\n\nGoal NOT visible. goal side: ', goal_side, "\n\n\n\n")
                vy = -goal_side* 0.5
                vtheta = -0.5*vy

            # if abs(vy) < 3*VY_MIN and abs(vtheta) < 3*VTHETA_MIN:

            if goal_x < GOAL_MIN: # TODO: tune GOAL_MIN
                # Kick time
                play_mode = 3
            else:
                play_mode = 5
                dribble = 1.

        elif play_mode == 3:
            # Stop and transfer to kick
            FACTOR = 1.
            print ('\n\tball distance: %.3f\n'%ball_x)
            vx = self.controller( (ball_x - BALL_MIN_PHASE2) * SCALE) + VX_OFFSET
            vtheta = (ball_theta-goal_theta)

            if ball_x < BALL_MIN_PHASE2 and abs(vtheta) < VTHETA_MIN:
                play_mode = 4
                left_offset = 0.5

        elif play_mode == 4:
            if left_offset > 0:
                print ('moving left '*100)
                left_offset -= DELAY
                vy = 0.3
                vx = 0.5
            else:
                vx = 0
                vy = 0

            # memory.walk_request.noWalk()
            # memory.kick_request.setFwdKick()

            # if not memory.kick_request.kick_running_:
                # play_mode = 1
            # vx = 0
            vtheta = 0

        else:
            if dribble > 0:
                print ("\n\n\n\ndribble\n\n\n\n")
                dribble -= DELAY
                vx = 0.75
                if ball.seen and goal.seen:
                    # vy = -ball_theta*0.3
                    vtheta = (ball_theta-goal_theta)
                else:
                    vy = 0
                    vtheta = 0
            else:
                play_mode = 1


        # Moving average
        vx = vx0 + FACTOR*(vx-vx0);
        vy = vy0 + FACTOR*(vy-vy0);
        vtheta = vtheta0 + FACTOR*(vtheta-vtheta0);

        # Just a check to stop the robot when values close to 0. Not very important.
        vx *= (abs(vx) > VX_MIN)
        vy *=  (abs(vy) > VY_MIN)
        vtheta *=  (abs(vtheta) > VTHETA_MIN)

        self.finish()

class Playing(LoopingStateMachine):
    def setup(self):
        global vx
        global vy
        global vtheta
        global goal_side
        global ball_side
        global dribble
        global left_offset
        global play_mode
        global FACTOR

        vx = 0.
        vy = 0.
        vtheta = 0.

        goal_side = 1
        ball_side = 1
        play_mode = 3
        dribble = 0.

        left_offset = 0.
        
        findball = FindBall()
        followball = FollowBall()
        stand = Stand()
        self.add_transition(stand, C, findball)
        self.add_transition(findball, C, followball, T(DELAY), findball)
