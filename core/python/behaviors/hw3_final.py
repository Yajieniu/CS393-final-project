"""hw3 behavior."""

from __future__ import print_function
from __future__ import division
from __future__ import absolute_import
from math import pi
from time import sleep

import memory
import mem_objects
import core
import commands
import cfgstiff
from task import Task
from state_machine import Node, S, C, T, LoopingStateMachine
import UTdebug

THETA_THRESHOLD = 0.05
RIGHT_FOOT_OFFSET = -0.25

DELAY = 0.05
BALL_MIN_PHASE1 = 250
BALL_MIN_PHASE2 = 170
GOAL_MIN = 1000

VX_MIN = 0.1
VX_MAX = 1
VY_MIN = 0.1
VTHETA_MIN = 0.1
VTHETA_MAX = 0.25

FACTOR = 0.001
SCALE = 0.005

VX_OFFSET = 0.1

P = 1#2e-3/SCALE
I = 1e-4/SCALE
D = 2e-4/SCALE

vx = 0.
vy = 0.
vtheta = 0.

goal_side = 1
ball_side = 1
play_mode = 1
dribble = 0.
kick_mode = False
kick_waiting = 3.0

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
        global play_mode
        global kick_waiting
        if not kick_mode and play_mode == 4: # Kick
            frames = self.getFrames()
            memory.walk_request.noWalk()
            goal = mem_objects.world_objects[core.WO_UNKNOWN_GOAL]
            print("**********goal distance: ", goal.visionDistance)
            memory.kick_request.setFwdKick()
            play_mode = 6
            kick_waiting = 3.0
            if self.getFrames() - frames > 150 and not memory.kick_request.kick_running_:
                self.finish()
                # play_mode = 1
        elif play_mode != 6:
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
        global kick_mode
        global kick_waiting
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
            if goal.fromTopCamera and goal_x < GOAL_MIN:
                kick_mode = True
        
        if ball.seen:
            ball_side = ball_theta / abs(ball_theta)

        # Changing speed
        if play_mode == 1:
            FACTOR = 0.2
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
            if not ball.seen or (ball.fromTopCamera or ball_x > 2*BALL_MIN_PHASE1):
                play_mode = 1

            vx = 0.
            if goal.seen:
                print ('\n\n\n\nGoal visible.')
                # vx = 0.2
                # vy = -ball_theta*0.5
                # vtheta = (ball_theta-goal_theta)
                if False:#abs(ball_theta) > 5*VY_MIN:
                    print (' ball\n\n\n\n')
                    vy = ball_theta
                    vtheta = 0
                else:
                    print (' goal\n\n\n\n')
                    vy = -goal_theta
                    vtheta = -0.5*vy
            else:
                print ('\n\n\n\nGoal NOT visible. goal side: ', goal_side, "\n\n\n\n")
                vy = -goal_side* 0.5
                vtheta = -0.5*vy

            if abs(vy) < VY_MIN and abs(vtheta) < VTHETA_MIN:
                if kick_mode:
                    play_mode = 3
                else:
                    play_mode = 5
                    dribble = 1.

        elif play_mode == 3:
            if not ball.seen or (ball.fromTopCamera or ball_x > 2*BALL_MIN_PHASE1):
            # if not ball.seen or (ball.fromTopCamera or ball_x > 1.5*BALL_MIN_PHASE2):
                play_mode = 1
            # Stop and transfer to kick
            FACTOR = 1.
            print ('\n\n\nPreparing for kick.\n\n\n\n')
            print ('\n\tball distance: %.3f\n'%ball_x)
            # vx = self.controller( (ball_x - BALL_MIN_PHASE2) * SCALE) + VX_OFFSET
            vx = 0
            vy = 0
            vtheta = 0
            if ball_x >= BALL_MIN_PHASE2:
                vx = 0.32
            if abs(ball_theta - RIGHT_FOOT_OFFSET) >= 0.07:
                vy = ball_theta - RIGHT_FOOT_OFFSET
                # vtheta = vy * 1.0
                # if goal.seen:
                    # vtheta = 0.1*(ball_theta - goal_theta)
                # vy = -ball_theta*0.5
                # vtheta = (ball_theta-goal_theta)
                # vtheta = -0.5*vy

            if ball_x < BALL_MIN_PHASE2 and abs(vy) < 0.07:
                play_mode = 4
                left_offset = 0.5

        elif play_mode == 4:
            # if left_offset > 0:
            #     print ('moving left '*100)
            #     left_offset -= DELAY
            #     vy = 0.3
            #     vx = 0.25
            # elif left_offset > -1:
            #     left_offset -= DELAY
            # else:
            #     vx = 0
            #     vy = 0
            #     kick_mode = False
            print ("\n\n\n\n\tkick!!\n\n\n\n")
            vx = 0
            vy = 0
            vtheta = 0
            kick_mode = False
            if ball.seen and not (ball_x < BALL_MIN_PHASE2 and abs(vy) < 0.05):
                play_mode = 3
                kick_mode = True
            else:
                sleep(0.5)


        elif play_mode == 5:
            if dribble > 0:
                print ("\n\n\n\ndribble\n\n\n\n")
                dribble -= DELAY
                vx = 0.75
                if ball.seen and goal.seen:
                    vtheta = ball_theta
                else:
                    vy = 0
                    vtheta = 0
            else:
                play_mode = 1
        else:
            if kick_waiting > 0:
                print ('\n\nWainting fo kick to finish\n\n');
                kick_waiting -= DELAY
            else:
                play_mode = 1


        print ("\n\n\t\t vx=%.3f, vy=%.3f, vtheta=%.3f" % (vx, vy, vtheta))

        # Moving average        
        vx = vx0 + FACTOR*(vx-vx0);
        vy = vy0 + FACTOR*(vy-vy0);
        vtheta = vtheta0 + FACTOR*(vtheta-vtheta0);

        # Just a check to stop the robot when values close to 0. Not very important.
        # vx *= (abs(vx) > VX_MIN)
        # vy *=  (abs(vy) > VY_MIN)
        # vtheta *=  (abs(vtheta) > VTHETA_MIN)

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
        play_mode = 1
        dribble = 0.

        left_offset = 0.
        
        findball = FindBall()
        followball = FollowBall()
        stand = Stand()
        self.add_transition(stand, C, findball)
        self.add_transition(findball, C, followball, T(DELAY), findball)
