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

X_THRESHOLD = 100
DIST_CLS_THRESHOLD = 200
DIST_FAR_THRESHOLD = 500

class Stand(Node):
    def run(self):
        commands.stand()
        if self.getTime() > 3.0:
            # memory.speech.say("playing stand complete")
            self.finish()

class TurnLeft(Node):
    def run(self):
        commands.setWalkVelocity(0, 0, -0.2)

class TurnRight(Node):
    def run(self):
        commands.setWalkVelocity(0, 0, 0.2)

class WalkFront(Node):
    def run(self):
        commands.setWalkVelocity(0.5, 0, 0)

class WalkBack(Node):
    def run(self):
        commands.setWalkVelocity(-0.5, 0, 0)

class Stop(Node):
    def run(self):
        commands.setWalkVelocity(0, 0, 0)

class Gazer(Node):
    def run(self):
        goal = mem_objects.world_objects[core.WO_OWN_GOAL]
        if goal.seen:
            x = goal.imageCenterX
            y = goal.imageCenterY
            distance = goal.visionDistance
            # print ("Detected goal: ", x, y, distance)

            if x - 640 <= - X_THRESHOLD:
                choice = "left"
            elif x - 640 >= X_THRESHOLD:
                choice = "right"
            # elif y > DIST_FAR_THRESHOLD:
            #     choice = "forward"
            # elif y < DIST_CLS_THRESHOLD:
            #     choice = "back"
            else:
                choice = "front"

        else:
            print ("No goal detected")
            choice = "stop"

        self.postSignal(choice)


class Playing(LoopingStateMachine):
    def setup(self):
        gazer = Gazer()
        stand = Stand()
        actions = {
            'left' : TurnLeft(),
            'right' : TurnRight(),
            'front' : WalkFront(),
            'stop' : Stop(),
        }

        self.add_transition(stand, C, gazer)
        for name, action in actions.items():
            self.add_transition(
                gazer, 
                S(name),
                action,
                T(0.2),
                gazer
            )
