"""Simple keeper behavior."""

from __future__ import print_function
from __future__ import division
from __future__ import absolute_import

import core
import commands
import mem_objects
from state_machine import Node, S, T, LoopingStateMachine
import UTdebug
from math import sqrt, atan2

CENTER_THRESHOLD = 50
V_THRESHOLD = 10

class BlockLeft(Node):
    def run(self):
        UTdebug.log(15, "Blocking left")


class BlockRight(Node):
    def run(self):
        UTdebug.log(15, "Blocking right")


class BlockCenter(Node):
    def run(self):
        UTdebug.log(15, "Blocking right")


class Blocker(Node):
    def run(self):
        ball = mem_objects.world_objects[core.WO_BALL]
        # import pdb; pdb.set_trace()
        distance = ball.distance
        x = ball.loc.x
        y = ball.loc.y
        vx = ball.absVel.x
        vy = ball.absVel.y
        v = sqrt(vx*vx+vy*vy)
        if v > V_THRESHOLD:
            norm_vx = vx / v
            norm_vy = vy / v
        else:
            norm_vx = 0.0
            norm_vy = 0.0
        bearing = atan2(x, y)
        # commands.setHeadPan(ball.bearing, 0.1)

        if ball.seen:
            end_x = x + distance * norm_vx
            end_y = y + distance * norm_vy

            print ("Distance: ", distance)
            print ("Predict end y: ", end_y)

            if end_y < -CENTER_THRESHOLD:
                print ("right")
            elif end_y > CENTER_THRESHOLD:
                print ("left")
            else:
                print ("center")
        # if ball.distance < 1000:



        # print (x, y, vx, vy)
        # if ball.distance < 500:
        #     # print (x, y, vx, vy)
        #     print ("ok")
        #     UTdebug.log(15, "Ball is close, blocking!")
        #     if ball.bearing > 30 * core.DEG_T_RAD:
        #         choice = "left"
        #     elif ball.bearing < -30 * core.DEG_T_RAD:
        #         choice = "right"
        #     else:
        #         choice = "center"
        #     self.postSignal(choice)
        # else:
        #     print ("not ok")


class Playing(LoopingStateMachine):
    def setup(self):
        blocker = Blocker()
        blocks = {"left": BlockLeft(),
                  "right": BlockRight(),
                  "center": BlockCenter()
                  }
        for name in blocks:
            b = blocks[name]
            self.add_transition(blocker, S(name), b, T(5), blocker)
