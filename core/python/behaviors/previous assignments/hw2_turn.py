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
from state_machine import Node, S, C, T, StateMachine
import UTdebug


class Stand(Node):
    def run(self):
        commands.stand()
        if self.getTime() > 3.0:
            self.finish()


class Turn(Node):
    def run(self):
        commands.setWalkVelocity(0, 0, 0.1)



class Playing(StateMachine):
    def setup(self):
        stand = Stand()
        turn = Turn()
        self.trans(stand, C, turn)