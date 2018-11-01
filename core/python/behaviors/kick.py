"""Simple behavior that stands, kicks, and then sits down."""

from __future__ import print_function
from __future__ import division
from __future__ import absolute_import

import memory
import pose
import commands
import cfgstiff
import mem_objects, core
from state_machine import StateMachine, Node, C


class Playing(StateMachine):
    class Stand(Node):
        def run(self):
            commands.stand()
            if self.getTime() > 3.0:
                self.finish()

    class Kick(Node):
        def run(self):
            if self.getFrames() <= 3:
                memory.walk_request.noWalk()
                goal = mem_objects.world_objects[core.WO_UNKNOWN_GOAL]
                print("**********goal distance: ", goal.visionDistance)
                memory.kick_request.setFwdKick()
            if self.getFrames() > 100 and not memory.kick_request.kick_running_:
                self.finish()

    class Walk(Node):
        def run(self):
            commands.setWalkVelocity(0.5, 0, 0)

    class Off(Node):
        def run(self):
            commands.setStiffness(cfgstiff.Zero)
            if self.getTime() > 2.0:
                memory.speech.say("turned off stiffness")
                self.finish()

    def setup(self):
        commands.setStiffness(cfgstiff.One)
        self.trans(self.Stand(), C, self.Kick())
