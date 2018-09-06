"""hw1 behavior."""

from __future__ import print_function
from __future__ import division
from __future__ import absolute_import

import memory
import pose
import commands
import cfgstiff
from task import Task
from state_machine import Node, C, T, StateMachine

class Playing(StateMachine):
    class Stand(Node):
        def run(self):
            commands.stand()
            if self.getTime() > 5.0:
                memory.speech.say("playing stand complete")
                self.finish()

    class HeadTurn(Node):
        def run(self):
            commands.setHeadPan(1.0, 2.0)
    
    class HeadTurnBack(Node):
        def run(self):
            commands.setHeadPan(0, 2.0)

    class ForwardWalk(Node):
        def run(self):
            commands.setWalkVelocity(0.5, 0, 0)

    class TurnInPlace(Node):
        def run(self):
            commands.setWalkVelocity(0, 0, 0.2)

    class CurveWalk(Node):
        def run(self):
            commands.setWalkVelocity(0.8, 0, 0.2)

    class Off(Node):
        def run(self):
            commands.setStiffness(cfgstiff.Zero)
            if self.getTime() > 2.0:
                memory.speech.say("turned off stiffness")
                self.finish()

    def setup(self):
        stand = self.Stand()
        sit = pose.Sit()
        headturn = self.HeadTurn()
        headturnback = self.HeadTurnBack()

        forwardwalk = self.ForwardWalk()
        turninplace = self.TurnInPlace()
        curvewalk = self.CurveWalk()

        off = self.Off()
        self.trans(stand, C, headturn, T(3.0), headturnback, T(3.0),
          forwardwalk, T(5.0), turninplace, T(5.0), curvewalk, T(5.0),
          sit, C, off)