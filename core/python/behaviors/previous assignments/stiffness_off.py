"""Stand"""

from __future__ import print_function
from __future__ import division
from __future__ import absolute_import

import time
import memory
import pose
import core
import commands
import cfgstiff
from task import Task
from state_machine import Node, S, C, T, LoopingStateMachine
from memory import joint_commands

readings = None
switch = False

class Playing(LoopingStateMachine):

    class Switch(Node):
        def run(self):
            if switch:
                commands.setStiffness(cfgstiff.Zero)

            else:
                readings = core.joint_values
                # TODO: Set joint values

                # commands.setStiffness()
                print ("Set stiffness")

                for i, value in enumerate(readings):
                    joint_commands.setJointCommand(i, value)

                # commands.setStiffness()

            if self.getTime() > 0.5:
                self.finish()


    class Touched(Node):
        def run(self):
            head_touched = core.sensor_values[core.headMiddle]

            if head_touched > 0.5:
                global switch
                switch = not switch
                # print (("switch: %s"%switch)*1000)
                time.sleep(1)
                self.postSignal('switch_phase')
            else:
                self.postSignal('remain')

    # class 

    def setup(self):
        switch = self.Switch()
        touched = self.Touched()
        self.trans(touched, S('switch_phase'), switch, C, touched)
        self.trans(touched, S('remain'), touched)


class Finished(Task):
    def run(self):
        pass