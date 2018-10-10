"""Stand"""

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
    class On(Node):
        def run(self):
        	# pass
        	# Get all joint readings
            # commands.setStiffness()
            # Set joint readingsd

    def setup(self):
        self.trans(self.On(), C, self.On())
