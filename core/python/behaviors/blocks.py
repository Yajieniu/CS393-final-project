"""hw4 behavior."""

from __future__ import print_function
from __future__ import division
from __future__ import absolute_import

import memory
import mem_objects
import core
import commands
import cfgstiff
from task import Task
from state_machine import Node, S, C, T, LoopingStateMachine
import UTdebug

# Added in assignment 6:
import math
import pose
import cfgpose
import time
import random

DELAY = 20.0

class TakeRest(Node):
	def run(self):
		commands.setWalkVelocity(0, 0, 0);

class Goalie(Node):
	actions = ['block_left']
	count = 0

	def run(self):
		if self.count < len(self.actions):
			action = self.actions[self.count]
		else:
			action = 'take_rest'
		self.count += 1

		robot = mem_objects.world_objects[memory.robot_state.WO_SELF]

		print ('\n\n\n', action, self.count, robot.loc.x, robot.loc.y, robot.orientation, '\n\n\n')
		self.postSignal(action)

		
class TryPose(Node):
	def run(self):
		self.poseSignal("try")

class Playing(LoopingStateMachine):
	def setup(self):
		nodes = {
			'block_left' : pose.Squat(time=10.),
			'block_right' : pose.Squat(time=10.),
			'take_rest' : TakeRest()
		}
		goalie = Goalie()

		for signal, node in nodes.iteritems():
			self.add_transition(goalie, S(signal), node, T(DELAY), goalie)