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

DELAY = 1.0

# class Stand(Node):
#     def run(self):
#         commands.stand()
#         if self.getTime() > 3.0:
#             memory.speech.say("stand up")
#             self.finish()

class TurnLeft(Node):
	def run(self):
		commands.setWalkVelocity(0, 0, 0.3);

class TurnRight(Node):
	def run(self):
		commands.setWalkVelocity(0, 0, -0.3);

class WalkFront(Node):
	def run(self):
		commands.setWalkVelocity(0.3, 0, 0);

class WalkBack(Node):
	def run(self):
		commands.setWalkVelocity(-0.3, 0, 0);

class WalkLeft(Node):
	def run(self):
		commands.setWalkVelocity(0.1, 0.5, 0.2);

class WalkRight(Node):
	def run(self):
		commands.setWalkVelocity(0.1, -0.5, -0.15);

class TakeRest(Node):
	def run(self):
		commands.setWalkVelocity(0, 0, 0);

class Goalie(Node):
	actions = ['walk_right', 'take_rest']*5 + ['walk_left', 'take_rest']*10 + ['walk_right', 'take_rest']*5
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
			'turn_left' : TurnLeft(),
			'turn_right' : TurnRight(),
			'walk_front'  : WalkFront(),
			'walk_back' : WalkBack(),
			'walk_left'  : WalkLeft(),
			'walk_right' : WalkRight(),
			'take_rest' : TakeRest()
		}
		goalie = Goalie()

		for signal, node in nodes.iteritems():
			self.add_transition(goalie, S(signal), node, T(DELAY), goalie)