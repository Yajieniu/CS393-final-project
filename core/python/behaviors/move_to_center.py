from __future__ import print_function
from __future__ import division
from __future__ import absolute_import

import math
import memory
import mem_objects
import pose
import core
import commands
import cfgstiff
import random
from task import Task
from state_machine import Node, C, S, T, LoopingStateMachine

BEACONS = [
	core.WO_BEACON_BLUE_YELLOW,
	core.WO_BEACON_YELLOW_BLUE,
	core.WO_BEACON_BLUE_PINK,
	core.WO_BEACON_PINK_BLUE,
	core.WO_BEACON_PINK_YELLOW,
	core.WO_BEACON_YELLOW_PINK,
]

DIST_THRESHOLD = 50

vx = 0.0
vy = 0.0
vtheta = 0.0

seen_two = 0.0

class Turner(Node):
	def run(self):
		commands.setWalkVelocity(0.0, -0.1, 0.2)

class Mover(Node):
	def run(self):
		commands.setWalkVelocity(vx, vy, 0.0)

class Localizer(Node):
	def run(self):
		global vx, vy, vtheta, seen_two
		beacons = []
		if sum([mem_objects.world_objects[beacon_name].seen for beacon_name in BEACONS]) >= 2:
			seen_two = 5 # Counter

		seen_two -= 1
		print (seen_two)
		if seen_two <= 0: # Counter
			self.postSignal('turn')
		else:
			robot = mem_objects.world_objects[memory.robot_state.WO_SELF]
			x, y = robot.loc.x, robot.loc.y
			robot_theta = robot.orientation # Robot orientation
			target_theta = math.atan2(y, x)

			dist = math.sqrt(x*x + y*y)
			tx = dist * math.cos(target_theta - robot_theta)
			ty = dist * math.sin(target_theta - robot_theta)

			if dist > DIST_THRESHOLD:

				theta = target_theta - robot_theta
				vx = -1.5 * math.cos(theta)
				vy = -1.5 * math.sin(theta)

				self.postSignal('move')

			else:
				self.postSignal('sit')

class Walker(Node):
	def run(self):
		commands.setWalkVelocity(vx, vy, vtheta)


class Playing(LoopingStateMachine):
	def setup(self):
		turner = Turner()
		localizer = Localizer()
		walker = Walker()
		sitter = pose.Sit()

		nodes = {
			'turn' : turner,
			'move' : walker,
			'sit'  : sitter,
		}

		# self.add_transition(turner, T(5.0), localizer)

		for signal, node in nodes.iteritems():
			if signal == 'sit':
				self.add_transition(localizer, S(signal), node)
			else:
				self.add_transition(localizer, S(signal), node, T(0.5), localizer)

