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

BEACONS = {
	core.WO_BEACON_BLUE_YELLOW : 0,
	core.WO_BEACON_YELLOW_BLUE : 0,
	core.WO_BEACON_BLUE_PINK : 0,
	core.WO_BEACON_PINK_BLUE : 0,
	core.WO_BEACON_PINK_YELLOW : 0,
	core.WO_BEACON_YELLOW_PINK : 0,
}

DIST_THRESHOLD = 50

vx = 0.0
vy = 0.0
vtheta = 0.0

seen_counter = 0.0

class Turner(Node):
	def run(self):
		commands.setHeadPan(0, 0.0)
		commands.setWalkVelocity(0.1, 0.0, 0.3)

class Mover(Node):
	def run(self):
		commands.setHeadPan(0, 0.0)
		commands.setWalkVelocity(vx, vy, 0.0)

class Localizer(Node):
	def run(self):
		global vx, vy, vtheta, seen_counter
		beacons = []
		for beacon_name in BEACONS:
			beacon = mem_objects.world_objects[beacon_name]
			if beacon.seen:
				BEACONS[beacon_name] = max(1, BEACONS[beacon_name])

		print ("\n\n\nTotal beacons seen: %d\n\n\n"%sum(BEACONS.values()))
		if sum(BEACONS.values()) >= 2:
			seen_counter = 10
			for beacon_name in BEACONS:
				BEACONS[beacon_name] = 0

		seen_counter -= 1
		if seen_counter <= 0: # Counter
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

# class HeadStraight(Node):
# 	def run(self):
# 		commands.setHeadPan(0, 0.0)
# 		if self.getTime() > 0.2:
# 			self.finish()

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

		# self.add_transition(, C, localizer)

		for signal, node in nodes.iteritems():
			if signal == 'sit':
				self.add_transition(localizer, S(signal), node, T(10), localizer)
			else:
				self.add_transition(localizer, S(signal), node, T(0.5), localizer)

