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

DIST_THRESHOLD = 20

vx = 0.0
vy = 0.0
vtheta = 0.0

seen_counter = 0.0

turning_counter = 0  #how many times it has turned since the last time s

# For head movement
head_bearing = 0
head_move_side = 1

only_turn_head = 0

START_TURN_THRESHOLD = 5
# START_WALK_THRESHOLD = 10
START_WALK_THRESHOLD = 50

STOP_DISTANCE = 800

class Turner(Node):
	def run(self):
		commands.setWalkVelocity(0.1, 0.0, 0.2)
		commands.setHeadPanTilt(pan=head_bearing, tilt=0, time=0.1)

class Localizer(Node):
	def run(self):
		global vx, vy, vtheta, seen_counter

		# for walk towards beacon
		global beacon_now_seen, beacon_last_seen, turning_counter

		# head movement code. start
		global head_bearing, head_move_side
		if head_bearing < -2:
			head_move_side = 1
		elif head_bearing > 2:
			head_move_side = -1
		head_bearing += head_move_side*0.1
		# head movement ends


		beacons = []
		num_beacons_seen = 0
		beacon_now_seen = -1

		for beacon_name in BEACONS:
			beacon = mem_objects.world_objects[beacon_name]
			if beacon.seen:
				num_beacons_seen += 1
				beacon_now_seen = beacon
				BEACONS[beacon_name] = max(1, BEACONS[beacon_name])


		print ("\n\n\n\n ***** Num beacons seen: ", sum(BEACONS.values()), " *****\n\n\n\n")


		if sum(BEACONS.values()) >= 3:
			seen_counter = 30
			for beacon_name in BEACONS:
				BEACONS[beacon_name] = 0


		seen_counter -= 1
		if seen_counter <= 0: # Counter

			if num_beacons_seen >=3:
				turning_counter = 0

			# walk towards a beacon if no other beacons can be seen
			if sum(BEACONS.values()) <= 2 and turning_counter >= START_WALK_THRESHOLD:

				turning_counter = 0

				# walks towards this beacon
				self.postSignal('walk_towards_beacon')
			else:
				global turning_counter, only_turn_head
				turning_counter += 1
				only_turn_head += 1
				self.postSignal('turn')
		else:
			robot = mem_objects.world_objects[memory.robot_state.WO_SELF]
			x, y = robot.loc.x, robot.loc.y
			robot_theta = robot.orientation # Robot orientation
			target_theta = math.atan2(y, x)

			dist = math.sqrt(x*x + y*y)
			tx = dist * math.cos(target_theta - robot_theta)
			ty = dist * math.sin(target_theta - robot_theta)

			if dist < DIST_THRESHOLD and (sum(BEACONS.values()) >= 2 or seen_counter >= 20):

				turning_counter = 0
				seen_counter = 0
				for beacon_name in BEACONS:
					BEACONS[beacon_name] = 0
				print("\n\n\n\n sitting sitting sitting sitting\n\n\n")
				self.postSignal('sit')

			else:
				theta = target_theta - robot_theta
				vx = -0.5 * math.cos(theta)
				vy = -0.5 * math.sin(theta)
				vtheta = 0

				self.postSignal('move')


class Walker(Node):
	def run(self):
		global turning_counter
		turning_counter = 0
		commands.setHeadPanTilt(pan=head_bearing, tilt=0, time=0.1)
		commands.setWalkVelocity(vx, vy, vtheta)


class BeaconWalker(Node):
	def run(self):
		global turning_counter, head_bearing, head_move_side

		if head_bearing < -2:
			head_move_side = 1
		elif head_bearing > 2:
			head_move_side = -1
		head_bearing += head_move_side*0.1
		turning_counter = 0
		commands.setHeadPanTilt(pan=head_bearing, tilt=0, time=0.1)
		commands.setWalkVelocity(vx, vy, vtheta)

# When we can only see one beacon after turning around, we walk towards
# this beacon

class WalkTowardsBeacon(Node):
	def run(self):
		num_beacons_seen = 0
		beac = -1 # no beacons are seen
		start_time = self.getTime()

		commands.setHeadPanTilt(pan=0, tilt=0, time=0.1, isChange=True)

		seen_beacon = None

		for beacon_name in BEACONS:
			beacon = mem_objects.world_objects[beacon_name]
			if beacon.seen:
				seen_beacon = beacon

		if not seen_beacon:
			commands.setWalkVelocity(0,0,0)
			self.postSignal('no_towards_beacon')
		else:
			dist = seen_beacon.visionDistance
			bearing = seen_beacon.visionBearing
			
			print ("beacon beating: ", bearing, "beacon distance", dist)

			if dist < STOP_DISTANCE:
				commands.setWalkVelocity(0,0,0)
				self.postSignal('stop_towards_beacon')
			else:
				global vx, vy, vtheta
				vx = 1
				vy = 0
				# vtheta = 0
				vtheta = bearing
				# commands.setWalkVelocity(0.5,0,bearing)
				self.postSignal('keep_towards_beacon')


class Playing(LoopingStateMachine):
	def setup(self):
		turner = Turner()
		localizer = Localizer()
		walker = Walker()
		beacon_walker = BeaconWalker()
		sitter = pose.Sit()
		walkTowardsBeacon = WalkTowardsBeacon()

		nodes = {
			'turn' : turner,
			'move' : walker,
			'sit'  : sitter,
			'walk_towards_beacon' : walkTowardsBeacon,
		}

		for signal, node in nodes.iteritems():
			if signal == 'sit':
				self.add_transition(localizer, S(signal), node, T(10), localizer)
			elif signal == 'walk_towards_beacon':
				self.add_transition(localizer, S(signal), node, C, localizer)
			else:
				self.add_transition(localizer, S(signal), node, T(0.1), localizer)


		self.add_transition(walkTowardsBeacon, S('no_towards_beacon'), localizer)
		self.add_transition(walkTowardsBeacon, S('stop_towards_beacon'), localizer)
		self.add_transition(walkTowardsBeacon, S('keep_towards_beacon'), beacon_walker, T(0.1), walkTowardsBeacon)


