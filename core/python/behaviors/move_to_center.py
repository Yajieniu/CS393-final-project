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

turning_counter = 0  #how many times it has turned since the last time s
single_beacon_turn_and_walk_mode = 0
beacon_last_seen = -2
beacon_now_seen = -1

# For head movement
head_bearing = 0
head_move_side = 1


class Turner(Node):
	def run(self):
		global turning_counter
		turning_counter += 1
		# print("tuning once**********\t", turning_counter, beacon_last_seen, beacon_now_seen, "\n")
		commands.setHeadPanTilt(pan=head_bearing, tilt=0, time=0.3)
		commands.setWalkVelocity(0.1, 0.0, 0.2)

class Mover(Node):
	def run(self):
		commands.setHeadPanTilt(pan=head_bearing, tilt=0, time=0.3)
		commands.setWalkVelocity(vx, vy, 0.0)

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
		head_bearing += head_move_side*0.3
		# head movement ends


		beacons = []
		num_beacons_seen = 0
		beacon_now_seen = -1

		# BlockWrapper blockWrapper = memory.BlockWrapper()
		# odo = blockWrapper.odometry().displacement.rotation;

		for beacon_name in BEACONS:
			beacon = mem_objects.world_objects[beacon_name]
			if beacon.seen:
				num_beacons_seen += 1
				beacon_now_seen = beacon
				BEACONS[beacon_name] = max(1, BEACONS[beacon_name])


		# -------- walk towards a beacon start --------#

		# walk towards a beacon if no other beacons can be seen
		if num_beacons_seen == 1:

			# start of looking_for_the_second_beacon mode
			# this is a flag, 1 is on, 0 is off
			# this flag can be set to 1 repeatly
			single_beacon_turn_and_walk_mode = 1

			# after turning a circle (turn 182 times), if only see one beacon
			# end of looking_for_the_second_beacon mode
			if beacon_last_seen == beacon_now_seen: # the robot turns a circle after turning 192 times
				if turning_counter >= 165:
					# print("*****************", )
					single_beacon_turn_and_walk_mode = 0
					turning_counter = 0

					# walks towards this beacon
					self.postSignal('walk_towards_beacon')


		if num_beacons_seen >=2:
			single_beacon_turn_and_walk_mode = 0

		# if there is a beacon in this frame
		if beacon_now_seen != -1:
			beacon_last_seen = beacon_now_seen  # mark this one as the last seen

		# ------- walks towards a beacon end -----------#


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
				vx = -0.8 * math.cos(theta)
				vy = -0.8 * math.sin(theta)

				self.postSignal('move')

			else:
				self.postSignal('turn')  # turn around in place, so it can adjust it's location again
				# when it estimates the wrong position

				# self.postSignal('sit')

# class HeadStraight(Node):
# 	def run(self):
# 		commands.setHeadPan(0, 0.0)
# 		if self.getTime() > 0.2:
# 			self.finish()

class Walker(Node):
	def run(self):
		commands.setWalkVelocity(vx, vy, vtheta)


# When we can only see one beacon after turning around, we walk towards
# this beacon

t1 = 0
class WalkTowardsBeacon(Node):
	def run(self):
		num_beacons_seen = 0
		beac = -1 # no beacons are seen
		start_time = self.getTime()

		for beacon_name in BEACONS:
			beacon = mem_objects.world_objects[beacon_name]
			if beacon.seen:
				num_beacons_seen += 1
				beac = beacon

		if num_beacons_seen > 1:
			print("$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$ More than One Beacon is Seen\n\n\n")
			self.finish()
		elif num_beacons_seen <1:
			global t1
			t1 += 1
			if t1 > 3:
				t1 = 0
				print("$$$$$$$$$$$$$$$$$$$ can not seen beacons")
				self.finish()

		# this part executes when num_beacon is exactly one

		beacon = beac  # the only beacon we can see
		dist = beacon.distance # assume this is the distance from robot to beacon, not sure
		stop_distance = 500    # assume it's 50 cm

		if dist < stop_distance:
			print("^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^ dist smaller than stop distance ")
			self.finish()

		commands.setWalkVelocity(0.5, 0, beacon.bearing)

		# after this function executing 0.5 seconds
		if self.getTime() - start_time > 0.5:  
			walkTowardsBeacon = WalkTowardsBeacon()
			walkTowardsBeacon.run()
            

class Playing(LoopingStateMachine):
	def setup(self):
		turner = Turner()
		localizer = Localizer()
		walker = Walker()
		sitter = pose.Sit()
		walkTowardsBeacon = WalkTowardsBeacon()

		nodes = {
			'turn' : turner,
			'move' : walker,
			'sit'  : sitter,
			'walk_towards_beacon' : walkTowardsBeacon,
		}

		# self.add_transition(, C, localizer)

		for signal, node in nodes.iteritems():
			if signal == 'sit':
				self.add_transition(localizer, S(signal), node, T(10), localizer)
			elif signal == 'walk_towards_beacon':
				self.add_transition(localizer, S(signal), node, C, localizer)
			else:
				self.add_transition(localizer, S(signal), node, T(0.5), localizer)


