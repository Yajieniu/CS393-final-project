"""hw6 behavior."""

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


SCALE = 0.005
P = 1#2e-3/SCALE
I = 1e-4/SCALE
D = 2e-4/SCALE

RIGHT = 1
LEFT = -1

RIGHT_FOOT_OFFSET =-0.35

class Attacker(Node):
	def __init__(self):
		super(Attacker, self).__init_()
		self.controller = PIDController(p)
		self.stage = 0
		self.head_bearing = 0
		self.head_move_side = 1 # start from turning right
		self.beacon_seen = 0
		self.direction = 0
		self.turned_degree = 0
		self.distance_left = 1
		self.ball = mem_objects.world_objects[core.WO_UNKNOWN_GOAL]
		self.goal = mem_objects.world_objects[core.WO_UNKNOWN_GOAL]

	def run(self):

		# stage 0: start from the start point, walk towards ball
		if self.stage == 0:
			# approach the ball
			# look at beacon
			# decide turning direction
			proceed = True
			proceed &= self.walkTowardsBall()
			proceed &= self.lookAtBeacon()
			proceed &= self.turnDirection()

			if proceed:
				self.stage = 1
				self.head_bearing = 0
				self.head_move_side = 1
					
		# stage 1: given turning direction
		# turn 90 degree around the ball
		if self.stage == 1:
			commands.setWalkVelocity(0.0, 0.1 * self.direction, 0.2)
			self.turned_degree += 0.1 # this number needs to change
			if self.turned_degree >= pi/2:
				self.stage = 2 

		# stage 2: dribble the ball for some distance
		# adjust the direction if goal could be seen
		if self.stage == 2:
			self.dribble()

		# stage 3: kick to the goal
		if self.stage == 3:
			if not self.goal.seen:  # do nothing
				return
			self.kick()


	def kick():
		x_offset = 10
		y_offset = 35

		ball_x = self.ball.visionDistance * cos(self.ball.visionBearing)
		ball_y = self.ball.visionDistance * sin(self.ball.visionBearing)

		X_THRESHOLD = 3
		Y_THRESHOLD = 3

		# kick if at the right place
		if abs(ball_x - x_offset) <= X_THRESHOLD && abs(ball_y - y_offset) <= Y_THRESHOLD:
			if abs(self.goal.visionBearing) > 0.1:
				self.postSignal("kick")
			else:
				commands.setWalkVelocity(0, 0, self.goal.visionBearing)
		else:
			x_side = abs(ball_x - x_offset) / (ball_x - x_offset)
			y_side = abs(ball_y - y_offset) / (ball_y - y_offset)

			commands.setWalkVelocity(x_side * 0.03, y_side * 0.03, self.goal.visionBearing)


	def dribble():

		# if ball not in view, find ball by turning head
		if not self.ball.seen:
			self.head_bearing = self.DecideHeadBearing()
			commands.setHeadPanTilt(pan=self.head_bearing, tilt=0, time=0.1)
			return
		else:
			self.head_bearing = 0
			commands.setHeadPanTilt(pan=0, tilt=0, time=0.1)

		# if departed from the goal, stand still, turn direction
		goal_theta = self.goal.visionBearing
		if self.goal.seen && abs(goal_theta) > 0.05:
			commands.setWalkVelocity(0.0, 0.1, goal_theta)
		# dribble the ball forwards, suppose current direction 
		# is towards goal. Do not turn direction
		else:
			y_distance = self.ball.visionDistance * sin(self.ball.visionBearing) y_distance - RIGHT_FOOT_OFFSET
			
			# if ball is not right in front
			# step right or left
			if abs(y_distance) >= 0.4:  # ball not in front
            	vx = 0
            	vy = y_distance

            # walk forward slowly
			else:
				vx = 0.05
				vy = 0
			commands.setWalkVelocity(vx, vy, 0.0)

			self.distance_left -= 0.05 # this value needs change

			if self.distance <= 0:
				self.stage = 3

	# assumption: ball is in the view, at the front
	# assumption: start at the beginning location
	# three components: approach to ball
	# 					find beacon
	#        			decide turning direction
	def walkTowardsBall():
		ball = mem_objects.world_objects[core.WO_BALL]
		ball_theta = ball.visionBearing
		ball_distance = ball.visionDistance

		if not ball.seen:  # stand still
			print('\n\n\n Ball not seen. \n\n\n\n')
		else:  # walk towards ball
			# ball is far away
			if ball.fromTopCamera or ball_distance > BALL_MIN_DISTANCE:
				# velocity from PID controller
				vx = self.controller( (ball_distance - BALL_MIN_DISTANCE) * SCALE + 0.1)
				vtheta = ball_theta
				commands.setWalkVelocity(vx, 0, vtheta)
			
			else:  # ball is close
				return False
		return True


	# look for a beacon, if not in view, 				
	def lookAtBeacon():
		if self.beacon_seen:
			return True

		BEACONS = {
			core.WO_BEACON_BLUE_YELLOW: 1,
			core.WO_BEACON_YELLOW_BLUE: 2,
			core.WO_BEACON_PINK_YELLOW: 3
		}

		for beacon_name in BEACONS:
			beacon = mem_objects.world_objects[beacon_name]
			if beacon.seen:
				self.beacon_seen = BEACONS[beacon_name]
				return True

		# turn head around to find beacon
		self.head_bearing = self.DecideHeadBearing()
		commands.setHeadPanTilt(pan=self.head_bearing, tilt=0, time=0.1)
		return False


	def turnDirection():
		if self.beacon_seen:
			commands.setHeadPanTilt(pan=0, tilt=0, time=0.1)
			self.turn = self.beacon_seen > 2 ? RIGHT:LEFT
			if self.beacon_seen not in {1,2,3}:
				print('\n\n\nERROR\n\n\n')
				return False
			return True
		return False


	def DecideHeadBearing():
		if self.head_bearing > 1: 
			self.head_move_side = -1
		if self.head_bearing < -1:
			head_move_side = 1
		head_bearing += head_move_side*0.1


class PIDController(object):
	def __init__(self, T=10):
		self.Kp = P
		self.Ki = I
		self.Kd = D

		self.last_error = 0.
		self.pterm = 0.
		self.iterm = 0.
		self.dterm = 0.
		self.i = 0

		self.T = T

	def __call__(self, error):
		self.pterm = error
		self.iterm += error
		self.dterm = (error - self.last_error) / DELAY

		self.i += 1
		if self.i >= self.T:
			self.clear()

		self.last_error = error

		return self.pterm * self.Kp + self.iterm * self.Ki + self.dterm * self.Kd

	def clear(self):
		self.pterm = 0.0
		self.iterm = 0.0
		self.dterm = 0.0
		self.last_error = 0.0
		self.i = 0


class Stand(Node):
    def run(self):
        commands.stand()
        commands.setHeadPanTilt(pan=0, tilt=0, time=0.5, isChange=True)
        if self.getTime() > 3.0:
            memory.speech.say("stand up")
            self.finish()


class SwitchMode(Node):
	def run(self):
		global pressed_count, mode
		global dribble_kick_counter, goalie_walk_count

		button = core.sensor_values[core.centerButton]
		if button:
			pressed_count += 1

		if pressed_count >= 2:
			pressed_count = 0
			dribble_kick_counter = 0
			goalie_walk_count = 0

			if mode == 'attacker':
				mode = 'goalie'
			else:
				mode = 'attacker'
				dribble_kick_counter = 0

		self.postSignal(mode)


class Playing(LoopingStateMachine):
	def setup(self):


		# Switcher setup
		stand = Stand()
		switcher = SwitchMode()
		attacker = Attacker()
		goalie = Goalie()
		kick = Kick()

		self.add_transition(stand, C, switcher)
		self.add_transition(switcher, S('attacker'), attacker)
		self.add_transition(attacker, S('kick'), kick, T(5), stand)


		# Goalie setup
		arms = {
			"left": pose.BlockLeft(time=3.),
			"right": pose.BlockRight(time=3.),
			"center": pose.BlockCenter(time=3.),
			"unseen": NotSeen(),
			"nomove": TakeRest(),
		}

		nodes = {
			'walk_left'  : WalkLeft(),
			'walk_right' : WalkRight(),
			'take_rest' : TakeRest()
		}

		self.add_transition(switcher, S('goalie'), goalie)
		for direction in arms:
			arm = arms[direction]
			if direction in ["left", "right", "center"]:
				self.add_transition(goalie, S(direction), arm, T(3), switcher)
			else:
				self.add_transition(goalie, S(direction), arm, T(0.1), switcher)

		for signal, node in nodes.iteritems():
			self.add_transition(goalie, S(signal), node, T(0.1), switcher)





