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

CENTER_MIN_THRESHOLD = 200
CENTER_MAX_THRESHOLD = 200
V_THRESHOLD = 35
X_THRESHOLD = 600

running_vx = 0.0
running_vy = 0.0
unseen_count = 0


# class TurnLeft(Node):
# 	def run(self):
# 		commands.setWalkVelocity(0, 0, 0.3);

# class TurnRight(Node):
# 	def run(self):
# 		commands.setWalkVelocity(0, 0, -0.3);

# class WalkFront(Node):
# 	def run(self):
# 		commands.setWalkVelocity(0.3, 0, 0);

# class WalkBack(Node):
# 	def run(self):
# 		commands.setWalkVelocity(-0.3, 0, 0);

class WalkLeft(Node):
	def run(self):
		commands.setWalkVelocity(0.1, 0.5, 0.2);

class WalkRight(Node):
	def run(self):
		commands.setWalkVelocity(0.1, -0.5, -0.15);

class TakeRest(Node):
	def run(self):
		commands.setWalkVelocity(0, 0, 0);

class RaiseLeft(Node):
	def run(self):
		print("\n\n\n***************************************\
			************************************************\
			*************\
			Raising left arm.\n\n\n\n")
		pose.RaiseLeftArm()


class RaiseRight(Node):
	def run(self):
		print("\n\n\n***************************************\
			************************************************\
			*************\
			Raising right arm.\n\n\n\n")
		pose.RaiseRightArm()


class RaiseBoth(Node):
	def run(self):
		print("\n\n\n***************************************\
			************************************************\
			*************\
			Raising both arms.\n\n\n\n")		
		pose.RaiseBothArms()

class NotSeen(Node):
	head_bearing = 0
	head_move_side = 1
	def run(self):
		if self.head_bearing < -1:
			self.head_move_side = 1
		elif self.head_bearing > 1:
			self.head_move_side = -1
		self.head_bearing += self.head_move_side*0.1
		commands.setHeadPan(self.head_bearing, 0.05)
		if self.getTime() > 0.1:
			self.finish()


class Wait(Node):
	def run(self):
		if self.getTime() > 1:
			self.finish()


class Goalie(Node):
	count = 0

	def run(self):

		global running_vx, running_vy, unseen_count
		# get predicted location and velocity
		ball = mem_objects.world_objects[core.WO_BALL]

		commands.setStiffness()
		distance = ball.distance
		x = ball.loc.x
		y = ball.loc.y
		vx = ball.absVel.x
		vy = ball.absVel.y

		v = math.sqrt(vx*vx+vy*vy)

		print ("\n\n\n\nVx: ", vx, "vy: ", vy, "V: ", v, "X: ", x, "Y: ", y)
		print ("bearing: ", ball.bearing, "\n\n\n\n")

		if not ball.seen:
			unseen_count += 1
			choice = "nomove"
		elif True: #v < V_THRESHOLD:
			commands.setHeadPan(ball.bearing, 0.1)
			if ball.bearing < -0.3 and self.count > -5:
				choice = "walk_right"
				self.count -= 1
			elif ball.bearing > 0.3 and self.count < 5:
				choice = "walk_left"
				self.count += 1
			else:
				choice = 'nomove'
		else:
			commands.setHeadPan(ball.bearing, 0.05)

			head_angle = core.joint_values[core.HeadPan]
			t = 5.0
			end_x = x + t * vx
			end_y = y + t * vy

			if end_x <= X_THRESHOLD:

				if end_y <= -CENTER_MAX_THRESHOLD:
					choice = "right"
				elif end_y >= CENTER_MAX_THRESHOLD:
					choice = "left"
				elif end_y < CENTER_MAX_THRESHOLD and end_y > -CENTER_MAX_THRESHOLD:
					choice = "center"
				else:
					choice = "nomove"
			else:
				choice = "nomove"

		if unseen_count >= 3:
			unseen_count = 0
			choice = "unseen"

		self.postSignal(choice)
		

class Playing(LoopingStateMachine):
	def setup(self):
		goalie = Goalie()
		sit = pose.SittingPose()
		wait = Wait
		arms = {
			"left": pose.RaiseLeftArm(time=1.),
			"right": pose.RaiseRightArm(time=1.),
			"center": pose.RaiseBothArms(time=1.),
			"unseen": NotSeen(),
			"nomove": TakeRest(),
		}

		nodes = {
			'walk_left'  : WalkLeft(),
			'walk_right' : WalkRight(),
			'take_rest' : TakeRest()
		}

		for direction in arms:
			arm = arms[direction]
			if direction in ["left", "right", "center"]:
				self.add_transition(goalie, S(direction), arm, T(0.1), goalie)
			else:
				self.add_transition(goalie, S(direction), arm, T(0.1), goalie)

		for signal, node in nodes.iteritems():
			self.add_transition(goalie, S(signal), node, T(1), nodes['take_rest'], T(0.5), goalie)
