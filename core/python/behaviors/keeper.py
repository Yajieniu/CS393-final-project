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

# Added in assignment 4:
import math
import pose
import cfgpose

import time

# State models: (x, y, vx, vy)

# Measurements:
# imageCenterX, imageCenterY, position p, visionBearing, 
# visionDistance

CENTER_MIN_THRESHOLD = 50
CENTER_MAX_THRESHOLD = 500
V_THRESHOLD = 20



class RaiseLeft(Node):
	# raise left arm 
	def run(self):
		# UTdebug.log(15, "\n\n\n**********Raising left arm.")
		print("\n\n\n***************************************\
			************************************************\
			*************\
			Raising left arm.\n\n\n\n")
		pose.RaiseLeftArm()


class RaiseRight(Node):
	# raise right arm
	def run(self):
		# UTdebug.log(15, "\n\n**********Raising right arm.")
		print("\n\n\n***************************************\
			************************************************\
			*************\
			Raising right arm.\n\n\n\n")
		pose.RaiseRightArm()


class RaiseBoth(Node):
	# raise both arms
	def run(self):
		# UTdebug.log(15, "\n\n**********Raising both arms.")
		print("\n\n\n***************************************\
			************************************************\
			*************\
			Raising both arms.\n\n\n\n")		
		pose.RaiseBothArms()

class NotSeen(Node):
	def run(self):
		commands.setHeadPan(0, 0.05)
		if self.getTime() > 0.1:
			self.finish()


class Wait(Node):
	def run(self):
		if self.getTime() > 1:
			self.finish()


class RaiseArms(Node):
	def run(self):
		# get predicted location and velocity
		ball = mem_objects.world_objects[core.WO_BALL]

		commands.setStiffness()
		# pose.SittingPose()
		# import pdb; pdb.set_trace()
		distance = ball.distance
		x = ball.loc.x
		y = ball.loc.y
		vx = ball.absVel.x
		vy = ball.absVel.y
		v = math.sqrt(vx*vx+vy*vy)
		# print (x, y, vx, vy)
		
		if v > 0:
			end_y = y + x * vy/vx
		else:
			end_y = y
		print ("\n\n\n\n End y: ", end_y)
		print ("\n\n\n\n V: ", v)
		print ("\n\n\n\n bearing: ", ball.bearing)


		if not ball.seen:
			print ("\n\n\nball not seen!\n\n\n")
			commands.setHeadPan(0, 0.05)
			choice = "unseen"
			# print ("\n\n\n\n Ball moving!!! \n\n\n\n")
		elif v < V_THRESHOLD:
			commands.setHeadPan(ball.bearing, 0.05)
			print ("\n\n\nball steady\n\n\n")
			choice = "nomove"
		else:
			commands.setHeadPan(ball.bearing, 0.05)
			print ("\n\n\n ball moving!!!\n\n\n")

			if v > V_THRESHOLD and vx < 0:
				if end_y < -CENTER_MIN_THRESHOLD and end_y > -CENTER_MAX_THRESHOLD:
					print ("\n\n\n\nleft!!!!!\n\n\n")
					choice = "left"
				elif end_y > CENTER_MIN_THRESHOLD and end_y < CENTER_MAX_THRESHOLD:
					print ("\n\n\n\nright!!!!!\n\n\n")
					choice = "right"
				elif end_y < CENTER_MIN_THRESHOLD and end_y > -CENTER_MIN_THRESHOLD:
					print ("\n\n\n\ncenter!!!!!\n\n\n")
					choice = "center"
				else:
					choice = "nomove"

			else:
				print ("\n\n\n\nno move\n\n\n\n")
				choice = "nomove"

		self.postSignal(choice)
		

class Playing(LoopingStateMachine):
	def setup(self):
		raiseArm = RaiseArms()
		sit = pose.SittingPose()
		wait = Wait
		arms = {
			"left": pose.RaiseLeftArm(time=0.3),
			"right": pose.RaiseRightArm(time=0.3),
			"center": pose.RaiseBothArms(time=0.3),
			"unseen": pose.SittingPose(time=0.3),
			"nomove": pose.SittingPose(time=0.3),
		}

		# arms = {"left": RaiseLeft(),
		# "right": RaiseRight(),
		# "center": RaiseBoth(),
		# "not_seen": NotSeen()
		# }
		# self.add_transition(sit, T(0.5), pose.RaiseRightArm(), T(5), sit)
		# self.trans(raiseArm, S("nomove"), sit, T(0.3), pose.RaiseBothArms(), T(5), raiseArm)

		for direction in arms:
			arm = arms[direction]
			self.add_transition(raiseArm, S(direction), arm, T(0.4), raiseArm)

			# if direction in ["left", "right", "center"]:
				# self.add_transition(raiseArm, S(direction), arm, T(0.4), wait, C, raiseArm)
			# else:
				# self.add_transition(raiseArm, S(direction), arm, T(0.4), raiseArm)
