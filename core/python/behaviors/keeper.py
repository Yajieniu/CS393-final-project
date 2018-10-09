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

# State models: (x, y, vx, vy)

# Measurements:
# imageCenterX, imageCenterY, position p, visionBearing, 
# visionDistance

CENTER_THRESHOLD = 50
V_THRESHOLD = 10



class RaiseLeft(Node):
	# raise left arm 
	def run(self):
		# UTdebug.log(15, "\n\n\n**********Raising left arm.")
		print("\n\n\n**********Raising left arm.")


class RaiseRight(Node):
	# raise right arm
	def run(self):
		# UTdebug.log(15, "\n\n**********Raising right arm.")
		print("\n\n\n**********Raising right arm.")


class RaiseBoth(Node):
	# raise both arms
	def run(self):
		# UTdebug.log(15, "\n\n**********Raising both arms.")
		print("\n\n\n**********Raising both arm.")



class RaiseArms(Node):
	def run(self):
		# get predicted location and velocity
		ball = mem_objects.world_objects[core.WO_BALL]
		# import pdb; pdb.set_trace()
		distance = ball.distance
		x = ball.loc.x
		y = ball.loc.y
		vx = ball.absVel.x
		vy = ball.absVel.y
		v = math.sqrt(vx*vx+vy*vy)
		print (x, y, vx, vy)

		if v > V_THRESHOLD and ball.seen:
			norm_vx = vx / v
			norm_vy = vy / v
			end_x = x + distance * norm_vx
			end_y = y + distance * norm_vy

			print ("Distance: ", distance)
			print ("Predict end y: ", end_y)

			if end_y < -CENTER_THRESHOLD:
				choice =  "right"
			elif end_y > CENTER_THRESHOLD:
				choice = "left"
			else:
				choice = "center"
			
			self.postSignal(choice)
		# else:
		# 	norm_vx = 0.0
		# 	norm_vy = 0.0
		# 	bearing = math.atan2(x, y)

		# commands.setHeadPan(ball.bearing, 0.1)
		else:
			self.postSignal('not_seen')

class Playing(LoopingStateMachine):
	def setup(self):
		raiseArm = RaiseArms()
		arms = {"left": RaiseLeft(),
		"right": RaiseRight(),
		"center": RaiseBoth(),
		"not_seen" : RaiseArms(),
		}

		for direction in arms:
			arm = arms[direction]
			self.add_transition(raiseArm, S(direction), arm, T(.5), raiseArm)
