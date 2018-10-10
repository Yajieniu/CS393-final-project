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

# State models: (x, y, vx, vy)

# Measurements:
# imageCenterX, imageCenterY, position p, visionBearing, 
# visionDistance

CENTER_THRESHOLD = 50
GOAL_SIDE = 500
V_THRESHOLD = 10



class RaiseLeft(Node):
	# raise left arm 
	def run(self):
		# UTdebug.log(15, "\n\n\n**********Raising left arm.")
		print("\n\n\n***************************************\
			************************************************\
			*************\
			Raising left arm.\n\n\n\n")
		stand = pose.BlockRight()
		stand.run()


class RaiseRight(Node):
	# raise right arm
	def run(self):
		# UTdebug.log(15, "\n\n**********Raising right arm.")
		print("\n\n\n***************************************\
			************************************************\
			*************\
			Raising right arm.\n\n\n\n")
		stand = pose.StandStraight()
		stand.run()


class RaiseBoth(Node):
	# raise both arms
	def run(self):
		# UTdebug.log(15, "\n\n**********Raising both arms.")
		print("\n\n\n***************************************\
			************************************************\
			*************\
			Raising both arms.\n\n\n\n")		
		stand = pose.StandStraight()
		stand.run()

class NotSeen(Node):
	def run(self):
		i =0;


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

		if vx < 0 and ball.seen:
			norm_vx = vx / v
			norm_vy = vy / v
			end_x = x + distance * norm_vx
			end_y = y + distance * norm_vy

			print ("Distance: ", distance)
			print ("Predict velocity: ", vx, vy)
			print ("Predict end y: ", end_y)
			# May need moving average for y. Fluctuating v can cause problems
		
		# if v > V_THRESHOLD and vx < 0 and ball.seen and abs(end_y) < GOAL_SIDE:
			if end_y < -CENTER_THRESHOLD:
				choice =  "right"
			elif end_y > CENTER_THRESHOLD:
				choice = "left"
			else:
				choice = "center"
			
			self.postSignal(choice)
			print("\n\n\n\n\n*******************************************\n", choice, "\n\n\n\n\n\n")
		# else:
		# 	norm_vx = 0.0
		# 	norm_vy = 0.0
		# 	bearing = math.atan2(x, y)

		# commands.setHeadPan(ball.bearing, 0.1)
		else:
			print("\n\n\n\n\n&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&\n", 'notnotnotnotnot_seen', 'v is', v, 'vx is', vx, "\n\n\n\n\n\n")
			self.postSignal('not_seen')
		

class Playing(LoopingStateMachine):
	def setup(self):
		raiseArm = RaiseArms()
		arms = {"left": pose.RaiseLeftArm(),
		"right": pose.RaiseRightArm(),
		"center": pose.RaiseBothArms(),
		"not_seen": notSeen()
		}

		for direction in arms:
			arm = arms[direction]
			self.add_transition(raiseArm, S(direction), arm, T(1), raiseArm)
