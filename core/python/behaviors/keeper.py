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
V_THRESHOLD = 250



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
		
		end_y = y + x * vy/vx
		print ("\n\n\n\n End y: ", end_y)
		print ("\n\n\n\n V: ", v)
		print ("\n\n\n\n bearing: ", ball.bearing)


		if not ball.seen:
			print ("\n\n\nball not seen!\n\n\n")
			commands.setHeadPan(0, 0.05, )
			choice = "unseen"
			# print ("\n\n\n\n Ball moving!!! \n\n\n\n")
		elif v < V_THRESHOLD:
			commands.setHeadPan(ball.bearing, 0.05)
			print ("\n\n\nball steady\n\n\n")
			choice = "nomove"
		else:
			commands.setHeadPan(ball.bearing, 0.05)
			print ("\n\n\n ball moving!!!\n\n\n")

			# end_y = y + x * vy/vx
			# print ("\n\n\n\n End y: ", end_y)
			# print ("\n\n\n\n V: ", v)
			# print ("\n\n\n"v, )

			if v > V_THRESHOLD:
				if end_y < -CENTER_THRESHOLD:
					print ("\n\n\n\nleft!!!!!\n\n\n")
					choice = "left"
				elif end_y > CENTER_THRESHOLD:
					print ("\n\n\n\nright!!!!!\n\n\n")
					choice = "right"
				else:
					print ("\n\n\n\ncenter!!!!!\n\n\n")
					choice = "center"

			else:
				choice = "nomove"

					# if vx < 0 and ball.seen:
		# 	norm_vx = vx / v
		# 	norm_vy = vy / v
		# 	end_x = x + distance * norm_vx
		# 	end_y = y + distance * norm_vy

		# 	print ("Distance: ", distance)
		# 	print ("Predict velocity: ", vx, vy)
		# 	print ("Predict end y: ", end_y)
		# 	# May need moving average for y. Fluctuating v can cause problems
		
		# # if v > V_THRESHOLD and vx < 0 and ball.seen and abs(end_y) < GOAL_SIDE:
		# 	if end_y < -CENTER_THRESHOLD:
		# 		choice =  "right"
		# 	elif end_y > CENTER_THRESHOLD:
		# 		choice = "left"
		# 	else:
		# 		choice = "center"
			
		# 	self.postSignal(choice)
		# 	print("\n\n\n\n\n*******************************************\n", choice, "\n\n\n\n\n\n")
		# # else:
		# # 	norm_vx = 0.0
		# # 	norm_vy = 0.0
		# # 	bearing = math.atan2(x, y)

		# else:
		# 	print("\n\n\n\n\n&&&&&&&&&&&&&&&&&&&&&&\n", 'notnotnotnotnot_seen or ball not approaching', 'v is', v, 'vx is', vx, "\n\n\n\n\n\n")
		self.postSignal('not_seen')
		

class Playing(LoopingStateMachine):
	def setup(self):
		raiseArm = RaiseArms()
		arms = {"left": pose.RaiseLeftArm(),
		"right": pose.RaiseRightArm(),
		"center": pose.RaiseBothArms(),
		"unseen": NotSeen(),
		"nomove": pose.SittingPose(),
		}

		# arms = {"left": RaiseLeft(),
		# "right": RaiseRight(),
		# "center": RaiseBoth(),
		# "not_seen": NotSeen()
		# }

		for direction in arms:
			arm = arms[direction]
			self.add_transition(raiseArm, S(direction), arm, T(0.05), raiseArm)
