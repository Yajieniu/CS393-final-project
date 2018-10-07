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
import numpy as np 

# State models: (x, y, vx, vy)

# Measurements:
# imageCenterX, imageCenterY, position p, visionBearing, 
# visionDistance

endingRightThreshold = 200 # in pixel
endingLeftThreshold = 160 # in pixel


class KilmanFilter(object):

	def __init__(self):
		self.n = 4                  # dimen of w
		self.m = 4					# dimen of u
		self.k = 5					# dimen of z

		self.A = np.eye(n)   		# A : n x n, for state
		self.B = np.zeros((n, m))	# B : n x m, for control
		self.C = np.zeros((k, n))	# C : k x n, for covariance
		self.R = np.eye(n)			# R : n x n, state noise covariance
		self.Q = np.eye(k)			# Q : k x k, measurement noise

		self.Cov = np.eye(n)        # covariance matrix, n x n
		self.w = np.ones(n)   		# states, starts as uniform distribution
		self.u = np.zeros((m, 1)) 	# control, always 0
		self.z = np.ones(k)			# needs to be updated before use

	def __call__(self, zt):	
		self.z = zt



class RaiseLeft(Node):
	# raise left arm 
	def run(self):
		UTdebug.log(15, "Raising left arm.")


class RaiseRight(Node):
	# raise right arm
	def run(self):
		UTdebug.log(15, "Raising right arm.")


class RaiseBoth(Node):
	# raise both arms
	def run(self):
		UTdebug.log(15, "Raising both arms.")


class RaiseArms(Node):
	def run(self):
		# get predicted location and velocity
		ball = mem_objects.world_objects[core.WO_BALL]

		# assuming accerlaration is 0
		endX = ball.worldX + ball.worldY * ball.veloX / ball.veloY

		if endX > endingRightThreshold:
			choice = "right"
		elif endX < endingLeftThreshold:
			choice = "left"
		else:
			choice = "center"

		self.poseSignal(choice)


class Playing(LoopingStateMachine):
	def setup(self):
		raiseArm = RaiseArms()
		arms = {"left": RaiseLeft(),
			   "right": RaiseRight(),
			   "center": RaiseBoth()
			   }

	   	for direction in arms:
	   		arm = arms[direction]
	   		self.add_transition(raiseArm, S(direction), arm, T(6), raiseArm)
