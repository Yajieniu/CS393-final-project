"""hw1 behavior."""

from __future__ import print_function
from __future__ import division
from __future__ import absolute_import
from math import pi

import memory
import mem_objects
import core
import commands
import cfgstiff
from task import Task
from state_machine import Node, S, C, T, LoopingStateMachine
import UTdebug

CENTER_THRESHOLD = 100
DELAY = 0.6

# Top camera range (0, 1278) x (0, 958)
X_RANGE = 1278
Y_RANGE = 958
X_THETA = 40 * core.DEG_T_RAD
Y_THETA = 30

x_diff = 0
y_diff = 0
# distance of center = 90. breadth = 100



class Stand(Node):
        def run(self):
            commands.stand()
            if self.getTime() > 3.0:
                # memory.speech.say("playing stand complete")
                self.finish()


class On(Node):
	def run(self):
		commands.setStiffness()
		self.finish()

class GazeCenter(Node):
	def run(self):
		self.finish()

class Gaze(Node):
	def run(self):
		commands.setHeadPanTilt(pan=x_diff, tilt=y_diff, time=abs(x_diff)*2+abs(y_diff)/30+0.5, isChange=True)
		if self.getTime() > abs(x_diff)*2+abs(y_diff)/30+0.8:
                	self.finish()

class Gazer(Node):
	def run(self):
		global x_diff
		global y_diff
		ball = mem_objects.world_objects[core.WO_BALL]
		if ball.seen:
			x = ball.imageCenterX
			y = ball.imageCenterY
			print ("Detected ball centroid: ", x, y)

			x_diff = -((x - X_RANGE/2)/X_RANGE)*X_THETA
			y_diff = y_diff - ((y - Y_RANGE/2)/Y_RANGE)*Y_THETA
			# if abs(x_diff) + abs(x_diff) <= 2*CENTER_THRESHOLD:
			# 	choice = "center"
			# elif x_diff <= -CENTER_THRESHOLD:
			# 	choice = "left"
			# elif x_diff >= CENTER_THRESHOLD:
			# 	choice = "right"
			# elif y_diff <= -CENTER_THRESHOLD:
			#  	choice = "top"
			# elif y_diff >= CENTER_THRESHOLD:
			#  	choice = "bottom"

		else:
			choice = "no_ball"
			print ("No ball detected")

			x_diff = 0
			y_diff = -21
	
		print ("moving to", x_diff, y_diff)	
		self.finish()



class Playing(LoopingStateMachine):
	def setup(self):
		gazer = Gazer()
		gaze = Gaze()
		on = On()
		self.add_transition(on, C, gazer)
		self.add_transition(gazer, C, gaze, C, gazer)

		# self.add_transition(gazer, S('no_ball'), gazer)
		# stand = Stand()
		# gaze1 = GazeLeft()
		# gaze2 = GazeLeft()
		# self.trans(stand, C, gaze1, T(3.0), gaze2, T(3.0))
