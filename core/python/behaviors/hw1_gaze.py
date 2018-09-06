"""hw1 behavior."""

from __future__ import print_function
from __future__ import division
from __future__ import absolute_import

import mem_objects
import core
import commands
import cfgstiff
from task import Task
from state_machine import Node, S, C, T, LoopingStateMachine
import UTdebug

CENTER_THRESHOLD = 50

class GazeCenter(Node):
	def run(self):
		pass

class GazeLeft(Node):
	def run(self):
		commands.setHeadPan(0.5, 2.0, True)


class GazeRight(Node):
	def run(self):
		commands.setHeadPan(-0.5, 2.0, True)

class GazeTop(Node):
	def run(self):
		commands.setHeadTilt(-20)

class GazeBottom(Node):
	def run(self):
		commands.setHeadTilt(-22)


class Gazer(Node):
	def run(self):
		ball = mem_objects.world_objects[core.WO_BALL]
		if ball.seen:
			x = ball.imageCenterX
			y = ball.imageCenterY
			print ("Detected ball centroid: ", x, y)

			x_diff = x - 640
			y_diff = y - 320
			if abs(x_diff) + abs(x_diff) <= 2*CENTER_THRESHOLD:
				choice = "center"
			elif x_diff <= -CENTER_THRESHOLD:
				choice = "left"
			elif x_diff >= CENTER_THRESHOLD:
				choice = "right"
			elif y_diff <= -CENTER_THRESHOLD:
				choice = "top"
			elif y_diff >= CENTER_THRESHOLD:
				choice = "bottom"

		else:
			choice = "no_ball"
			print ("No ball detected")
		
		self.postSignal(choice)



class Playing(LoopingStateMachine):
	def setup(self):
		gazer = Gazer()
		gazes = {
			'center': GazeCenter(),
			'left': GazeLeft(),
			'right': GazeRight(),
			'top': GazeTop(),
			'bottom': GazeBottom(),
		}
		for name in gazes:
			self.add_transition(gazer, S(name), gazes[name], T(5), gazer)

		self.add_transition(gazer, S('no_ball'), gazer)