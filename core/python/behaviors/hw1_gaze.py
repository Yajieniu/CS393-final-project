"""hw1 behavior."""

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

CENTER_THRESHOLD = 100
DELAY = 0.6

class Stand(Node):
        def run(self):
            commands.stand()
            if self.getTime() > 3.0:
                # memory.speech.say("playing stand complete")
                self.finish()


class GazeCenter(Node):
	def run(self):
		self.finish()

class GazeLeft(Node):
	def run(self):
		commands.setHeadPan(0.1, DELAY, True)
		if self.getTime() > DELAY + 0.2:
                	self.finish()
		# commands.setWalkVelocity(0.5, 0, 0)

class GazeRight(Node):
	def run(self):
		commands.setHeadPan(-0.1, DELAY, True)
                if self.getTime() > DELAY + 0.2:
                        self.finish()

class GazeTop(Node):
	def run(self):
		commands.setHeadTilt(-20)
		if self.getTime() > DELAY:
                        self.finish()

class GazeBottom(Node):
	def run(self):
		# print ("bottom")
		commands.setHeadTilt(-22)
		if self.getTime() > DELAY:
                        self.finish()
		# commands.setHeadPan(1.0, 2.0)

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
		print (choice)
		self.postSignal(choice)



class Playing(LoopingStateMachine):
	def setup(self):
		gazer = Gazer()
		stand = Stand()
		gazes = {
			'center': GazeCenter(),
			'left': GazeLeft(),
			'right': GazeRight(),
			'top': GazeTop(),
			'bottom': GazeBottom(),
		}
		self.add_transition(stand, C, gazer)
		for name in gazes:
			self.add_transition(gazer, S(name), gazes[name], C, gazer)

		# self.add_transition(gazer, S('no_ball'), gazer)
		# stand = Stand()
		# gaze1 = GazeLeft()
		# gaze2 = GazeLeft()
		# self.trans(stand, C, gaze1, T(3.0), gaze2, T(3.0))
