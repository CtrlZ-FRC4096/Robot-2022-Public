"""
Ctrl-Z FRC Team 4096
FIRST Robotics Competition 2020
Code for robot "------"
contact@team4096.org
"""
from typing import TYPE_CHECKING, List, Optional

# This is to help vscode
import dill as pickle

if TYPE_CHECKING:
	from robot import Robot

import enum


import const
import networktables

import wpilib
from commands2 import Subsystem


class BallLocation(enum.Enum):
	LOWEST = "lowest"
	FLOATING = "floating"
	TOP = "top"
	AIR = "air"

class Ball:
	color: Optional[str]
	location: Optional[BallLocation]

	def __init__(self, color: Optional[str], location: Optional[BallLocation]) -> None:
		self.color = color
		self.location = location


class Indexer(Subsystem):
	def __init__(self, robot: "Robot"):
		super().__init__()

		self.is_automatic = True

		self.robot = robot

		self.balls: List[Ball] = []

		self.motor_lower = wpilib.PWMSparkMax(const.PWM_INDEXER_LOWER)
		self.motor_top = wpilib.PWMSparkMax(const.PWM_INDEXER_UPPER)
		self.motor_top.setInverted(True)

		# Network tables storing color values (change later)
		self.nt_inst = networktables.NetworkTablesInstance.getDefault()

		# limit switch
		# IMPORTANT: LIMIT SWITCH GROUND WIRE IS BLUE (not black)
		self.limit_switch_lower = wpilib.AnalogInput(const.AIO_INDEXER_LOWER)
		self.limit_switch_upper = wpilib.AnalogInput(const.AIO_INDEXER_UPPER)

		# Tracks color of balls in the indexer
		# None is no ball
		self.upper_ball_color = None
		self.lower_ball_color = None
		self.upper_ball_in_pos = False
		self.lower_ball_in_pos = False

		self.should_feed = False

	def get_lowest_switch_ball_color(self):
		return "red" if self.is_ball_red() else "blue"

	def get_limit_switch_lowest(self):
		#color sensor
		color_entry = self.nt_inst.getEntry('/proximity1')
		color = color_entry.getDouble(None)
		if color == None:
			return False

		if color > 200:
			return True

		return False

	def get_limit_switch_lower(self):
		#lower indexer proximity sensor
		if self.limit_switch_lower.getVoltage() < 1:
			return True
		return False

	def get_limit_switch_upper(self):
		#upper indexer proximity sensor
		if self.limit_switch_upper.getVoltage() < 1:
			return True
		return False

	def run_top(self, speed=0.75):
		self.motor_top.set(speed)

	def run_lower(self, speed=0.75):
		self.motor_lower.set(speed)


	def intake(self):
		# self.run_lower(0.5)
		self.run_top(1)
		self.run_lower(1)

	def outtake(self):
		self.run_lower(-1)
		self.run_top(-1)


	def stop(self):
		self.should_feed = False
		self.run_lower(0)
		self.run_top(0)

	def get_color(self):
		color_entry = self.nt_inst.getEntry('/rawcolor1')
		color = color_entry.getDoubleArray([])

		if color:
			r = color[0]
			g = color[1]
			b = color[2]
			mag = r + g + b
			return [r / mag, b / mag , g / mag]
		else:
			return None

	def is_ball_red(self):
		ball_color = self.get_color()

		if ball_color and ball_color[0] > ball_color[2]:
			return True
		return False

	def locate_balls(self):
		lowest = self.get_limit_switch_lowest()
		lowest_color = self.get_lowest_switch_ball_color()
		upper = self.get_limit_switch_upper()

		if not lowest:
			for ball in self.balls:
				if ball.location == BallLocation.LOWEST:
					ball.location = BallLocation.FLOATING

		if lowest and all(ball.location != BallLocation.LOWEST for ball in self.balls):
			self.balls.append(Ball(color=lowest_color, location=BallLocation.LOWEST))

		if not upper:
			self.balls = [ball for ball in self.balls if ball.location != BallLocation.TOP]

		if upper and all(ball.location != BallLocation.TOP for ball in self.balls):
			for ball in self.balls:
				if ball.location == BallLocation.FLOATING:
					ball.location = BallLocation.TOP


	def auto_serialize(self):
		lowest = self.get_limit_switch_lowest()
		upper = self.get_limit_switch_upper()

		lowest = any(ball.location == BallLocation.LOWEST for ball in self.balls)
		floating = any(ball.location == BallLocation.FLOATING for ball in self.balls)
		upper = any(ball.location == BallLocation.TOP for ball in self.balls)

		if self.should_feed:
			self.run_top()
			self.run_lower()
			return

		if len(self.balls) >= 2:
			self.robot.intake.stop()
			self.robot.intake.up()

		if upper and floating:
			self.run_lower(0)
			self.run_top(0)
			self.robot.intake.stop()
			self.robot.intake.up()
			return

		if upper and not floating and lowest:
			self.run_top(0)
			self.run_lower()
			return

		if not upper and (floating or lowest):
			self.run_top()
			self.run_lower()
			return

		if upper and not floating and not lowest:
			self.run_top(0)
			if self.robot.intake.running_in:
				self.run_lower()
			else:
				self.run_lower(0)

		if not upper and not floating and not lowest:
			if self.robot.intake.running_in:
				self.run_top()
				self.run_lower()
			else:
				self.run_top(0)
				self.run_lower(0)

	def periodic(self):
		self.locate_balls()
		if self.is_automatic:
			self.auto_serialize() # TODO CHANGE THIS LATER

	def log(self):
		self.robot.nt_robot.putBoolean('Lowest Switch', self.get_limit_switch_lowest())
		self.robot.nt_robot.putBoolean('Upper Switch', self.get_limit_switch_upper())
		self.robot.nt_robot.putValue('Lower Color', self.lower_ball_color)
		self.robot.nt_robot.putValue('Upper Color', self.upper_ball_color)
		self.robot.nt_robot.putBoolean('lower in pos', self.lower_ball_in_pos)
		self.robot.nt_robot.putBoolean('upper in pos', self.upper_ball_in_pos)
		self.robot.nt_robot.putBoolean("shouldFeed", self.should_feed)
		# self.robot.nt_robot.putBoolean('Blah', True)
		self.robot.nt_robot.putRaw("indexer balls", pickle.dumps(self.balls))

