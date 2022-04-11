"""
Ctrl-Z FRC Team 4096
FIRST Robotics Competition 2020
Code for robot "Chao-Z"
contact@team4096.org
"""

# This is to help vscode
from typing import TYPE_CHECKING
from wpimath.controller import PIDController

if TYPE_CHECKING:
	from robot import Robot

from commands2 import Subsystem

import rev

import const

### CLASSES ###

class Hood(Subsystem):

	def __init__(self, robot: "Robot"):

		super().__init__()

		self.robot = robot

		self.reduction = (13/68)*(13/68)*(19/13)*(11/266)

		# Heading target
		self.target = 4
		self.is_automatic = True
		self.moving = False

		# PID Controller setup, measured in degrees from horizontal
		self.pid = PIDController(
			0.06,
			0.002,
			0.001,
		)
		self.pid.setTolerance(0.1)
		self.pid.reset()

		# Motor
		self.motor = rev.CANSparkMax(const.CAN_HOOD, rev.CANSparkMaxLowLevel.MotorType.kBrushless)

		# reset controllers to a known/factory state
		self.motor.restoreFactoryDefaults()
		self.motor.setIdleMode(rev.CANSparkMax.IdleMode.kBrake)
		self.motor.setSmartCurrentLimit(20)

		# sets turret motor's encoder's positivity
		self.motor.setInverted(const.HOOD_INVERTED)

		self.encoder = self.motor.getEncoder()
		self.motor.setInverted(const.HOOD_INVERTED)
		self.encoder.setPositionConversionFactor(self.reduction*360)
		self.encoder.setPosition(self.target)

		self.lower_limit = const.HOOD_LOWER_LIMIT
		self.upper_limit = const.HOOD_UPPER_LIMIT

	def get_angle(self):
		'''
		Returns angle of the hood in degrees from horizontal
		'''
		return self.encoder.getPosition()

	def run(self, value):
		'''
		Runs hood motor at speed unless at limit based off of encoder, positive is up
		@parameter (double) value        Speed for the hood to move at
		'''
		if value > 0 and self.get_angle() >= self.upper_limit:
			self.motor.stopMotor()
		elif value < 0 and self.get_angle() <= self.lower_limit:
			self.motor.stopMotor()
		else:
			self.motor.set(value)

	def rotate_down(self, speed):
		self.run(-speed)

	def rotate_up(self, speed):
		self.run(speed)

	def stop(self):
		self.motor.stopMotor()

	def at_target(self):
		if self.target is None:
			return False
		if abs(self.get_angle() - self.target) <= 0.5:
			return True
		return False

	def set_target(self, target):
		'''
		Sets target hood angle
		@parameter (double) angle Target angle
		'''
		if target > self.lower_limit and target < self.upper_limit:
			self.target = target
		else:
			print("Hood target out of range!")

	def set_target_limelight(self):
		'''
		Sets target hood angle, can take movement into consideration
		@parameter (double) angle Target angle
		@parameter (boolean) moving Whether or not to take the robot's movement into account
		'''
		# distance as reported by the limelight
		if not self.robot.limelight.is_target_visible():
			return

		dist = self.robot.limelight.get_distance_to_target()
		real_dist = const.LL_TO_REAL_DIST(dist)

		if self.moving:
			#calculated fudge based on how fast the robot is moving away from the
			real_dist += self.robot.turret.tof * self.robot.turret.colinear_hub_speed * 39.37

		angle = const.DIST_TO_HOOD(real_dist)
		if dist >= 100:
			angle += 2

		self.set_target(angle)

	def periodic(self):
		"""
		Calculates and sets motor output
		"""
		# if not self.is_automatic:
		# self.motor.stopMotor()
		# return

		if self.is_automatic:
			self.set_target_limelight()

		if self.target is not None:
			output = self.pid.calculate(self.get_angle(), self.target)
			#output += math.copysign(0.03, output)
			self.motor.set(output)
		else:
			self.motor.stopMotor()

	def log(self):
		'''
		logs various things about this subsystem
		'''
		self.robot.nt_robot.putNumber("Hood Angle", self.get_angle())
		self.robot.nt_robot.putBoolean("At Target", self.at_target())
		self.robot.nt_robot.putBoolean("Hood Auto Tracking", self.is_automatic)