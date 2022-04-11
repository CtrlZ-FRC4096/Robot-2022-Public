"""
Ctrl-Z FRC Team 4096
FIRST Robotics Competition 2022
Code for robot "Chao-Z"
contact@team4096.org

Some code adapted from:
https://github.com/SwerveDriveSpecialties
"""

# This is to help vscode
from math import sqrt, atan2, radians, pi, copysign
from typing import TYPE_CHECKING
from numpy import sign

from wpimath.geometry._geometry import Pose2d
if TYPE_CHECKING:
	from robot import Robot
import const

# from wpilib.command import Command, CommandGroup, WaitCommand
import commands2
from commands2 import Command, CommandBase

import wpimath.kinematics
from wpimath.geometry import Translation2d, Rotation2d


class Drive_Swerve(Command):
	def __init__(self, robot: "Robot", get_left_right, get_forward_back, get_rotate):
		'''
		initializes tank drive movement.
		:param robot: the robot object
		:param get_r: Used to get the x angle, function that determines y direction
		:param get_y: Used to get the y angle, function that determines the y value and direction
		rotation and direction of rotation. Z value must be given if it separate from the joystick.
		'''
		super().__init__()

		self.robot = robot

		self.hasRequirement(self.robot.drivetrain)
		# self.setInterruptible(True)

		self.get_left_right = get_left_right
		self.get_forward_back = get_forward_back
		self.get_rotate = get_rotate

	# Ovrerides
	def getRequirements(self):
		return set( [self.robot.drivetrain] )

	# Methods
	def execute(self):
		"""
		Get r and y values from our joystick axes and send them to subsystem
		"""
		left_right = self.get_left_right()
		forward_back = self.get_forward_back()
		rotate = self.get_rotate()

		# left_right = self.get_left_right.getAsDouble()
		# forward_back = self.get_forward_back.getAsDouble()
		# rotation = self.get_rotate.getAsDouble()

		# print('joy =', left_right)
		# print('{0:.2f}, {1:.2f}, {2:.2f}'.format( left_right, forward_back, rotate ))

		"""
		def fromFieldRelativeSpeeds(
			vx: meters_per_second,
			vy: meters_per_second,
			omega: radians_per_second,
			robotAngle: wpimath.geometry._geometry.Rotation2d)

			-> ChassisSpeeds:
		"""

		# ChassisSpeeds does it weird, +X is considered forward, +Y is left
		chassis_speeds = wpimath.kinematics.ChassisSpeeds.fromFieldRelativeSpeeds(
		    vx = forward_back,
		    vy = -left_right,
		    omega = rotate * 1.1,
		    robotAngle = self.robot.drivetrain.get_gyro_heading(),
		)

		self.robot.drivetrain.drive(chassis_speeds)

	def isFinished(self):
		return False

	def end(self, interrupted):
		self.robot.drivetrain.stop()

	def interrupted(self):
		self.end(True)

class Drive_Swerve_Field_Steering(Command):
	def __init__(self, robot: "Robot", get_left_right, get_forward_back, get_turn_x, get_turn_y):
		'''
		Drives the robot with field oriented steering
		:param robot: the robot object
		:param get_left_right: left to right translation
		:param get_forward_back: forward-back translation
		:param turn_x: turning stick x input
		:param turn_y: turning stick y input
		rotation and direction of rotation. Z value must be given if it separate from the joystick.
		'''
		super().__init__()

		self.robot = robot

		self.hasRequirement(self.robot.drivetrain)
		# self.setInterruptible(True)

		self.get_left_right = get_left_right
		self.get_forward_back = get_forward_back
		self.get_turn_x = get_turn_x
		self.get_turn_y = get_turn_y

	# Ovrerides
	def getRequirements(self):
		return set( [self.robot.drivetrain] )

	# Methods
	def execute(self):
		"""
		Get r and y values from our joystick axes and send them to subsystem
		"""
		left_right = self.get_left_right()
		forward_back = self.get_forward_back()
		turn_x = self.get_turn_x()
		turn_y = self.get_turn_y()
		magnitude = sqrt(turn_x ** 2 + turn_y ** 2)

		if abs(turn_x) > 0.05 or abs(turn_y) > 0.05:
			target_heading = (atan2(turn_y, turn_x) + pi/2) % (2*pi)
			self.robot.drivetrain.target_heading = target_heading
		else:
			target_heading = self.robot.drivetrain.target_heading

		# left_right = self.get_left_right.getAsDouble()
		# forward_back = self.get_forward_back.getAsDouble()
		# rotation = self.get_rotate.getAsDouble()

		# print('joy =', left_right)
		# print('{0:.2f}, {1:.2f}, {2:.2f}'.format( left_right, forward_back, rotate ))

		"""
		def fromFieldRelativeSpeeds(
			vx: meters_per_second,
			vy: meters_per_second,
			omega: radians_per_second,
			robotAngle: wpimath.geometry._geometry.Rotation2d)

			-> ChassisSpeeds:
		"""

		current_heading = self.robot.drivetrain.get_gyro_heading().radians()
		if abs(target_heading - current_heading) > pi:
			target_heading += 2*pi * sign(current_heading - target_heading)

		omega = self.robot.drivetrain.theta_pid.calculate(current_heading, target_heading)

		# error = target_heading - current_heading

		# omega = 0.5 * error / pi

		# print("target_heading", target_heading)
		# print("current_heading", current_heading)
		# print("omega", omega)
		# print("------------")


		# ChassisSpeeds does it weird, +X is considered forward, +Y is left
		chassis_speeds = wpimath.kinematics.ChassisSpeeds.fromFieldRelativeSpeeds(
		    vx = forward_back,
		    vy = -left_right,
		    omega = omega,
		    robotAngle = self.robot.drivetrain.get_gyro_heading(),
		)

		self.robot.drivetrain.drive(chassis_speeds)

	def isFinished(self):
		return False

	def end(self, interrupted):
		self.robot.drivetrain.stop()

	def interrupted(self):
		self.end(True)


class Set_Gyro(Command):
	def __init__(self, robot: "Robot", new_heading = 0):
		'''
		Sets gyro, default is 0
		'''
		super().__init__()

		self.robot = robot
		self.new_heading = new_heading

		self.hasRequirement(self.robot.drivetrain)
		# self.setInterruptible(True)

	# Ovrerides
	def getRequirements(self):
		return set( [self.robot.drivetrain] )

	# Methods
	def execute(self):
		"""
		Resets gyro
		"""
		self.robot.drivetrain.set_gyro(self.new_heading)
		# self.robot.drivetrain.target_heading = 0

	def isFinished(self):
		return True

	def end(self, interrupted):
		pass

	def interrupted(self):
		self.end(True)

class Reset_Odometry(Command):
	def __init__(self, robot: "Robot"):
		'''
		Resets odometry
		'''
		super().__init__()

		self.robot = robot

		self.hasRequirement(self.robot.drivetrain)
		# self.setInterruptible(True)

	# Ovrerides
	def getRequirements(self):
		return set( [self.robot.drivetrain] )

	# Methods
	def execute(self):
		"""
		Resets odometry
		"""
		self.robot.drivetrain.odometry.resetPosition(Pose2d(Translation2d(0, 0), Rotation2d(0)), Rotation2d(0))

	def isFinished(self):
		return True

	def end(self, interrupted):
		pass

	def interrupted(self):
		self.end(True)