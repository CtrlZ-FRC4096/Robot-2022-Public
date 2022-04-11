"""
Ctrl-Z FRC Team 4096
FIRST Robotics Competition 2020
Code for robot "----"
contact@team4096.org
"""

# This is to help vscode
from typing import TYPE_CHECKING
from wpimath.controller import PIDController

import wpimath
if TYPE_CHECKING:
	from robot import Robot

from commands2 import Subsystem

import rev

import const

from math import cos, sin, radians, atan2, sqrt
import math

### CLASSES ###

class Turret(Subsystem):

	def __init__(self, robot: "Robot"):

		super().__init__()

		self.is_automatic = False
		self.moving = False

		#### janky shooting while moving stuff
		# target_asjusted_dist is the distance to target when leading. Away is positive
		self.target_adjusted_dist = 0
		# tangential_hub_speed is the tangential speed of the robot with respect to the hub. Right is positive
		self.tangential_hub_speed = 0
		# colinear_hub_speed is the speed of the robot away from the hub. Away is positive
		self.colinear_hub_speed = 0
		# theta_fudge is the angle added due to the tangential speed of the robot with respect to the hub. CCW is positive
		self.theta_fudge = 0
		# hangtime of the shot
		self.tof = 0

		self.robot = robot

		self.reduction = (29/84)*(29/84)*(18/180)

		# Heading target
		self.target = 360

		# When False will turn to the exact heading, otherwise will turn to an optimized heading
		self.optimize = True

		# PID Controller setup, measured in turret degrees
		self.pid = PIDController(
            0.012,
            0,
            0.00012,
        )

		self.pid.setTolerance(1.0)
		self.pid.reset()

		# Motors
		self.motor = rev.CANSparkMax(const.CAN_TURRET, rev.CANSparkMaxLowLevel.MotorType.kBrushless)

		# reset controllers to a known/factory state
		self.motor.restoreFactoryDefaults()
		self.motor.setIdleMode(rev.CANSparkMax.IdleMode.kBrake)
		self.motor.setSmartCurrentLimit(20)

		# sets turret motor's encoder so CCW rotation of the turret is positive <-- This needs to be checked and updated
		self.motor.setInverted(const.TURRET_INVERTED)

		self.encoder = self.motor.getEncoder()
		self.motor.setInverted(const.TURRET_INVERTED)
		self.encoder.setPositionConversionFactor(self.reduction*360)
		self.encoder.setPosition(self.target)

	def get_heading(self):
		'''
		Returns heading of the turret.
		Counterclockwise is positive.
		0 and 360 point directly away from the intake,
		180 and 540 point towards the intake
		'''
		return self.encoder.getPosition()

	# def safe_run(self, value):
	# 	'''
	# 	Runs turret at speed unless at limit based off of encoder and hall effect sensors, positive is CCW
	# 	@parameter (double) value		Speed for the turret to rotate at
	# 	'''
	# 	if value > 0 and self.get_heading() > 270:
	# 		if self.ccw_switch.get() == False or self.get_heading() >= 540:
	# 			self.motor.stopMotor()
	# 		else:
	# 			self.motor.set(value)
	# 	elif value < 0 and self.get_heading() < 270:
	# 		if self.cw_switch.get() == False or self.get_heading() <= 0:
	# 			self.motor.stopMotor()
	# 		else:
	# 			self.motor.set(value)

	def run(self, value):
		'''
		Runs turret at speed unless at limit based off of encoder, positive is CCW
		@parameter (double) value		Speed for the turret to rotate at
		'''
		if value > 0 and self.get_heading() > 270:
			if self.get_heading() >= 540:
				self.motor.stopMotor()
			else:
				self.motor.set(value)
		elif value < 0 and self.get_heading() < 270:
			if self.get_heading() <= 0:
				self.motor.stopMotor()
			else:
				self.motor.set(value)

	def rotate_left(self, speed):
		self.run(-speed)

	def rotate_right(self, speed):
		self.run(speed)

	def stop(self):
		self.motor.stopMotor()

	def optimize_angle(self, angle):
		'''
		Returns optimized angle outside of
		@parameter (double) angle		Angle to be optimized
		'''
		if angle >= 540:
			return angle - 360
		elif angle <= 0:
			return angle + 360
		# if angle is in the lower or upper keep out zones it will rotate the turret 360
		if angle < const.TURRET_STAY_OUT:
			return angle + 360
		elif angle > 540 - const.TURRET_STAY_OUT:
			return angle - 360
		# if angle is in an area where it can 360, returns the closest angle
		elif angle < 180 - const.TURRET_STAY_OUT:
			if abs(self.get_heading() - angle) > 180:
				return angle + 360
			else:
				return angle
		elif angle < 180 + const.TURRET_STAY_OUT:
			return angle
		elif abs(self.get_heading() - angle) > 180:
			return angle - 360
		# only area that is left is an area where it can't 360, so returns angle
		else:
			return angle

	def set_target(self, target, optimize, moving):
		'''
		Sets turret target heading, can be optimized
		@parameter (double) angle			Target angle
		@parameter (boolean) optimize		Whether or not to optimize the target angle
		@parameter (boolean) lead			Whether or not to "lead" the shot based on the robot's tangential velocity with respect to the hub
		'''
		self.optimize = optimize
		self.target = target
		self.moving = moving

	def recalc_adjusted_dist(self):
		# finds angle between robot and hub measured in degrees from away, CCW is positive
		angle_to_hub = (self.robot.turret.get_heading() + 180 - self.robot.limelight.get_angle_to_target()) % 360
		
		# pulls chassis speeds from drivetrain and calculates tangential and colinear speed
		vx = self.robot.drivetrain.chassis_speeds.vx
		a = -vx * cos(radians(angle_to_hub))
		vy = self.robot.drivetrain.chassis_speeds.vy
		b = vx * sin(radians(angle_to_hub))
		c = vy * cos(radians(angle_to_hub))
		d = -vy * sin(radians(angle_to_hub))
		self.tangential_hub_speed = a + c
		self.colinear_hub_speed = b + d
		dist = self.robot.limelight.get_distance_to_target()
		real_dist = const.LL_TO_REAL_DIST(dist)
		self.tof = const.DIST_TO_TOF(real_dist)
		added_y_dist = self.tof * self.tangential_hub_speed
		self.target_adjusted_dist = sqrt(dist ** 2 + added_y_dist ** 2)
		self.theta_fudge = atan2(self.target_adjusted_dist, added_y_dist)
		self.target += self.theta_fudge

	def set_target_limelight(self):
		if abs(self.target - self.get_heading()) > 10:
			return
		self.set_target(self.get_heading() - self.robot.limelight.get_angle_to_target(), True, self.moving)
		if self.moving:
			self.recalc_adjusted_dist()

	def periodic(self):
		"""
		Calculates and sets motor output
		"""
		# if not self.is_automatic:
		# 	self.motor.stopMotor()
		# 	return



		if self.is_automatic:
			self.set_target_limelight()

		if self.target is not None and self.optimize:
			self.target = self.optimize_angle(self.target)
			output = self.pid.calculate(self.get_heading(), self.target)
			if abs(output) > 0.01:
				output += math.copysign(0.03, output)
			self.motor.set(output)
		elif self.target != None:
			output = self.pid.calculate(self.get_heading(), self.target)
			if abs(output) > 0.01:
				output += math.copysign(0.03, output)
			self.motor.set(output)
		else:
			self.motor.stopMotor()

	def log(self):
		'''
		logs various things about this subsystem
		'''
		self.robot.nt_robot.putNumber("Turret Heading", self.get_heading())
		self.robot.nt_robot.putNumber("limelight", self.robot.limelight.get_angle_to_target())
		self.robot.nt_robot.putBoolean("Turret Auto Tracking", self.is_automatic)





