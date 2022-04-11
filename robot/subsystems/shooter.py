"""
Ctrl-Z FRC Team 4096
FIRST Robotics Competition 2020
Code for robot "----"
contact@team4096.org
"""

# This is to help vscode
from typing import TYPE_CHECKING
if TYPE_CHECKING:
	from robot import Robot

from commands2 import Subsystem

import const
import ctre


### CLASSES ###

class Shooter(Subsystem):

	def __init__(self, robot: "Robot"):

		super().__init__()

		self.robot = robot

		# create motor controller objects
		self.motor_top = ctre.WPI_TalonFX(const.CAN_SHOOTER_UPPER)
		self.motor_bottom = ctre.WPI_TalonFX(const.CAN_SHOOTER_LOWER)

		# reset controllers to a known/factory state
		self.motor_top.configFactoryDefault()
		self.motor_bottom.configFactoryDefault()

		self.motor_top.setNeutralMode(ctre.NeutralMode.Coast)
		self.motor_bottom.setNeutralMode(ctre.NeutralMode.Coast)

		self.motor_top.setInverted(True)
		self.motor_bottom.setInverted(True)


		self.motor_bottom.configSelectedFeedbackSensor(ctre.FeedbackDevice.IntegratedSensor)
		self.motor_bottom.setSensorPhase(True)

		self.motor_top.configSelectedFeedbackSensor(ctre.FeedbackDevice.IntegratedSensor)
		self.motor_top.setSensorPhase(True)

		self.motor_top.configNominalOutputForward(0)
		self.motor_top.configNominalOutputReverse(0)
		self.motor_top.configPeakOutputForward(1)
		self.motor_top.configPeakOutputReverse(0)

		self.motor_bottom.configNominalOutputForward(0)
		self.motor_bottom.configNominalOutputReverse(0)
		self.motor_bottom.configPeakOutputForward(1)
		self.motor_bottom.configPeakOutputReverse(0)


		self.motor_top.config_kF(0, 1023*.416/9150)
		self.motor_top.config_kP(0, 0.05)
		self.motor_top.config_kI(0, 0.001)
		self.motor_top.config_kD(0, 0)
		self.motor_top.config_IntegralZone(0, 500)

		self.motor_bottom.config_kF(0, 1023*.416/9700)
		self.motor_bottom.config_kP(0, 0.084811)
		self.motor_bottom.config_kI(0, 0.001)
		self.motor_bottom.config_kD(0, 0)
		self.motor_bottom.config_IntegralZone(0, 500)


		# TRY THIS - Voltage Compensation
		self.motor_top.enableVoltageCompensation(True)
		self.motor_top.configVoltageCompSaturation(12.0)

		self.motor_bottom.enableVoltageCompensation(True)
		self.motor_bottom.configVoltageCompSaturation(12.0)


		# TRY THIS - smaller velocity measurement window
		self.motor_top.configVelocityMeasurementPeriod( ctre.SensorVelocityMeasPeriod.Period_2Ms )
		self.motor_top.configVelocityMeasurementWindow(8)

		self.motor_bottom.configVelocityMeasurementPeriod( ctre.SensorVelocityMeasPeriod.Period_2Ms )
		self.motor_bottom.configVelocityMeasurementWindow(8)

		# nt stuff
		self.pid_mode = False
		self.velocity = False

		self.is_at_speed = False
		self.moving = False

	def set_percent(self, bottom_speed, top_speed):
		# self.motor0.set(ctre.ControlMode.)
		self.motor_bottom.set(ctre.ControlMode.PercentOutput, bottom_speed)
		self.motor_top.set(ctre.ControlMode.PercentOutput, top_speed)
		self.pid_mode = False
		self.velocity = 0
		# print(self.motor0.getSelectedSensorPosition(), self.motor0.getSelectedSensorVelocity())


	def stop(self):
		self.set_percent(0, 0)

	def set_velocity(self, bot_velocity, top_velocity):
		if top_velocity > const.SHOOTER_MAX_VELOCITY or bot_velocity > const.SHOOTER_MAX_VELOCITY:
			print('The desired velocity is greater than the max velocity of the shooter.')

		if bot_velocity < 1:
			self.motor_bottom.set(0)
		else:
			self.motor_bottom.set(ctre.ControlMode.Velocity, bot_velocity, ctre.DemandType.ArbitraryFeedForward, 0.0767)

		if top_velocity < 1:
			self.motor_top.set(0)
		else:
			self.motor_top.set(ctre.ControlMode.Velocity, top_velocity, ctre.DemandType.ArbitraryFeedForward, 0.67203 / 12)
		#self.motor_bottom.set(ctre.ControlMode.Velocity, bot_velocity)

		self.pid_mode = True
		self.top_velocity = top_velocity
		self.bot_velocity = bot_velocity
		self.atSpeed = False

	def periodic(self):
		pass

		# # TRY THIS - Derivative Zone (Like Integral Zone)
		# if self.is_at_speed:
		# 	self.motor0.config_kD(40)
		# else:
		# 	self.motor0.config_kD(const.SHOOTER_KD)

	def auto_spin(self):
		if not self.robot.limelight.is_target_visible():
			return

		dist = self.robot.limelight.get_distance_to_target()

		real_dist = const.LL_TO_REAL_DIST(dist)
		real_dist = max(real_dist, real_dist + (real_dist - 100) * 0.1)

		if self.moving:
			real_dist += self.robot.turret.tof * self.robot.turret.colinear_hub_speed * 39.37

		bottom_velocity = const.DIST_TO_BOT(real_dist)
		top_velocity = const.DIST_TO_TOP(real_dist)

		self.robot.shooter.set_velocity(bottom_velocity, top_velocity)

	def log(self):
		'''
		logs various things about this subsystem
		'''
		self.robot.nt_robot.putBoolean("Using Shooter PID", self.pid_mode)
		self.robot.nt_robot.putNumber("Shooter Top Velocity", self.motor_top.getSelectedSensorVelocity())
		self.robot.nt_robot.putNumber("Shooter Bottom Velocity", self.motor_bottom.getSelectedSensorVelocity())
		self.robot.nt_robot.putNumber("Shooter Velocity Setpoint", self.velocity)
		self.robot.nt_robot.putNumber("Shooter Fudge", const.DIST_SHOOTER_FUDGE)
		# self.robot.nt_robot.putBoolean("Shooter At Speed?", self.is_at_speed)
		# print("Shooter at speed" + str(self.motor0.getSelectedSensorVelocity()))
