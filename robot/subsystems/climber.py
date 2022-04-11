"""
Ctrl-Z FRC Team 4096
FIRST Robotics Competition 2020
Code for robot "------"
contact@team4096.org
"""
# This is to help vscode
from typing import TYPE_CHECKING
if TYPE_CHECKING:
	from robot import Robot

import wpilib
from commands2 import Subsystem

import const

import ctre


class Climber(Subsystem):
	def __init__(self, robot: "Robot"):
		super().__init__()

		self.robot = robot

		self.motor_lift_1 = ctre.WPI_TalonFX(const.CAN_CLIMBER_LIFT_1, canbus="tooth")
		self.motor_lift_2 = ctre.WPI_TalonFX(const.CAN_CLIMBER_LIFT_2, canbus="tooth")

		self.motor_lift_1.setNeutralMode(ctre.NeutralMode.Brake)
		self.motor_lift_2.setNeutralMode(ctre.NeutralMode.Brake)


		self.motor_lift_1.configSupplyCurrentLimit(ctre.SupplyCurrentLimitConfiguration(True, 20, 0, 0))
		self.motor_lift_2.configSupplyCurrentLimit(ctre.SupplyCurrentLimitConfiguration(True, 20, 0, 0))

		self.motor_lift_1.configForwardSoftLimitThreshold(175_000)
		self.motor_lift_1.configForwardSoftLimitEnable(True)

		self.motor_lift_2.follow(self.motor_lift_1)


		# Solenoids
		self.deploy_solenoid = wpilib.DoubleSolenoid(
			wpilib.PneumaticsModuleType.REVPH,
			const.PCM_SOLENOID_CLIMBER_DEPLOY_1,
			const.PCM_SOLENOID_CLIMBER_DEPLOY_2,
		)

		self.tilt_solenoid = wpilib.DoubleSolenoid(
			wpilib.PneumaticsModuleType.REVPH,
			const.PCM_SOLENOID_CLIMBER_TILT_BRAKE_2,
			const.PCM_SOLENOID_CLIMBER_TILT_BRAKE_1,
		)

		self.lift_brake_solenoid = wpilib.DoubleSolenoid(
			wpilib.PneumaticsModuleType.REVPH,
			const.PCM_SOLENOID_CLIMBER_LIFT_BRAKE_2,
			const.PCM_SOLENOID_CLIMBER_LIFT_BRAKE_1,
		)

		self.is_deployed = False


	def wind(self, speed=0.7):
		speed = abs(speed)
		if speed < 0.03 or False: #self.bottom_limit_switch.get():
			self.set_lift_brake_on()
			speed = 0
		else:
			self.set_lift_brake_off()
		self.motor_lift_1.set(-speed)

	def unwind(self, speed=0.7):
		speed = abs(speed)
		if speed < 0.03 or False: # self.top_limit_switch.get():
			self.set_lift_brake_on()
			speed = 0
		else:
			self.set_lift_brake_off()
		self.motor_lift_1.set(speed)


	def stop(self):
		self.motor_lift_1.set(0)
		self.set_lift_brake_on()

	def deploy(self):
		self.deploy_solenoid.set( wpilib.DoubleSolenoid.Value.kReverse )
		self.is_deployed = True

	def undeploy(self):
		self.deploy_solenoid.set( wpilib.DoubleSolenoid.Value.kForward )
		self.is_deployed = False

	def set_tilt_forward(self):
		self.tilt_solenoid.set( wpilib.DoubleSolenoid.Value.kForward )

	def set_tilt_backward(self):
		self.tilt_solenoid.set( wpilib.DoubleSolenoid.Value.kReverse )

	def set_lift_brake_on(self):
		self.lift_brake_solenoid.set( wpilib.DoubleSolenoid.Value.kForward )

	def set_lift_brake_off(self):
		self.lift_brake_solenoid.set( wpilib.DoubleSolenoid.Value.kReverse )

	def set_lift_brake(self, state: bool):
		if state:
			self.set_lift_brake_on()
		else:
			self.set_lift_brake_off()

	def get_lift_position(self):
		return self.motor_lift_1.getSelectedSensorPosition()

	def log(self):
		# print( 'ty = {0:.2f}'.format( self.nt_table.getNumber('ty', 0)) )
		pass
