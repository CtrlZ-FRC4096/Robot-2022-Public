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

from commands2 import Subsystem
import wpilib
import ctre

import const

class Intake(Subsystem):
	def __init__(self, robot: "Robot"):
		super().__init__()

		self.robot = robot

		self.motor = ctre.WPI_TalonFX(const.CAN_INTAKE, canbus = 'tooth')
		#self.motor = wpilib.PWMTalonFX(const.PWM_INTAKE)

		self.piston_deploy = wpilib.DoubleSolenoid(
			wpilib.PneumaticsModuleType.REVPH,
			const.PH_SOLENOID_INTAKE_DEPLOY_1,
			const.PH_SOLENOID_INTAKE_DEPLOY_2,
		)

		self.piston_kicker = wpilib.DoubleSolenoid(
			wpilib.PneumaticsModuleType.REVPH,
			const.PH_SOLENOID_INTAKE_KICKER_1,
			const.PH_SOLENOID_INTAKE_KICKER_2,
		)

		self.running_in = False

	def _run(self, speed):
		self.motor.set(speed)
		self.running_in = True

	def intake(self, speed=0.8):
		self._run(abs(speed))
		self.running_in = True

	def outtake(self, speed=0.8):
		self._run(-abs(speed))
		self.running_in = False

	def stop(self):
		self._run(0)
		self.running_in = False

	def down(self):
		self.piston_deploy.set(wpilib.DoubleSolenoid.Value.kForward)
		self.piston_kicker.set(wpilib.DoubleSolenoid.Value.kForward)

	def up(self):
		self.piston_deploy.set(wpilib.DoubleSolenoid.Value.kReverse)
		self.piston_kicker.set(wpilib.DoubleSolenoid.Value.kReverse)

	def log(self):
		self.robot.nt_robot.putBoolean('intake running', self.running_in)
		pass
