"""
Ctrl-Z FRC Team 4096
FIRST Robotics Competition 2022
Code for robot "swerve drivetrain prototype"
contact@team4096.org

Some code adapted from:
https://github.com/SwerveDriveSpecialties
"""

# This is to help vscode
from typing import TYPE_CHECKING
if TYPE_CHECKING:
	from robot import Robot

from subsystems.shooter import Shooter

import wpilib

from customcontroller.custom_button import CustomButton

from itertools import zip_longest

###  IMPORTS ###

# Subsystems

# Commands
import commands.drivetrain

# Controls
from customcontroller import XboxCommandController
from commands2.button import Button
from commands2 import ParallelCommandGroup, Subsystem

import wpilib.interfaces
import const
from wpilib import Timer


class OI:
	"""
	Operator Input - This class ties together controls and commands.
	"""

	def __init__(self, robot: "Robot"):
		self.robot = robot

		#Controllers

		# self.driver1 = XboxController(0)
		self.driver1 = XboxCommandController(0)
		self.driver2 = XboxCommandController(1)



		self.driver1.LEFT_JOY_Y.setInverted(True)
		self.driver1.RIGHT_JOY_X.setInverted(True)

		self.driver1.LEFT_JOY_X.setDeadzone(.05)
		self.driver1.LEFT_JOY_Y.setDeadzone(.05)
		self.driver1.RIGHT_JOY_X.setDeadzone(.05)
		self.driver1.RIGHT_JOY_Y.setDeadzone(.05)

		def slow_drive_speed():
			self.driver1.LEFT_JOY_X.setModifier(lambda x: x * abs(x) * 0.25)
			self.driver1.LEFT_JOY_Y.setModifier(lambda x: x * abs(x) * 0.25)
			self.driver1.RIGHT_JOY_X.setModifier(lambda x: x * abs(x) * 0.25)

		def regular_drive_speed():
			self.driver1.LEFT_JOY_X.setModifier(lambda x: x * abs(x) * 0.8)
			self.driver1.LEFT_JOY_Y.setModifier(lambda x: x * abs(x) * 0.8)
			self.driver1.RIGHT_JOY_X.setModifier(lambda x: x * abs(x) * 0.9)

		def fast_drive_speed():
			self.driver1.LEFT_JOY_X.setModifier(lambda x: x * abs(x) * 0.93)
			self.driver1.LEFT_JOY_Y.setModifier(lambda x: x * abs(x) * 0.93)
			self.driver1.RIGHT_JOY_X.setModifier(lambda x: x * abs(x) * 0.8)

		regular_drive_speed()
		# self.driver1.LEFT_JOY_X.setModifier(lambda x: x * abs(x) * 0.87)
		# self.driver1.LEFT_JOY_Y.setModifier(lambda x: x * abs(x) * 0.87)
		# self.driver1.RIGHT_JOY_X.setModifier(lambda x: x * abs(x) * 0.75)
		# self.driver1.RIGHT_JOY_Y.setModifier(lambda x: x * abs(x) * 0.75)

		@self.driver2.Y.whenPressed(requirements=[robot.shooter, robot.turret, robot.hood])
		def prep_for_climb():
			robot.turret.is_automatic = False
			robot.hood.set_target(4)
			robot.hood.is_automatic = False
			robot.shooter.stop()
			robot.turret.set_target(180, False, False)

		self.driver2.RIGHT_JOY_Y.setInverted(True)
		self.driver2.LEFT_JOY_Y.setInverted(True)

		### Driving ###

		self.driver1.A.whenPressed(commands.drivetrain.Set_Gyro(self.robot))
		self.driver1.B.whenPressed(commands.drivetrain.Reset_Odometry(self.robot))

		self.drive_command = commands.drivetrain.Drive_Swerve(
			self.robot,
		 	self.driver1.LEFT_JOY_X,
		 	self.driver1.LEFT_JOY_Y,
		 	self.driver1.RIGHT_JOY_X,
		)

		# self.drive_command = commands.drivetrain.Drive_Swerve_Field_Steering(
		# 	self.robot,
		#  	self.driver1.LEFT_JOY_X,
		#  	self.driver1.LEFT_JOY_Y,
		#  	self.driver1.RIGHT_JOY_X,
		# 	self.driver1.RIGHT_JOY_Y,
		# )

		self.robot.drivetrain.setDefaultCommand(self.drive_command)



		# Driver 1 Controls     #################################################

		@self.driver1.RIGHT_BUMPER.whenPressed(requirements = [robot.intake])
		def _():
			robot.intake.down()
			timer = wpilib.Timer()
			timer.start()

			while not timer.hasElapsed(0.2):
				yield

			while True:
				robot.intake.intake(.4)
				yield

		@self.driver1.RIGHT_BUMPER.whenReleased(requirements = [robot.intake])
		def _():
			robot.intake.stop()
			timer = wpilib.Timer()
			timer.start()

			while not timer.hasElapsed(.2):
				yield

			robot.intake.up()

		@self.driver1.LEFT_TRIGGER_AS_BUTTON.whenPressed()
		def _():
			fast_drive_speed()

		@self.driver1.LEFT_BUMPER.whenPressed()
		def _():
			slow_drive_speed()

		# released

		@self.driver1.LEFT_BUMPER.whenReleased()
		def _():
			regular_drive_speed()

		@self.driver1.LEFT_TRIGGER_AS_BUTTON.whenReleased()
		def _():
			regular_drive_speed()



		@self.driver2.POV.UP.whenPressed(requirements=[robot.climber])
		def _():
			robot.climber.set_tilt_forward()


		@self.driver2.POV.DOWN.whenPressed(requirements=[robot.climber])
		def _():
			robot.climber.set_tilt_backward()

		######################################################################################

		@self.driver2.RIGHT_JOY_UP.whenPressed(requirements=[robot.climber])
		def _():
			robot.climber.set_lift_brake_off()

			while True:
				robot.climber.unwind(self.driver2.RIGHT_JOY_Y.get())
				yield

		@self.driver2.RIGHT_JOY_UP.whenReleased(requirements=[robot.climber])
		def _():
			robot.climber.stop()

		@self.driver2.RIGHT_JOY_DOWN.whenPressed(requirements=[robot.climber])
		def _():
			robot.climber.set_lift_brake_off()

			while True:
				robot.climber.wind(self.driver2.RIGHT_JOY_Y.get())
				yield

		@self.driver2.RIGHT_JOY_DOWN.whenReleased(requirements=[robot.climber])
		def _():
			robot.climber.stop()


		@self.driver2.B.whenPressed(requirements=[robot.shooter])
		def _():
			robot.shooter.set_velocity(0, 0)

		class Dummy1(Subsystem):
			pass

		dummy1 = Dummy1()

		@self.driver2.RIGHT_BUMPER.whenPressed(requirements=[dummy1])
		def _():
			while True:
				robot.climber.set_lift_brake_off()
				yield

		@self.driver2.RIGHT_BUMPER.whenReleased(requirements=[dummy1])
		def _():
			robot.climber.set_lift_brake_on()

		@self.driver2.A.whenPressed(requirements=[robot.shooter])
		def _():
			while True:
				yield
				# robot.hood.is_automatic = False
				# robot.hood.set_target(28)
				# robot.shooter.set_velocity(5000, 10000)
				self.robot.shooter.auto_spin()


		@self.driver2.B.whenReleased(requirements=[robot.shooter])
		def _():
			robot.shooter.set_velocity(0, 0)

		@self.driver2.X.whenPressed
		def _():
			robot.indexer.should_feed = True

		@self.driver2.X.whenReleased
		def _():
			robot.indexer.should_feed = False

		@self.driver2.LEFT_STICK.whenPressed
		def _():
			robot.turret.is_automatic = not robot.turret.is_automatic


		@self.driver2.LEFT_TRIGGER_AS_BUTTON.whenPressed
		def _():
			const.DIST_SHOOTER_FUDGE -= 12
			print(f"{const.DIST_SHOOTER_FUDGE=}")

		@self.driver2.RIGHT_TRIGGER_AS_BUTTON.whenPressed
		def _():
			const.DIST_SHOOTER_FUDGE += 12
			print(f"{const.DIST_SHOOTER_FUDGE=}")

		@self.driver1.Y.whenPressed
		def _():
			robot.indexer.is_automatic = not robot.indexer.is_automatic

		return
		self.climb_button = CustomButton(lambda: self.driver2.START() and self.driver2.BACK())
		@self.climb_button.whenPressed(requirements=[robot.climber])
		def _():
			# NEED TO COMMENT OUT BRAKES FROM LIFT AND TILE SUBSYSTEM METHODS
			climber_max = 175000
			def lift_to(setpoint, speed=0.5):
				setpoint = setpoint * climber_max
				if abs(robot.climber.get_lift_position() - setpoint) < 0.001:
					return

				speed = abs(speed)
				if robot.climber.get_lift_position() > setpoint:
					while robot.climber.get_lift_position() > setpoint:
						robot.climber.wind(speed)
						yield
				else:
					while robot.climber.get_lift_position() < setpoint:
						robot.climber.unwind(speed)
						yield

				robot.climber.wind(0)
				robot.climber.set_lift_brake_off()

			def wait(seconds):
				timer = wpilib.Timer()
				timer.start()
				while not timer.hasElapsed(seconds):
					yield

			def dither():
				robot.climber.set_lift_brake_off()

				for _ in range(5):
					robot.climber.unwind(0.1)
					yield from wait(0.1)
					robot.climber.wind(0.1)
					yield from wait(0.1)

				robot.climber.wind(0)

			###### START DOING STUFF HERE ######

			robot.turret.is_automatic = False
			robot.hood.is_automatic = False

			# lift_to -> percentage

			lift_stall_amount = 0.1

			robot.climber.set_lift_brake_off()

			# Dither
			yield from dither()

			# Get fully onto bar 2
			yield from lift_to(0.1)
			robot.climber.wind(lift_stall_amount)


			# Get onto bar 3

			yield from lift_to(0.5)
			yield from lift_to(1.0)
			yield from lift_to(0.9)
			yield from lift_to(0.1)
			robot.climber.wind(lift_stall_amount)

			# Get onto bar 4

			yield from lift_to(1.0)
			yield from lift_to(0.1)
			robot.climber.wind(lift_stall_amount)
			robot.climber.set_lift_brake_on()



	def log( self ):
		pass
