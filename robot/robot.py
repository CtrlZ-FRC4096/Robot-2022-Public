#! python3
"""
Ctrl-Z FRC Team 4096
FIRST Robotics Competition 2022
Code for robot "Chao-Z"
contact@team4096.org

Some code adapted from:
https://github.com/SwerveDriveSpecialties
"""

DEBUG = True

# Import our files
import math
import logging
import time
import sys

import wpilib
import commands2

import networktables

from remote_shell import RemoteShell

import const
import oi

import subsystems.drivetrain
import commands.autonomous

import subsystems.climber
import subsystems.indexer
import subsystems.turret
import subsystems.intake
import subsystems.shooter
import subsystems.hood

import subsystems.limelight

log = logging.getLogger('robot')
networktables.NetworkTables.initialize()



class Robot(commands2.TimedCommandRobot):
	"""
	Main robot class.

	This is the central object, holding instances of all the robot subsystem
	and sensor classes.

	It also contains the init & periodic methods for autonomous and
	teloperated modes, called during mode changes and repeatedly when those
	modes are active.

	The one instance of this class is also passed as an argument to the
	various other classes, so they have full access to all its properties.
	"""
	def robotInit(self):

		self.match_time = -1
		const.IS_SIMULATION = self.isSimulation( )

		# NetworkTables
		self.nt_robot = networktables.NetworkTables.getTable('Robot')

		# Remote Shell
		if DEBUG:
			self.remote_shell = RemoteShell(self)
			print("Started Remote Shell")


		### Subsystems ###

		# Type hints as placeholders. These objects haven't been created yet.
		#self.turret: subsystems.turret.Turret
		#self.shooter: subsystems.shooter.Shooter
		#self.hood: subsystems.hood.Hood
		#self.limelight: subsystems.limelight.Limelight

		# Actual subsystems

		self.drivetrain = subsystems.drivetrain.Drivetrain(self)
		self.climber = subsystems.climber.Climber(self)
		self.intake = subsystems.intake.Intake(self)
		self.indexer = subsystems.indexer.Indexer(self)
		self.turret = subsystems.turret.Turret(self)
		self.shooter = subsystems.shooter.Shooter(self)
		self.limelight = subsystems.limelight.Limelight(self)
		self.hood = subsystems.hood.Hood(self)

		self.subsystems = [
			self.drivetrain,
			self.climber,
			self.intake,
			self.indexer,
			self.turret,
			self.shooter,
			self.limelight,
			self.hood,
		]

		### OTHER ###

		# Driverstation
		self.driverstation : wpilib.DriverStation = wpilib.DriverStation.getInstance()

		# Operator Input
		self.oi = oi.OI(self)

		## Scheduler ##
		self.scheduler = commands2.CommandScheduler.getInstance()


		### LOGGING ###

		# Timers for NetworkTables update so we don't use too much bandwidth
		self.log_timer = wpilib.Timer()
		self.log_timer.start()
		self.log_timer_delay = 0.25		# 4 times/second

		# Disable LW telemetry before comp to improve loop dtimes
		# wpilib.LiveWindow.disableAllTelemetry()

		self.match_time = -1

		self.auto_chooser = wpilib.SendableChooser()
		# self.auto_chooser.addOption("NAME", COMMAND)
		wpilib.SmartDashboard.putData("Auto Chooser", self.auto_chooser)


	def robotPeriodic(self):
		# Stuff for the wpilib simulator
		self.log()
		self.oi.log()

		for subystem in self.subsystems:
			subystem.periodic()


	### DISABLED ###

	def disabledInit(self):
		self.scheduler.cancelAll()

		for subsystem in self.subsystems:
			subsystem.stop()

		self.limelight.turn_leds_off()


	def disabledPeriodic(self):
		pass
		self.limelight.turn_leds_off()

		# error = self.drivetrain.gyro.setFusedHeading(const.GYRO_RESET_VALUE, 0)
		# if error != 0:
		# 	print('Failed to reset gyro. Error code ', error)
		# # self.drivetrain.target_heading = math.radians(self.drivetrain.gyro.getFusedHeading())

		self.alliance_color = {
			wpilib.DriverStation.Alliance.kRed: "red",
			wpilib.DriverStation.Alliance.kBlue: "blue"
		}.get(self.driverstation.getAlliance(), "red")


	### AUTONOMOUS ###

	def autonomousInit(self):
		self.limelight.turn_leds_on()
		#self.selected_auto_mode = commands.autonomous.DriveTrajectory(self, 'Test10Ft.path')
		#self.selected_auto_mode = commands.autonomous.DriveTrajectory(self, 'Test10Ft180.path')
		# self.selected_auto_mode = commands.autonomous.DriveTrajectory(self, 'Slalom.path')
		#self.selected_auto_mode = commands.autonomous.DriveTrajectory(self, 'WiggleReverse.path')
		# self.scheduler.schedule(self.selected_auto_mode)

		# self.selected_auto_mode = self.auto_chooser.getSelected()

		#self.scheduler.schedule(commands.autonomous.Lone_Two_Ball(self)) # type: ignore
		#self.scheduler.schedule(commands.autonomous.DriveTrajectory(self, './paths/5Ball1', 2, 3, False))
		#self.scheduler.schedule(commands.autonomous.Hangar_Two_Ball_Auto(self))  # type: ignore
		#self.scheduler.schedule(commands.autonomous.Five_Ball_Auto(self))  # type: ignore
		#self.scheduler.schedule(commands.autonomous.DriveTrajectory(self, './paths/CalibrationPath', 2, 3, False))
		# self.scheduler.schedule(commands.autonomous.Basic_Auto(self)) # type: ignore
		#self.scheduler.schedule(commands.autonomous.Taxi_And_Shoot(self))  # type: ignore
		self.scheduler.schedule(commands.autonomous.Basic_Two_Ball_Auto(self)) # type: ignore
		#self.scheduler.schedule(commands.autonomous.Basic_Hangar_Two_Ball_Auto(self)) # type: ignore

	def autonomousPeriodic(self):
		self.scheduler.run()


	### TELEOPERATED ###

	def teleopInit(self):
		# Removes any leftover commands from the scheduler
		self.scheduler.cancelAll()
		self.turret.is_automatic = True
		self.hood.is_automatic = True
		self.indexer.should_feed = False
		self.limelight.turn_leds_on()
		self.climber.set_tilt_forward()

	def teleopPeriodic(self):
		self.scheduler.run()

		# self.turret.run(-self.oi.driver2.LEFT_JOY_X())


	### MISC ###

	def log(self):
		"""
		Logs some info to shuffleboard, and standard output
		"""

		if not self.log_timer.advanceIfElapsed(self.log_timer_delay):
			return

		#self.nt_robot.putString('Pressure', '{0:.2f}'.format(self.get_pressure()))

		for s in self.subsystems:
			s.log()

		self.match_time = self.driverstation.getMatchTime()
		self.nt_robot.putNumber('Match Time', self.match_time)


	def tsa(self, bot_velocity, top_velocity, hood_angle):
		self.shooter.set_velocity(bot_velocity,  top_velocity)
		self.hood.set_target(hood_angle)


### MAIN ###

if __name__ == "__main__":
	wpilib.run(Robot)
