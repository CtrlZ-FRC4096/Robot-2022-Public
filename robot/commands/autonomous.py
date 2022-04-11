# This is to help vscode
from typing import TYPE_CHECKING

from wpilib._wpilib import SmartDashboard, wait
from wpimath.controller import PIDController, ProfiledPIDController
from wpimath.kinematics._kinematics import SwerveModuleState
from wpimath.trajectory import TrapezoidProfile
from wpimath.geometry._geometry import Pose2d, Rotation2d
from commands.drivetrain import Drive_Swerve
import commands.autonomous
#from pathplannerlib.swerve_path_controller import SwervePathController
#from commands.drive_follow_path import DriveFollowPath

from pathplannerlib import PathPlanner
if TYPE_CHECKING:
	from robot import Robot
from commands2 import CommandBase, ParallelCommandGroup, ParallelRaceGroup, SequentialCommandGroup
from commands2._impl import CommandBase
from wpimath import geometry, controller
from wpimath.kinematics import ChassisSpeeds
from wpilib import Timer
from wpimath import controller
import const
import math

from coroutine import commandify, CoroutineCommand

import commands.drivetrain

# @commandify
# def Auto1(robot: Robot):
# 	pass

# 	if robot.nt_robot.get
# 	DriveTrajectory()



class DriveTrajectory(CommandBase):
	def __init__(self, robot: "Robot", path, max_vel, max_accel, reversed):
		super().__init__()
		self.robot 	= robot
		self.path 	= path
		self.target = PathPlanner.loadPath(self.path, max_vel, max_accel, reversed)

		self.timer = Timer()

		# Removed the code to align all wheels as it seems to create extra jittering before driving
		#self.robot.drivetrain.swerve_module_front_left.set(0, math.radians(0))
		#self.robot.drivetrain.swerve_module_front_right.set(0, math.radians(0))
		#self.robot.drivetrain.swerve_module_back_left.set(0, math.radians(0))
		#self.robot.drivetrain.swerve_module_back_right.set(0, math.radians(0))

		#temp_angle = (self.target.getSample(0).pose.rotation().degrees() + 360) % 360
		#rotation = Rotation2d(math.radians(temp_angle * 64))
		#self.robot.drivetrain.gyro.setFusedHeading(rotation.degrees(), 0)
		rotation = self.target.sample(0).holonomicRotation
		self.robot.drivetrain.set_gyro(rotation.degrees())
		self.robot.drivetrain.odometry.resetPosition(self.target.sample(0).pose, rotation)

		self.hcontroller = betterHolonomicDriveController()
		self.timer.start()

	# Overrides
	def getRequirements(self):
		return set( [self.robot.drivetrain] )

	def execute(self):
		targetState = self.target.sample(self.timer.get())
		currentPose = self.robot.drivetrain.getPose2d()
		self.hcontroller.drive(self.robot, currentPose, targetState)
		#print('Current pose:')
		#print(currentPose)
		###  speeds = self.hcontroller.calculate(currentPose, targetState, targetState.pose.rotation())
		#speeds = self.hcontroller.calculate(currentPose, targetPose, targetVel, targetState.pose.rotation())
		###  theta = self.testThetaController.calculate(currentPose.rotation().radians(), targetState.pose.rotation().radians())
		#theta = self.testThetaController.calculate(0, 0)
		###  print('current angle, target angle, difference:')
		###  print(currentPose.rotation().radians())
		###  print(targetState.pose.rotation().radians())
		###  print(targetState.pose.rotation().radians() - currentPose.rotation().radians())
		###  print('Controller calc:')
		###  print(speeds)
		###  print('testThetaController values:')
		###  print(theta)
		###  print('Position error:')
		###  print(self.testThetaController.getPositionError())
		###  print('Current pose:')
		###  print(currentPose)
		###  print('Target state:')
		###  print(targetState)
		#print('Target pose:')
		#print(targetPose)
		#print('Target vel:')
		#print(targetVel)
		###  print('Angle ref:')
		###  print(targetState.pose.rotation())
		"""calcs = self.hcontroller.calculate(currentPose, targetState)
		vx = calcs[0]
		vy = calcs[1]
		omega = calcs[2]
		Drive_Swerve(self.robot, vy, vx, 0)"""
		#self.robot.drivetrain.set_all_states(self.robot.drivetrain.get_kinematics().toSwerveModuleStates(speeds))


	def isFinished(self):
		finished = self.timer.hasElapsed(self.target.getTotalTime())
		#print('finished =', finished)
		return finished

	def end(self, interrupted):
		self.timer.stop()
		self.robot.drivetrain.defense()
		pass

	def interrupted(self):
		self.end(True)

class betterHolonomicDriveController():
	def __init__(self):
		'''
		Creates drive controller
		'''
		self.x_controller = controller.PIDController(const.X_KP, const.X_KI, const.X_KD)
		self.y_controller = controller.PIDController(const.Y_KP, const.Y_KI, const.Y_KD)
		self.theta_controller = controller.PIDController(const.THETA_KP, const.THETA_KI, const.THETA_KD)
		self.x_controller.reset()
		self.y_controller.reset()
		self.theta_controller.reset()

	def calculate(self, currentPose, targetState):
		'''
		Calculates chassis speeds
		@parameter (Pose2d) currentPose		Robot's current pose
		@parameter (State) targetState		Robot's target state
		@returns (tuple) (vx, vy, omega)	Robot's left-right, forward-backward, and angular velocities (m/s, m/s, rad/s)
		'''
		# calculates the percent outputs in the x, y, and theta directions
		vx = self.x_controller.calculate(currentPose.X(), targetState.pose.X())
		vy = self.y_controller.calculate(currentPose.Y(), targetState.pose.Y())
		# current and target angles must be put into the range [-math.pi, math.pi)
		targetHeading = self.optimize(currentPose, targetState)
		omega = self.theta_controller.calculate(currentPose.rotation().radians(), targetHeading)
		#print('currentPose, targetPose, (vx, vy, omega):')
		#print(currentPose)
		#print(targetState.pose)
		#print('currentPose, targetPose, current heading, target heading, (vx, vy, omega):')
		#print(currentPose, targetState.pose, currentPose.rotation().radians(), targetHeading)
		#print((vx, vy, omega))
		return (vx, vy, omega)

	def optimize(self, currentPose, targetState):
		'''
		.enableContinuousInput() doesn't work so we have this
		@parameter (Pose2d) currentPose		Robot's current pose
		@parameter (State) targetState		Robot's target state
		@returns (double) targetHeading		optimized heading
		'''
		currentHeading = currentPose.rotation().radians()
		# targetHeading = targetState.pose.rotation().radians()
		targetHeading = targetState.holonomicRotation.radians()
		if targetHeading - currentHeading > math.pi:
			targetHeading -= (2 * math.pi)
			#print('optimized')
		elif targetHeading - currentHeading < -math.pi:
			targetHeading += (2 * math.pi)
			#print('optimized')
		return targetHeading

	def drive(self, robot, currentPose, targetState):
		'''
		Calculates chassis speeds and drives the robot
		'''
		calcs = self.calculate(currentPose, targetState)
		left_right = calcs[0]
		forward_back = calcs[1]
		omega = calcs[2]
		chassis_speeds = ChassisSpeeds.fromFieldRelativeSpeeds(
		    vx = left_right,
		    vy = forward_back,
		    omega = omega,
		    robotAngle = currentPose.rotation(),
		)
		#print('ChassisSpeeds')
		#print(chassis_speeds)
		robot.drivetrain.drive(chassis_speeds)
		robot.drivetrain.periodic()
		#SmartDashboard.putData('X PID', self.x_controller)
		#SmartDashboard.putData('Y PID', self.y_controller)
		#moduleStates = robot.drivetrain.get_all_states()
		#robot.drivetrain.odometry.update(robot.drivetrain.get_rotation(), moduleStates[0], moduleStates[1], moduleStates[2], moduleStates[3])




# # IN PROGRESS AUTO
# class Lone_Two_Ball(ParallelCommandGroup):
# 	'''
# 	Runs lone two ball auto and grabs alliance color from FMS
# 	Just kidding, it will grab alliance color from FMS later, now its hardcoded LOL!
# 	'''
# 	def __init__(self, robot: "Robot"):
# 		super().__init__()

# 		self.robot = robot

# 		self.addCommands(
# 			SequentialCommandGroup(
# 				commands.intake.Down(self.robot),
# 				ParallelRaceGroup(
# 					commands.shooter.Auto_Spin_Up(self.robot),
# 					DriveTrajectory(self.robot, "BlueHPtoLoneShot.path", 2.5, 4.0, False),
# 					commands.intake.In(self.robot),
# 				)

# 			)
# 		)

@commandify
def Lone_Two_Ball(robot: "Robot"):
	robot.intake.down()
	robot.intake.intake()

	color = 'Blue'
	if robot.alliance_color == "red":
		color = 'Red'

	path = color + 'LoneBall.path'

	drive_command = DriveTrajectory(robot, path, 2.5, 4.0, False)
	drive_command.initialize()

	while not drive_command.isFinished():
		drive_command.execute()
		robot.shooter.auto_spin()
		yield

	while True:
		robot.shooter.auto_spin()
		robot.indexer.should_feed = True
		yield

@commandify
def Basic_Auto(robot: "Robot"):
	# robot.intake.down()
	# robot.intake.intake()

	robot.turret.is_automatic = False

	timer = Timer()
	timer.start()

	robot.turret.set_target(360, False, False)

	while not timer.hasElapsed(5):
		robot.shooter.auto_spin()
		yield

	robot.indexer.should_feed = True

	timer = Timer()
	timer.start()

	while not timer.hasElapsed(3):
		yield

	robot.shooter.stop()
	robot.indexer.should_feed = False
	robot.intake.down()
	robot.intake.intake()
	robot.drivetrain.chassis_speeds = ChassisSpeeds(.1, 0, 0.0)

	timer = Timer()
	timer.start()
	while not timer.hasElapsed(5):
		yield

	robot.drivetrain.chassis_speeds = ChassisSpeeds(0.0, 0.0, 0.0)


@commandify
def Basic_Two_Ball_Auto(robot: "Robot"):
	robot.drivetrain.gyro.setFusedHeading(270 * 64, 0)
	robot.intake.down()
	robot.turret.is_automatic = False
	robot.climber.set_tilt_forward()
	#robot.turret.set_target(360, False, False) #This is optional if we decide to rotate the turret's starting location

	timer = Timer()
	timer.start()

	while not timer.hasElapsed(2.0):
		robot.intake.intake()
		robot.drivetrain.chassis_speeds = ChassisSpeeds(0.1, 0.0, 0.0)
		yield
	robot.drivetrain.chassis_speeds = ChassisSpeeds(0.0, 0.0, 0.0)

	timer = Timer()
	timer.start()

	while not timer.hasElapsed(1.5):
		robot.turret.is_automatic = True
		# robot.hood.set_target(32)
		# robot.shooter.set_velocity(5000, 10000)
		robot.shooter.auto_spin()
		#robot.drivetrain.chassis_speeds = ChassisSpeeds(0.1, 0.0, 0.0)
		yield

	robot.intake.up()

	while not timer.hasElapsed(2):
		robot.drivetrain.chassis_speeds = ChassisSpeeds(0.0, 0.0, 0.0)
		yield


	timer = Timer()
	timer.start()

	while not timer.hasElapsed(5):
		robot.intake.stop()
		robot.indexer.should_feed = True
		yield

	robot.indexer.should_feed = False

@commandify
def Basic_Hangar_Two_Ball_Auto(robot: "Robot"):
	robot.drivetrain.gyro.setFusedHeading(137 * 64, 0)
	robot.intake.down()
	robot.turret.is_automatic = False
	robot.climber.set_tilt_forward()
	#robot.turret.set_target(360, False, False) #This is optional if we decide to rotate the turret's starting location

	timer = Timer()
	timer.start()

	while not timer.hasElapsed(1.75):
		robot.intake.intake()
		robot.drivetrain.chassis_speeds = ChassisSpeeds(0.1, 0.07, 0)
		yield
	robot.drivetrain.chassis_speeds = ChassisSpeeds(0.0, 0.0, 0.0)

	timer = Timer()
	timer.start()

	while not timer.hasElapsed(1):
		robot.turret.is_automatic = True
		# robot.hood.set_target(32)
		# robot.shooter.set_velocity(5000, 10000)
		robot.shooter.auto_spin()
		#robot.drivetrain.chassis_speeds = ChassisSpeeds(0.1, 0.0, 0.0)
		yield

	robot.intake.up()

	while not timer.hasElapsed(2):
		robot.drivetrain.chassis_speeds = ChassisSpeeds(0.0, 0.0, 0.0)
		yield


	timer = Timer()
	timer.start()

	while not timer.hasElapsed(5):
		robot.intake.stop()
		robot.indexer.should_feed = True
		yield

	robot.indexer.should_feed = False


@commandify
def Hangar_Two_Ball_Auto(robot: "Robot"):
	robot.turret.is_automatic = False
	robot.hood.set_target(15)
	robot.turret.set_target(285+90, False, False) #This is optional if we decide to rotate the turret's starting location


	timer = Timer()
	timer.start()

	cmd = DriveTrajectory(robot, './paths/2Ball', 3, 4, False)
	cmd.initialize()
	while not timer.hasElapsed(2):
		robot.intake.down()
		robot.intake.intake()
		robot.turret.is_automatic = True
		robot.hood.is_automatic = True
		robot.shooter.auto_spin()
		cmd.execute()
		yield

	robot.intake.stop()
	robot.intake.up()

	timer = Timer()
	timer.start()

	robot.drivetrain.chassis_speeds = ChassisSpeeds(0.0, 0.0, 0.0)

	while not timer.hasElapsed(10):
		robot.indexer.should_feed = True
		yield

	robot.indexer.should_feed = False

@commandify
def Five_Ball_Auto(robot: "Robot"):
	robot.turret.is_automatic = False
	robot.hood.set_target(15)
	robot.turret.set_target(360, False, False) #This is optional if we decide to rotate the turret's starting location

	timer = Timer()
	timer.start()

	drive = DriveTrajectory(robot, './paths/5Ball1', 3, 4, False)
	drive.initialize()
	while not drive.isFinished():
		drive.execute()
		robot.intake.down()
		robot.intake.intake()
		robot.turret.is_automatic = True
		robot.hood.is_automatic = True
		robot.shooter.auto_spin()
		yield

	robot.drivetrain.chassis_speeds = ChassisSpeeds(0.0, 0.0, 0.0)

	timer = Timer()
	timer.start()

	while not timer.hasElapsed(.5):
		robot.indexer.should_feed = True
		yield

	robot.indexer.should_feed = False

	timer = Timer()
	timer.start()

	drive = DriveTrajectory(robot, './paths/5Ball2', 3, 4, False)
	drive.initialize()
	while not drive.isFinished():
		drive.execute()
		yield

	robot.drivetrain.chassis_speeds = ChassisSpeeds(0.0, 0.0, 0.0)

	timer = Timer()
	timer.start()

	while not timer.hasElapsed(2):
		robot.indexer.should_feed = True
		yield

	robot.indexer.should_feed = False

	timer = Timer()
	timer.start()

	drive = DriveTrajectory(robot, './paths/5Ball3', 3, 4, False)
	drive.initialize()
	while not drive.isFinished():
		drive.execute()
		yield

	robot.drivetrain.chassis_speeds = ChassisSpeeds(0.0, 0.0, 0.0)

	timer = Timer()
	timer.start()

	while not timer.hasElapsed(2):
		yield

	timer = Timer()
	timer.start()

	drive = DriveTrajectory(robot, './paths/5Ball4', 3, 4, False)
	drive.initialize()
	while not drive.isFinished():
		drive.execute()
		robot.intake.stop()
		robot.intake.up()
		yield

	robot.drivetrain.chassis_speeds = ChassisSpeeds(0.0, 0.0, 0.0)

	timer = Timer()
	timer.start()

	while not timer.hasElapsed(2.55):
		robot.indexer.should_feed = True
		yield


@commandify
def Taxi_And_Shoot(robot: "Robot"):
	robot.intake.down()
	robot.turret.is_automatic = False
	robot.hood.set_target(15)
	robot.climber.set_tilt_forward()
	timer = Timer()
	timer.start()

	while not timer.hasElapsed(3):
		robot.intake.intake()
		robot.drivetrain.chassis_speeds = ChassisSpeeds(-0.1, 0, 0)
		yield

	robot.drivetrain.chassis_speeds = ChassisSpeeds(0, 0, 0)

	robot.turret.set_target(180, False, False)

	timer = Timer()
	timer.start()

	while not timer.hasElapsed(2):
		robot.turret.is_automatic = True
		robot.shooter.auto_spin()
		robot.hood.is_automatic = True
		yield

	robot.intake.up()

	# while not timer.hasElapsed(3.5):
	# 	yield

	timer = Timer()
	timer.start()

	while not timer.hasElapsed(5):
		robot.intake.stop()
		robot.indexer.should_feed = True
		yield

	robot.indexer.should_feed = False
