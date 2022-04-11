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

from typing import Tuple
import math

from commands2 import Subsystem

import ctre
from math import degrees, pi, sin, cos, radians
from wpilib import Timer

from wpimath.kinematics import SwerveDrive4Kinematics, SwerveDrive4Odometry, SwerveModuleState, ChassisSpeeds
from wpimath.geometry import Translation2d, Rotation2d, Pose2d
from wpimath.controller import PIDController

import const
from swervelib.swerve_module import Swerve_Module
from swervelib.mk4_module_configuration import MK4_Module_Configuration



### CONSTANTS, ENUMS ###

MK4_L2 = MK4_Module_Configuration(
    0.10033,										# wheelDiameter
    (14.0 / 50.0) * (27.0 / 17.0) * (15.0 / 45.0),	# driveReduction
    True,											# driveInverted
    (15.0 / 32.0) * (10.0 / 60.0),					# steerReduction
    True,											# steerInverted
)


### CLASSES ###

class Drivetrain(Subsystem):
	def __init__(self, robot: "Robot"):
		super().__init__()

		self.robot = robot

		self.MAX_VOLTAGE = 12.0

		# Measure the drivetrain's maximum velocity or calculate the theoretical.
		# The formula for calculating the theoretical maximum velocity is:
		# <Motor free speed RPM> / 60 * <Drive reduction> * <Wheel diameter meters> * pi
		# The maximum velocity of the robot in meters per second.
		# This is a measure of how fast the robot should be able to drive in a straight line.
		self.MAX_VELOCITY_METERS_PER_SECOND = 6380.0 / 60.0 * MK4_L2.getDriveReduction() * MK4_L2.getWheelDiameter() * math.pi

		# Here we calculate the theoretical maximum angular velocity. You can also replace this with a measured amount.
		self.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND = self.MAX_VELOCITY_METERS_PER_SECOND / math.hypot(const.DRIVETRAIN_TRACKWIDTH_METERS / 2.0, const.DRIVETRAIN_WHEELBASE_METERS / 2.0)

		self.gyro = ctre.PigeonIMU(const.CAN_GYRO_ID)

		self.gyro_offset = 0

		self.gyro.setFusedHeading(const.GYRO_RESET_VALUE)

		self.kinematics = SwerveDrive4Kinematics(
            # Front Left
            Translation2d(const.DRIVETRAIN_TRACKWIDTH_METERS / 2.0, const.DRIVETRAIN_WHEELBASE_METERS / 2.0),
            # Front right
            Translation2d(const.DRIVETRAIN_TRACKWIDTH_METERS / 2.0, -const.DRIVETRAIN_WHEELBASE_METERS / 2.0),
            # Back left
            Translation2d(-const.DRIVETRAIN_TRACKWIDTH_METERS / 2.0, const.DRIVETRAIN_WHEELBASE_METERS / 2.0),
            # Back right
            Translation2d(-const.DRIVETRAIN_TRACKWIDTH_METERS / 2.0, -const.DRIVETRAIN_WHEELBASE_METERS / 2.0)
        )

		self.odometry = SwerveDrive4Odometry( self.kinematics, Rotation2d.fromDegrees(self.gyro.getFusedHeading()) )
		#self.odometry = customOdometry(self.robot, Pose2d(Translation2d(0, 0), Rotation2d(radians(self.gyro.getFusedHeading()))))
		#self.odometry.reset(Pose2d(Translation2d(0, 0), Rotation2d(pi / 2)))

		self.target_heading = radians(self.gyro.getFusedHeading())

		# PID controller for field oriented steering
		self.theta_pid = PIDController(
			const.THETA_KP,
			const.THETA_KI,
			const.THETA_KD,
		)

		self.theta_pid.setTolerance(math.radians(2))

		# Create our swerve module objects
		self.swerve_module_front_left = Swerve_Module(
            robot,
            module_name = const.FRONT_LEFT,
            drive_motor_id = const.DRIVE_MOTOR_1_ID,
            steer_motor_id = const.STEER_MOTOR_1_ID,
            steer_encoder_id = const.STEER_ENCODER_1_ID,
            steer_offset = const.STEER_ENCODER_1_OFFSET,
            MK4_L2 = MK4_L2,
        )

		self.swerve_module_front_right = Swerve_Module(
            robot,
            module_name = const.FRONT_RIGHT,
            drive_motor_id = const.DRIVE_MOTOR_2_ID,
            steer_motor_id = const.STEER_MOTOR_2_ID,
            steer_encoder_id = const.STEER_ENCODER_2_ID,
            steer_offset = const.STEER_ENCODER_2_OFFSET,
            MK4_L2 = MK4_L2,
        )

		self.swerve_module_back_left = Swerve_Module(
            robot,
            module_name = const.BACK_LEFT,
            drive_motor_id = const.DRIVE_MOTOR_3_ID,
            steer_motor_id = const.STEER_MOTOR_3_ID,
            steer_encoder_id = const.STEER_ENCODER_3_ID,
            steer_offset = const.STEER_ENCODER_3_OFFSET,
            MK4_L2 = MK4_L2,
        )

		self.swerve_module_back_right = Swerve_Module(
            robot,
            module_name = const.BACK_RIGHT,
            drive_motor_id = const.DRIVE_MOTOR_4_ID,
            steer_motor_id = const.STEER_MOTOR_4_ID,
            steer_encoder_id = const.STEER_ENCODER_4_ID,
            steer_offset = const.STEER_ENCODER_4_OFFSET,
            MK4_L2 = MK4_L2,
        )

		# Dict of swerve modules
		self.modules = {
            const.FRONT_LEFT:	self.swerve_module_front_left,
            const.FRONT_RIGHT:	self.swerve_module_front_right,
            const.BACK_LEFT:	self.swerve_module_back_left,
            const.BACK_RIGHT:	self.swerve_module_back_right
        }

		self.chassis_speeds = ChassisSpeeds(0.0, 0.0, 0.0)

	def getPose2d(self):
		return self.odometry.getPose()

	def get_gyro_heading(self):
		'''
		Returns Rotation2d of gyro heading on range [0, 2pi) radians
		'''
		return Rotation2d.fromDegrees((self.gyro.getFusedHeading() - self.gyro_offset) % 360)

	def get_gyro_heading_degrees(self):
		'''
		Returns gyro heading in degrees on range [0, 360) degrees
		'''
		return self.get_gyro_heading().degrees()

	def set_gyro(self, new_heading = 0):
		self.gyro_offset = self.gyro.getFusedHeading() - new_heading

	#def zero_gyro(self):
		#self.odometry.resetPosition(
			#Pose2d(self.odometry.getPose().getTranslation(), Rotation2d.fromDegrees(0.0)),
			#Rotation2d.fromDegrees(self.gyro.getFusedHeading())
		#)

	#def get_kinematics(self):
		#return self.kinematics

	def get_all_states(self):
		fl = SwerveModuleState(self.swerve_module_front_left.getDriveVelocityMetersPerSecond(), Rotation2d(math.radians(self.swerve_module_front_left.getSteerAngle())))
		fr = SwerveModuleState(self.swerve_module_front_right.getDriveVelocityMetersPerSecond(), Rotation2d(math.radians(self.swerve_module_front_right.getSteerAngle())))
		bl = SwerveModuleState(self.swerve_module_back_left.getDriveVelocityMetersPerSecond(), Rotation2d(math.radians(self.swerve_module_back_left.getSteerAngle())))
		br = SwerveModuleState(self.swerve_module_back_right.getDriveVelocityMetersPerSecond(), Rotation2d(math.radians(self.swerve_module_back_right.getSteerAngle())))
		return (fl, fr, bl, br)

	def set_all_states(self, states):
		self.swerve_module_front_left.set(states[0].speedMetersPerSecond, states[0].angle.getRadians())
		self.swerve_module_front_right.set(states[1].speedMetersPerSecond, states[1].angle.getRadians())
		self.swerve_module_back_left.set(states[2].speedMetersPerSecond, states[2].angle.getRadians())
		self.swerve_module_back_right.set(states[3].speedMetersPerSecond, states[3].angle.getRadians())

	def get_rotation(self):
		'''
		THIS RETURNS THE ODOMETRY ROTATION, NOT GYRO ROTATION
		'''
		return self.odometry.getPose().rotation()

	#def set_gyro(self):
		#self.gyro.setFusedHeading(90.0)

	def drive(self, chassis_speeds):
		self.chassis_speeds = chassis_speeds

	def defense(self):
		self.swerve_module_front_left.set(0, math.radians(45))
		self.swerve_module_front_right.set(0, math.radians(135))
		self.swerve_module_back_left.set(0, math.radians(135))
		self.swerve_module_back_right.set(0, math.radians(45))

	# Override
	def periodic(self):
		self.odometry.update(
            self.get_gyro_heading(),
            SwerveModuleState(self.swerve_module_front_left.getDriveVelocityMetersPerSecond(), Rotation2d(radians(self.swerve_module_front_left.getSteerAngle()))),
            SwerveModuleState(self.swerve_module_front_right.getDriveVelocityMetersPerSecond(), Rotation2d(radians(self.swerve_module_front_right.getSteerAngle()))),
            SwerveModuleState(self.swerve_module_back_left.getDriveVelocityMetersPerSecond(), Rotation2d(radians(self.swerve_module_back_left.getSteerAngle()))),
            SwerveModuleState(self.swerve_module_back_right.getDriveVelocityMetersPerSecond(), Rotation2d(radians(self.swerve_module_back_right.getSteerAngle()))),
            #self.get_all_states()[0],
            #self.get_all_states()[1],
            #self.get_all_states()[2],
            #self.get_all_states()[3],
        )
		#self.odometry.update()

		# Needed to fix broken type hints
		states: Tuple[SwerveModuleState, SwerveModuleState, SwerveModuleState, SwerveModuleState]

		# print(self.chassis_speeds)
		states = self.kinematics.toSwerveModuleStates(self.chassis_speeds) # type: ignore
		# print(states)


		# Need to normalize here? Was in one example but not another
		states = SwerveDrive4Kinematics.desaturateWheelSpeeds(states, self.MAX_VELOCITY_METERS_PER_SECOND) # type: ignore
		self.swerve_module_front_left.set(states[0].speed, states[0].angle.radians())
		self.swerve_module_front_right.set(states[1].speed, states[1].angle.radians())
		self.swerve_module_back_left.set(states[2].speed, states[2].angle.radians())
		self.swerve_module_back_right.set(states[3].speed, states[3].angle.radians())
		#print(self.chassis_speeds)
		#print('states, fl, fr, bl, br ticks')
		#print(states)
		#print(self.swerve_module_front_left.getDriveVelocityMetersPerSecond())
		#print(self.swerve_module_front_right.getDriveVelocityMetersPerSecond())
		#print(self.swerve_module_back_left.getDriveVelocityMetersPerSecond())
		#print(self.swerve_module_back_right.getDriveVelocityMetersPerSecond())


	def stop(self):
		self.chassis_speeds = ChassisSpeeds(0.0, 0.0, 0.0)

		# self.swerve_module_front_left.set(0, 0)
		# self.swerve_module_front_right.set(0, 0)
		# self.swerve_module_back_left.set(0, 0)
		# self.swerve_module_back_right.set(0, 0)


	def log(self):
		'''
		logs various things about the drivetrain
		'''
		#for key in self._requested_angles:
			#self.robot.nt_robot.putNumber('drive/{0}_angle'.format( key ), self._requested_angles[key])
			#self.robot.nt_robot.putNumber('drive/{0}_speed'.format( key ), self._requested_speeds[key])

		# Let each module log stuff
		for module in self.modules.values():
			module.log()
		#self.odometry.log()
		self.robot.nt_robot.putNumber('Odometry X (Away)', self.odometry.getPose().X())
		self.robot.nt_robot.putNumber('Odometry Y (Left)', self.odometry.getPose().Y())
		self.robot.nt_robot.putNumber('Fused Heading', self.get_gyro_heading_degrees())



class customOdometry():
	def __init__(self, robot: "Robot", pose):
		'''
		Creates an odometry object
		@parameter (robot) robot		Robot
		@parameter (Pose2d) pose		Robot's starting pose
		'''
		self.robot = robot
		self.x = 0
		self.y = 0
		self.timer = Timer()
		self.lastTime = 0
		self.timeStep = 0
		self.timer.start()

	def reset(self, pose):
		'''
		Resets odometry pose to passed pose and timer to 0
		@parameter (Pose2d) pose		Robot's reset pose
		'''
		self.x = pose.X()
		self.y = pose.Y()
		self.timer.reset()
		self.timer.start()

	def getTime(self):
		'''
		Returns time
		'''
		return self.timer.get()

	def getPose(self):
		'''
		Returns robot pose (Pose2d)
		'''
		return Pose2d(Translation2d(self.x, self.y), self.robot.drivetrain.get_gyro_heading())

	def update(self, fl, fr, bl, br):
		'''
		Updates odometry
		@parameters SwerveModuleState (x4) pass in the swerve module states
		'''
		A_BL = sin(-bl.angle.radians()) * bl.speed
		A_BR = sin(-br.angle.radians()) * br.speed
		B_FL = sin(-fl.angle.radians()) * fl.speed
		B_FR = sin(-fr.angle.radians()) * fr.speed
		C_FR = cos(-fr.angle.radians()) * fr.speed
		C_BR = cos(-br.angle.radians()) * br.speed
		D_FL = cos(-fl.angle.radians()) * fl.speed
		D_BL = cos(-bl.angle.radians()) * bl.speed
		A = (A_BL + A_BR) / 2
		B = (B_FL + B_FR) / 2
		C = (C_FR + C_BR) / 2
		D = (D_FL + D_BL) / 2
		ROT_1   = (B - A) / const.DRIVETRAIN_WHEELBASE_METERS
		ROT_2   = (C - D) / const.DRIVETRAIN_TRACKWIDTH_METERS
		ROT     = (ROT_1 + ROT_2) / 2
		FWD_1   = ROT * (const.DRIVETRAIN_WHEELBASE_METERS / 2) + A
		FWD_2   = -ROT * (const.DRIVETRAIN_WHEELBASE_METERS / 2) + B
		FWD     = (FWD_1 + FWD_2) / 2
		STR_1   = ROT * (const.DRIVETRAIN_TRACKWIDTH_METERS / 2) + C
		STR_2   = -ROT * (const.DRIVETRAIN_TRACKWIDTH_METERS / 2) + D
		STR     = (STR_1 + STR_2) / 2
		theta = self.robot.drivetrain.get_gyro_heading().radians()
		FWD_NEW = FWD * cos(theta) + STR * sin(theta)
		STR_NEW = STR * cos(theta) - FWD * sin(theta)
		curTime = self.timer.get()
		timeStep = curTime - self.lastTime
		self.lastTime = curTime
		self.y -= FWD_NEW * timeStep
		self.x += STR_NEW * timeStep

	def log(self):
		self.robot.nt_robot.putNumber('Odometry X', self.x)
		self.robot.nt_robot.putNumber('Odometry Y', self.y)
