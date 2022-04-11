import ctre
from ctre._ctre import FeedbackDevice, RemoteSensorSource, TalonFX, TalonFXControlMode, TalonFXFeedbackDevice

import wpilib
from wpilib import drive
from wpilib._wpilib import SmartDashboard
import wpimath
import wpimath.controller
from wpimath.geometry._geometry import Rotation2d
import const
import math

import swervelib.drive_controller
from swervelib.mk4_module_configuration import MK4_Module_Configuration
import swervelib.steer_controller
import swervelib.cancoder

from robot import Robot

class Swerve_Module( ):
	"""
	Individual swerve module
	"""
	def __init__(self, robot: "Robot", module_name, drive_motor_id, steer_motor_id, steer_encoder_id, steer_offset, MK4_L2):
		self.robot = robot

		self.module_name = module_name

		self.drive_controller	= swervelib.drive_controller.Drive_Controller(drive_motor_id, canbus="tooth")
		self.steer_controller	= swervelib.steer_controller.Steer_Controller(steer_motor_id, canbus="tooth")
		self.steer_encoder		= swervelib.cancoder.CANCoder(steer_encoder_id, canbus="tooth")

		self.MK4_L2 = MK4_L2

		# Configure integrated encoder
		self.steer_encoder.configMagnetOffset(steer_offset)

		# Setting up PID objects
		self.steer_pid_controller = wpimath.controller.PIDController(const.STEER_KP, const.STEER_KI, const.STEER_KD)	# could try .35 or .375 for kP, DON'T CHANGE kD
		SmartDashboard.putData('Steer PID', self.steer_pid_controller)
		self.steer_pid_controller.setTolerance(0)
		self.steer_pid_controller.enableContinuousInput(-math.pi, math.pi)

	# Override
	def getDriveVelocity(self):
		return self.drive_controller.getStateVelocity()

	# Override
	def getSteerAngle(self):
		return self.steer_encoder.getAbsolutePosition()

	def getDriveVelocityMetersPerSecond(self):
		return self.drive_controller.getSelectedSensorVelocity() * const.METERS_PER_DRIVE_TICK

	def optimize(self, drive_voltage, steer_angle, current_angle):
		delta = steer_angle - current_angle

		if abs(delta) > math.pi / 2.0 and abs(delta) < 3.0 / 2.0 * math.pi:
			if steer_angle >= math.pi:
				return (-drive_voltage, steer_angle - math.pi)
			else:
				return (-drive_voltage, steer_angle + math.pi)
		else:
			return (drive_voltage, steer_angle)

	def set(self, drive_speed, steer_angle):
		#print( '1 drive = {0:.02f}, steer = {1:.02f}'.format(drive_voltage, steer_angle))
		new_state = self.optimize(drive_speed, steer_angle, self.steer_encoder.getAngleRadians())
		drive_speed = new_state[0]
		steer_angle = new_state[1]

		#print( '2 drive = {0:.02f}, steer = {1:.02f}'.format(drive_voltage, steer_angle))

		steer_output = self.steer_pid_controller.calculate(self.steer_encoder.getAngleRadians(), steer_angle)

		#print( 'drive_speed = {0:.02f}'.format( drive_speed ))
		#print( 'drive_output = {0:.02f}'.format( drive_output ))

		self.drive_controller.set(TalonFXControlMode.PercentOutput, drive_speed / const.MAX_VEL_METERS * 12)
		self.steer_controller.set(TalonFXControlMode.PercentOutput, -steer_output)


	def log(self):
		'''
		logs various things about this swerve module
		'''
		pass
		self.robot.nt_robot.putNumber('drive/module_{0}/drive_velocity'.format( self.module_name ), self.getDriveVelocity())
		self.robot.nt_robot.putNumber('drive/module_{0}/steer_angle'.format( self.module_name ), self.getSteerAngle())
