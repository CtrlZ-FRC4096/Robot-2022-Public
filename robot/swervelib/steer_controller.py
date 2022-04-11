import ctre
from ctre._ctre import NeutralMode

import wpilib
# import wpilib.controller <-- doesn't exist
import const
import math


class Steer_Controller(ctre.WPI_TalonFX):
	def __init__(self, device_id, canbus=""):
		super().__init__(device_id, canbus=canbus)

		self.configFactoryDefault()
		self.setNeutralMode(NeutralMode.Brake)
		self.configOpenloopRamp(.035)

		self.CAN_TIMEOUT_MS = 250
		self.STATUS_FRAME_GENERAL_PERIOD_MS = 250
		self.TICKS_PER_ROTATION = 2048.0

		self.VELOCITY_CONSTANT = 0
		self.ACCELERATION_CONSTANT = 0
		self.STATIC_CONSTANT = 0

		self.NOMINAL_VOLTAGE = 12
		self.CURRENT_LIMIT = 20

		self.reference_angle_radians = 0
		self.motor_encoder_position_coefficient = 1.0	# pass this in instead?

		self.configSupplyCurrentLimit(ctre.SupplyCurrentLimitConfiguration(True, 20, 0, 0))

	# Overrides
	def getReferenceAngle(self):
		return self.reference_angle_radians

	def setReferenceAngle(self, reference_angle_radians, difference):
		#current_angle_radians = self.getSelectedSensorPosition() * self.motor_encoder_position_coefficient
		#current_angle_radians_mod = current_angle_radians % (2.0 * math.pi)
		current_angle_radians = reference_angle_radians + difference
		current_angle_radians_mod = current_angle_radians % (2.0 * math.pi)
		if current_angle_radians_mod < 0:
			current_angle_radians_mod += 2.0 * math.pi

		# The reference angle has the range [0, 2pi) but the Falcon's encoder can go above that
		adjusted_reference_angle_radians = reference_angle_radians + current_angle_radians - current_angle_radians_mod

		if reference_angle_radians - current_angle_radians_mod > math.pi:
			adjusted_reference_angle_radians -= 2.0 * math.pi
		elif reference_angle_radians - current_angle_radians_mod < -math.pi:
			adjusted_reference_angle_radians += 2.0 * math.pi

		val = self.rotate_pid.calculate(current_angle_radians, adjusted_reference_angle_radians)
		#print( 'angle = {0:.02f}'.format( val ) )

		self.set(ctre.TalonFXControlMode.PercentOutput, val/100)

		self.reference_angle_radians = reference_angle_radians

	def getStateAngle(self):
		motor_angle_radians = self.getSelectedSensorPosition() * self.motor_encoder_position_coefficient
		motor_angle_radians %= 2.0 * math.pi

		if motor_angle_radians < 0:
			motor_angle_radians += 2.0 * math.pi

		return motor_angle_radians

