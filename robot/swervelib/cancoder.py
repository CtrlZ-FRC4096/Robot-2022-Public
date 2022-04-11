import ctre

import wpilib
# import wpilib.controller <-- doesn't exist
import const
import math

import swervelib.cancoder_configuration
import swervelib.ctre_utils


class CANCoder(ctre.CANCoder):
	def __init__(self, device_number, canbus=""):
		ctre.CANCoder.__init__(self, deviceNumber = device_number, canbus=canbus)

		self.direction = const.COUNTER_CLOCKWISE
		self.periodMilliseconds = 10
		self.magnet_offset_radians = 0.0

		self.configuration = swervelib.cancoder_configuration.CANCoder_Absolute_Configuration(device_number, self.magnet_offset_radians)

		self.config = ctre.CANCoderConfiguration( )

		self.config.absoluteSensorRange = ctre.AbsoluteSensorRange.Unsigned_0_to_360
		self.config.magnetOffsetDegrees = math.degrees(self.configuration.getOffset())
		self.config.sensorDirection = self.direction == const.CLOCKWISE

		# Check for configuration errors
		result = self.configAllSettings(self.config, 250)

		if not const.IS_SIMULATION:
			swervelib.ctre_utils.check_ctre_error(result, 'Failed to configure CANCoder')

		result = self.setStatusFramePeriod(ctre.CANCoderStatusFrame.SensorData, self.periodMilliseconds, 250)

		if not const.IS_SIMULATION:
			swervelib.ctre_utils.check_ctre_error(result, 'Failed to configure CANCoder update rate')


	# Overrides
	def getAbsoluteAngle(self):
		angle = math.radians(self.getAbsolutePosition())
		angle %= 2.0 * math.pi

		if angle < 0:
			angle += 2.0 * math.pi

		return angle

	def getAngleRadians(self):
		angle = math.radians(self.getAbsolutePosition())
		return angle
