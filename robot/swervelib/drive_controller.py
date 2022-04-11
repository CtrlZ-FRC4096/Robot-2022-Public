import ctre
from ctre._ctre import NeutralMode, TalonFXControlMode

import wpilib
# import wpilib.controller <-- doesn't exist
import const
import math


#class Falcon500:
	#def __init__(self, sb_container, gear_ratio, drive_motor_port, steer_motor_port, steer_encoder_port):
		#self.sb_container = sb_container
		#self.gear_ratio = gear_ratio
		#self.drive_motor_port = drive_motor_port
		#self.steer_motor_port = steer_motor_port
		#self.steer_encoder_port = steer_encoder_port

# TODO: Keep going here. Was working out how to best port this:
# https://github.com/SwerveDriveSpecialties/swerve-lib/blob/feb950b8ba9cd3fc3868390f18d0134f82742a19/src/main/java/com/swervedrivespecialties/swervelib/ctre/Falcon500SteerControllerFactoryBuilder.java#L218

class Drive_Controller(ctre.WPI_TalonFX):
	def __init__(self, device_id, canbus=""):
		# ctre.TalonFX.__init__(self, device_id)
		super().__init__(device_id, canbus=canbus)
		self.configFactoryDefault()
		self.setNeutralMode(NeutralMode.Brake)
		self.configOpenloopRamp(.035)
		self.CAN_TIMEOUT_MS = 250
		self.STATUS_FRAME_GENERAL_PERIOD_MS = 250
		self.TICKS_PER_ROTATION = 2048.0

		self.NOMINAL_VOLTAGE = 12
		self.CURRENT_LIMIT = 80

		self.configSupplyCurrentLimit(ctre.SupplyCurrentLimitConfiguration(True, 25, 0, 0))


	# Overrides
	def setReferenceVoltage(self, percent):
		self.set(TalonFXControlMode.PercentOutput , percent)

	def getStateVelocity(self):
		return self.getSelectedSensorVelocity()


