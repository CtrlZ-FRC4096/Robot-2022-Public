"""
Ctrl-Z FRC Team 4096
FIRST Robotics Competition 2022
Code for robot "Chao-Z"
contact@team4096.org

Some code adapted from:
https://github.com/SwerveDriveSpecialties
"""

class MK4_Module_Configuration( ):
	"""
	Additional Mk4 module configuration parameters.
	The configuration parameters here are used to customize the behavior of the Mk4 swerve module.
	Each setting is initialized to a default that should be adequate for most use cases.
	"""

	def __init__(self, wheelDiameter, driveReduction, driveInverted, steerReduction, steerInverted):
		self.wheelDiameter = wheelDiameter
		self.driveReduction = driveReduction
		self.driveInverted = driveInverted
		self.steerReduction = steerReduction
		self.steerInverted = steerInverted

		self.nominalVoltage = 12.0
		self.driveCurrentLimit = 80.0
		self.steerCurrentLimit = 20.0


	def getWheelDiameter(self):
		return self.wheelDiameter


	def getDriveReduction(self):
		"""
		* Gets the overall reduction of the drive system.
		* If this value is multiplied by drive motor rotations the result would be drive wheel rotations.
		"""
		return self.driveReduction


	def isDriveInverted(self):
		return self.driveInverted


	def getSteerReduction(self):
		return self.steerReduction


	def isSteerInverted(self):
		return self.steerInverted


	def getNominalVoltage(self):
		return self.nominalVoltage


	def setNominalVoltage(self, nominalVoltage):
		self.nominalVoltage = nominalVoltage


	def getDriveCurrentLimit(self):
		return self.driveCurrentLimit


	def setDriveCurrentLimit(self, driveCurrentLimit):
		self.driveCurrentLimit = driveCurrentLimit


	def getSteerCurrentLimit(self):
		return self.steerCurrentLimit


	def setSteerCurrentLimit(self, steerCurrentLimit):
		self.steerCurrentLimit = steerCurrentLimit

