import ctre

import wpilib
# import wpilib.controller <-- doesn't exist
import const
import math


class CANCoder_Absolute_Configuration( ):
	def __init__(self, id, offset):
		self.id = id
		self.offset = offset

	def getId(self):
		return self.id

	def getOffset(self):
		return self.offset

