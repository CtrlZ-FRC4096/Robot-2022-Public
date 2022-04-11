import ctre

import wpilib
# import wpilib.controller <-- doesn't exist
import const
import math


def check_ctre_error(error_code, message):
	if error_code != ctre.ErrorCode.OK:
		raise RuntimeError('{0}, {1}'.format(message, error_code))

