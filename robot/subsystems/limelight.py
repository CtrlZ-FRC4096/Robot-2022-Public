"""
Ctrl-Z FRC Team 4096
FIRST Robotics Competition 2020
Code for robot "------"
contact@team4096.org
"""

# This is to help vscode
import math
from typing import TYPE_CHECKING
if TYPE_CHECKING:
	from robot import Robot

from math import radians, tan

from commands2 import Subsystem
import networktables

import const

class Limelight(Subsystem):
	def __init__(self, robot: "Robot"):
		super().__init__()

		self.robot = robot

		self.driver_camera_mode = const.LIMELIGHT_DRIVER_CAMERA_MODE_DEFAULT
		self.nt_table = networktables.NetworkTables.getTable("limelight")


	def set_startup_modes(self):
		self.nt_table.putNumber('stream', const.LIMELIGHT_STREAM_MODE)		# 0 = side-by-side, 1 = PIP Main, 2 = PIP Secondary
		self.set_driver_mode(const.LIMELIGHT_DRIVER_CAMERA_MODE_DEFAULT)
		self.turn_leds_off()


	def get_angle_to_target(self):
		return self.nt_table.getNumber( 'tx', 0)


	def is_target_visible(self):
		# See if there's any target in view
		return self.nt_table.getNumber('tv', 0) == 1


	def set_driver_mode(self, mode):
		self.driver_camera_mode = mode
		self.nt_table.putNumber('camMode', mode)


	def set_led_mode(self, mode):
		self.nt_table.putNumber('ledMode', mode)


	def turn_leds_on(self):
		self.set_led_mode(const.LIMELIGHT_LED_MODE_ON)


	def turn_leds_off(self):
		self.set_led_mode(const.LIMELIGHT_LED_MODE_OFF)


	def get_distance_to_target(self):
		# ty = self.nt_table.getNumber('ty', 0)

		# hood_angle = self.robot.hood.get_angle()

		# SHOOTER_WHEEL_BOTTOM_SHAFT_HEIGHT = 40
		# limelight_mount_height = SHOOTER_WHEEL_BOTTOM_SHAFT_HEIGHT + 13.5 * math.sin(math.radians(hood_angle + 5))
		# mount_angle = hood_angle + 29
		# hoop_diameter = 4*12 + 6
		# offset = 0.7 * hoop_diameter

		# distance = (const.TARGET_DISTANCE_FROM_GROUND - limelight_mount_height) / tan(radians(mount_angle + ty)) + offset

		ty = radians(self.nt_table.getNumber('ty', 0))
		overall_angle = radians(54-self.robot.hood.get_angle())
		distance = (const.TARGET_DISTANCE_FROM_GROUND - const.LIMELIGHT_MOUNT_HEIGHT) / tan(overall_angle + ty) + const.LIMELIGHT_DISTANCE_FUDGE_INCHES

		return distance


	def toggle_driver_mode(self):
		# We're not using a driver camera this year, 2022
		if self.driver_camera_mode == const.LIMELIGHT_DRIVER_CAMERA_MODE_ENABLED:
			self.set_driver_mode(const.LIMELIGHT_DRIVER_CAMERA_MODE_DISABLED)
		else:
			self.set_driver_mode(const.LIMELIGHT_DRIVER_CAMERA_MODE_ENABLED)

	def get_target_corners(self):
		targets = []

		for i in range(3):
			x_name = 'tx' + str(i)
			y_name = 'ty' + str(i)

			# Convert from -1 to 1.0 screenspace to pixels
			x_ss = self.nt_table.getNumber( x_name, 0 )
			y_ss = self.nt_table.getNumber( y_name, 0 )
			x = int( ( x_ss / 2.0 + 0.5 ) * 320 )
			y = int( ( y_ss / 2.0 + 0.5 ) * 240 )

			corners = ( x, y )
			targets.append( corners )

		return targets

	def get_main_target(self):
		x_ss = self.nt_table.getNumber( 'tx', None) # type: ignore
		y_ss = self.nt_table.getNumber( 'ty', None) # type: ignore

		return (x_ss, y_ss)

	def get_galactic_search_path(self):
		target = self.get_main_target()
		if target[0] is not None:
			#if target[0] < -10:
			return 'searchreda.wpilib.json'
			#else:
				#return 'searchredb.wpilib.json'
		print('target = ', target)

	def log(self):
		# Put number & red/green indicator on Shuffleboard
		if self.is_target_visible():
			self.robot.nt_robot.putBoolean('Target is Visible', True)
			self.robot.nt_robot.putString( 'Target Distance', '{0:.2f}'.format( self.get_distance_to_target() ) )
			# print( 'Targets =', self.get_target_corners() )
			# print('path_filename = ', self.get_galactic_search_path())
		else:
			self.robot.nt_robot.putBoolean('Target is Visible', False)
			self.robot.nt_robot.putString( 'Target Distance', '-1' )


	def stop(self):
		pass
