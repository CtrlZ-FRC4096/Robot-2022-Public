"""
Ctrl-Z FRC Team 4096
FIRST Robotics Competition 2022
Code for robot "swerve drivetrain prototype"
contact@team4096.org

Some code adapted from:
https://github.com/SwerveDriveSpecialties

Some code adapted from:
https://github.com/SwerveDriveSpecialties
"""

"""
Prepend these to any port IDs.
DIO = Digital I/O
AIN = Analog Input
PWM = Pulse Width Modulation
CAN = Controller Area Network
PCM = Pneumatic Control Module
PDP = Power Distribution Panel
"""

from commands2._impl import TrapezoidProfileCommand
#from wpilib import controller
from wpimath import trajectory
import math

from numpy.polynomial import Polynomial

#from wpimath.trajectory._trajectory import TrajectoryConfig

### CONSTANTS ###

# Is running simulator. Value is set in robot.py, robotInit
IS_SIMULATION		= False

# Directions, mainly used for swerve module positions on drivetrain
FRONT_LEFT			= 'front_left'
FRONT_RIGHT			= 'front_right'
BACK_LEFT			= 'back_left'
BACK_RIGHT			= 'back_right'

# Used in ctre configurations
CLOCKWISE			= True
COUNTER_CLOCKWISE	= False

# Field centric directions/axes
FIELD_FRONT_BACK	= 'field_front_back'
FIELD_LEFT_RIGHT	= 'field_left_right'
FIELD_ROTATION		= 'field_rotation'

# Robot drivebase dimensions, in inches and meters
DRIVE_BASE_WIDTH	= 25
DRIVE_BASE_LENGTH	= 25
DRIVETRAIN_TRACKWIDTH_METERS	= 0.635
DRIVETRAIN_WHEELBASE_METERS		= 0.635

# Robot speeds
MAX_VEL_METERS 		= 6380 / 60 * (12.0 / 24.0) * (22.0 / 24.0) * (15.0 / 45.0) * 0.10033 * math.pi
MAX_ANG_VEL_RAD    	= MAX_VEL_METERS / math.hypot(DRIVETRAIN_TRACKWIDTH_METERS / 2.0, DRIVETRAIN_WHEELBASE_METERS / 2.0)
MAX_ANG_ACCEL      	= 6 * math.pi #This isn't used anywhere? Also just a WAG, so probably not accurate

# Module Steer Constants

STEER_KP = 0.43
STEER_KI = 0.0
STEER_KD = 0.004

# Swerve Module 1 (Front Left)

DRIVE_MOTOR_1_ID	= 15
STEER_MOTOR_1_ID	= 16
STEER_ENCODER_1_ID	= 25
STEER_ENCODER_1_OFFSET = 360-252.5

# Swerve Module 2 (Front Right)

DRIVE_MOTOR_2_ID	= 4
STEER_MOTOR_2_ID	= 5
STEER_ENCODER_2_ID	= 26
STEER_ENCODER_2_OFFSET = 360-54.8

# Swerve Module 3 (Back Left)

DRIVE_MOTOR_3_ID	= 18
STEER_MOTOR_3_ID	= 17
STEER_ENCODER_3_ID	= 27
STEER_ENCODER_3_OFFSET = 360-322.6

# Swerve Module 4 (Back Right)

DRIVE_MOTOR_4_ID	= 3
STEER_MOTOR_4_ID	= 2
STEER_ENCODER_4_ID	= 28
STEER_ENCODER_4_OFFSET = 360-210.8

GYRO_RESET_VALUE = 0


# Other

CAN_GYRO_ID   		= 24
CAN_PDH             = 21

# Auto
AUTO_RESOLUTION         = .02 #path resolution in seconds
MAX_VEL_METERS_AUTO     = 1 #This is the max velocity you want the robot to drive at, not its true max velocity
MAX_ANG_VEL_RAD_AUTO    = MAX_VEL_METERS_AUTO / math.hypot(DRIVETRAIN_TRACKWIDTH_METERS / 2.0, DRIVETRAIN_WHEELBASE_METERS / 2.0) #This is the max velocity you want the robot to rotate at, not its true max rotational velocity
MAX_ACCEL_AUTO			= 4 #This is the max rate you want the robot to accelerate at, not its true max acceleration
MAX_ANG_ACCEL_AUTO      = 6 * math.pi #This is the max rate you want the robot to accelerate at, not its true max acceleration
X_KP                    = 0.16#0.12667925	#0.73225
X_KI                    = 0.015 #0.015 #0.0346173		#0.2001
X_KD                    = 0.0#0.01165449	#0.067367
Y_KP                    = X_KP
Y_KI                    = X_KI
Y_KD                    = X_KD
THETA_KP                = .232 #* 2.866 * 5.0
THETA_KI                = 0.0625 #* 2.866 * 5.0
THETA_KD                = 0


# Drive reduction * wheel diameter * pi * motor encoder ticks per 100ms * 10 (to get the encoder units to ticks per second)
# THIS IS THE CALCULATED VALUE, THIS VALUE SHOULD BE MEASURED AND CALIBRATED
METERS_PER_DRIVE_TICK   = (12.0 / 24.0) * (22.0 / 24.0) * (15.0 / 45.0) * 0.10033 * math.pi / 2048 * 10


# Intake constants
CAN_INTAKE = 7
#PWM_INTAKE = 9
PH_SOLENOID_INTAKE_DEPLOY_1 = 5
PH_SOLENOID_INTAKE_DEPLOY_2 = 6
PH_SOLENOID_INTAKE_KICKER_1 = 9
PH_SOLENOID_INTAKE_KICKER_2 = 10


# Indexer constants

PWM_INDEXER_LOWER = 3
PWM_INDEXER_UPPER = 2

AIO_INDEXER_UPPER = 2
AIO_INDEXER_LOWER = 1

# Hood constants

# In inches
HOOD_MIN_DISTANCE = 55
HOOD_MAX_DISTANCE = 240

HOOD_MIN_DISTANCE_ANGLE = 43.4
HOOD_MAX_DISTANCE_ANGLE = 20

# Limelight mapping
#DISTANCES_REAL = [  44,   64,   97,  117,  139,  168,   187,   213,   275,]
#DISTANCES_LL =   [  52,   74,  108,  134,  157,  185,   222,   242,   290,]

DISTANCES_REAL = [  44,   64,   97,  117,  139,  168,   187,   214,   259,]
DISTANCES_LL =   [  52,   74,  108,  134,  157,  185,   222,   235,   283,]

# Shooter mapping
# DISTANCES =      [  44,   64,   97,  117,  139,  168,   187,   213,]
# HOOD_ANGLES =    [  12,   18,   27,   29,   35,   39,    39,    42,]
# BOT_VELOCITIES = [3100, 3100, 3400, 3500, 3800, 4200,  4650,  5200,]
# TOP_VELOCITIES = [6800, 6800, 7500, 7750, 8400, 9150, 10200, 11400,]
DISTANCES = [80, 165, 300]
HOOD_ANGLES = [16, 25, 33]
BOT_VELOCITIES = [3700, 4400, 6000]
TOP_VELOCITIES = [6000 , 9400, 11000]


DISTANCES_TOF = [0, 3, 6]
TOF_SECONDS = [0, 1, 2]


# LL_TO_REAL_DIST = Polynomial.fit(DISTANCES_LL, DISTANCES_REAL, 2)
LL_TO_REAL_DIST = lambda x: x

_DIST_TO_HOOD = Polynomial.fit(DISTANCES, HOOD_ANGLES, 2)
DIST_TO_TOF = Polynomial.fit(DISTANCES_TOF, TOF_SECONDS, 2)
# 80 3700 7300 16
# 165 4400 9400 25
# 300 6000 11000 33
# 125 4600 7000 23

# DIST_TO_BOT = Polynomial.fit(DISTANCES, BOT_VELOCITIES, 2)
# DIST_TO_TOP = Polynomial.fit(DISTANCES, TOP_VELOCITIES, 2)
_DIST_TO_BOT = Polynomial.fit(DISTANCES, BOT_VELOCITIES, 2)
_DIST_TO_TOP = Polynomial.fit(DISTANCES, TOP_VELOCITIES, 2)

DIST_SHOOTER_FUDGE = 24

DIST_TO_TOP = lambda x: _DIST_TO_TOP(x + DIST_SHOOTER_FUDGE)
DIST_TO_BOT = lambda x: _DIST_TO_BOT(x + DIST_SHOOTER_FUDGE)
DIST_TO_HOOD = lambda x: _DIST_TO_HOOD(x)

# INTERPOLATION_X_TABLE = [97, 136, 182]
# LOWER_INTERPOLATION_Y_TABLE = [4000, 4000, 4700]
# UPPER_INTERPOLATION_Y_TABLE = [5000, 5400, 6200]
# HOOD_INTERPOLATION_Y_TABLE = []
# HOOD_INTERPOLATION_X_DIST_TABLE = []
# HOOD_INTERPOLATION_Y_TOF_TABLE = []

HOOD_LOWER_LIMIT = 4
HOOD_UPPER_LIMIT = 55

CAN_HOOD = 21
HOOD_INVERTED = False

# Turret constants
CAN_TURRET = 20

TURRET_KP = 0.0
TURRET_KI = 0.0
TURRET_KD = 0.0

TURRET_LEAD_M = 0.0
TURRET_LEAD_B = 0.0

TURRET_STAY_OUT = 30

DIO_TURRET_CW_SWITCH = 2
DIO_TURRET_CCW_SWITCH = 3

# Needs to be checked
TURRET_INVERTED = False


# Shooter constants
CAN_SHOOTER_UPPER = 12
CAN_SHOOTER_LOWER = 10

SHOOTER_MAX_VELOCITY = 0

#Shooter PID constants
SHOOTER_TOP_KF = 0
SHOOTER_TOP_KP = 1000
SHOOTER_TOP_KI = 0
SHOOTER_TOP_KD = 0
SHOOTER_TOP_IZ = 0

SHOOTER_BOT_KF = 0
SHOOTER_BOT_KP = 1000
SHOOTER_BOT_KI = 0
SHOOTER_BOT_KD = 0
SHOOTER_BOT_IZ = 0

# Limelight constants
LIMELIGHT_DRIVER_CAMERA_MODE_DEFAULT = 0
LIMELIGHT_DRIVER_CAMERA_MODE_ENABLED = 1
LIMELIGHT_DRIVER_CAMERA_MODE_DISABLED = 2
LIMELIGHT_STREAM_MODE = 0
LIMELIGHT_LED_MODE_ON = 3
LIMELIGHT_LED_MODE_OFF = 1
TARGET_DISTANCE_FROM_GROUND = 104
LIMELIGHT_MOUNT_HEIGHT = 44
LIMELIGHT_MOUNT_ANGLE = 0
LIMELIGHT_DISTANCE_FUDGE_INCHES = 0

JENNY = 8375_309

# Climber
AIO_CLIMBER_TILT_ENCODER	= 0

DIO_CLIMBER_BOTTOM_LIMIT	= 0
DIO_CLIMBER_TOP_LIMIT		= 1

PCM_SOLENOID_CLIMBER_DEPLOY_1 = 7
PCM_SOLENOID_CLIMBER_DEPLOY_2 = 8

PCM_SOLENOID_CLIMBER_TILT_BRAKE_1 = 3
PCM_SOLENOID_CLIMBER_TILT_BRAKE_2 = 4

PCM_SOLENOID_CLIMBER_LIFT_BRAKE_1 = 1
PCM_SOLENOID_CLIMBER_LIFT_BRAKE_2 = 2

CAN_CLIMBER_LIFT_1	= 0
CAN_CLIMBER_LIFT_2	= 1
CAN_CLIMBER_TILT	= 19