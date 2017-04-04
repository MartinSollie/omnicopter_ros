Header header

uint8 MODE_CONTROL_FORCE_ENU = 0
uint8 MODE_CONTROL_FORCE_BODY = 1
uint8 MODE_CONTROL_FORCE_BODY_UP = 2		# Pure upwards force, like quadrotors
uint8 MODE_CONTROL_ALTHOLD_FORCE_ENU = 3 	# Attitude not controlled since we only have altimeter on one side
uint8 MODE_CONTROL_ALTHOLD_FORCE_BODY = 4	# Pitch and roll not controlled
uint8 MODE_CONTROL_ALTHOLD_FORCE_BODY_UP = 5
uint8 position_control_mode

uint8 MODE_CONTROL_ATT = 6					# Attitude quaternion setpoint
uint8 MODE_CONTROL_RATES = 7				# Angular velocity setpoint
uint8 MODE_CONTROL_RP_ATT_Y_RATE = 8		# Control roll and pitch angles, and yawrate
uint8 MODE_CONTROL_YAWRATE = 9				# Control roll and pitch angles to 0, control yawrate
uint8 MODE_CONTROL_ZERO_ATT = 10			# Zero RPY, no manual input
uint8 attitude_control_mode


