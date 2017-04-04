Header header

uint8 SETPOINT_TYPE_POS = 0
uint8 SETPOINT_TYPE_VEL = 1
uint8 SETPOINT_TYPE_FORCE_ENU = 2
uint8 SETPOINT_TYPE_FORCE_BODY = 3
uint8 SETPOINT_TYPE_CLIMBRATE_FORCE_ENU = 4
uint8 SETPOINT_TYPE_ALTHOLD_FORCE_ENU = 5
uint8 type

# Position setpoint in local ENU frame [m]
float32 px
float32 py
float32 pz

# Velocity setpoint in local ENU frame [m/s]
float32 vx 
float32 vy
float32 vz

# Force setpoint [N]
float32 fx 
float32 fy
float32 fz



