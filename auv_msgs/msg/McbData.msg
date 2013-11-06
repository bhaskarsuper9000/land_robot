# MCB Modes
uint8 NOTHING_MODE=0
uint8 CONTROL_MODE=1
uint8 SET_YAW_PARAMETER_MODE=2
uint8 SET_DEPTH_PARAMETER_MODE=3
uint8 SET_SWAY_PARAMETER_MODE=4
uint8 SET_SURGE_PARAMETER_MODE=5
uint8 SET_PITCH_PARAMETER_MODE=6
uint8 SET_PSB_PARAMETER_MODE=7
uint8 CLEAR_BUFFERS_MODE=15

#PSB modes: only checked when Mcb mode is in SET_PSB_PARAMETERS
#uint8 NOTHING_MODE=0
uint8 CALIBRATE_INTERCEPT_MODE=1
uint8 SET_SLOPE_INTERCEPT_MODE=2

uint8 mcb_mode
uint8 psb_mode
bool yaw
bool pitch
bool depth
bool surge
bool sway
uint8 psb_slope
uint16 psb_intercept
