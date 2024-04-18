uint64 timestamp # time since system start (microseconds)

float32 speed # [m/s] requested rover ground speed
float32 steering # [rad] requested rover steering angle
bool manual # manual steering flag
float32 speed_actual # [m/s] actual rover ground speed (for loging)
float32 lookahead_distance # [m] pure pursuit controller lookahead distance (for loging)
float32 heading_error # [deg] Heading error towards lookahead point (for loging)

# TOPICS ackermann_drive_setpoint
