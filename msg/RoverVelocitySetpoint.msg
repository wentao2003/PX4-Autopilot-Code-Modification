uint64 timestamp # time since system start (microseconds)

float32[2] velocity_ned # 2 Dimensional velocity setpoint in NED frame (Use this OR velocity_xyz and set other to NAN, velocity_ned is prioritized)
float32[2] velocity_xyz # 2 Dimensional velocity setpoint in X, Y, Z body frame (X: Positiv = forwards, Negativ = backwards, Y: Positiv = right, Negativ = left)

# Only for ackermann and differential rovers
bool backwards	        # Flag for backwards driving (only relevant for ackermann and differential rovers when specifying velocity in NED frame with velocity_ned)

# Only for differential and mecanum rovers
float32 yaw	        # Yaw setpoint [rad] (Mecanum: Can be specified in addition to velocity vector, Differential: Used for spot turning if finite)

# TOPICS rover_velocity_setpoint
