uint64 timestamp # time since system start (microseconds)

# Only for ackermann rovers
float32 normalized_steering_angle # [-1, 1] Normalized steering angle (Positiv: Steer right, Negativ: Steer left)

# Only for differential/mecanum rovers
float32 normalized_speed_diff     # [-1, 1] Normalized speed difference between the left and right wheels of the rover (Positiv = Turn right, Negativ: Turn left)

# TOPICS rover_steering_setpoint
