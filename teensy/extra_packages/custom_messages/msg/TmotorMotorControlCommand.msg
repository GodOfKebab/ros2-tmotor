# motor id (usually, this is 0x1 by default)
uint8 id

# desired angular position in radians
float64 angular_position

# k_p gain with range 0.-500.
float64 k_p

# desired angular velocity in rad/s
float64 angular_velocity

# k_d gain with range 0.-5.
float64 k_d

# (desired/gravity vector) torque in N.m
float64 torque
