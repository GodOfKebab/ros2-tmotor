# "motor packets" in servo mode received from the motor according to the datasheet

# "Driver board ID code"
uint8 id

# angular position in radians (+- 2 turns)
float64 angular_position

# angular velocity in rad/s (depends on the motor, +-45 for AK60)
float64 angular_velocity

# torque applied in N.m (depends on the motor, +-15 for AK60)
float64 torque

# temperature in C (>=1C accuracy)
int8 temp

# error code flag (according to the datasheet section 5.2.1 or page 39/52)
#   - O means no fault
#   - 1 means over temperature fault
#   - 2 means over current fault
#   - 3 means over voltage fault
#   - 4 means under voltage fault
#   - 5 means encoder fault
#   - 6 means phase current unbalance fault (The hardware may be damaged)
#   - 7 means teensy error
uint8 error_code
