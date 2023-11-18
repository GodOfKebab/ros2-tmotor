# "motor packets" in servo mode received from the motor according to the datasheet

# angular position in radians (>=0.1 degree accuracy)
float64 angular_position

# angular velocity in rad/s (>=0.1 degree/s accuracy)
float64 angular_velocity

# current draw in A (>=10 mA accuracy)
float64 current

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
