# Executable quintic MINCO trajectory. Each piece uses local seconds t in [0, durations[i]].
# coeff_x/y[6*i+k] is the coefficient of t^k (ascending powers, k=0..5).
std_msgs/Header header
builtin_interfaces/Time start_time
int32 traj_id
int32 order
float64 fixed_z
float64[] durations
float64[] coeff_x
float64[] coeff_y
