Header header
string name         # finger name
int32 motor_id   # motor ids controlling this finger(joint)
int32 motor_temps # motor temperatures, same order as motor_ids

float64 goal_pos    # commanded position (in radians)
float64 current_pos # current joint position (in radians)
float64 error       # error between commanded and current positions (in radians)
float64 velocity    # current joint speed (in radians per second)
float64 load        # current load
bool is_moving      # is joint currently in motion


