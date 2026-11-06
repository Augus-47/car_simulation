# Unified c305 vehicle command / feedback contract.
# velocity: signed longitudinal speed [m/s], positive forward.
# steering: equivalent center front-wheel steering angle [degree], positive left.
# brake: normalized request/state [0.0, 1.0].
# gears: reserved; keep 0.0. Direction is represented by velocity sign.
std_msgs/Header header
float32 velocity
float32 steering
float32 brake
float32 gears
string led
float32 battary
