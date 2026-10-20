#!/bin/bash
# Start de bridge (verbinding met de robot) en de kaart/verken-node.
source /opt/ros/jazzy/setup.bash

python3 /maqueen/maqueen_bridge.py --ros-args \
    -p port:="$PORT" \
    -p max_lin:="${MAX_LIN:-0.2}" \
    -p min_pwm:="${MIN_PWM:-50}" &

python3 /maqueen/maqueen_slam.py &

# Stopt een van beide, dan stopt de container (en herstart Docker hem)
wait -n
exit 1
