#!/bin/bash

ros2 launch launch/all.launch.py &

python3 src/simulator/simulator/spawn_vehicle.py
