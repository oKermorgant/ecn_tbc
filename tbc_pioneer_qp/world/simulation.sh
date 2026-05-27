#!/usr/bin/bash

pioneer_dir=$(dirname $0)/..
watchdog=$pioneer_dir/build/watchdog

if [[ ! -e $watchdog ]]; then
    echo "compiling base libraries..."
    mkdir -p $pioneer_dir/build
    (cd $pioneer_dir/build && cmake .. &> /dev/null && make watchdog -j4 &> /dev/null)
fi

# screen -dmS watchdog bash -c $watchdog

#ign gazebo -r $pioneer_dir/world/pioneer_world.sdf  &> /dev/null
gz sim -r $pioneer_dir/world/pioneer_world.sdf  &> /dev/null

# kill watchdog when Gazebo exits
# screen -S watchdog -X stuff '^C'
