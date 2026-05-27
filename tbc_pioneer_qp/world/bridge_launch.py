#!/usr/bin/env python3
from simple_launch import SimpleLauncher, GazeboBridge


def generate_launch_description():

    sl = SimpleLauncher(use_sim_time = True)

    bridges = [GazeboBridge.clock()]

    bridges.append(GazeboBridge('/pioneer/camera', '/pioneer/image', 'sensor_msgs/Image', GazeboBridge.gz2ros))

    sl.create_gz_bridge(bridges, 'pioneer_bridge')

    return sl.launch_description()
