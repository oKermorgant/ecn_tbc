#!/usr/bin/env python3
from simple_launch import SimpleLauncher


def generate_launch_description():

    sl = SimpleLauncher()
    sl.gz_launch(sl.find('tbc_pioneer_qp','pioneer_world.sdf'))

    return sl.launch_description()