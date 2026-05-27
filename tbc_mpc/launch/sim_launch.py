from simple_launch import SimpleLauncher


def generate_launch_description():

    sl = SimpleLauncher()
    manual = sl.declare_arg('manual', False)
    sl.declare_arg('use_astar', True, description = 'Use A* for planning (otherwise use hybrid)')

    # run simulation
    sl.include('map_simulator', 'simulation2d_launch.py',
        launch_arguments={'map': sl.find('tbc_mpc', 'ecn_map.yaml'),
                                'display': False,
                                'map_server': True})

    # spawn Zoe vehicle
    sl.include('tbc_mpc', 'zoe_launch.py',
               launch_arguments={'manual': manual, 'x': 223.86, 'y': 79.18, 'theta': 0.559})

    # run RViz
    sl.rviz(sl.find('tbc_mpc', 'zoe.rviz'))

    # run global planner
    with sl.group(ns = 'zoe'):
        sl.node('nav2_planner', 'planner_server',
                parameters = [sl.find('tbc_mpc', 'nav2.yaml')],
                remappings = {'/zoe/map': '/map','map': '/map'})
        sl.node('nav2_lifecycle_manager','lifecycle_manager',name='lifecycle_manager',
                output='screen',
                parameters=[{'autostart': True,
                            'node_names': ['planner_server']}])

        sl.node('tbc_mpc', 'navigator',
                parameters = sl.arg_map('use_astar'))

    return sl.launch_description()
