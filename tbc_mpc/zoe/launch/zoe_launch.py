from simple_launch import SimpleLauncher


def generate_launch_description():

    sl = SimpleLauncher()
    ns = sl.declare_arg('ns', 'zoe')
    sl.declare_arg('manual', True)

    x = sl.declare_arg('x', 0.)
    y = sl.declare_arg('y', 0.)
    theta = sl.declare_arg('theta', 0.)

    sl.declare_arg('use_angle_cmd', False, description = 'Whether to use steering angle or velocity in the command')

    with sl.group(ns=ns):

        sl.robot_state_publisher('tbc_mpc', 'zoe.xacro', xacro_args={'ns': ns})
        sl.node('map_simulator', 'spawn',
                parameters = {'x': x,
                              'y': y,
                              'theta': theta,
                              'size': [1.6, 2.8, 0.1],
                              'shape': 'rectangle',
                              'robot_color': [0,0,0],
                              'laser_color': [255,0,0],
                              'static_tf_odom': True})

        sl.node('map_simulator', 'kinematics.py', parameters = {'use_angle_cmd': sl.arg('use_angle_cmd'),
                                                                'dt': 0.01})

        with sl.group(if_arg = 'manual'):
            sl.node('slider_publisher', arguments=[sl.find('tbc_mpc', 'cmd.yaml')])

    return sl.launch_description()
