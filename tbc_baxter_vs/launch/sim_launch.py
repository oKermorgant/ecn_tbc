from simple_launch import SimpleLauncher, GazeboBridge
from launch.actions import ExecuteProcess

def generate_launch_description():

    sl = SimpleLauncher(use_sim_time=True)
    manual = sl.declare_arg('manual', True)

    sl.gz_launch(sl.find('tbc_baxter_vs', 'baxter_world.sdf'), '-r')
    sl.include('baxter_gz', 'upload_launch.py')

    with sl.group(ns = 'ball'):
        bridges = [GazeboBridge.clock(),
                   GazeboBridge('/model/ball/pose', 'pose', 'geometry_msgs/Pose', GazeboBridge.gz2ros),
                   GazeboBridge('/model/ball/cmd_vel', 'cmd_vel', 'geometry_msgs/Twist', GazeboBridge.ros2gz)]
        sl.create_gz_bridge(bridges)

        with sl.group(if_arg = 'manual'):
            sl.node('slider_publisher', 'slider_publisher', name='ball_setpoint',
                    arguments = [sl.find('tbc_baxter_vs', 'ball_setpoint.yaml')])

        sl.node('tbc_baxter_vs', 'ball_motion.py',
                parameters = sl.arg_map('manual'))

    return sl.launch_description()
