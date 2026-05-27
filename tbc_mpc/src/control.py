#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from tbc_mpc.splines import Splines
import numpy as np
from time import time

from ackermann_msgs.msg import AckermannDrive
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
from tf2_ros import Buffer, TransformListener
from sensor_msgs.msg import JointState
from math import atan2, cos, sin

dt = 0.1
Nhorizon = 10
Tmax = Nhorizon * dt
Qx = Qy = 1e2
Rv = 1e-1
Rsteer = 1e3


class Control(Node):

    def now(self):
        s,ns = self.get_clock().now().seconds_nanoseconds()
        return s + 1e-9*ns

    def __init__(self):

        super().__init__('mpc', namespace='zoe')

        # splines change path to local reference trajectory up to Tmax
        self.v0 = 1.
        self.splines = Splines()

        api = self.declare_parameter('api', 'dompc').value

        if api == 'dompc':
            from dompc_api import ZoeMPC
        elif api == 'acados':
            from acados_api import ZoeMPC
        else:
            from mppi_api import ZoeMPC
        self.solver = ZoeMPC(dt, Nhorizon, Qx, Qy, Rv, Rsteer)

        # init plumbing
        self.cmd = AckermannDrive()
        self.cmd_pub = self.create_publisher(AckermannDrive, 'cmd', 1)
        self.steering = None
        self.js_sub = self.create_subscription(JointState, 'joint_states', self.js_cb, 1)
        self.tf_buffer = Buffer(node=self)
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.path_sub = self.create_subscription(Path, 'plan', self.path_cb, 1)

        self.spline_path_pub = self.create_publisher(Path, 'local_plan', 1)
        self.spline_path = Path()
        self.spline_path.header.frame_id = 'map'

        self.mpc_path_pub = self.create_publisher(Path, 'predicted_plan', 1)
        self.mpc_path = Path()
        self.mpc_path.header.frame_id = 'map'

        self.create_timer(dt, self.track)

    def js_cb(self, js: JointState):
        if 'steering' not in js.name:
            return
        self.steering = js.position[js.name.index('steering')]

    def path_cb(self, path: Path):
        self.splines.set_path(path)

    def pose(self):
        now = rclpy.time.Time()
        if not self.tf_buffer.can_transform('map', 'zoe/base_link', now):
            return [None]*3
        pose = self.tf_buffer.lookup_transform('map', 'zoe/base_link', now).transform

        return pose.translation.x, pose.translation.y, 2*atan2(pose.rotation.z,pose.rotation.w)

    def publish_ref(self, ref):
        '''
        publishes output of spline fitting on Tmax horizon for reference
        '''
        self.spline_path.poses = []
        self.spline_path.header.stamp = self.get_clock().now().to_msg()

        for t in np.linspace(0, Tmax*1.5, 20):
            wp = ref(t)
            pose = PoseStamped()
            pose.pose.position.x = wp[0]
            pose.pose.position.y = wp[1]
            pose.pose.position.z = 0.2
            self.spline_path.poses.append(pose)
        self.spline_path_pub.publish(self.spline_path)

    def move(self, u):
        self.cmd.speed = float(u[0])
        self.cmd.steering_angle_velocity = float(u[1])
        self.cmd_pub.publish(self.cmd)

    def stop(self):
        self.move([0,0])

    def track(self):

        x,y,theta = self.pose()

        if self.steering is None or x is None:
            self.stop()
            return

        # build full state SE(2) + steering angle
        x0 = np.array([x,y,theta,self.steering])

        # ref is a function t -> (x,y) from 0 to Tmax
        ref = self.splines.spline_from(x0, Tmax, self.v0)

        if ref is None:
            self.stop()
            return
        self.publish_ref(ref)

        # call MPC from the current state
        tc = time()
        u, traj = self.solver.solve(x0, ref)

        if u is not None:

            print(f'MPC solved for n = {Nhorizon}, took {1000*(time()-tc): .2f} ms')

            self.move(u)
            self.v0 = u[0]

            # also display predicted trajectory
            self.mpc_path.poses = []
            for x,y,theta,_ in traj:
                pose = PoseStamped()
                pose.pose.position.x = float(x)
                pose.pose.position.y = float(y)
                pose.pose.position.z = 0.2
                pose.pose.orientation.z = sin(theta/2.)
                pose.pose.orientation.w = cos(theta/2.)
                self.mpc_path.poses.append(pose)
            self.mpc_path.header.stamp = self.get_clock().now().to_msg()
            self.mpc_path_pub.publish(self.mpc_path)
        else:
            print(f'MPC could not be solved for n = {Nhorizon}, took {1000*(time()-tc): .2f} ms')


rclpy.init()
rclpy.spin(Control())
rclpy.shutdown()
