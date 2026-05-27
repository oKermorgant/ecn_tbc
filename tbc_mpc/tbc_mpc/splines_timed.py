from nav_msgs.msg import Path
import numpy as np


def curvature(pp, p, pn) -> float:

    z1 = pp[0]+pp[1]*1j
    z2 = p[0]+p[1]*1j
    z3 = pn[0]+pn[1]*1j

    if (z1 == z2) or (z2 == z3) or (z3 == z1):
        raise ValueError(f"Duplicate points: {z1}, {z2}, {z3}")

    w = (z3 - z1)/(z2 - z1)

    # You should change 0 to a small tolerance for floating point comparisons
    if abs(w.imag) <= 0:
        return 0.

    c = (z2 - z1)*(w - abs(w)**2)/(2j*w.imag) + z1  # Simplified denominator
    r = abs(z1 - c)

    return 1./r


max_vel = 0.277778 * 30      # 30 km/h
max_vel = 0.277778 * 10      # 10 km/h
max_curv = 1./1.2             # curvature radius in [m^{-1}]
min_vel = max_vel/2         # at highest curvature
max_acc = .5*9.81


class WP:
    def __init__(self, t, pose = None, x = None, y = None):
        if pose is not None:
            self.x = pose.pose.position.x
            self.y = pose.pose.position.y
        else:
            self.x = x
            self.y = y
        self.t = t

    def dist_sq(self,x,y):
        return (x-self.x)**2+(y-self.y)**2

    def relvel_to(self, other):
        return np.sqrt(self.dist_sq(other.x,other.y))/(other.t-self.t)

    def interp(self, other):
        dmin = 0.1
        d = np.sqrt(self.dist_sq(other.x, other.y))
        path = []
        dt = other.t-self.t
        dx = other.x-self.x
        dy = other.y-self.y
        steps = int(d/dmin)
        print('interp on',steps)
        for step in np.linspace(0,1,steps):
            path.append(WP(self.t + step*dt,
                           x = self.x+step*dx,
                           y=self.y+step*dy))
        return path[1:]


class Splines:

    def __init__(self):
        self.path = None

    def set_path(self, path: Path):

        if not len(path.poses):
            self.path = None
            return

        v = np.zeros(len(path.poses))
        v[0] = v[-1] = 0.
        d = np.zeros(len(path.poses))
        wp = [0]

        def toXY(p):
            return np.array([p.pose.position.x, p.pose.position.y])

        # compute correct relative times to go through each pose
        # first pose is @ t = 0
        for k,pose in enumerate(path.poses[1:-1]):

            pp = toXY(path.poses[k])
            p = toXY(pose)
            pn = toXY(path.poses[k+2])
            v1 = p-pp
            v2 = pn-p
            d[k+1] = np.linalg.norm(v1)

            if np.dot(v1,v2) > 0:
                c = curvature(pp, p, pn) / max_curv
                c = min(1, max(c, 0))
                # c = 0 -> max_vel, c = 1 -> min_vel
                v[k+1] = max_vel*(1-c) + min_vel*c
            else:
                wp.append(k+1)

        d[-1] = d[-2]
        wp.append(len(path.poses)-1)

        # ensure max acceleration
        vi = np.array(v[:])
        ok = False
        while not ok:
            ok = True
            dv = abs(vi[1:] - vi[:-1])
            k = np.argmax(dv)
            vm = .5*(vi[k+1] + vi[k])
            dt = d[k+1] / vm
            acc = vm/dt
            if acc > 1.1*max_acc:
                ok = False
                if vi[k+1] > vi[k]:
                    vi[k+1] = vi[k] + max_acc*dt
                else:
                    vi[k] = vi[k+1] + max_acc*dt

        # write timestamps
        self.path = [WP(0., pose = path.poses[0])]
        for k, pose in enumerate(path.poses[1:]):
            vm = .5*(vi[k+1] + vi[k])
            if vm != 0.:
                t = self.path[-1].t + d[k+1] / vm
            else:
                t = self.path[-1].t + 0.01
            self.path += self.path[-1].interp(WP(t, pose = pose))
        print('from',len(path.poses),'to',len(self.path))

    def spline_from(self, t0, x0, Tmax, v0 = 0):

        if not self.path:
            print('no path to track')
            return None

        Tmax *= 2

        x,y,_,_ = x0
        # get nearest forward point
        idx = min(np.argmin([wp.dist_sq(x,y) for wp in self.path])+1, len(self.path)-1)

        # check we are not already there
        start = self.path[idx]
        dx = np.sqrt(start.dist_sq(x,y))

        # first join starting point at given velocity and max accel
        A = .5*max_acc
        B = v0
        C = -dx
        D = np.sqrt(B**2-4*A*C)
        dt = [(-B+e*D)/(2*A) for e in (-1,1)]
        dt = [v for v in dt if v > 0][0]

        # dt = 0.1

        T = [t0,t0+dt]
        xy = np.array([[x,y],[start.x, start.y]])

        # time ref wrt current position
        t_offset = start.t - t0 - dt

        for wp in self.path[idx+1:]:
            T.append(wp.t - t_offset)
            xy = np.vstack((xy,[wp.x,wp.y]))
            if T[-1] > Tmax:
                break
        print('fit on',len(T))

        # hand-rolled polynomial fit to avoid Scipy version mess
        deg = 2
        Ta = np.array(T).reshape(-1,1)-t0
        Ts = np.linalg.pinv(np.hstack([Ta**d for d in range(deg+1)]))
        C = (Ts @ xy).T

        def sp(t):
            ts = np.array([(t-t0)**d for d in range(deg+1)]).reshape(-1,1)
            return (C@ts).flatten()

        def ref(t, include_u = False):
            ret = sp(min(t,T[-1]))
            return np.hstack((ret, [0,0]*(1+include_u)))

        # self.path = self.path[max(0,idx-2):]

        return ref
