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


max_vel = 0.277778 * 25      # 25 km/h
#max_vel = 0.277778 * 10      # 10 km/h
max_curv = 1./1.2             # curvature radius in [m^{-1}]
min_vel = max_vel/2         # at highest curvature
max_acc = .5*9.81


class WP:
    def __init__(self, pose = None, x = None, y = None):
        if pose is not None:
            self.x = pose.pose.position.x
            self.y = pose.pose.position.y
        else:
            self.x = x
            self.y = y

    def dist_sq(self,x,y):
        return (x-self.x)**2+(y-self.y)**2

    def dist(self, x = 0., y = 0.):
        return np.sqrt(self.dist_sq(x,y))

    def interp(self, other):
        dmin = 0.1
        d = self.dist(other.x, other.y)
        path = []
        dx = other.x-self.x
        dy = other.y-self.y
        steps = int(d/dmin)
        print('interp on',steps)
        for step in np.linspace(0,1,steps):
            path.append(WP(x = self.x+step*dx,
                           y=self.y+step*dy))
        return path[1:]


class Splines:

    def __init__(self):
        self.path = None

    def set_path(self, path: Path):

        if not len(path.poses):
            self.path = None
            return

        self.path = [WP(pose = path.poses[0])]
        for pose in path.poses[1:]:
            self.path.append(WP(pose = pose))
            # self.path += self.path[-1].interp(WP(pose = pose))
        print('from',len(path.poses),'to',len(self.path))

    def spline_from(self, x0, Tmax, v0 = 0):

        if not self.path:
            print('no path to track')
            return None

        # display reference a bit further than MPC prediction
        Tmax *= 1.5

        # get nearest forward point
        x,y,_,_ = x0
        offset = 5
        start = cur = min(np.argmin([wp.dist_sq(x,y) for wp in self.path])+offset, len(self.path)-1)

        # get last one
        T = [0.]
        prev = self.path[cur]
        xy = np.array([[prev.x, prev.y]])
        while T[-1] < Tmax and cur < len(self.path) -1:
            cur += 1
            wp = self.path[cur]
            xy = np.vstack((xy,[wp.x,wp.y]))
            T.append(T[-1] + prev.dist(wp.x, wp.y)/max_vel)
            prev = wp
        print(f'fit on {len(T)}: {T}, end @ {cur}/{len(self.path)}')

        if len(T) < 3:
            def sp(t):
                return xy[-1]
        else:
            # hand-rolled polynomial fit to avoid Scipy version mess
            deg = 3
            Ta = np.array(T).reshape(-1,1)
            Ts = np.linalg.pinv(np.hstack([Ta**d for d in range(deg+1)]))
            C = (Ts @ xy).T

            def sp(t):
                ts = np.array([t**d for d in range(deg+1)]).reshape(-1,1)
                return (C@ts).flatten()

        def ref(t, include_u = False):
            ret = sp(min(t,T[-1]))
            return np.hstack((ret, [0,0]*(1+include_u)))

        self.path = self.path[max(0,start-offset - 1):]

        return ref
