"""Bounded, route-constrained GPS/encoder progress estimator (not a full pose EKF)."""
import bisect
import math


class RouteGeometry:
    def __init__(self, points):
        if len(points) < 2:
            raise ValueError('route needs at least two points')
        self.points = points
        self.s = [0.0]
        for a, b in zip(points, points[1:]):
            self.s.append(self.s[-1] + math.hypot(b.x-a.x, b.y-a.y))
        if self.s[-1] <= 0:
            raise ValueError('route has no length')

    def index(self, progress):
        return max(0, min(len(self.points)-1, bisect.bisect_right(self.s, progress)-1))

    def project(self, x, y, center, window):
        candidates = []
        for i in range(max(0, self.index(center-window)-1),
                       min(len(self.points)-1, self.index(center+window)+1)):
            a, b = self.points[i:i+2]
            dx, dy = b.x-a.x, b.y-a.y
            length2 = dx*dx+dy*dy
            if length2 <= 1e-12:
                continue
            t = max(0., min(1., ((x-a.x)*dx+(y-a.y)*dy)/length2))
            s = self.s[i]+t*math.sqrt(length2)
            if abs(s-center) <= window:
                candidates.append((math.hypot(x-a.x-t*dx, y-a.y-t*dy), s))
        if not candidates:
            raise ValueError('no route segment in search window')
        return min(candidates)


class FusedProgress:
    def __init__(self, route, initial_index=0, innovation_m=1.5,
                 max_cross_track_m=0.75, gps_max_std_m=0.5,
                 max_dr_sec=3.0, max_dr_m=2.0, max_std_m=0.5,
                 odom_timeout_sec=0.3, initial_window_m=3.0):
        if not 0 <= initial_index < len(route.points):
            raise ValueError('initial_index outside route')
        self.route = route
        self.s = route.s[initial_index]
        self.variance = 0.25
        self.innovation = innovation_m
        self.cross_track = max_cross_track_m
        self.gps_max_std = gps_max_std_m
        self.max_dr_sec, self.max_dr_m = max_dr_sec, max_dr_m
        self.max_std, self.odom_timeout = max_std_m, odom_timeout_sec
        self.initial_window = initial_window_m
        self.initialized = False
        self.good_fixes = 0
        self.last_gps = self.last_odom = None
        self.dr_distance = 0.0
        self.fault = ''
        self.gps_reason = 'waiting for GPS anchor'

    def predict(self, signed_distance, now):
        if not math.isfinite(signed_distance) or abs(signed_distance) > 1.0:
            self.fault = 'odom discontinuity; reset required'
            return False
        self.last_odom = now
        if self.initialized:
            self.s += signed_distance
            self.variance += 0.0025*abs(signed_distance)
            self.dr_distance += abs(signed_distance)
        return True

    def correct(self, x, y, variance, now):
        if not all(math.isfinite(v) for v in (x, y, variance)) or not 0 <= variance <= self.gps_max_std**2:
            self.gps_reason = 'GPS covariance rejected'
            return False
        if self.last_odom is None or now-self.last_odom > self.odom_timeout:
            self.gps_reason = 'fresh odometry required to anchor GPS'
            return False
        window = self.innovation if self.initialized else self.initial_window
        try:
            lateral, measured_s = self.route.project(x, y, self.s, window)
        except ValueError as exc:
            self.gps_reason = str(exc)
            return False
        if lateral > self.cross_track or abs(measured_s-self.s) > window:
            self.gps_reason = 'GPS route/innovation gate rejected'
            self.good_fixes = 0
            return False
        r = max(variance, 0.0025)
        if not self.initialized:
            if self.good_fixes and abs(measured_s-self.s) > 0.5:
                self.good_fixes = 0
            self.s = measured_s
            self.variance = r
            self.good_fixes += 1
            self.initialized = self.good_fixes >= 3
        else:
            gain = self.variance/(self.variance+r)
            self.s += gain*(measured_s-self.s)
            self.variance = (1-gain)*self.variance
        self.last_gps = now
        self.dr_distance = 0.0
        self.gps_reason = ''
        return True

    def validity(self, now):
        if self.fault:
            return False, self.fault
        if not self.initialized:
            return False, 'waiting for three consistent GPS fixes'
        if self.last_odom is None or now-self.last_odom > self.odom_timeout:
            return False, 'odometry stale'
        if now-self.last_gps > self.max_dr_sec or self.dr_distance > self.max_dr_m:
            return False, 'dead-reckoning limit exceeded'
        if math.sqrt(self.variance) > self.max_std:
            return False, 'position uncertainty exceeded'
        if not -0.25 <= self.s < self.route.s[-1]:
            return False, 'route end or outside route'
        return True, ''


class WaypointSchedule:
    def __init__(self, route, initial_index=0, ramp_stop_index=-1,
                 ramp_first_index=-1, front_offset_m=0.0,
                 acceleration_start_index=-1, acceleration_end_index=-1):
        self.route = route
        self.completed = set()
        self.pending = None
        self.ramp_configured = False
        self.targets = []
        if front_offset_m < 0 or not math.isfinite(front_offset_m):
            raise ValueError('front offset must be finite and nonnegative')
        if ramp_stop_index >= 0 or ramp_first_index >= 0:
            if not 0 <= ramp_first_index < ramp_stop_index < len(route.points):
                raise ValueError('explicit first and second ramp indices required in route order')
            if any(route.points[i].section != 2 for i in (ramp_first_index, ramp_stop_index)):
                raise ValueError('second ramp waypoint must belong to section 2')
            self.ramp_configured = True
        for p in route.points:
            if p.index < initial_index:
                continue
            is_ramp = p.index == ramp_stop_index
            if is_ramp or (p.stop_line and p.section in (4, 6, 8, 11)):
                target_s = route.s[p.index]-(1.0 if is_ramp else 2.0)-front_offset_m
                if target_s < route.s[initial_index]:
                    raise ValueError('selected route begins after a required stop target')
                self.targets.append((p.index, p.section, target_s))
        accel = [p.index for p in route.points if p.section == 9]
        self.accel_start = self.accel_end = None
        if accel and (acceleration_start_index >= 0 or acceleration_end_index >= 0):
            start = acceleration_start_index
            end = acceleration_end_index
            if start not in accel or end not in accel or start >= end:
                raise ValueError('acceleration indices must be ordered within section 9')
            self.accel_start, self.accel_end = route.s[start], route.s[end]

    def snapshot(self, progress):
        p = self.route.points[self.route.index(progress)]
        if self.pending is None:
            self.pending = next((t for t in self.targets if t[0] not in self.completed), None)
        target = self.pending
        # Expose the next stop before its section boundary; once its target is
        # reached, prevent a position jump from switching past an unserved stop.
        section = target[1] if target and progress >= target[2] else p.section
        visible = target if target and (section == target[1] or progress >= target[2]-3.) else None
        return dict(section=section, route_index=p.index,
                    stop_index=visible[0] if visible else -1,
                    stop_section=visible[1] if visible else -1,
                    stop_remaining_m=visible[2]-progress if visible else None,
                    ramp_configured=self.ramp_configured,
                    acceleration_configured=self.accel_start is not None,
                    acceleration_active=bool(self.accel_start is not None and
                                             self.accel_start <= progress < self.accel_end))

    def release(self, index):
        if self.pending is None or self.pending[0] != index:
            return False
        self.completed.add(index)
        self.pending = None
        return True
