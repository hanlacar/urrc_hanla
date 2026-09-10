"""Decision-layer pitch smoothing; residual measures pitch shake, not road roughness."""
import math


class RampPitchFilter:
    def __init__(self, tau_sec=.25, rough_tau_sec=.3, rough_limit_deg=1.0,
                 outlier_limit_deg=45.0):
        values = (tau_sec, rough_tau_sec, rough_limit_deg, outlier_limit_deg)
        if not all(math.isfinite(v) and v > 0 for v in values):
            raise ValueError('Ramp filter parameters must be finite and positive')
        self.tau, self.rough_tau, self.rough_limit, self.outlier_limit = values
        self.reset()

    def reset(self):
        self.pitch = None
        self.stamp = None
        self.energy = 0.0
        self.roughness = 0.0
        self.valid = False

    def update(self, pitch, stamp, valid=True):
        if (not valid or not math.isfinite(pitch) or not math.isfinite(stamp)
                or abs(pitch) > self.outlier_limit):
            self.reset()
            return False
        if self.stamp is None:
            self.pitch, self.stamp = pitch, stamp
            self.valid = True
            return True
        dt = stamp-self.stamp
        if dt <= 0:
            self.valid = False
            return False
        a = -math.expm1(-dt/self.tau)
        self.pitch += a*(pitch-self.pitch)
        residual = pitch-self.pitch
        b = -math.expm1(-dt/self.rough_tau)
        self.energy += b*(residual*residual-self.energy)
        self.roughness = math.sqrt(max(0.0, self.energy))
        self.stamp = stamp
        self.valid = True
        return self.roughness <= self.rough_limit
