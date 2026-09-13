"""Fail-closed command decisions for the incremental MCU_PAD v6 protocol."""

import math


class PadV6Autonomy:
    def __init__(self, center_adc=484, adc_per_degree=18.0,
                 minimum_adc=158, maximum_adc=798):
        self.center = int(center_adc)
        self.adc_per_degree = float(adc_per_degree)
        self.minimum = int(minimum_adc)
        self.maximum = int(maximum_adc)
        if not self.minimum < self.center < self.maximum:
            raise ValueError("ADC limits must contain the center")
        if not math.isfinite(self.adc_per_degree) or self.adc_per_degree <= 0:
            raise ValueError("adc_per_degree must be positive")
        self.armed = False
        self.moving = False
        self.last_adc = None

    def arm(self, enabled):
        self.armed = bool(enabled)
        self.moving = False
        self.last_adc = None
        return [b"x"]

    def update(self, valid, target_speed_mps, steering_deg, drive_stage):
        finite = all(math.isfinite(float(v)) for v in
                     (target_speed_mps, steering_deg, drive_stage))
        if (not self.armed or not valid or not finite or
                float(target_speed_mps) <= 0 or int(drive_stage) <= 0):
            self.moving = False
            self.last_adc = None
            return [b"x"]
        adc = int(round(self.center-float(steering_deg)*self.adc_per_degree))
        adc = max(self.minimum, min(self.maximum, adc))
        packets = []
        if not self.moving:
            packets.append(b"w")  # FIELD/ODOM firmware: zero -> breakaway PWM 40
            self.moving = True
        if adc != self.last_adc:
            packets.append(f"G{adc}\n".encode("ascii"))
            self.last_adc = adc
        return packets
