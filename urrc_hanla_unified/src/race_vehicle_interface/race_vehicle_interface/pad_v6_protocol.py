"""Pure command state for MCU_PAD_SINGLE_v6_AUTO's FIELD/ODOM protocol."""


class PadV6CommandState:
    def __init__(self, center_adc=484, minimum_adc=158, maximum_adc=798,
                 steering_step_adc=64):
        values = (int(minimum_adc), int(center_adc), int(maximum_adc))
        if not values[0] < values[1] < values[2]:
            raise ValueError("steering ADC limits must contain the center")
        if int(steering_step_adc) <= 0:
            raise ValueError("steering_step_adc must be positive")
        self.minimum_adc, self.center_adc, self.maximum_adc = values
        self.step = int(steering_step_adc)
        self.target_adc = self.center_adc
        self.armed = False
        self.estimated_pwm = 0

    def stop(self):
        self.estimated_pwm = 0
        return b"x"

    def toggle_arm(self):
        self.armed = not self.armed
        self.estimated_pwm = 0
        return self.stop()

    def key(self, key):
        key = str(key).lower()
        if key == "e":
            return self.toggle_arm()
        if key in ("x", " "):
            return self.stop()
        if key == "w":
            if not self.armed:
                return None
            self.estimated_pwm = (40 if self.estimated_pwm == 0 else
                                  min(100, self.estimated_pwm + 10))
            return b"w"
        if key == "s":
            if not self.armed:
                return None
            self.estimated_pwm = (-40 if self.estimated_pwm == 0 else
                                  max(-100, self.estimated_pwm - 10))
            return b"s"
        if key == "a":
            self.target_adc = min(self.maximum_adc, self.target_adc + self.step)
            return f"G{self.target_adc}\n".encode("ascii")
        if key == "d":
            self.target_adc = max(self.minimum_adc, self.target_adc - self.step)
            return f"G{self.target_adc}\n".encode("ascii")
        if key == "c":
            self.target_adc = self.center_adc
            return f"G{self.center_adc}\n".encode("ascii")
        if key == "i":
            return b"S\n"
        return None
