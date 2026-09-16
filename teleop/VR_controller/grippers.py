"""Dual gripper PD and binary feedback; independent of ROS and XR."""
import math
from pathlib import Path
import yaml

CONFIG_PATH = Path(__file__).resolve().parent / 'config/grippers.yaml'


def validate_side(config):
    for key in ('kp', 'kd', 'motion_time_s', 'torque_limit_nm'):
        value = float(config[key])
        if not math.isfinite(value) or value < 0 or (key != 'kd' and value == 0):
            raise ValueError(f'Invalid gripper {key}')
    if config.get('calibrated'):
        opened, closed = float(config['open_deg']), float(config['closed_deg'])
        if not all(map(math.isfinite, (opened, closed))) or abs(closed-opened) < 5:
            raise ValueError('Calibrated endpoints must be finite and at least 5 degrees apart')
    return config


class Gripper:
    def __init__(self, config):
        self.config = validate_side(config)
        self.ready = False
        self.measured = self.command = -1
        self.q = self.dq = 0.0
        self.start = self.end = 0.0
        self.t0 = None

    def feedback(self, q, dq):
        if not self.config.get('calibrated'):
            return
        self.q, self.dq = q, dq
        fraction = (q-self.config['open_deg']) / (self.config['closed_deg']-self.config['open_deg'])
        if not self.ready:
            self.measured = int(fraction >= .5)
            self.command = self.measured
            self.start = self.end = q  # Hold current position until an explicit toggle.
            self.t0 = None
            self.ready = True
        elif fraction >= .6:
            self.measured = 1
        elif fraction <= .4:
            self.measured = 0

    def toggle(self, now):
        if not self.ready:
            return False
        self.command = 1-self.command
        self.start = self.q
        self.end = self.config['closed_deg' if self.command else 'open_deg']
        self.t0 = now
        return True

    def torque(self, now):
        if not self.ready:
            return 0.0
        s = 1.0 if self.t0 is None else max(0.0, min(1.0, (now-self.t0)/self.config['motion_time_s']))
        qdes = self.start+(self.end-self.start)*(10*s**3-15*s**4+6*s**5)
        torque = self.config['kp']*math.radians(qdes-self.q)-self.config['kd']*self.dq
        limit = self.config['torque_limit_nm']
        return max(-limit, min(limit, torque))

    def invalidate(self):
        self.ready = False
        self.measured = self.command = -1


class DualGrippers:
    def __init__(self, config):
        self.sides = {side: Gripper(config[side]) for side in ('left','right')}
        self.timeout = float(config.get('feedback_timeout_s', .2))
        if not math.isfinite(self.timeout) or self.timeout <= 0:
            raise ValueError('feedback_timeout_s must be positive')
        self.last_feedback = None

    def feedback(self, data, now):
        if len(data) != 8 or not all(math.isfinite(float(data[i])) for i in (0,1,4,5)):
            self.last_feedback = None
            for side in self.sides.values(): side.invalidate()
            return
        if self.last_feedback is not None and now-self.last_feedback > self.timeout:
            for side in self.sides.values(): side.invalidate()
        for name,index in [('left',0),('right',4)]:
            self.sides[name].feedback(float(data[index]),float(data[index+1]))
        self.last_feedback = now

    def step(self, now, toggles):
        if self.last_feedback is None or now-self.last_feedback > self.timeout:
            for side in self.sides.values(): side.invalidate()
        for name, toggle in toggles.items():
            if toggle: self.sides[name].toggle(now)
        torque = [self.sides[name].torque(now) for name in ('left','right')]
        states = [self.sides[name].measured for name in ('left','right')]
        states += [self.sides[name].command for name in ('left','right')]
        return torque, states


def load_config(path=CONFIG_PATH):
    return yaml.safe_load(Path(path).read_text())
