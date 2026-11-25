"""A small simulated hotend for exercising control_mpc without hardware.

The plant is a lumped thermal model (heater -> block -> sensor) with a part
cooling fan and filament heating. Simulated time replaces the reactor clock,
so calibration runs that take minutes on a printer finish in milliseconds.
"""

from __future__ import annotations

import contextlib
import math
import random

from klippy.extras import control_mpc

SIM_DT = 0.01
REPORT_TIME = 0.300
MAX_HEAT_TIME = 3.0
MIN_PWM_CHANGE_RATIO = 0.05


class Plant:
    def __init__(
        self,
        heater_w=50.0,
        block_c=18.0,  # J/K
        beta=0.25,  # 1/s, sensor responsiveness
        h0=0.10,  # W/K, no fan
        h_fan=0.10,  # W/K, extra at full fan
        heater_c=0.0,  # J/K, 0 disables the heater cartridge lag
        heater_g=2.5,  # W/K, heater cartridge to block
        ptc_alpha=0.0,  # fractional power drop per K above 200C
        filament_k=0.0054,  # J/mm/K
        ambient=25.0,
        noise=0.0,
        seed=1,
    ):
        self.heater_w = heater_w
        self.block_c = block_c
        self.beta = beta
        self.h0 = h0
        self.h_fan = h_fan
        self.heater_c = heater_c
        self.heater_g = heater_g
        self.ptc_alpha = ptc_alpha
        self.filament_k = filament_k
        self.ambient = ambient
        self.noise = noise
        self.rng = random.Random(seed)
        self.heater_temp = self.block_temp = self.sensor_temp = ambient

    def heater_power(self, temp):
        return self.heater_w * (1.0 - self.ptc_alpha * (temp - 200.0))

    def transfer(self, fan):
        return self.h0 + self.h_fan * fan

    def step(self, dt, duty, fan, extrude_speed):
        p_in = duty * self.heater_power(self.block_temp)
        if self.heater_c > 0.0:
            q_block = self.heater_g * (self.heater_temp - self.block_temp)
            self.heater_temp += (p_in - q_block) * dt / self.heater_c
        else:
            q_block = p_in
        delta = self.block_temp - self.ambient
        loss = delta * self.transfer(fan)
        loss += delta * max(0.0, extrude_speed) * self.filament_k
        self.block_temp += (q_block - loss) * dt / self.block_c
        self.sensor_temp += (
            self.beta * (self.block_temp - self.sensor_temp) * dt
        )

    def measure(self):
        return self.sensor_temp + self.rng.gauss(0.0, self.noise)


class SimError(Exception):
    pass


class FakeFan:
    def __init__(self, max_power=1.0):
        self.max_power = max_power
        self.value = 0.0
        self.requests = []

    def set_speed(self, value, print_time=None):
        self.value = value
        self.requests.append(value)

    def get_status(self, eventtime):
        return {
            "power": self.value * self.max_power,
            "value": self.value,
            "speed": self.value * self.max_power,
            "rpm": None,
        }


class FakeExtruder:
    def __init__(self, sim, heater):
        self.sim = sim
        self.heater = heater
        self.max_e_dist = 50.0

    def get_heater(self):
        return self.heater

    def find_past_position(self, print_time):
        return self.sim.extruder_position(print_time)


class FakeToolhead:
    def __init__(self, extruder):
        self.extruder = extruder

    def get_extruder(self):
        return self.extruder


class FakeGCode:
    def register_mux_command(self, *args, **kwargs):
        pass


class FakeConfigfile:
    def __init__(self):
        self.values = {}

    def set(self, section, option, value):
        self.values[option] = value


class FakeReactor:
    def __init__(self, sim):
        self.sim = sim

    def monotonic(self):
        return self.sim.time


class FakePrinter:
    command_error = SimError

    def __init__(self, sim):
        self.reactor = FakeReactor(sim)
        self.objects = {
            "gcode": FakeGCode(),
            "configfile": FakeConfigfile(),
        }

    def get_reactor(self):
        return self.reactor

    def lookup_object(self, name, default=None):
        return self.objects.get(name, default)


class FakeHeater:
    """Mirrors the parts of heaters.Heater that the controllers touch."""

    def __init__(self, sim, max_power=1.0, name="extruder"):
        self.sim = sim
        self.printer = sim.printer
        self.reactor = sim.printer.reactor
        self.name = name
        self.max_power = max_power
        self.min_pwm_change = max_power * MIN_PWM_CHANGE_RATIO
        self.pwm_delay = REPORT_TIME
        self.last_temp = self.smoothed_temp = self.target_temp = 0.0
        self.last_temp_time = 0.0
        self.next_pwm_time = 0.0
        self.last_pwm_value = 0.0
        self.control = None

    def get_name(self):
        return self.name

    def get_max_power(self):
        return self.max_power

    def get_pwm_delay(self):
        return self.pwm_delay

    def set_pwm(self, read_time, value):
        if self.target_temp <= 0.0:
            value = 0.0
        if (read_time < self.next_pwm_time or not self.last_pwm_value) and abs(
            value - self.last_pwm_value
        ) < self.min_pwm_change:
            return
        pwm_time = read_time + self.pwm_delay
        self.next_pwm_time = (
            pwm_time + MAX_HEAT_TIME - (3.0 * self.pwm_delay + 0.001)
        )
        self.last_pwm_value = value
        self.sim.pwm_queue.append((pwm_time, value))

    def temperature_callback(self, read_time, temp):
        time_diff = read_time - self.last_temp_time
        self.last_temp = temp
        self.last_temp_time = read_time
        self.control.temperature_update(read_time, temp, self.target_temp)
        self.smoothed_temp += (temp - self.smoothed_temp) * min(time_diff, 1.0)

    def get_temp(self, eventtime):
        return self.smoothed_temp, self.target_temp

    def set_temp(self, degrees):
        if degrees != 0.0 and hasattr(self.control, "check_valid"):
            self.control.check_valid()
        self.target_temp = degrees

    def alter_target(self, target_temp):
        self.target_temp = target_temp

    def get_status(self, eventtime):
        status = {
            "temperature": self.smoothed_temp,
            "target": self.target_temp,
            "power": self.last_pwm_value,
        }
        if hasattr(self.control, "get_status"):
            status["control_stats"] = self.control.get_status(eventtime)
        return status

    def check_not_calibrating(self):
        pass

    @contextlib.contextmanager
    def calibration(self):
        heater = self

        class Calibration:
            def set_control(self, control, keep_target=True):
                old_control = heater.control
                heater.control = control
                return old_control

        yield Calibration()

    def get_control(self):
        return self.control


class FakeGCmd:
    error = SimError

    def __init__(self, sim, params=None):
        self.sim = sim
        self.params = params or {}
        self.responses = []
        self._commandline = "SIM"

    def get(self, name, default=None, **kwargs):
        return self.params.get(name, default)

    def get_float(self, name, default=None, **kwargs):
        value = self.params.get(name, default)
        return None if value is None else float(value)

    def get_int(self, name, default=None, **kwargs):
        value = self.params.get(name, default)
        return None if value is None else int(value)

    def get_command(self):
        return "MPC_CALIBRATE"

    def respond_info(self, msg):
        self.responses.append(msg)

    def wait_while(self, condition_cb, error_on_cancel=True, interval=1.0):
        start = self.sim.time
        while condition_cb(self.sim.time):
            self.sim.advance(interval)
            if self.sim.time - start > self.sim.wait_timeout:
                raise SimError("simulated wait did not finish")


class Sim:
    def __init__(self, plant=None, fan=None, flow=None, max_power=1.0):
        self.plant = plant or Plant()
        self.time = 10.0
        self.wait_timeout = 3600.0
        self.printer = FakePrinter(self)
        self.heater = FakeHeater(self, max_power)
        self.fan = fan
        self.flow = flow or (lambda t: 0.0)  # filament mm/s
        self.pwm_queue = []
        self.duty = 0.0
        self.next_sample = self.time + REPORT_TIME
        self.positions = [0.0]
        self.position_start = self.time
        self.log = []  # (time, sensor, block, applied duty)
        self.printer.objects["toolhead"] = FakeToolhead(
            FakeExtruder(self, self.heater)
        )

    def profile(self, **overrides):
        profile = {
            "name": "default",
            "control": "mpc",
            "block_heat_capacity": None,
            "ambient_transfer": None,
            "sensor_responsiveness": None,
            "target_reach_time": 2.0,
            "heater_power": self.plant.heater_w,
            "smoothing": 0.83,
            "min_ambient_change": 1.0,
            "steady_state_rate": 0.5,
            "filament_diameter": 1.75,
            "filament_density": 1.2,
            "filament_heat_capacity": 1.8,
            "maximum_retract": 2.0,
            "filament_temp_src": (control_mpc.FILAMENT_TEMP_SRC_AMBIENT,),
            "ambient_temp_sensor": None,
            "cooling_fan": self.fan,
            "fan_ambient_transfer": [],
        }
        profile.update(overrides)
        return profile

    def make_control(self, **overrides):
        control = control_mpc.ControlMPC(
            self.profile(**overrides),
            self.heater,
            load_clean=True,
            register=False,
        )
        self.heater.control = control
        return control

    def extruder_position(self, print_time):
        if print_time <= self.position_start:
            return 0.0
        index = (print_time - self.position_start) / SIM_DT
        while len(self.positions) < int(index) + 2:
            t = self.position_start + (len(self.positions) - 1) * SIM_DT
            self.positions.append(self.positions[-1] + self.flow(t) * SIM_DT)
        low = int(index)
        frac = index - low
        return self.positions[low] * (1 - frac) + self.positions[low + 1] * frac

    def advance(self, duration):
        end = self.time + duration - 1e-9
        while self.time < end:
            while self.pwm_queue and self.pwm_queue[0][0] <= self.time + 1e-9:
                self.duty = self.pwm_queue.pop(0)[1]
            fan = self.fan.value * self.fan.max_power if self.fan else 0.0
            self.plant.step(SIM_DT, self.duty, fan, self.flow(self.time))
            self.time += SIM_DT
            if self.time >= self.next_sample - 1e-9:
                self.heater.temperature_callback(
                    self.next_sample, self.plant.measure()
                )
                self.next_sample += REPORT_TIME
                self.log.append(
                    (
                        self.time,
                        self.plant.sensor_temp,
                        self.plant.block_temp,
                        self.duty,
                    )
                )

    def calibrate(self, params=None, **overrides):
        """Run MPC_CALIBRATE; returns (saved config values, gcmd)."""
        control = self.make_control(**overrides)
        gcmd = FakeGCmd(self, params)
        control_mpc.MpcCalibrate(self.printer, self.heater, control).run(gcmd)
        return self.printer.objects["configfile"].values, gcmd


def filament_speed(volumetric_flow, diameter=1.75):
    """mm^3/s to filament mm/s"""
    return volumetric_flow / (math.pi * (diameter / 2.0) ** 2)
