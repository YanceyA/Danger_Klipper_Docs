import logging
import math
import time

AMBIENT_TEMP = 25.0
PIN_MIN_TIME = 0.100

# Time the heater stays off after the calibration heat-up. The way the
# sensor overshoots and turns around is what identifies the sensor lag.
HEATUP_COAST_TIME = 10.0
FIT_MAX_ITERATIONS = 400
FIT_MIN_SAMPLES = 20
# Holding the temperature before measuring the heat loss: the temperature
# has to stay within HOLD_TOLERANCE for HOLD_STABLE_TIME.
HOLD_TOLERANCE = 0.5
HOLD_STABLE_TIME = 20.0
HOLD_MIN_TIME = 30.0
HOLD_TIMEOUT = 180.0
# The fan sweep backs off when the heater can no longer hold the
# temperature. A fan step that failed is retried in smaller steps, down to
# FAN_SWEEP_MIN_STEP.
FAN_SWEEP_MAX_SAG = 8.0
FAN_SWEEP_MIN_STEP = 0.2
# and it ends once the heat loss gets this close to the maximum heater power
FAN_SWEEP_MAX_POWER = 0.95
# Limits for a heater power table extended beyond its listed temperatures
HEATER_POWER_MIN_RATIO = 0.5
HEATER_POWER_MAX_RATIO = 2.0

FILAMENT_TEMP_SRC_AMBIENT = "ambient"
FILAMENT_TEMP_SRC_FIXED = "fixed"
FILAMENT_TEMP_SRC_SENSOR = "sensor"


def parse_heater_power(entries):
    """Parse heater_power into a list of (temperature, watts).

    Either a single power, or "temperature:power" pairs for heaters whose
    power changes with temperature (PTC heaters).
    """
    entries = [str(entry).strip() for entry in entries]
    entries = [entry for entry in entries if entry]
    if len(entries) == 1 and ":" not in entries[0]:
        table = [(0.0, float(entries[0]))]
    else:
        table = []
        for entry in entries:
            temp, sep, power = entry.partition(":")
            if not sep:
                raise ValueError(
                    "expected a single power or temperature:power pairs"
                )
            table.append((float(temp), float(power)))
        table.sort()
    if not table:
        raise ValueError("no heater power given")
    if any(power <= 0.0 for _, power in table):
        raise ValueError("heater power must be above 0")
    for idx in range(1, len(table)):
        if table[idx][0] == table[idx - 1][0]:
            raise ValueError("duplicate temperature %g" % (table[idx][0],))
    return table


def _heater_power_table(heater_power):
    if isinstance(heater_power, (int, float)):
        return [(0.0, float(heater_power))]
    return heater_power


def heater_power_at(table, temp):
    """Heater power at full duty with the heater block at `temp`"""
    if len(table) == 1:
        return table[0][1]
    # Interpolate; beyond the listed temperatures the nearest segment is
    # extended, within sane limits.
    idx = 1
    while idx < len(table) - 1 and temp > table[idx][0]:
        idx += 1
    low_temp, low_power = table[idx - 1]
    high_temp, high_power = table[idx]
    power = low_power + (high_power - low_power) * (temp - low_temp) / (
        high_temp - low_temp
    )
    powers = [entry[1] for entry in table]
    return max(
        HEATER_POWER_MIN_RATIO * min(powers),
        min(HEATER_POWER_MAX_RATIO * max(powers), power),
    )


def parse_fan_ambient_transfer(entries):
    """Parse a fan heat-loss table into a list of (fan speed, W/K).

    Entries are either plain values, evenly spaced from 0% to 100% fan, or
    "speed:value" pairs with the fan speed given as 0..1.
    """
    entries = [str(entry).strip() for entry in entries]
    entries = [entry for entry in entries if entry]
    if not entries:
        return []
    if not any(":" in entry for entry in entries):
        values = [float(entry) for entry in entries]
        last = max(1, len(values) - 1)
        return [(idx / last, value) for idx, value in enumerate(values)]
    table = []
    for entry in entries:
        speed, sep, value = entry.partition(":")
        if not sep:
            raise ValueError("cannot mix plain values and speed:value pairs")
        speed, value = float(speed), float(value)
        if speed < 0.0 or speed > 1.0:
            raise ValueError("fan speeds must be between 0 and 1")
        table.append((speed, value))
    table.sort()
    for idx in range(1, len(table)):
        if table[idx][0] == table[idx - 1][0]:
            raise ValueError("duplicate fan speed %g" % (table[idx][0],))
    return table


def format_fan_ambient_transfer(table):
    count = len(table)
    if count > 1 and all(
        abs(speed - idx / (count - 1)) < 1e-6
        for idx, (speed, _) in enumerate(table)
    ):
        # Evenly spaced from 0% to 100%: the plain list is sufficient
        return ", ".join(f"{value:.6g}" for _, value in table)
    return ", ".join(f"{speed:.4g}:{value:.6g}" for speed, value in table)


def interpolate_fan_ambient_transfer(table, fan_speed):
    if fan_speed <= table[0][0]:
        return table[0][1]
    for idx in range(1, len(table)):
        speed, value = table[idx]
        if fan_speed <= speed:
            prev_speed, prev_value = table[idx - 1]
            frac = (fan_speed - prev_speed) / (speed - prev_speed)
            return prev_value + (value - prev_value) * frac
    return table[-1][1]


class ControlMPC:
    def __init__(self, profile, heater, load_clean=False, register=True):
        self.profile = profile
        self._load_profile()
        self.heater = heater

        self.want_ambient_refresh = self.ambient_sensor is not None
        self.state_block_temp = (
            AMBIENT_TEMP if load_clean else self._heater_temp()
        )
        self.state_sensor_temp = self.state_block_temp
        self.state_ambient_temp = AMBIENT_TEMP
        self._update_heater_power()

        self.last_power = 0.0
        self.last_loss_ambient = 0.0
        self.last_loss_filament = 0.0
        self.last_time = 0.0
        self.last_temp_time = 0.0

        self.printer = heater.printer
        self.toolhead = None

        if not register:
            return

        gcode = self.printer.lookup_object("gcode")
        gcode.register_mux_command(
            "MPC_CALIBRATE",
            "HEATER",
            heater.get_name(),
            self.cmd_MPC_CALIBRATE,
            desc=self.cmd_MPC_CALIBRATE_help,
        )
        gcode.register_mux_command(
            "MPC_SET",
            "HEATER",
            heater.get_name(),
            self.cmd_MPC_SET,
            desc=self.cmd_MPC_SET_help,
        )

    cmd_MPC_SET_help = "Set MPC parameter"

    def cmd_MPC_SET(self, gcmd):
        self.heater.check_not_calibrating()
        self.const_filament_diameter = gcmd.get_float(
            "FILAMENT_DIAMETER", self.const_filament_diameter
        )
        self.const_filament_density = gcmd.get_float(
            "FILAMENT_DENSITY", self.const_filament_density
        )
        self.const_filament_heat_capacity = gcmd.get_float(
            "FILAMENT_HEAT_CAPACITY", self.const_filament_heat_capacity
        )

        self.const_block_heat_capacity = gcmd.get_float(
            "BLOCK_HEAT_CAPACITY", self.const_block_heat_capacity
        )
        self.const_sensor_responsiveness = gcmd.get_float(
            "SENSOR_RESPONSIVENESS", self.const_sensor_responsiveness
        )
        self.const_ambient_transfer = gcmd.get_float(
            "AMBIENT_TRANSFER", self.const_ambient_transfer
        )

        if gcmd.get("FAN_AMBIENT_TRANSFER", None):
            try:
                self.const_fan_ambient_transfer = parse_fan_ambient_transfer(
                    gcmd.get("FAN_AMBIENT_TRANSFER").split(",")
                )
            except ValueError:
                raise gcmd.error(
                    f"Error on '{gcmd._commandline}': unable to parse FAN_AMBIENT_TRANSFER\n"
                    "Must be a comma-separated list of values ('0.05,0.07,0.08')"
                    " or of fan speed:value pairs ('0:0.05,0.3:0.07,1:0.08')"
                )

        temp = gcmd.get("FILAMENT_TEMP", None)
        if temp is not None:
            temp = temp.lower().strip()
            if temp == "sensor":
                self.filament_temp_src = (FILAMENT_TEMP_SRC_SENSOR,)
            elif temp == "ambient":
                self.filament_temp_src = (FILAMENT_TEMP_SRC_AMBIENT,)
            else:
                try:
                    value = float(temp)
                except ValueError:
                    raise gcmd.error(
                        f"Error on '{gcmd._commandline}': unable to parse FILAMENT_TEMP\n"
                        "Valid options are 'sensor', 'ambient', or number."
                    )
                self.filament_temp_src = (FILAMENT_TEMP_SRC_FIXED, value)

        self._update_filament_const()

    cmd_MPC_CALIBRATE_help = "Run MPC calibration"

    def cmd_MPC_CALIBRATE(self, gcmd):
        cal = MpcCalibrate(self.printer, self.heater, self)
        cal.run(gcmd)

    # Helpers

    def _heater_temp(self):
        return self.heater.get_temp(self.heater.reactor.monotonic())[0]

    def _load_profile(self):
        self.const_block_heat_capacity = self.profile["block_heat_capacity"]
        self.const_ambient_transfer = self.profile["ambient_transfer"]
        self.const_target_reach_time = self.profile["target_reach_time"]
        self.const_heater_power = _heater_power_table(
            self.profile["heater_power"]
        )
        self.const_smoothing = self.profile["smoothing"]
        self.const_sensor_responsiveness = self.profile["sensor_responsiveness"]
        self.const_min_ambient_change = self.profile["min_ambient_change"]
        self.const_steady_state_rate = self.profile["steady_state_rate"]
        self.const_filament_diameter = self.profile["filament_diameter"]
        self.const_filament_density = self.profile["filament_density"]
        self.const_filament_heat_capacity = self.profile[
            "filament_heat_capacity"
        ]
        self.const_maximum_retract = self.profile["maximum_retract"]
        self.filament_temp_src = self.profile["filament_temp_src"]
        self._update_filament_const()
        self.ambient_sensor = self.profile["ambient_temp_sensor"]
        self.cooling_fan = self.profile["cooling_fan"]
        self.const_fan_ambient_transfer = self.profile["fan_ambient_transfer"]

    def is_valid(self):
        return (
            self.const_block_heat_capacity is not None
            and self.const_ambient_transfer is not None
            and self.const_sensor_responsiveness is not None
        )

    def check_valid(self):
        if self.is_valid():
            return
        name = self.heater.get_name()
        raise self.printer.command_error(
            f"Cannot activate '{name}' as MPC control is not fully configured.\n\n"
            f"Run 'MPC_CALIBRATE' or ensure 'block_heat_capacity', 'sensor_responsiveness', and "
            f"'ambient_transfer' settings are defined for '{name}'."
        )

    def heater_power(self, temp):
        return heater_power_at(self.const_heater_power, temp)

    def _update_heater_power(self):
        # The power of a PTC heater depends on its temperature
        self.full_heater_power = self.heater_power(self.state_block_temp)
        self.heater_max_power = (
            self.heater.get_max_power() * self.full_heater_power
        )

    def _update_filament_const(self):
        radius = self.const_filament_diameter / 2.0
        self.const_filament_cross_section_heat_capacity = (
            (radius * radius)  # mm^2
            * math.pi  # 1
            / 1000.0  # mm^3 => cm^3
            * self.const_filament_density  # g/cm^3
            * self.const_filament_heat_capacity  # J/g/K
        )

    # Control interface

    def temperature_update(self, read_time, temp, target_temp):
        if not self.is_valid():
            self.heater.set_pwm(read_time, 0.0)
            return

        dt = read_time - self.last_temp_time
        if self.last_temp_time == 0.0 or dt < 0.0 or dt > 1.0:
            dt = 0.1

        # Extruder position
        extrude_speed_prev = 0.0
        extrude_speed_next = 0.0
        if target_temp != 0.0:
            if self.toolhead is None:
                self.toolhead = self.printer.lookup_object("toolhead")
            if self.toolhead is not None:
                extruder = self.toolhead.get_extruder()
                if (
                    hasattr(extruder, "find_past_position")
                    and extruder.get_heater() == self.heater
                ):
                    pos = extruder.find_past_position(read_time)

                    pos_prev = extruder.find_past_position(read_time - dt)
                    pos_moved = max(-self.const_maximum_retract, pos - pos_prev)
                    extrude_speed_prev = pos_moved / dt

                    pos_next = extruder.find_past_position(read_time + dt)
                    pos_move = max(-self.const_maximum_retract, pos_next - pos)
                    extrude_speed_next = pos_move / dt

        # Modulate ambient transfer coefficient with fan speed
        ambient_transfer = self.const_ambient_transfer
        if self.cooling_fan and len(self.const_fan_ambient_transfer) > 1:
            # Use the requested fan value (0..1), which is what calibration
            # indexes the table by. The reported "speed" is scaled by the
            # fan's max_power and would read the wrong part of the curve.
            fan_speed = max(
                0.0, min(1.0, self.cooling_fan.get_status(read_time)["value"])
            )
            ambient_transfer = interpolate_fan_ambient_transfer(
                self.const_fan_ambient_transfer, fan_speed
            )

        # Simulate

        # Expected power by heating at last power setting
        expected_heating = self.last_power
        # Expected power from block to ambient
        block_ambient_delta = self.state_block_temp - self.state_ambient_temp
        expected_ambient_transfer = block_ambient_delta * ambient_transfer
        block_filament_delta = self.state_block_temp - self.filament_temp(
            read_time, self.state_ambient_temp
        )
        expected_filament_transfer = (
            block_filament_delta
            * extrude_speed_prev
            * self.const_filament_cross_section_heat_capacity
        )

        # Expected block dT since last period
        expected_block_dT = (
            (
                expected_heating
                - expected_ambient_transfer
                - expected_filament_transfer
            )
            * dt
            / self.const_block_heat_capacity
        )
        self.state_block_temp += expected_block_dT

        # Expected sensor dT since last period
        expected_sensor_dT = (
            (self.state_block_temp - self.state_sensor_temp)
            * self.const_sensor_responsiveness
            * dt
        )
        self.state_sensor_temp += expected_sensor_dT

        # Correct

        smoothing = 1 - (1 - self.const_smoothing) ** dt
        adjustment_dT = (temp - self.state_sensor_temp) * smoothing
        self.state_block_temp += adjustment_dT
        self.state_sensor_temp += adjustment_dT

        if self.want_ambient_refresh:
            temp = self.ambient_sensor.get_temp(read_time)[0]
            if temp != 0.0:
                self.state_ambient_temp = temp
                self.want_ambient_refresh = False
        # Only correct ambient when close to steady state: the output power
        # is not clipped, or the asymptotic temperature is reached.
        # (last_power is in watts, so compare against the maximum power)
        if (0.0 < self.last_power < self.heater_max_power) or abs(
            expected_block_dT + adjustment_dT
        ) < self.const_steady_state_rate * dt:
            if adjustment_dT > 0.0:
                ambient_delta = max(
                    adjustment_dT, self.const_min_ambient_change * dt
                )
            else:
                ambient_delta = min(
                    adjustment_dT, -self.const_min_ambient_change * dt
                )
            self.state_ambient_temp += ambient_delta

        # Output

        self._update_heater_power()

        # Amount of power needed to reach the target temperature in the desired time

        heating_power = (
            (target_temp - self.state_block_temp)
            * self.const_block_heat_capacity
            / self.const_target_reach_time
        )
        # Losses (+ = lost from block, - = gained to block)
        block_ambient_delta = self.state_block_temp - self.state_ambient_temp
        loss_ambient = block_ambient_delta * ambient_transfer
        block_filament_delta = self.state_block_temp - self.filament_temp(
            read_time, self.state_ambient_temp
        )
        loss_filament = (
            block_filament_delta
            * extrude_speed_next
            * self.const_filament_cross_section_heat_capacity
        )

        if target_temp != 0.0:
            # The required power is the desired heating power + compensation for all the losses
            power = max(
                0.0,
                min(
                    self.heater_max_power,
                    heating_power + loss_ambient + loss_filament,
                ),
            )
        else:
            power = 0

        duty = power / self.full_heater_power

        # logging.info(
        #     "mpc: [%.3f/%.3f] %.2f => %.2f / %.2f / %.2f = %.2f[%.2f+%.2f+%.2f] / %.2f, dT %.2f, E %.2f=>%.2f",
        #     dt,
        #     smoothing,
        #     temp,
        #     self.state_block_temp,
        #     self.state_sensor_temp,
        #     self.state_ambient_temp,
        #     power,
        #     heating_power,
        #     loss_ambient,
        #     loss_filament,
        #     duty,
        #     adjustment_dT,
        #     extrude_speed_prev,
        #     extrude_speed_next,
        # )

        self.last_power = power
        self.last_loss_ambient = loss_ambient
        self.last_loss_filament = loss_filament
        self.last_temp_time = read_time
        self.heater.set_pwm(read_time, duty)

    def filament_temp(self, read_time, ambient_temp):
        src = self.filament_temp_src
        if src[0] == FILAMENT_TEMP_SRC_FIXED:
            return src[1]
        elif (
            src[0] == FILAMENT_TEMP_SRC_SENSOR
            and self.ambient_sensor is not None
        ):
            return self.ambient_sensor.get_temp(read_time)[0]
        else:
            return ambient_temp

    def check_busy(self, eventtime, smoothed_temp, target_temp):
        return abs(target_temp - smoothed_temp) > 1.0

    def update_smooth_time(self):
        pass

    def get_profile(self):
        return self.profile

    def get_type(self):
        return "mpc"

    def get_status(self, eventtime):
        return {
            "temp_block": self.state_block_temp,
            "temp_sensor": self.state_sensor_temp,
            "temp_ambient": self.state_ambient_temp,
            "power": self.last_power,
            "loss_ambient": self.last_loss_ambient,
            "loss_filament": self.last_loss_filament,
            "filament_temp": self.filament_temp_src,
            "filament_heat_capacity": self.const_filament_heat_capacity,
            "filament_density": self.const_filament_density,
        }


######################################################################
# Model identification
######################################################################


def _applied_duties(samples, pwm_delay):
    """Average heater duty that was in effect between consecutive samples.

    A duty requested at a sample only reaches the heater pwm_delay later.
    """
    times = [sample[0] for sample in samples]
    count = len(samples)
    duties = [0.0]
    last = -1  # last request already in effect at the interval start
    for idx in range(1, count):
        start, end = times[idx - 1], times[idx]
        while last + 1 < count and times[last + 1] + pwm_delay <= start:
            last += 1
        duty = samples[last][2] if last >= 0 else 0.0
        total = 0.0
        pos = start
        nxt = last + 1
        while nxt < count and times[nxt] + pwm_delay < end:
            change = times[nxt] + pwm_delay
            total += duty * (change - pos)
            pos = change
            duty = samples[nxt][2]
            nxt += 1
        total += duty * (end - pos)
        duties.append(total / (end - start) if end > start else duty)
    return duties


def _simulate_model(
    samples,
    duties,
    ambient_temp,
    heater_power,
    block_heat_capacity,
    ambient_transfer,
    sensor_responsiveness,
):
    """Run the thermal model used by ControlMPC over a recorded heat-up.

    Returns the sum of squared sensor errors and the final modelled block
    and sensor temperatures.
    """
    block_temp = sensor_temp = samples[0][1]
    last_time = samples[0][0]
    error = 0.0
    power_table = _heater_power_table(heater_power)
    full_power = power_table[0][1]
    constant_power = len(power_table) == 1
    for idx in range(1, len(samples)):
        read_time, temp, _ = samples[idx]
        dt = read_time - last_time
        last_time = read_time
        if not constant_power:
            full_power = heater_power_at(power_table, block_temp)
        block_temp += (
            (
                duties[idx] * full_power
                - (block_temp - ambient_temp) * ambient_transfer
            )
            * dt
            / block_heat_capacity
        )
        sensor_temp += (block_temp - sensor_temp) * sensor_responsiveness * dt
        diff = sensor_temp - temp
        error += diff * diff
    return error, block_temp, sensor_temp


def _nelder_mead(cost, start, step, max_iterations, tick=None):
    """Minimal Nelder-Mead simplex minimiser"""
    size = len(start)
    points = [list(start)]
    for idx in range(size):
        point = list(start)
        point[idx] += step
        points.append(point)
    values = [cost(point) for point in points]
    for _ in range(max_iterations):
        if tick is not None:
            tick()
        order = sorted(range(size + 1), key=values.__getitem__)
        points = [points[idx] for idx in order]
        values = [values[idx] for idx in order]
        if values[-1] - values[0] <= 1e-9 * (values[0] + 1e-9):
            break
        worst = points[-1]
        center = [
            sum(point[idx] for point in points[:-1]) / size
            for idx in range(size)
        ]
        reflected = [2.0 * center[idx] - worst[idx] for idx in range(size)]
        reflected_value = cost(reflected)
        if reflected_value < values[0]:
            expanded = [
                3.0 * center[idx] - 2.0 * worst[idx] for idx in range(size)
            ]
            expanded_value = cost(expanded)
            if expanded_value < reflected_value:
                points[-1], values[-1] = expanded, expanded_value
            else:
                points[-1], values[-1] = reflected, reflected_value
        elif reflected_value < values[-2]:
            points[-1], values[-1] = reflected, reflected_value
        else:
            contracted = [
                0.5 * (center[idx] + worst[idx]) for idx in range(size)
            ]
            contracted_value = cost(contracted)
            if contracted_value < values[-1]:
                points[-1], values[-1] = contracted, contracted_value
            else:
                best = points[0]
                points = [best] + [
                    [0.5 * (best[idx] + point[idx]) for idx in range(size)]
                    for point in points[1:]
                ]
                values = [values[0]] + [cost(point) for point in points[1:]]
    best = min(range(size + 1), key=values.__getitem__)
    return points[best], values[best]


def fit_thermal_model(
    samples,
    ambient_temp,
    heater_power,
    pwm_delay,
    ambient_transfer=None,
    tick=None,
):
    """Least-squares fit of the MPC thermal model to a recorded heat-up.

    samples is a list of (time, sensor temperature, requested duty). The
    block heat capacity and sensor responsiveness are always fitted; the
    ambient transfer is fitted too unless it is given. tick() is called
    regularly so the caller can keep the reactor running.
    """
    if len(samples) < FIT_MIN_SAMPLES:
        raise ValueError("Not enough temperature samples to fit the model")
    duties = _applied_duties(samples, pwm_delay)
    temps = [sample[1] for sample in samples]
    rise = max(temps) - temps[0]
    # Only for the starting point of the fit; the model uses the full table
    nominal_power = heater_power_at(
        _heater_power_table(heater_power), 0.5 * (temps[0] + max(temps))
    )
    energy = sum(
        duties[idx] * nominal_power * (samples[idx][0] - samples[idx - 1][0])
        for idx in range(1, len(samples))
    )
    if rise < 10.0 or energy <= 0.0:
        raise ValueError("Heater did not heat up enough to fit the model")
    fit_transfer = ambient_transfer is None

    def unpack(params):
        # Parameters are fitted as logarithms to keep them positive
        values = [math.exp(max(-20.0, min(20.0, param))) for param in params]
        if fit_transfer:
            return values
        return values + [ambient_transfer]

    def cost(params):
        capacity, responsiveness, transfer = unpack(params)
        error, _, _ = _simulate_model(
            samples,
            duties,
            ambient_temp,
            heater_power,
            capacity,
            transfer,
            responsiveness,
        )
        return error if math.isfinite(error) else float("inf")

    start = [math.log(0.8 * energy / rise), math.log(0.2)]
    if fit_transfer:
        hottest = max(max(temps) - ambient_temp, 10.0)
        start.append(math.log(nominal_power / (4.0 * hottest)))
    best, error = _nelder_mead(cost, start, 0.3, FIT_MAX_ITERATIONS, tick)
    capacity, responsiveness, transfer = unpack(best)
    if not math.isfinite(error):
        raise ValueError("Unable to fit the thermal model")
    _, block_temp, sensor_temp = _simulate_model(
        samples,
        duties,
        ambient_temp,
        heater_power,
        capacity,
        transfer,
        responsiveness,
    )
    return {
        "block_heat_capacity": capacity,
        "sensor_responsiveness": responsiveness,
        "ambient_transfer": transfer,
        "rms_error": math.sqrt(error / (len(samples) - 1)),
        "block_temp": block_temp,
        "sensor_temp": sensor_temp,
    }


class MpcCalibrate:
    def __init__(self, printer, heater, orig_control):
        self.printer = printer
        self.heater = heater
        self.orig_control = orig_control

    def run(self, gcmd):
        with self.heater.calibration() as calibration:
            self._run(gcmd, calibration)

    def _run(self, gcmd, calibration):
        measure_time = gcmd.get_float("MEASURE_TIME", 15.0, minval=2.0)
        fan_settle_time = gcmd.get_float("FAN_SETTLE_TIME", 10.0, minval=0.0)
        fan_speeds = self.get_fan_speeds(gcmd)
        default_target_temp = (
            90.0 if self.heater.get_name() == "heater_bed" else 200.0
        )
        target_temp = gcmd.get_float("TARGET", default_target_temp, minval=60.0)
        threshold_temp = gcmd.get_float(
            "THRESHOLD", max(50.0, min(100, target_temp - 100.0))
        )

        control = TuningControl(self.heater)
        old_control = calibration.set_control(control)
        try:
            ambient_temp = self.await_ambient(gcmd, control, threshold_temp)
            heater_power = self.orig_control.const_heater_power
            if self.orig_control.cooling_fan is not None:
                # A running fan would end up in the fitted model
                self.orig_control.cooling_fan.set_speed(0.0)
            self.heatup_test(gcmd, target_temp, control)
            # Fit the whole heat-up record. The heater stays off (and is
            # still being recorded) while this runs.
            fit = self.fit_model(gcmd, control.log, ambient_temp, heater_power)
            logging.info("MPC calibration heat-up fit: %s", fit)
            gcmd.respond_info(
                "Model fitted to %d samples, error %.2f degrees RMS"
                % (len(control.log), fit["rms_error"])
            )

            profile = dict(self.orig_control.profile)
            for key in [
                "block_heat_capacity",
                "ambient_transfer",
                "sensor_responsiveness",
            ]:
                profile[key] = fit[key]
            new_control = ControlMPC(profile, self.heater, False, False)
            # Bring the model up to date with what was recorded since
            samples = list(control.log)
            _, block_temp, sensor_temp = _simulate_model(
                samples,
                _applied_duties(samples, self.heater.get_pwm_delay()),
                ambient_temp,
                heater_power,
                fit["block_heat_capacity"],
                fit["ambient_transfer"],
                fit["sensor_responsiveness"],
            )
            new_control.state_block_temp = block_temp
            new_control.state_sensor_temp = sensor_temp
            new_control.state_ambient_temp = ambient_temp
            calibration.set_control(new_control)
            control.logging = False

            transfer_res = self.transfer_test(
                gcmd,
                new_control,
                target_temp,
                ambient_temp,
                fan_speeds,
                fan_settle_time,
                measure_time,
            )
            ambient_transfer = transfer_res["ambient_transfer"]
            fan_ambient_transfer = format_fan_ambient_transfer(
                transfer_res["fan_ambient_transfer"]
            )
            # The heat loss measured while holding the temperature is more
            # accurate than what the heat-up alone can tell; refit the
            # remaining parameters around it.
            fit = self.fit_model(
                gcmd, samples, ambient_temp, heater_power, ambient_transfer
            )
            logging.info("MPC calibration refit: %s", fit)
            block_heat_capacity = fit["block_heat_capacity"]
            sensor_responsiveness = fit["sensor_responsiveness"]

            cfgname = self.heater.get_name()
            gcmd.respond_info(
                f"Finished MPC calibration of heater '{cfgname}'\n"
                "Measured:\n "
                f"  block_heat_capacity={block_heat_capacity:#.6g} [J/K]\n"
                f"  sensor_responsiveness={sensor_responsiveness:#.6g} [K/s/K]\n"
                f"  ambient_transfer={ambient_transfer:#.6g} [W/K]\n"
                f"  fan_ambient_transfer={fan_ambient_transfer} [W/K]\n"
            )

            configfile = self.heater.printer.lookup_object("configfile")
            configfile.set(cfgname, "control", "mpc")
            configfile.set(
                cfgname, "block_heat_capacity", f"{block_heat_capacity:#.6g}"
            )
            configfile.set(
                cfgname,
                "sensor_responsiveness",
                f"{sensor_responsiveness:#.6g}",
            )
            configfile.set(
                cfgname, "ambient_transfer", f"{ambient_transfer:#.6g}"
            )
            configfile.set(
                cfgname,
                "fan_ambient_transfer",
                fan_ambient_transfer,
            )

        except self.printer.command_error as e:
            raise gcmd.error("%s failed: %s" % (gcmd.get_command(), e))
        finally:
            calibration.set_control(old_control)
            self.heater.alter_target(0.0)

    def get_fan_speeds(self, gcmd):
        """Fan speeds (0..1) to measure, always starting with the fan off"""
        speeds = gcmd.get("FAN_SPEEDS", None)
        if speeds is None:
            breakpoints = gcmd.get_int("FAN_BREAKPOINTS", 3, minval=2)
            return [idx / (breakpoints - 1) for idx in range(breakpoints)]
        try:
            speeds = {float(speed) for speed in speeds.split(",")}
        except ValueError:
            raise gcmd.error(
                "Unable to parse FAN_SPEEDS, it must be a comma-separated"
                " list of fan speeds ('0.1,0.2,0.3,0.5,1')"
            )
        if any(speed < 0.0 or speed > 1.0 for speed in speeds):
            raise gcmd.error("FAN_SPEEDS must be between 0 and 1")
        return sorted(speeds | {0.0})

    def wait_stable(self, gcmd, target_temp):
        """
        Wait until the temperature has stayed close to the target for a
        while. Returns False if that did not happen within HOLD_TIMEOUT; the
        heat loss measurement corrects for a temperature that still drifts.
        """
        start = [None]
        in_range_since = [None]
        stable = [False]

        def process(eventtime):
            if start[0] is None:
                start[0] = eventtime
            temp, _ = self.heater.get_temp(eventtime)
            if abs(temp - target_temp) > HOLD_TOLERANCE:
                in_range_since[0] = None
            elif in_range_since[0] is None:
                in_range_since[0] = eventtime
            elapsed = eventtime - start[0]
            if (
                in_range_since[0] is not None
                and eventtime - in_range_since[0] >= HOLD_STABLE_TIME
                and elapsed >= HOLD_MIN_TIME
            ):
                stable[0] = True
                return False
            return elapsed < HOLD_TIMEOUT

        gcmd.wait_while(process, True, 0.5)
        return stable[0]

    def wait_settle(self, max_rate, gcmd):
        last_temp = None
        next_check = None
        samples = []

        def process(eventtime):
            temp, _ = self.heater.get_temp(eventtime)
            samples.append((eventtime, temp))
            while samples[0][0] < eventtime - 10.0:
                samples.pop(0)
            dT = samples[-1][1] - samples[0][1]
            dt = samples[-1][0] - samples[0][0]
            if dt < 8.0:
                return True
            rate = abs(dT / dt)
            return not rate < max_rate

        gcmd.wait_while(process)
        return samples[-1][1]

    def await_ambient(self, gcmd, control, minimum_temp):
        self.heater.alter_target(1.0)  # Turn on fan to increase settling speed
        if self.orig_control.ambient_sensor is not None:
            # If we have an ambient sensor we won't waste time waiting for ambient.
            # We do however need to wait for sub minimum_temp(we pick -5 C relative).
            reported = [False]
            target = minimum_temp - 5

            def process(eventtime):
                temp, _ = self.heater.get_temp(eventtime)
                ret = temp > target
                if ret and not reported[0]:
                    gcmd.respond_info(
                        f"Waiting for heater to drop below {target} degrees Celsius"
                    )
                    reported[0] = True
                return ret

            gcmd.wait_while(process)
            self.heater.alter_target(0.0)
            return self.orig_control.ambient_sensor.get_temp(
                self.heater.reactor.monotonic()
            )[0]

        gcmd.respond_info("Waiting for heater to settle at ambient temperature")
        ambient_temp = self.wait_settle(0.01, gcmd)
        self.heater.alter_target(0.0)
        return ambient_temp

    def heatup_test(self, gcmd, target_temp, control):
        gcmd.respond_info(
            "Performing heatup test, target is %.1f degrees" % (target_temp,)
        )
        interval = self.heater.get_pwm_delay()
        control.log = []
        control.logging = True
        control.set_output(self.heater.get_max_power(), target_temp)

        def heating(eventtime):
            temp, _ = self.heater.get_temp(eventtime)
            return temp < target_temp

        gcmd.wait_while(heating, True, interval)
        # Let the heater coast. Recording continues, also after returning,
        # until the caller switches the tuning controller off.
        control.set_output(0.0, target_temp)
        gcmd.sleep(HEATUP_COAST_TIME)

    def fit_model(
        self, gcmd, samples, ambient_temp, heater_power, ambient_transfer=None
    ):
        last_yield = [time.monotonic()]

        def tick():
            # The fit takes a noticeable amount of CPU time on small hosts;
            # give the reactor a chance to run.
            if time.monotonic() - last_yield[0] > 0.050:
                gcmd.sleep(0.001)
                last_yield[0] = time.monotonic()

        try:
            return fit_thermal_model(
                list(samples),
                ambient_temp,
                heater_power,
                self.heater.get_pwm_delay(),
                ambient_transfer,
                tick,
            )
        except ValueError as e:
            raise self.printer.command_error(str(e))

    def transfer_test(
        self,
        gcmd,
        control,
        target_temp,
        ambient_temp,
        fan_speeds,
        fan_settle_time,
        measure_time,
    ):
        fan = self.orig_control.cooling_fan
        self.heater.set_temp(target_temp)
        gcmd.respond_info(
            "Performing ambient transfer tests, target is %.1f degrees"
            % (target_temp,)
        )

        reactor = self.heater.reactor
        min_temp = target_temp - FAN_SWEEP_MAX_SAG

        def holding(eventtime):
            temp, _ = self.heater.get_temp(eventtime)
            return temp >= min_temp

        def measure(speed, min_temp=None):
            result = self.measure_loss(gcmd, control, measure_time, min_temp)
            if result is None:
                return False
            power, temp = result
            last_power[0] = power
            transfer = power / (temp - ambient_temp)
            gcmd.respond_info(
                f"{speed * 100.0:.0f}% fan: {power:.2f} W, {transfer:.4f} W/K"
            )
            table.append((speed, transfer))
            # Let the controller use what has been measured so far. Without
            # this every fan step is a disturbance the model knows nothing
            # about, which it can only absorb slowly through its ambient
            # temperature estimate.
            control.const_ambient_transfer = table[0][1]
            control.const_fan_ambient_transfer = list(table)
            control.state_ambient_temp = ambient_temp
            return True

        table = []
        last_power = [0.0]
        power_limit = FAN_SWEEP_MAX_POWER * control.heater_max_power
        try:
            if fan is not None:
                fan.set_speed(0.0)
            gcmd.respond_info("Waiting for temperature to stabilize")
            if not self.wait_stable(gcmd, target_temp):
                gcmd.respond_info(
                    "Temperature is still settling, measuring anyway"
                )
            measure(0.0)
            pending = [speed for speed in fan_speeds if speed > 0.0]
            done = 0.0  # highest fan speed measured so far
            while fan is not None and pending:
                speed = pending.pop(0)
                fan.set_speed(speed)
                settled = reactor.monotonic() + fan_settle_time
                gcmd.wait_while(
                    lambda eventtime: (
                        eventtime < settled and holding(eventtime)
                    ),
                    True,
                    0.5,
                )
                if holding(reactor.monotonic()) and measure(speed, min_temp):
                    done = speed
                    if last_power[0] < power_limit:
                        continue
                    gcmd.respond_info(
                        f"The heater is at its limit with {speed * 100.0:.0f}%"
                        " fan, not testing higher fan speeds"
                    )
                    break
                # Back off before the temperature drops far enough for
                # verify_heater to step in.
                fan.set_speed(0.0)
                if speed - done <= FAN_SWEEP_MIN_STEP + 1e-6:
                    gcmd.respond_info(
                        f"The heater cannot hold {target_temp:.0f} degrees"
                        f" with {speed * 100.0:.0f}% fan, not testing"
                        " this or higher fan speeds"
                    )
                    break
                # The step was too large for a fan the model knows nothing
                # about yet; approach it through an intermediate speed.
                middle = round((done + speed) / 2.0, 3)
                gcmd.respond_info(
                    f"Temperature dropped too far with {speed * 100.0:.0f}%"
                    f" fan, measuring {middle * 100.0:.0f}% first"
                )
                pending = [middle, speed] + pending
                if not self.wait_stable(gcmd, target_temp):
                    gcmd.respond_info(
                        "Temperature did not recover, not testing higher"
                        " fan speeds"
                    )
                    break
        finally:
            if fan is not None:
                fan.set_speed(0.0)

        return {
            "ambient_transfer": table[0][1],
            "fan_ambient_transfer": table if fan is not None else [],
        }

    def measure_loss(self, gcmd, control, duration, min_temp=None):
        """
        Measure the heat lost by the block over `duration` seconds as the
        energy put in by the heater minus the energy that went into changing
        the (modelled) block temperature. Unlike averaging the power at a
        perfectly steady temperature this does not need the temperature to
        settle first. Returns the lost power and the average temperature, or
        None if the temperature fell below min_temp.
        """
        block_temps = [None, None]  # at the first and the last poll
        last = [None, 0.0]  # time and duty at the previous poll
        totals = [0.0, 0.0, 0.0]  # time, energy, temp*time
        too_cold = [False]

        def process(eventtime):
            temp, _ = self.heater.get_temp(eventtime)
            if min_temp is not None and temp < min_temp:
                too_cold[0] = True
                return False
            duty = self.heater.get_status(eventtime)["power"]
            block_temps[1] = control.state_block_temp
            if last[0] is None:
                block_temps[0] = block_temps[1]
            else:
                dt = eventtime - last[0]
                totals[0] += dt
                totals[1] += last[1] * control.full_heater_power * dt
                totals[2] += temp * dt
            last[0], last[1] = eventtime, duty
            return totals[0] < duration

        gcmd.wait_while(process, True, self.heater.get_pwm_delay())
        if too_cold[0]:
            return None
        elapsed = totals[0]
        stored = control.const_block_heat_capacity * (
            block_temps[1] - block_temps[0]
        )
        return (totals[1] - stored) / elapsed, totals[2] / elapsed


class TuningControl:
    def __init__(self, heater):
        self.value = 0.0
        self.target = None
        self.heater = heater
        self.log = []
        self.logging = False

    def temperature_update(self, read_time, temp, target_temp):
        if self.logging:
            self.log.append((read_time, temp, self.value))
        self.heater.set_pwm(read_time, self.value)

    def check_busy(self, eventtime, smoothed_temp, target_temp):
        return self.value != 0.0 or self.target != 0

    def set_output(self, value, target):
        self.value = value
        self.target = target
        self.heater.set_temp(target)

    def get_profile(self):
        return {"name": "tuning"}

    def get_type(self):
        return "tuning"
