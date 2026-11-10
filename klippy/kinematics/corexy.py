# Code for handling the kinematics of corexy robots
#
# Copyright (C) 2017-2021  Kevin O'Connor <kevin@koconnor.net>
#
# This file may be distributed under the terms of the GNU GPLv3 license.
import logging
import math

from klippy import stepper
from klippy.extras import endstop_phase, homing

PROBES_PER_WALL = 2  # last wall probes that must not be early and agree
PROBE_RETRIES = 3  # extra wall probes allowed before giving up
# Whole-cycle AB offsets that COREXY_HOMING_CALIBRATE samples at
CAL_GRID = [(0, 0), (1, 0), (0, 1), (1, 1), (2, 1), (1, 2)]


# A rail as used by the single-motor diagonal probes. The motor turns
# sqrt(2) times faster than the toolhead moves, the held motor never moves
# (which a rehome would reject), and a retract after a stall is pointless.
class DiagonalProbeRail:
    def __init__(self, rail):
        self.rail = rail

    def __getattr__(self, name):
        return getattr(self.rail, name)

    def get_homing_info(self):
        hi = self.rail.get_homing_info()
        return hi._replace(
            speed=hi.speed * math.sqrt(0.5), retract_dist=0.0, min_home_dist=0.0
        )


class CoreXYKinematics:
    def __init__(self, toolhead, config):
        # Setup axis rails
        self.rails = [
            stepper.LookupMultiRail(config.getsection("stepper_" + n))
            for n in "xyz"
        ]
        for s in self.rails[1].get_steppers():
            self.rails[0].get_endstops()[0][0].add_stepper(s)
        for s in self.rails[0].get_steppers():
            self.rails[1].get_endstops()[0][0].add_stepper(s)
        self.rails[0].setup_itersolve("corexy_stepper_alloc", b"+")
        self.rails[1].setup_itersolve("corexy_stepper_alloc", b"-")
        self.rails[2].setup_itersolve("cartesian_stepper_alloc", b"z")
        for s in self.get_steppers():
            s.set_trapq(toolhead.get_trapq())
        # Setup boundary checks
        max_velocity, max_accel = toolhead.get_max_velocity()
        self.max_z_velocity = config.getfloat(
            "max_z_velocity", max_velocity, above=0.0, maxval=max_velocity
        )
        self.max_z_accel = config.getfloat(
            "max_z_accel", max_accel, above=0.0, maxval=max_accel
        )
        self.limits = [(1.0, -1.0)] * 3
        ranges = [r.get_range() for r in self.rails]
        self.axes_min = toolhead.Coord(*[r[0] for r in ranges], e=0.0)
        self.axes_max = toolhead.Coord(*[r[1] for r in ranges], e=0.0)
        self.supports_dual_carriage = False
        # Setup precise (phase snapped) XY homing
        self.printer = config.get_printer()
        self.toolhead = toolhead
        self.precise_homing = config.getboolean("corexy_precise_homing", False)
        self.precise_offset = config.getfloat(
            "corexy_precise_homing_offset", 5.0, above=0.0
        )
        self.precise_origin = config.getfloatlist(
            "corexy_precise_homing_origin", None, count=2
        )
        self.precise_axes = set()
        if self.precise_homing:
            for rail in self.rails[:2]:
                if not rail.get_homing_info().use_sensorless_homing:
                    raise config.error(
                        "corexy_precise_homing requires sensorless homing"
                        " on %s" % (rail.get_name(),)
                    )
                if config.has_section("endstop_phase " + rail.get_name()):
                    raise config.error(
                        "corexy_precise_homing is incompatible with"
                        " [endstop_phase %s]" % (rail.get_name(),)
                    )
            self.printer.register_event_handler(
                "klippy:connect", self._handle_connect
            )
            self.printer.lookup_object("gcode").register_command(
                "COREXY_HOMING_CALIBRATE",
                self.cmd_COREXY_HOMING_CALIBRATE,
                desc=self.cmd_COREXY_HOMING_CALIBRATE_help,
            )

    def _handle_connect(self):
        self.tmc = []
        for rail in self.rails[:2]:
            name = rail.get_name()
            for driver in endstop_phase.TRINAMIC_DRIVERS:
                module = self.printer.lookup_object(
                    "%s %s" % (driver, name), None
                )
                if module is not None:
                    self.tmc.append(module)
                    break
            else:
                raise self.printer.config_error(
                    "corexy_precise_homing requires a TMC driver on %s"
                    % (name,)
                )

    def get_steppers(self):
        return [s for rail in self.rails for s in rail.get_steppers()]

    def calc_position(self, stepper_positions):
        pos = [stepper_positions[rail.get_name()] for rail in self.rails]
        return [0.5 * (pos[0] + pos[1]), 0.5 * (pos[0] - pos[1]), pos[2]]

    def set_position(self, newpos, homing_axes):
        for i, rail in enumerate(self.rails):
            rail.set_position(newpos)
            if "xyz"[i] in homing_axes:
                self.limits[i] = rail.get_range()

    def note_z_not_homed(self):
        self.clear_homing_state("z")

    def clear_homing_state(self, clear_axes):
        for axis, axis_name in enumerate("xyz"):
            if axis_name in clear_axes:
                self.limits[axis] = (1.0, -1.0)
                self.precise_axes.discard(axis)

    def home(self, homing_state):
        # Each axis is homed independently and in order
        for axis in homing_state.get_axes():
            rail = self.rails[axis]
            # Determine movement
            position_min, position_max = rail.get_range()
            hi = rail.get_homing_info()
            homepos = [None, None, None, None]
            homepos[axis] = hi.position_endstop
            forcepos = list(homepos)
            if hi.positive_dir:
                forcepos[axis] -= 1.5 * (hi.position_endstop - position_min)
            else:
                forcepos[axis] += 1.5 * (position_max - hi.position_endstop)
            # Perform homing
            homing_state.home_rails([rail], forcepos, homepos)
            # Snap XY once both X and Y have been homed here (before any Z)
            self.precise_axes.add(axis)
            if self.precise_homing and {0, 1} <= self.precise_axes:
                self.precise_axes.clear()
                if self.precise_origin is None:
                    self.printer.lookup_object("gcode").respond_info(
                        "corexy precise homing is not calibrated;"
                        " run COREXY_HOMING_CALIBRATE"
                    )
                else:
                    self._snap_home(*self._measure_cycles(homing_state))

    def _move_ab(self, a, b, speed):
        x, y = self.toolhead.get_position()[:2]
        self.toolhead.manual_move([x + 0.5 * (a + b), y + 0.5 * (a - b)], speed)
        self.toolhead.flush_step_generation()

    # Precise homing: park `offset` mm from both walls and nudge both motors
    # onto a whole electrical cycle. Then drive one motor alone, so the head
    # runs on a 45 degree diagonal, into the X wall and then the Y wall,
    # watching only that motor's stallguard (B when X and Y home the same
    # way, else A). With motor travels d_x and d_y the corner is at
    # (sx * d_x / 2, sy * d_y / 2) from the start, ie A = (sx*d_x + sy*d_y) / 2
    # and B = (sx*d_x - sy*d_y) / 2. Rounding these to whole cycles (around
    # the calibrated origin) puts the corner on the same motor phases on
    # every home, regardless of where within a cycle stallguard fired.
    def _measure_cycles(self, homing_state, grid=(0, 0)):
        # Returns the corner relative to the start in cycles of [A, B], and
        # the mm per cycle of [A, B]
        toolhead = self.toolhead
        his = [rail.get_homing_info() for rail in self.rails[:2]]
        dirs = [1.0 if hi.positive_dir else -1.0 for hi in his]
        offset, speed = self.precise_offset, his[0].retract_speed
        park = [hi.position_endstop - d * offset for hi, d in zip(his, dirs)]
        toolhead.manual_move(park, speed)
        toolhead.flush_step_generation()
        steppers = [rail.get_steppers()[0] for rail in self.rails[:2]]
        cycles, nudge = [], []
        for s, tmc, g in zip(steppers, self.tmc, grid):
            moff, phases = tmc.get_phase_offset()
            if moff is None:
                if self.printer.get_start_args().get("debugoutput") is None:
                    raise self.printer.command_error(
                        "Stepper %s phase unknown" % (s.get_name(),)
                    )
                moff = 0
            steps = (-(s.get_mcu_position() + moff)) % phases - g * phases
            cycles.append(phases * s.get_step_dist())
            nudge.append(steps * s.get_step_dist())
        self._move_ab(*nudge, speed)
        start = toolhead.get_position()
        m = 1 if his[0].positive_dir == his[1].positive_dir else 0
        rail, s = DiagonalProbeRail(self.rails[m]), steppers[m]
        start_mcu = s.get_mcu_position()
        dists = []
        for wall in (0, 1):
            # Fake frame with the wall at movepos, reached by a pure A or B
            # vector of 2 * offset per axis (to this wall, from the other).
            # A trigger before half the expected motor travel is early.
            vec = [-d * 2.0 * offset for d in dirs]
            vec[wall] = -vec[wall]
            forcepos = start[:2] + [None]
            forcepos[wall] = his[wall].position_endstop - vec[wall]
            movepos = [forcepos[0] + vec[0], forcepos[1] + vec[1], None]
            early = abs(his[wall].position_endstop - start[wall])
            samples = []
            for _ in range(PROBES_PER_WALL + PROBE_RETRIES):
                homing_state.home_rails([rail], forcepos, movepos)
                trig = homing_state.get_trigger_position(s.get_name())
                # Return to the start by step count; the frame is fake
                back = [0.0, 0.0]
                back[m] = (start_mcu - s.get_mcu_position()) * s.get_step_dist()
                self._move_ab(*back, speed)
                samples.append(abs(trig - start_mcu) * s.get_step_dist())
                last = samples[-PROBES_PER_WALL:]
                if (
                    len(last) == PROBES_PER_WALL
                    and min(last) > early
                    and max(last) - min(last) <= 0.25 * cycles[m]
                ):
                    dists.append(sum(last) / PROBES_PER_WALL)
                    break
            else:
                raise self.printer.command_error(
                    "corexy precise homing: inconsistent %s wall probes (%s)"
                    % ("XY"[wall], ", ".join("%.3f" % d for d in samples))
                )
        toolhead.set_position(start)
        dx, dy = dirs[0] * dists[0], dirs[1] * dists[1]
        c = [0.5 * (dx + dy) / cycles[0], 0.5 * (dx - dy) / cycles[1]]
        return c, cycles

    def _snap_home(self, c, cycles):
        rel = [ci - oi for ci, oi in zip(c, self.precise_origin)]
        n = [round(r) for r in rel]
        drift = max(abs(r - ni) for r, ni in zip(rel, n))
        if drift > 0.25:
            raise self.printer.command_error(
                "corexy precise homing: phase origin unstable (cycles A %.3f"
                " B %.3f, origin A %.3f B %.3f); run COREXY_HOMING_CALIBRATE"
                % (c[0], c[1], self.precise_origin[0], self.precise_origin[1])
            )
        if drift > 0.15:
            self.printer.lookup_object("gcode").respond_info(
                "corexy precise homing: phase origin has drifted %.2f cycles;"
                " consider running COREXY_HOMING_CALIBRATE" % (drift,)
            )
        logging.info(
            "corexy precise homing: cycles A %.3f B %.3f -> %d %d", *c, *n
        )
        a, b = n[0] * cycles[0], n[1] * cycles[1]
        ex, ey = [r.get_homing_info().position_endstop for r in self.rails[:2]]
        pos = self.toolhead.get_position()
        pos[:2] = [ex - 0.5 * (a + b), ey - 0.5 * (a - b)]
        self.toolhead.set_position(pos)

    cmd_COREXY_HOMING_CALIBRATE_help = "Calibrate CoreXY precise homing origin"

    def cmd_COREXY_HOMING_CALIBRATE(self, gcmd):
        if any(lo > hi for lo, hi in self.limits[:2]):
            raise gcmd.error("Must home X and Y first")
        samples = gcmd.get_int("SAMPLES", 5, minval=2)
        homing_state = homing.Homing(self.printer, gcmd)
        homing_state.set_axes([0, 1])
        results = []
        try:
            for i in range(samples):
                grid = CAL_GRID[i % len(CAL_GRID)]
                c, cycles = self._measure_cycles(homing_state, grid)
                results.append(c)
                gcmd.respond_info("cycles A %.3f B %.3f" % tuple(c))
        except self.printer.command_error:
            self.clear_homing_state("xy")
            raise
        # Circular mean and spread of the fractional cycles
        origin = []
        for vals in zip(*results):
            angles = [2.0 * math.pi * v for v in vals]
            sin, cos = sum(map(math.sin, angles)), sum(map(math.cos, angles))
            o = math.atan2(sin, cos) / (2.0 * math.pi)
            if max(abs((v - o + 0.5) % 1.0 - 0.5) for v in vals) > 0.25:
                raise gcmd.error(
                    "corexy precise homing not repeatable; check sensorless"
                    " homing tuning"
                )
            origin.append(o)
        self.precise_origin = origin
        self._snap_home(c, cycles)
        value = "%.3f, %.3f" % tuple(origin)
        gcmd.respond_info(
            "printer: corexy_precise_homing_origin: %s\n"
            "The SAVE_CONFIG command will update the printer config file\n"
            "with the above and restart the printer." % (value,)
        )
        configfile = self.printer.lookup_object("configfile")
        configfile.set("printer", "corexy_precise_homing_origin", value)

    def _check_endstops(self, move):
        end_pos = move.end_pos
        for i in (0, 1, 2):
            if move.axes_d[i] and (
                end_pos[i] < self.limits[i][0] or end_pos[i] > self.limits[i][1]
            ):
                if self.limits[i][0] > self.limits[i][1]:
                    raise move.move_error("Must home axis first")
                raise move.move_error()

    def check_move(self, move):
        limits = self.limits
        xpos, ypos = move.end_pos[:2]
        if (
            xpos < limits[0][0]
            or xpos > limits[0][1]
            or ypos < limits[1][0]
            or ypos > limits[1][1]
        ):
            self._check_endstops(move)
        if not move.axes_d[2]:
            # Normal XY move - use defaults
            return
        # Move with Z - update velocity and accel for slower Z axis
        self._check_endstops(move)
        z_ratio = move.move_d / abs(move.axes_d[2])
        move.limit_speed(
            self.max_z_velocity * z_ratio, self.max_z_accel * z_ratio
        )

    def get_status(self, eventtime):
        axes = [a for a, (l, h) in zip("xyz", self.limits) if l <= h]
        return {
            "homed_axes": "".join(axes),
            "axis_minimum": self.axes_min,
            "axis_maximum": self.axes_max,
        }


def load_kinematics(toolhead, config):
    return CoreXYKinematics(toolhead, config)
