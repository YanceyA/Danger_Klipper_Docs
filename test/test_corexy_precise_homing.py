"""CoreXY precise homing against a simulated machine with real wall stalls.

The klippy simulator triggers every homing move at its end, so the geometry
of the diagonal wall probes is checked here with a pure python machine: a
physical head (motor A = x+y, B = x-y) between two walls, and the toolhead
frame klippy believes in, which homing moves re-define like the real ones.
"""

import collections
import math
from types import SimpleNamespace

import pytest

from klippy.kinematics import corexy

SD = 40.0 / (200 * 16)  # step distance
PHASES = 64
CYCLE = SD * PHASES  # 0.8 mm per electrical cycle
WALLS = (113.37, -41.02)  # physical X and Y wall, unrelated to the frame
HomingInfo = collections.namedtuple(
    "homing_info",
    "speed position_endstop retract_speed retract_dist positive_dir"
    " second_homing_speed use_sensorless_homing min_home_dist accel",
)
DIRS = [(1, 1), (-1, -1), (1, -1), (-1, 1)]


class CommandError(Exception):
    pass


class Machine:
    """Fake toolhead and homing_state driving a simulated physical head."""

    def __init__(self, dirs, phys, errs=()):
        self.dirs, self.phys = dirs, list(phys)
        self.pos = [0.0] * 4  # toolhead frame
        self.errs = list(errs)  # per homing move stall error, motor mm
        self.probes = []  # (moving motor, held motor mcu, homing info)
        self.axes = [0, 1]

    def mcu(self, i):
        x, y = self.phys
        return round((x + y if i == 0 else x - y) / SD)

    def get_position(self):
        return list(self.pos)

    def set_position(self, pos, homing_axes=""):
        self.pos = list(pos)

    def flush_step_generation(self):
        pass

    def manual_move(self, coord, speed):
        new = [c if c is not None else p for c, p in zip(coord, self.pos)]
        for i in (0, 1):
            self.phys[i] += new[i] - self.pos[i]
        self.pos[: len(new)] = new

    def get_axes(self):
        return self.axes

    def get_trigger_position(self, name):
        return self.trig[name]

    def home_rails(self, rails, forcepos, movepos):
        def fill(c):
            return [p if v is None else v for v, p in zip(c, self.pos)]

        start, end = fill(forcepos), fill(movepos)
        v = [e - s for e, s in zip(end[:2], start[:2])]
        ts = [
            (w - p) / vi
            for w, p, vi, d in zip(WALLS, self.phys, v, self.dirs)
            if vi * d > 0
        ]
        if not ts or min(ts) > 1.0:
            raise CommandError("No trigger after full movement")
        speed = max(abs(v[0] + v[1]), abs(v[0] - v[1]))
        t = min(ts) + (self.errs.pop(0) if self.errs else 0.0) / speed
        before = [self.mcu(0), self.mcu(1)]
        self.phys = [p + t * vi for p, vi in zip(self.phys, v)]
        moved = [i for i in (0, 1) if self.mcu(i) != before[i]]
        if isinstance(rails[0], corexy.DiagonalProbeRail):
            assert len(moved) == 1
            held = 1 - moved[0]
            hi = rails[0].get_homing_info()
            self.probes.append((moved[0], before[held], hi))
        self.pos = end  # the trigger point is at movepos
        self.trig = {"stepper_x": self.mcu(0), "stepper_y": self.mcu(1)}

    def frame_error_ab(self, kin):
        # Reported minus true position (relative to the walls' endstops)
        ends = [r.get_homing_info().position_endstop for r in kin.rails]
        ex, ey = [
            p - (q - w + e)
            for p, q, w, e in zip(self.pos, self.phys, WALLS, ends)
        ]
        return [(ex + ey) / CYCLE, (ex - ey) / CYCLE]


def make_kin(m, moff=(0, 0), origin=(0.0, 0.0)):
    kin = corexy.CoreXYKinematics.__new__(corexy.CoreXYKinematics)
    kin.rails = []
    for i, (name, d) in enumerate(zip(["stepper_x", "stepper_y"], m.dirs)):
        s = SimpleNamespace(
            get_name=lambda n=name: n,
            get_step_dist=lambda: SD,
            get_mcu_position=lambda i=i: m.mcu(i),
        )
        end = 200.0 if d > 0 else 0.0
        hi = HomingInfo(50.0, end, 30.0, 5.0, d > 0, 50.0, True, 5.0, None)
        kin.rails.append(
            SimpleNamespace(
                get_name=lambda n=name: n,
                get_steppers=lambda s=s: [s],
                get_homing_info=lambda hi=hi: hi,
                get_range=lambda: (0.0, 200.0),
            )
        )
    kin.tmc = [
        SimpleNamespace(get_phase_offset=lambda o=o: (o, PHASES)) for o in moff
    ]
    kin.messages = []
    kin.staged = {}  # gcode and configfile in one
    gcode = SimpleNamespace(
        respond_info=kin.messages.append,
        set=lambda sec, opt, val: kin.staged.update({opt: val}),
    )
    kin.printer = SimpleNamespace(
        command_error=CommandError,
        lookup_object=lambda name: gcode,
        get_start_args=lambda: {},
    )
    kin.toolhead = m
    kin.precise_homing, kin.precise_offset = True, 5.0
    kin.precise_origin, kin.precise_axes = origin, set()
    kin.limits = [(1.0, -1.0)] * 3
    return kin


def classic_homed(dirs, moff=(0, 0), frame_err=(0.0, 0.0), errs=()):
    # Head 20 mm inside the walls, frame as left by an imprecise classic home
    m = Machine(dirs, [w - 20.0 * d for w, d in zip(WALLS, dirs)], errs)
    kin = make_kin(m, moff)
    ends = [r.get_homing_info().position_endstop for r in kin.rails]
    m.pos[:2] = [
        p - w + e + fe for p, w, e, fe in zip(m.phys, WALLS, ends, frame_err)
    ]
    return m, kin


def true_fraction(moff):
    # Fractional cycles of the true walls from a phase aligned start
    wa, wb = WALLS[0] + WALLS[1], WALLS[0] - WALLS[1]
    return [(w / SD + o) / PHASES % 1.0 for w, o in zip((wa, wb), moff)]


@pytest.mark.parametrize("dirs", DIRS)
@pytest.mark.parametrize(
    "moff, frame_err", [((0, 0), (0.0, 0.0)), ((17, 41), (0.31, -0.57))]
)
def test_measure_cycles_geometry(dirs, moff, frame_err):
    m, kin = classic_homed(dirs, moff, frame_err)
    frame = [p - q for p, q in zip(m.pos, m.phys)]
    c, cycles = kin._measure_cycles(m)
    assert cycles == pytest.approx([CYCLE, CYCLE])
    # One motor drives (B when X and Y home the same way), the other holds
    motor = 1 if dirs[0] == dirs[1] else 0
    assert len(m.probes) == 4
    assert {p[0] for p in m.probes} == {motor}
    assert {p[1] for p in m.probes} == {m.mcu(1 - motor)}
    for _, _, hi in m.probes:
        assert hi.speed == pytest.approx(50.0 * math.sqrt(0.5))
        assert hi.retract_dist == hi.min_home_dist == 0.0
    # Back at the phase aligned start in the original frame
    assert [p - q for p, q in zip(m.pos, m.phys)] == pytest.approx(
        frame, abs=SD
    )
    assert all((m.mcu(i) + moff[i]) % PHASES == 0 for i in (0, 1))
    ends = [r.get_homing_info().position_endstop for r in kin.rails]
    for p, e, d in zip(m.pos, ends, dirs):
        assert abs(p - (e - d * 5.0)) < CYCLE  # within the nudge of the park
    # The cycle counts are the true corner offset from the start
    dx, dy = [w - p for w, p in zip(WALLS, m.phys)]
    expect = [(dx + dy) / CYCLE, (dx - dy) / CYCLE]
    assert c == pytest.approx(expect, abs=2 * SD / CYCLE)


@pytest.mark.parametrize("dirs", DIRS)
@pytest.mark.parametrize("moff", [(0, 0), (23, 50)])
def test_home_snaps_corner_to_same_phase(dirs, moff):
    frac = true_fraction(moff)
    errors = []
    # Two homes from different places with different stall errors
    for start, errs in [(30.0, [0.0, 0.0]), (71.3, [-0.47, -1.13])]:
        m = Machine(dirs, [w - start * d for w, d in zip(WALLS, dirs)], errs)
        kin = make_kin(m, moff, origin=[f + 0.1 for f in frac])
        kin.home(m)
        assert len(m.probes) == 4 and not kin.messages
        errors.append(m.frame_error_ab(kin))
    # The corner is a whole number of cycles from the phase aligned start,
    # so whatever the classic home error, the frame error is the same
    assert errors[0] == pytest.approx(errors[1], abs=2 * SD / CYCLE)
    assert errors[0] == pytest.approx(frac, abs=2 * SD / CYCLE)


@pytest.mark.parametrize(
    "c, origin, n, warn",
    [
        ([10.1, -3.05], [0.0, 0.0], [10, -3], False),
        ([10.2, -3.0], [0.0, 0.0], [10, -3], True),
        ([10.0, -2.8], [0.0, 0.0], [10, -3], True),
        ([10.45, -3.0], [0.4, 0.0], [10, -3], False),
        ([10.3, -3.0], [0.0, 0.0], None, None),
        ([10.0, -3.5], [0.0, 0.0], None, None),
        ([10.45, -3.0], [-0.2, 0.0], None, None),
    ],
)
def test_snap_home_rounding(c, origin, n, warn):
    m, kin = classic_homed((-1, 1))
    kin.precise_origin = origin
    if n is None:
        with pytest.raises(CommandError, match="unstable"):
            kin._snap_home(c, [CYCLE, CYCLE])
        return
    kin._snap_home(c, [CYCLE, CYCLE])
    a, b = n[0] * CYCLE, n[1] * CYCLE
    assert m.pos[:2] == pytest.approx([-0.5 * (a + b), 200 - 0.5 * (a - b)])
    assert bool(kin.messages) == warn


@pytest.mark.parametrize(
    "errs, probes",
    [
        ([-7.5, -7.5, 0.0, -0.05], 6),  # early twice, then agreeing
        ([-0.4, 0.0, 0.1], 5),  # disagreeing, then two agreeing
        ([0.0, -0.4] * 3, None),  # never agreeing
    ],
)
def test_probe_acceptance(errs, probes):
    m, kin = classic_homed((1, 1), errs=errs)
    if probes is None:
        with pytest.raises(CommandError, match="inconsistent X wall probes"):
            kin._measure_cycles(m)
        assert len(m.probes) == corexy.PROBES_PER_WALL + corexy.PROBE_RETRIES
        return
    c, _ = kin._measure_cycles(m)
    assert len(m.probes) == probes
    # Only the two accepted probes count: their mean stall error 0.0 or 0.05
    dx, dy = [w - p for w, p in zip(WALLS, m.phys)]
    err = 0.5 * (errs[probes - 4] + errs[probes - 3]) / 2
    expect = [(dx + dy + err) / CYCLE, (dx - dy + err) / CYCLE]
    assert c == pytest.approx(expect, abs=2 * SD / CYCLE)


def test_home_snaps_only_after_x_and_y():
    m, kin = classic_homed((1, 1))
    kin.precise_origin = None
    kin.home(m)  # uncalibrated: notice, no probes
    assert len(kin.messages) == 1 and not m.probes
    kin.precise_origin = true_fraction((0, 0))
    m.axes = [0]
    kin.home(m)
    kin.clear_homing_state("xy")
    m.axes = [1]
    kin.home(m)
    assert not m.probes  # X was cleared in between
    m.axes = [0]
    kin.home(m)
    assert len(m.probes) == 4  # G28 Y then G28 X snaps once
    kin.home(m)
    assert len(m.probes) == 4  # Y not homed since the snap


@pytest.mark.parametrize(
    "errs, error",
    [
        ([0.04, 0.0, -0.03, 0.05] * 5, None),
        ([0.0] * 8 + [-0.6, -0.6] + [0.0] * 10, "not repeatable"),
        ([0.0, -0.4] * 3, "inconsistent"),
    ],
)
def test_calibrate(monkeypatch, errs, error):
    m, kin = classic_homed((1, 1), moff=(9, 30), errs=errs)
    kin.precise_origin = None
    kin.limits = [(0.0, 200.0)] * 2 + [(1.0, -1.0)]
    monkeypatch.setattr(corexy.homing, "Homing", lambda p, g: m)
    m.set_axes = lambda axes: None
    gcmd = SimpleNamespace(
        get_int=lambda name, default, minval: default,
        respond_info=kin.messages.append,
        error=CommandError,
    )
    if error:
        with pytest.raises(CommandError, match=error):
            kin.cmd_COREXY_HOMING_CALIBRATE(gcmd)
        homed = kin.limits[0][0] <= kin.limits[0][1]
        assert homed == (error == "not repeatable")
        return
    kin.cmd_COREXY_HOMING_CALIBRATE(gcmd)
    # 5 samples at whole-cycle grid offsets, circular mean of the fractions
    assert len(m.probes) == 20
    frac = true_fraction((9, 30))
    for o, f in zip(kin.precise_origin, frac):
        assert -0.5 < o <= 0.5
        assert (o - f + 0.5) % 1.0 - 0.5 == pytest.approx(0.0, abs=0.03)
    value = "%.3f, %.3f" % tuple(kin.precise_origin)
    assert kin.staged == {"corexy_precise_homing_origin": value}
    # Snapped: the frame error is the calibrated origin (within the noise)
    err = m.frame_error_ab(kin)
    assert err == pytest.approx(kin.precise_origin, abs=0.03)
