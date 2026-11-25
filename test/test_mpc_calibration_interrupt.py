"""An interrupted MPC fan sweep stops the calibration fan."""

from types import SimpleNamespace

import pytest
import test_gcode_interrupt as commands

from klippy.extras import control_mpc
from klippy.gcode import WaitInterruption

dispatch = commands.dispatch


def test_mpc_fan_sweep_interrupt_restores_fan(dispatch):
    gcode, reactor, printer, _ = dispatch
    heater, _ = commands.calibration_heater(dispatch)
    heater.get_temp = lambda eventtime: (200.0, heater.target_temp)
    speeds = []
    fan = SimpleNamespace(set_speed=speeds.append)
    original = SimpleNamespace(cooling_fan=fan)
    control = SimpleNamespace(heater_max_power=50.0, full_heater_power=50.0)
    mpc = control_mpc.MpcCalibrate(printer, heater, original)
    mpc.wait_stable = lambda gcmd, target_temp: True

    def measure_loss(gcmd, control, duration, min_temp=None):
        if speeds and speeds[-1] == 1.0:
            reactor.register_callback(
                lambda eventtime: gcode.run_script("HEATER_INTERRUPT")
            )
            gcmd.wait_while(lambda eventtime: True, interval=30.0)
        return 10.0, 200.0

    mpc.measure_loss = measure_loss

    def calibrate(gcmd):
        mpc.transfer_test(gcmd, control, 200.0, 20.0, [0.0, 1.0], 0.0, 15.0)
        pytest.fail("cancelled calibration continued")

    gcode.register_command("MPC_FAN_TEST", calibrate)

    def action():
        with pytest.raises(WaitInterruption):
            gcode.run_script("MPC_FAN_TEST")

    commands.run_in_reactor(reactor, action)
    assert speeds == [0.0, 1.0, 0.0]


def test_filament_feed_runs_scripts_while_waiting(dispatch):
    """MPC_CALIBRATE_FILAMENT queues extrusion from inside its wait loops."""
    gcode, reactor, printer, _ = dispatch
    extruded = []
    gcode.register_command(
        "G1", lambda gcmd: extruded.append(gcmd.get_float("E"))
    )
    polls = []

    def calibrate(gcmd):
        def feed(eventtime):
            polls.append(eventtime)
            gcmd.run_script("G1 E%.3f F300" % (0.5 * len(polls),))
            return len(polls) < 3

        gcmd.wait_while(feed, True, 0.001)

    gcode.register_command("MPC_FEED_TEST", calibrate)
    commands.run_in_reactor(reactor, lambda: gcode.run_script("MPC_FEED_TEST"))
    assert extruded == [0.5, 1.0, 1.5]
