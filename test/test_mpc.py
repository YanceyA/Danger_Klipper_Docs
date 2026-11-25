"""MPC controller and calibration tests against a simulated hotend."""

import configparser
import math
from types import SimpleNamespace

import pytest
from klippy_testing.mpc_sim import FakeFan, Plant, Sim

from klippy import configfile
from klippy.extras import (
    control_mpc,
    heaters,
    mpc_ambient_temperature,
    mpc_block_temperature,
)

CALIBRATED = {
    "block_heat_capacity": 18.0,
    "sensor_responsiveness": 0.25,
    "ambient_transfer": 0.10,
}


def hot_control(sim, temp=200.0, **overrides):
    """A calibrated controller whose model already sits at `temp`."""
    control = sim.make_control(**{**CALIBRATED, **overrides})
    control.state_block_temp = control.state_sensor_temp = temp
    control.last_temp_time = sim.time
    sim.heater.target_temp = temp
    return control


######################################################################
# Heater power
######################################################################


def test_heater_power_parsing():
    parse = control_mpc.parse_heater_power
    assert parse(["50"]) == [(0.0, 50.0)]
    assert parse(["300:51", "100:69"]) == [(100.0, 69.0), (300.0, 51.0)]
    for bad in ([], ["0"], ["100:69", "51"], ["100:69", "100:51"], ["100:-5"]):
        with pytest.raises(ValueError):
            parse(bad)


def test_heater_power_table_lookup():
    power_at = control_mpc.heater_power_at
    assert power_at([(0.0, 50.0)], 250.0) == 50.0
    table = [(180.0, 72.0), (240.0, 65.0), (300.0, 60.0)]
    assert power_at(table, 180.0) == 72.0
    assert power_at(table, 210.0) == pytest.approx(68.5)
    assert power_at(table, 270.0) == pytest.approx(62.5)
    # The end segments are extended beyond the listed temperatures...
    assert power_at(table, 120.0) == pytest.approx(79.0)
    assert power_at(table, 330.0) == pytest.approx(57.5)
    # ...but only within limits
    steep = [(200.0, 60.0), (210.0, 30.0)]
    assert power_at(steep, 20.0) == 120.0
    assert power_at(steep, 400.0) == 15.0


def test_ptc_heater_duty_follows_block_temperature():
    sim = Sim()
    control = hot_control(
        sim, temp=300.0, heater_power=[(100.0, 69.0), (300.0, 51.0)]
    )
    # Far below target: the heater is driven fully on, which is now 51 W
    control.temperature_update(sim.time + 0.3, 300.0, 350.0)
    assert control.heater_max_power == pytest.approx(51.0, rel=0.01)
    assert control.last_power == pytest.approx(control.heater_max_power)
    assert sim.heater.last_pwm_value == pytest.approx(1.0)


def test_calibration_with_ptc_heater():
    # 60 W at 200C, losing 15% per 100C: the heater is much stronger while
    # the block is cold than a single power value can express.
    def calibrate(heater_power):
        plant = Plant(heater_w=60.0, block_c=20.0, ptc_alpha=0.0015)
        values, gcmd = Sim(plant=plant).calibrate(heater_power=heater_power)
        return float(values["block_heat_capacity"])

    table = control_mpc.parse_heater_power(["100:69", "300:51"])
    assert calibrate(table) == pytest.approx(20.0, rel=0.03)
    assert abs(calibrate(60.0) - 20.0) > 1.5


######################################################################
# Fan model
######################################################################


def test_fan_lookup_uses_requested_value_not_scaled_speed():
    # Calibration indexes the table by the requested fan value, so a fan
    # with max_power < 1 must not be looked up by its scaled speed.
    fan = FakeFan(max_power=0.5)
    sim = Sim(fan=fan)
    control = hot_control(
        sim, fan_ambient_transfer=[(0.0, 0.10), (0.5, 0.20), (1.0, 0.30)]
    )
    fan.set_speed(1.0)
    control.temperature_update(sim.time + 0.3, 200.0, 200.0)
    assert control.last_loss_ambient == pytest.approx(
        0.30 * (control.state_block_temp - control.state_ambient_temp)
    )


def test_fan_table_parsing():
    parse = control_mpc.parse_fan_ambient_transfer
    assert parse([]) == []
    # Plain values are evenly spaced from 0% to 100% fan
    assert parse(["0.1", "0.2", "0.3"]) == [(0.0, 0.1), (0.5, 0.2), (1.0, 0.3)]
    assert parse(["1:0.3", " 0:0.1", "0.2:0.15 "]) == [
        (0.0, 0.1),
        (0.2, 0.15),
        (1.0, 0.3),
    ]
    for bad in (["0:0.1", "0.2"], ["0:0.1", "1.5:0.2"], ["0:0.1", "0:0.2"]):
        with pytest.raises(ValueError):
            parse(bad)


def test_fan_table_formatting_round_trips():
    parse = control_mpc.parse_fan_ambient_transfer
    fmt = control_mpc.format_fan_ambient_transfer
    even = [(0.0, 0.1), (0.5, 0.2), (1.0, 0.3)]
    assert fmt(even) == "0.1, 0.2, 0.3"
    uneven = [(0.0, 0.1), (0.2, 0.15), (1.0, 0.3)]
    assert fmt(uneven) == "0:0.1, 0.2:0.15, 1:0.3"
    for table in (even, uneven, [(0.0, 0.1)], []):
        assert parse(fmt(table).split(",")) == table


def test_fan_table_interpolation():
    table = [(0.0, 0.10), (0.2, 0.12), (1.0, 0.20)]
    lookup = control_mpc.interpolate_fan_ambient_transfer
    assert lookup(table, 0.0) == 0.10
    assert lookup(table, 0.1) == pytest.approx(0.11)
    assert lookup(table, 0.6) == pytest.approx(0.16)
    assert lookup(table, 1.0) == 0.20
    # Outside the measured range the nearest measurement is used
    assert lookup(table[1:], 0.0) == 0.12
    assert lookup(table[:2], 1.0) == 0.12


######################################################################
# Ambient adaptation
######################################################################


def test_ambient_adapts_while_output_is_not_clipped():
    # 40 W into a 200C block changes the temperature faster than
    # steady_state_rate, so only the "output not clipped" condition can
    # allow the ambient estimate to adapt.
    sim = Sim()
    control = hot_control(sim)
    control.last_power = 40.0
    control.temperature_update(sim.time + 0.3, 200.5, 200.0)
    assert control.state_ambient_temp > control_mpc.AMBIENT_TEMP


def test_ambient_frozen_while_output_is_clipped():
    sim = Sim()
    control = hot_control(sim)
    control.last_power = control.heater_max_power
    control.temperature_update(sim.time + 0.3, 200.5, 200.0)
    assert control.state_ambient_temp == control_mpc.AMBIENT_TEMP


######################################################################
# Filament feed forward
######################################################################


def test_model_prediction_honours_filament_temperature_source():
    def block_after_extruding(filament_temp_src):
        sim = Sim(flow=lambda t: 10.0)
        control = hot_control(sim, filament_temp_src=filament_temp_src)
        sim.time += 0.3
        control.temperature_update(sim.time, 200.0, 200.0)
        return control.state_block_temp

    ambient = block_after_extruding((control_mpc.FILAMENT_TEMP_SRC_AMBIENT,))
    preheated = block_after_extruding(
        (control_mpc.FILAMENT_TEMP_SRC_FIXED, 150.0)
    )
    # Warmer filament takes less heat out of the modelled block
    assert preheated > ambient


######################################################################
# Virtual sensors
######################################################################


def virtual_sensor(cls, control):
    sensor = cls.__new__(cls)
    sensor.name = "virtual"
    sensor.min_temp, sensor.max_temp = 0.0, 300.0
    sensor.ignore = sensor.echo_limits_to_console = False
    sensor.heater = SimpleNamespace(
        smoothed_temp=123.0,
        get_control=lambda: control,
        mcu_pwm=SimpleNamespace(
            get_mcu=lambda: SimpleNamespace(non_critical_disconnected=False)
        ),
    )
    reported = []
    sensor.temperature_callback = lambda time, temp: reported.append(temp)
    return sensor, reported


def test_virtual_sensors_survive_the_tuning_controller():
    # MPC_CALIBRATE swaps in a tuning controller that has no model state
    tuning = control_mpc.TuningControl(SimpleNamespace())
    sensor, reported = virtual_sensor(
        mpc_block_temperature.MPC_BLOCK_TEMP_WRAPPER, tuning
    )
    sensor.process_temp_update(tuning, 1.0)
    assert reported == [123.0]

    sensor, reported = virtual_sensor(
        mpc_ambient_temperature.MPC_AMBIENT_TEMP_WRAPPER, tuning
    )
    sensor.process_temp_update(tuning, 1.0)
    assert reported == [control_mpc.AMBIENT_TEMP]


def test_virtual_sensors_report_model_state():
    sim = Sim(plant=Plant())
    control = hot_control(sim, temp=210.0)
    control.state_ambient_temp = 31.0
    sensor, reported = virtual_sensor(
        mpc_block_temperature.MPC_BLOCK_TEMP_WRAPPER, control
    )
    sensor.process_temp_update(control, 1.0)
    assert reported == [210.0]

    sensor, reported = virtual_sensor(
        mpc_ambient_temperature.MPC_AMBIENT_TEMP_WRAPPER, control
    )
    sensor.process_temp_update(control, 1.0)
    assert reported == [31.0]


######################################################################
# Calibration: model identification
######################################################################


def record_heatup(plant, target=200.0):
    """Full power until `target`, then coast; like the calibration does."""
    sim = Sim(plant=plant)
    tuning = control_mpc.TuningControl(sim.heater)
    sim.heater.control = tuning
    tuning.logging = True
    tuning.set_output(1.0, target)
    while sim.heater.smoothed_temp < target:
        sim.advance(0.3)
    tuning.set_output(0.0, target)
    sim.advance(control_mpc.HEATUP_COAST_TIME)
    return tuning.log


@pytest.mark.parametrize("noise", [0.0, 0.25])
def test_fit_recovers_model_parameters(noise):
    plant = Plant(heater_w=50.0, block_c=18.0, beta=0.25, h0=0.10, noise=noise)
    fit = control_mpc.fit_thermal_model(record_heatup(plant), 25.0, 50.0, 0.3)
    assert fit["block_heat_capacity"] == pytest.approx(18.0, rel=0.03)
    assert fit["ambient_transfer"] == pytest.approx(0.10, rel=0.10)
    assert fit["sensor_responsiveness"] == pytest.approx(0.25, rel=0.15)
    assert fit["rms_error"] < noise + 0.1


def test_fit_with_known_ambient_transfer():
    plant = Plant(block_c=18.0, beta=0.25, h0=0.10)
    fit = control_mpc.fit_thermal_model(
        record_heatup(plant), 25.0, 50.0, 0.3, ambient_transfer=0.10
    )
    assert fit["ambient_transfer"] == 0.10
    assert fit["block_heat_capacity"] == pytest.approx(18.0, rel=0.03)


def test_fit_rejects_a_heater_that_did_not_heat():
    samples = [(0.3 * i, 25.0, 1.0) for i in range(100)]
    with pytest.raises(ValueError):
        control_mpc.fit_thermal_model(samples, 25.0, 50.0, 0.3)
    with pytest.raises(ValueError):
        control_mpc.fit_thermal_model(samples[:5], 25.0, 50.0, 0.3)


@pytest.mark.parametrize("beta", [0.125, 0.25, 0.5])
def test_calibration_identifies_the_sensor(beta):
    # The calibrated sensor responsiveness must follow the real sensor
    plant = Plant(heater_w=50.0, block_c=18.0, beta=beta, h0=0.10)
    values, _ = Sim(plant=plant).calibrate()
    # The controller steps its model once per sensor update, so the value
    # that reproduces the sensor is the per-update equivalent.
    equivalent = (1.0 - math.exp(-beta * 0.3)) / 0.3
    assert float(values["sensor_responsiveness"]) == pytest.approx(
        equivalent, rel=0.1
    )
    assert float(values["block_heat_capacity"]) == pytest.approx(18.0, rel=0.05)
    assert float(values["ambient_transfer"]) == pytest.approx(0.10, rel=0.05)


@pytest.mark.parametrize(
    "plant",
    [
        Plant(heater_w=70.0, block_c=25.0),
        Plant(heater_w=40.0, block_c=14.0),
        Plant(heater_w=50.0, block_c=18.0, ambient=45.0),
        Plant(heater_w=50.0, block_c=18.0, noise=0.25),
    ],
    ids=["70W", "40W", "warm chamber", "noisy sensor"],
)
def test_calibration_across_hotends(plant):
    values, _ = Sim(plant=plant, fan=FakeFan()).calibrate()
    assert float(values["block_heat_capacity"]) == pytest.approx(
        plant.block_c, rel=0.05
    )
    assert float(values["sensor_responsiveness"]) == pytest.approx(
        plant.beta, rel=0.2
    )
    fan_table = [float(v) for v in values["fan_ambient_transfer"].split(",")]
    assert fan_table[0] == pytest.approx(0.10, rel=0.06)
    assert fan_table[-1] == pytest.approx(0.20, rel=0.06)


######################################################################
# Calibration: fan sweep
######################################################################


def fan_calibration(plant, params=None):
    fan = FakeFan()
    sim = Sim(plant=plant, fan=fan)
    values, gcmd = sim.calibrate(params)
    table = control_mpc.parse_fan_ambient_transfer(
        values["fan_ambient_transfer"].split(",")
    )
    return sim, fan, table, gcmd


def coldest_while_holding(sim, target=200.0):
    reached = next(i for i, row in enumerate(sim.log) if row[1] >= target)
    return min(row[1] for row in sim.log[reached:])


def test_fan_sweep_measures_requested_speeds():
    plant = Plant(h0=0.10, h_fan=0.10)
    sim, fan, table, _ = fan_calibration(plant, {"FAN_SPEEDS": "0.1,0.3,1"})
    assert [speed for speed, _ in table] == [0.0, 0.1, 0.3, 1.0]
    for speed, transfer in table:
        assert transfer == pytest.approx(plant.transfer(speed), rel=0.05)
    assert fan.value == 0.0


def test_fan_sweep_is_fast_and_holds_temperature():
    # Measured fan speeds are fed forward, so later steps barely disturb
    # the temperature and no long wait is needed between them.
    sim, _, table, _ = fan_calibration(Plant(), {"FAN_BREAKPOINTS": 7})
    assert len(table) == 7
    assert sim.time < 420.0
    assert coldest_while_holding(sim) > 200.0 - 3.0


def test_fan_sweep_approaches_a_large_step_gradually():
    # One step to a strong fan cools the block faster than a model that
    # knows nothing about the fan can react; it is retried via 50%.
    plant = Plant(h0=0.10, h_fan=0.16)
    sim, fan, table, gcmd = fan_calibration(plant, {"FAN_BREAKPOINTS": 2})
    assert [speed for speed, _ in table] == [0.0, 0.5, 1.0]
    assert table[-1][1] == pytest.approx(0.26, rel=0.05)
    assert any("measuring 50% first" in msg for msg in gcmd.responses)
    limit = 200.0 - control_mpc.FAN_SWEEP_MAX_SAG
    assert coldest_while_holding(sim) > limit - 1.0


def test_fan_sweep_stops_at_the_heater_limit():
    # At full fan this block loses more heat than the heater can supply.
    # The sweep must finish with what could be measured, not hang.
    plant = Plant(heater_w=40.0, block_c=14.0, h0=0.10, h_fan=0.20)
    sim, fan, table, gcmd = fan_calibration(plant)
    speeds = [speed for speed, _ in table]
    assert speeds[0] == 0.0 and 0.0 < speeds[-1] < 1.0
    for speed, transfer in table:
        assert transfer == pytest.approx(plant.transfer(speed), rel=0.06)
    assert any("not testing" in msg for msg in gcmd.responses)
    assert fan.value == 0.0
    limit = 200.0 - control_mpc.FAN_SWEEP_MAX_SAG
    assert coldest_while_holding(sim) > limit - 1.0


def test_calibration_without_a_fan():
    sim = Sim(plant=Plant())
    values, _ = sim.calibrate()
    assert values["fan_ambient_transfer"] == ""
    assert float(values["ambient_transfer"]) == pytest.approx(0.10, rel=0.05)


######################################################################
# Configuration
######################################################################


def mpc_profile_from_config(**options):
    fileconfig = configparser.RawConfigParser()
    fileconfig.add_section("extruder")
    for option, value in {
        "control": "mpc",
        "heater_power": "50",
        **options,
    }.items():
        fileconfig.set("extruder", option, value)
    printer = SimpleNamespace(lookup_object=lambda name, default=None: default)
    config = configfile.ConfigWrapper(printer, fileconfig, {}, "extruder")
    manager = heaters.Heater.ProfileManager.__new__(
        heaters.Heater.ProfileManager
    )
    manager.outer_instance = SimpleNamespace(config=config)
    manager.profiles = {}
    return manager._init_profile(config, "default")


def test_config_fan_table_formats():
    profile = mpc_profile_from_config(fan_ambient_transfer="0.1, 0.2, 0.3")
    assert profile["fan_ambient_transfer"] == [
        (0.0, 0.1),
        (0.5, 0.2),
        (1.0, 0.3),
    ]
    profile = mpc_profile_from_config(
        fan_ambient_transfer="0:0.1, 0.2:0.15, 1:0.3"
    )
    assert profile["fan_ambient_transfer"] == [
        (0.0, 0.1),
        (0.2, 0.15),
        (1.0, 0.3),
    ]
    assert mpc_profile_from_config()["fan_ambient_transfer"] == []
    with pytest.raises(configfile.error, match="fan_ambient_transfer"):
        mpc_profile_from_config(fan_ambient_transfer="0:0.1, 2:0.3")


def test_config_heater_power_formats():
    assert mpc_profile_from_config()["heater_power"] == [(0.0, 50.0)]
    profile = mpc_profile_from_config(heater_power="180:72, 240:65, 300:60")
    assert profile["heater_power"] == [
        (180.0, 72.0),
        (240.0, 65.0),
        (300.0, 60.0),
    ]
    with pytest.raises(configfile.error, match="heater_power"):
        mpc_profile_from_config(heater_power="0")
