"""MPC controller and calibration tests against a simulated hotend."""

import math
from types import SimpleNamespace

import pytest
from klippy_testing.mpc_sim import FakeFan, Plant, Sim

from klippy.extras import (
    control_mpc,
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
# Fan model
######################################################################


def test_fan_lookup_uses_requested_value_not_scaled_speed():
    # Calibration indexes the table by the requested fan value, so a fan
    # with max_power < 1 must not be looked up by its scaled speed.
    fan = FakeFan(max_power=0.5)
    sim = Sim(fan=fan)
    control = hot_control(sim, fan_ambient_transfer=[0.10, 0.20, 0.30])
    fan.set_speed(1.0)
    control.temperature_update(sim.time + 0.3, 200.0, 200.0)
    assert control.last_loss_ambient == pytest.approx(
        0.30 * (control.state_block_temp - control.state_ambient_temp)
    )


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
