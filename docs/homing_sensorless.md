# Smart Sensorless Homing
Sensorless homing has been integrated into Danger Klipper (DK) and no longer requires a complex macros to set currents, error check movement, and configure movement patterns. A simple set of config options enables smart sensorless in DK.

# Key benefits of DK Smart Sensorless:
 - Home_current parameter to automatically reduce the stepper current and dwell time while homing.
 - Min_home_dist to backoff toolhead if traveled distance is shorter. Smart sensorless will try again.
 - If 2nd homing attempt movement is smaller than homing_retract_dist an error is produced. A tolerance for this distance is avaliable for configure.
 - No macros required
 - Easy to setup run and homing currents on per axis basis
 - Second home with configurable speed to improve accuracy of homing operation. 

# Quick Setup Guide
1. Remove any sensorless homing macros or macro functions currently configured.
2. Update [stepper_x] and [stepper_y] to at least include the following sensorless parameters with appropriate values.
   ```
   [stepper_x or stepper_y]
   endstop_pin: 
   homing_speed: 
   homing_retract_dist: 
   homing_retract_speed:
   min_home_dist: 
   use_sensorless_homing: True
   ```
3. Update [tmcxxxx stepper_x] and [tmcxxxx stepper_y] to at least include the following sensorless parameters with appropriate values.
   ```
   home_current:
   diag0_pin: 
   driver_SGT: 
   ```
5. Determine appropriate sensorless driver threshold homing values, driver_SGT. It is reccomended to retune the values even if coming from a functional macro based sensorless system.
6. Save the config and validate that the printer can home reliably both cold and hot.

## Sensorless driver threshold tuning guides
- Eric Zimmerman [https://github.com/EricZimmerman/VoronTools/blob/main/Sensorless.md]
- Clee Voron Sensorless Guide [https://docs.vorondesign.com/community/howto/clee/sensorless_xy_homing.html]
  

# Smart Sensorless Config Options

[stepper]
```
endstop_pin:
#   Endstop switch detection pin. If this endstop pin is on a
#   different mcu than the stepper motor then it enables "multi-mcu
#   homing". This parameter must be provided for the X, Y, and Z
#   steppers on cartesian style printers.
#   See config option for [tmcxxxx] section for details on configuring a
#   virtual pin for sensorless homing.
#homing_speed: 5.0
#   Maximum velocity (in mm/s) of the stepper when homing. The default
#   is 5mm/s.
#homing_retract_dist: 5.0
#   Distance to backoff (in mm) before homing a second time during
#   homing. If `use_sensorless_homing` is false, this setting can be set
#   to zero to disable the second home. If `use_sensorless_homing` is
#   true, this setting can be > 0 to backoff after homing. The default
#   is 5mm.
#homing_retract_speed:
#   Speed to use on the retract move after homing in case this should
#   be different from the homing speed, which is the default for this
#   parameter
#min_home_dist:
#   Minimum distance (in mm) for toolhead before sensorless homing. If closer
#   than `min_home_dist` to endstop, it moves away to this distance, then homes.
#   If further, it directly homes and retracts to `homing_retract_dist`.
#   The default is equal to `homing_retract_dist`.
#second_homing_speed:
#   Velocity (in mm/s) of the stepper when performing the second home.
#   The default is homing_speed/2.
#homing_positive_dir:
#   If true, homing will cause the stepper to move in a positive
#   direction (away from zero); if false, home towards zero. It is
#   better to use the default than to specify this parameter. The
#   default is true if position_endstop is near position_max and false
#   if near position_min.
#use_sensorless_homing:
#   If true, disables the second home action if homing_retract_dist > 0.
#   The default is true if endstop_pin is configured to use virtual_endstop
```

[danger_options]
```
#homing_elapsed_distance_tolerance: 0.5
#   Tolerance (in mm) for distance moved in the second homing. Ensures the
#   second homing distance closely matches the `min_home_dist` when using
#   sensorless homing. The default is 0.5mm.
```

[tmcxxxx]
```
#home_current:
#   The amount of current (in amps RMS) to configure the driver to use
#   during homing procedures. The default is to not reduce the current.
#current_change_dwell_time:
#   The amount of time (in seconds) to wait after changing homing current.
#   The default is 0.5 seconds.
#driver_SGT: 0
#   Set the given register during the configuration of the TMC2130
#   chip. This may be used to set custom motor parameters. The
#   defaults for each parameter are next to the parameter name in the
#   above list.
#diag0_pin:
#diag1_pin:
#   The micro-controller pin attached to one of the DIAG lines of the
#   TMC2130 chip. Only a single diag pin should be specified. The pin
#   is "active low" and is thus normally prefaced with "^!". Setting
#   this creates a "tmc2130_stepper_x:virtual_endstop" virtual pin
#   which may be used as the stepper's endstop_pin. Doing this enables
#   "sensorless homing". (Be sure to also set driver_SGT to an
#   appropriate sensitivity value.) The default is to not enable
#   sensorless homing.
```

These config options are distilled from:
- [https://github.com/DangerKlippers/danger-klipper/blob/master/docs/Config_Reference.md#stepper ]
- [https://github.com/DangerKlippers/danger-klipper/blob/master/docs/Config_Reference.md#tmc-stepper-driver-configuration ]


# Sample Reference Config

- TODO Add sample reference config for ?TMC5160 and octopus PRO with 2804s?


# Setup Tips

## Run and homing currents
- TO DO

## Second homings
- TO DO

## Retract distance
- TO DO

## General 
- Home fast (>= 80mm/s) to generate consistent EMF with higher sensitivity
- Tune sensitivity until a consistent home without skipped steps or false triggers occurs.
- Reduce homing current at a set sensitivity to soften the homing bump. Start at run_current and work down.
- X and Y axis likely require different tuned parameters
- Retract distance should be greater than than min home dist.
- If retract is configured the second_homing_speed is reccomended to be the same as homing_speed.
- Sensorless need a dwell time to reset registers and ensure that any current changes are properly enabled.
- A sucessfull home travels [min_home_dist + tolerance].
- The tuning experience is different depending on the stepper drivers and motors being used. One size does not fit all.
- TODO: How does tuning sensorless on 24v vs 48v work?

> [!NOTE]
> For AWD systems a simple macro might be required to disable and re-enable X1/Y1 steppers for homing.
> TODO: ADD MACRO EXAMPLE

# Issues that can affect sensorless:
- Belt tension issues can cause inconsistent homing and poor performance across temperatures
- Motion binding, especially from static friction or temperature effects


# Specific Stepper Driver Guidance

## TMC2209
-Reccomend homing currnet to be 50% of running current

## TMC2240
- TO DO

## TMC2160
- TO DO

## TMC5160
-homing current not required?

# Motor Specific Guidance

## 2004s
- TO DO

## 2504s
- TO DO
  
## 2804s
- TO DO

# Advanced Homing Methods
- Currently a custom macro is required to home Y before X.
- TODO: ADD MACRO EXAMPLE

