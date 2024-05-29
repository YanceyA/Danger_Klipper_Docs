# Sensorless Homing
Sensorless homing has been integrated into Danger Klipper and no longer requires sensorless homing macros to set currents, error check movements, and configure movement patterns. For DK smart sensorless a simple configuration will suffice.

This document is supplemental and in some places superceedes the sensorless homing information contained in [https://github.com/DangerKlippers/danger-klipper/blob/master/docs/TMC_Drivers.md]


Basic Outline:
- Setup config
  
- Determine homing values
  
- First Homing



Updated guidance on:
homing currents
-Home at 50%-100%
-Different for various drivers
-AWD guidance to disable steppers
run current
current reset
homing speeds
-Reccomended to home at higher speeds than Klipper (40-100mm/s)
second homings
retract distance

Setting up alternative homing methods
-Y before X
-No need for homing override



DK config
#homing_elapsed_distance_tolerance: 0.5
#   Tolerance (in mm) for distance moved in the second homing. Ensures the
#   second homing distance closely matches the `min_home_dist` when using
#   sensorless homing. The default is 0.5mm.

stepper:
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
#use_sensorless_homing:
#   If true, disables the second home action if homing_retract_dist > 0.
#   The default is true if endstop_pin is configured to use virtual_endstop

TMC:
#run_current:
#home_current:
#current_change_dwell_time:
