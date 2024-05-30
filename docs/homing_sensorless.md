# Sensorless Homing
Sensorless homing has been integrated into Danger Klipper and no longer requires sensorless homing macros to set currents, error checks movements, and configure movement patterns. For DK smart sensorless a simple configuration will suffice.

dangerklipper sensorless homing has:
 - home_current to automatically change the stepper current and dwell time while homing
 - min_home_dist to backoff if traveled shorter and try again..
 - error if 2nd try is smaller than homing_retract_dist with configurable tolerance
 - no macros needed
- No macros required
- Simple config
- No need for homing override
- Run Current setup per axis
- Homing current setup per axis
- Second home to improved accuracy
  
the gist is, you can set homing current in the config block (no need for macros for that), and set retract distance (which doesnt work in klipper mainline)


This document is supplemental and in some places superceedes the sensorless homing information contained in [https://github.com/DangerKlippers/danger-klipper/blob/master/docs/TMC_Drivers.md]


Basic Outline:
- Setup config
  
- Determine homing values
  
- First Homing

What is actually reccomended method to set sensitiviy, speed, and currnet.


Tips:
- Home fast (80mm/s) to generate consistent EMF
- retract distance higher than min home dist

Deathsneeze: tl;dr: make sure printer.cfg / mcu jumper stuff is correct, dont try and home slow, start at high sens and walk it down till you get a consistent home without it skipping steps or false triggers, then reduce homing current to soften the bump. on 48v its even easier, as you only have like 2 values of sens that work, and you can tune how hard it bumps with both homing speed and homing current

TODO Updated guidance on:
homing currents
- Home at 50%-100% run currnet
- Different for various drivers
- AWD guidance to disable steppers X1/Y1 steppers before home

run currents

homing speeds
- Reccomended to home at higher speeds than Klipper reccomended (40-100mm/s)
- 
second homings

retract distance


Issues that can affect sensorless:
- Belt tension issues
- Motion binding, especially from static friction
- Motion binding from tempeature


Specific Stepper Driver Guidance:

TMC2209
-reccomend 1/2 running current

TMC2240

TMC2160

TMC5160
-homing current not required?

Motor Specific Guidance:

2004s

2504s

2804s



Setting up alternative homing methods
- Y before X


References:
Eric Zimmerman
[https://github.com/EricZimmerman/VoronTools/blob/main/Sensorless.md]
Klipper Sensorless

Clee Voron Sensorless Guide
[https://docs.vorondesign.com/community/howto/clee/sensorless_xy_homing.html]





DK config
```
homing_elapsed_distance_tolerance: 0.5
Tolerance (in mm) for distance moved in the second homing. Ensures the
second homing distance closely matches the `min_home_dist` when using
sensorless homing. The default is 0.5mm.
```

stepper
```
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
```

```
TMC:
#run_current:
#home_current:
#current_change_dwell_time:
```
