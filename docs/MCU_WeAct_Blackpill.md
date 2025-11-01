# WeAct Blackpill STM32F411 Guide

The WeAct "Blackpill" STM32F411 development board is a compact and
inexpensive controller that exposes nearly every GPIO from an
STM32F411CEU6. When paired with external stepper drivers it provides a
capable controller for smaller printers or experimental motion stages.
This guide summarizes the bootloader options, flashing methods, and pin
recommendations needed to integrate the board with Kalico.

## Hardware overview

* **MCU**: STM32F411CEU6 (100&nbsp;MHz Cortex-M4, 512&nbsp;KiB flash,
  128&nbsp;KiB SRAM)
* **Clocking**: 25&nbsp;MHz HSE, 32.768&nbsp;kHz LSE, internal PLL up to
  100&nbsp;MHz system clock.
* **USB**: Full-speed device on PA11/PA12 (micro USB connector).
* **Supply rails**: 5&nbsp;V on VBUS, on-board LDO provides 3.3&nbsp;V
  (max ~500&nbsp;mA available to peripherals). All GPIO are 3.3&nbsp;V only;
  level shifting is required for 5&nbsp;V peripherals.
* **Boot controls**: Dedicated BOOT0 pushbutton (or jumper) and NRST
  reset button next to the USB connector. BOOT1 is tied low by default
  and is accessible on PB2 if advanced boot flows are required.
* **I/O headers**: Dual 24-pin headers. See
  [`config/generic-weact-blackpill-stm32f411.cfg`](../config/generic-weact-blackpill-stm32f411.cfg)
  for a map of the silkscreen labels to STM32 port names.

Hardware revision 3.1 and newer additionally bring out PC6–PC10 on the
J2 header. Earlier revisions leave those signals unpopulated.

## Building Kalico firmware

Use `make menuconfig` and select the following options:

1. **Micro-controller Architecture**: `STMicroelectronics STM32`.
2. **Processor model**: `STM32F411`.
3. **Clock Reference**: `25 MHz crystal` (matches the WeAct board).
4. **Communication interface**:
   * `USB (on PA11/PA12)` for DFU or HID flashing via the on-board USB
     connector, or
   * `Serial (on USART1 PA10/PA9)` when wiring a USB-UART adapter to the
     header.
5. **Bootloader offset**:
   * `No bootloader` (flash offset 0x0000) for the ROM DFU
     bootloader or when using a hardware probe.
   * `16KiB bootloader` when using the STM32 HID bootloader ported for
     the Blackpill.

After saving the configuration run `make` to build the firmware. The
output binary is stored at `out/klipper.bin`.

## Flashing options

### System DFU (USB)

The STM32F411 system bootloader supports DFU without any custom
firmware. To enter DFU mode:

1. Hold the **BOOT0** button (or move the BOOT0 jumper to 3.3&nbsp;V).
2. Tap **NRST** while still holding BOOT0, then release BOOT0.
3. Connect the USB cable to the host.

The board enumerates as `0483:df11`. Flash with either `make flash`
(after configuring a DFU serial ID) or with `dfu-util` directly:

```sh
dfu-util -d 0483:df11 -a 0 -s 0x08000000:leave -D out/klipper.bin
```

Because the system bootloader does not reserve flash, keep the firmware
compiled for "No bootloader" (start address 0x08000000).

### STM32 HID bootloader

Users who prefer a USB mass-storage style workflow may flash the
[STM32 HID bootloader](https://github.com/Arksine/STM32_HID_Bootloader)
to the Blackpill. The HID bootloader consumes 16&nbsp;KiB of flash; build
Kalico with the `16KiB bootloader` offset so that `klipper.bin` starts at
0x08004000. Once installed, enter the HID bootloader by double-tapping
NRST. Upload the firmware with the `hid-flash` utility:

```sh
./hid-flash.py out/klipper.bin
```

### USART flashing (3.3&nbsp;V serial)

Wire a USB-to-UART adapter (3.3&nbsp;V logic) to USART1: connect the
adapter's TX to PA10 (RX1), RX to PA9 (TX1), and GND to GND. Place the
board into the system bootloader using BOOT0/NRST as described above and
use [`stm32flash`](https://sourceforge.net/projects/stm32flash/):

```sh
stm32flash -w out/klipper.bin -v -g 0x0 /dev/ttyUSB0
```

Serial flashing also requires the firmware to be built for `No
bootloader`.

### SWD/JTAG probes

A 4-pin SWD footprint is available near the USB connector. Any ST-Link,
J-Link, or Black Magic probe can program the device directly with
OpenOCD. This method is useful when recovering a board with corrupted
flash.

## Wiring guidelines

The sample configuration
[`generic-weact-blackpill-stm32f411.cfg`](../config/generic-weact-blackpill-stm32f411.cfg)
assigns commonly used functions to the header pins:

| Function         | Pin(s)                          | Notes |
| ---------------- | -------------------------------- | ----- |
| Stepper steps    | PB10 (X), PB0 (Y), PA6 (Z), PA4 (E) | Connect to STEP inputs on external drivers. |
| Stepper dirs     | PB2 (X), PB1 (Y), PA7 (Z), PA5 (E) | Invert with `!` prefix if driver requires. |
| Enables          | PB12 (X), PB13 (Y), PB11 (Z), PA15 (E) | Active-low outputs. |
| Endstops         | PC13 (X), PC14 (Y), PC15 (Z) | Enable pull-ups in firmware (`^` prefix). |
| Hotend heater    | PA8                             | Drives MOSFET via external gate driver or SSR. |
| Bed heater       | PA9                             | PWM capable. |
| Part cooling fan | PA10                            | Default `fan` output. |
| Hotend fan       | PB6                             | Configured as `[heater_fan nozzle_cooling_fan]`. |
| Thermistors      | PA0 (hotend), PA1 (bed)         | 3.3&nbsp;V ADC inputs. |
| USB/UART         | PA11/PA12 (USB), PA9/PA10 (USART1) | Choose one communication method. |

When driving heaters or steppers the Blackpill must control external
MOSFETs or driver modules—do not connect loads directly to the MCU pins.
Pay attention to common ground and provide adequate 5&nbsp;V and 3.3&nbsp;V
regulation for any attached peripherals.

For CAN or other advanced buses the board exposes spare timers on PB8
and PB9, and extra PWM outputs (PB3–PB7, PC6–PC10 on later revisions)
that can be repurposed as needed.

## Additional resources

* [WeAct Studio documentation repository](https://github.com/WeActStudio/WeActStudio.MiniSTM32F4x1)
  (hardware revisions, schematics, boot jumpers).
* [STM32F411 reference manual (RM0383)](https://www.st.com/resource/en/reference_manual/dm00119316.pdf)
  for detailed peripheral descriptions.
* [Kalico Benchmarks](Benchmarks.md#stm32f4-step-rate-benchmark)
  include notes on F4 performance expectations.
