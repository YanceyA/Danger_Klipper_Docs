# STM32F411 Bring-up Review Plan

## Context
- Branch adds STM32F411 (WeAct Blackpill) bring-up support.
- Reviewed commits: `fd048689`, `5dfaaf35`, and `747f3325`.

## Commit Review
### `fd048689` – Adjust STM32F411 defaults and pin availability
- Adds `MACH_STM32F411` to `src/stm32/Kconfig` with CPU, flash, RAM, and clock defaults plus a 25 MHz HSE preference.
- Extends `scripts/flash_usb.py` so `stm32f411xe` targets reuse the STM32F4 flashing path.
- Updates ADC/GPIO/PWM code to align F411 with F401 capabilities, keeping the shared pin/timer tables consistent across STM32F4 parts.
- Extends `src/stm32/stm32f4.c` so F411 shares the existing F401 PLL/clock flow without altering other STM32F4 variants.
- Risks: ensuring timer/baud calculations on F411 match expectations, and filling in automated build/docs updates for the new target.

### `5dfaaf35` – Map STM32F411 macro to xE variant
- Ensures the legacy `STM32F411` define promotes to `STM32F411xE` so vendor headers resolve correctly.
- Low-risk compatibility tweak.

### `747f3325` – Treat STM32F411 like F401 for clock setup
- Brings F411 into the existing F401 PLL/clock selection paths in `src/stm32/stm32f4.c`.
- Provides the conditional scaffolding relied upon by the later refactor.

## Gap Analysis Toward MVP
- **Clock regression risk:** Need regression coverage on F401/F4x5/F446 designs to ensure the small F411-specific changes didn’t inadvertently alter shared behaviour.
- **Build/test scaffolding:** Missing CI config (`test/configs/stm32f411.config`) and user-facing sample configs/documentation for the new MCU selection.
- **Peripheral validation:** ADC temperature channel routing, PWM timers (TIM1/TIM3/TIM9), and USB CDC/DFU enumeration need hardware-in-the-loop confirmation at the new 96 MHz default.
- **Tooling/docs:** Documentation should describe menuconfig selections, clock source expectations (25 MHz HSE vs HSI), and flashing steps for supported boards.

## Proposed Review & Validation Plan
1. Add an automated STM32F411 build target (`make`, `make flash` dry-run) to CI to ensure toolchain coverage.
2. Perform smoke tests on WeAct Blackpill hardware covering USB DFU + CDC, one UART, PWM output, and ADC readback at 96 MHz.
3. Regression-test representative STM32F401/F405/F446 boards for UART baud accuracy, PWM frequencies, and USB stability under the shared clock logic.
4. Draft user documentation and example configuration snippets that explain setup, clock source choices, and flashing for the STM32F411 MVP.

## Open Questions
- Should GPIOE/TIMxE stay available when targeting STM32F411 packages that expose those pins (e.g. Nucleo-F411RE), and how should that selection be modeled?
- Do we anticipate internal-oscillator builds, and if so what validation do we need for the HSI-based PLL path?
- Is 96 MHz the right default, or should we provide an option for 84 MHz to align with existing timing expectations?

## Refinement Plan (Core MCU Focus)
1. **Limit `stm32f4.c` edits** to the minimum needed: add F411 into the existing F401 flow for `FREQ_PERIPH_DIV`, PLL setup, and prescaler handling without reshaping shared helpers.
2. **Keep shared peripheral tables intact** (`src/stm32/gpio.c:20`, `src/stm32/hard_pwm.c:140`) so all STM32F411 packages see the same pin availability as other F4 MCUs; defer Blackpill-specific pin reservations to board configs.
3. **Extend component-specific cases** by mirroring F401 conditionals (ADC temperature enable, PWM timer lists, Kconfig defaults), ensuring feature parity while confining edits to existing conditional blocks.
4. **Maintain tooling parity** by mapping `stm32f411xe` to existing STM32F4 flashing logic and adding the legacy macro alias in `lib/stm32f4/include/stm32f4xx.h`, without broader script changes.
5. **Defer board support** (pin maps, menuconfig snippets, docs) to a follow-up branch so the current changeset introduces only MCU-level support aligned with existing STM32F4 patterns.
