# G2FD — three-channel CAN FD gs_usb adapter firmware

Open-source firmware for a three-channel CAN FD USB adapter built on the
Artery AT32F456ZET7 (Cortex-M4). The device speaks the
[gs_usb](https://github.com/candle-usb/candleLight_fw) (candleLight) protocol
and enumerates as `0x1209:0x6868` (PID assigned under the pid.codes
open-source hardware registry):

- **Linux** — SocketCAN via the in-kernel `gs_usb` driver; the channels appear
  as `can0`/`can1`/`can2`. A one-line patch adding the ID to the kernel
  `gs_usb_table` is pending upstream (see below).
- **Windows** — WinUSB via Microsoft OS 2.0 descriptors, driver-free; works
  with [cangaroo](https://github.com/HubertD/cangaroo) and gs_usb-compatible
  tools (e.g. python-can's `gs_usb` backend).

## Features

- 3 × CAN FD channels
  - Channel mapping: `ch0 = CAN3`, `ch1 = CAN2`, `ch2 = CAN1`
  - Default bit timing: 500 kbit/s arbitration / 2 Mbit/s data (75 % sample
    point, configurable per channel via gs_usb bit-timing commands)
  - ISO CAN FD mode enabled; hw timestamping on received frames
- Frame-loss-free design: dual SRAM ring queues between the CAN ISRs and the
  USB bulk endpoints; large shared data arrays live in a single 64 KiB on-chip
  SRAM arena
- USB: OTG FS device, single bulk-IN (`0x81`) / bulk-OUT (`0x02`) interface
- LED indicators for per-channel RX/TX activity and error states
- Slim, dependency-free codebase: no RTOS, no filesystem, no logging

## Repository layout

```
├── CMakeLists.txt          # ARM GCC build (CMake)
├── cmake/                  # toolchain file for arm-none-eabi-gcc
├── gcc/                    # GCC linker script (256 KiB flash / 144 KiB RAM)
├── mdk_v5/                 # Keil MDK project (AC5) + scatter file
├── src/                    # application code (CAN, USB, gs_usb protocol, LED)
├── inc/                    # application headers
├── libraries/              # Artery AT32F45x CMSIS + peripheral drivers
└── middlewares/            # Artery USB device stack + gs_usb class
```

## Building

The image is linked at flash base `0x08000000` and is a **standalone**
application — there is no bootloader handshake, no version record, no
signature. Both build systems produce the same layout.

### Keil MDK (ARM Compiler 5)

1. Open `mdk_v5/gs_usb_device.uvprojx` (device: AT32F456ZET7,
   pack: `ArteryTek.AT32F45x_DFP`).
2. **Project → Clean Targets**, then **Rebuild** (required after switching
   images — the flash programming algorithm caches).
3. Output: `mdk_v5/objects/gs_usb_device.hex`.

### CMake + ARM GCC

Requires `arm-none-eabi-gcc` (12.x tested) and CMake ≥ 3.15. If the toolchain
is not on `PATH`, pass its `bin` directory:

```sh
cmake -B gcc/build -DARM_GCC_TOOLCHAIN_DIR=C:/path/to/arm-gcc/bin
cmake --build gcc/build -j
```

Output: `gcc/build/g2fd.elf` / `.hex` / `.bin` (~22 KiB code, ~68 KiB data).

## Flashing

> **WARNING — the image is programmed at `0x08000000` and overwrites whatever
> is already there, including any on-board bootloader / updater firmware.**
> After flashing this firmware the device will no longer run the factory
> bootloader. This is intentional for the open-source build: the device is a
> pure gs_usb adapter afterwards.

- **Keil**: flash directly from the IDE (AT-Link / J-Link / CMSIS-DAP), make
  sure `Clean` + `Rebuild` was done first.
- **Command line**: flash `g2fd.hex` or `g2fd.bin` with any SWD programmer
  supported by your toolchain (e.g. OpenOCD `at32f45x` target, J-Flash,
  STM32CubeProgrammer with generic Cortex-M settings).

Power-cycle the board after flashing.

## Host-side usage

### Linux (SocketCAN)

```sh
ip link set can0 up type can bitrate 500000 dbitrate 2000000 fd on
candump can0
cansend can0 123##1DEADBEEF
```

`0x1209:0x6868` is not in the mainline kernel `gs_usb` device table yet — a
one-line patch for `drivers/net/can/usb/gs_usb.c` is pending upstream. Until a
kernel including it ships, rebuild the `gs_usb` module with the ID added, or
bind it manually.

### Windows (cangaroo)

Build or download [cangaroo](https://github.com/HubertD/cangaroo); the device
enumerates driver-free (WinUSB) and is detected automatically.

## Hardware notes

- MCU: AT32F456ZET7 (Cortex-M4, 512 KiB flash — the firmware uses the lower
  256 KiB window), 144 KiB SRAM
- Transceivers on CAN1/CAN2/CAN3 with standby pins driven low (normal mode);
  pin mapping is in `src/bsp_can_init.c`
- USB: OTG FS device, 48 MHz device clock derived from PLLU

## Firmware behavior notes

- The gs_usb mode flags are accepted, but only START/RESET and
  `BERR_REPORTING` change device state; the CAN controllers always run in
  normal (communicate) mode.
- Linux channels enumerate in reverse hardware order (`can0` = CAN3,
  `can1` = CAN2, `can2` = CAN1); label your cabling accordingly.

## License

This project's own code (`src/`, `inc/`, build files) is released under the
[MIT License](LICENSE). The Artery peripheral library, CMSIS port and USB
stack under `libraries/` and `middlewares/usb_drivers/` remain under
Artery Technology's BSP license terms as shipped by Artery (see `LICENSE`
third-party notes and the file headers).
