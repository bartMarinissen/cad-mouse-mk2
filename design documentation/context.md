**The Device and Its Purpose**
The device is a 6-Degrees-of-Freedom (6DOF) magnetic 3D mouse. It is designed to track the precise real-time pose—both the translation (X, Y, Z) and rotation (pitch, roll, yaw)—of a manipulatable knob. Devices like this are typically used as secondary input peripherals for navigating 3D space in CAD and modeling software.

**The Base Project**
Our work is based on an existing open-source repository: `sb-ocr/cad-mouse-mk2`. The firmware for this project is built on the Earle Philhower Arduino core and operates on a classic, non-blocking embedded "superloop" architecture. The base project successfully handles the peripheral tasks: hardware initialization, I2C/SPI bus management, LED ring control, and an asynchronous USB HID stack that formats and sends reports to the host PC at 100Hz–125Hz.

**The Hardware**

* **Microcontroller:** A Raspberry Pi RP2040 (a dual-core ARM Cortex-M0+). Notably, this chip lacks a hardware Floating Point Unit (FPU), meaning all floating-point math is handled in software.
* **Sensors:** 3x Infineon TLI493D-A2B6 3D Hall-effect sensors, which measure the magnetic flux density (in milliTeslas) across three axes and are read via an I2C bus. Datasheet and manual available in this dir.
* **Magnets:** 3x axially polarized 6x6mm cylindrical neodymium magnets.

**The Physical Layout**
The internal geometry relies on a 3-magnet, 3-sensor setup:

* The three magnets are embedded inside the movable knob, arranged in an equilateral triangle with a side length of approximately 2 cm.
* The three Hall-effect sensors are statically mounted on the PCB beneath the knob.
* When the knob is at its neutral, resting position, the sensors sit exactly 6 mm directly below their corresponding magnets.

**The Magnet Sizes and Properties**
The physical hardware relies on three **6x6mm cylindrical neodymium magnets** (rather low grade in my case) embedded in the knob.

* **Axial Polarization:** These cylinders are axially magnetized, meaning their north and south poles are located on the flat top and bottom circular faces. This is mathematically critical: it guarantees the magnetic field they project is radially symmetric around their local Z-axis, which allows the forward model (the 2D bicubic interpolator) to cleanly predict the field regardless of how the knob is twisted (yawed).

* **The Origin:** The fixed world origin `(0,0,0)` is located directly on the PCB, exactly at the center of the equilateral triangle formed by the three Hall-effect sensors. The world Z-axis points straight up toward the physical knob. The X axis is side to side (through the buttons); the Y axis is front to back (through the USB-C port) — see `firmware/include/Config.h`.

* **The frames** We have two frames: the world frame, and the knob frame. The sensors are in the world frame and don't move when the knob does. Their axes align with the axes explained above. The magnets are in the knob frame, and move with the knob they are positioned vertically (with their axis of symmetry parallel to Z). The nominal locations of the sensors and the magnets are recorded in `firmware/include/magnet_model/positions.h`. A calibration may record tiny ofsets on the locations and orientations of both. (as of writing, calibration records an offset and a rotation for the magnets)

**The PlatformIO Usage**
The firmware is written in C++ and uses **PlatformIO** (via the Earle Philhower Arduino core) as its build system rather than the standard Arduino IDE. PlatformIO is strictly necessary in this context because of the heavy mathematical load of the pose solver.

for claude:
A previous session concluded otherwise from a 403 on
  `api.github.com`, but the toolchain ships from
  `github.com/.../releases/download/`, a different host. Check with
  `pio pkg install` before believing you can't build.

* **Compiler Optimization:** The RP2040 lacks a hardware Floating Point Unit (FPU), so the heavy Gauss-Newton solver is entirely CPU-bound. PlatformIO’s `platformio.ini` configuration file lets us override default build flags; the current build uses `-O2` (up from Arduino's default size optimization `-Os`), and manages toolchain quirks (like avoiding unsupported Link-Time Optimization `-flto` flags, which break arduino-pico's soft-float symbol wrapping). `-O3` was measured (see `TODO/Performance.md`) to cut executed instructions further but costs several KB of RAM the interpolation table needs, so it's not the default.
