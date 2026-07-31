**The Device and Its Purpose**
The device is a 6-Degrees-of-Freedom (6DOF) magnetic 3D mouse. It is designed to track the precise real-time pose—both the translation (X, Y, Z) and rotation (pitch, roll, yaw)—of a manipulatable knob. Devices like this are typically used as secondary input peripherals for navigating 3D space in CAD and modeling software.

**The Base Project**
Our work is based on an existing open-source repository: `sb-ocr/cad-mouse-mk2`. The firmware for this project is built on the Earle Philhower Arduino core and operates on a classic, non-blocking embedded "superloop" architecture. The base project successfully handles the peripheral tasks: hardware initialization, I2C/SPI bus management, LED ring control, and an asynchronous USB HID stack that formats and sends reports to the host PC at 100Hz–125Hz.

**The Hardware**

* **Microcontroller:** A Raspberry Pi RP2040 (a dual-core ARM Cortex-M0+). Notably, this chip lacks a hardware Floating Point Unit (FPU), meaning all floating-point math is handled in software.
* **Sensors:** 3x Infineon TLI493D-A2B6 3D Hall-effect sensors, which measure the magnetic flux density (in milliTeslas) across three axes and are read via an I2C bus.
* **Magnets:** 3x axially polarized 6x6mm cylindrical neodymium magnets.

**The Physical Layout**
The internal geometry relies on a 3-magnet, 3-sensor setup:

* The three magnets are embedded inside the movable knob, arranged in an equilateral triangle with a side length of approximately 2 cm.
* The three Hall-effect sensors are statically mounted on the PCB beneath the knob.
* When the knob is at its neutral, resting position, the sensors sit exactly 6 mm directly below their corresponding magnets.

**The Magnet Sizes and Properties**
The physical hardware relies on three **6x6mm cylindrical neodymium magnets** (e.g., N52 grade) embedded in the knob.

* **Axial Polarization:** These cylinders are axially magnetized, meaning their north and south poles are located on the flat top and bottom circular faces. This is mathematically critical: it guarantees the magnetic field they project is radially symmetric around their local Z-axis, which allows the forward model (the 2D bicubic interpolator) to cleanly predict the field regardless of how the knob is twisted (yawed).
* **Manufacturing Tolerances:** In the context of the solver, the physical reality of these magnets introduces a "Phantom Tilt" problem. If one 6x6mm magnet is even 5% stronger than the others out of the factory, the solver will see a stronger magnetic field and mathematically assume that specific magnet must have been pushed closer to the sensor. This physical tolerance issue necessitates the use of a "blind bundle adjustment" calibration routine to mathematically measure and normalize the exact strength of all three magnets.
**Axis Orientation and Coordinate Frames**
The geometry of the mouse relies on a strict global and local coordinate system mapping:

* **The Origin:** The fixed global origin `(0,0,0)` is located directly on the PCB, exactly at the center of the equilateral triangle formed by the three Hall-effect sensors. The global Z-axis points straight up toward the physical knob. The Y axis is side to side (through the knobs)

**The PlatformIO Usage**
The firmware is written in C++ and uses **PlatformIO** (via the Earle Philhower Arduino core) as its build system rather than the standard Arduino IDE. PlatformIO is strictly necessary in this context because of the heavy mathematical load of the pose solver.

* **Compiler Optimization:** The RP2040 lacks a hardware Floating Point Unit (FPU), so the heavy Gauss-Newton solver is entirely CPU-bound. PlatformIO’s `platformio.ini` configuration file allows us to override default build flags (switching from Arduino's default size optimization `-Os` to aggressive speed optimization `-O3`), and manage toolchain quirks (like avoiding unsupported Link-Time Optimization `-flto` flags on the Windows toolchain).
* **Overclocking:** PlatformIO makes it trivial to safely overclock the RP2040 from its factory 133 MHz up to 250 MHz. Because the mathematical workload is purely CPU-bound, this single configuration change effectively halves the execution time of the solver with zero code changes.

**The Problem with the Original Code**
The primary motivation for our design intervention was a fundamental flaw in the original project's motion math. The original code naively assumed that the raw magnetic field strength readings ($B_x, B_y, B_z$) returned by the Hall-effect sensors were directly equivalent to the physical spatial coordinates ($X, Y, Z$) of the nearby magnets. Because magnetic fields decay non-linearly and curve through space, mapping field strength directly to Cartesian coordinates resulted in highly inaccurate and physically impossible pose estimations.

**What We Are Changing**
We are keeping the base project's structural skeleton—the hardware initialization, the sequential superloop flow, the LED state machine, and the asynchronous USB HID reporting stack. What we are completely gutting and replacing is the core motion control logic. We are stripping out the flawed location-mapping math to make way for a mathematically rigorous forward model and solver that can accurately infer the true physical pose of the knob based on the complex magnetic fields intersecting the sensors.

**Axis Orientation and Coordinate Frames**
The geometry of the mouse relies on a strict global and local coordinate system mapping:

* The Origin: The fixed global origin (0,0,0) is located directly on the PCB, exactly at the center of the equilateral triangle formed by the three Hall-effect sensors. The global Z-axis points straight up toward the physical knob.
 

