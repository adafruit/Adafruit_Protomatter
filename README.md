# Adafruit_Protomatter [![Build Status](https://github.com/adafruit/Adafruit_Protomatter/workflows/Arduino%20Library%20CI/badge.svg)](https://github.com/adafruit/Adafruit_Protomatter/actions)

"I used protomatter in the Genesis matrix." - David Marcus, Star Trek III

Code for driving HUB75-style RGB LED matrices, targeted at 32-bit MCUs
using brute-force GPIO (that is, not relying on DMA or other specialized
peripherals beyond a timer interrupt, goal being portability).

# Matrix Concepts and Jargon

HUB75 RGB LED matrices are basically a set of six concurrent shift register
chains, each with one output bit per column, the six chains being red, green
and blue bits for two non-adjacent rows, plus a set of row drivers (each
driving the aforementioned two rows) selected by a combination of address
lines. The number of address lines determines the overall matrix height
(3 to 5 bits is common...as an example, 3 address lines = 2^3 = 8 distinct
address line combinations, each driving two rows = 16 pixels high). Address
0 enables rows 0 and height/2, address 1 enables rows 1 and height/2+1, etc.
Shift register chain length determines matrix width...32 and 64 pixels are
common...matrices can be chained to increase width, a 64-pixel wide matrix
is equivalent to two 32-pixel chained matrices, and so forth.

These matrices render only ONE BIT each for red, green and blue, they DO NOT
natively display full color and must be quickly refreshed by the driving
microcontroller, basically PWM-ing the intermediate shades (this in addition
to the row scanning that must be performed).

There are a few peculiar RGB LED matrices that have the same physical
connection but work a bit differently -- they might have only have three
shift register chains rather than six, or might use a shift register for
the row selection rather than a set of address lines. The code presented
here DOES NOT support these matrix variants. Aim is to provide support for
all HUB75 matrices in the Adafruit shop. Please don't submit pull requests
for these other matrices as we have no means to test them. If you require
this functionality, it's OK to create a fork of the code, which Git can
help keep up-to-date with any future changes here!

# Hardware Requirements and Jargon

The common ground for architectures to support this library:

* 32-bit device (e.g. ARM core, ESP32 and others)
* One or more 32-bit GPIO PORTs with atomic (single-cycle, not
  read-modify-write) bitmask SET and CLEAR registers. A bitmask TOGGLE
  register, if present, may improve performance but is NOT required.
* There may be performance or storage benefits if the architecture tolerates
  8-bit or word-aligned 16-bit accesses within the 32-bit PORT registers
  (e.g. writing just one of four bytes, rather than the whole 32 bits), but
  this is NOT a hardware requirement. Also, the library does not use any
  unaligned accesses (i.e. "middle word" of a 32-bit register), even if a
  device tolerates such.

# Software Components

This repository currently consists of:

* An Arduino C++ library (files Adafruit_Protomatter.cpp and
  Adafruit_Protomatter.h, plus the "examples" directory). The Arduino code
  is dependent on the Adafruit_GFX library.

* An underlying C library (files core.c, core.h and headers in the arch
  directory) that might be adaptable to other runtime environments (e.g.
  CircuitPython).

# Arduino Library

This supersedes the RGBmatrixPanel library on non-AVR devices, as the older
library has painted itself into a few corners. The newer library uses a
single constructor for all matrix setups, potentially handling parallel
chains (not yet fully implemented), various matrix sizes and chain lengths,
and variable bit depths from 1 to 6 (refresh rate is a function of all of
these). Note however that it is NOT A DROP-IN REPLACEMENT for RGBmatrixPanel.
The constructor is entirely different, and there are several changes in the
available functions. Also, all colors in the new library are specified as
5/6/5-bit RGB (as this is what the GFX library GFXcanvas16 type uses, being
aimed at low-cost color LCD displays), even if the matrix is configured for
a lower bit depth (colors will be decimated/quantized in this case).

It does have some new limitations, mostly significant RAM overhead (hence
no plans for AVR port) and (with a few exceptions) that all RGB data pins
and the clock pin MUST be on the same PORT register (e.g. all PORTA or PORTB
,can't intermix). RAM overhead is somewhat reduced (but still large) if
those pins are all in a single 8-bit byte within the PORT (they do not need
to be contiguous or sequential within this byte, if for instance it makes
PCB routing easier, but they should all aim for a single byte). Other pins
(matrix address lines, latch and output enable) can reside on any PORT or bit.

# C Library

The underlying C library is focused on *driving* the matrix and does not
provide any drawing operations of its own. That must be handled by
higher-level code, as in the Arduino wrapper which uses the Adafruit_GFX
drawing functions.

The C code has the same limitations as the Arduino library: all RGB data
pins and the clock pin MUST be on the same PORT register, and it's most
memory efficient (though still slightly gluttonous) if those pins are all
within the same 8-bit byte within the PORT (they do not need to be
contiguous or sequential within that byte). Other pins (matrix address lines,
latch and output enable) can reside on any PORT or bit.

When adapting this code to new devices (e.g. iMX) or new runtime environments
(e.g. CircuitPython), goal is to put all the device- or platform-specific
code into a new header file in the arch directory (or completely separate
source files, as in the Arduino library .cpp and .h). core.c contains only
the device-neutral bitbang code and should not have any "#ifdef DEVICE"- or
"#ifdef ENVIRONMENT"-like lines (exception for the 565 color conversion
functions, since the internal representation is common to both Arduino and
CircuitPython). Macros for things like getting a PORT register address from
a pin, or setting up a timer peripheral, all occur in the arch header files,
which are ONLY #included by core.c (to prevent problems like multiple
instances of ISR functions, which must be singularly declared at
compile-time).

Most macros and functions begin with the prefix **\_PM\_** in order to
avoid naming collisions with other code (exception being static functions,
which can't be seen outside their source file).

# Pull Requests

If you encounter artifacts (noise, sparkles, dropouts and other issues) and
it seems to resolve by adjusting the NOP counts, please do not submit this
as a PR claiming a fix. Quite often what improves stability for one matrix
type can make things worse for other types. Instead, open an issue and
describe the hardware (both microcontroller and RGB matrix) and what worked
for you. A general solution working across all matrix types typically
involves monitoring the signals on a logic analyzer and aiming for a 50%
duty cycle on the CLK signal, 20 MHz or less, and then testing across a
wide variety of different matrix types to confirm; trial and error on just
a single matrix type is problematic. Maintainers: this goes for you too.
Don't merge a "fix" unless you've checked it out on a 'scope and on tested
across a broad range of matrices.
