# Adafruit_Protomatter

An Arduino library for HUB75-style RGB LED matrices, targeted at 32-bit
microcontrollers using "brute force" GPIO (that is, not relying on DMA or
any specialized peripherals beyond timer interrupts, goal being portability).
This does assume the existence of 32-bit GPIO ports with bit SET and CLEAR
registers. A bit TOGGLE register (if available) can improve performance.

This might supersede the RGBmatrixPanel library on non-AVR devices, as the
older library has painted itself into a few corners. The newer library uses
a single constructor for all matrix setups, handling parallel chains,
various matrix sizes and chain lengths, and variable bit depths from 1 to 6
(refresh rate is a function of all of these). Note however that it is
NOT A DROP-IN REPLACEMENT for RGBmatrixPanel. The constructor is entirely
different, and there are several changes in the available functions. Also,
all colors in the new library are specified as 5/6/5-bit RGB (as this is
what the GFX library GFXcanvas16 type uses, aimed at low-cost color LCD
displays), even if the matrix is configured for a lower bit depth (colors
will be decimated/quantized in this case).

It does have some new limitations, mostly significant RAM overhead (hence
no plans for AVR port) and that all RGB data pins and the clock pin MUST be
on the same PORT register (e.g. all PORTA or PORTB, can't intermix). RAM
overhead is somewhat reduced (but still large) if those pins are all in a
single 8-bit byte within the PORT (they do not need to be contiguous or
sequential within this byte, if for instance it makes PCB routing easier,
but they should all aim for a single byte). Other pins (matrix address lines,
latch and output enable) can reside on any PORT or bit.

Name is likely to change as it's nondescriptive and tedious to type in code.
It's a reference to a line in Star Trek III, "I used protomatter in the
Genesis matrix."
