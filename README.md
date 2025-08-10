# Speedometer-Corrector-PIC-based
Digital speedometer corrector, based on a PIC 18F1320.

This is a Microchip PIC 18F1320 project for correcting a speedometer when you change tire size or differential gearing. It was initially designed for a Jeep Wrangler TJ (1997-2006). It may also work on other vehicles if they use a 5 volt squarewave signal from the speed sensor to the speedometer or to the PCM/ECU, within the range of 0 Hz to a little over 200 Hz. Some vehicles use a 12 volt speedometer signal, which would require several changes to this circuit to work. Some vehicles may use other signals, like a sinewave, or a CAN bus serial stream. This circuit will not work for those vehicles.

## Important Note

Installing this device involves splicing into your vehicle wiring, and altering the way a sensor reports to the PCM/ECU (engine computer). If you're uncomfortable with doing that, don't. An error could be very costly. It also requires you to burn a program into a PIC microcontroller. If you don't have the equipment to do that, it will cost money and a learning curve to get set up. It's probably not worth it unless you're going to play with other PIC microcontrollers for your own projects. On the other hand, the circuit is simple, with common through-hole parts, and the firmware is short and clean. Calibration is pretty easy.

## How It Works

The Speedo Corrector is a small box which goes between the Vehicle Speed Sensor on your transmission or transfer case and your engine computer (PCM/ECU). It alters the signal to correct for a change in tire diameter or gearing.

First, you take a drive to compare the indicated speed on your speedometer with a GPS speed. You'll do a simple calculation to get a "correction factor", which you'll enter into the Speedo Corrector on a set of 8 switches.

The Speedo Corrector measures the period of the incoming squarewave from the speed sensor, and produces a longer or shorter output squarewave, depending on the correction factor you set.

## License

The firmware is released as Open Source software.
Firmware License: GPL v3

The hardware is released as Open Hardware.
Hardware License: CERN-OHL-S v2

## Hardware

This simple circuit uses common through-hole parts. You can build it on a sea-of-holes proto board, wiring it point-to-point. Or you can make a real PC board, and the KiCad schematic and PCB layout, with Gerber files, is included.

## Firmware

The firmware is pretty short, and is written in C. You'll need the equipment to burn the firmware into the PIC chip.  Microchip MPLAB X IDE v6.20 (free), Microchip XC8 v3.00 compiler (free), Microchip PICkit3 or 4 (costs money) or equivalent.

## Where can I get one?

You'll have to acquire the parts and build it yourself.

## Related Projects

Speedometer Corrector (Arduino-based): An Arduino Pro Mini based speedometer corrector circuit.

https://github.com/OldBuzz/Speedometer-Corrector-Arduino-based

(I previously built an Arduino-based speedometer corrector, based on Tom M'Guinness's design. It works well, and I ran it in my Jeep for over two years.)
