pi-blaster
==========

This project enables PWM on the GPIO pins you request of a Raspberry Pi. The
technique used is extremely efficient: does not use the CPU and gives very
stable pulses.

Pi-blaster project is based on the excellent work of Richard Hirst for
[ServoBlaster](https://github.com/richardghirst/PiBits).


## How to install

### Debian system

Thanks to [Boyuan Yang](https://github.com/hosiet), we now have a Raspbian package for
pi-blaster. You can install it with apt-get:

    sudo apt-get install pi-blaster

Note that the Debian package might be slightly out of date. For the most recent
version, compile from source or make your own Debian package (see below).

### Arch Linux

Thanks to [Patrick Wozniak](https://github.com/patlux), you can easily install pi-blaster
on archlinux with:

    yaourt -S pi-blaster-git

And activate the systemd-service with:

    sudo systemctl enable pi-blaster

### Build your own deb package and install with dpkg

The Debian package relies on systemd which means you must have Raspbian 8 or
later (aka "Jessie"). Run `cat /etc/debian_version`) to check what version
you are currently running.

Install the debian tools required to compile and prepare the package:

    sudo apt-get install debhelper dh-autoreconf dh-systemd dpkg-dev \
      init-system-helpers autoconf

And build the package:

    dpkg-buildpackage -us -uc -i && sudo dpkg -i ../pi-blaster*.deb


### Build and install directly from source

The build environment is based on Autotools to allow for compilation on the
Raspberry Pi, or cross-compilation.

You may need to install `autoconf` to build. On Raspbian and other Debian based
systems this can be achieved with:

    sudo apt-get install autoconf

Building is extremely simple:

    ./autogen.sh
    ./configure
    make

To start pi-blaster and have it relaunched automatically on every reboot:

    sudo make install

Or to start pi-blaster manually run:

    sudo ./pi-blaster

And to uninstall, simply run:

    sudo make uninstall

This will stop pi-blaster and prevent it from starting automatically on the next
reboot.

## How to use

pi-blaster creates a special file (FIFO) in `/dev/pi-blaster`. Any application
on your Raspberry Pi can write to it (this means that only pi-blaster needs to
be root, your application can run as a normal user).

**Important: when using pi-blaster, the GPIO pins you send to it are configured
as output.**

To set the value of a PIN, you write a command to `/dev/pi-blaster` in the form
`<GPIOPinName>=<value>` where `<value>` must be a number between 0 and 1
(included).

You must use the GPIO number (BCM xx in the diagram below).

[![List of pins thanks to pinout.xyz](pinout.xyz.png)](http://pinout.xyz)


Examples:

  * To completely turn on GPIO pin 17:

    echo "17=1" > /dev/pi-blaster

  * To set GPIO pin 17 to a PWM of 20%

    echo "17=0.2" > /dev/pi-blaster

  * To turn off GPIO pin 17:

    echo "17=0" > /dev/pi-blaster

  * To release a pin so it can be used as digital GPIO or an input:

    echo "release 17" > /dev/pi-blaster

### NodeJS Library

NodeJS users can use [pi-blaster.js](https://github.com/sarfata/pi-blaster.js).

### C#

A C# example was contributed by [Vili
Volcini](https://plus.google.com/109312219443477679717/posts). It is available
on [this stackoverflow
thread](http://stackoverflow.com/questions/17241071/writing-to-fifo-file-linux-monoc).

## How to adjust the frequency and the resolution of the PWM

On startup, pi-blaster gives you the frequency of the PWM, the number of steps
that you can control, the maximum and the minimum period of a pulse.

    sudo ./pi-blaster
    Using hardware:               PWM
    Number of channels:           8
    PWM frequency:                100 Hz
    PWM steps:                    1000
    Maximum period (100  %):      10000us
    Minimum period (0.100%):      10us

You can adjust those by changing a few defines at the top of the source code:

 * `NUM_SAMPLES`: The number of steps
 * `SAMPLE_US`: The time of one step (minimum period)

If you do not need a resolution of 1000 steps (approximately equivalent to a 10
bit DAC), then you can reduce the number of samples or increase the duration of
the steps.

Richard Hirst who wrote the original code recommended not going below 2us for
`SAMPLE_US`.

## Options

To override the default list of supported GPIO pins and specify fewer (or more)
you can specify a comma separated list of GPIO numbers. This is also the default
list:

    --gpio 4,17,18,27,21,22,23,24,25

To use the BCM2835's PCM peripheral instead of its PWM peripheral to time the
DMA transfers, pass the option:

    --pcm

This is useful if you are already using the chip's PWM peripheral, for example
for audio output.

To invert the pulse (off = pin HIGH, pulse = pin LOW), use:

    --invert

This can be useful for common anode LEDs or other devices that expect an
active-low signal.

To keep pi-blaster running in the foreground without running as a daemon use:

    -D

To view help or version information, use:

    --help

    --version

## Warnings and other caveats

**Pins being used by pi-blaster will be configured as outputs. Do not plug
something on an input or you might destroy it!**

This daemon uses the hardware PWM generator of the raspberry pi to get precise
timings. This might interfere with your sound card output.  There is
experimental support for a PCM time-source. If you are interested, I suggest you
look at Richard Hirst original project (ServoBlaster) and try the `--pcm`
option.

## A practical example: high-power RGB lighting

This library was developed for TBideas high power LED driver. You can read more
about this project on [our blog][blog].

## Contributors

* Richard Hirst for [ServoBlaster](https://github.com/richardghirst/PiBits)
* Thomas Sarlandie (https://github.com/sarfata)
* Michael Vitousek (https://github.com/mvitousek/pi-blaster)
* Pete Nelson (https://github.com/petiepooo)
* Edgar Siva (https://github.com/edgarsilva)
* Alex Lennon (https://github.com/ajlennon)
* Lara Maia (https://github.com/LaraCraft304)
* Pattrick Hüper (https://github.com/phueper)
* Boyuan Yang (https://github.com/hosiet)
* penfold42 (https://github.com/penfold42)
* Thomas McVay (https://github.com/ThomasMcVay)

## Want to support this project?

The best way to contribute is to write code for the features you would like to
see and to make a pull-requests.

## License

Released under The MIT License.

Note: This project was initially released by Richard Hist under the GPL v3
License. Richard gave me explicit permission to distribute this derivative work
under the MIT License.

    Copyright (c) 2013 Thomas Sarlandie - Richard Hirst

    Permission is hereby granted, free of charge, to any person obtaining a copy
    of this software and associated documentation files (the "Software"), to
    deal in the Software without restriction, including without limitation the
    rights to use, copy, modify, merge, publish, distribute, sublicense, and/or
    sell copies of the Software, and to permit persons to whom the Software is
    furnished to do so, subject to the following conditions:

    The above copyright notice and this permission notice shall be included in
    all copies or substantial portions of the Software.

    THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
    IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
    FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
    AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
    LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
    FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS
    IN THE SOFTWARE.

[blog]:
http://www.tbideas.com/blog/2013/02/controling-a-high-power-rgb-led-with-a-raspberry-pi/
