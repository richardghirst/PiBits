pi-blaster
==========

This project enables PWM on all the GPIO pins of a Raspberry Pi. The technique used is extremely efficient: does not use the CPU and gives very stable pulses.

This project is based on the excellent work of Richard Hirst for ServoBlaster: https://github.com/richardghirst/PiBits

## How to build and install

This project is only distributed as source files. Building it is extremely simple.

    make

To start pi-blaster and have it relaunched automatically on every reboot:

    sudo make install

## How to start manually

To start pi-blaster manually run:

    sudo ./pi-blaster
    
## How to uninstall

Simply run:

    sudo make uninstall
    
This will stop pi-blaster and prevent it from starting automatically on the next reboot.

## How to use

pi-blaster creates a special file (FIFO) in `/dev/pi-blaster`. Any application on your Raspberry Pi can write to it (this means that only pi-blaster needs to be root, your application can run as a normal user).

**Important: when using pi-blaster, all the pins are configured as output.**

To set the value of a PIN, you write a command to `/dev/pi-blaster` in the form <channel>=<value> where <value> must be a number between 0 and 1 (included).

    Channel number    GPIO number   Pin in P1 header
          0               4             P1-7
          1              17             P1-11
          2              18             P1-12
          3              21             P1-13
          4              22             P1-15
          5              23             P1-16
          6              24             P1-18
          7              25             P1-22

Examples:
  * To completely turn off pin0: 

    echo "0=0" > /dev/pi-blaster

  * To completely turn on pin1:

    echo "1=1" > /dev/pi-blaster

  * To set pin1 to a PWM of 20%

    echo "1=0.2"

## How to adjust the frequency and the resolution of the PWM

On startup, pi-blaster gives you the frequency of the PWM, the number of steps that you can control, the maximum and the minimum period of a pulse.

    sudo ./pi-blaster
    Using hardware:                   PWM
    Number of channels:                 8
    PWM frequency:                 100 Hz
    PWM steps:                       1000
    Maximum period (100  %):      10000us
    Minimum period (0.100%):         10us  

You can adjust those by changing a few defines at the top of the source code:

 * `NUM_SAMPLES`: The number of steps
 * `SAMPLE_US`: The time of one step (minimum period)

If you do not neet a resolution of 1000 steps (approximately equivalent to a 10 bit DAC), then you can reduce the number of samples or increase the duration of the steps.

Richard Hirst who wrote the original code recommended not going below 2us for `SAMPLE_US`.

## Warnings and other caveats

**All the pins will be configured as outputs. Do not plug something on an input or you might destroy it!**

This daemon uses the hardware PWM generator of the raspberry pi to get precise timings. This might interfere with your sound card output.
There is experimental support for a PCM time-source. If you are interested, I suggest you look at Richard Hirst original project (ServoBlaster) and try the `--pcm` option.

## A practical example: high-power RGB lighting

This library was developed for TBideas high power LED driver. You can read more about this project on [our blog][blog].

## License

I usually publish my work under the CC-BY-SA license or the MIT License but this project was initially released by Richard as GPLv3 and so it stays GPLv3.

[blog]: http://www.tbideas.com/blog/2013/02/controling-a-high-power-rgb-led-with-a-raspberry-pi/