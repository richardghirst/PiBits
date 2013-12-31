                                ServoBlaster

This is software for the RaspberryPi, which provides an interface to drive
multiple servos via the GPIO pins.   You control the servo positions by sending
commands to the driver saying what pulse width a particular servo output should
use.  The driver maintains that pulse width until you send a new command
requesting some other width.

By default is it configured to drive 8 servos, although you can configure it to
drive up to 21.  Servos typically need an active high pulse of somewhere
between 0.5ms and 2.5ms, where the pulse width controls the position of the
servo.  The pulse should be repeated approximately every 20ms, although pulse
frequency is not critical.  The pulse width is critical, as that translates
directly to the servo position.

In addition to driving servos, ServoBlaster can be configured to generate pulse
widths between 0 and 100% of the cycle time, making it suitable for controlling
the brightness of up to 21 LEDs, for example.

The driver creates a device file, /dev/servoblaster, in to which you can send
commands.  The command format is either

     <servo-number>=<servo-position>
or
     P<header>-<pin>=<servo-position>

For the first format <servo-number> is the sevo number, which by default is a
number between 0 and 7, inclusive.  For the second format <header> is either
'1' or '5', depending on which header your servo is connected to, and <pin> is
the pin number on that header you have connected it to.  By default
<servo-position> is the pulse width you want in units of 10us, although that
can be changed via command line arguments, and can also be specified in units
of microseconds, or as a percentage of the maximum allowed pulse width.

So, if you want to set servo 3 to a pulse width of 1.2ms you could do this at
the shell prompt:

echo 3=120 > /dev/servoblaster

120 is in units of 10us by default, so that is 1200us, or 1.2ms.

Alternatively, using the second command format, if you had a servo connected to
P1 pin 12 you could set that to a width of 1.2ms as follows:

echo P1-12=120 > /dev/servoblaster

As an alternative to setting absolute servo positions, you can specify
adjustments relative to the current position via a '+' or '-' prefix.  For
example, the following would increase the servo pulse width by 10 step units:

echo P1-12=+10 > /dev/servoblaster

Beware that within the driver the servo position and any width adjustments are
rounded down to the nearest step-size, which is 10us by default.  So, if you
continually issued "0=+1us" commands, the servo would never move, and "0=-19us"
is treated the same as "0=-10us".  In addition, relative adjustments are
silently truncated to keep the servos within the allowed min/max range. 
If you set a servo width to 0 it turns off the servo output, without changing
the current servo position.

The code defaults to driving 8 servos, the control signals of which should be
connected to P1 header pins as follows:

     Servo number    GPIO number   Pin in P1 header
          0               4             P1-7
          1              17             P1-11
          2              18             P1-12
          3             21/27           P1-13
          4              22             P1-15
          5              23             P1-16
          6              24             P1-18
          7              25             P1-22

P1-13 is connected to either GPIO-21 or GPIO-27, depending on board revision.

Command line options described later in this document allow you to configure
which header pins are used for servo control, and what servo numbers they map
to.

Note that servoblaster must know at startup which header pins it is control of.
If you want to use a header pin that is not in the default list above, you
must use the appropriate command line option to inform servoblaster.

The driver also creates a /dev/servoblaster-cfg file, which describes which
pins servoblaster is currently configured to use.

When the driver is first loaded the GPIO pins are configure to be outputs, and
their pulse widths are set to 0.  This is so that servos don't jump to some
arbitrary position when you load the driver.  Once you know where you want your
servos positioned, write a value to /dev/servoblaster to enable the respective
output.  When the driver is unloaded it attempts to shut down the outputs
cleanly and revert the GPIO pins to their original configuration, rather than
cutting some pulse short and causing a servo position to jump.

The driver takes note of how many servos you have configured and distributes
the start time for the servo pulses evenly across the cycle time.  This way
the driver aims to ensure that only one servo pulse will be active at a time,
which should help minimise total drive current needed.

In the following description it refers to using the PWM peripheral.  For the
user space implementation it can instead use the PCM peripheral, see below
for details.  Using PCM is typically a better option, as the 3.5mm jack also
uses the PWM peripheral, so ServoBlaster can interfere with sound output.

The driver works by setting up a linked list of DMA control blocks with the
last one linked back to the first, so once initialised the DMA controller
cycles round continuously and the driver does not need to get involved except
when a pulse width needs to be changed.  For a given period there are two DMA
control blocks; the first transfers a single word to the GPIO 'clear output'
register, while the second transfers some number of words to the PWM FIFO to
generate the required pulse width time.  In addition, interspersed with these
control blocks is one for each configured servo which is used to set an output.

While the driver does use the PWM peripheral, it only uses it to pace the DMA
transfers, so as to generate accurate delays.

I used Panalyzer running on one Pi to monitor the servo outputs from a second
Pi.  The pulse widths and frequencies seem very stable, even under heavy SD
card use.  This is expected, because the pulse generation is effectively
handled in hardware and not influenced by interrupt latency or scheduling
effects.

The driver uses DMA channel 14, and PWM channel 1.  It makes no attempt to
protect against other code using those peripherals.  It sets the relevant GPIO
pins to be outputs when the driver is loaded, so please ensure that you are not
driving those pins externally.

I would of course recommend some buffering between the GPIO outputs and the
servo controls, to protect the Pi.  That said, I'm living dangerously and doing
without :-)  If you just want to experiment with a small servo you can probably
take the 5 volts for it from the header pins on the Pi, but I find that doing
anything non-trivial with four servos connected pulls the 5 volts down far
enough to crash the Pi.



There are two implementations of ServoBlaster; a kernel module based one, and a
user space daemon.  The kernel module based one is the original, but is more of
a pain to build because you need a matching kernel build.  The user space
daemon implementation is much more convenient to use and now has rather more
features than the kernel based one.  I would strongly recommend you use the
user space implementation.

The kernel module implementation is in the subdirectory 'kernel', while the
user space implementation can be found in subdirectory 'user'.

Details specific to each implementation are provided in separate sections
below.


The user space daemon
---------------------

To use this daemon grab the servod.c source and Makefile and:

$ make servod
$ ./servod --help

Usage: ./servod <options>

Options:
  --pcm               tells servod to use PCM rather than PWM hardware
                      to implement delays
  --idle-timeout=Nms  tells servod to stop sending servo pulses for a
                      given output N milliseconds after the last update
  --cycle-time=Nus    Control pulse cycle time in microseconds, default
                      20000us
  --step-size=Nus     Pulse width increment step size in microseconds,
                      default 10us
  --min={N|Nus|N%}    specifies the minimum allowed pulse width, default
                      50 steps or 500us
  --max={N|Nus|N%}    specifies the maximum allowed pulse width, default
                      250 steps or 2500us
  --invert            Inverts outputs
  --dma-chan=N        tells servod which dma channel to use, default 14
  --p1pins=<list>     tells servod which pins on the P1 header to use
  --p5pins=<list>     tells servod which pins on the P5 header to use

where <list> defaults to "7,11,12,13,15,16,18,22" for p1pins and
"" for p5pins.  p5pins is only valid on rev 2 boards.

min and max values can be specified in units of steps, in microseconds,
or as a percentage of the cycle time.  So, for example, if cycle time is
20000us and step size is 10us then the following are equivalent:

          --min=50   --min=500us    --min=2.5%

For the default configuration, example commands to set the first servo
to the mid position would be any of:

  echo 0=150 > /dev/servoblaster        # Specify as a number of steps
  echo 0=50% > /dev/servoblaster        # Specify as a percentage
  echo 0=1500us > /dev/servoblaster     # Specify as microseconds
  echo P1-7=150 > /dev/servoblaster     # Using P1 pin number rather
  echo P1-7=50% > /dev/servoblaster     # ... than servo number
  echo P1-7=1500us > /dev/servoblaster

Servo adjustments may also be specified relative to the current
position by adding a '+' or '-' prefix to the width as follows:

  echo 0=+10 > /dev/servoblaster
  echo 0=-20 > /dev/servoblaster

$ sudo ./servod

Board revision:                  1
Using hardware:                PWM
Using DMA channel:              14
Idle timeout:             Disabled
Number of servos:                8
Servo cycle time:            20000us
Pulse increment step size:      10us
Minimum width value:            50 (500us)
Maximum width value:           250 (2500us)
Output levels:              Normal

Using P1 pins:               7,11,12,13,15,16,18,22

Servo mapping:
     0 on P1-7           GPIO-4
     1 on P1-11          GPIO-17
     2 on P1-12          GPIO-18
     3 on P1-13          GPIO-21
     4 on P1-15          GPIO-22
     5 on P1-16          GPIO-23
     6 on P1-18          GPIO-24
     7 on P1-22          GPIO-25

$ 

The prompt will return immediately, and servod is left running in the
background.  You can check it is running via the "ps ax" command.  If you want
to stop servod, the easiest way is to run:

$ sudo killall servod

Note that use of PWM will interfere with 3.5mm jack audio output.  Instead
of using the PWM hardware, you can use the PCM hardware, which is less likely
to cause a conflict.  Please be aware that the PCM mode is very lightly tested
at present.

Some people have requested that a servo output turns off automatically if no
new pulse width has been requested recently, and I've had two reports of
servos overheating when driven for long periods of time.  To support this
request, servod implements an idle timeout which can be specified at
module load time.  The value is specified in milliseconds.

Typical small servos take a few 100 milliseconds to rotate from one extreme
to the other, so for small values of idle-timeout you might find the control
pulse is turned off before your servo has reached the required position.
idle-timeout defaults to 0, which disables the feature.

By default ServoBlaster attempts to protect your servos by setting minimum and
maximum values on the pulse width of 50 and 250 (500us and 2.5ms).  If you want
to generate pulse widths up to 100% for some other purpose, you need to specify
the minimum and maximum values you want (probably as 0 and 2000, for 0ms and
20ms).

If you are connecting some external drive circuitry you may want active low
rather than active high outputs.  In that case you can specify an option to
invert the outputs.

If you want finer control over your servos, you can change the step increment
size from 10us to some value as low as 2us, via --step-size.  Note that if you
do change the step size then any min, max, or servo pulse widths you specified
in terms of step size units will also have to change.  You'll probably find
it less confusing to switch to using microsecond values everywhere.

If you are driving LEDs, for example, you may want a shorter cycle time, as
flickering may be visible with a 20ms (50Hz) cycle.  You can change the cycle
time via the --cycle-time option.

The final options relate to which header pins you want to use to drive your
servos.  On a Rev 1 board you can use up to 17 pins on the P1 header, and on a
Rev 2 board there are an additional 4 pins available on the P5 header.  The
default option is the equivalent of specifying

   --p1pins=7,11,12,13,15,16,18,22

As another example, if for some reason you want only two servos but you want
them to be referenced as servos 4 and 5 (perhaps you have existing software
that uses those servo numbers), you can use '0' as a placeholder for unused
servo IDs, as follows:

$ sudo ./servod --p1pins=0,0,0,0,15,16
...
Using P1 pins:           0,0,0,0,15,16

Servo mapping:
     4 on P1-15          GPIO-22
     5 on P1-16          GPIO-23

If you wanted to refer to the servos by their P1 header pins you could do
something like this, which gives you servos 7, 11, 12 and 15 on P1 header
pins 7, 11, 12 and 15:

   --p1pins=0,0,0,0,0,0,0,7,0,0,0,11,12,0,0,15

If you specify both --p1pins and --p5pins, then the order in which you specify
them is relevant because servo numbers are allocated in the order the
parameters are specified on the command line.

For the full set of P1 and P5 header pins you can use, please refer to the
GPIO section of the following web page:

  http://elinux.org/Rpi_Low-level_peripherals

It should also be clear from the servod.c source which header pins ServoBlaster
will allow you to select.  Clearly if you tell ServoBlaster to use pins that
are normally used for some other purpose, then that other functionality will
not be available while servod is running.


If you want servod to start automatically when the system boots, then you
can install it along with a startup script as follows:

$ sudo make install

You may wish to edit /etc/init.d/servoblaster to change the parameters that are
specified in that script (e.g.  the idle-timeout, which is set to 2 seconds in
the shipped version of that script).



The kernel space implementation
-------------------------------

Please note that the user space implementation is the preferred one to use and
the kernel implementation (servoblaster.ko) has been depreciated.

The kernel implementation is missing most of the command line options available
in the user space implementation, and always uses the default set of 8 pins
described above.  In addition, the kernel implementation only supports the
first command format described above.

Upon reading /dev/servoblaster, the device file provides feedback as to what
position each servo is currently set.  For example, after starting the driver
and sending the command "3=120", you would see:

pi@raspberrypi ~ $ cat /dev/servoblaster
0 0
1 0
2 0
3 120
4 0
5 0
6 0
7 0
pi@raspberrypi ~ $ 

Please read the driver source for more details.  The comments at the top of
servoblaster.c also explain how to make your system create the
/dev/servoblaster device node automatically when the driver is loaded.
Alternatively running "make install" in the driver source directory will also
create the necessary files.  Further to this, running "make install_autostart"
will create those files, plus perform the necessary changes to make
servoblaster be automatically loaded at boot.

If you wish to compile the module yourself, the approach I took was to run
rpi-update to get the latest kernel from github, then follow the instructions
on the wiki (http://elinux.org/RPi_Kernel_Compilation) to compile the kernel,
then edit the servoblaster Makefile to point at your kernel tree, then build
servoblaster.

As the mapping of GPIO to P1 header pins changed between Rev 1 and Rev 2
boards, you will need to modify servoblaster.c appropriately for your board.
Please uncomment the define for REV_1 or REV_2 as appropriate.

It is not currently possible to make the kernel implementation use the PCM
hardware rather than the PWM hardware, therefore it will interfere with 3.5mm
jack audio output.

Some people have requested that a servo output turns off automatically if no
new pulse width has been requested recently, and I've had two reports of
servos overheating when driven for long periods of time.  To support this
request, ServoBlaster implements an idle timeout which can be specified at
module load time.  The value is specified in milliseconds, so if you want
to drive your servos for 2 seconds following each new width request you would
do this:

sudo insmod ./servoblaster.ko idle_timeout=2000

Typical small servos take a few 100 milliseconds to rotate from one extreme
to the other, so for small values of idle_timeout you might find the control
pulse is turned off before your servo has reached the required position.
idle_timeout defaults to 0, which disables the feature.

NOTE: There is some doubt over how to configure the PWM clock at present.  For
me the clock is 600KHz, which leads to a tick length of 10us.  However at least
one person has reported that the pulses are out by about a factor of about 8,
and so are repeated every 2.5ms rather than every 20ms.  To work round this I
have added two module parameters:

tick_scale defaults to 6, which should be a divisor of 600KHz, which should
give a tick of 10us.  You set the pulse width in ticks (echo 2=27 >
/dev/servoblaster to set 27 ticks).

cycle_ticks is the cycle time in ticks, and defaults to 2000 to give 20ms if
one tick is 10us.  cycle_ticks should be a multiple of 8.  The max pulse width
you can specify by writing to /dev/servoblaster is (cycle_ticks/8 - 1), so for
the default parameters it is 249, or 2.49ms.

For example:

sudo insmod ./servoblaster.ko tick_scale=48

should slow it down by a factor of 8 (6*8=48).

If you can't get quite what you want with tick_scale, you can also tweak
cycle_ticks.

Eventually I might get round to letting you specify how many servo control
outputs you want, and which outputs to use, via module parameters.

As of August 30th 2012 the servoblaster.ko module is built against a 2.6.27+
kernel source from github.


Related projects:

Ville has written a simple Qt wrapper for servoblaster, which you can find
here: https://github.com/vranki/kittinger/blob/master/servocontrol.cpp & .h

Todd wrote a nice script to provide a simple user interface to control your
servos, see his sbcontrol.sh script here:
http://www.raspberrypi.org/phpBB3/viewtopic.php?f=37&t=15011&start=25#p187675


Richard Hirst <richardghirst@gmail.com>  December 2013

