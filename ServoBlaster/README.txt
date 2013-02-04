                                ServoBlaster

This is software for the RaspberryPi, which provides an interface to drive
multiple servos via the GPIO pins.   You control the servo postions by sending
commands to the driver saying what pulse width a particular servo output should
use.  The driver maintains that pulse width until you send a new command
requesting some other width.

Currently is it configured to drive 8 servos.  Servos typically need an active
high pulse of somewhere between 0.5ms and 2.5ms, where the pulse width controls
the position of the servo.  The pulse should be repeated approximately every
20ms, although pulse frequency is not critical.  The pulse width is critical,
as that translates directly to the servo position.

The driver creates a device file, /dev/servoblaster, in to which you can send
commands.  The command format is "<servo-number>=<sero-position>", where servo
number is a number from 0 to 7 inclusive, and servo position is the pulse width
you want in units of 10us.  So, if you want to set servo 3 to a pulse width of
1.2ms you could do this at the shell prompt:

echo 3=120 > /dev/servoblaster

120 is in units of 10us, so that is 1200us, or 1.2ms.

If you set a servo width to 0 it turns off the servo output, without changing
the current servo position.

The code supports 8 servos, the control signals of which should be connected
to P1 header pins as follows:

     Servo number    GPIO number   Pin in P1 header
          0               4             P1-7
          1              17             P1-11
          2              18             P1-12
          3              21             P1-13
          4              22             P1-15
          5              23             P1-16
          6              24             P1-18
          7              25             P1-22

When the driver is first loaded the GPIO pins are configure to be outputs, and
their pulse widths are set to 0.  This is so that servos don't jump to some
arbitrary position when you load the driver.  Once you know where you want your
servos positioned, write a value to /dev/servoblaster to enable the respective
output.  When the driver is unloaded it attempts to shut down the outputs
cleanly, rather than cutting some pulse short and causing a servo position to
jump.

The driver allocates a timeslot of 2.5ms to each output (8 servos resulting in
a cycle time of 20ms).  A servo output is set high at the start of its 2.5ms
timeslot, and set low after the appropriate delay.  There is then a further
delay to take us to the end of that timeslot before the next servo output is
set high.  This way there is only ever one servo output active at a time, which
helps keep the code simple.

In the following description it refers to using the PWM peripheral.  For the
user space implementation it can instead use the PCM peripheral, see below
for details.  Using PCM is typically a better option, as the 3.5mm jack also
uses the PWM peripheral, so ServoBlaster can interfere with sound output.

The driver works by setting up a linked list of DMA control blocks with the
last one linked back to the first, so once initialized the DMA controller
cycles round continuously and the driver does not need to get involved except
when a pulse width needs to be changed.  For a given servo there are four DMA
control blocks; the first transfers a single word to the GPIO 'set output'
register, the second transfers some number of words to the PWM FIFO to generate
the required pulse width time, the third transfers a single word to the GPIO
'clear output' register, and the fourth transfers a number of words to the PWM
FIFO to generate a delay up to the end of the 2.5ms timeslot.

While the driver does use the PWM peripheral, it only uses it to pace the DMA
transfers, so as to generate accurate delays.  The PWM is set up such that it
consumes one word from the FIFO every 10us, so to generate a delay of 1.2ms the
driver sets the DMA transfer count to 480 (1200/10*4, as the FIFO is 32 bits
wide).  The PWM is set to request data as soon as there is a single word free
in the FIFO, so there should be no burst transfers to upset the timing.

I used Panalyzer running on one Pi to mointor the servo outputs from a second
Pi.  The pulse widths and frequencies seem very stable, even under heavy SD
card use.  This is expected, because the pulse generation is effectively
handled in hardware and not influenced by interrupt latency or scheduling
effects.

The driver uses DMA channel 0, and PWM channel 1.  It makes no attempt to
protect against other code using those peripherals.  It sets the relevant GPIO
pins to be outputs when the driver is loaded, so please ensure that you are not
driving those pins externally.

I would of course recommend some buffering between the GPIO outputs and the
servo controls, to protect the Pi.  That said, I'm living dangerously and doing
without :-)  If you just want to experiment with a small servo you can probably
take the 5 volts for it from the header pins on the Pi, but I find that doing
anything non-trivial with four servos connected pulls the 5 volts down far
enough to crash the Pi!



There are two implementions of ServoBlaster; a kernel module based one, and
a user space daemon.  The kernel module based one is the original, and is
more mature.  The user space daemon implementation is much more convenient to
use but is less well tested and does not have all the features of the kernel
based one.  I would recommend you try the user space implementation first, as
it is likely to be easier to get going.

Details specific to each implementation are provided in separate sections
below.


The user space daemon
---------------------

To use this daemon grab the servod.c source and Makefile and:

$ make servod
$ sudo ./servod
Using hardware:        PWM
Number of servos:        8
Servo cycle time:    20000us
Pulse width units:      10us
Maximum width value:   249 (2490us)
$

The prompt will return immediately, and servod is left running in the
background.  You can check it is running via the "ps ax" command.
If you want to stop servod, the easiest way is to run:

$ sudo killall servod

Note that use of PWM will interfere with 3.5mm jack audio output.  Instead
of using the PWM hardware, you can use the PCM hardware, which is less likely
to cause a conflict.  Please be aware that the PCM mode is very lightly tested
at present.  To use PCM mode, invoke servod as follows:

$ sudo ./servod --pcm
Using hardware:        PCM
...

Features not currently supported in the user space implementation:

- does not support the timeout= option to turn off servo outputs after
  some specified delay.  You must set a servo width to 0 to turn off an
  output, if you want to.
- you cannot read /dev/servoblaster to see the current servo settings



The kernel space implementation
-------------------------------

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
me the clock is 600KHz, which leads to a tick lenght of 10us.  However at least
one person has reported that the pulses are out by about a factor of about 8,
and so are repeated every 2.5ms rather than every 20ms.  To work round this I
have added two module parameters:

tick_scale defaults to 6, which should be a divisor of 600KHz, which should
give a tick of 10us.  You set the pulse width in ticks (echo 2=27 >
/dev/panalyzer to set 27 ticks).

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


Richard Hirst <richardghirst@gmail.com>  January 2013

