                                ServoBlaster

This is a Linux kernel driver for the RaspberryPi, which provides an interface
to drive multiple servos via the GPIO pins.   You control the servo postions by
sending commands to the driver saying what pulse width a particular servo
output should use.  The driver maintains that pulse width until you send a new
command requesting some other width.

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

When the driver is first loaded the GPIO pins are configure to be outputs, and
their pulse widths are set to 0.  This is so that servos don't jump to some
arbitrary postion when you load the driver.  Once you know where you want your
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

Please read the driver source for more details, such as which GPIO pin maps to
which servo number.  The comments at the top of servoblaster.c also explain how
to make your system create the /dev/servoblaster device node automatically when
the driver is loaded.

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

If you wish to compile the module yourself, the approach I took was to run
rpi-update to get the latest kernel from github, then follow the instructions
on the wiki (http://elinux.org/RPi_Kernel_Compilation) to compile the kernel,
then edit the servoblaster Makefile to point at your kernel tree, then build
servoblaster.

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


Richard Hirst <richardghirst@gmail.com>  August 2012

