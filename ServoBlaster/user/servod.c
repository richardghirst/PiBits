/*
 * servod.c Multiple Servo Driver for the RaspberryPi
 * Copyright (c) 2013 Richard Hirst <richardghirst@gmail.com>
 *
 * This program provides very similar functionality to servoblaster, except
 * that rather than implementing it as a kernel module, servod implements
 * the functionality as a usr space daemon.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.
 */

/* TODO: Separate idle timeout handling from genuine set-to-zero requests */
/* TODO: Add ability to specify time frame over which an adjustment should be made */
/* TODO: Add servoctl utility to set and query servo positions, etc */
/* TODO: Add slow-start option */

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include <errno.h>
#include <stdarg.h>
#include <stdint.h>
#include <signal.h>
#include <time.h>
#include <sys/time.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <sys/mman.h>
#include <getopt.h>
#include <math.h>

#include "mailbox.h"

#define DMY	255	// Used to represent an invalid P1 pin, or unmapped servo

#define NUM_P1PINS	40
#define NUM_P5PINS	8

#define MAX_SERVOS	32	/* Only 21 really, but this lets you map servo IDs
				 * to P1 pins, if you want to
				 */
#define MAX_MEMORY_USAGE	(16*1024*1024)	/* Somewhat arbitrary limit of 16MB */

#define DEFAULT_CYCLE_TIME_US	20000
#define DEFAULT_STEP_TIME_US	10
#define DEFAULT_SERVO_MIN_US	500
#define DEFAULT_SERVO_MAX_US	2500

#define DEVFILE			"/dev/servoblaster"
#define CFGFILE			"/dev/servoblaster-cfg"

#define PAGE_SIZE		4096
#define PAGE_SHIFT		12

#define DMA_CHAN_SIZE		0x100
#define DMA_CHAN_MIN		0
#define DMA_CHAN_MAX		14
#define DMA_CHAN_DEFAULT	14

#define DMA_BASE_OFFSET		0x00007000
#define DMA_LEN			DMA_CHAN_SIZE * (DMA_CHAN_MAX+1)
#define PWM_BASE_OFFSET		0x0020C000
#define PWM_LEN			0x28
#define CLK_BASE_OFFSET	        0x00101000
#define CLK_LEN			0xA8
#define GPIO_BASE_OFFSET	0x00200000
#define GPIO_LEN		0x100
#define PCM_BASE_OFFSET		0x00203000
#define PCM_LEN			0x24

#define DMA_VIRT_BASE		(periph_virt_base + DMA_BASE_OFFSET)
#define PWM_VIRT_BASE		(periph_virt_base + PWM_BASE_OFFSET)
#define CLK_VIRT_BASE		(periph_virt_base + CLK_BASE_OFFSET)
#define GPIO_VIRT_BASE		(periph_virt_base + GPIO_BASE_OFFSET)
#define PCM_VIRT_BASE		(periph_virt_base + PCM_BASE_OFFSET)

#define PWM_PHYS_BASE		(periph_phys_base + PWM_BASE_OFFSET)
#define PCM_PHYS_BASE		(periph_phys_base + PCM_BASE_OFFSET)
#define GPIO_PHYS_BASE		(periph_phys_base + GPIO_BASE_OFFSET)

#define DMA_NO_WIDE_BURSTS	(1<<26)
#define DMA_WAIT_RESP		(1<<3)
#define DMA_D_DREQ		(1<<6)
#define DMA_PER_MAP(x)		((x)<<16)
#define DMA_END			(1<<1)
#define DMA_RESET		(1<<31)
#define DMA_INT			(1<<2)

#define DMA_CS			(0x00/4)
#define DMA_CONBLK_AD		(0x04/4)
#define DMA_SOURCE_AD		(0x0c/4)
#define DMA_DEBUG		(0x20/4)

#define GPIO_FSEL0		(0x00/4)
#define GPIO_SET0		(0x1c/4)
#define GPIO_CLR0		(0x28/4)
#define GPIO_LEV0		(0x34/4)
#define GPIO_PULLEN		(0x94/4)
#define GPIO_PULLCLK		(0x98/4)

#define GPIO_MODE_IN		0
#define GPIO_MODE_OUT		1

#define PWM_CTL			(0x00/4)
#define PWM_DMAC		(0x08/4)
#define PWM_RNG1		(0x10/4)
#define PWM_FIFO		(0x18/4)

#define PWMCLK_CNTL		40
#define PWMCLK_DIV		41

#define PWMCTL_MODE1		(1<<1)
#define PWMCTL_PWEN1		(1<<0)
#define PWMCTL_CLRF		(1<<6)
#define PWMCTL_USEF1		(1<<5)

#define PWMDMAC_ENAB		(1<<31)
#define PWMDMAC_THRSHLD		((15<<8)|(15<<0))

#define PCM_CS_A		(0x00/4)
#define PCM_FIFO_A		(0x04/4)
#define PCM_MODE_A		(0x08/4)
#define PCM_RXC_A		(0x0c/4)
#define PCM_TXC_A		(0x10/4)
#define PCM_DREQ_A		(0x14/4)
#define PCM_INTEN_A		(0x18/4)
#define PCM_INT_STC_A		(0x1c/4)
#define PCM_GRAY		(0x20/4)

#define PCMCLK_CNTL		38
#define PCMCLK_DIV		39

#define DELAY_VIA_PWM		0
#define DELAY_VIA_PCM		1

#define ROUNDUP(val, blksz)	(((val)+((blksz)-1)) & ~(blksz-1))

typedef struct {
	uint32_t info, src, dst, length,
		 stride, next, pad[2];
} dma_cb_t;

#define BUS_TO_PHYS(x) ((x)&~0xC0000000)

/* Define which P1 header pins to use by default.  These are the eight standard
 * GPIO pins (those coloured green in the diagram on this page:
 *    http://elinux.org/Rpi_Low-level_peripherals
 *
 * Which P1 header pins are actually used can be overridden via command line
 * parameter '--p1pins=...'.
 */

static char *default_p1_pins = "7,11,12,13,15,16,18,22";
static char *default_p5_pins = "";

static uint8_t rev1_p1pin2gpio_map[] = {
	DMY,	// P1-1   3v3
	DMY,	// P1-2   5v
	0,	// P1-3   GPIO 0 (SDA)
	DMY,	// P1-4   5v
	1,	// P1-5   GPIO 1 (SCL)
	DMY,	// P1-6   Ground
	4,	// P1-7   GPIO 4 (GPCLK0)
	14,	// P1-8   GPIO 14 (TXD)
	DMY,	// P1-9   Ground
	15,	// P1-10  GPIO 15 (RXD)
	17,	// P1-11  GPIO 17
	18,	// P1-12  GPIO 18 (PCM_CLK)
	21,	// P1-13  GPIO 21
	DMY,	// P1-14  Ground
	22,	// P1-15  GPIO 22
	23,	// P1-16  GPIO 23
	DMY,	// P1-17  3v3
	24,	// P1-18  GPIO 24
	10,	// P1-19  GPIO 10 (MOSI)
	DMY,	// P1-20  Ground
	9,	// P1-21  GPIO 9 (MISO)
	25,	// P1-22  GPIO 25
	11,	// P1-23  GPIO 11 (SCLK)
	8,	// P1-24  GPIO 8 (CE0)
	DMY,	// P1-25  Ground
	7,	// P1-26  GPIO 7 (CE1)
};

static uint8_t rev1_p5pin2gpio_map[] = {
	DMY,	// (P5-1 on rev 2 boards)
	DMY,	// (P5-2 on rev 2 boards)
	DMY,	// (P5-3 on rev 2 boards)
	DMY,	// (P5-4 on rev 2 boards)
	DMY,	// (P5-5 on rev 2 boards)
	DMY,	// (P5-6 on rev 2 boards)
	DMY,	// (P5-7 on rev 2 boards)
	DMY,	// (P5-8 on rev 2 boards)
};

static uint8_t rev2_p1pin2gpio_map[] = {
	DMY,	// P1-1   3v3
	DMY,	// P1-2   5v
	2,	// P1-3   GPIO 2 (SDA)
	DMY,	// P1-4   5v
	3,	// P1-5   GPIO 3 (SCL)
	DMY,	// P1-6   Ground
	4,	// P1-7   GPIO 4 (GPCLK0)
	14,	// P1-8   GPIO 14 (TXD)
	DMY,	// P1-9   Ground
	15,	// P1-10  GPIO 15 (RXD)
	17,	// P1-11  GPIO 17
	18,	// P1-12  GPIO 18 (PCM_CLK)
	27,	// P1-13  GPIO 27
	DMY,	// P1-14  Ground
	22,	// P1-15  GPIO 22
	23,	// P1-16  GPIO 23
	DMY,	// P1-17  3v3
	24,	// P1-18  GPIO 24
	10,	// P1-19  GPIO 10 (MOSI)
	DMY,	// P1-20  Ground
	9,	// P1-21  GPIO 9 (MISO)
	25,	// P1-22  GPIO 25
	11,	// P1-23  GPIO 11 (SCLK)
	8,	// P1-24  GPIO 8 (CE0)
	DMY,	// P1-25  Ground
	7,	// P1-26  GPIO 7 (CE1)
};

static uint8_t rev2_p5pin2gpio_map[] = {
	DMY,	// P5-1   5v0
	DMY,	// P5-2   3v3
	28,	// P5-3   GPIO 28 (I2C0_SDA)
	29,	// P5-4   GPIO 29 (I2C0_SCL)
	30,	// P5-5   GPIO 30
	31,	// P5-6   GPIO 31
	DMY,	// P5-7   Ground
	DMY,	// P5-8   Ground
};

static uint8_t bplus_p1pin2gpio_map[] = {
	DMY,	// P1-1   3v3
	DMY,	// P1-2   5v
	2,	// P1-3   GPIO 2 (SDA)
	DMY,	// P1-4   5v
	3,	// P1-5   GPIO 3 (SCL)
	DMY,	// P1-6   Ground
	4,	// P1-7   GPIO 4 (GPCLK0)
	14,	// P1-8   GPIO 14 (TXD)
	DMY,	// P1-9   Ground
	15,	// P1-10  GPIO 15 (RXD)
	17,	// P1-11  GPIO 17
	18,	// P1-12  GPIO 18 (PCM_CLK)
	27,	// P1-13  GPIO 27
	DMY,	// P1-14  Ground
	22,	// P1-15  GPIO 22
	23,	// P1-16  GPIO 23
	DMY,	// P1-17  3v3
	24,	// P1-18  GPIO 24
	10,	// P1-19  GPIO 10 (MOSI)
	DMY,	// P1-20  Ground
	9,	// P1-21  GPIO 9 (MISO)
	25,	// P1-22  GPIO 25
	11,	// P1-23  GPIO 11 (SCLK)
	8,	// P1-24  GPIO 8 (CE0)
	DMY,	// P1-25  Ground
	7,	// P1-26  GPIO 7 (CE1)
	DMY,	// P1-27  ID_SD
	DMY,	// P1-28  ID_SC
	5,	// P1-29  GPIO 5
	DMY,	// P1-30  Ground
	6,	// P1-31  GPIO 5
	12,	// P1-32  GPIO 12
	13,	// P1-33  GPIO 13
	DMY,	// P1-34  Ground
	19,	// P1-35  GPIO 19
	16,	// P1-36  GPIO 16
	26,	// P1-37  GPIO 26
	20,	// P1-38  GPIO 20
	DMY,	// P1-39  Ground
	21,	// P1-40  GPIO 21
};

// cycle_time_us is the pulse cycle time per servo, in microseconds.
// Typically it should be 20ms, or 20000us.

// step_time_us is the pulse width increment granularity, again in microseconds.
// Setting step_time_us too low will likely cause problems as the DMA controller
// will use too much memory bandwidth.  10us is a good value, though you
// might be ok setting it as low as 2us.

static int cycle_time_us;
static int step_time_us;

static uint8_t servo2gpio[MAX_SERVOS];
static uint8_t p1pin2servo[NUM_P1PINS+1];
static uint8_t p5pin2servo[NUM_P5PINS+1];
static int servostart[MAX_SERVOS];
static int servowidth[MAX_SERVOS];
static int num_servos;
static uint32_t gpiomode[MAX_SERVOS];
static int restore_gpio_modes;

static volatile uint32_t *pwm_reg;
static volatile uint32_t *pcm_reg;
static volatile uint32_t *clk_reg;
static volatile uint32_t *dma_reg;
static volatile uint32_t *gpio_reg;

static int delay_hw = DELAY_VIA_PWM;

static struct timeval *servo_kill_time;

static int dma_chan;
static int idle_timeout;
static int invert = 0;
static int servo_min_ticks;
static int servo_max_ticks;
static int num_samples;
static int num_cbs;
static int num_pages;
static uint32_t *turnoff_mask;
static uint32_t *turnon_mask;
static dma_cb_t *cb_base;

static int board_model;
static int gpio_cfg;

static uint32_t periph_phys_base;
static uint32_t periph_virt_base;
static uint32_t dram_phys_base;
static uint32_t mem_flag;

static char *gpio_desc[] = {
	"Unknown",
	"P1 (26 pins)",
	"P1 (26 pins), P5 (8 pins)",
	"P1 (40 pins)"
};

static struct {
	int handle;		/* From mbox_open() */
	uint32_t size;		/* Required size */
	unsigned mem_ref;	/* From mem_alloc() */
	unsigned bus_addr;	/* From mem_lock() */
	uint8_t *virt_addr;	/* From mapmem() */
} mbox;
	
static void set_servo(int servo, int width);
static void set_servo_idle(int servo);
static void gpio_set_mode(uint32_t gpio, uint32_t mode);
static char *gpio2pinname(uint8_t gpio);

static void
udelay(int us)
{
	struct timespec ts = { 0, us * 1000 };

	nanosleep(&ts, NULL);
}

static void
terminate(int dummy)
{
	int i;

	if (dma_reg && mbox.virt_addr) {
		for (i = 0; i < MAX_SERVOS; i++) {
			if (servo2gpio[i] != DMY)
				set_servo(i, 0);
		}
		udelay(cycle_time_us);
		dma_reg[DMA_CS] = DMA_RESET;
		udelay(10);
	}
	if (restore_gpio_modes) {
		for (i = 0; i < MAX_SERVOS; i++) {
			if (servo2gpio[i] != DMY)
				gpio_set_mode(servo2gpio[i], gpiomode[i]);
		}
	}
	if (mbox.virt_addr != NULL) {
		unmapmem(mbox.virt_addr, mbox.size);
		mem_unlock(mbox.handle, mbox.mem_ref);
		mem_free(mbox.handle, mbox.mem_ref);
		if (mbox.handle >= 0)
			mbox_close(mbox.handle);
	}

	unlink(DEVFILE);
	unlink(CFGFILE);
	exit(1);
}

static void
fatal(char *fmt, ...)
{
	va_list ap;

	va_start(ap, fmt);
	vfprintf(stderr, fmt, ap);
	va_end(ap);
	terminate(0);
}

static void
init_idle_timers(void)
{
	servo_kill_time = calloc(MAX_SERVOS, sizeof(struct timeval));
	if (!servo_kill_time)
		fatal("servod: calloc() failed\n");
}

static void
update_idle_time(int servo)
{
	if (idle_timeout == 0)
		return;

	gettimeofday(servo_kill_time + servo, NULL);
	servo_kill_time[servo].tv_sec += idle_timeout / 1000;
	servo_kill_time[servo].tv_usec += (idle_timeout % 1000) * 1000;
	while (servo_kill_time[servo].tv_usec >= 1000000) {
		servo_kill_time[servo].tv_usec -= 1000000;
		servo_kill_time[servo].tv_sec++;
	}
}

static void
get_next_idle_timeout(struct timeval *tv)
{
	int i;
	struct timeval now;
	struct timeval min = { 60, 0 };
	long this_diff, min_diff;

	gettimeofday(&now, NULL);
	for (i = 0; i < MAX_SERVOS; i++) {
		if (servo2gpio[i] == DMY || servo_kill_time[i].tv_sec == 0)
			continue;
		else if (servo_kill_time[i].tv_sec < now.tv_sec ||
			(servo_kill_time[i].tv_sec == now.tv_sec &&
			 servo_kill_time[i].tv_usec <= now.tv_usec)) {
			servo_kill_time[i].tv_sec = 0;
			set_servo_idle(i);
		} else {
			this_diff = (servo_kill_time[i].tv_sec - now.tv_sec) * 1000000
				+ servo_kill_time[i].tv_usec - now.tv_usec;
			min_diff = min.tv_sec * 1000000 + min.tv_usec;
			if (this_diff < min_diff) {
				min.tv_sec = this_diff / 1000000;
				min.tv_usec = this_diff % 1000000;
			}
		}
	}
	*tv = min;
}

static uint32_t gpio_get_mode(uint32_t gpio)
{
	uint32_t fsel = gpio_reg[GPIO_FSEL0 + gpio/10];

	return (fsel >> ((gpio % 10) * 3)) & 7;
}

static void
gpio_set_mode(uint32_t gpio, uint32_t mode)
{
	uint32_t fsel = gpio_reg[GPIO_FSEL0 + gpio/10];

	fsel &= ~(7 << ((gpio % 10) * 3));
	fsel |= mode << ((gpio % 10) * 3);
	gpio_reg[GPIO_FSEL0 + gpio/10] = fsel;
}

static void
gpio_set(int gpio, int level)
{
	if (level)
		gpio_reg[GPIO_SET0] = 1 << gpio;
	else
		gpio_reg[GPIO_CLR0] = 1 << gpio;
}

static uint32_t
mem_virt_to_phys(void *virt)
{
	uint32_t offset = (uint8_t *)virt - mbox.virt_addr;

	return mbox.bus_addr + offset;
}

static void *
map_peripheral(uint32_t base, uint32_t len)
{
	int fd = open("/dev/mem", O_RDWR|O_SYNC);
	void * vaddr;

	if (fd < 0)
		fatal("servod: Failed to open /dev/mem: %m\n");
	vaddr = mmap(NULL, len, PROT_READ|PROT_WRITE, MAP_SHARED, fd, base);
	if (vaddr == MAP_FAILED)
		fatal("servod: Failed to map peripheral at 0x%08x: %m\n", base);
	close(fd);

	return vaddr;
}

static void
set_servo_idle(int servo)
{
	/* Just remove the 'turn-on' action and allow the 'turn-off' action at
	 * the end of the current pulse to turn it off.  Special case if
	 * current width is 100%; in that case there will be no 'turn-off'
	 * action, so we will need to force the output off here.  We must not
	 * force the output in other cases, because that might lead to
	 * truncated pulses which would make a servo change position.
	 */
	turnon_mask[servo] = 0;
	if (servowidth[servo] == num_samples)
		gpio_set(servo2gpio[servo], invert ? 1 : 0);
}

/* Carefully add or remove bits from the turnoff_mask such that regardless
 * of where the DMA controller is in its cycle, and whether we are increasing
 * or decreasing the pulse width, the generated pulse will only ever be the
 * old width or the new width.  If we don't take such care then there could be
 * a cycle with some pulse width between the two requested ones.  That doesn't
 * really matter for servos, but when driving LEDs some odd intensity for one
 * cycle can be noticeable.  It may be that the servo output has been turned
 * off via the inactivity timer, which is handled by always setting the turnon
 * mask appropriately at the end of this function.
 */
static void
set_servo(int servo, int width)
{
	volatile uint32_t *dp;
	int i;
	uint32_t mask = 1 << servo2gpio[servo];


	if (width > servowidth[servo]) {
		dp = turnoff_mask + servostart[servo] + width;
		if (dp >= turnoff_mask + num_samples)
			dp -= num_samples;

		for (i = width; i > servowidth[servo]; i--) {
			dp--;
			if (dp < turnoff_mask)
				dp = turnoff_mask + num_samples - 1;
			//printf("%5d, clearing at %p\n", dp - ctl->turnoff, dp);
			*dp &= ~mask;
		}
	} else if (width < servowidth[servo]) {
		dp = turnoff_mask + servostart[servo] + width;
		if (dp >= turnoff_mask + num_samples)
			dp -= num_samples;

		for (i = width; i < servowidth[servo]; i++) {
			//printf("%5d, setting at %p\n", dp - ctl->turnoff, dp);
			*dp++ |= mask;
			if (dp >= turnoff_mask + num_samples)
				dp = turnoff_mask;
		}
	}
	servowidth[servo] = width;
	if (width == 0) {
		turnon_mask[servo] = 0;
	} else {
		turnon_mask[servo] = mask;
	}
	update_idle_time(servo);
}

static void
setup_sighandlers(void)
{
	int i;

	// Catch all signals possible - it is vital we kill the DMA engine
	// on process exit!
	for (i = 0; i < 64; i++) {
		struct sigaction sa;

		memset(&sa, 0, sizeof(sa));
		sa.sa_handler = terminate;
		sigaction(i, &sa, NULL);
	}
}

static void
init_ctrl_data(void)
{
	dma_cb_t *cbp = cb_base;
	uint32_t phys_fifo_addr, cbinfo;
	uint32_t phys_gpclr0;
	uint32_t phys_gpset0;
	int servo, i, numservos = 0, curstart = 0;
	uint32_t maskall = 0;

	if (invert) {
		phys_gpclr0 = GPIO_PHYS_BASE + 0x1c;
		phys_gpset0 = GPIO_PHYS_BASE + 0x28;
	} else {
		phys_gpclr0 = GPIO_PHYS_BASE + 0x28;
		phys_gpset0 = GPIO_PHYS_BASE + 0x1c;
	}

	if (delay_hw == DELAY_VIA_PWM) {
		phys_fifo_addr = PWM_PHYS_BASE + 0x18;
		cbinfo = DMA_NO_WIDE_BURSTS | DMA_WAIT_RESP | DMA_D_DREQ | DMA_PER_MAP(5);
	} else {
		phys_fifo_addr = PCM_PHYS_BASE + 0x04;
		cbinfo = DMA_NO_WIDE_BURSTS | DMA_WAIT_RESP | DMA_D_DREQ | DMA_PER_MAP(2);
	}

	memset(turnon_mask, 0, MAX_SERVOS * sizeof(*turnon_mask));

	for (servo = 0 ; servo < MAX_SERVOS; servo++) {
		servowidth[servo] = 0;
		if (servo2gpio[servo] != DMY) {
			numservos++;
			maskall |= 1 << servo2gpio[servo];
		}
	}

	for (i = 0; i < num_samples; i++)
		turnoff_mask[i] = maskall;

	for (servo = 0; servo < MAX_SERVOS; servo++) {
		if (servo2gpio[servo] != DMY) {
			servostart[servo] = curstart;
			curstart += num_samples / num_servos;
		}
	}

	servo = 0;
	while (servo < MAX_SERVOS && servo2gpio[servo] == DMY)
		servo++;

	for (i = 0; i < num_samples; i++) {
		cbp->info = DMA_NO_WIDE_BURSTS | DMA_WAIT_RESP;
		cbp->src = mem_virt_to_phys(turnoff_mask + i);
		cbp->dst = phys_gpclr0;
		cbp->length = 4;
		cbp->stride = 0;
		cbp->next = mem_virt_to_phys(cbp + 1);
		cbp++;
		if (i == servostart[servo]) {
			cbp->info = DMA_NO_WIDE_BURSTS | DMA_WAIT_RESP;
			cbp->src = mem_virt_to_phys(turnon_mask + servo);
			cbp->dst = phys_gpset0;
			cbp->length = 4;
			cbp->stride = 0;
			cbp->next = mem_virt_to_phys(cbp + 1);
			cbp++;
			servo++;
			while (servo < MAX_SERVOS && servo2gpio[servo] == DMY)
				servo++;
		}
		// Delay
		cbp->info = cbinfo;
		cbp->src = mem_virt_to_phys(turnoff_mask);	// Any data will do
		cbp->dst = phys_fifo_addr;
		cbp->length = 4;
		cbp->stride = 0;
		cbp->next = mem_virt_to_phys(cbp + 1);
		cbp++;
	}
	cbp--;
	cbp->next = mem_virt_to_phys(cb_base);
}

static void
init_hardware(void)
{
	if (delay_hw == DELAY_VIA_PWM) {
		// Initialise PWM
		pwm_reg[PWM_CTL] = 0;
		udelay(10);
		clk_reg[PWMCLK_CNTL] = 0x5A000006;		// Source=PLLD (500MHz)
		udelay(100);
		clk_reg[PWMCLK_DIV] = 0x5A000000 | (500<<12);	// set pwm div to 500, giving 1MHz
		udelay(100);
		clk_reg[PWMCLK_CNTL] = 0x5A000016;		// Source=PLLD and enable
		udelay(100);
		pwm_reg[PWM_RNG1] = step_time_us;
		udelay(10);
		pwm_reg[PWM_DMAC] = PWMDMAC_ENAB | PWMDMAC_THRSHLD;
		udelay(10);
		pwm_reg[PWM_CTL] = PWMCTL_CLRF;
		udelay(10);
		pwm_reg[PWM_CTL] = PWMCTL_USEF1 | PWMCTL_PWEN1;
		udelay(10);
	} else {
		// Initialise PCM
		pcm_reg[PCM_CS_A] = 1;				// Disable Rx+Tx, Enable PCM block
		udelay(100);
		clk_reg[PCMCLK_CNTL] = 0x5A000006;		// Source=PLLD (500MHz)
		udelay(100);
		clk_reg[PCMCLK_DIV] = 0x5A000000 | (500<<12);	// Set pcm div to 500, giving 1MHz
		udelay(100);
		clk_reg[PCMCLK_CNTL] = 0x5A000016;		// Source=PLLD and enable
		udelay(100);
		pcm_reg[PCM_TXC_A] = 0<<31 | 1<<30 | 0<<20 | 0<<16; // 1 channel, 8 bits
		udelay(100);
		pcm_reg[PCM_MODE_A] = (step_time_us - 1) << 10;
		udelay(100);
		pcm_reg[PCM_CS_A] |= 1<<4 | 1<<3;		// Clear FIFOs
		udelay(100);
		pcm_reg[PCM_DREQ_A] = 64<<24 | 64<<8;		// DMA Req when one slot is free?
		udelay(100);
		pcm_reg[PCM_CS_A] |= 1<<9;			// Enable DMA
		udelay(100);
	}

	// Initialise the DMA
	dma_reg[DMA_CS] = DMA_RESET;
	udelay(10);
	dma_reg[DMA_CS] = DMA_INT | DMA_END;
	dma_reg[DMA_CONBLK_AD] = mem_virt_to_phys(cb_base);
	dma_reg[DMA_DEBUG] = 7; // clear debug error flags
	dma_reg[DMA_CS] = 0x10880001;	// go, mid priority, wait for outstanding writes

	if (delay_hw == DELAY_VIA_PCM) {
		pcm_reg[PCM_CS_A] |= 1<<2;			// Enable Tx
	}
}

static void
do_status(char *filename)
{
	uint32_t last;
	int status = -1;
	char *p;
	int fd;
	const char *dma_dead = "ERROR: DMA not running\n";

	while (*filename == ' ')
		filename++;
	p = filename + strlen(filename) - 1;
	while (p > filename && (*p == '\n' || *p == '\r' || *p == ' '))
		*p-- = '\0';

	last = dma_reg[DMA_CONBLK_AD];
	udelay(step_time_us*2);
	if (dma_reg[DMA_CONBLK_AD] != last)
		status = 0;
	if ((fd = open(filename, O_WRONLY|O_CREAT, 0666)) >= 0) {
		if (status == 0)
			write(fd, "OK\n", 3);
		else
			write(fd, dma_dead, strlen(dma_dead));
		close(fd);
	} else {
		printf("Failed to open %s for writing: %m\n", filename);
	}
}

static void
do_debug(void)
{
	int i;
	uint32_t mask = 0;
	uint32_t last;

	last = dma_reg[DMA_CONBLK_AD];
	udelay(step_time_us*2);
	printf("%08x %08x\n", last, dma_reg[DMA_CONBLK_AD]);

	printf("---------------------------\n");
	printf("Servo  Start  Width  TurnOn\n");
	for (i = 0; i < MAX_SERVOS; i++) {
		if (servo2gpio[i] != DMY) {
			printf("%3d: %6d %6d %6d\n", i, servostart[i],
					servowidth[i], !!turnon_mask[i]);
			mask |= 1 << servo2gpio[i];
		}
	}
	printf("\nData:\n");
	last = 0xffffffff;
	for (i = 0; i < num_samples; i++) {
		uint32_t curr = turnoff_mask[i] & mask;
		if (curr != last)
			printf("@%5d: %08x\n", i, curr);
		last = curr;
	}
	printf("---------------------------\n");
}

static int
parse_width(int servo, char *width_arg)
{
	char *p;
	char *digits = width_arg;
	double width;

	if (*width_arg == '-' || *width_arg == '+') {
		digits++;
	}

	if (*digits < '0' || *digits > '9') {
		return -1;
	}
	width = strtod(digits, &p);

	if (*p == '\0') {
		/* Specified in steps */
	} else if (!strcmp(p, "us")) {
		width /= step_time_us;
	} else if (!strcmp(p, "%")) {
		width = width * (servo_max_ticks - servo_min_ticks) / 100.0 + servo_min_ticks;
	} else {
		return -1;
	}
	width = floor(width);
	if (*width_arg == '+') {
		width = servowidth[servo] + width;
		if (width > servo_max_ticks)
			width = servo_max_ticks;
	} else if (*width_arg == '-') {
		width = servowidth[servo] - width;
		if (width < servo_min_ticks)
			width = servo_min_ticks;
	}

	if (width == 0) {
		return (int)width;
	} else if (width < servo_min_ticks || width > servo_max_ticks) {
		return -1;
	} else {
		return (int)width;
	}
}

static void
go_go_go(void)
{
	int fd;
	struct timeval tv;
	static char line[128];
	int nchars = 0;

	if ((fd = open(DEVFILE, O_RDWR|O_NONBLOCK)) == -1)
		fatal("servod: Failed to open %s: %m\n", DEVFILE);

	for (;;) {
		int n, width, servo;
		fd_set ifds;
		char width_arg[64];

		FD_ZERO(&ifds);
		FD_SET(fd, &ifds);
		get_next_idle_timeout(&tv);
		if ((n = select(fd+1, &ifds, NULL, NULL, &tv)) != 1)
			continue;
		while (read(fd, line+nchars, 1) == 1) {
			if (line[nchars] == '\n') {
				line[++nchars] = '\0';
				nchars = 0;
				if (line[0] == 'p' || line[0] == 'P') {
					int hdr, pin, width;

					n = sscanf(line+1, "%d-%d=%s", &hdr, &pin, width_arg);
					if (n != 3) {
						fprintf(stderr, "Bad input: %s", line);
					} else if (hdr != 1 && hdr != 5) {
						fprintf(stderr, "Invalid header P%d\n", hdr);
					} else if (pin < 1 ||
							(hdr == 1 && pin > NUM_P1PINS) ||
							(hdr == 5 && pin > NUM_P5PINS)) {
						fprintf(stderr, "Invalid pin number P%d-%d\n", hdr, pin);
					} else if ((hdr == 1 && p1pin2servo[pin] == DMY) ||
						   (hdr == 5 && p5pin2servo[pin] == DMY)) {
							fprintf(stderr, "P%d-%d is not mapped to a servo\n", hdr, pin);
					} else {
						if (hdr == 1) {
							servo = p1pin2servo[pin];
						} else {
							servo = p5pin2servo[pin];
						}
						if ((width = parse_width(servo, width_arg)) < 0) {
							fprintf(stderr, "Invalid width specified\n");
						} else {
							set_servo(servo, width);
						}
					}
				} else {
					n = sscanf(line, "%d=%s", &servo, width_arg);
					if (!strcmp(line, "debug\n")) {
						do_debug();
					} else if (!strncmp(line, "status ", 7)) {
						do_status(line + 7);
					} else if (n != 2) {
						fprintf(stderr, "Bad input: %s", line);
					} else if (servo < 0 || servo >= MAX_SERVOS) {
						fprintf(stderr, "Invalid servo number %d\n", servo);
					} else if (servo2gpio[servo] == DMY) {
						fprintf(stderr, "Servo %d is not mapped to a GPIO pin\n", servo);
					} else if ((width = parse_width(servo, width_arg)) < 0) {
						fprintf(stderr, "Invalid width specified\n");
					} else {
						set_servo(servo, width);
					}
				}
			} else {
				if (++nchars >= 126) {
					fprintf(stderr, "Input too long\n");
					nchars = 0;
				}
			}
		}
	}
}

/* Determining the board revision is a lot more complicated than it should be
 * (see comments in wiringPi for details).  We will just look at the last two
 * digits of the Revision string and treat '00' and '01' as errors, '02' and
 * '03' as rev 1, and any other hex value as rev 2.  'Pi1 and Pi2 are
 * differentiated by the Hardware being BCM2708 or BCM2709.
 */
static void
get_model_and_revision(void)
{
	char buf[128], revstr[128], modelstr[128];
	char *ptr, *end, *res;
	int board_revision;
	FILE *fp;

	revstr[0] = modelstr[0] = '\0';

	fp = fopen("/proc/cpuinfo", "r");

	if (!fp)
		fatal("Unable to open /proc/cpuinfo: %m\n");

	while ((res = fgets(buf, 128, fp))) {
		if (!strncasecmp("hardware", buf, 8))
			memcpy(modelstr, buf, 128);
		else if (!strncasecmp(buf, "revision", 8))
			memcpy(revstr, buf, 128);
	}
	fclose(fp);

	if (modelstr[0] == '\0')
		fatal("servod: No 'Hardware' record in /proc/cpuinfo\n");
	if (revstr[0] == '\0')
		fatal("servod: No 'Revision' record in /proc/cpuinfo\n");

	if (strstr(modelstr, "BCM2708"))
		board_model = 1;
	else if (strstr(modelstr, "BCM2709"))
		board_model = 2;
	else if (strstr(modelstr, "BCM2835"))	/* Quick hack for Pi-Zero */
		board_model = 1;
	else
		fatal("servod: Cannot parse the hardware name string\n");

	ptr = revstr + strlen(revstr) - 3;
	board_revision = strtol(ptr, &end, 16);
	if (end != ptr + 2)
		fatal("servod: Failed to parse Revision string\n");
	if (board_revision < 1)
		fatal("servod: Invalid board Revision\n");
	else if (board_revision < 4)
		gpio_cfg = 1;
	else if (board_revision < 16)
		gpio_cfg = 2;
	else
		gpio_cfg = 3;

	if (board_model == 1) {
		periph_virt_base = 0x20000000;
		periph_phys_base = 0x7e000000;
		dram_phys_base   = 0x40000000;
		mem_flag         = 0x0c;
	} else {
		periph_virt_base = 0x3f000000;
		periph_phys_base = 0x7e000000;
		dram_phys_base   = 0xc0000000;
		mem_flag         = 0x04;
	}
}

static void
parse_pin_lists(int p1first, char *p1pins, char*p5pins)
{
	char *name, *pins;
	int i, mapcnt;
	uint8_t *map, *pNpin2servo;
	int lst, servo = 0;
	FILE *fp;

	memset(servo2gpio, DMY, sizeof(servo2gpio));
	memset(p1pin2servo, DMY, sizeof(p1pin2servo));
	memset(p5pin2servo, DMY, sizeof(p5pin2servo));
	for (lst = 0; lst < 2; lst++) {
		if (lst == 0 && p1first) {
			name = "P1";
			pins = p1pins;
			if (board_model == 1 && gpio_cfg == 1) {
				map = rev1_p1pin2gpio_map;
				mapcnt = sizeof(rev1_p1pin2gpio_map);
			} else if (board_model == 1 && gpio_cfg == 2) {
				map = rev2_p1pin2gpio_map;
				mapcnt = sizeof(rev2_p1pin2gpio_map);
			} else {
				map = bplus_p1pin2gpio_map;
				mapcnt = sizeof(bplus_p1pin2gpio_map);
			}
			pNpin2servo = p1pin2servo;
		} else {
			name = "P5";
			pins = p5pins;
			if (board_model == 1 && gpio_cfg == 1) {
				map = rev1_p5pin2gpio_map;
				mapcnt = sizeof(rev1_p5pin2gpio_map);
			} else if (board_model == 1 && gpio_cfg == 2) {
				map = rev2_p5pin2gpio_map;
				mapcnt = sizeof(rev2_p5pin2gpio_map);
			} else {
				map = NULL;
				mapcnt = 0;
			}
			pNpin2servo = p5pin2servo;
		}
		while (*pins) {
			char *end;
			long pin = strtol(pins, &end, 0);

			if (*end && (end == pins || *end != ','))
				fatal("Invalid character '%c' in %s pin list\n", *end, name);
			if (pin < 0 || pin > mapcnt)
				fatal("Invalid pin number %d in %s pin list\n", pin, name);
			if (servo == MAX_SERVOS)
				fatal("Too many servos specified\n");
			if (pin == 0) {
				servo++;
			} else {
				if (map[pin-1] == DMY)
					fatal("Pin %d on header %s cannot be used for a servo output\n", pin, name);
				pNpin2servo[pin] = servo;
				servo2gpio[servo++] = map[pin-1];
				num_servos++;
			}
			pins = end;
			if (*pins == ',')
				pins++;
		}
	}
	/* Write a cfg file so can tell which pins are used for servos */
	fp = fopen(CFGFILE, "w");
	if (fp) {
		if (p1first)
			fprintf(fp, "p1pins=%s\np5pins=%s\n", p1pins, p5pins);
		else
			fprintf(fp, "p5pins=%s\np1pins=%s\n", p5pins, p1pins);
		fprintf(fp, "\nServo mapping:\n");
		for (i = 0; i < MAX_SERVOS; i++) {
			if (servo2gpio[i] == DMY)
				continue;
			fprintf(fp, "    %2d on %-5s          GPIO-%d\n", i, gpio2pinname(servo2gpio[i]), servo2gpio[i]);
		}
		fclose(fp);
	}
}

static uint8_t
gpiosearch(uint8_t gpio, uint8_t *map, int len)
{
	while (--len) {
		if (map[len] == gpio)
			return len+1;
	}
	return 0;
}

static char *
gpio2pinname(uint8_t gpio)
{
	static char res[16];
	uint8_t pin;

	if (board_model == 1 && gpio_cfg == 1) {
		if ((pin = gpiosearch(gpio, rev1_p1pin2gpio_map, sizeof(rev1_p1pin2gpio_map))))
			sprintf(res, "P1-%d", pin);
		else if ((pin = gpiosearch(gpio, rev1_p5pin2gpio_map, sizeof(rev1_p5pin2gpio_map))))
			sprintf(res, "P5-%d", pin);
		else
			fatal("Cannot map GPIO %d to a header pin\n", gpio);
	} else if (board_model == 1 && gpio_cfg == 2) {
		if ((pin = gpiosearch(gpio, rev2_p1pin2gpio_map, sizeof(rev2_p1pin2gpio_map))))
			sprintf(res, "P1-%d", pin);
		else if ((pin = gpiosearch(gpio, rev2_p5pin2gpio_map, sizeof(rev2_p5pin2gpio_map))))
			sprintf(res, "P5-%d", pin);
		else
			fatal("Cannot map GPIO %d to a header pin\n", gpio);
	} else {
		if ((pin = gpiosearch(gpio, bplus_p1pin2gpio_map, sizeof(bplus_p1pin2gpio_map))))
			sprintf(res, "P1-%d", pin);
		else
			fatal("Cannot map GPIO %d to a header pin\n", gpio);
	}

	return res;
}

static int
parse_min_max_arg(char *arg, char *name)
{
	char *p;
	double val = strtod(arg, &p);

	if (*arg < '0' || *arg > '9' || val < 0) {
		fatal("Invalid %s value specified\n", name);
	} else if (*p == '\0') {
		if (val != floor(val)) {
			fatal("Invalid %s value specified\n", name);
		}
		return (int)val;
	} else if (!strcmp(p, "us")) {
		if (val != floor(val)) {
			fatal("Invalid %s value specified\n", name);
		}
		if ((int)val % step_time_us) {
			fatal("%s value is not a multiple of step-time\n", name);
		}
		return val / step_time_us;
	} else if (!strcmp(p, "%")) {
		if (val < 0 || val > 100.0) {
			fatal("%s value must be between 0% and 100% inclusive\n", name);
		}
		return (int)(val * (double)cycle_time_us / 100.0 / step_time_us);
	} else {
		fatal("Invalid %s value specified\n", name);
	}

	return -1;	/* Never reached */
}

int
main(int argc, char **argv)
{
	int i;
	char *p1pins = default_p1_pins;
	char *p5pins = default_p5_pins;
	int p1first = 1, hadp1 = 0, hadp5 = 0;
	char *servo_min_arg = NULL;
	char *servo_max_arg = NULL;
	char *idle_timeout_arg = NULL;
	char *cycle_time_arg = NULL;
	char *step_time_arg = NULL;
	char *dma_chan_arg = NULL;
	char *p;
	int daemonize = 1;

	setvbuf(stdout, NULL, _IOLBF, 0);

	while (1) {
		int c;
		int option_index;

		static struct option long_options[] = {
			{ "pcm",          no_argument,       0, 'p' },
			{ "idle-timeout", required_argument, 0, 't' },
			{ "help",         no_argument,       0, 'h' },
			{ "p1pins",       required_argument, 0, '1' },
			{ "p5pins",       required_argument, 0, '5' },
			{ "min",          required_argument, 0, 'm' },
			{ "max",          required_argument, 0, 'x' },
			{ "invert",       no_argument,       0, 'i' },
			{ "cycle-time",   required_argument, 0, 'c' },
			{ "step-size",    required_argument, 0, 's' },
			{ "debug",        no_argument,       0, 'f' },
			{ "dma-chan",     required_argument, 0, 'd' },
			{ 0,              0,                 0, 0   }
		};

		c = getopt_long(argc, argv, "mxhnt:15icsfd", long_options, &option_index);
		if (c == -1) {
			break;
		} else if (c =='d') {
			dma_chan_arg = optarg;
		} else if (c == 'f') {
			daemonize = 0;
		} else if (c == 'p') {
			delay_hw = DELAY_VIA_PCM;
		} else if (c == 't') {
			idle_timeout_arg = optarg;
		} else if (c == 'c') {
			cycle_time_arg = optarg;
		} else if (c == 's') {
			step_time_arg = optarg;
		} else if (c == 'm') {
			servo_min_arg = optarg;
		} else if (c == 'x') {
			servo_max_arg = optarg;
		} else if (c == 'i') {
			invert = 1;
		} else if (c == 'h') {
			printf("\nUsage: %s <options>\n\n"
				"Options:\n"
                                "  --pcm               tells servod to use PCM rather than PWM hardware\n"
                                "                      to implement delays\n"
				"  --idle-timeout=Nms  tells servod to stop sending servo pulses for a\n"
				"                      given output N milliseconds after the last update\n"
				"  --cycle-time=Nus    Control pulse cycle time in microseconds, default\n"
				"                      %dus\n"
				"  --step-size=Nus     Pulse width increment step size in microseconds,\n"
				"                      default %dus\n"
				"  --min={N|Nus|N%%}    specifies the minimum allowed pulse width, default\n"
				"                      %d steps or %dus\n"
				"  --max={N|Nus|N%%}    specifies the maximum allowed pulse width, default\n"
				"                      %d steps or %dus\n"
				"  --invert            Inverts outputs\n"
				"  --dma-chan=N        tells servod which dma channel to use, default %d\n"
				"  --p1pins=<list>     tells servod which pins on the P1 header to use\n"
				"  --p5pins=<list>     tells servod which pins on the P5 header to use\n"
				"\nwhere <list> defaults to \"%s\" for p1pins and\n"
				"\"%s\" for p5pins.  p5pins is only valid on rev 2 boards.\n\n"
				"min and max values can be specified in units of steps, in microseconds,\n"
				"or as a percentage of the cycle time.  So, for example, if cycle time is\n"
				"20000us and step size is 10us then the following are equivalent:\n\n"
				"          --min=50   --min=500us    --min=2.5%%\n\n"
				"For the default configuration, example commands to set the first servo\n"
				"to the mid position would be any of:\n\n"
				"  echo 0=150 > /dev/servoblaster        # Specify as a number of steps\n"
				"  echo 0=50%% > /dev/servoblaster        # Specify as a percentage\n"
				"  echo 0=1500us > /dev/servoblaster     # Specify as microseconds\n"
				"  echo P1-7=150 > /dev/servoblaster     # Using P1 pin number rather\n"
				"  echo P1-7=50%% > /dev/servoblaster     # ... than servo number\n"
				"  echo P1-7=1500us > /dev/servoblaster\n\n"
				"Servo adjustments may also be specified relative to the current\n"
				"position by adding a '+' or '-' prefix to the width as follows:\n\n"
				"  echo 0=+10 > /dev/servoblaster\n"
				"  echo 0=-20 > /dev/servoblaster\n\n",
				argv[0],
				DEFAULT_CYCLE_TIME_US,
				DEFAULT_STEP_TIME_US,
				DEFAULT_SERVO_MIN_US/DEFAULT_STEP_TIME_US, DEFAULT_SERVO_MIN_US,
				DEFAULT_SERVO_MAX_US/DEFAULT_STEP_TIME_US, DEFAULT_SERVO_MAX_US,
				DMA_CHAN_DEFAULT, default_p1_pins, default_p5_pins);
			exit(0);
		} else if (c == '1') {
			p1pins = optarg;
			hadp1 = 1;
			if (!hadp5)
				p1first = 1;
		} else if (c == '5') {
			p5pins = optarg;
			hadp5 = 1;
			if (!hadp1)
				p1first = 0;
		} else {
			fatal("Invalid parameter\n");
		}
	}
	get_model_and_revision();
	if (board_model == 1 && gpio_cfg == 1 && p5pins[0])
		fatal("Board model 1 revision 1 does not have a P5 header\n");

	parse_pin_lists(p1first, p1pins, p5pins);

	if (dma_chan_arg) {
		dma_chan = strtol(dma_chan_arg, &p, 10);
		if (*dma_chan_arg < '0' || *dma_chan_arg > '9' ||
				*p || dma_chan < DMA_CHAN_MIN || dma_chan > DMA_CHAN_MAX)
			fatal("Invalid dma-chan specified\n");
	} else {
		dma_chan = DMA_CHAN_DEFAULT;
	}

	if (idle_timeout_arg) {
		idle_timeout = strtol(idle_timeout_arg, &p, 10);
		if (*idle_timeout_arg < '0' || *idle_timeout_arg > '9' ||
				(*p && strcmp(p, "ms")) ||
				idle_timeout < 10 || idle_timeout > 3600000)
			fatal("Invalid idle-timeout specified\n");
	} else {
		idle_timeout = 0;
	}

	if (cycle_time_arg) {
		cycle_time_us = strtol(cycle_time_arg, &p, 10);
		if (*cycle_time_arg < '0' || *cycle_time_arg > '9' ||
				(*p && strcmp(p, "us")) ||
				cycle_time_us < 1000 || cycle_time_us > 1000000)
			fatal("Invalid cycle-time specified\n");
	} else {
		cycle_time_us = DEFAULT_CYCLE_TIME_US;
	}

	if (step_time_arg) {
		step_time_us = strtol(step_time_arg, &p, 10);
		if (*step_time_arg < '0' || *step_time_arg > '9' ||
				(*p && strcmp(p, "us")) ||
				step_time_us < 2 || step_time_us > 1000) {
			fatal("Invalid step-size specified\n");
		}
	} else {
		step_time_us = DEFAULT_STEP_TIME_US;
	}

	if (cycle_time_us % step_time_us) {
		fatal("cycle-time is not a multiple of step-size\n");
	}

	if (cycle_time_us / step_time_us < 100) {
		fatal("cycle-time must be at least 100 * step-size\n");
	}

	if (servo_min_arg) {
		servo_min_ticks = parse_min_max_arg(servo_min_arg, "min");
	} else {
		servo_min_ticks = DEFAULT_SERVO_MIN_US / step_time_us;
	}

	if (servo_max_arg) {
		servo_max_ticks = parse_min_max_arg(servo_max_arg, "max");
	} else {
		servo_max_ticks = DEFAULT_SERVO_MAX_US / step_time_us;
	}

	num_samples = cycle_time_us / step_time_us;
	num_cbs =     num_samples * 2 + MAX_SERVOS;
	num_pages =   (num_cbs * sizeof(dma_cb_t) + num_samples * 4 +
				MAX_SERVOS * 4 + PAGE_SIZE - 1) >> PAGE_SHIFT;

	if (num_pages > MAX_MEMORY_USAGE / PAGE_SIZE) {
		fatal("Using too much memory; reduce cycle-time or increase step-size\n");
	}

	if (servo_max_ticks > num_samples) {
		fatal("max value is larger than cycle time\n");
	}
	if (servo_min_ticks >= servo_max_ticks) {
		fatal("min value is >= max value\n");
	}

	printf("\nBoard model:               %7d\n", board_model);
	printf("GPIO configuration:            %s\n", gpio_desc[gpio_cfg]);
	printf("Using hardware:                %s\n", delay_hw == DELAY_VIA_PWM ? "PWM" : "PCM");
	printf("Using DMA channel:         %7d\n", dma_chan);
	if (idle_timeout)
		printf("Idle timeout:              %7dms\n", idle_timeout);
	else
		printf("Idle timeout:             Disabled\n");
	printf("Number of servos:          %7d\n", num_servos);
	printf("Servo cycle time:          %7dus\n", cycle_time_us);
	printf("Pulse increment step size: %7dus\n", step_time_us);
	printf("Minimum width value:       %7d (%dus)\n", servo_min_ticks,
						servo_min_ticks * step_time_us);
	printf("Maximum width value:       %7d (%dus)\n", servo_max_ticks,
						servo_max_ticks * step_time_us);
	printf("Output levels:            %s\n", invert ? "Inverted" : "  Normal");
	printf("\nUsing P1 pins:               %s\n", p1pins);
	if (board_model == 1 && gpio_cfg == 2)
		printf("Using P5 pins:               %s\n", p5pins);
	printf("\nServo mapping:\n");
	for (i = 0; i < MAX_SERVOS; i++) {
		if (servo2gpio[i] == DMY)
			continue;
		printf("    %2d on %-5s          GPIO-%d\n", i, gpio2pinname(servo2gpio[i]), servo2gpio[i]);
	}
	printf("\n");

	init_idle_timers();
	setup_sighandlers();

	dma_reg = map_peripheral(DMA_VIRT_BASE, DMA_LEN);
	dma_reg += dma_chan * DMA_CHAN_SIZE / sizeof(uint32_t);
	pwm_reg = map_peripheral(PWM_VIRT_BASE, PWM_LEN);
	pcm_reg = map_peripheral(PCM_VIRT_BASE, PCM_LEN);
	clk_reg = map_peripheral(CLK_VIRT_BASE, CLK_LEN);
	gpio_reg = map_peripheral(GPIO_VIRT_BASE, GPIO_LEN);

	/* Use the mailbox interface to the VC to ask for physical memory */
	// Use the mailbox interface to request memory from the VideoCore
	// We specifiy (-1) for the handle rather than calling mbox_open()
	// so multiple users can share the resource.
	mbox.handle = -1; // mbox_open();
	mbox.size = num_pages * 4096;
	mbox.mem_ref = mem_alloc(mbox.handle, mbox.size, 4096, mem_flag);
	if (mbox.mem_ref < 0) {
		fatal("Failed to alloc memory from VideoCore\n");
	}
	mbox.bus_addr = mem_lock(mbox.handle, mbox.mem_ref);
	if (mbox.bus_addr == ~0) {
		mem_free(mbox.handle, mbox.size);
		fatal("Failed to lock memory\n");
	}
	mbox.virt_addr = mapmem(BUS_TO_PHYS(mbox.bus_addr), mbox.size);

	turnoff_mask = (uint32_t *)mbox.virt_addr;
	turnon_mask = (uint32_t *)(mbox.virt_addr + num_samples * sizeof(uint32_t));
	cb_base = (dma_cb_t *)(mbox.virt_addr +
		ROUNDUP(num_samples + MAX_SERVOS, 8) * sizeof(uint32_t));

	for (i = 0; i < MAX_SERVOS; i++) {
		if (servo2gpio[i] == DMY)
			continue;
		gpiomode[i] = gpio_get_mode(servo2gpio[i]);
		gpio_set(servo2gpio[i], invert ? 1 : 0);
		gpio_set_mode(servo2gpio[i], GPIO_MODE_OUT);
	}
	restore_gpio_modes = 1;

	init_ctrl_data();
	init_hardware();

	unlink(DEVFILE);
	if (mkfifo(DEVFILE, 0666) < 0)
		fatal("servod: Failed to create %s: %m\n", DEVFILE);
	if (chmod(DEVFILE, 0666) < 0)
		fatal("servod: Failed to set permissions on %s: %m\n", DEVFILE);

	if (daemonize && daemon(0,1) < 0)
		fatal("servod: Failed to daemonize process: %m\n");

	go_go_go();

	return 0;
}

