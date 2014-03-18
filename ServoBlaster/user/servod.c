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

#include "servod.h"

#define MAX_MEMORY_USAGE	(16*1024*1024)	/* Somewhat arbitrary limit of 16MB */

#define DEFAULT_CYCLE_TIME_US	20000
#define DEFAULT_STEP_TIME_US	10
#define DEFAULT_SERVO_MIN_US	500
#define DEFAULT_SERVO_MAX_US	2500

#define CFGFILE			"/dev/servoblaster-cfg"

#define PAGE_SIZE		4096
#define PAGE_SHIFT		12

#define DMA_CHAN_SIZE		0x100
#define DMA_CHAN_MIN		0
#define DMA_CHAN_MAX		14
#define DMA_CHAN_DEFAULT	14

#define DMA_BASE		0x20007000
#define DMA_LEN			DMA_CHAN_SIZE * (DMA_CHAN_MAX+1)
#define PWM_BASE		0x2020C000
#define PWM_LEN			0x28
#define CLK_BASE	    0x20101000
#define CLK_LEN			0xA8
#define GPIO_BASE		0x20200000
#define GPIO_LEN		0x100
#define PCM_BASE		0x20203000
#define PCM_LEN			0x24

#define DMA_NO_WIDE_BURSTS	(1<<26)
#define DMA_WAIT_RESP		(1<<3)
#define DMA_D_DREQ		(1<<6)
#define DMA_PER_MAP(x)	((x)<<16)
#define DMA_END			(1<<1)
#define DMA_RESET		(1<<31)
#define DMA_INT			(1<<2)

#define DMA_CS			(0x00/4)
#define DMA_CONBLK_AD	(0x04/4)
#define DMA_DEBUG		(0x20/4)

#define GPIO_FSEL0		(0x00/4)
#define GPIO_SET0		(0x1c/4)
#define GPIO_CLR0		(0x28/4)
#define GPIO_LEV0		(0x34/4)
#define GPIO_PULLEN		(0x94/4)
#define GPIO_PULLCLK	(0x98/4)

#define GPIO_MODE_IN	0
#define GPIO_MODE_OUT	1

#define PWM_CTL			(0x00/4)
#define PWM_DMAC		(0x08/4)
#define PWM_RNG1		(0x10/4)
#define PWM_FIFO		(0x18/4)

#define PWMCLK_CNTL		40
#define PWMCLK_DIV		41

#define PWMCTL_MODE1	(1<<1)
#define PWMCTL_PWEN1	(1<<0)
#define PWMCTL_CLRF		(1<<6)
#define PWMCTL_USEF1	(1<<5)

#define PWMDMAC_ENAB	(1<<31)
#define PWMDMAC_THRSHLD	((15<<8)|(15<<0))

#define PCM_CS_A		(0x00/4)
#define PCM_FIFO_A		(0x04/4)
#define PCM_MODE_A		(0x08/4)
#define PCM_RXC_A		(0x0c/4)
#define PCM_TXC_A		(0x10/4)
#define PCM_DREQ_A		(0x14/4)
#define PCM_INTEN_A		(0x18/4)
#define PCM_INT_STC_A	(0x1c/4)
#define PCM_GRAY		(0x20/4)

#define PCMCLK_CNTL		38
#define PCMCLK_DIV		39

#define DELAY_VIA_PWM	0
#define DELAY_VIA_PCM	1

#define ROUNDUP(val, blksz)	(((val)+((blksz)-1)) & ~(blksz-1))

typedef struct {
	uint32_t info, src, dst, length,
		 stride, next, pad[2];
} dma_cb_t;

typedef struct {
	uint32_t physaddr;
} page_map_t;

/* Define which P1 header pins to use by default.  These are the eight standard
 * GPIO pins (those coloured green in the diagram on this page:
 *    http://elinux.org/Rpi_Low-level_peripherals
 *
 * Which P1 header pins are actually used can be overridden via command line
 * parameter '--p1pins=...'.
 */

static char *default_p1_pins = "7,11,12,13,15,16,18,22";
static char *default_p5_pins = "";
static char *gpio_pins = "";

static uint8_t rev1_p1pin2gpio_map[] = {
	DMY,	// P1-1   3v3
	DMY,	// P1-2   5v
	0,		// P1-3   GPIO 0 (SDA)
	DMY,	// P1-4   5v
	1,		// P1-5   GPIO 1 (SCL)
	DMY,	// P1-6   Ground
	4,		// P1-7   GPIO 4 (GPCLK0)
	14,		// P1-8   GPIO 14 (TXD)
	DMY,	// P1-9   Ground
	15,		// P1-10  GPIO 15 (RXD)
	17,		// P1-11  GPIO 17
	18,		// P1-12  GPIO 18 (PCM_CLK)
	21,		// P1-13  GPIO 21
	DMY,	// P1-14  Ground
	22,		// P1-15  GPIO 22
	23,		// P1-16  GPIO 23
	DMY,	// P1-17  3v3
	24,		// P1-18  GPIO 24
	10,		// P1-19  GPIO 10 (MOSI)
	DMY,	// P1-20  Ground
	9,		// P1-21  GPIO 9 (MISO)
	25,		// P1-22  GPIO 25
	11,		// P1-23  GPIO 11 (SCLK)
	8,		// P1-24  GPIO 8 (CE0)
	DMY,	// P1-25  Ground
	7,		// P1-26  GPIO 7 (CE1)
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
	2,		// P1-3   GPIO 2 (SDA)
	DMY,	// P1-4   5v
	3,		// P1-5   GPIO 3 (SCL)
	DMY,	// P1-6   Ground
	4,		// P1-7   GPIO 4 (GPCLK0)
	14,		// P1-8   GPIO 14 (TXD)
	DMY,	// P1-9   Ground
	15,		// P1-10  GPIO 15 (RXD)
	17,		// P1-11  GPIO 17
	18,		// P1-12  GPIO 18 (PCM_CLK)
	27,		// P1-13  GPIO 27
	DMY,	// P1-14  Ground
	22,		// P1-15  GPIO 22
	23,		// P1-16  GPIO 23
	DMY,	// P1-17  3v3
	24,		// P1-18  GPIO 24
	10,		// P1-19  GPIO 10 (MOSI)
	DMY,	// P1-20  Ground
	9,		// P1-21  GPIO 9 (MISO)
	25,		// P1-22  GPIO 25
	11,		// P1-23  GPIO 11 (SCLK)
	8,		// P1-24  GPIO 8 (CE0)
	DMY,	// P1-25  Ground
	7,		// P1-26  GPIO 7 (CE1)
};

static uint8_t rev2_p5pin2gpio_map[] = {
	DMY,	// P5-1   5v0
	DMY,	// P5-2   3v3
	28,		// P5-3   GPIO 28 (I2C0_SDA)
	29,		// P5-4   GPIO 29 (I2C0_SCL)
	30,		// P5-5   GPIO 30
	31,		// P5-6   GPIO 31
	DMY,	// P5-7   Ground
	DMY,	// P5-8   Ground
};

// cycle_time_us is the pulse cycle time per servo, in microseconds.
// Typically it should be 20ms, or 20000us.

// step_time_us is the pulse width increment granularity, again in microseconds.
// Setting step_time_us too low will likely cause problems as the DMA controller
// will use too much memory bandwidth.  10us is a good value, though you
// might be ok setting it as low as 2us.

static int cycle_time_us;
int servo_step_time_us;
int servo_min_ticks;
int servo_max_ticks;

servo_t servos[MAX_SERVOS];

uint8_t gpio2servo[NUM_GPIO];
uint8_t p1pin2servo[NUM_P1PINS+1];
uint8_t p5pin2servo[NUM_P5PINS+1];
static int num_servos;
static int restore_gpio_modes;

page_map_t *page_map;

static uint8_t *virtbase;
static uint8_t *virtcached;

static volatile uint32_t *pwm_reg;
static volatile uint32_t *pcm_reg;
static volatile uint32_t *clk_reg;
static volatile uint32_t *dma_reg;
static volatile uint32_t *gpio_reg;

static int delay_hw = DELAY_VIA_PWM;

static struct timeval *servo_kill_time;

int daemonize = 1;
uint16_t cmd_port = 0;

static int dma_chan;
static int idle_timeout;
static int invert = 0;
static int num_samples;
static int num_cbs;
static int num_pages;
static uint32_t *turnoff_mask;
uint32_t *turnon_mask;
static dma_cb_t *cb_base;

static void set_servo_idle(int servo);
static void gpio_set_mode(uint32_t gpio, uint8_t mode);
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

	if (dma_reg && virtbase) {
		for (i = 0; i < MAX_SERVOS; i++) {
			if (servos[i].gpio != DMY)
				set_servo(i, 0);
		}
		udelay(cycle_time_us);
		dma_reg[DMA_CS] = DMA_RESET;
		udelay(10);
	}
	if (restore_gpio_modes) {
		for (i = 0; i < MAX_SERVOS; i++) {
			if (servos[i].gpio != DMY)
				gpio_set_mode(servos[i].gpio, servos[i].gpiomode);
		}
	}
	unlink(DEVFILE);
	unlink(CFGFILE);
	exit(1);
}

void
fatal(char *fmt, ...)
{
	va_list ap;

	va_start(ap, fmt);
	vfprintf(stderr, fmt, ap);
	va_end(ap);
	terminate(0);
}

/* prints to the given file descriptor */
void
printfd(int fdout, char *fmt, ...)
{
	va_list ap;
	char buf[512];

	va_start(ap, fmt);
	vsnprintf(buf, sizeof(buf), fmt, ap);
	write(fdout, buf, strlen(buf));
	va_end(ap);
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

void
get_next_idle_timeout(struct timeval *tv)
{
	int i;
	struct timeval now;
	struct timeval min = { 60, 0 };
	long this_diff, min_diff;

	gettimeofday(&now, NULL);
	for (i = 0; i < MAX_SERVOS; i++) {
		if (servos[i].gpio == DMY || servo_kill_time[i].tv_sec == 0)
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

static uint8_t gpio_get_mode(uint8_t gpio)
{
	uint32_t fsel = gpio_reg[GPIO_FSEL0 + gpio/10];

	return (fsel >> ((gpio % 10) * 3)) & 7;
}

static void
gpio_set_mode(uint32_t gpio, uint8_t mode)
{
	uint32_t fsel = gpio_reg[GPIO_FSEL0 + gpio/10];

	fsel &= ~(7 << ((gpio % 10) * 3));
	fsel |= (uint32_t)mode << ((gpio % 10) * 3);
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
	uint32_t offset = (uint8_t *)virt - virtbase;

	return page_map[offset >> PAGE_SHIFT].physaddr + (offset % PAGE_SIZE);
}

static void *
map_peripheral(uint32_t base, uint32_t len)
{
	int fd = open("/dev/mem", O_RDWR);
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
	if (servos[servo].width == num_samples)
		gpio_set(servos[servo].gpio, invert ? 1 : 0);
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
void
set_servo(int servo, int width)
{
	volatile uint32_t *dp;
	int i;
	uint32_t mask = 1 << servos[servo].gpio;

	if (width > servos[servo].width) {
		dp = turnoff_mask + servos[servo].start + width;
		if (dp >= turnoff_mask + num_samples)
			dp -= num_samples;

		for (i = width; i > servos[servo].width; i--) {
			dp--;
			if (dp < turnoff_mask)
				dp = turnoff_mask + num_samples - 1;
			//printf("%5d, clearing at %p\n", dp - ctl->turnoff, dp);
			*dp &= ~mask;
		}
	} else if (width < servos[servo].width) {
		dp = turnoff_mask + servos[servo].start + width;
		if (dp >= turnoff_mask + num_samples)
			dp -= num_samples;

		for (i = width; i < servos[servo].width; i++) {
			//printf("%5d, setting at %p\n", dp - ctl->turnoff, dp);
			*dp++ |= mask;
			if (dp >= turnoff_mask + num_samples)
				dp = turnoff_mask;
		}
	}
	servos[servo].width = width;
	if (width == 0) {
		turnon_mask[servo] = 0;
	} else {
		turnon_mask[servo] = mask;
	}
	update_idle_time(servo);
}

static void
make_pagemap(void)
{
	int i, fd, memfd, pid;
	char pagemap_fn[64];

	page_map = malloc(num_pages * sizeof(*page_map));
	if (page_map == 0)
		fatal("servod: Failed to malloc page_map: %m\n");
	memfd = open("/dev/mem", O_RDWR);
	if (memfd < 0)
		fatal("servod: Failed to open /dev/mem: %m\n");
	pid = getpid();
	sprintf(pagemap_fn, "/proc/%d/pagemap", pid);
	fd = open(pagemap_fn, O_RDONLY);
	if (fd < 0)
		fatal("servod: Failed to open %s: %m\n", pagemap_fn);
	if (lseek(fd, (uint32_t)(size_t)virtcached >> 9, SEEK_SET) !=
						(uint32_t)(size_t)virtcached >> 9) {
		fatal("servod: Failed to seek on %s: %m\n", pagemap_fn);
	}
	for (i = 0; i < num_pages; i++) {
		uint64_t pfn;
		if (read(fd, &pfn, sizeof(pfn)) != sizeof(pfn))
			fatal("servod: Failed to read %s: %m\n", pagemap_fn);
		if (((pfn >> 55) & 0x1bf) != 0x10c)
			fatal("servod: Page %d not present (pfn 0x%016llx)\n", i, pfn);
		page_map[i].physaddr = (uint32_t)pfn << PAGE_SHIFT | 0x40000000;
		if (mmap(virtbase + i * PAGE_SIZE, PAGE_SIZE, PROT_READ|PROT_WRITE,
			MAP_SHARED|MAP_FIXED|MAP_LOCKED|MAP_NORESERVE,
			memfd, (uint32_t)pfn << PAGE_SHIFT | 0x40000000) !=
				virtbase + i * PAGE_SIZE) {
			fatal("Failed to create uncached map of page %d at %p\n",
				i,  virtbase + i * PAGE_SIZE);
		}
	}
	close(fd);
	close(memfd);
	memset(virtbase, 0, num_pages * PAGE_SIZE);
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
		phys_gpclr0 = 0x7e200000 + 0x1c;
		phys_gpset0 = 0x7e200000 + 0x28;
	} else {
		phys_gpclr0 = 0x7e200000 + 0x28;
		phys_gpset0 = 0x7e200000 + 0x1c;
	}

	if (delay_hw == DELAY_VIA_PWM) {
		phys_fifo_addr = (PWM_BASE | 0x7e000000) + 0x18;
		cbinfo = DMA_NO_WIDE_BURSTS | DMA_WAIT_RESP | DMA_D_DREQ | DMA_PER_MAP(5);
	} else {
		phys_fifo_addr = (PCM_BASE | 0x7e000000) + 0x04;
		cbinfo = DMA_NO_WIDE_BURSTS | DMA_WAIT_RESP | DMA_D_DREQ | DMA_PER_MAP(2);
	}

	memset(turnon_mask, 0, MAX_SERVOS * sizeof(*turnon_mask));

	for (servo = 0 ; servo < MAX_SERVOS; servo++) {
		servos[servo].width = 0;
		if (servos[servo].gpio != DMY) {
			numservos++;
			maskall |= 1 << servos[servo].gpio;
		}
	}

	for (i = 0; i < num_samples; i++)
		turnoff_mask[i] = maskall;

	for (servo = 0; servo < MAX_SERVOS; servo++) {
		if (servos[servo].gpio != DMY) {
			servos[servo].start = curstart;
			curstart += num_samples / num_servos;
		}
	}

	servo = 0;
	while (servo < MAX_SERVOS && servos[servo].gpio == DMY)
		servo++;

	for (i = 0; i < num_samples; i++) {
		cbp->info = DMA_NO_WIDE_BURSTS | DMA_WAIT_RESP;
		cbp->src = mem_virt_to_phys(turnoff_mask + i);
		cbp->dst = phys_gpclr0;
		cbp->length = 4;
		cbp->stride = 0;
		cbp->next = mem_virt_to_phys(cbp + 1);
		cbp++;
		if (i == servos[servo].start) {
			cbp->info = DMA_NO_WIDE_BURSTS | DMA_WAIT_RESP;
			cbp->src = mem_virt_to_phys(turnon_mask + servo);
			cbp->dst = phys_gpset0;
			cbp->length = 4;
			cbp->stride = 0;
			cbp->next = mem_virt_to_phys(cbp + 1);
			cbp++;
			servo++;
			while (servo < MAX_SERVOS && servos[servo].gpio == DMY)
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
		pwm_reg[PWM_RNG1] = servo_step_time_us;
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
		pcm_reg[PCM_MODE_A] = (servo_step_time_us - 1) << 10;
		udelay(100);
		pcm_reg[PCM_CS_A] |= 1<<4 | 1<<3;		// Clear FIFOs
		udelay(100);
		pcm_reg[PCM_DREQ_A] = 64<<24 | 64<<8;	// DMA Req when one slot is free?
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

void
do_debug(int fdout)
{
	int i;
	uint32_t mask = 0;
	uint32_t last = 0xffffffff;

	printfd(fdout, "---------------------------\n");
	printfd(fdout, "Servo  Start  Width  TurnOn\n");
	for (i = 0; i < MAX_SERVOS; i++) {
		if (servos[i].gpio != DMY) {
			printfd(fdout, "%3d: %6d %6d %6d\n", i, servos[i].start,
				servos[i].width, !!turnon_mask[i]);
			mask |= 1 << servos[i].gpio;
		}
	}
	printfd(fdout, "\nData:\n");
	for (i = 0; i < num_samples; i++) {
		uint32_t curr = turnoff_mask[i] & mask;
		if (curr != last)
			printfd(fdout, "@%5d: %08x\n", i, curr);
		last = curr;
	}
	printfd(fdout, "---------------------------\n");
}


/* Determining the board revision is a lot more complicated than it should be
 * (see comments in wiringPi for details).  We will just look at the last two
 * digits of the Revision string and treat '00' and '01' as errors, '02' and
 * '03' as rev 1, and any other hex value as rev 2.
 */
static int
board_rev(void)
{
	char buf[128];
	char *ptr, *end, *res;
	static int rev = 0;
	FILE *fp;

	if (rev)
		return rev;

	fp = fopen("/proc/cpuinfo", "r");

	if (!fp)
		fatal("Unable to open /proc/cpuinfo: %m\n");

	while ((res = fgets(buf, 128, fp))) {
		if (!strncmp(buf, "Revision", 8))
			break;
	}
	fclose(fp);

	if (!res)
		fatal("servod: No 'Revision' record in /proc/cpuinfo\n");

	ptr = buf + strlen(buf) - 3;
	rev = strtol(ptr, &end, 16);
	if (end != ptr + 2)
		fatal("servod: Failed to parse Revision string\n");
	if (rev < 1)
		fatal("servod: Invalid board Revision\n");
	else if (rev < 4)
		rev = 1;
	else
		rev = 2;

	return rev;
}

static void
parse_pin_lists(int p1first, char *p1pins, char*p5pins)
{
	char *name, *pins;
	int i, mapcnt;
	uint8_t *map, *pNpin2servo;
	int lst, servo = 0;
	FILE *fp;

	for(i = 0; i< MAX_SERVOS; i++)
		servos[i].gpio = DMY;
	memset(p1pin2servo, DMY, sizeof(p1pin2servo));
	memset(p5pin2servo, DMY, sizeof(p5pin2servo));
	for (lst = 0; lst < 2; lst++) {
		if (lst == 0 && p1first) {
			name = "P1";
			pins = p1pins;
			if (board_rev() == 1) {
				map = rev1_p1pin2gpio_map;
				mapcnt = sizeof(rev1_p1pin2gpio_map);
			} else {
				map = rev2_p1pin2gpio_map;
				mapcnt = sizeof(rev2_p1pin2gpio_map);
			}
			pNpin2servo = p1pin2servo;
		} else {
			name = "P5";
			pins = p5pins;
			if (board_rev() == 1) {
				map = rev1_p5pin2gpio_map;
				mapcnt = sizeof(rev1_p5pin2gpio_map);
			} else {
				map = rev2_p5pin2gpio_map;
				mapcnt = sizeof(rev2_p5pin2gpio_map);
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
				servos[servo++].gpio = map[pin-1];
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
			if (servos[i].gpio == DMY)
				continue;
			fprintf(fp, "    %2d on %-5s          GPIO-%d\n", i, gpio2pinname(servos[i].gpio), servos[i].gpio);
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

	if (board_rev() == 1) {
		if ((pin = gpiosearch(gpio, rev1_p1pin2gpio_map, sizeof(rev1_p1pin2gpio_map))))
			sprintf(res, "P1-%d", pin);
		else if ((pin = gpiosearch(gpio, rev1_p5pin2gpio_map, sizeof(rev1_p5pin2gpio_map))))
			sprintf(res, "P5-%d", pin);
		else
			fatal("Cannot map GPIO %d to a header pin\n", gpio);
	} else {
		if ((pin = gpiosearch(gpio, rev2_p1pin2gpio_map, sizeof(rev2_p1pin2gpio_map))))
			sprintf(res, "P1-%d", pin);
		else if ((pin = gpiosearch(gpio, rev2_p5pin2gpio_map, sizeof(rev2_p5pin2gpio_map))))
			sprintf(res, "P5-%d", pin);
		else
			fatal("Cannot map GPIO %d to a header pin\n", gpio);
	}

	return res;
}

static int
parse_gpio_list(char *gpio, char **p1pins, char **p5pins)
{
	char *pins;
	FILE *fp;
	int servo = 0;
	int p1mapcnt, p5mapcnt;
	uint8_t *p1map, *p5map;
	static char p1list[256];
	static char p5list[256];

	for(int i = 0; i < MAX_SERVOS; i++)
		servos[i].gpio = DMY;

	memset(gpio2servo, DMY, sizeof(gpio2servo));
	memset(p1pin2servo, DMY, sizeof(p1pin2servo));
	memset(p5pin2servo, DMY, sizeof(p5pin2servo));
	memset(p1list, 0, sizeof(p1list));
	memset(p5list, 0, sizeof(p5list));

	if (board_rev() == 1) {
		p1map = rev1_p1pin2gpio_map;
		p1mapcnt = sizeof(rev1_p1pin2gpio_map);
		p5map = rev1_p5pin2gpio_map;
		p5mapcnt = sizeof(rev1_p5pin2gpio_map);
	} else {
		p1map = rev2_p1pin2gpio_map;
		p1mapcnt = sizeof(rev2_p1pin2gpio_map);
		p5map = rev2_p5pin2gpio_map;
		p5mapcnt = sizeof(rev2_p5pin2gpio_map);
	}
	pins = gpio;
	while (*pins) {
		char *end;
		uint8_t pin;
		int gpio = (int)strtol(pins, &end, 0);

		if (*end && (end == pins || *end != ','))
			fatal("Invalid character '%c' in gpio list\n", *end);

		pins = end;
		if (*pins == ',')
			pins++;

		if (gpio == 0) {
			servo++;
			continue;
		}

		if (gpio < 0 || gpio > MAX_GPIO)
			fatal("Invalid pin number %d in gpio list\n", gpio);
		/* find corresponding pin and add it to a list */
		if ((pin = gpiosearch(gpio, p1map, p1mapcnt)) != 0) {
			char *pos = strchr(p1list, 0);
			if (pos != p1list) {
				*pos = ',';
				pos++;
			}
			sprintf(pos, "%d", pin);
			p1pin2servo[pin] = servo;
		}
		else if ((pin = gpiosearch(gpio, p5map, p5mapcnt)) != 0) {
			char *pos = strchr(p5list, 0);
			if (pos != p5list) {
				*pos = ',';
				pos++;
			}
			sprintf(pos, "%d", pin);
			p5pin2servo[pin] = servo;
		}
		if (pin == 0)
			fatal("GPIO %d cannot be used for a servo output\n", gpio);
		gpio2servo[gpio] = servo;
		servos[servo++].gpio = gpio;
		num_servos++;
		if (servo == MAX_SERVOS)
			fatal("Too many servos specified\n");
	}

	/* Write a cfg file so can tell which pins are used for servos */
	fp = fopen(CFGFILE, "w");
	if (fp) {
		fprintf(fp, "gpio=%s\n", gpio);
		fprintf(fp, "p1pins=%s\np5pins=%s\n", p1list, p5list);
		fprintf(fp, "\nServo mapping:\n");
		int i;
		for (i = 0; i < MAX_SERVOS; i++) {
			if (servos[i].gpio == DMY)
				continue;
			fprintf(fp, "    %2d on %-5s          GPIO-%d\n", i, gpio2pinname(servos[i].gpio), servos[i].gpio);
		}
		fclose(fp);
	}

	*p1pins = p1list;
	*p5pins = p5list;
	return servo;
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
		if ((int)val % servo_step_time_us) {
			fatal("%s value is not a multiple of step-time\n", name);
		}
		return val / servo_step_time_us;
	} else if (!strcmp(p, "%")) {
		if (val < 0 || val > 100.0) {
			fatal("%s value must be between 0% and 100% inclusive\n", name);
		}
		return (int)(val * (double)cycle_time_us / 100.0 / servo_step_time_us);
	} else {
		fatal("Invalid %s value specified\n", name);
	}

	return -1;	/* Never reached */
}

static void
parse_servo_min_max(char *arg, const char *name)
{
	char *p;
	int servo;
	double min_ticks, max_ticks;

	p = arg;
	for(p = arg; *p; p++) {
		servo = strtol(p, &p, 10);
		if (*p != ':')
			fatal("Invalid %s format specified\n", name);
		min_ticks = strtod(p+1, &p);
		if (*p != '-')
			fatal("Invalid %s format specified\n", name);
		max_ticks = strtod(p+1, &p);

		if (*p == '%') {
			if (min_ticks < 0 || min_ticks > 100.0 || max_ticks < 0 || max_ticks > 100.0)
				fatal("%s value must be between 0% and 100% inclusive\n", name);

			servos[servo].min_width = (int)(min_ticks * (double)cycle_time_us / 100.0 / servo_step_time_us);
			servos[servo].max_width = (int)(max_ticks * (double)cycle_time_us / 100.0 / servo_step_time_us);
			p++;
			if (*p == '\0')
				break;
			if (*p == ',')
				continue;
			fatal("Invalid %s format specified\n", name);
		}

		if (min_ticks != floor(min_ticks) || max_ticks != floor(max_ticks))
			fatal("Invalid %s value specified\n", name);

		if (*p == ',' || *p == '\0') {
			servos[servo].min_width = min_ticks;
			servos[servo].max_width = max_ticks;
			if (*p == '\0')
				break;
			continue;
		}
		if (p[0] == 'u' && p[1] == 's') {
			if (((int)min_ticks % servo_step_time_us) || ((int)max_ticks % servo_step_time_us))
				fatal("%s value is not a multiple of step-time\n", name);

			servos[servo].min_width = (int)min_ticks / servo_step_time_us;
			servos[servo].max_width = (int)max_ticks / servo_step_time_us;
			p += 2;
			if (*p == '\0')
				break;
			if (*p == ',')
				continue;
		}
		fatal("Invalid %s format specified\n", name);
	}
}

static void
parse_servo_init(char *arg, const char *name)
{
	char *p;
	int servo;
	double init;

	p = arg;
	for(p = arg; *p; p++) {
		servo = strtol(p, &p, 10);
		if (*p != ':')
			fatal("Invalid %s format specified\n", name);
		init = strtod(p+1, &p);
		if (*p == '%') {
			if (init < 0 || init > 100.0 )
				fatal("%s value must be between 0% and 100% inclusive\n", name);

			servos[servo].init = (int)(init / 100.0 * (double)(servos[servo].max_width - servos[servo].min_width)) + servos[servo].min_width;
			p++;
			if (*p == '\0')
				break;
			if (*p == ',')
				continue;
			fatal("Invalid %s format specified\n", name);
		}

		if (init != floor(init))
			fatal("Invalid %s value specified\n", name);

		if (*p == ',' || *p == '\0') {
			servos[servo].init = (int)init;
			if (*p == '\0')
				break;
			continue;
		}
		if (p[0] == 'u' && p[1] == 's') {
			if (((int)init % servo_step_time_us))
				fatal("%s value is not a multiple of step-time\n", name);

			servos[servo].init = (int)(init / servo_step_time_us);
			p += 2;
			if (*p == '\0')
				break;
			if (*p == ',')
				continue;
		}
		fatal("Invalid %s format specified\n", name);
	}
}

int
servod_init(int argc, char **argv)
{
	int i;
	char *p1pins = default_p1_pins;
	char *p5pins = default_p5_pins;
	int p1first = 1, hadp1 = 0, hadp5 = 0, hadgpio = 0;
	char *servo_min_arg = NULL;
	char *servo_max_arg = NULL;
	char *servo_min_max = NULL;
	char *servo_init_arg = NULL;
	char *idle_timeout_arg = NULL;
	char *cycle_time_arg = NULL;
	char *step_time_arg = NULL;
	char *dma_chan_arg = NULL;
	char *p;

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
			{ "gpio",         required_argument, 0, 'g' },
			{ "min",          required_argument, 0, 'm' },
			{ "max",          required_argument, 0, 'x' },
			{ "min-max",      required_argument, 0, 'r' },
			{ "invert",       no_argument,       0, 'i' },
			{ "cycle-time",   required_argument, 0, 'c' },
			{ "step-size",    required_argument, 0, 's' },
			{ "init",         required_argument, 0, 'I' },
			{ "port",         required_argument, 0, 'P' },
			{ "debug",        no_argument,       0, 'f' },
			{ "dma-chan",     required_argument, 0, 'd' },
			{ 0,              0,                 0, 0   }
		};

		c = getopt_long(argc, argv, "mxhnt:15icsfdgIP", long_options, &option_index);
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
				"  --min-max=servo:min-max{N|Nus|N%%} Comma separated list of min/max pulse\n"
				"                      width range for servos. Range must be within the limit\n"
				"                      specified by --min and --max values\n"
				"  --init=servo:{N|Nus|N%%} Comma separated list of initial pulse width for servos\n"
				"  --invert            Inverts outputs\n"
				"  --port=N            TCP port number to listen for set/get commands\n"
				"  --dma-chan=N        tells servod which dma channel to use, default %d\n"
				"  --gpio=<list>       tells servod which GPIO pins to use\n"
				"  --p1pins=<list>     tells servod which pins on the P1 header to use\n"
				"  --p5pins=<list>     tells servod which pins on the P5 header to use\n"
				"\nwhere <list> defaults to \"%s\" for p1pins and\n"
				"\"%s\" for p5pins and gpio.  p5pins is only valid on rev 2 boards.\n\n"
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
				"  echo G4=1500us > /dev/servoblaster    # Using GPIO number\n\n"
				"Servo adjustments may also be specified relative to the current\n"
				"position by adding a '+' or '-' prefix to the width as follows:\n\n"
				"  echo 0=+10 > /dev/servoblaster\n"
				"  echo 0=-20 > /dev/servoblaster\n"
				"If --init was used to specify initial position then servo can be reset\n"
				"using following command:\n"
				"  echo 0=reset > /dev/servoblaster\n\n",
				argv[0],
				DEFAULT_CYCLE_TIME_US,
				DEFAULT_STEP_TIME_US,
				DEFAULT_SERVO_MIN_US/DEFAULT_STEP_TIME_US, DEFAULT_SERVO_MIN_US,
				DEFAULT_SERVO_MAX_US/DEFAULT_STEP_TIME_US, DEFAULT_SERVO_MAX_US,
				DMA_CHAN_DEFAULT, default_p1_pins, default_p5_pins);
			exit(0);
		} else if (c == '1') {
			if (hadgpio)
				fatal("--p1pins cannot be used with --gpio\n");
			p1pins = optarg;
			hadp1 = 1;
			if (!hadp5)
				p1first = 1;
		} else if (c == '5') {
			if (hadgpio)
				fatal("--p5pins cannot be used with --gpio\n");
			p5pins = optarg;
			hadp5 = 1;
			if (!hadp1)
				p1first = 0;
		} else if (c == 'g') {
			if (hadp1 || hadp5)
				fatal("--gpio cannot be used with --p1pins or --p5pins\n");
			hadgpio = 1;
			gpio_pins = optarg;
		} else if (c == 'r') {
			servo_min_max = optarg;
		} else if (c == 'I') {
			servo_init_arg = optarg;
		} else if (c == 'P') {
			uint32_t port = atoi(optarg);
			if (port > 0xFFFF)
				fatal("Invalid port number\n");
			cmd_port = (uint16_t)port;
		} else {
			fatal("Invalid parameter\n");
		}
	}
	if (board_rev() == 1 && p5pins[0])
		fatal("Board rev 1 does not have a P5 header\n");

	if (hadgpio)
		parse_gpio_list(gpio_pins, &p1pins, &p5pins);
	else
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
		servo_step_time_us = strtol(step_time_arg, &p, 10);
		if (*step_time_arg < '0' || *step_time_arg > '9' ||
				(*p && strcmp(p, "us")) ||
				servo_step_time_us < 2 || servo_step_time_us > 1000) {
			fatal("Invalid step-size specified\n");
		}
	} else {
		servo_step_time_us = DEFAULT_STEP_TIME_US;
	}

	if (cycle_time_us % servo_step_time_us) {
		fatal("cycle-time is not a multiple of step-size\n");
	}

	if (cycle_time_us / servo_step_time_us < 100) {
		fatal("cycle-time must be at least 100 * step-size\n");
	}

	if (servo_min_arg) {
		servo_min_ticks = parse_min_max_arg(servo_min_arg, "min");
	} else {
		servo_min_ticks = DEFAULT_SERVO_MIN_US / servo_step_time_us;
	}

	if (servo_max_arg) {
		servo_max_ticks = parse_min_max_arg(servo_max_arg, "max");
	} else {
		servo_max_ticks = DEFAULT_SERVO_MAX_US / servo_step_time_us;
	}

	num_samples = cycle_time_us / servo_step_time_us;
	num_cbs     = num_samples * 2 + MAX_SERVOS;
	num_pages   = (num_cbs * sizeof(dma_cb_t) + num_samples * 4 +
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

	if (servo_min_max)
		parse_servo_min_max(servo_min_max, "min-max");

	for(i = 0; i < MAX_SERVOS; i++) {
		/* sanity check for min width */
		if (servos[i].min_width == 0)
			servos[i].min_width = servo_min_ticks;
		else if (servos[i].min_width < servo_min_ticks)
			fatal("servo %d min value is less than minimum width\n", i);
		/* sanity check for max width */
		if (servos[i].max_width == 0)
			servos[i].max_width = servo_max_ticks;
		else if (servos[i].max_width > servo_max_ticks)
			fatal("servo %d max value is greater than maximum width\n", i);
	}

	if (servo_init_arg)
		parse_servo_init(servo_init_arg, "init");

	for(i = 0; i < MAX_SERVOS; i++) {
		/* sanity check for initial width */
		if (servos[i].init != 0) {
			if ((servos[i].init < servos[i].min_width) || (servos[i].init > servos[i].max_width))
				fatal("servo %d init value %d is outside of min/max limits %d/%d\n", 
					i, servos[i].init, servos[i].min_width, servos[i].max_width);
		}
	}

	printf("\nBoard revision:            %7d\n", board_rev());
	printf("Using hardware:                %s\n", delay_hw == DELAY_VIA_PWM ? "PWM" : "PCM");
	printf("Using DMA channel:         %7d\n", dma_chan);
	if (idle_timeout)
		printf("Idle timeout:              %7dms\n", idle_timeout);
	else
		printf("Idle timeout:             Disabled\n");
	printf("Number of servos:          %7d\n", num_servos);
	printf("Servo cycle time:          %7dus\n", cycle_time_us);
	printf("Pulse increment step size: %7dus\n", servo_step_time_us);
	printf("Minimum width value:       %7d (%dus)\n", servo_min_ticks,
						servo_min_ticks * servo_step_time_us);
	printf("Maximum width value:       %7d (%dus)\n", servo_max_ticks,
						servo_max_ticks * servo_step_time_us);
	printf("Output levels:            %s\n", invert ? "Inverted" : "  Normal");
	printf("\nUsing GPIO pins:             %s\n", gpio_pins);
	printf("Using P1 pins:               %s\n", p1pins);
	if (board_rev() > 1)
		printf("Using P5 pins:               %s\n", p5pins);
	printf("\nServo mapping:\n");
	printf("     #    Header-pin     GPIO      Range, us     Init, us\n");
	for (i = 0; i < MAX_SERVOS; i++) {
		if (servos[i].gpio == DMY)
			continue;
		printf("    %2d on %-5s          GPIO-%d %7d-%-7d",
			i, gpio2pinname(servos[i].gpio), servos[i].gpio,
			servos[i].min_width * servo_step_time_us, servos[i].max_width * servo_step_time_us);
		if (servos[i].init)
			printf(" %d", servos[i].init * servo_step_time_us);
		printf("\n");
	}
	printf("\n");

	init_idle_timers();
	setup_sighandlers();

	dma_reg = map_peripheral(DMA_BASE, DMA_LEN);
	dma_reg += dma_chan * DMA_CHAN_SIZE / sizeof(uint32_t);
	pwm_reg = map_peripheral(PWM_BASE, PWM_LEN);
	pcm_reg = map_peripheral(PCM_BASE, PCM_LEN);
	clk_reg = map_peripheral(CLK_BASE, CLK_LEN);
	gpio_reg = map_peripheral(GPIO_BASE, GPIO_LEN);

	/*
	 * Map the pages to our virtual address space; this reserves them and
	 * locks them in memory.  However, these are L1 & L2 non-coherent
	 * cached pages and we want coherent access to them so the DMA
	 * controller sees our changes immediately.  To get that, we create a
	 * second mapping of the same size and immediately free it.  This gives
	 * us an address in our virtual address space where we can map in a
	 * coherent view of the physical pages that were allocated by the first
	 * mmap(). This coherent mapping happens in make_pagemap().  All
	 * accesses to our memory that is shared with the DMA controller are
	 * via this second coherent mapping.  The memset() below forces the
	 * pages to be allocated.
	 */
	virtcached = mmap(NULL, num_pages * PAGE_SIZE, PROT_READ|PROT_WRITE,
			MAP_SHARED|MAP_ANONYMOUS|MAP_NORESERVE|MAP_LOCKED,
			-1, 0);
	if (virtcached == MAP_FAILED)
		fatal("servod: Failed to mmap for cached pages: %m\n");
	if ((unsigned long)virtcached & (PAGE_SIZE-1))
		fatal("servod: Virtual address is not page aligned\n");
	memset(virtcached, 0, num_pages * PAGE_SIZE);

	virtbase = mmap(NULL, num_pages * PAGE_SIZE, PROT_READ|PROT_WRITE,
			MAP_SHARED|MAP_ANONYMOUS|MAP_NORESERVE|MAP_LOCKED,
			-1, 0);
	if (virtbase == MAP_FAILED)
		fatal("servod: Failed to mmap uncached pages: %m\n");
	if ((unsigned long)virtbase & (PAGE_SIZE-1))
		fatal("servod: Virtual address is not page aligned\n");
	munmap(virtbase, num_pages * PAGE_SIZE);

	make_pagemap();

	/*
	 * Now the memory is all mapped, we can set up the pointers to the
	 * bit masks used to turn outputs on and off, and to the DMA control
	 * blocks.  The control blocks must be 32 byte aligned (so round up
	 * to multiple of 8, as we're then multiplying by 4).
	 */
	turnoff_mask = (uint32_t *)virtbase;
	turnon_mask = (uint32_t *)(virtbase + num_samples * sizeof(uint32_t));
	cb_base = (dma_cb_t *)(virtbase +
		ROUNDUP(num_samples + MAX_SERVOS, 8) * sizeof(uint32_t));

	for (i = 0; i < MAX_SERVOS; i++) {
		if (servos[i].gpio == DMY)
			continue;
		servos[i].gpiomode = gpio_get_mode(servos[i].gpio);
		gpio_set(servos[i].gpio, invert ? 1 : 0);
		gpio_set_mode(servos[i].gpio, GPIO_MODE_OUT);
	}
	restore_gpio_modes = 1;

	init_ctrl_data();
	init_hardware();

	unlink(DEVFILE);
	if (mkfifo(DEVFILE, 0666) < 0)
		fatal("servod: Failed to create %s: %m\n", DEVFILE);
	if (chmod(DEVFILE, 0666) < 0)
		fatal("servod: Failed to set permissions on %s: %m\n", DEVFILE);

	for(i = 0; i < MAX_SERVOS; i++) {
		reset_servo(i);
	}

	return 0;
}
