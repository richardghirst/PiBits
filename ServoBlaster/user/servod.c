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

/* Define which P1 header pins to use by default.  These are the eight standard
 * GPIO pins (those coloured green in the diagram on this page:
 *    http://elinux.org/Rpi_Low-level_peripherals
 *
 * Which P1 header pins are actually used can be overridden via command line
 * parameter '--p1pins=...'.
 */

static char *default_p1_pins = "7,11,12,13,15,16,18,22";
static char *default_p5_pins = "";

#define DMY	255	// Used to represent an invalid P1 pin, or unmapped servo

#define NUM_P1PINS	26
#define NUM_P5PINS	8

#define MAX_SERVOS	21	// Count of GPIO pins on P1 and P5

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

static uint8_t servo2gpio[MAX_SERVOS];
static int servostart[MAX_SERVOS];
static int servowidth[MAX_SERVOS];
static int num_servos;
static uint32_t gpiomode[MAX_SERVOS];
static int restore_gpio_modes;

#define DEVFILE			"/dev/servoblaster"

#define PAGE_SIZE		4096
#define PAGE_SHIFT		12

// CYCLE_TIME_US is the pulse cycle time per servo, in microseconds.
// Typically it should be 20ms, or 20000us.

// SAMPLE_US is the pulse width increment granularity, again in microseconds.
// Setting SAMPLE_US too low will likely cause problems as the DMA controller
// will use too much memory bandwidth.  10us is a good value, though you
// might be ok setting it as low as 2us.

#define CYCLE_TIME_US		20000
#define SAMPLE_US		10
#define DEFAULT_MIN		(500/SAMPLE_US)
#define DEFAULT_MAX		(2500/SAMPLE_US)
#define NUM_SAMPLES		(CYCLE_TIME_US/SAMPLE_US)
#define NUM_CBS			(NUM_SAMPLES*2+MAX_SERVOS)

#define NUM_PAGES		((NUM_CBS * 32 + NUM_SAMPLES * 4 + \
					MAX_SERVOS * 4 + \
					PAGE_SIZE - 1) >> PAGE_SHIFT)

#define DMA_BASE		0x20007000
#define DMA_LEN			0x24
#define PWM_BASE		0x2020C000
#define PWM_LEN			0x28
#define CLK_BASE	        0x20101000
#define CLK_LEN			0xA8
#define GPIO_BASE		0x20200000
#define GPIO_LEN		0x100
#define PCM_BASE		0x20203000
#define PCM_LEN			0x24

#define DMA_NO_WIDE_BURSTS	(1<<26)
#define DMA_WAIT_RESP		(1<<3)
#define DMA_D_DREQ		(1<<6)
#define DMA_PER_MAP(x)		((x)<<16)
#define DMA_END			(1<<1)
#define DMA_RESET		(1<<31)
#define DMA_INT			(1<<2)

#define DMA_CS			(0x00/4)
#define DMA_CONBLK_AD		(0x04/4)
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

struct ctl {
	uint32_t turnoff[NUM_SAMPLES];
	uint32_t turnon[ROUNDUP(MAX_SERVOS,8)];
	dma_cb_t cb[NUM_CBS];
};

typedef struct {
	uint32_t physaddr;
} page_map_t;

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

static int idle_timeout = 0;
static int invert = 0;
static int servo_min = DEFAULT_MIN;
static int servo_max = DEFAULT_MAX;

static void set_servo(int servo, int width);
static void gpio_set_mode(uint32_t gpio, uint32_t mode);

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
			if (servo2gpio[i] != DMY)
				set_servo(i, 0);
		}
		udelay(CYCLE_TIME_US);
		dma_reg[DMA_CS] = DMA_RESET;
		udelay(10);
	}
	if (restore_gpio_modes) {
		for (i = 0; i < MAX_SERVOS; i++) {
			if (servo2gpio[i] != DMY)
				gpio_set_mode(servo2gpio[i], gpiomode[i]);
		}
	}
	unlink(DEVFILE);
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
			set_servo(i, 0);
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

/* 'servo' has already been validated as to refer to a mapped servo */
static void
set_servo(int servo, int width)
{
	struct ctl *ctl = (struct ctl *)virtbase;
	volatile uint32_t *dp;
	int i;
	uint32_t mask = 1 << servo2gpio[servo];

	if (width == servowidth[servo])
		return;

	if (width == 0)
		ctl->turnon[servo] = 0;

	if (width > servowidth[servo]) {
		dp = ctl->turnoff + servostart[servo] + width;
		if (dp >= ctl->turnoff + NUM_SAMPLES)
			dp -= NUM_SAMPLES;

		for (i = width; i > servowidth[servo]; i--) {
			dp--;
			if (dp < ctl->turnoff)
				dp = ctl->turnoff + NUM_SAMPLES - 1;
			//printf("%5d, clearing at %p\n", dp - ctl->turnoff, dp);
			*dp &= ~mask;
		}
	} else {
		dp = ctl->turnoff + servostart[servo] + width;
		if (dp >= ctl->turnoff + NUM_SAMPLES)
			dp -= NUM_SAMPLES;

		for (i = width; i < servowidth[servo]; i++) {
			//printf("%5d, setting at %p\n", dp - ctl->turnoff, dp);
			*dp++ |= mask;
			if (dp >= ctl->turnoff + NUM_SAMPLES)
				dp = ctl->turnoff;
		}
	}
	servowidth[servo] = width;

	if (width)
		ctl->turnon[servo] = mask;

	update_idle_time(servo);
}

static void
make_pagemap(void)
{
	int i, fd, memfd, pid;
	char pagemap_fn[64];

	page_map = malloc(NUM_PAGES * sizeof(*page_map));
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
	if (lseek(fd, (uint32_t)virtcached >> 9, SEEK_SET) !=
						(uint32_t)virtcached >> 9) {
		fatal("servod: Failed to seek on %s: %m\n", pagemap_fn);
	}
	for (i = 0; i < NUM_PAGES; i++) {
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
	memset(virtbase, 0, NUM_PAGES * PAGE_SIZE);
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
	struct ctl *ctl = (struct ctl *)virtbase;
	dma_cb_t *cbp = ctl->cb;
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

	memset(ctl->turnon, 0, sizeof(ctl->turnon));

	for (servo = 0 ; servo < MAX_SERVOS; servo++) {
		servowidth[servo] = 0;
		if (servo2gpio[servo] != DMY) {
			numservos++;
			maskall |= 1 << servo2gpio[servo];
		}
	}

	for (i = 0; i < NUM_SAMPLES; i++)
		ctl->turnoff[i] = maskall;

	for (servo = 0; servo < MAX_SERVOS; servo++) {
		if (servo2gpio[servo] != DMY) {
			servostart[servo] = curstart;
			curstart += NUM_SAMPLES / num_servos;
		}
	}

	servo = 0;
	while (servo < MAX_SERVOS && servo2gpio[servo] == DMY)
		servo++;

	for (i = 0; i < NUM_SAMPLES; i++) {
		cbp->info = DMA_NO_WIDE_BURSTS | DMA_WAIT_RESP;
		cbp->src = mem_virt_to_phys(ctl->turnoff + i);
		cbp->dst = phys_gpclr0;
		cbp->length = 4;
		cbp->stride = 0;
		cbp->next = mem_virt_to_phys(cbp + 1);
		cbp++;
		if (i == servostart[servo]) {
			cbp->info = DMA_NO_WIDE_BURSTS | DMA_WAIT_RESP;
			cbp->src = mem_virt_to_phys(ctl->turnon + servo);
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
		cbp->src = mem_virt_to_phys(ctl);	// Any data will do
		cbp->dst = phys_fifo_addr;
		cbp->length = 4;
		cbp->stride = 0;
		cbp->next = mem_virt_to_phys(cbp + 1);
		cbp++;
	}
	cbp--;
	cbp->next = mem_virt_to_phys(ctl->cb);
}

static void
init_hardware(void)
{
	struct ctl *ctl = (struct ctl *)virtbase;

	if (delay_hw == DELAY_VIA_PWM) {
		// Initialise PWM
		pwm_reg[PWM_CTL] = 0;
		udelay(10);
		clk_reg[PWMCLK_CNTL] = 0x5A000006;		// Source=PLLD (500MHz)
		udelay(100);
		clk_reg[PWMCLK_DIV] = 0x5A000000 | (50<<12);	// set pwm div to 50, giving 10MHz
		udelay(100);
		clk_reg[PWMCLK_CNTL] = 0x5A000016;		// Source=PLLD and enable
		udelay(100);
		pwm_reg[PWM_RNG1] = SAMPLE_US * 10;
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
		clk_reg[PCMCLK_DIV] = 0x5A000000 | (50<<12);	// Set pcm div to 50, giving 10MHz
		udelay(100);
		clk_reg[PCMCLK_CNTL] = 0x5A000016;		// Source=PLLD and enable
		udelay(100);
		pcm_reg[PCM_TXC_A] = 0<<31 | 1<<30 | 0<<20 | 0<<16; // 1 channel, 8 bits
		udelay(100);
		pcm_reg[PCM_MODE_A] = (SAMPLE_US * 10 - 1) << 10;
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
	dma_reg[DMA_CONBLK_AD] = mem_virt_to_phys(ctl->cb);
	dma_reg[DMA_DEBUG] = 7; // clear debug error flags
	dma_reg[DMA_CS] = 0x10880001;	// go, mid priority, wait for outstanding writes

	if (delay_hw == DELAY_VIA_PCM) {
		pcm_reg[PCM_CS_A] |= 1<<2;			// Enable Tx
	}
}

static void
do_debug(void)
{
	int i;
	uint32_t mask = 0;
	uint32_t last = 0xffffffff;
	struct ctl *ctl = (struct ctl *)virtbase;

	printf("Servo  Start  Width\n");
	for (i = 0; i < MAX_SERVOS; i++) {
		if (servo2gpio[i] != DMY) {
			printf("%3d: %6d %6d\n", i, servostart[i], servowidth[i]);
			mask |= 1 << servo2gpio[i];
		}
	}
	printf("\nData:\n");
	for (i = 0; i < NUM_SAMPLES; i++) {
		uint32_t curr = ctl->turnoff[i] & mask;
		if (curr != last)
			printf("%5d: %08x\n", i, curr);
		last = curr;
	}
}

static void
go_go_go(void)
{
	int fd;
	struct timeval tv;
	static char line[128];
	int nchars = 0;
	char nl;

	if ((fd = open(DEVFILE, O_RDWR|O_NONBLOCK)) == -1)
		fatal("servod: Failed to open %s: %m\n", DEVFILE);

	for (;;) {
		int n, width, servo;
		fd_set ifds;

		FD_ZERO(&ifds);
		FD_SET(fd, &ifds);
		get_next_idle_timeout(&tv);
		if ((n = select(fd+1, &ifds, NULL, NULL, &tv)) != 1)
			continue;
		while (read(fd, line+nchars, 1) == 1) {
			if (line[nchars] == '\n') {
				line[++nchars] = '\0';
				nchars = 0;
				n = sscanf(line, "%d=%d%c", &servo, &width, &nl);
				if (!strcmp(line, "debug\n")) {
					do_debug();
				} else if (n !=3 || nl != '\n') {
					fprintf(stderr, "Bad input: %s", line);
				} else if (servo < 0 || servo >= MAX_SERVOS) {
					fprintf(stderr, "Invalid servo number %d\n", servo);
				} else if (servo2gpio[servo] == DMY) {
					fprintf(stderr, "Servo %d is not mapped to a GPIO pin\n", servo);
				} else if (width && (width < servo_min || width > servo_max)) {
					fprintf(stderr, "Invalid width %d\n", width);
				} else {
					set_servo(servo, width);
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
	int mapcnt;
	uint8_t *map;
	int lst, servo = 0;

	memset(servo2gpio, DMY, sizeof(servo2gpio));
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
				servo2gpio[servo++] = map[pin-1];
				num_servos++;
			}
			pins = end;
			if (*pins == ',')
				pins++;
		}
	}
}

static uint8_t gpiosearch(uint8_t gpio, uint8_t *map, int len)
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

int
main(int argc, char **argv)
{
	int i;
	char *p1pins = default_p1_pins;
	char *p5pins = default_p5_pins;
	int p1first = 1, hadp1 = 0, hadp5 = 0;

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
			{ 0,              0,                 0, 0   }
		};

		c = getopt_long(argc, argv, "mxhnt:15i", long_options, &option_index);
		if (c == -1) {
			break;
		} else if (c == 'p') {
			delay_hw = DELAY_VIA_PCM;
		} else if (c == 't') {
			idle_timeout = atoi(optarg);
			if (idle_timeout < 10 || idle_timeout > 3600000)
				fatal("Invalid idle_timeout specified\n");
		} else if (c == 'h') {
			printf("\nUsage: %s <options>\n\n"
				"Options:\n"
                                "  --pcm             tells servod to use PCM rather than PWM hardware\n"
                                "                    to implement delays\n"
				"  --idle-timeout=N  tells servod to stop sending servo pulses for a\n"
				"                    given output N milliseconds after the last update\n"
				"  --min=N           specifies the minimum allowed pulse width, default\n"
				"                    %d or %dus\n"
				"  --max=N           specifies the maximum allowed pulse width, default\n"
				"                    %d or %dus\n"
				"  --invert          Inverts outputs\n"
				"  --p1pins=<list>   tells servod which pins on the P1 header to use\n"
				"  --p5pins=<list>   tells servod which pins on the P5 header to use\n"
				"\nwhere <list> defaults to \"%s\" for p1pins and\n"
				"\"%s\" for p5pins.  p5pins is only valid on rev 2 boards.\n\n",
				argv[0],
				DEFAULT_MIN, DEFAULT_MIN * SAMPLE_US,
				DEFAULT_MAX, DEFAULT_MAX * SAMPLE_US,
				default_p1_pins, default_p5_pins);
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
		} else if (c == 'm') {
			servo_min = atoi(optarg);
		} else if (c == 'x') {
			servo_max = atoi(optarg);
		} else if (c == 'i') {
			invert = 1;
		} else {
			fatal("Invalid parameter\n");
		}
	}
	if (servo_min < 0 || servo_max > NUM_SAMPLES || servo_min >= servo_max)
		fatal("Invalid min and/or max values, min=%d, max=%d\n", servo_min, servo_max);
	if (board_rev() == 1 && p5pins[0])
		fatal("Board rev 1 does not have a P5 header\n");

	parse_pin_lists(p1first, p1pins, p5pins);

	printf("\nBoard revision:          %5d\n", board_rev());
	printf("Using hardware:            %s\n", delay_hw == DELAY_VIA_PWM ? "PWM" : "PCM");
	if (idle_timeout)
		printf("Idle timeout:            %5dms\n", idle_timeout);
	else
		printf("Idle timeout:         Disabled\n");
	printf("Number of servos:        %5d\n", num_servos);
	printf("Servo cycle time:        %5dus\n", CYCLE_TIME_US);
	printf("Pulse width units:       %5dus\n", SAMPLE_US);
	printf("Minimum width value:     %5d (%dus)\n", servo_min,
						servo_min * SAMPLE_US);
	printf("Maximum width value:     %5d (%dus)\n", servo_max,
						servo_max * SAMPLE_US);
	printf("Output levels:        %s\n", invert ? "Inverted" : "  Normal");
	printf("\nUsing P1 pins:           %s\n", p1pins);
	if (board_rev() > 1)
		printf("Using P5 pins:           %s\n", p5pins);
	printf("\nServo mapping:\n");
	for (i = 0; i < MAX_SERVOS; i++) {
		if (servo2gpio[i] == DMY)
			continue;
		printf("    %2d on %-5s          GPIO-%d\n", i, gpio2pinname(servo2gpio[i]), servo2gpio[i]);
	}
	printf("\n");

	init_idle_timers();
	setup_sighandlers();

	dma_reg = map_peripheral(DMA_BASE, DMA_LEN);
	pwm_reg = map_peripheral(PWM_BASE, PWM_LEN);
	pcm_reg = map_peripheral(PCM_BASE, PCM_LEN);
	clk_reg = map_peripheral(CLK_BASE, CLK_LEN);
	gpio_reg = map_peripheral(GPIO_BASE, GPIO_LEN);

	/*
	 * Map the pages to our vitual address space; this reserves them and
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
	virtcached = mmap(NULL, NUM_PAGES * PAGE_SIZE, PROT_READ|PROT_WRITE,
			MAP_SHARED|MAP_ANONYMOUS|MAP_NORESERVE|MAP_LOCKED,
			-1, 0);
	if (virtcached == MAP_FAILED)
		fatal("servod: Failed to mmap for cached pages: %m\n");
	if ((unsigned long)virtcached & (PAGE_SIZE-1))
		fatal("servod: Virtual address is not page aligned\n");
	memset(virtcached, 0, NUM_PAGES * PAGE_SIZE);

	virtbase = mmap(NULL, NUM_PAGES * PAGE_SIZE, PROT_READ|PROT_WRITE,
			MAP_SHARED|MAP_ANONYMOUS|MAP_NORESERVE|MAP_LOCKED,
			-1, 0);
	if (virtbase == MAP_FAILED)
		fatal("servod: Failed to mmap uncached pages: %m\n");
	if ((unsigned long)virtbase & (PAGE_SIZE-1))
		fatal("servod: Virtual address is not page aligned\n");
	munmap(virtbase, NUM_PAGES * PAGE_SIZE);

	make_pagemap();

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

	if (daemon(0,1) < 0)
		fatal("servod: Failed to daemonize process: %m\n");

	go_go_go();

	return 0;
}

