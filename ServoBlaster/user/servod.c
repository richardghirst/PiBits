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
/* TODO: Add slow-start option */

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include <stdarg.h>
#include <stdint.h>
#include <signal.h>
#include <sys/time.h>
#include <fcntl.h>
#include <sys/mman.h>

#include "servod.h"

#define MAX_MEMORY_USAGE	(16*1024*1024)	/* Somewhat arbitrary limit of 16MB */

#define PAGE_SIZE		4096
#define PAGE_SHIFT		12

#define DMA_CHAN_SIZE	0x100
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

#define ROUNDUP(val, blksz)	(((val)+((blksz)-1)) & ~(blksz-1))

typedef struct {
	uint32_t info, src, dst, length,
		 stride, next, pad[2];
} dma_cb_t;

typedef struct {
	uint32_t physaddr;
} page_map_t;

static servod_cfg_t servod_cfg;
servo_t servos[MAX_SERVOS];
static uint32_t move_mask;

uint8_t gpio2servo[NUM_GPIO];
uint8_t p1pin2servo[NUM_P1PINS+1];
uint8_t p5pin2servo[NUM_P5PINS+1];

static page_map_t *page_map;

static uint8_t *virtbase;
static uint8_t *virtcached;

static volatile uint32_t *pwm_reg;
static volatile uint32_t *pcm_reg;
static volatile uint32_t *clk_reg;
static volatile uint32_t *dma_reg;
static volatile uint32_t *gpio_reg;

static struct timeval *servo_kill_time;

static int num_samples;
static int num_cbs;
static int num_pages;
static uint32_t *turnoff_mask;
static uint32_t *turnon_mask;
static dma_cb_t *cb_base;

static char err_buf[512];

static void set_servo_idle(int servo);
static void gpio_set_mode(uint32_t gpio, uint8_t mode);

/* GPIO to RPi headers map */
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

static uint8_t *p1pin2gpio_map;
static uint8_t *p5pin2gpio_map;

static void
udelay(int us)
{
	struct timespec ts = { 0, us * 1000 };

	nanosleep(&ts, NULL);
}

static void
unmap_peripheral(void *memaddr, uint32_t len)
{
	void **vaddr = (void **)memaddr;
	if (*vaddr) {
		munmap(*vaddr, len);
		*vaddr = NULL;
	}
}

void servod_close(void)
{
	int i;

	if (dma_reg && virtbase) {
		for (i = 0; i < MAX_SERVOS; i++) {
			if (servos[i].gpio != DMY)
				set_servo(i, 0);
		}
		udelay(servod_cfg.cycle_time_us);
		dma_reg[DMA_CS] = DMA_RESET;
		udelay(10);
	}

	if (servod_cfg.restore_gpio_modes) {
		for (i = 0; i < MAX_SERVOS; i++) {
			if (servos[i].gpio != DMY)
				gpio_set_mode(servos[i].gpio, servos[i].gpiomode);
		}
	}

	for (i = 0; i < MAX_SERVOS; i++) {
		servos[i].gpio = DMY;
	}

	unmap_peripheral(&dma_reg, DMA_LEN);
	unmap_peripheral(&pwm_reg, PWM_LEN);
	unmap_peripheral(&pcm_reg, PCM_LEN);
	unmap_peripheral(&clk_reg, CLK_LEN);
	unmap_peripheral(&gpio_reg, GPIO_LEN);

	unlink(CFGFILE);
}

/* prints to the given file descriptor */
void
printfd(int fdout, const char *fmt, ...)
{
	va_list ap;
	char buf[MAX_PRINTFD_LEN];

	va_start(ap, fmt);
	vsnprintf(buf, sizeof(buf), fmt, ap);
	write(fdout, buf, strlen(buf));
	va_end(ap);
}

uint8_t gpiosearch(uint8_t gpio, uint8_t *p)
{
	/* First search P1 pins */
	uint8_t *map = p1pin2gpio_map;
	int len = NUM_P1PINS + 1;
	while (--len) {
		if (map[len] == gpio) {
			if (p) *p = PIN_P1HEAD;
			return len+1;
		}
	}
	/* Search P5 pins */
	map = p5pin2gpio_map;
	len = NUM_P5PINS + 1;
	while (--len) {
		if (map[len] == gpio) {
			if (p) *p = PIN_P5HEAD;
			return len+1;
		}
	}

	/* Not found */
	return 0;
}

/* Map pin+header to GPIO 
 * Return value:
 *     -1 in pin/head invalid
 *    DMY if GPIO is invalid
 *    GPIO index on success
 */
int pin2gpio(uint8_t pin, uint8_t head)
{
	int len;
	uint8_t *map;
	
	if (head == PIN_P1HEAD) {
		map = p1pin2gpio_map;
		len = NUM_P1PINS + 1;
	}
	else if (head == PIN_P5HEAD) {
		map = p5pin2gpio_map;
		len = NUM_P5PINS + 1;
	}
	else
		return -1;

	if (pin > len)
		return -1;

	return map[pin -1];
}

static int
logerror(const char *fmt, ...)
{
	va_list ap;

	va_start(ap, fmt);
	vsnprintf(err_buf, sizeof(err_buf), fmt, ap);
	va_end(ap);

	return -1;
}

const char *servod_error(void)
{
	return err_buf;
}

char *gpio2pinname(uint8_t gpio)
{
	static char res[16];
	uint8_t pin, head;

	if ((pin = gpiosearch(gpio, &head))) {
		sprintf(res, "P%d-%d", head, pin);
		return res;
	}

	logerror("Cannot map GPIO %d to a header pin", gpio);
	return NULL;
}

int get_servo_state(uint8_t servo)
{
	return !!turnon_mask[servo];
}

static int
init_idle_timers(void)
{
	servo_kill_time = (struct timeval *)calloc(MAX_SERVOS, sizeof(struct timeval));
	if (!servo_kill_time)
		return logerror("init_idle_timers calloc() failed");
	return 0;
}

static void
update_idle_time(int servo)
{
	if (servod_cfg.idle_timeout == 0)
		return;

	gettimeofday(servo_kill_time + servo, NULL);
	servo_kill_time[servo].tv_sec += servod_cfg.idle_timeout / 1000;
	servo_kill_time[servo].tv_usec += (servod_cfg.idle_timeout % 1000) * 1000;
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

	if (servod_cfg.idle_timeout == 0) {
		*tv = min;
		return;
	}

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

void
servo_speed_handler(void)
{
	if (move_mask == 0)
		return;

	for (int i = 0; i < MAX_SERVOS; i++) {
		if (servos[i].gpio == DMY)
			continue;
		if (servos[i].flags & SRVF_MOVING) {
			int delta = servos[i].setpoint - servos[i].width;
			int dir = (delta < 0) ? -1 : 1;
			delta *= dir;
			if (delta > servos[i].spt)
				delta = servos[i].spt;
			int width = servos[i].width + delta*dir;
			if (servos[i].flags & SRVF_REVERSE)
				width = servos[i].max_width - width + servos[i].min_width; 
			set_servo(i, width);
			if (servos[i].setpoint == servos[i].width) {
				servos[i].flags &= ~SRVF_MOVING;
				move_mask &= ~(1 << i);
			}
		}
	}
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

	if (fd < 0) {
		logerror("Failed to open /dev/mem: %m");
		return NULL;
	}
	vaddr = mmap(NULL, len, PROT_READ|PROT_WRITE, MAP_SHARED, fd, base);
	if (vaddr == MAP_FAILED) {
		logerror("Failed to map peripheral at 0x%08x: %m", base);
		return NULL;
	}
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
		gpio_set(servos[servo].gpio, servod_cfg.invert ? 1 : 0);
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
set_servo(uint8_t servo, int width)
{
	volatile uint32_t *dp;
	int i;
	uint32_t mask = 1 << servos[servo].gpio;

	if (width && servos[servo].flags & SRVF_REVERSE)
		width = servos[servo].max_width - width + servos[servo].min_width;

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
		servos[servo].flags &= ~SRVF_MOVING;
		move_mask &= ~(1 << servo);
	} else {
		turnon_mask[servo] = mask;
	}
	update_idle_time(servo);
}

int
set_servo_ex(uint8_t servo, uint8_t wtype, int width)
{
	if (servo >= MAX_SERVOS || servos[servo].gpio == DMY)
		return -1;

	uint8_t delayed = wtype & WIDTH_DELAY;
	wtype &= ~WIDTH_DELAY;

	/* convert width to ticks if needed */
	switch(wtype) {
	case WIDTH_TICKS:
		break;
	case WIDTH_MICROSEC:
		width = width / servod_cfg.servo_step_time_us;
		break;
	case WIDTH_PERCENT:
		width = (int)((double)width * (0.01 * (servos[servo].max_width - servos[servo].min_width) + 0.5));
		break;
	case WIDTH_PERMILLE:
		width = (int)((double)width * (0.001 * (servos[servo].max_width - servos[servo].min_width) + 0.5));
		break;
	default:
		return -1;
	}

	if (width < servos[servo].min_width || width > servos[servo].max_width)
		return -1;

	if (width && delayed && servos[servo].spt) {
		if (servos[servo].flags & SRVF_REVERSE)
			width = servos[servo].max_width - width + servos[servo].min_width;
		if (servos[servo].setpoint != width) {
			servos[servo].setpoint = width;
			servos[servo].flags |= SRVF_MOVING;
			move_mask |= 1 << servo;
			return 0;
		}
		else {
			servos[servo].flags &= ~SRVF_MOVING;
			move_mask &= ~(1 << servo);
		}
	}

	set_servo(servo, width);

	return 0;
}

int
set_servo_dir(uint8_t servo, uint8_t dir)
{
	if (servo >= MAX_SERVOS)
		return -1;

	if (dir)
		servos[servo].flags |= SRVF_REVERSE;
	else
		servos[servo].flags &= ~SRVF_REVERSE;

	return 0;
}

int
set_servo_speed(uint8_t servo, uint32_t rim)
{
	if (servo >= MAX_SERVOS)
		return -1;

	if (rim == 0) {
		servos[servo].rim = 0;
		servos[servo].spt = 0;
		servos[servo].flags &= ~SRVF_MOVING;
		move_mask &= ~(1 << servo);
		return 0;
	}

	int range = servos[servo].max_width - servos[servo].min_width;
	if (range <= 0) /* invalid range */
		return -1; 

	uint32_t ticks = (rim * 1000) / servod_cfg.cycle_time_us; /* total ticks per range */
	if (ticks == 0)
		return -1;

	double speed = (double)range / (double)ticks; /* steps per tick, double precision */
	int spt = (int)(speed + 0.5); /* steps per tick, integer */
	if (spt == 0)
		spt = 1;

	servos[servo].spt = spt;
	/* calculate actual speed */
	servos[servo].rim = ((range/spt) * servod_cfg.cycle_time_us) / 1000;

	return 0;
}

/* add_servo(.):
 * servo - index 0:MAX_SERVOS-1
 * head  - PIN_P1HEAD, PIN_P5HEAD of PIN_GPIO
 * pin   - pin or gpio number
 * wtype - width type of wmin, wmax
 *         WIDTH_TICK     width in pulse incremental steps
 *         WIDTH_MICROSEC width in microseconds, will be rounded to pulse increment step
 *         WIDTH_PERMILLE width in permilles of pulse cycle time 
 * wmin  - width min value
 * wmax  - width max value
 * itype - initial width value type, INIT_NONE, INIT_TICK, INIT_MICROSEC, INIT_PERCENT
 * init  - initial width value (if itype is not INIT_NONE)
 */
int add_servo(uint8_t servo, uint8_t head, uint8_t pin, uint8_t wtype, int wmin, int wmax, uint8_t itype, int init)
{
	int i, gpio = pin;

	/* lot of sanity checks...*/
	/* check index validity */
	if (servo >= MAX_SERVOS || servos[servo].gpio != DMY)
		return -1;

	if (head != PIN_GPIO) {
		gpio = pin2gpio(pin, head);
		if (gpio < 0 || gpio == DMY)
			return -1;
	}

	/* check gpio validity */
	pin = gpiosearch(gpio, &head);
	if (pin == 0)
		return -1;
	uint8_t *pin2servo = (head == PIN_P1HEAD) ? p1pin2servo : p5pin2servo;
	/* check if gpio already in use */
	for(i = 0; i < MAX_SERVOS; i++) {
		if (servos[i].gpio == gpio)
			return -1;
	}

	/* convert width to ticks if needed */
	switch(wtype) {
	case WIDTH_TICKS:
		break;
	case WIDTH_MICROSEC:
		wmin = wmin / servod_cfg.servo_step_time_us;
		wmax = wmax / servod_cfg.servo_step_time_us;
		break;
	case WIDTH_PERCENT:
		wmin = (int)((double)wmin * ((0.01 * servod_cfg.cycle_time_us) / servod_cfg.servo_step_time_us) + 0.5);
		wmax = (int)((double)wmax * ((0.01 * servod_cfg.cycle_time_us) / servod_cfg.servo_step_time_us) + 0.5);
		break;
	case WIDTH_PERMILLE:
		wmin = (int)((double)wmin * ((0.001 * servod_cfg.cycle_time_us) / servod_cfg.servo_step_time_us) + 0.5);
		wmax = (int)((double)wmax * ((0.001 * servod_cfg.cycle_time_us) / servod_cfg.servo_step_time_us) + 0.5);
		break;
	default:
		return -1;
	}

	/* check pulse boundaries */
	if ((wmin < servod_cfg.servo_min_ticks) || (wmax > servod_cfg.servo_max_ticks))
		return -1;

	/* check init position */
	switch(itype) {
	case INIT_NONE:
	case INIT_TICKS:
		break;
	case INIT_MICROSEC:
		init = init * servod_cfg.servo_step_time_us;
		break;
	case INIT_PERCENT:
		init = (wmax - wmin)/100.0 * init + wmin;
		break;
	case INIT_PERMILLE:
		init = (wmax - wmin)/1000.0 * init + wmin;
		break;
	default:
		return -1;
	}

	servos[servo].gpio = gpio;
	servos[servo].min_width = wmin;
	servos[servo].max_width = wmax;
	if (itype != INIT_NONE) {
		servos[servo].init = init;
		if ((init < wmin) || (init > wmax))
			return -1;
	}
	else
		servos[servo].init = -1;

	/* add mappings */
	gpio2servo[gpio] = servo;
	pin2servo[pin] = servo;

	return 0;
}

static int
make_pagemap(void)
{
	int i, fd, memfd, pid;
	char pagemap_fn[64];

	page_map = (page_map_t *)malloc(num_pages * sizeof(*page_map));
	if (page_map == 0)
		return logerror("Failed to malloc page_map: %m");
	memfd = open("/dev/mem", O_RDWR);
	if (memfd < 0)
		return logerror("Failed to open /dev/mem: %m");
	pid = getpid();
	sprintf(pagemap_fn, "/proc/%d/pagemap", pid);
	fd = open(pagemap_fn, O_RDONLY);
	if (fd < 0)
		return logerror("Failed to open %s: %m\n", pagemap_fn);
	if ((uint32_t)lseek(fd, (uint32_t)(size_t)virtcached >> 9, SEEK_SET) !=
						(uint32_t)(size_t)virtcached >> 9) {
		return logerror("Failed to seek on %s: %m", pagemap_fn);
	}
	for (i = 0; i < num_pages; i++) {
		uint64_t pfn;
		if (read(fd, &pfn, sizeof(pfn)) != sizeof(pfn))
			return logerror("Failed to read %s: %m", pagemap_fn);
		if (((pfn >> 55) & 0x1bf) != 0x10c)
			return logerror("Page %d not present (pfn 0x%016llx)", i, pfn);
		page_map[i].physaddr = (uint32_t)pfn << PAGE_SHIFT | 0x40000000;
		if (mmap(virtbase + i * PAGE_SIZE, PAGE_SIZE, PROT_READ|PROT_WRITE,
			MAP_SHARED|MAP_FIXED|MAP_LOCKED|MAP_NORESERVE,
			memfd, (uint32_t)pfn << PAGE_SHIFT | 0x40000000) !=
				virtbase + i * PAGE_SIZE) {
			return logerror("Failed to create uncached map of page %d at %p",
				i,  virtbase + i * PAGE_SIZE);
		}
	}
	close(fd);
	close(memfd);
	memset(virtbase, 0, num_pages * PAGE_SIZE);

	return 0;
}

static void terminate(int dummy)
{
	servod_close();
	if (servod_cfg.uexit)
		servod_cfg.uexit(servod_cfg.udata);
	exit(-1);
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

	if (servod_cfg.invert) {
		phys_gpclr0 = 0x7e200000 + 0x1c;
		phys_gpset0 = 0x7e200000 + 0x28;
	} else {
		phys_gpclr0 = 0x7e200000 + 0x28;
		phys_gpset0 = 0x7e200000 + 0x1c;
	}

	if (servod_cfg.delay_hw == DELAY_VIA_PWM) {
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
			curstart += num_samples / numservos;
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
	if (servod_cfg.delay_hw == DELAY_VIA_PWM) {
		// Initialise PWM
		pwm_reg[PWM_CTL] = 0;
		udelay(10);
		clk_reg[PWMCLK_CNTL] = 0x5A000006;		// Source=PLLD (500MHz)
		udelay(100);
		clk_reg[PWMCLK_DIV] = 0x5A000000 | (500<<12);	// set pwm div to 500, giving 1MHz
		udelay(100);
		clk_reg[PWMCLK_CNTL] = 0x5A000016;		// Source=PLLD and enable
		udelay(100);
		pwm_reg[PWM_RNG1] = servod_cfg.servo_step_time_us;
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
		pcm_reg[PCM_MODE_A] = (servod_cfg.servo_step_time_us - 1) << 10;
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

	if (servod_cfg.delay_hw == DELAY_VIA_PCM) {
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
int
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
		return logerror("Unable to open /proc/cpuinfo: %m");

	while ((res = fgets(buf, 128, fp))) {
		if (!strncmp(buf, "Revision", 8))
			break;
	}
	fclose(fp);

	if (!res)
		return logerror("No 'Revision' record in /proc/cpuinfo");

	ptr = buf + strlen(buf) - 3;
	rev = strtol(ptr, &end, 16);
	if (end != ptr + 2)
		return logerror("Failed to parse Revision string");
	if (rev < 1)
		return logerror("Invalid board Revision");
	else if (rev < 4)
		rev = 1;
	else
		rev = 2;

	return rev;
}

void
servod_dump(int fdout)
{
	char gpio_pins[128];
	char p1pins[128];
	char p5pins[128];

	memset(gpio_pins, 0, sizeof(gpio_pins));
	memset(p1pins, 0, sizeof(p1pins));
	memset(p5pins, 0, sizeof(p5pins));

	int num_servos = 0;
	/* convert gpio to pins */
	for(int i = 0; i< MAX_SERVOS; i++) {
		if (servos[i].gpio == DMY)
			continue;
		char *pos = strchr(gpio_pins, 0);
		if (pos != gpio_pins)
			*pos++ = ',';
		sprintf(pos, "%d", servos[i].gpio);
		char *pin = gpio2pinname(servos[i].gpio);
		char *pins = p1pins;
		if (pin[1] == '5')
			pins = p5pins;
		pos = strchr(pins, 0);
		if (pos != pins)
			*pos++ = ',';
		strcat(pos, pin + 3);

		num_servos++;
	}

	printfd(fdout, "\nBoard revision:            %7d\n", board_rev());
	printfd(fdout, "Using hardware:                %s\n", servod_cfg.delay_hw == DELAY_VIA_PWM ? "PWM" : "PCM");
	printfd(fdout, "Using DMA channel:         %7d\n", servod_cfg.dma_chan);
	if (servod_cfg.idle_timeout)
		printfd(fdout, "Idle timeout:              %7dms\n", servod_cfg.idle_timeout);
	else
		printfd(fdout, "Idle timeout:             Disabled\n");
	printfd(fdout, "Number of servos:          %7d\n", num_servos);
	printfd(fdout, "Servo cycle time:          %7dus\n", servod_cfg.cycle_time_us);
	printfd(fdout, "Pulse increment step size: %7dus\n", servod_cfg.servo_step_time_us);
	printfd(fdout, "Total number of steps:     %7d\n", servod_cfg.cycle_time_us / servod_cfg.servo_step_time_us);
	printfd(fdout, "Minimum width value:       %7d (%dus, %.2f%%)\n", servod_cfg.servo_min_ticks,
		servod_cfg.servo_min_ticks * servod_cfg.servo_step_time_us,
		(100.0 * servod_cfg.servo_min_ticks) / (servod_cfg.cycle_time_us / servod_cfg.servo_step_time_us));
	printfd(fdout, "Maximum width value:       %7d (%dus, %.2f%%)\n", servod_cfg.servo_max_ticks,
		servod_cfg.servo_max_ticks * servod_cfg.servo_step_time_us,
		(100.0 * servod_cfg.servo_max_ticks) / (servod_cfg.cycle_time_us / servod_cfg.servo_step_time_us));
	printfd(fdout, "Output levels:            %s\n", servod_cfg.invert ? "Inverted" : "  Normal");
	printfd(fdout, "\nUsing GPIO pins:             %s\n", gpio_pins);
	printfd(fdout, "Using P1 pins:               %s\n", p1pins);
	if (board_rev() > 1)
		printfd(fdout, "Using P5 pins:               %s\n", p5pins);
	printfd(fdout, "\nServo mapping:\n");
	printfd(fdout, "     #    Header-pin     GPIO      Range, us     Init, us Speed, ms  Dir\n");
	for (int i = 0; i < MAX_SERVOS; i++) {
		if (servos[i].gpio == DMY)
			continue;
		printfd(fdout, "    %2d on %-5s          GPIO-%d %7d-%-7d",
			i, gpio2pinname(servos[i].gpio), servos[i].gpio,
			servos[i].min_width * servod_cfg.servo_step_time_us,
			servos[i].max_width * servod_cfg.servo_step_time_us);
		if (servos[i].init != -1)
			printfd(fdout, "  %7d", servos[i].init * servod_cfg.servo_step_time_us);
		else
			printfd(fdout, "         ");
		if (servos[i].rim != 0)
			printfd(fdout, "   %7d", servos[i].rim);
		printfd(fdout, "  %s", (servos[i].flags & SRVF_REVERSE) ? "CW" : "CCW");
		printfd(fdout, "\n");
	}
	printfd(fdout, "\n");
}

servod_cfg_t *servod_create(void *data, servod_exit *uexit)
{
	int i;
	int rev;

	/* Initialise maps for commands handling */
	memset(gpio2servo, DMY, sizeof(gpio2servo));
	memset(p1pin2servo, DMY, sizeof(p1pin2servo));
	memset(p5pin2servo, DMY, sizeof(p5pin2servo));

	rev = board_rev();
	if (rev < 1)
		return NULL;

	if (rev == 1) {
		p1pin2gpio_map = rev1_p1pin2gpio_map;
		p5pin2gpio_map = rev1_p5pin2gpio_map;
	} 
	else {
		p1pin2gpio_map = rev2_p1pin2gpio_map;
		p5pin2gpio_map = rev2_p5pin2gpio_map;
	}

	/* Initialize servos */
	memset(servos, 0, sizeof(servos));
	for(i = 0; i < MAX_SERVOS; i++) {
		servos[i].gpio = DMY;
		servos[i].init = -1;
		servos[i].min_width = -1;
		servos[i].max_width = -1;
	}

	/* set default configuration */
	memset(&servod_cfg, 0, sizeof(servod_cfg));
	servod_cfg.delay_hw = DELAY_VIA_PWM;
	servod_cfg.dma_chan = DMA_CHAN_DEFAULT;
	servod_cfg.cycle_time_us = DEFAULT_CYCLE_TIME_US;
	servod_cfg.servo_step_time_us = DEFAULT_STEP_TIME_US;
	servod_cfg.servo_min_ticks = DEFAULT_SERVO_MIN_US / DEFAULT_STEP_TIME_US;
	servod_cfg.servo_max_ticks = DEFAULT_SERVO_MAX_US / DEFAULT_STEP_TIME_US;
	servod_cfg.restore_gpio_modes = 1;
	servod_cfg.setup_sighandlers = 1;
	servod_cfg.udata = data;
	servod_cfg.uexit = uexit;

	return &servod_cfg;
}

int
servod_init(void)
{
	int i, numservos = 0;

	for (i = 0; i < MAX_SERVOS; i++) {
		if (servos[i].gpio == DMY)
			continue;
		numservos++;
	}
	if (numservos == 0)
		return logerror("At least one servo must be configured");

	num_samples = servod_cfg.cycle_time_us / servod_cfg.servo_step_time_us;
	num_cbs     = num_samples * 2 + MAX_SERVOS;
	num_pages   = (num_cbs * sizeof(dma_cb_t) + num_samples * 4 +
		MAX_SERVOS * 4 + PAGE_SIZE - 1) >> PAGE_SHIFT;

	if (num_pages > MAX_MEMORY_USAGE / PAGE_SIZE)
		return logerror("Using too much memory; reduce cycle-time or increase step-size");

	if (servod_cfg.servo_max_ticks > num_samples)
		return logerror("max value is larger than cycle time");

	if (init_idle_timers() != 0)
		return -1;

	if (servod_cfg.setup_sighandlers)
		setup_sighandlers();

	dma_reg = (volatile uint32_t *)map_peripheral(DMA_BASE, DMA_LEN);
	dma_reg += servod_cfg.dma_chan * DMA_CHAN_SIZE / sizeof(uint32_t);
	pwm_reg = (volatile uint32_t *)map_peripheral(PWM_BASE, PWM_LEN);
	pcm_reg = (volatile uint32_t *)map_peripheral(PCM_BASE, PCM_LEN);
	clk_reg = (volatile uint32_t *)map_peripheral(CLK_BASE, CLK_LEN);
	gpio_reg = (volatile uint32_t *)map_peripheral(GPIO_BASE, GPIO_LEN);

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
	virtcached = (uint8_t *)mmap(NULL, num_pages * PAGE_SIZE, PROT_READ|PROT_WRITE,
			MAP_SHARED|MAP_ANONYMOUS|MAP_NORESERVE|MAP_LOCKED,
			-1, 0);
	if (virtcached == MAP_FAILED)
		return logerror("Failed to mmap for cached pages: %m");
	if ((unsigned long)virtcached & (PAGE_SIZE-1))
		return logerror("Virtual address is not page aligned");
	memset(virtcached, 0, num_pages * PAGE_SIZE);

	virtbase = (uint8_t *)mmap(NULL, num_pages * PAGE_SIZE, PROT_READ|PROT_WRITE,
			MAP_SHARED|MAP_ANONYMOUS|MAP_NORESERVE|MAP_LOCKED,
			-1, 0);
	if (virtbase == MAP_FAILED)
		return logerror("Failed to mmap uncached pages: %m");
	if ((unsigned long)virtbase & (PAGE_SIZE-1))
		return logerror("Virtual address is not page aligned");
	munmap(virtbase, num_pages * PAGE_SIZE);

	if (make_pagemap() != 0)
		return -1;

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
		gpio_set(servos[i].gpio, servod_cfg.invert ? 1 : 0);
		gpio_set_mode(servos[i].gpio, GPIO_MODE_OUT);
	}

	init_ctrl_data();
	init_hardware();

	return 0;
}

