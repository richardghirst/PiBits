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

/* Define which GPIOs/P1-pins to use slightly differently for Rev 1 and
 * Rev 2 boards, as P1-13 is different.  On Rev 1 GPIO 27 was used for
 * camera control and GPIO-21 was routed to P1-13, but on Rev 2 it is the
 * other way round.
 */

static uint8_t servo2gpio_rev1[] = {
	4,	// P1-7
	17,	// P1-11
	18,	// P1-12
	21,	// P1-13
	22,	// P1-15
	23,	// P1-16
	24,	// P1-18
	25,	// P1-22
};

static uint8_t servo2gpio_rev2[] = {
	4,	// P1-7
	17,	// P1-11
	18,	// P1-12
	27,	// P1-13
	22,	// P1-15
	23,	// P1-16
	24,	// P1-18
	25,	// P1-22
};

static uint8_t *servo2gpio;
static int num_servos;

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
#define SERVO_TIME_US		(CYCLE_TIME_US/num_servos)
#define SERVO_SAMPLES		(SERVO_TIME_US/SAMPLE_US)
#define SERVO_MIN		0
#define SERVO_MAX		(SERVO_SAMPLES - 1)
#define NUM_SAMPLES		(CYCLE_TIME_US/SAMPLE_US)
#define NUM_CBS			(NUM_SAMPLES*2)

#define NUM_PAGES		((NUM_CBS * 32 + NUM_SAMPLES * 4 + \
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

typedef struct {
	uint32_t info, src, dst, length,
		 stride, next, pad[2];
} dma_cb_t;

struct ctl {
	uint32_t sample[NUM_SAMPLES];
	dma_cb_t cb[NUM_CBS];
};

typedef struct {
	uint8_t *virtaddr;
	uint32_t physaddr;
} page_map_t;

page_map_t *page_map;

static uint8_t *virtbase;

static volatile uint32_t *pwm_reg;
static volatile uint32_t *pcm_reg;
static volatile uint32_t *clk_reg;
static volatile uint32_t *dma_reg;
static volatile uint32_t *gpio_reg;

static int delay_hw = DELAY_VIA_PWM;

static struct timeval *servo_kill_time;

static int idle_timeout = 0;

static void set_servo(int servo, int width);

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
		for (i = 0; i < num_servos; i++)
			set_servo(i, 0);
		udelay(CYCLE_TIME_US);
		dma_reg[DMA_CS] = DMA_RESET;
		udelay(10);
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
	servo_kill_time = calloc(num_servos, sizeof(struct timeval));
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
	for (i = 0; i < num_servos; i++) {
		if (servo_kill_time[i].tv_sec == 0)
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

static void
gpio_set_mode(uint32_t pin, uint32_t mode)
{
	uint32_t fsel = gpio_reg[GPIO_FSEL0 + pin/10];

	fsel &= ~(7 << ((pin % 10) * 3));
	fsel |= mode << ((pin % 10) * 3);
	gpio_reg[GPIO_FSEL0 + pin/10] = fsel;
}

static void
gpio_set(int pin, int level)
{
	if (level)
		gpio_reg[GPIO_SET0] = 1 << pin;
	else
		gpio_reg[GPIO_CLR0] = 1 << pin;
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
set_servo(int servo, int width)
{
	struct ctl *ctl = (struct ctl *)virtbase;
	dma_cb_t *cbp = ctl->cb + servo * SERVO_SAMPLES * 2;
	uint32_t phys_gpclr0 = 0x7e200000 + 0x28;
	uint32_t phys_gpset0 = 0x7e200000 + 0x1c;
	uint32_t *dp = ctl->sample + servo * SERVO_SAMPLES;
	int i;
	uint32_t mask = 1 << servo2gpio[servo];

	dp[width] = mask;

	if (width == 0) {
		cbp->dst = phys_gpclr0;
	} else {
		for (i = width - 1; i > 0; i--)
			dp[i] = 0;
		dp[0] = mask;
		cbp->dst = phys_gpset0;
		update_idle_time(servo);
	}
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
	if (lseek(fd, (uint32_t)virtbase >> 9, SEEK_SET) !=
						(uint32_t)virtbase >> 9) {
		fatal("servod: Failed to seek on %s: %m\n", pagemap_fn);
	}
	for (i = 0; i < NUM_PAGES; i++) {
		uint64_t pfn;
		page_map[i].virtaddr = virtbase + i * PAGE_SIZE;
		// Following line forces page to be allocated
		page_map[i].virtaddr[0] = 0;
		if (read(fd, &pfn, sizeof(pfn)) != sizeof(pfn))
			fatal("servod: Failed to read %s: %m\n", pagemap_fn);
		if (((pfn >> 55) & 0x1bf) != 0x10c)
			fatal("servod: Page %d not present (pfn 0x%016llx)\n", i, pfn);
		page_map[i].physaddr = (uint32_t)pfn << PAGE_SHIFT | 0x40000000;
	}
	close(fd);
	close(memfd);
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
	uint32_t phys_fifo_addr;
	uint32_t phys_gpclr0 = 0x7e200000 + 0x28;
	int servo, i;

	if (delay_hw == DELAY_VIA_PWM)
		phys_fifo_addr = (PWM_BASE | 0x7e000000) + 0x18;
	else
		phys_fifo_addr = (PCM_BASE | 0x7e000000) + 0x04;

	memset(ctl->sample, 0, sizeof(ctl->sample));
	for (servo = 0 ; servo < num_servos; servo++) {
		for (i = 0; i < SERVO_SAMPLES; i++)
			ctl->sample[servo * SERVO_SAMPLES + i] = 1 << servo2gpio[servo];
	}

	for (i = 0; i < NUM_SAMPLES; i++) {
		cbp->info = DMA_NO_WIDE_BURSTS | DMA_WAIT_RESP;
		cbp->src = mem_virt_to_phys(ctl->sample + i);
		cbp->dst = phys_gpclr0;
		cbp->length = 4;
		cbp->stride = 0;
		cbp->next = mem_virt_to_phys(cbp + 1);
		cbp++;
		// Delay
		if (delay_hw == DELAY_VIA_PWM)
			cbp->info = DMA_NO_WIDE_BURSTS | DMA_WAIT_RESP | DMA_D_DREQ | DMA_PER_MAP(5);
		else
			cbp->info = DMA_NO_WIDE_BURSTS | DMA_WAIT_RESP | DMA_D_DREQ | DMA_PER_MAP(2);
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
				if (n !=3 || nl != '\n') {
					fprintf(stderr, "Bad input: %s", line);
				} else if (servo < 0 || servo >= num_servos) {
					fprintf(stderr, "Invalid servo number %d\n", servo);
				} else if (width < SERVO_MIN || width > SERVO_MAX) {
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

int
main(int argc, char **argv)
{
	int i;

	while (1) {
		int c;
		int option_index;

		static struct option long_options[] = {
			{ "pcm",          no_argument,       0, 'p' },
			{ "idle-timeout", required_argument, 0, 't' },
			{ "idle_timeout", required_argument, 0, 't' },
			{ "help",         no_argument,       0, 'h' },
			{ 0,              0,                 0, 0   }
		};

		c = getopt_long(argc, argv, "hpt:", long_options, &option_index);
		if (c == -1) {
			break;
		} else if (c == 'p') {
			delay_hw = DELAY_VIA_PCM;
		} else if (c == 't') {
			idle_timeout = atoi(optarg);
			if (idle_timeout < 10 || idle_timeout > 3600000)
				fatal("Invalid idle_timeout specified\n");
		} else if (c == 'h') {
			printf("\nUsage: %s [--pcm] [--idle-timeout=N]\n\n"
				"Where:\n"
                                "  --pcm             tells servod to use PCM rather than PWM hardware\n"
                                "                    to implement delays\n"
				"  --idle-timeout=N  tells servod to stop sending servo pulses for a\n"
				"                    given output N milliseconds after the last update\n\n",
				argv[0]);
			exit(0);
		} else {
			fatal("Invalid parameter\n");
		}
	}

	/* Select the correct pin mapping based on board rev */
	if (board_rev() == 1) {
		servo2gpio = servo2gpio_rev1;
		num_servos = sizeof(servo2gpio_rev1);
	} else {
		servo2gpio = servo2gpio_rev2;
		num_servos = sizeof(servo2gpio_rev2);
	}

	printf("Board revision:      %5d\n", board_rev());
	printf("Using hardware:      %5s\n", delay_hw == DELAY_VIA_PWM ? "PWM" : "PCM");
	printf("Idle timeout (ms):   %5d\n", idle_timeout);
	printf("Number of servos:    %5d\n", num_servos);
	printf("Servo cycle time:    %5dus\n", CYCLE_TIME_US);
	printf("Pulse width units:   %5dus\n", SAMPLE_US);
	printf("Maximum width value: %5d (%dus)\n", SERVO_MAX,
						SERVO_MAX * SAMPLE_US);

	init_idle_timers();
	setup_sighandlers();

	dma_reg = map_peripheral(DMA_BASE, DMA_LEN);
	pwm_reg = map_peripheral(PWM_BASE, PWM_LEN);
	pcm_reg = map_peripheral(PCM_BASE, PCM_LEN);
	clk_reg = map_peripheral(CLK_BASE, CLK_LEN);
	gpio_reg = map_peripheral(GPIO_BASE, GPIO_LEN);

	virtbase = mmap(NULL, NUM_PAGES * PAGE_SIZE, PROT_READ|PROT_WRITE,
			MAP_SHARED|MAP_ANONYMOUS|MAP_NORESERVE|MAP_LOCKED,
			-1, 0);
	if (virtbase == MAP_FAILED)
		fatal("servod: Failed to mmap physical pages: %m\n");
	if ((unsigned long)virtbase & (PAGE_SIZE-1))
		fatal("servod: Virtual address is not page aligned\n");

	make_pagemap();

	for (i = 0; i < num_servos; i++) {
		gpio_set(servo2gpio[i], 0);
		gpio_set_mode(servo2gpio[i], GPIO_MODE_OUT);
	}

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

