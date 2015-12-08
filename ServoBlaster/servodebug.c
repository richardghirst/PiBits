/*
 * servodebug.c - a utility to help debug issues with ServoBlaster
 *
 * This code should be compiled with the command:
 *
 *   gcc -Wall -O2 -o servodebug servodebug.c
 *
 * It should be run with the command:
 *
 *   sudo chrt 1 ./servodebug
 *
 * Richard Hirst - Nov 25th 2012
 */

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include <errno.h>
#include <stdarg.h>
#include <stdint.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <sys/mman.h>
#include <signal.h>
#include <time.h>
#include <sys/time.h>

#define BCM2708_DMA_NO_WIDE_BURSTS	(1<<26)
#define BCM2708_DMA_WAIT_RESP		(1<<3)
#define BCM2708_DMA_D_DREQ		(1<<6)
#define BCM2708_DMA_PER_MAP(x)		((x)<<16)
#define BCM2708_DMA_END			(1<<1)
#define BCM2708_DMA_RESET		(1<<31)
#define BCM2708_DMA_INT			(1<<2)

#define DMA_CHAN_SIZE		0x100
#define DMA_CHAN_MIN		0
#define DMA_CHAN_MAX		14
#define DMA_CHAN_DEFAULT	14

#define DMA_CS			(0x00/4)
#define DMA_CONBLK_AD		(0x04/4)
#define DMA_DEBUG		(0x20/4)

#define GPIO_BASE		0x3f200000
#define GPIO_LEN		0xB4
#define DMA_BASE		0x3f007000
#define DMA_LEN			DMA_CHAN_SIZE * (DMA_CHAN_MAX+1)
#define PWM_BASE		0x3f20C000
#define PWM_LEN			0x28
#define CLK_BASE	        0x3f101000
#define CLK_LEN			0xA8
#define TICK_BASE		0x3f003000
#define TICK_LEN		0x08

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
// I think this means it requests as soon as there is one free slot in the FIFO
// which is what we want as burst DMA would mess up our timing..
#define PWMDMAC_THRSHLD		((15<<8)|(15<<0))

#define GPFSEL0			(0x00/4)
#define GPCLR0			(0x28/4)
#define GPLEV0			(0x34/4)

typedef struct {
	uint32_t info, src, dst, length,
		 stride, next, pad[2];
} dma_cb_t;

static volatile uint32_t *gpio_reg;
static volatile uint32_t *pwm_reg;
static volatile uint32_t *clk_reg;
static volatile uint32_t *dma_reg;
static volatile uint32_t *tick_reg;

static uint32_t dma_chan = DMA_CHAN_DEFAULT;

static uint8_t servo2gpio[] = {
                4,      // P1-7
                17,     // P1-11
#ifdef PWM0_ON_GPIO18
                1,      // P1-5 (GPIO-18, P1-12 is currently PWM0, for debug)
#else
                18,     // P1-12
#endif
                21,     // P1-13
                22,     // P1-15
                23,     // P1-16
                24,     // P1-18
                25,     // P1-22
};
#define NUM_SERVOS      (sizeof(servo2gpio)/sizeof(servo2gpio[0]))

struct {
	uint32_t stamp;
	uint32_t levels;
} trans[20];

static void
fatal(char *fmt, ...)
{
	va_list ap;

	va_start(ap, fmt);
	vfprintf(stderr, fmt, ap);
	va_end(ap);
	exit(1);
}

static void
msleep(int ms)
{
	struct timespec ts = { 0, ms * 1000 * 1000 };

	if (nanosleep(&ts, NULL)) {
		fprintf(stderr, "nanosleep() failed: %s\n", strerror(errno));
		exit(1);
	}
}

static void *
map_peripheral(uint32_t base, uint32_t len)
{
	int fd = open("/dev/mem", O_RDWR);
	void * vaddr;

	if (fd < 0)
		fatal("Failed to open /dev/mem: %m\n");
	vaddr = mmap(NULL, len, PROT_READ|PROT_WRITE, MAP_SHARED, fd, base);
	if (vaddr == MAP_FAILED)
		fatal("Failed to map peripheral at 0x%08x: %m\n", base);
	close(fd);

	return vaddr;
}

int
main(int argc, char **argv)
{
	int tr, i;
	uint32_t v1, v2, mask, t1;
	struct timeval tv1, tv2;

	printf("This code should be compiled with the command:\n\n");
	printf("  gcc -Wall -O2 -o servodebug servodebug.c\n");
	printf("\nIt should be run with the command:\n\n");
	printf("  sudo chrt 1 ./servodebug\n\n");
	gpio_reg = map_peripheral(GPIO_BASE, GPIO_LEN);
	dma_reg = map_peripheral(DMA_BASE, DMA_LEN);
	dma_reg += dma_chan * DMA_CHAN_SIZE / sizeof(uint32_t);
	pwm_reg = map_peripheral(PWM_BASE, PWM_LEN);
	clk_reg = map_peripheral(CLK_BASE, CLK_LEN);
	tick_reg = map_peripheral(TICK_BASE, TICK_LEN);

	v1 = dma_reg[DMA_CONBLK_AD];
	msleep(5);
	v2 = dma_reg[DMA_CONBLK_AD];

	if (v1 == v2)
		fatal("DMA controller is not running - is the module loaded?\n");
	printf("\nGood, DMA controller is running\n");

	printf("\nTiming 1M cycles of system tick, should take 1000000us\n");
	printf("but small variations are to be expected\n");
	gettimeofday(&tv1, NULL);
	v1 = tick_reg[1];
	while (tick_reg[1] - v1 < 1000000)
		;
	gettimeofday(&tv2, NULL);
	i = (tv2.tv_sec - tv1.tv_sec) * 1000000 + tv2.tv_usec - tv1.tv_usec;
	printf("\n1M increments of system tick measured at %dus\n", i);

	printf("\nMonitoring for the first 20 transitions on servo control lines\n");

	for (i = 0, mask = 0; i < NUM_SERVOS; i++)
		mask |= 1 << servo2gpio[i];

	t1 = tick_reg[1];
	v1 = gpio_reg[GPLEV0] & mask;
	for (tr = 0; tr < 20; tr++) {
		do {
			if (tick_reg[1] - t1 > 1000000)
				fatal("No servo pulses generated - have you tried \"echo 0=100 > /dev/servoblaster\"?\n");
			v2 = gpio_reg[GPLEV0] & mask;
		} while (v1 == v2);
		trans[tr].stamp = tick_reg[1];
		trans[tr].levels = v2;
		v1 = v2;
	}

	for (tr = 1; tr < 20; tr++)
		trans[tr].stamp -= trans[0].stamp;
	trans[0].stamp = 0;
	printf("\n    time   outputs\n    (us)   76543210\n");
	for (tr = 0; tr < 20; tr++) {
		printf("   %6d  ", trans[tr].stamp);
		for (i = NUM_SERVOS - 1; i >= 0; i--)
			printf("%s", trans[tr].levels & (1 << servo2gpio[i]) ? "1" : "0");
		printf("\n");
	}
	printf("\n");

	return 0;
}

