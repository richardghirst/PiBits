/*
 * RaspberryPi based FM transmitter.  For the original idea, see:
 *
 * http://www.icrobotics.co.uk/wiki/index.php/Turning_the_Raspberry_Pi_Into_an_FM_Transmitter
 *
 * All credit to Oliver Mattos and Oskar Weigl for creating the original code.
 * 
 * I have taken their idea and reworked it to use the Pi DMA engine, so
 * reducing the CPU overhead for playing a .wav file from 100% to about 1.6%.
 *
 * I have implemented this in user space, using an idea I picked up from Joan
 * on the Raspberry Pi forums - credit to Joan for the DMA from user space
 * idea.
 *
 * The idea of feeding the PWM FIFO in order to pace DMA control blocks comes
 * from ServoBlaster, and I take credit for that :-)
 *
 * This code uses DMA channel 0 and the PWM hardware, with no regard for
 * whether something else might be trying to use it at the same time (such as
 * the 3.5mm jack audio driver).
 *
 * I know nothing much about sound, subsampling, or FM broadcasting, so it is
 * quite likely the sound quality produced by this code can be improved by
 * someone who knows what they are doing.  There may be issues realting to
 * caching, as the user space process just writes to its virtual address space,
 * and expects the DMA controller to see the data; it seems to work for me
 * though.
 *
 * NOTE: THIS CODE MAY WELL CRASH YOUR PI, TRASH YOUR FILE SYSTEMS, AND
 * POTENTIALLY EVEN DAMAGE YOUR HARDWARE.  THIS IS BECAUSE IT STARTS UP THE DMA
 * CONTROLLER USING MEMORY OWNED BY A USER PROCESS.  IF THAT USER PROCESS EXITS
 * WITHOUT STOPPING THE DMA CONTROLLER, ALL HELL COULD BREAK LOOSE AS THE
 * MEMORY GETS REALLOCATED TO OTHER PROCESSES WHILE THE DMA CONTROLLER IS STILL
 * USING IT.  I HAVE ATTEMPTED TO MINIMISE ANY RISK BY CATCHING SIGNALS AND
 * RESETTING THE DMA CONTROLLER BEFORE EXITING, BUT YOU HAVE BEEN WARNED.  I
 * ACCEPT NO LIABILITY OR RESPONSIBILITY FOR ANYTHING THAT HAPPENS AS A RESULT
 * OF YOU RUNNING THIS CODE.  IF IT BREAKS, YOU GET TO KEEP ALL THE PIECES.
 *
 * NOTE ALSO:  THIS MAY BE ILLEGAL IN YOUR COUNTRY.  HERE ARE SOME COMMENTS
 * FROM MORE KNOWLEDGEABLE PEOPLE ON THE FORUM:
 *
 * "Just be aware that in some countries FM broadcast and especially long
 * distance FM broadcast could get yourself into trouble with the law, stray FM
 * broadcasts over Airband aviation is also strictly forbidden."
 *
 * "A low pass filter is really really required for this as it has strong
 * harmonics at the 3rd, 5th 7th and 9th which sit in licensed and rather
 * essential bands, ie GSM, HAM, emergency services and others. Polluting these
 * frequencies is immoral and dangerous, whereas "breaking in" on FM bands is
 * just plain illegal."
 *
 * "Don't get caught, this GPIO use has the potential to exceed the legal
 * limits by about 2000% with a proper aerial."
 *
 *
 * As for the original code, this code is released under the GPL.
 *
 * Richard Hirst <richardghirst@gmail.com>  December 2012
 */

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include <errno.h>
#include <stdarg.h>
#include <stdint.h>
#include <math.h>
#include <time.h>
#include <signal.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <sys/mman.h>

// The .wav file is mono at 22050Hz, which means we have a new sample every
// 45.4us.  We want to adjust the 100MHz core frequency at 10 times that so as
// to provide some level of subsampling to improve quality.  The basic idea is
// to maintain a buffer of 4000 values to write to the clock control register
// and then arrange for the DMA controller to write the values sequentially at
// 4.54us intervals.  The control code can then wake up every 10ms or so and
// populate the buffer with new samples.  At 4.54us per sample, a 4000 sample
// buffer will last a bit over 18ms, so waking every 10ms should be sufficient.
//
// Total memory needed is:
//
// The frequencies		4000 * 4
// CBs to set the frequency	4000 * 32
// CBs to cause delays		4000 * 32
//
// Process can wake every 10ms and update all samples based on where the DMA
// CB is pointed.

#define NUM_SAMPLES		4000
#define NUM_CBS			(NUM_SAMPLES * 2)

#define BCM2708_DMA_NO_WIDE_BURSTS	(1<<26)
#define BCM2708_DMA_WAIT_RESP		(1<<3)
#define BCM2708_DMA_D_DREQ		(1<<6)
#define BCM2708_DMA_PER_MAP(x)		((x)<<16)
#define BCM2708_DMA_END			(1<<1)
#define BCM2708_DMA_RESET		(1<<31)
#define BCM2708_DMA_INT			(1<<2)

#define DMA_CS			(0x00/4)
#define DMA_CONBLK_AD		(0x04/4)
#define DMA_DEBUG		(0x20/4)

#define DMA_BASE		0x20007000
#define DMA_LEN			0x24
#define PWM_BASE		0x2020C000
#define PWM_LEN			0x28
#define CLK_BASE	        0x20101000
#define CLK_LEN			0xA8
#define GPIO_BASE		0x20200000
#define GPIO_LEN		0xB4

#define PWM_CTL			(0x00/4)
#define PWM_DMAC		(0x08/4)
#define PWM_RNG1		(0x10/4)
#define PWM_FIFO		(0x18/4)

#define PWMCLK_CNTL		40
#define PWMCLK_DIV		41

#define GPCLK_CNTL		(0x70/4)
#define GPCLK_DIV		(0x74/4)

#define PWMCTL_MODE1		(1<<1)
#define PWMCTL_PWEN1		(1<<0)
#define PWMCTL_CLRF		(1<<6)
#define PWMCTL_USEF1		(1<<5)

#define PWMDMAC_ENAB		(1<<31)
// I think this means it requests as soon as there is one free slot in the FIFO
// which is what we want as burst DMA would mess up our timing..
#define PWMDMAC_THRSHLD		((15<<8)|(15<<0))

#define GPFSEL0			(0x00/4)

#define PLLFREQ			500000000	// PLLD is running at 500MHz
#define CARRIERFREQ		100000000	// Carrier frequency is 100MHz
// The deviation specifies how wide the signal is. Use 25.0 for WBFM
// (broadcast radio) and about 3.5 for NBFM (walkie-talkie style radio)
#define DEVIATION		25.0

typedef struct {
	uint32_t info, src, dst, length,
		 stride, next, pad[2];
} dma_cb_t;

typedef struct {
	uint8_t *virtaddr;
	uint32_t physaddr;
} page_map_t;

page_map_t *page_map;

static uint8_t *virtbase;

static volatile uint32_t *pwm_reg;
static volatile uint32_t *clk_reg;
static volatile uint32_t *dma_reg;
static volatile uint32_t *gpio_reg;

struct control_data_s {
	dma_cb_t cb[NUM_CBS];
	uint32_t sample[NUM_SAMPLES];
};

#define PAGE_SIZE	4096
#define PAGE_SHIFT	12
#define NUM_PAGES	((sizeof(struct control_data_s) + PAGE_SIZE - 1) >> PAGE_SHIFT)

static struct control_data_s *ctl;

static void
udelay(int us)
{
	struct timespec ts = { 0, us * 1000 };

	nanosleep(&ts, NULL);
}

static void
terminate(int dummy)
{
	if (dma_reg) {
		dma_reg[DMA_CS] = BCM2708_DMA_RESET;
		udelay(10);
	}
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

static uint32_t
mem_virt_to_phys(void *virt)
{
	uint32_t offset = (uint8_t *)virt - virtbase;

	return page_map[offset >> PAGE_SHIFT].physaddr + (offset % PAGE_SIZE);
}

static uint32_t
mem_phys_to_virt(uint32_t phys)
{
	uint32_t pg_offset = phys & (PAGE_SIZE - 1);
	uint32_t pg_addr = phys - pg_offset;
	int i;

	for (i = 0; i < NUM_PAGES; i++) {
		if (page_map[i].physaddr == pg_addr) {
			return (uint32_t)virtbase + i * PAGE_SIZE + pg_offset;
		}
	}
	fatal("Failed to reverse map phys addr %08x\n", phys);

	return 0;
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
	int i, fd, pid, freq_ctl;
	char pagemap_fn[64];

	// Catch all signals possible - it is vital we kill the DMA engine
	// on process exit!
	for (i = 0; i < 64; i++) {
		struct sigaction sa;

		memset(&sa, 0, sizeof(sa));
		sa.sa_handler = terminate;
		sigaction(i, &sa, NULL);
	}

	// Calculate the frequency control word
	// The fractional part is stored in the lower 12 bits
	freq_ctl = ((float)(PLLFREQ / CARRIERFREQ)) * ( 1 << 12 );
		
	dma_reg = map_peripheral(DMA_BASE, DMA_LEN);
	pwm_reg = map_peripheral(PWM_BASE, PWM_LEN);
	clk_reg = map_peripheral(CLK_BASE, CLK_LEN);
	gpio_reg = map_peripheral(GPIO_BASE, GPIO_LEN);

	virtbase = mmap(NULL, NUM_PAGES * PAGE_SIZE, PROT_READ|PROT_WRITE,
			MAP_SHARED|MAP_ANONYMOUS|MAP_NORESERVE|MAP_LOCKED,
			-1, 0);
	if (virtbase == MAP_FAILED)
		fatal("Failed to mmap physical pages: %m\n");
	if ((unsigned long)virtbase & (PAGE_SIZE-1))
		fatal("Virtual address is not page aligned\n");
	printf("Virtual memory mapped at %p\n", virtbase);
	page_map = malloc(NUM_PAGES * sizeof(*page_map));
	if (page_map == 0)
		fatal("Failed to malloc page_map: %m\n");
	pid = getpid();
	sprintf(pagemap_fn, "/proc/%d/pagemap", pid);
	fd = open(pagemap_fn, O_RDONLY);
	if (fd < 0)
		fatal("Failed to open %s: %m\n", pagemap_fn);
	if (lseek(fd, (unsigned long)virtbase >> 9, SEEK_SET) != (unsigned long)virtbase >> 9)
		fatal("Failed to seek on %s: %m\n", pagemap_fn);
//	printf("Page map:\n");
	for (i = 0; i < NUM_PAGES; i++) {
		uint64_t pfn;
		page_map[i].virtaddr = virtbase + i * PAGE_SIZE;
		// Following line forces page to be allocated
		page_map[i].virtaddr[0] = 0;
		if (read(fd, &pfn, sizeof(pfn)) != sizeof(pfn))
			fatal("Failed to read %s: %m\n", pagemap_fn);
		if ((pfn >> 55)&0xfbf != 0x10c)  // pagemap bits: https://www.kernel.org/doc/Documentation/vm/pagemap.txt
			fatal("Page %d not present (pfn 0x%016llx)\n", i, pfn);
		page_map[i].physaddr = (uint32_t)pfn << PAGE_SHIFT | 0x40000000;
//		printf("  %2d: %8p ==> 0x%08x [0x%016llx]\n", i, page_map[i].virtaddr, page_map[i].physaddr, pfn);
	}

	// GPIO4 needs to be ALT FUNC 0 to otuput the clock
	gpio_reg[GPFSEL0] = (gpio_reg[GPFSEL0] & ~(7 << 12)) | (4 << 12);

	// Program GPCLK to use MASH setting 1, so fractional dividers work
	clk_reg[GPCLK_CNTL] = 0x5A << 24 | 6;
	udelay(100);
	clk_reg[GPCLK_CNTL] = 0x5A << 24 | 1 << 9 | 1 << 4 | 6;

	ctl = (struct control_data_s *)virtbase;
	dma_cb_t *cbp = ctl->cb;
	uint32_t phys_sample_dst = 0x7e101074;
	uint32_t phys_pwm_fifo_addr = 0x7e20c000 + 0x18;

	for (i = 0; i < NUM_SAMPLES; i++) {
		ctl->sample[i] = 0x5a << 24 | freq_ctl;	// Silence
		// Write a frequency sample
		cbp->info = BCM2708_DMA_NO_WIDE_BURSTS | BCM2708_DMA_WAIT_RESP;
		cbp->src = mem_virt_to_phys(ctl->sample + i);
		cbp->dst = phys_sample_dst;
		cbp->length = 4;
		cbp->stride = 0;
		cbp->next = mem_virt_to_phys(cbp + 1);
		cbp++;
		// Delay
		cbp->info = BCM2708_DMA_NO_WIDE_BURSTS | BCM2708_DMA_WAIT_RESP | BCM2708_DMA_D_DREQ | BCM2708_DMA_PER_MAP(5);
		cbp->src = mem_virt_to_phys(virtbase);
		cbp->dst = phys_pwm_fifo_addr;
		cbp->length = 4;
		cbp->stride = 0;
		cbp->next = mem_virt_to_phys(cbp + 1);
		cbp++;
	}
	cbp--;
	cbp->next = mem_virt_to_phys(virtbase);

	// Initialise PWM to use a 100MHz clock too, and set the range to
	// 454 bits, which is 4.54us, the rate at which we want to update
	// the GPCLK control register.
	pwm_reg[PWM_CTL] = 0;
	udelay(10);
	clk_reg[PWMCLK_CNTL] = 0x5A000006;              // Source=PLLD and disable
	udelay(100);
	clk_reg[PWMCLK_DIV] = 0x5A000000 | (5<<12);    // set pwm div to 5, for 100MHz
	udelay(100);
	clk_reg[PWMCLK_CNTL] = 0x5A000016;              // Source=PLLD and enable
	udelay(100);
	pwm_reg[PWM_RNG1] = 454;
	udelay(10);
	pwm_reg[PWM_DMAC] = PWMDMAC_ENAB | PWMDMAC_THRSHLD;
	udelay(10);
	pwm_reg[PWM_CTL] = PWMCTL_CLRF;
	udelay(10);
	pwm_reg[PWM_CTL] = PWMCTL_USEF1 | PWMCTL_PWEN1;
	udelay(10);

	// Initialise the DMA
	dma_reg[DMA_CS] = BCM2708_DMA_RESET;
	udelay(10);
	dma_reg[DMA_CS] = BCM2708_DMA_INT | BCM2708_DMA_END;
	dma_reg[DMA_CONBLK_AD] = mem_virt_to_phys(ctl->cb);
	dma_reg[DMA_DEBUG] = 7; // clear debug error flags
	dma_reg[DMA_CS] = 0x10880001;	// go, mid priority, wait for outstanding writes

	// Nearly there.. open the .wav file specified on the cmdline
	fd = 0;

	if (argc > 1) {
        	fd = open(argv[1], 'r');

		if (fd < 0)
			fatal("Failed to open .wav file\n");
	}

        short data[1024];
        int data_len = read(fd, data, sizeof(data));
	if (data_len < 0)
		fatal("Failed to read .wav file\n");
	data_len /= 2;
	if (data_len < 23)
		fatal("Initial read of .wav file too short\n");

	uint32_t last_cb = (uint32_t)ctl->cb;
	int data_index = 22;

	for (;;) {
		usleep(10000);

		uint32_t cur_cb = mem_phys_to_virt(dma_reg[DMA_CONBLK_AD]);
		int last_sample = (last_cb - (uint32_t)virtbase) / (sizeof(dma_cb_t) * 2);
		int this_sample = (cur_cb - (uint32_t)virtbase) / (sizeof(dma_cb_t) * 2);
		int free_slots = this_sample - last_sample;

		if (free_slots < 0)
			free_slots += NUM_SAMPLES;

		while (free_slots >= 10) {
			float dval = (float)(data[data_index])/65536.0 * DEVIATION;
			int intval = (int)((floor)(dval));
			int frac = (int)((dval - (float)intval) * 10.0);
			int j;

			// I'm sure this code could do a better job of subsampling, either by
			// distributing the '+1's evenly across the 10 subsamples, or maybe
			// by taking the previous and next samples in to account too.
			for (j = 0; j < 10; j++) {
				ctl->sample[last_sample++] = (0x5A << 24 | freq_ctl) + (frac > j ? intval + 1 : intval);
				if (last_sample == NUM_SAMPLES)
					last_sample = 0;
			}
			free_slots -= 10;
			if (++data_index >= data_len) {
        			data_len = read(fd, data, sizeof(data));
				data_index = 0;
				if (data_len < 0)
					fatal("Error reading data: %m\n");
				// Should really wait for outstanding samples to be processed here..
				data_len /= 2;
				if (data_len == 0)
					terminate(0);
			}
		}
		last_cb = (uint32_t)virtbase + last_sample * sizeof(dma_cb_t) * 2;
	}

	terminate(0);

	return 0;
}

