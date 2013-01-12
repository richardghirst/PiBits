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

#define GPIO_BASE		0x20200000
#define GPIO_LEN		0x100

#define GPIO_FSEL0		(0x00/4)
#define GPIO_SET0		(0x1c/4)
#define GPIO_CLR0		(0x28/4)
#define GPIO_LEV0		(0x34/4)
#define GPIO_PULLEN		(0x94/4)
#define GPIO_PULLCLK		(0x98/4)

#define GPIO_MODE_IN		0
#define GPIO_MODE_OUT		1

#define CLK_PIN			4
#define DATA_PIN		17


static volatile uint32_t *gpio_reg;

//static int verbose = 1;

static void
udelay(int us)
{
	struct timespec ts = { 0, us * 1000 };

	if (us == 1) {
		volatile uint32_t x = 0;
		int i;
		for (i = 0; i < 30; i++)
			x += gpio_reg[0];
	}
	nanosleep(&ts, NULL);
}

static void
fatal(char *fmt, ...)
{
	va_list ap;

	va_start(ap, fmt);
	vfprintf(stderr, fmt, ap);
	va_end(ap);
	exit(1);
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

static void
gpio_set_mode(uint32_t pin, uint32_t mode)
{
	uint32_t fsel = gpio_reg[GPIO_FSEL0 + pin/10];

	fsel &= ~(7 << ((pin % 10) * 3));
	fsel |= mode << ((pin % 10) * 3);
	gpio_reg[GPIO_FSEL0 + pin/10] = fsel;
//printf("Pin %d mode %d\n", pin, mode);
}

static void
gpio_set(int pin, int level)
{
	if (level)
		gpio_reg[GPIO_SET0] = 1 << pin;
	else
		gpio_reg[GPIO_CLR0] = 1 << pin;
//printf("Pin %d set to %d\n", pin, level ?  1 : 0);
}

static int
gpio_get(int pin)
{
	int level = (gpio_reg[GPIO_LEV0] >> pin) & 1;
//printf("Pin %d read as %d\n", pin, level);

	return level;
}

static uint8_t
read_sensor(int reg)
{
	uint8_t val = 0;
	int i;

	gpio_set(DATA_PIN, 1);
	gpio_set_mode(DATA_PIN, GPIO_MODE_OUT);
	udelay(1);
	for (i = 0x80; i; i >>= 1) {
		gpio_set(CLK_PIN, 0);
		gpio_set(DATA_PIN, reg & i);
		udelay(1);
		gpio_set(CLK_PIN, 1);
		udelay(1);
	}
	gpio_set_mode(DATA_PIN, GPIO_MODE_IN);

	udelay(100);

	for (val = 0, i = 0; i < 8; i++) {
		gpio_set(CLK_PIN, 0);
		udelay(1);
		gpio_set(CLK_PIN, 1);
		udelay(1);
		val <<= 1;
		val |= gpio_get(DATA_PIN);
		udelay(1);
	}

	return val;
}

static void
write_sensor(int reg, int val)
{
	int i;

	reg |= 0x80;
	for (i = 0x80; i; i >>= 1) {
		gpio_set(CLK_PIN, 0);
		udelay(1);
		gpio_set_mode(DATA_PIN, GPIO_MODE_OUT);
		gpio_set(DATA_PIN, reg & i);
		udelay(1);
		gpio_set(CLK_PIN, 1);
		udelay(1);
	}

	for (i = 0x80; i; i >>= 1) {
		gpio_set(CLK_PIN, 0);
		udelay(1);
		gpio_set(DATA_PIN, val & i);
		udelay(1);
		gpio_set(CLK_PIN, 1);
		udelay(1);
	}
	gpio_set_mode(DATA_PIN, GPIO_MODE_IN);
}

int
main(int argc, char **argv)
{
	int i;
	uint8_t v;
	uint8_t img[256];

	gpio_reg = map_peripheral(GPIO_BASE, GPIO_LEN);

	gpio_set(GPIO_PULLEN, 1);
	udelay(1);
	gpio_set(GPIO_PULLCLK, 1 << DATA_PIN);
	udelay(1);
	gpio_set(GPIO_PULLEN, 0);
	gpio_set(GPIO_PULLCLK, 0);

	gpio_set(CLK_PIN, 1);
	gpio_set(DATA_PIN, 1);
	gpio_set_mode(CLK_PIN, GPIO_MODE_OUT);
	gpio_set_mode(DATA_PIN, GPIO_MODE_IN);

	sleep(1);	// Ensure we're synchronized

	write_sensor(0x0a,0x80);
	udelay(1000);
	write_sensor(0x0a,0x01);
	udelay(1000);

	printf("Product ID: %02x Revision: %02x\n", read_sensor(0), read_sensor(1));

	write_sensor(0x0a, 0x09);
	for (i = 0; i < 256; i++) {
		if (read_sensor(0x0d) != i)
			printf("Error, expecting register %d\n", i);
		v = read_sensor(0x0c);
		if (v & 0xc0)
			printf("Bad value %02x\n", v);
		img[i] = v;
	}

	for (i = 255; i >= 0; i--) {
		printf("%02x ", img[i]);
		if ((i & 0x0f) == 0)
			printf("\n");
	}
	printf("\n");

	return 0;
}

