/*
 * servoblaster.c Multiple Servo Driver for the RaspberryPi
 * Copyright (c) 2012 Richard Hirst <richardghirst@gmail.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.	 See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.
 */

/*
 * If you want the device node created automatically create these two
 * files, and make /lib/udev/servoblaster executable (chmod +x):
 *
 * ============= /etc/udev/rules.d/20-servoblaster.rules =============
 * SUBSYSTEM=="module", DEVPATH=="/module/servoblaster", RUN+="/lib/udev/servoblaster"
 * ===================================================================
 *
 * ===================== /lib/udev/servoblaster ======================
 * #!/bin/bash
 *
 * if [ "$ACTION" = "remove" ]; then
 *         rm -f /dev/servoblaster
 * elif [ "$ACTION" = "add" ]; then
 *          major=$( sed -n 's/ servoblaster//p' /proc/devices )
 *        [ "$major" ] && mknod -m 0666 /dev/servoblaster c $major 0
 * fi
 *
 * exit 0
 * ===================================================================
 */

#include <linux/module.h>
#include <linux/string.h>
#include <linux/fs.h>
#include <linux/io.h>
#include <linux/vmalloc.h>
#include <linux/cdev.h>
#include <linux/scatterlist.h>
#include <linux/delay.h>
#include <linux/sched.h>
#include <linux/slab.h>
#include <linux/timer.h>
#include <mach/platform.h>
#include <asm/uaccess.h>
#include <mach/dma.h>

#define GPIO_LEN		0xb4
#define DMA_LEN			0x24
#define PWM_BASE		(BCM2708_PERI_BASE + 0x20C000)
#define PWM_LEN			0x28
#define CLK_BASE		(BCM2708_PERI_BASE + 0x101000)
#define CLK_LEN			0xA8

#define GPFSEL0			(0x00/4)
#define GPFSEL1			(0x04/4)
#define GPSET0			(0x1c/4)
#define GPCLR0			(0x28/4)

#define PWM_CTL			(0x00/4)
#define PWM_STA			(0x04/4)
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
// I think this means it requests as soon as there is one free slot in the FIFO
// which is what we want as burst DMA would mess up our timing..
#define PWMDMAC_THRSHLD	((15<<8)|(15<<0))

#define DMA_CS			(BCM2708_DMA_CS/4)
#define DMA_CONBLK_AD	(BCM2708_DMA_ADDR/4)
#define DMA_DEBUG		(BCM2708_DMA_DEBUG/4)

#define BCM2708_DMA_END				(1<<1)	// Why is this not in mach/dma.h ?
#define BCM2708_DMA_NO_WIDE_BURSTS	(1<<26)

static int dev_open(struct inode *, struct file *);
static int dev_close(struct inode *, struct file *);
static ssize_t dev_read(struct file *, char *, size_t, loff_t *);
static ssize_t dev_write(struct file *, const char *, size_t, loff_t *);
static long dev_ioctl(struct file *, unsigned int, unsigned long);

static struct file_operations fops = 
{
	.open = dev_open,
	.read = dev_read,
	.write = dev_write,
	.release = dev_close,
	.unlocked_ioctl = dev_ioctl,
	.compat_ioctl = dev_ioctl,
};

// Define REV_1 or REV_2 depending on whcih rev of Pi you have.  Alternatively
// just don't try to use P1-13.

//#define REV_1
//#define REV_2

// Map servo channels to GPIO pins
static uint8_t servo2gpio[] = {
		4,	// P1-7
		17,	// P1-11
#ifdef PWM0_ON_GPIO18
		1,	// P1-5 (GPIO-18, P1-12 is currently PWM0, for debug)
#else
		18,	// P1-12
#endif
#if defined(REV_1)
		21,	// P1-13
#elif defined(REV_2)
		27,	// P1-13
#else
  #error "You must define REV_1 or REV_2"
#endif
		22,	// P1-15
		23,	// P1-16
		24,	// P1-18
		25,	// P1-22
};
#define NUM_SERVOS	(sizeof(servo2gpio)/sizeof(servo2gpio[0]))

// Per-servo timeouts, so we can shut down a servo output after some period
// without a new command - some people want this because the report servos
// overheating after a time.
static struct timer_list idle_timer[NUM_SERVOS];

// This struct is used to store all temporary data associated with a given
// open() of /dev/servoblaster
struct private_data
{
	// Stores the return string for a read of /dev/servoblaster
	// Allowing 10 chars per line.
	int rd_len;
	char rd_data[NUM_SERVOS * 10];

	// Stores partial command strings between calls to write(), in case
	// someone uses multiple write() calls to issue a single command.
	int partial_len;
	char partial_cmd[10];

	// If we get bad data on a write() we reject all subsequent writes
	// until the process closes and reopens the device.
	int reject_writes;
};

// Structure of our control data, stored in a 4K page, and accessed by dma controller
struct ctldata_s {
	struct bcm2708_dma_cb cb[NUM_SERVOS * 4];	// gpio-hi, delay, gpio-lo, delay, for each servo output
	uint32_t gpiodata[NUM_SERVOS];			// set-pin, clear-pin values, per servo output
	uint32_t pwmdata;				// the word we write to the pwm fifo
};

static struct ctldata_s *ctl;
static unsigned long ctldatabase;
static volatile uint32_t *gpio_reg;
static volatile uint32_t *dma_reg;
static volatile uint32_t *clk_reg;
static volatile uint32_t *pwm_reg;

static dev_t devno;
static struct cdev my_cdev;
static int my_major;
static int cycle_ticks = 2000;
static int tick_scale = 6;
static int idle_timeout = 0;

// This records the written count values so we can display them on a read()
// call.  Cannot derive data directly from DMA control blocks as current
// algorithm has a special case for a count of zero.
static int servo_pos[NUM_SERVOS] = { 0 };

static void servo_timeout(unsigned long servo)
{
	// Clear GPIO output next time round
	ctl->cb[servo*4+0].dst = ((GPIO_BASE + GPCLR0*4) & 0x00ffffff) | 0x7e000000;
}

// Wait until we're not processing the given servo (actually wait until
// we are not processing the low period of the previous servo, or the
// high period of this one).
static int wait_for_servo(int servo)
{
	local_irq_disable();
	for (;;) {
		int cb = (dma_reg[DMA_CONBLK_AD] - ((uint32_t)ctl->cb & 0x7fffffff)) / sizeof(ctl->cb[0]);

		if (servo > 0) {
			if (cb < servo*4-2 || cb > servo*4+2)
				break;
		} else {
			if (cb > 2 && cb < NUM_SERVOS*4-2)
				break;
		}
		local_irq_enable();
		set_current_state(TASK_INTERRUPTIBLE);
		if (schedule_timeout(1))
			return -EINTR;
		local_irq_disable();
	}
	// Return with IRQs disabled!!!
	return 0;
}

int init_module(void)
{
	int res, i, s;
	
	res = alloc_chrdev_region(&devno, 0, 1, "servoblaster");
	if (res < 0) {
		printk(KERN_WARNING "ServoBlaster: Can't allocated device number\n");
		return res;
	}
	my_major = MAJOR(devno);
	cdev_init(&my_cdev, &fops);
	my_cdev.owner = THIS_MODULE;
	my_cdev.ops = &fops;
	res = cdev_add(&my_cdev, MKDEV(my_major, 0), 1);
	if (res) {
		printk(KERN_WARNING "ServoBlaster: Error %d adding device\n", res);
		unregister_chrdev_region(devno, 1);
		return res;
	}

	for (i = 0; i < NUM_SERVOS; i++)
		setup_timer(idle_timer + i, servo_timeout, i);

	if (idle_timeout && idle_timeout < 20) {
		printk(KERN_WARNING "ServoBlaster: Increased idle timeout to minimum of 20ms\n");
		idle_timeout = 20;
	}

	ctldatabase = __get_free_pages(GFP_KERNEL, 0);
	printk(KERN_INFO "ServoBlaster: Control page is at 0x%lx, cycle_ticks %d, tick_scale %d, idle_timeout %d\n",
			ctldatabase, cycle_ticks, tick_scale, idle_timeout);
	if (ctldatabase == 0) {
		printk(KERN_WARNING "ServoBlaster: alloc_pages failed\n");
		cdev_del(&my_cdev);
		unregister_chrdev_region(devno, 1);
		return -EFAULT;
	}
	ctl = (struct ctldata_s *)ctldatabase;
	gpio_reg = (uint32_t *)ioremap(GPIO_BASE, GPIO_LEN);
	dma_reg  = (uint32_t *)ioremap(DMA_BASE,  DMA_LEN);
	clk_reg  = (uint32_t *)ioremap(CLK_BASE,  CLK_LEN);
	pwm_reg  = (uint32_t *)ioremap(PWM_BASE,  PWM_LEN);

	memset(ctl, 0, sizeof(*ctl));

	// Set all servo control pins to be outputs
	for (i = 0; i < NUM_SERVOS; i++) {
		int gpio = servo2gpio[i];
		int fnreg = gpio/10 + GPFSEL0;
		int fnshft = (gpio %10) * 3;
		gpio_reg[GPCLR0] = 1 << gpio;
		gpio_reg[fnreg] = (gpio_reg[fnreg] & ~(7 << fnshft)) | (1 << fnshft);
	}
#ifdef PWM0_ON_GPIO18
	// Set pwm0 output on gpio18
	gpio_reg[GPCLR0] = 1 << 18;
	gpio_reg[GPFSEL1] = (gpio_reg[GPFSEL1] & ~(7 << 8*3)) | ( 2 << 8*3);
#endif

	// Build the DMA CB chain
	for (s = 0; s < NUM_SERVOS; s++) {
		int i = s*4;
		// Set gpio high
		ctl->gpiodata[s] = 1 << servo2gpio[s];
		ctl->cb[i].info   = BCM2708_DMA_NO_WIDE_BURSTS | BCM2708_DMA_WAIT_RESP;
		ctl->cb[i].src    = (uint32_t)(&ctl->gpiodata[s]) & 0x7fffffff;
		// We clear the GPIO here initially, so outputs go to 0 on startup
		// Once someone writes to /dev/servoblaster we'll patch it to a 'set'
		ctl->cb[i].dst    = ((GPIO_BASE + GPCLR0*4) & 0x00ffffff) | 0x7e000000;
		ctl->cb[i].length = sizeof(uint32_t);
		ctl->cb[i].stride = 0;
		ctl->cb[i].next = (uint32_t)(ctl->cb + i + 1) & 0x7fffffff;
		// delay
		i++;
		ctl->cb[i].info   = BCM2708_DMA_NO_WIDE_BURSTS | BCM2708_DMA_WAIT_RESP | BCM2708_DMA_D_DREQ | BCM2708_DMA_PER_MAP(5);
		ctl->cb[i].src    = (uint32_t)(&ctl->pwmdata) & 0x7fffffff;
		ctl->cb[i].dst    = ((PWM_BASE + PWM_FIFO*4) & 0x00ffffff) | 0x7e000000;
		ctl->cb[i].length = sizeof(uint32_t) * 1;	// default 1 tick high
		ctl->cb[i].stride = 0;
		ctl->cb[i].next = (uint32_t)(ctl->cb + i + 1) & 0x7fffffff;
		// Set gpio lo
		i++;
		ctl->cb[i].info   = BCM2708_DMA_NO_WIDE_BURSTS | BCM2708_DMA_WAIT_RESP;
		ctl->cb[i].src    = (uint32_t)(&ctl->gpiodata[s]) & 0x7fffffff;
		ctl->cb[i].dst    = ((GPIO_BASE + GPCLR0*4) & 0x00ffffff) | 0x7e000000;
		ctl->cb[i].length = sizeof(uint32_t);
		ctl->cb[i].stride = 0;
		ctl->cb[i].next = (uint32_t)(ctl->cb + i + 1) & 0x7fffffff;
		// delay
		i++;
		ctl->cb[i].info   = BCM2708_DMA_NO_WIDE_BURSTS | BCM2708_DMA_WAIT_RESP | BCM2708_DMA_D_DREQ | BCM2708_DMA_PER_MAP(5);
		ctl->cb[i].src    = (uint32_t)(&ctl->pwmdata) & 0x7fffffff;
		ctl->cb[i].dst    = ((PWM_BASE + PWM_FIFO*4) & 0x00ffffff) | 0x7e000000;
		ctl->cb[i].length = sizeof(uint32_t) * (cycle_ticks / 8 - 1);
		ctl->cb[i].stride = 0;
		ctl->cb[i].next = (uint32_t)(ctl->cb + i + 1) & 0x7fffffff;
	}
	// Point last cb back to first one so it loops continuously
	ctl->cb[NUM_SERVOS*4-1].next = (uint32_t)(ctl->cb) & 0x7fffffff;

	// Initialise PWM - these delays may not all be necessary, but at least
	// I seem to be able to switch between PWM audio and servoblaster
	// reliably with this code.
	pwm_reg[PWM_CTL] = 0;
	udelay(10);
	pwm_reg[PWM_STA] = pwm_reg[PWM_STA];
	udelay(10);
	clk_reg[PWMCLK_CNTL] = 0x5A000000;
	clk_reg[PWMCLK_DIV] = 0x5A000000;
	clk_reg[PWMCLK_CNTL] = 0x5A000001;              // Source=osc
	clk_reg[PWMCLK_DIV] = 0x5A000000 | (32<<12);    // set pwm div to 32 (19.2MHz/32 = 600KHz)
	udelay(500);					// Delay needed before enabling
	clk_reg[PWMCLK_CNTL] = 0x5A000011;              // Source=osc and enable

	udelay(500);

	pwm_reg[PWM_RNG1] = tick_scale;				// 600KHz/6 = 10us per FIFO write
	udelay(10);
	ctl->pwmdata = 1;					// Give a pulse of one clock width for each fifo write
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
	dma_reg[DMA_CONBLK_AD] = (uint32_t)(ctl->cb) & 0x7fffffff;
	dma_reg[DMA_DEBUG] = 7; // clear debug error flags
	udelay(10);
	dma_reg[DMA_CS] = 0x10880001;	// go, mid priority, wait for outstanding writes

	return 0;
}


void cleanup_module(void)
{
	int servo;

	// Take care to stop servos with outputs low, so we don't get
	// spurious movement on module unload
	for (servo = 0; servo < NUM_SERVOS; servo++) {
		del_timer(idle_timer + servo);
		// Wait until we're not driving this servo
		if (wait_for_servo(servo))
			break;
		// Patch the control block so it stays low
		ctl->cb[servo*4+0].dst = ((GPIO_BASE + GPCLR0*4) & 0x00ffffff) | 0x7e000000;
		local_irq_enable();
	}
	// Wait 20ms to be sure it has finished it's cycle an all outputs are low
	msleep(20);
	// Now we can kill everything
	dma_reg[DMA_CS] = BCM2708_DMA_RESET;
	pwm_reg[PWM_CTL] = 0;
	udelay(10);
	free_pages(ctldatabase, 0);
	iounmap(gpio_reg);
	iounmap(dma_reg);
	iounmap(clk_reg);
	iounmap(pwm_reg);
	cdev_del(&my_cdev);
	unregister_chrdev_region(devno, 1);
}

// kmalloc the temporary data required for each user:
static int dev_open(struct inode *inod, struct file *fil)
{
	fil->private_data = kmalloc(sizeof(struct private_data), GFP_KERNEL);
	if (0 == fil->private_data)
	{
		printk(KERN_WARNING "ServoBlaster: Failed to allocate user data\n");
		return -ENOMEM;
	}
	memset(fil->private_data, 0, sizeof(struct private_data));

	return 0;
}

static ssize_t dev_read(struct file *filp, char *buf, size_t count, loff_t *f_pos)
{
	ssize_t ret = 0;
	struct private_data* const pdata = filp->private_data;

	// Only proceed if we have private data, else return EOF.
	if (pdata) {
		if (0 == *f_pos) {
			int servo;
			char *p = pdata->rd_data, *end = p + sizeof(pdata->rd_data);

			// Get fresh data
			for (servo = 0; servo < NUM_SERVOS; ++servo) {
				p += snprintf(p, end - p, "%i=%i\n", servo,
					servo_pos[servo]);
			}
			pdata->rd_len = p - pdata->rd_data;
		}

		if (*f_pos < pdata->rd_len) {
			if (count > pdata->rd_len - *f_pos)
				count = pdata->rd_len - *f_pos;
			if (copy_to_user(buf, pdata->rd_data + *f_pos, count))
				return -EFAULT;
			*f_pos += count;
			ret = count;
		}
	}

	return ret;
}

static int set_servo(int servo, int cnt) 
{
	if (servo < 0 || servo >= NUM_SERVOS) {
		printk(KERN_WARNING "ServoBlaster: Bad servo number %d\n", servo);
		return -EINVAL;
	}
	if (cnt < 0 || cnt > cycle_ticks / 8 - 1) {
		printk(KERN_WARNING "ServoBlaster: Bad value %d\n", cnt);
		return -EINVAL;
	}
	if (wait_for_servo(servo))
		return -EINTR;

	if (idle_timeout)
		mod_timer(idle_timer + servo, jiffies + msecs_to_jiffies(idle_timeout));

	// Normally, the first GPIO transfer sets the output, while the second
	// clears it after a delay.  For the special case of a delay of 0, we
	// ensure that the first GPIO transfer also clears the output.
	if (cnt == 0) {
		ctl->cb[servo*4+0].dst = ((GPIO_BASE + GPCLR0*4) & 0x00ffffff) | 0x7e000000;
	} else {
		ctl->cb[servo*4+0].dst = ((GPIO_BASE + GPSET0*4) & 0x00ffffff) | 0x7e000000;
		ctl->cb[servo*4+1].length = cnt * sizeof(uint32_t);
		ctl->cb[servo*4+3].length = (cycle_ticks / 8 - cnt) * sizeof(uint32_t);
	}
	servo_pos[servo] = cnt;	// Record data for use by dev_read
	local_irq_enable();

	return 0;
}

static ssize_t dev_write(struct file *filp,const char *user_buf,size_t count,loff_t *f_pos)
{
	struct private_data* const pdata = filp->private_data;
	char buf[128], *p = buf, nl;
	int len = pdata->partial_len;

	if (0 == pdata)
		return -EFAULT;
	if (pdata->reject_writes)
		return -EINVAL;
	memcpy(buf, pdata->partial_cmd, len);
	pdata->partial_len = 0;
	if (count > sizeof(buf) - len - 1)
		count = sizeof(buf) - len - 1;
	if (copy_from_user(buf+len, user_buf, count))
		return -EFAULT;
	len += count;
	buf[len] = '\0';
	while (p < buf+len) {
		int servo, cnt, res;

		if (strchr(p, '\n')) {
			if (sscanf(p, "%d=%d%c", &servo, &cnt, &nl) != 3 ||
					nl != '\n') {
				printk(KERN_WARNING "ServoBlaster: Bad data format\n");
				pdata->reject_writes = 1;
				return -EINVAL;
			}
			res = set_servo(servo, cnt);
			if (res < 0) {
				pdata->reject_writes = 1;
				return res;
			}
			p = strchr(p, '\n') + 1;
		}
		else if (buf+len - p > 10) {
			printk(KERN_WARNING "ServoBlaster: Bad data format\n");
			pdata->reject_writes = 1;
			return -EINVAL;
		}
		else {
			// assume more data is coming...
			break;
		}
	}
	pdata->partial_len = buf+len - p;
	memcpy(pdata->partial_cmd, p, pdata->partial_len);

	return count;
}

static int dev_close(struct inode *inod,struct file *fil)
{
	struct private_data* const pdata = fil->private_data;
	int ret = 0;

	if (pdata) {
		if (pdata->partial_len) {
			printk(KERN_WARNING "ServoBlaster: partial command "
				"pending on close()\n");
			ret = -EIO;
		}
		// Free process data.
		kfree(pdata);
	}

	return ret;
}

static long dev_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
	return -EINVAL;
}

MODULE_DESCRIPTION("ServoBlaster, Multiple Servo Driver for the RaspberryPi");
MODULE_AUTHOR("Richard Hirst <richardghirst@gmail.com>");
MODULE_LICENSE("GPL v2");

module_param(cycle_ticks, int, 0);
MODULE_PARM_DESC(cycle_ticks, "number of ticks per cycle, max pulse is cycle_ticks/8");

module_param(tick_scale, int, 0);
MODULE_PARM_DESC(tick_scale, "scale the tick length, 6 should be 10us");

module_param(idle_timeout, int, 0);
MODULE_PARM_DESC(idle_timeout, "Idle timeout, after which we turn off a servo output (ms)");

