#include <stdint.h>
#include <stdlib.h>
#include <unistd.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <sys/mman.h>
#include <gtk/gtk.h>

/* Surface to store current scribbles */
static cairo_surface_t *surface = NULL;

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

static float xscale = 0.60;
static float yscale = 0.60;

int winsize = 1024;

static void
udelay(int us)
{
	struct timespec ts = { 0, us * 1000 };

	if (us <= 100) {
		volatile uint32_t x = 0;
		int i;
		for (i = 0; i < 30*us; i++)
			x += gpio_reg[0];
		return;
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

static void
init_sensor(void)
{
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
	write_sensor(0x0a,0x11);
	udelay(1000);

	printf("Product ID: %02x Revision: %02x\n", read_sensor(0), read_sensor(1));
}

int
get_image(int *xoff, int *yoff, uint8_t *img)
{
	int i;
	uint8_t v;
	int m, x, y;

	m = read_sensor(2);
	x = (signed char)read_sensor(3);
	y = (signed char)read_sensor(4);
	printf("m=0x%02x, x=%d, y=%d\n", m, x, y);
	*xoff = x;
	*yoff = y;

	write_sensor(0x0a, 0x19);
	for (i = 0; i < 256; i++) {
//		if (read_sensor(0x0d) != i)
//			printf("Error, expecting register %d\n", i);
		v = read_sensor(0x0c);
		img[i] = v;
		if (v & 0xc0) {
//			printf("Bad value %02x\n", v);
			i--;
		}
	}
	write_sensor(0x0a, 0x11);

#if 0
	for (i = 255; i >= 0; i--) {
		printf("%02x ", img[i]);
		if ((i & 0x0f) == 0)
			printf("\n");
	}
	printf("\n");
#endif

	return 0;
}

#define SZ	1

/* Draw a rectangle on the surface at the given position */
static void
draw_image (GtkWidget *widget)
{
  cairo_t *cr;
  uint8_t img[256];
  int r, c, xoff, yoff;
  static float x, y;

  get_image(&xoff, &yoff, img);

  x += (float)xoff * xscale;
  y -= (float)yoff * yscale;
  if (x < 0)
    x = 0;
  if (x > winsize - 17)
    x = winsize - 17;
  if (y < 0)
    y = 0;
  if (y > winsize - 17)
    y = winsize - 17;

//x=0; y=0;
  /* Paint to the surface, where we store our state */
  cr = cairo_create (surface);

  for (r = 0; r < 16; r++) {
    for (c = 0; c < 16; c++) {
      cairo_set_source_rgb (cr, (double)(img[(240-r*16) + 15 - c])/(double)64, 0, 0);
      cairo_rectangle (cr, (r+x)*SZ, (c+y)*SZ, SZ, SZ);
      cairo_fill (cr);
    }
  }

  cairo_destroy (cr);

  /* Now invalidate the affected region of the drawing area. */
  gtk_widget_queue_draw_area (widget, x*SZ, y*SZ, 16*SZ, 16*SZ);
}

static void
clear_surface (void)
{
  cairo_t *cr;

  cr = cairo_create (surface);

  cairo_set_source_rgb (cr, 1, 1, 1);
  cairo_paint (cr);

  cairo_destroy (cr);
}

/* Create a new surface of the appropriate size to store our scribbles */
static gboolean
configure_event_cb (GtkWidget         *widget,
            GdkEventConfigure *event,
            gpointer           data)
{
  if (surface)
    cairo_surface_destroy (surface);

  surface = gdk_window_create_similar_surface (gtk_widget_get_window (widget),
                                       CAIRO_CONTENT_COLOR,
                                       gtk_widget_get_allocated_width (widget),
                                       gtk_widget_get_allocated_height (widget));

  /* Initialize the surface to white */
  clear_surface ();

  draw_image(widget);

  /* We've handled the configure event, no need for further processing. */
  return TRUE;
}

/* Redraw the screen from the surface. Note that the ::draw
 * signal receives a ready-to-be-used cairo_t that is already
 * clipped to only draw the exposed areas of the widget
 */
static gboolean
draw_cb (GtkWidget *widget,
 cairo_t   *cr,
 gpointer   data)
{
  cairo_set_source_surface (cr, surface, 0, 0);
  cairo_rectangle (cr, 3, 3, 6, 6);
  cairo_fill (cr);
  cairo_paint (cr);

  return FALSE;
}

static void
close_window (void)
{
  if (surface)
    cairo_surface_destroy (surface);

  gtk_main_quit ();
}

static gboolean
time_handler(GtkWidget *widget)
{
	static volatile int processing;
	static volatile int ticker;

	if (processing) {
		;
	} else if (ticker > 0) {
		ticker--;
	} else {
		processing = 1;
		ticker = 1;
		draw_image(widget);
		processing = 0;
	}

	return 1;
}
int
main (int   argc,
      char *argv[])
{
  GtkWidget *window;
  GtkWidget *frame;
  GtkWidget *da;

  if (argc > 1) {
    sscanf(argv[1], "%f", &xscale);
    yscale = xscale;
  }
  if (argc > 2) {
    sscanf(argv[2], "%f", &yscale);
  }
  printf("Using scales %f, %f\n", xscale, yscale);

  init_sensor();

  gtk_init (&argc, &argv);

  window = gtk_window_new (GTK_WINDOW_TOPLEVEL);
  gtk_window_set_title (GTK_WINDOW (window), "Drawing Area");

  g_signal_connect (window, "destroy", G_CALLBACK (close_window), NULL);

  gtk_container_set_border_width (GTK_CONTAINER (window), 8);

  frame = gtk_frame_new (NULL);
  gtk_frame_set_shadow_type (GTK_FRAME (frame), GTK_SHADOW_IN);
  gtk_container_add (GTK_CONTAINER (window), frame);

  da = gtk_drawing_area_new ();
  /* set a minimum size */
  gtk_widget_set_size_request (da, winsize, winsize);

  gtk_container_add (GTK_CONTAINER (frame), da);

  /* Signals used to handle the backing surface */
  g_signal_connect (da, "draw",
            G_CALLBACK (draw_cb), NULL);
  g_signal_connect (da,"configure-event",
            G_CALLBACK (configure_event_cb), NULL);

  g_timeout_add(2, (GSourceFunc) time_handler, (gpointer) da);
  gtk_widget_show_all (window);

  gtk_main ();

  return 0;
}

