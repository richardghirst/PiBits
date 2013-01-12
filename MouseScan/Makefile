all:	MouseDump MouseScan

CFLAGS = -Wall -g -O2 `pkg-config gtkmm-3.0 --cflags --libs`

MouseScan:	MouseScan.o
	$(CC) -o $@ $^ -lm `pkg-config gtkmm-3.0 --cflags --libs`
