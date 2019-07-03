TARGET=../tests/pwmTest ../tests/motorPwmTest ../tests/motorVoltTest ../tests/decoderTest ../tests/sensorTest ../tests/pidTest ../tests/stop
SRCS=pwm.c quanser.c sensors.c

FLAGS=-O2 -Wall -g -MMD
INCLUDE=-I. -I../include
LIBDIR=-L../lib
LIBS=-lgalileo2

CC=$(CROSS_COMPILE)gcc
CFLAGS=$(FLAGS) $(INCLUDE)
LDFLAGS=$(LIBDIR) $(LIBS)

all: $(TARGET)

$(TARGET): %: %.c $(SRCS:.c=.o)
	$(CC) $(CFLAGS) -o $@ $^ $(LDFLAGS)

%.o: %.c
	$(CC) $(CFLAGS) -c -o $@ $<

-include $(SRCS:.c=.d)

clean:
	rm -f $(TARGET) *~ *.bak *.o *.d ../tests/*.d
