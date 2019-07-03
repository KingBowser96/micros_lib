TARGET = ./src/prova
SRCS = $(TARGET).c ./src/mylib.c

FLAGS=-O2 -Wall -g -MMD
INCLUDE=-I /include
LIBDIR=
LIBS=

CC=$(CROSS_COMPILE)gcc
CFLAGS=$(FLAGS) $(INCLUDE)
LDFLAGS=$(LIBDIR) $(LIBS)

all: $(TARGET)

$(TARGET): $(SRCS:.c=.o)
	$(CC) -o $@ $^ $(LDFLAGS)

%.o: %.c
	$(CC) $(CFLAGS) -c -o $@ $<

-include $(SRCS:.c=.d)

clean: distclean
	rm -f $(TARGET)

distclean: 
	rm -f *~ *.bak ./src/*.o ./src/*.d
