# CC=gcc
CC=clang
RM=rm -f
CFLAGS=-Wall -Oz
# CFLAGS=-Wall -g -pg
# DEFS=
# LDFLAGS=-static

# Uncomment under Win32 (CYGWIN/MinGW):
# EXE=.exe

NRF905_DEMOD=nrf905_demod$(EXE)
COBJ=nrf905_demod.o lib_crc.o

all: $(NRF905_DEMOD)

$(NRF905_DEMOD): $(COBJ)
	$(CC) ${LDFLAGS} -o $(NRF905_DEMOD) $(COBJ)
	strip $(NRF905_DEMOD)

lib_crc.o: lib_crc.h

.c.o:
	$(CC) ${CFLAGS} ${DEFS} -c $*.c

clean:
	$(RM) $(NRF905_DEMOD) *.o core
