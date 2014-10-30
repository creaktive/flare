CC=gcc
# CC=clang
RM=rm -f
CFLAGS=-Wall -Ofast
# CFLAGS=-Wall -g -pg
# DEFS=
# LDFLAGS=-static

# Uncomment under Win32 (CYGWIN/MinGW):
# EXE=.exe

NRF905_DEMOD=nrf905_demod$(EXE)
FLARM_DECODE=flarm_decode$(EXE)

all: $(NRF905_DEMOD)

$(NRF905_DEMOD): nrf905_demod.o lib_crc.o
	$(CC) ${LDFLAGS} -o $(NRF905_DEMOD) nrf905_demod.o lib_crc.o
	strip $(NRF905_DEMOD)

$(FLARM_DECODE): flarm_decode.o lib_crc.o
	$(CC) ${LDFLAGS} -o $(FLARM_DECODE) flarm_decode.o lib_crc.o
	strip $(FLARM_DECODE)

lib_crc.o: lib_crc.h

flarm_decode.o: flarm_codec.h

.c.o:
	$(CC) ${CFLAGS} ${DEFS} -c $*.c

clean:
	$(RM) $(NRF905_DEMOD) $(FLARM_DECODE) *.o core
