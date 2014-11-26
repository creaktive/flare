CC=gcc
# CC=clang
RM=rm -f
CFLAGS=-Wall -Ofast -I/opt/local/include
# CFLAGS=-Wall -g -I/opt/local/include
# DEFS=
LDFLAGS=-L/opt/local/lib

# Uncomment under Win32 (CYGWIN/MinGW):
# EXE=.exe

NRF905_DEMOD=nrf905_demod$(EXE)
FLARM_DECODE=flarm_decode$(EXE)

all: $(NRF905_DEMOD) $(FLARM_DECODE)
	# strip $(NRF905_DEMOD) $(FLARM_DECODE)

$(NRF905_DEMOD): nrf905_demod.o lib_crc.o
	$(CC) ${LDFLAGS} -lfftw3 -lm -o $(NRF905_DEMOD) nrf905_demod.o lib_crc.o

$(FLARM_DECODE): flarm_decode.o lib_crc.o
	$(CC) ${LDFLAGS} -lm -o $(FLARM_DECODE) flarm_decode.o lib_crc.o

lib_crc.o: lib_crc.h

flarm_decode.o: flarm_codec.h

.c.o:
	$(CC) ${CFLAGS} ${DEFS} -c $*.c

clean:
	$(RM) $(NRF905_DEMOD) $(FLARM_DECODE) *.o core
