# code by </MATRIX>@Neod Anderjon
# Makefile for fork of dump1090, to improve devices like hackrf airspy, etc
# last generate file: dump1090-sdrplus

# install directory
PREFIX	    ?=	/usr/local
# gcc compile option
CFLAGS	    ?=	-O1 -g -Wall -W $(shell pkg-config --cflags librtlsdr libhackrf libairspy soxr)
LDLIBS	    +=	$(shell pkg-config --libs librtlsdr libhackrf libairspy soxr) -lpthread -lm -lmirsdrapi-rsp
# compiler select
CC		    ?=	gcc
# install tool
INSTALL	    ?=	install
PROGNAME    =	dump1090-sdrplus # target bin file

# make all
.PHONY: all
all: dump1090-sdrplus

%.o: %.c
	$(CC) $(CFLAGS) -c $<

dump1090-sdrplus: dump1090.o netpro.o data.o devices.o # generate middle file: *.o files
	$(CC) -g -o dump1090-sdrplus dump1090.o netpro.o data.o devices.o $(LDFLAGS) $(LDLIBS)
	@echo "code by </MATRIX>@Neod Anderjon"

# install to system local directory
.PHONY: install
install: dump1090-sdrplus
	$(INSTALL) -d $(PREFIX)/bin
	$(INSTALL) -m 0755 dump1090-sdrplus $(PREFIX)/bin
	@echo "code by </MATRIX>@Neod Anderjon"

# delete obj files
clean:
	rm -r *.o
	@echo "code by </MATRIX>@Neod Anderjon"


