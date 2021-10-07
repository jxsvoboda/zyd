#
# ZyDAS driver development makefile
#

# Your onnv-gate dir
GATE= /usr/src/onnv-gate

# Compiling for 32-bit or 64-bit kernel
#ISA= amd64
ISA=i386

# Should work without changes

ifeq ($(ISA),amd64)
MOPTS= -m64 -mcmodel=kernel
DRVDIR= /kernel/drv/amd64
else
MOPTS= -m32
DRVDIR= /kernel/drv
endif

CC= gcc
LD= ld
WARNINGS = -Wall -Werror -Wno-unknown-pragmas
CFLAGS= -D_KERNEL -ffreestanding -fno-builtin $(WARNINGS) $(MOPTS)
LDFLAGS= -r -dy -Nmisc/mac -Nmisc/net80211 -Nmisc/usba
INCLUDE= -I$(GATE)/usr/src/uts/common
OBJS= zyd.o zyd_fw.o zyd_hw.o zyd_usb.o zyd_util.o
CONFDIR= /kernel/drv

.PHONY: all clean install uninstall

all: zyd
  
zyd: $(OBJS)
	$(LD) $(LDFLAGS) -o $@ $(OBJS)

%.o: %.c *.h
	$(CC) $(CFLAGS) $(INCLUDE) -c $*.c

clean:
	rm -f *.o
	rm -f zyd

# Install module to the driver directory
install:
	chmod 755 zyd
	cp zyd $(DRVDIR)/zyd

uninstall:
	rm -f $(DRVDIR)/zyd

# Add/remove driver from the system
add:
	add_drv -n -i '"usbace,1211"' zyd

rem:
	rem_drv zyd
