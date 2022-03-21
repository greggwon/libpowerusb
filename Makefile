OFILES=\
	PwrUSBImp.o \
	PwrUSBHid.o
HFILES=\
	PwrUSBHid.h \
	PwrUSBImp.h

HEADER=\
	PwrUSBImp.h \
	PwrUSBHid.h

POWER_RULES=99-powerusb.rules
HID=RULES=98-disable-hid.rules

INCLDIR=/usr/local/include/powerusb-1.0

LIB=libpowerusb.so

LIBDIR=/usr/local/lib

CFLAGS=-fPIC -I. -Wno-format-security -g
CXXFLAGS=$(CFLAGS) -I. -Wno-format-security -g
LDFLAGS=-g

all: $(LIB)

PwrUSBImp.o : PwrUSBImp.h

$(LIB): $(OFILES)
	g++ -shared -o $(LIB) $(OFILES) $(LDFLAGS)

clean:
	rm -f $(LIB) $(OFILES)

install: $(LIB)
	cp $(LIB) $(LIBDIR)/$(LIB)
	mkdir -p $(INCLDIR)
	chmod 775 $(INCLDIR)
	cp $(HEADER) $(INCLDIR)/
	chmod 664 $(INCLDIR)/*
	cp $(POWER_RULES) /etc/udev/rules.d/
	udevadm trigger
