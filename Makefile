OFILES=\
	PwrUSBImp.o \
	PwrUSBHid.o
HFILES=\
	PwrUSBHid.h \
	PwrUSBImp.h

HEADER=\
	PwrUSBImp.h \
	PwrUSBHid.h

INCLDIR=/usr/local/include/powerusb-1.0

LIB=libpowerusb.so

LIBDIR=/usr/local/lib

CFLAGS=-fPIC -I. -Wno-format-security
CXXFLAGS=$(CFLAGS) -I. -Wno-format-security

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
