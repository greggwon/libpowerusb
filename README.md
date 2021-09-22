PowerUSB Note:

	The source code has been compiled and tested on Ubuntu 20.04 with no issues. The
	usage of packages and installation of the packages hereby mentioned are with respect to
	Ubuntu. For other than Ubuntu based systems use the equivalent platform specific packages
	and install procedure.

PowerUSB usage procedure :
	
	Before making use of powerusb application make sure the following packages are installed :
		1. libudev
		2. libudev-dev
		3. libusb-1.0-0-dev

	To install the above packages, run the following commands in the terminal
		- sudo apt-get install libudev
		- sudo apt-get install libudev-dev
		- sudo apt-get install libusb-1.0-0-dev

PowerUSB API :

	Use the provided Makefile to build libpowerusb.so.  This can then be copied into
	an appropriate directory for use by applications.

	Use 'sudo make install' to install it under the /usr/local/ directory tree on
	your system.  The installation paths are in the Makefile and can be altered for
	installation into other locations.
