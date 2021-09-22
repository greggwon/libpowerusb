PowerUSB Notes :

	The source code has been compiled and tested on Ubuntu 20.04 with no issues. The
	usage of packages and installation of the packages hereby mentioned are with respect to
	Ubuntu. For other than Ubuntu based systems use the equivalent platform specific packages
	and install procedure.

PowerUSB Setup :
	
	Before making use of powerusb application make sure the following packages are installed :
		1. libudev
		2. libudev-dev
		3. libusb-1.0-0-dev

	To install the above packages, run the following commands in the terminal
		- sudo apt-get install libudev
		- sudo apt-get install libudev-dev
		- sudo apt-get install libusb-1.0-0-dev

PowerUSB Library :

	Use the provided Makefile to build the libpowerusb.so shared library.  This can then
	be copied into an appropriate directory for use by applications.

	Use 'sudo make install' to install things under the /usr/local/ directory tree on
	your system.  The installation paths are in the Makefile and can be altered for
	installation into other locations.

99-powerusb.rules:

        This is a udev file for HIDAPI devices which changes the permissions to 0666 (world
        readable/writable) for a specified device on Linux systems. Drop this file into
        /etc/udev/rules.d and unplug and re-plug your device. This is all that is necessary
        to see the new permissions. If it is decided not to drop this file in /etc/udev/rules.d
        then the application can be used with only super user privileges or log in as
        super user and assign read and write permissions for the device.
