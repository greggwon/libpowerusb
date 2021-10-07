#ifndef PWRUSBIMP_H__
#define PWRUSBIMP_H__

#include <stdio.h>
#include <string.h>
#include <string>
#include <stdlib.h>
#include <ctype.h>
#include <locale.h>
#include <errno.h>

#include <unistd.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <sys/ioctl.h>
#include <sys/utsname.h>
#include <fcntl.h>
#include <pthread.h>

#include <PwrUSBHid.h>
#include <PwrUSBImp.h>

const int POWER_USB_MAXNUM = 4;

const char *MY_DEVICE_ID ="Vid_04d8&Pid_003F"; // This is the Vendor ID and Product ID for PowerUSB microcontroller

const int BUF_LEN=256;
const int BUF_WRT=65;

// Commands to write to PowerUSB
// :.,'as/\([^ 	]*\)[ 	]\([^ 	]*\)[ 	][ 	]*\([^ 	]*\)/const char \2 = \3;/
const char ON_PORT1 = 'A';
const char OFF_PORT1 = 'B';
const char ON_PORT2 = 'C';
const char OFF_PORT2 = 'D';
const char ON_PORT3 = 'E';
const char OFF_PORT3 = 'P';

const char DEFON_PORT1 = 'N';
const char DEFOFF_PORT1 = 'F';
const char DEFON_PORT2 = 'G';
const char DEFOFF_PORT2 = 'Q';
const char DEFON_PORT3 = 'O';
const char DEFOFF_PORT3 = 'H';

const int READ_P1 = 0xa1;
const int READ_P2 = 0xa2;
const int READ_P3 = 0xac;

const int READ_P1_PWRUP = 0xa3;
const int READ_P2_PWRUP = 0xa4;
const int READ_P3_PWRUP = 0xad;
	
const int READ_FIRMWARE_VER = 0xa7;
const int READ_MODEL = 0xaa;

const int READ_CURRENT = 0xb1;
const int READ_CURRENT_CUM = 0xb2;
const int RESET_CURRENT_COUNT = 0xb3;
const int WRITE_OVERLOAD = 0xb4;
const int READ_OVERLOAD = 0xb5;
const int SET_CURRENT_RATIO = 0xb6;
const int RESET_BOARD = 0xc1;
const int SET_CURRENT_OFFSET = 0xc2;

const int ALL_PORT_ON = 0xa5;
const int ALL_PORT_OFF = 0xa6;
const int SET_MODE = 0xa8;
const int READ_MODE = 0xa9;

// Digital IO
const int SET_IO_DIRECTION=0xd1;
const int SET_IO_OUTPUT=0xd3;
const int GET_IO_INPUT=0xd4;
const int SET_IO_CLOCK=0xd5;
const int GET_IO_OUTPUT=0xd6;
const int SET_IO_TRIGGER=0xd7;

#define TRUE  1
#define FALSE 0

const unsigned short PWRUSB_VENDOR_ID = 0x4d8;
const unsigned short PWRUSB_PRODUCT_ID = 0x03f;

const int PWRUSB_NO_CHANGE=-1;

class PowerUSB {

private:
	// Local Functions
	int checkConnected();
	int writeData(int len);
	int readData();

	// GLOBAL Variables

	// The pointers to all devices we could open and access, but not linearly
	// stored.  Inaccessible devices will have a NULL pointer in this array.
	hid_device *AttachedDeviceHandles[POWER_USB_MAXNUM];

	// The number of devices found, but not necessarily accessible
	int AttachedDeviceCount;
	
	// The attached, accessible device index, currently being used.
	int CurrentDevice;
	int AttachedState;
	unsigned char OUTBuffer[BUF_WRT];	// Allocate a memory buffer equal to the OUT endpoint size + 1
	unsigned char INBuffer[BUF_WRT];	// Allocate a memory buffer equal to the IN endpoint size + 1
	void *sharedLibraryHandle;	// Pointer to Shared Library
	bool onOffCheck( int port, int status );
	int setCurrentDevice(int count);

public:
	static bool debugging;
	PowerUSB();
	virtual ~PowerUSB();
	int init(int *model);
	int close();
	int getCurrentDevice();

	// Checks to see if PowerUSB device is connected to computer
	/////////////////////////////////////////////////////////////////
	int checkStatus();

	// Sets the state of the outlet 1 and 2
	// A=Outlet1 On. B=Outlet1 Off
	// C=Outlet2 On  D=Outlet2 Off
	////////////////////////////////////////
	int setPort(int port1, int port2, int port3);
	int setDefaultState(int state1, int state2, int state3);
	int readPortState(int *port1, int *port2, int *port3);
	int readDefaultPortState(int *port1, int *port2, int *port3);

	// Reads the current version of firmware
	// As of Nov 1st 2010. This function is not implemented in firmware
	///////////////////////////////////////////////////////////////////////
	int getFirmwareVersion();

	// Returns
	//0: Not connected or unknown
	//1: Basic model. Computer companion
	//2: Digital IO model. 
	//3: Computer Watchdog Model
	//4: Smart Pro Model
	int getModel();
	std::string getModelName();

	// Current Sensing Related Functions
	////////////////////////////////////////////////////
	int readCurrentDevice(int *current);
	int readCurrentCum(int *currentCum);
	int resetCurrentCounter();
	int setCurrentSensRatio(int currentRatio);
	int setOverload(int overload);
	int getOverload();
	int resetBoard();
	int setCurrentOffset();

	///////////////////////////////////////
	// Digital IO related functions
	/////////////////////////////////////
	int setIODirection(int direction[]);
	int setOutputState(int output[]);
	int getInputState(int input[]);
	int generateClock(int port, int onTime, int offTime);
	int getOutputState(int output[]);
	int setInputTrigger(int port, int outlet1, int outlet2, int outlet3, int out1, int out2);
	// Watchdog related functions
	/////////////////////////////////////

	#define	START_WDT			0x90
	#define	STOP_WDT			0x91		
	#define	POWER_CYCLE			0x92
	#define	READ_WDT 			0x93	//-> return the all status.
	#define	HEART_BEAT			0x94

	// void (*makePacket)( unsigned char buf[BUF_WRT], int &n )
	template<typename T>
	int performUSBTask( T fp );

	template<typename T, typename S>
	int performUSBTaskWithResult( T fp, S resp );

	// Starts watchdog in the PowerUSB. 
	// HbTimeSec: expected time for heartbeat
	// numHbMisses: Number of accepted consicutive misses in heartbeat
	// resetTimeSec: Amount of time to switch off the computer outlet
	/////////////////////////////////////////////////////////////////////////

	int startWatchdogTimer(int HbTimeSec, int numHbMisses, int resetTimeSec);

	// Stops the watchdog timer in PowerUSB
	/////////////////////////////////////////////
	int stopWatchdogTimer();

	// Get the current state of Watchdog in PowerUSB
	// Return 0: watchdog is not running, 1: Watchdog is active, 2: In PowerCycle phase
	///////////////////////////////////////////////////////////////////////////////////////
	int getWatchdogStatus();

	// Sends the Heartbeat to PowerUSB
	///////////////////////////////////
	int sendHeartBeat();

	// Switch off the computer outlet and switch it back on after resetTimeSec
	///////////////////////////////////////////////////////////////////////////////
	int powerCycle(int resetTimeSec);

	bool haveDeviceAtIndex( int useDev );
	bool selectDevice( int useDev );
};
#endif
