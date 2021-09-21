#include "PwrUSBImp.h"
#include "PwrUSBHid.h"
#include <string>
#include <cstdarg>

template <typename T, typename... Args>
static void debug( T fmt, Args... args ) {
	int len = std::snprintf( nullptr, 0, fmt, args... );
	char buf[len+100];
	std::snprintf( buf, sizeof(buf), fmt, args... );
	if( PowerUSB::debugging ) fprintf( stderr, "%s", buf );
}

bool PowerUSB::debugging;

PowerUSB::PowerUSB() {
	debugging= 1;
	CurrentDevice = -1;
	AttachedState = 0;
	AttachedDeviceCount = 0;
	memset( AttachedDeviceHandles, 0, sizeof(AttachedDeviceHandles) );
	memset( OUTBuffer, 0, sizeof( OUTBuffer ) );
	memset( INBuffer, 0, sizeof(INBuffer ) );
}

int PowerUSB::init(int *model)
{
	int i, r = -1;

	debug( "init: model pointer=%p, CurrentDevice=%d\n", model, CurrentDevice );

	// Already opened? just return
	if( CurrentDevice >= 0 ) {
		return CurrentDevice;
	}
	*model = 0;

	// Load the linux libhid.so which supports HID class functions
	debug( "Checking what's connected\n");
	if ((AttachedDeviceCount = checkConnected()) > 0)		
	{
		debug( "%d devices attached, check for 0x%04x:0x%04x\n",
			AttachedDeviceCount, VENDOR_ID, PRODUCT_ID);
		// For each of the connected devices, check to see whether it can read and write.
		for (i = 0; i < AttachedDeviceCount; i++)
		{
			if(AttachedDeviceHandles[i] != NULL)
			{
				// Let the rest of the PC application know the USB device is connected,
				// and it is safe to read/write to it
				AttachedState = TRUE;
				debug( "Found Device[%d] is attached\n", i );
				r = i;
			}
			else    // For some reason a device was physically plugged in, but one or
			        // more of the read/write handles didn't open successfully...
			{
				// Let the rest of this application known not to read/write to the device.
				AttachedState = FALSE;		
				debug( "Oops Device[%d] is NOT attached\n", i );
			}
		}
	}
	else	// Device must not be connected (or not programmed with correct firmware)
	{
		AttachedState = FALSE;
	}

	// Use last device found
	CurrentDevice = AttachedDeviceCount - 1;
	if (r >= 0)
	{
		debug( "Getting model...");
		*model = getModel();
		debug( "model=%p == %d\n", model, *model );
		return AttachedDeviceCount;
	}
	return r;
}

int PowerUSB::close()
{
	debug( "Closing PowerUSB, attached=%d\n", AttachedState );
	if (AttachedState)
	{
		AttachedState = FALSE;
		for (int i = 0; i < AttachedDeviceCount; i++)
		{
			if( AttachedDeviceHandles[i] != NULL ) {
				hid_close(AttachedDeviceHandles[i]);
				AttachedDeviceHandles[i] = NULL;
			}
		}
	}
	CurrentDevice = -1;
	return 0;
}

int PowerUSB::setCurrentDevice(int count)
{
	debug( "SetCurrentPowerUSB =%d, attached=%d\n", count, AttachedState );
	if (count >= AttachedDeviceCount)
		count = AttachedDeviceCount - 1;
	CurrentDevice = count;
	return count;
}

// Checks to see if PowerUSB device is connected to computer
/////////////////////////////////////////////////////////////////
int PowerUSB::checkStatus()
{
	int conn = 0;
	if (checkConnected() > 0)
		conn = 1;
	debug( "Status shows %sconnected\n", conn ? "" : "not " );
	return conn;
}

// Sets the state of the outlet 1 and 2
// A=Outlet1 On. B=Outlet1 Off
// C=Outlet2 On  D=Outlet2 Off
////////////////////////////////////////
int PowerUSB::setPort(int port1, int port2, int port3)
{
	int r;

	debug( "Setting Port States to %d, %d, %d, attached=%d\n", port1, port2, port3, AttachedState );

	if(AttachedState != TRUE)
		return -1;

	//The first byte is the "Report ID" and does not get sent over the USB bus.  Always set = 0.
	OUTBuffer[0] = 0;

	if (port1 >= 0)
	{
		if (port1)
			OUTBuffer[1]= ON_PORT1;
		else
			OUTBuffer[1] = OFF_PORT1;

		debug("Setting port1 to %s\n", OUTBuffer[1]== ON_PORT1 ? "ON" : "OFF" );
		r = writeData(2);
		usleep(20*1000);
	}

	if (port2 >= 0)
	{
		if (port2)
			OUTBuffer[1] = ON_PORT2;
		else
			OUTBuffer[1] = OFF_PORT2;
		debug("Setting port2 to %s\n", OUTBuffer[1]== ON_PORT2 ? "ON" : "OFF" );
		r = writeData(2);
		usleep(20*1000);	
	}

	if (port3 >= 0 && port3 <= 1)
	{
		if (port3)
			OUTBuffer[1] = ON_PORT3;
		else
			OUTBuffer[1] = OFF_PORT3;
		debug("Setting port3 to %s\n", OUTBuffer[1]== ON_PORT3 ? "ON" : "OFF" );
		r = writeData(2);	
	}

	readData();			// read and clear the input buffer
	return r;
}

int PowerUSB::setDefaultState(int state1, int state2, int state3)
{
	int r;

	debug( "SetDefaultStatePowerUSB =%d,%d,%d, attached=%d\n", state1, state2, state3, AttachedState );
	if(AttachedState != TRUE)
		return -1;

	//The first byte is the "Report ID" and does not get sent over the USB bus.  Always set = 0.
	OUTBuffer[0] = 0;
	if (state1 >= 0)
	{
		if (state1)
			OUTBuffer[1]= DEFON_PORT1;
		else
			OUTBuffer[1] = DEFOFF_PORT1;
		r = writeData(2);
		usleep(20*1000);
	}

	if (state2 >= 0)
	{
		if (state2)
			OUTBuffer[1] = DEFON_PORT2;
		else
			OUTBuffer[1] = DEFOFF_PORT2;
		r = writeData(2);
		usleep(20*1000);	
	}

	if (state3 >= 0 && state3 <= 1)
	{
		if (state3)
			OUTBuffer[1] = DEFON_PORT3;
		else
			OUTBuffer[1] = DEFOFF_PORT3;
		r = writeData(2);	
	}
	// read and clear the input buffer
	readData();
	return r;
}

int PowerUSB::readPortState(int *port1, int *port2, int *port3)
{
	int r;

	debug( "ReadPortStatePowerUSB port1=%p, port2=%p, port3=%p, attached=%d\n",
		port1, port2, port3, AttachedState );
	if(AttachedState != TRUE)
		return -1;
	
	if (*port1 > 0)
	{
		//The first byte is the "Report ID" and does not get sent over the USB bus.  Always set = 0.
		OUTBuffer[0] = 0;
		OUTBuffer[1]= READ_P1;
		writeData(2);
		usleep(20*1000);
		r = readData();
		*port1 = (INBuffer[0]==0 ? 0:1);
		usleep(30*1000);
	}
	if (*port2 > 0)
	{
		//The first byte is the "Report ID" and does not get sent over the USB bus.  Always set = 0.
		OUTBuffer[0] = 0;
		OUTBuffer[1]= READ_P2;
		writeData(2);
		usleep(20*1000);
		r = readData();
		*port2 = (INBuffer[0]==0 ? 0:1);
		usleep(30*1000);
	}
	if (*port3 > 0)
	{
		//The first byte is the "Report ID" and does not get sent over the USB bus.  Always set = 0.
		OUTBuffer[0] = 0;
		OUTBuffer[1]= READ_P3;
		writeData(2);
		usleep(20*1000);
		r = readData();
		*port3 = (INBuffer[0]==0 ? 0:1);
		usleep(30*1000);
	}
	return r;
}

int PowerUSB::readDefaultPortState(int *port1, int *port2, int *port3)
{
	int r;

	debug( "readDefaultPortState: port1=%p, port2=%p, port3=%p, attached=%d\n", port1, port2, port3, AttachedState );
	if(AttachedState != TRUE)
		return -1;

	if (*port1 > 0)
	{
		//The first byte is the "Report ID" and does not get sent over the USB bus.  Always set = 0.
		OUTBuffer[0] = 0;
		OUTBuffer[1]= READ_P1_PWRUP;
		writeData(2);
		usleep(20*1000);
		r=readData();
		*port1 = (INBuffer[0]==0 ? 0:1);
		usleep(30*1000);
	}
	if (*port2 > 0)
	{
		//The first byte is the "Report ID" and does not get sent over the USB bus.  Always set = 0.
		OUTBuffer[0] = 0;
		OUTBuffer[1]= READ_P2_PWRUP;
		writeData(2);
		usleep(20*1000);
		r=readData();
		*port2 = (INBuffer[0]==0 ? 0:1);
		usleep(30*1000);
	}
	if (*port3 > 0)
	{
		//The first byte is the "Report ID" and does not get sent over the USB bus.  Always set = 0.
		OUTBuffer[0] = 0;
		OUTBuffer[1]= READ_P3_PWRUP;
		writeData(2);
		usleep(20*1000);
		r=readData();
		*port3 = (INBuffer[0]==0 ? 0:1);
		usleep(30*1000);
	}
	return r;
}

// Reads the current version of firmware
// As of Nov 1st 2010. This function is not implemented in firmware
///////////////////////////////////////////////////////////////////////
int PowerUSB::getFirmwareVersion()
{
	int r;
	debug( "GetFirmwareVersionPortUSB: attached=%d\n", AttachedState );
	if(AttachedState != TRUE)
		return -1;
	//The first byte is the "Report ID" and does not get sent over the USB bus.  Always set = 0.
	OUTBuffer[0] = 0;
	OUTBuffer[1]= READ_FIRMWARE_VER;
	r = writeData(2);
	readData();
	r = INBuffer[0];
	usleep(20*1000);
	debug( "GetFirmwareVersionPortUSB: vers=%d\n", r );
	return r;
}

std::string PowerUSB::getModelName() {
	const char *name;
	switch( getModel() ) {
		case 0: name = "Not connected"; break;
		case 1: name = "Basic"; break;
		case 2: name = "Digital I/O"; break;
		case 3: name = "Computer Watchdog"; break;
		case 4: name = "Smart Pro"; break;
		default: name = "unknown"; break;
	}
	return std::string(name);
}

// Returns
//0: Not connected or unknown
//1: Basic model. Computer companion
//2: Digital IO model. 
//3: Computer Watchdog Model
//4: Smart Pro Model
int PowerUSB::getModel()
{
	int r;
	debug( "getModel: attached=%d\n", AttachedState );
	if(AttachedState != TRUE)
		return -1;
	//The first byte is the "Report ID" and does not get sent over the USB bus.  Always set = 0.
	OUTBuffer[0] = 0;
	OUTBuffer[1]= READ_MODEL;
	r = writeData(2);
	usleep(50*1000);
	readData();
	r = INBuffer[0];
	usleep(20*1000);
	debug( "getModel: model=%d\n", r );
	return r;
}

// Current Sensing Related Functions
////////////////////////////////////////////////////
int PowerUSB::readCurrentDevice(int *current)
{
	int r;
	debug( "readCurrentDevice: attached=%d\n", AttachedState );
	if(AttachedState != TRUE)
		return -1;
	//The first byte is the "Report ID" and does not get sent over the USB bus.  Always set = 0.
	OUTBuffer[0] = 0;
	OUTBuffer[1]= READ_CURRENT;
	r = writeData(2);
	usleep(20*1000);
	r = readData();
	if (r >= 0)
		*current = INBuffer[0]<<8 | INBuffer[1];
	else
		*current = 0;
	debug( "readCurrent: current=%p == %d\n", current, *current );
	return r;
}

int PowerUSB::readCurrentCum(int *currentCum)
{
	int r;
	debug( "readCurrentCum: attached=%d\n", AttachedState );
	if(AttachedState != TRUE)
		return -1;
	//The first byte is the "Report ID" and does not get sent over the USB bus.  Always set = 0.
	OUTBuffer[0] = 0;
	OUTBuffer[1]= READ_CURRENT_CUM;
	r = writeData(2);
	usleep(20*1000);
	r = readData();
	if (r >= 0)
		*currentCum = INBuffer[0]<<24 | INBuffer[1]<<16 | INBuffer[2]<<8 | INBuffer[3];
	else
		*currentCum = 0;
	debug( "readCurrentCum: currentCum=%p == %d\n", currentCum, *currentCum );
	return r;
}

int PowerUSB::resetCurrentCounter()
{
	int r;
	debug( "resetCurrentCounter: attached=%d\n", AttachedState );
	if(AttachedState != TRUE)
		return -1;
	//The first byte is the "Report ID" and does not get sent over the USB bus.  Always set = 0.
	OUTBuffer[0] = 0;
	OUTBuffer[1]= RESET_CURRENT_COUNT;
	r = writeData(2);
	usleep(20*1000);
	readData();			// read and clear the input buffer
	debug( "readCurrentCounter: write=%d\n", r );
	return r;
}

int PowerUSB::setCurrentSensRatio(int currentRatio)
{
	int r;
	debug( "resetCurrentSensRatio: attached=%d, currentRation=%d\n", AttachedState, currentRatio );
	if(AttachedState != TRUE)
		return -1;
	//The first byte is the "Report ID" and does not get sent over the USB bus.  Always set = 0.
	OUTBuffer[0] = 0;
	OUTBuffer[1]= SET_CURRENT_RATIO;
	OUTBuffer[2]= currentRatio;
	r = writeData(3);
	r = readData();
	int ratio=-1;
	if (r >= 0)
		ratio= INBuffer[0];
	debug( "resetCurrentSensRatio: ratio=%d\n", ratio );
	return ratio;
}

int PowerUSB::setOverload(int overload)
{
	int r;
	debug( "setOverload: attached=%d, overload=%d\n", AttachedState, overload );
	if(AttachedState != TRUE)
		return -1;
	//The first byte is the "Report ID" and does not get sent over the USB bus.  Always set = 0.
	OUTBuffer[0] = 0;
	OUTBuffer[1]= WRITE_OVERLOAD;
	OUTBuffer[2] = overload; 
	r = writeData(2);
	usleep(20*1000);
	readData();			// read and clear the input buffer
	debug( "setOverload: overload set result=%d\n", r );
	return r;
}

int PowerUSB::getOverload()
{
	int r;
	debug( "getOverload: attached=%d\n", AttachedState );
	if(AttachedState != TRUE)
		return -1;
	//The first byte is the "Report ID" and does not get sent over the USB bus.  Always set = 0.
	OUTBuffer[0] = 0;
	OUTBuffer[1]= READ_OVERLOAD;
	r = writeData(2);
	r = readData();
	int over = -1;
	if (r >= 0)
		over = INBuffer[0];
	debug( "getOverload: attached=%d found overload=%d\n", AttachedState, over );
	return r;
}

int PowerUSB::resetBoard()
{
	int r;
	debug( "resetBoard: attached=%d\n", AttachedState );
	if(AttachedState != TRUE)
		return -1;
	//The first byte is the "Report ID" and does not get sent over the USB bus.  Always set = 0.
	OUTBuffer[0] = 0;
	OUTBuffer[1]= RESET_BOARD;
	r = writeData(2);
	debug( "resetBoard: r=%d\n", r );
	usleep(20*1000);
	readData();			// read and clear the input buffer
	return r;
}

int PowerUSB::setCurrentOffset()
{
	int r;
	debug( "setCurrentOffset, connected=%d", AttachedState );
	if( AttachedState != TRUE )
		return -1;

	//The first byte is the "Report ID" and does not get sent over the USB bus.  Always set = 0.
	OUTBuffer[0] = 0;
	OUTBuffer[1]= SET_CURRENT_OFFSET;
	r = writeData(2);
	usleep(20*1000);
	readData();			// read and clear the input buffer
	return r;
}


///////////////////////////////////////
// Digital IO related functions
/////////////////////////////////////
int PowerUSB::setIODirection(int direction[])
{
	int r, i;
	unsigned char outFlag;
	debug( "setIODirection: %02x%02x%02x%02x%02x%02x%02x, attached=%d",
		direction[0],
		direction[1],
		direction[2],
		direction[3],
		direction[4],
		direction[5],
		direction[6], AttachedState );

	if(AttachedState != TRUE)
		return -1;
	outFlag = 0;
	// 7 IO ports. put 7 bytes as bits in one byte
	for (i = 0; i < 7; i++) {
		if (direction[i]) {
			outFlag = (outFlag | (0x01 << i));
		}
	}
	//The first byte is the "Report ID" and does not get sent over the USB bus.  Always set = 0.
	OUTBuffer[0] = 0;
	OUTBuffer[1]= SET_IO_DIRECTION;
	OUTBuffer[2] = outFlag;
	r = writeData(3);
	usleep(20*1000);
	readData();			// read and clear the input buffer
	return r;
}

int PowerUSB::setOutputState(int output[])
{
	int r, i;
	unsigned char outFlag;

	if(AttachedState != TRUE)
		return -1;
	outFlag = 0;
	// 7 IO ports. put 7 bytes as bits in one byte
	for (i = 0; i < 7; i++) {
		if (output[i]) {
			outFlag = (outFlag | (0x01 << i));
		}
	}
	//The first byte is the "Report ID" and does not get sent over the USB bus.  Always set = 0.
	OUTBuffer[0] = 0;
	OUTBuffer[1]= SET_IO_OUTPUT;
	OUTBuffer[2] = outFlag;
	r = writeData(3);
	usleep(20*1000);
	readData();			// read and clear the input buffer
	return r;
}




int PowerUSB::getInputState(int input[])
{
	int r, i;
	unsigned char ch;

	if(AttachedState != TRUE)
		return -1;
	//The first byte is the "Report ID" and does not get sent over the USB bus.  Always set = 0.
	OUTBuffer[0] = 0;
	OUTBuffer[1]= GET_IO_INPUT;
	r = writeData(2);
	usleep(30*1000);
	r = readData();			// read and clear the input buffer
	ch = INBuffer[0];
	if (r >= 0)
	{
		for (i = 0; i < 7; i++)
			input[i] = (ch >> i) & 0x01;
	}
	return r;
}


int PowerUSB::generateClock(int port, int onTime, int offTime)
{
	int r;

	if(AttachedState != TRUE)
		return -1;

	//The first byte is the "Report ID" and does not get sent over the USB bus.  Always set = 0.
	OUTBuffer[0] = 0;
	OUTBuffer[1]= SET_IO_CLOCK;
	OUTBuffer[2] = port;
	OUTBuffer[3] = onTime;
	OUTBuffer[4] = offTime;

	r = writeData(5);
	usleep(20*1000);

	// read and clear the input buffer
	readData();
	return r;
}

int PowerUSB::getOutputState(int output[])
{
	int r, i;
	unsigned char ch;

	if(AttachedState != TRUE)
		return -1;
	//The first byte is the "Report ID" and does not get sent over the USB bus.  Always set = 0.
	OUTBuffer[0] = 0;
	OUTBuffer[1]= GET_IO_OUTPUT;
	r = writeData(2);
	usleep(30*1000);
	// read and clear the input buffer
	r = readData();
	ch = INBuffer[0];
	if (r >= 0)
	{
		for (i = 0; i < 7; i++) {
			output[i] = (ch >> i) & 0x01;
		}
	}
	return r;
}

int PowerUSB::setInputTrigger(int port, int outlet1, int outlet2, int outlet3, int out1, int out2)
{
	int r;

	debug( "SetInputTriggerPowerUSB: %d, %d, %d, %d, %d, %d, attached=%d\n",
		port,
		outlet1, outlet2, outlet3,
		out1, out2, AttachedState );

	if(AttachedState != TRUE)
		return -1;

	//The first byte is the "Report ID" and does not get sent over the USB bus.  Always set = 0.
	OUTBuffer[0] = 0;
	OUTBuffer[1]= SET_IO_TRIGGER;
	OUTBuffer[2] = port;
	OUTBuffer[3] = (outlet1 >> 8) | 0x00ff;
	OUTBuffer[4] = (outlet1 | 0x00ff);
	OUTBuffer[5] = (outlet2 >> 8) | 0x00ff;
	OUTBuffer[6] = (outlet2 | 0x00ff);
	OUTBuffer[7] = (outlet3 >> 8) | 0x00ff;
	OUTBuffer[8] = (outlet3 | 0x00ff);
	OUTBuffer[9] = (out1 >> 8) | 0x00ff;
	OUTBuffer[10] = (out1 | 0x00ff);
	OUTBuffer[11] = (out2 >> 8) | 0x00ff;
	OUTBuffer[12] = (out2 | 0x00ff);
	r = writeData(5); // Why is this not 13?
	usleep(20*1000);
	readData();			// read and clear the input buffer
	return r;
}

// Starts watchdog in the PowerUSB. 
// HbTimeSec: expected time for heartbeat
// numHbMisses: Number of accepted consicutive misses in heartbeat
// resetTimeSec: Amount of time to switch off the computer outlet
/////////////////////////////////////////////////////////////////////////

int PowerUSB::startWatchdogTimer(int HbTimeSec, int numHbMisses, int resetTimeSec)
{
	int r=0;
	if(AttachedState != TRUE)
		return -1;

	OUTBuffer[0] = 0;					
	OUTBuffer[1] = START_WDT;			// start watchdog code
	OUTBuffer[2] = 0;					// 
	OUTBuffer[3] = HbTimeSec;				// heartbeat time (all in seconds)
	OUTBuffer[4] = numHbMisses;			// hb times
	OUTBuffer[5] = resetTimeSec;			// reset time  
	r = writeData(6);
	return r;
}

// Stops the watchdog timer in PowerUSB
/////////////////////////////////////////////
int PowerUSB::stopWatchdogTimer()
{
	int r=0;
	if(AttachedState != TRUE)
		return -1;
	OUTBuffer[0] = 0;			
	OUTBuffer[1] = STOP_WDT;			// stop watchdog code
	r = writeData(2);
	usleep(100*1000);
	readData();
	usleep(50*1000);
	return r;
}

// Get the current state of Watchdog in PowerUSB
// Return 0: watchdog is not running, 1: Watchdog is active, 2: In PowerCycle phase
///////////////////////////////////////////////////////////////////////////////////////
int PowerUSB::getWatchdogStatus()
{
	int r;
	if(AttachedState != TRUE)
		return -1;
	OUTBuffer[0] = 0;			//The first byte is the "Report ID" and does not get sent over the USB bus.  Always set = 0.
	OUTBuffer[1]= READ_WDT;
	r = writeData(2);
	usleep(100*1000);
	readData();
	r = INBuffer[0];
	usleep(20*1000);
	return r;
}

// Sends the Heartbeat to PowerUSB
///////////////////////////////////
int PowerUSB::sendHeartBeat()
{
	int r=0;
	if(AttachedState != TRUE)
		return -1;
	OUTBuffer[0] = 0;			
	OUTBuffer[1] = HEART_BEAT;			// send heart beat
	r = writeData(2);
	return r;
}

// Switch off the computer outlet and switch it back on after resetTimeSec
///////////////////////////////////////////////////////////////////////////////
int PowerUSB::powerCycle(int resetTimeSec)
{
	int r=0;
	if(AttachedState != TRUE)
		return -1;
	OUTBuffer[0] = 0;			
	OUTBuffer[1] = POWER_CYCLE;			// send heart beat
	OUTBuffer[2] = resetTimeSec;
	r = writeData(3);
	usleep(100*1000);
	readData();
	usleep(50*1000);
	return r;
}

// Writes the data to current open device
/////////////////////////////////////////////////////////////////////////////////////////////////
int PowerUSB::writeData(int len)
{
	int i, r=0;
	debug( "writeData(len=%d): connected=%d, currentDev=%d (%p)\n",
		len, AttachedState, CurrentDevice,
		CurrentDevice >= 0 ? AttachedDeviceHandles[CurrentDevice] : NULL );
	if(AttachedState != TRUE || CurrentDevice < 0 )
		return -1;

	// Initialize unused bytes to 0xFF for lower EMI and power consumption
	// when driving the USB cable.
	memset( OUTBuffer+len, 0xff, BUF_WRT );

	//Now send the packet to the USB firmware on the microcontroller
	//Blocking function, unless an "overlapped" structure is used
	int status = hid_set_nonblocking(AttachedDeviceHandles[CurrentDevice], 1);		
	
	if(status != -1)
	{
		r = hid_write(AttachedDeviceHandles[CurrentDevice], OUTBuffer, BUF_WRT);
	}
	return r;
}

// Reads the current open device
/////////////////////////////////////////////////////////////////////////////////////////////////
int PowerUSB::readData()
{
	int r = -1; 

	debug( "readData: connected=%d\n", AttachedState );
	if(AttachedState != TRUE)
		return -1;
	
	//Blocking function, unless an "overlapped" structure is used
	int status = hid_set_nonblocking(AttachedDeviceHandles[CurrentDevice], 1); 	

	if(status != -1)
	{
		r = hid_read(AttachedDeviceHandles[CurrentDevice], INBuffer, BUF_WRT);
	}
	return r;
}

// Searches all the devices in the computer and selects the devices that match our PID and VID
/////////////////////////////////////////////////////////////////////////////////////////////////
int PowerUSB::checkConnected()
{
	int FoundIndex = 0;
	hid_device *DeviceHandle = NULL;
	struct hid_device_info *devs = NULL, *cur_dev = NULL;

	devs = hid_enumerate(0x4d8, 0x3f);
	cur_dev = devs;	
	debug( "checkConnected: devs=%p\n", devs );
	int attach_cnt = 0;
	while (cur_dev) 
	{
		if(FoundIndex < POWER_USB_MAXNUM)
		{
			if( AttachedDeviceHandles[FoundIndex] == NULL ) {
				debug( "Open for 0x%04hx:0x%04x: serial#=%ls\n", cur_dev->vendor_id, cur_dev->product_id, cur_dev->serial_number ? cur_dev->serial_number : L"NULL" );
				DeviceHandle = hid_open(cur_dev->vendor_id, cur_dev->product_id, cur_dev->serial_number);
				if( DeviceHandle != NULL ) {
					debug( "hid_open == %p\n", DeviceHandle );


					debug( " # device_handle=%p\n", DeviceHandle->device_handle);
					debug( " # input_endpoint=%d\n", DeviceHandle->input_endpoint);
					debug( " # output_endpoint=%d\n", DeviceHandle->output_endpoint);
					debug( " # input_ep_max_packet_size=%d\n", DeviceHandle->input_ep_max_packet_size);
					debug( " # interface=%d\n", DeviceHandle->interface);
					debug( " # manufacturer_index=%d\n", DeviceHandle->manufacturer_index);
					debug( " # product_index=%d\n", DeviceHandle->product_index);
					debug( " # serial_index=%d\n", DeviceHandle->serial_index);
					debug( " # blocking=%d\n", DeviceHandle->blocking); /* boolean */
					debug( " # shutdown_thread=%d\n", DeviceHandle->shutdown_thread);
					debug( " # transfer=%p\n", DeviceHandle->transfer);
					debug( " # input_reports=%p\n", DeviceHandle->input_reports);

					debug( " * Path '%s' handle = %p\n",  cur_dev->path, DeviceHandle );
					debug( " * path=%s\n", cur_dev->path);
					debug( " * vendor_id=0x%04hx\n", cur_dev->vendor_id);
					debug( " * product_id=0x%04hx\n", cur_dev->product_id);
					debug( " * serial_number=%ls\n", cur_dev->serial_number);
					debug( " * release_number=%hu\n", cur_dev->release_number);
					debug( " * manufacturer_string=%ls\n", cur_dev->manufacturer_string);
					debug( " * product_string=%ls\n", cur_dev->product_string);
					debug( " * usage_page=%hu\n", cur_dev->usage_page);
					debug( " * usage=%hu\n", cur_dev->usage);
					debug( " * interface_number=%d\n", cur_dev->interface_number);
					debug( " * next=%p\n", cur_dev->next);

					++attach_cnt;
					AttachedDeviceHandles[FoundIndex] = DeviceHandle;
				} else {
					perror("hid_open_path");
					debug( "Cannot open 0x%04x:0x%04x ('%s')\n",
						cur_dev->vendor_id, cur_dev->product_id, cur_dev->path );
				}
			} else {
				++attach_cnt;
				debug( "Already have handler at FoundIndex=%d\n", FoundIndex );
			}
		}
		FoundIndex = FoundIndex + 1;
		cur_dev = cur_dev->next;
	}

	hid_free_enumeration(devs);

	// nothing to open, stop
	if( attach_cnt == 0 ) {
		debug( "checkConnected: did not find any attached devices\n");
		return -1;
	}
	debug( "checkConnected: found dev=%d\n", FoundIndex );
	return FoundIndex;
}
