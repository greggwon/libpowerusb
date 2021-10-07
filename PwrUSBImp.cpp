/****************************************************************************
 * This rewrite of the PowerUSBImp class provides a more customizable and
 * resilent programming interface to the HID library use.
 *
 * Author:  Gregg Wonderly (SeqTech LLC - http://www.seqtek.com)
 ***************************************************************************/
 
#include "PwrUSBImp.h"
#include "PwrUSBHid.h"
#include <string>
#include <cstdarg>

/**
 *  Function for printing out debugging if enabled.  The template provides
 *  support for multiple arguments.
 */
template <typename T, typename... Args>
static void debug( T fmt, Args... args ) {
	if( PowerUSB::debugging ) {
		int len = std::snprintf( nullptr, 0, fmt, args... );
		char buf[len+100];
		std::snprintf( buf, sizeof(buf), fmt, args... );
		fprintf( stdout, "%s", buf );
	}
}

/**
 *  Class Level debugging variable can be set with static class reference.
 *  PowerUSB::debugging = 1;
 */
bool PowerUSB::debugging;

PowerUSB::PowerUSB() {
	CurrentDevice = -1;
	AttachedState = FALSE;
	AttachedDeviceCount = 0;
	memset( AttachedDeviceHandles, 0, sizeof(AttachedDeviceHandles) );
	memset( OUTBuffer, 0, sizeof( OUTBuffer ) );
	memset( INBuffer, 0, sizeof(INBuffer ) );
}

PowerUSB::~PowerUSB() {
	debug("~PowerUSB: hid_shutdown\n");
	hid_shutdown();
	debug("~PowerUSB: hid_exit\n");
	hid_exit();
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
			AttachedDeviceCount, PWRUSB_VENDOR_ID, PWRUSB_PRODUCT_ID);
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
				debug("closing at index %d: %p\n", i,  AttachedDeviceHandles[i]);
				hid_close(AttachedDeviceHandles[i]);
				AttachedDeviceHandles[i] = NULL;
			}
		}
		debug("Shutting down all HID and freeing data\n");
		hid_shutdown();
	}

	// Clear out all saved state on close.
	CurrentDevice = -1;
	AttachedState = FALSE;
	AttachedDeviceCount = 0;
	memset( AttachedDeviceHandles, 0, sizeof(AttachedDeviceHandles) );
	memset( OUTBuffer, 0, sizeof( OUTBuffer ) );
	memset( INBuffer, 0, sizeof(INBuffer ) );

	return 0;
}

int PowerUSB::getCurrentDevice() {
	return CurrentDevice;
}

/**
 *  Set the referenced device to the passed index.  When there is no
 *  such device, the current device remains unchanged.
 *
 *  Returns: current selected device after change, if any.
 */ 
int PowerUSB::setCurrentDevice(int devno)
{
	debug( "SetCurrentPowerUSB =%d, attached=%d\n", devno, AttachedState );
	if (devno < 0 || devno >= AttachedDeviceCount) {
		devno = AttachedDeviceCount - 1;
	}
	if( devno >= 0 )
		CurrentDevice = devno;
	return CurrentDevice;
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

template<typename T>
int PowerUSB::performUSBTask( T makePacket ) {
	debug( "PerformUSBTask attached=%d\n", AttachedState );

	if(!AttachedState)
		return -1;

	int n = 0;
	makePacket( OUTBuffer, n );
	int r = writeData(n);
	usleep(50*1000);	
	int i = readData();	 // read and clear the input buffer
	debug("PerformUSBTask=%d, readData: %d\n", r, i );
	return r;
}

template<typename T, typename S>
int PowerUSB::performUSBTaskWithResult( T makePacket, S results ) {
	debug( "PerformUSBTask attached=%d\n", AttachedState );

	if(!AttachedState)
		return -1;

	int n = 0;
	makePacket( OUTBuffer, n );
	int r = writeData(n);
	if( r >= 0 ) {
		usleep(50*1000);	
		int i = readData();			// read and clear the input buffer
		debug("Action reply: %d\n", i );
		results( INBuffer, i );
	}
	return r;
}

typedef struct _PortOper {
	int port;
	int on, off;
	_PortOper( int p, int onOp, int offOp ) {
		port = p;
		on = onOp;
		off = offOp;
	}
} PortOper;

typedef struct _PortState {
	int *port;
	int oper;
	_PortState( int *p, int op ) {
		port = p;
		oper = op;
	}
} PortState;

// Sets the state of the outlet 1 and 2
// A=Outlet1 On. B=Outlet1 Off
// C=Outlet2 On  D=Outlet2 Off
////////////////////////////////////////
int PowerUSB::setPort(int port1, int port2, int port3)
{
	PortOper ops[] = { 
		{ port1, ON_PORT1, OFF_PORT1 },
		{ port2, ON_PORT2, OFF_PORT2 },
		{ port3, ON_PORT3, OFF_PORT3 },
	};
	int n = 0;
	debug( "SetStatePowerUSB =%d,%d,%d, attached=%d\n", port1, port2, port3, AttachedState );
	for( int i = 0; i < 3; ++i ) {
		if( ops[i].port >= 0 ) {
			int r = performUSBTask( [&i,&ops]( unsigned char *buf, int &n ) {
				buf[n++] = 0;
				if( ops[i].port ) buf[n++] = ops[i].on;
				else buf[n++] = ops[i].off;
				debug("Setting port=%d to %d ('%c')\n", i+1, buf[1], buf[1] );
			} );
			n++;
		}
	}
	return n;
}

int PowerUSB::setDefaultState(int state1, int state2, int state3)
{
	PortOper ops[] = { 
		{ state1, DEFON_PORT1, DEFOFF_PORT1 },
		{ state2, DEFON_PORT2, DEFOFF_PORT2 },
		{ state3, DEFON_PORT3, DEFOFF_PORT3 },
	};

	debug( "SetDefaultStatePowerUSB =%d,%d,%d, attached=%d\n", state1, state2, state3, AttachedState );
	int n = 0;
	for( int i = 0; i < 3; ++i ) {
		if( ops[i].port >= 0 ) {
			int r = performUSBTask( [&i,&ops]( unsigned char *buf, int &n ) {
				buf[n++] = 0;
				if( ops[i].port ) buf[n++] = ops[i].on;
				else buf[n++] = ops[i].off;
				debug("Setting default state for port=%d to %d ('%c')\n", i+1, buf[1], buf[1] );
			} );
			n++;
		}
	}
	return n;
}

int PowerUSB::readPortState(int *port1, int *port2, int *port3)
{
	PortState ops[] = { 
		{ port1, READ_P1 },
		{ port2, READ_P2 },
		{ port3, READ_P3 },
	};
	debug( "readPortState: 1=%p, 2=%p, 3=%p, attached=%d\n", port1, port2, port3, AttachedState );
	int n = 0;
	for( int i = 0; i < 3; ++i ) {
		if( *ops[i].port ) {
			int r = performUSBTaskWithResult( [&i,&ops]( unsigned char *buf, int &n ) {
				buf[n++] = 0;
				buf[n++] = ops[i].oper;
			}, [&i, &ops]( unsigned char *results, int n ) {
				*ops[i].port = results[0];
				debug("found port=%d is %d ('%s')\n", i+1, results[0], results[0] ? "on" : "off" );
			});
			n++;
		}
	}
	return n;
}

bool PowerUSB::onOffCheck( int port, int status ) {
	bool state;
	debug("onOffCheck( %d, %d )\n", port, status );
	switch(status) {
		case DEFOFF_PORT1:
		case DEFOFF_PORT2:
		case DEFOFF_PORT3:
		case OFF_PORT1:
		case OFF_PORT2:
		case OFF_PORT3:
			state = false;
			break;

		case DEFON_PORT1:
		case DEFON_PORT2:
		case DEFON_PORT3:
		case ON_PORT1:
		case ON_PORT2:
		case ON_PORT3:
			state = true;
			break;

		default:
			state = status;
			break;
	}
	return state;
}

int PowerUSB::readDefaultPortState( int *port1, int *port2, int *port3 )
{
	PortState ops[] = { 
		{ port1, READ_P1_PWRUP },
		{ port2, READ_P2_PWRUP },
		{ port3, READ_P3_PWRUP },
	};
	debug( "readPortState: 1=%p, 2=%p, 3=%p, attached=%d\n", port1, port2, port3, AttachedState );
	int n = 0;
	for( int i = 0; i < 3; ++i ) {
		if( *ops[i].port ) {
			int r = performUSBTaskWithResult( [&i,&ops]( unsigned char *buf, int &n ) {
				buf[n++] = 0;
				buf[n++] = ops[i].oper;
			}, [&i, &ops]( unsigned char *results, int n ) {
				*ops[i].port = results[0];
				debug("found default state for port=%d is %d ('%s')\n", i+1, results[0], results[0] ? "on" : "off" );
			});
			n++;
		}
	}
	return n;
}

// Reads the current version of firmware
// As of Nov 1st 2010. This function is not implemented in firmware
///////////////////////////////////////////////////////////////////////
int PowerUSB::getFirmwareVersion()
{
	int r, vers = -1;
	r = performUSBTaskWithResult( [](unsigned char *buf, int &n ){
		buf[n++] = 0;
		buf[n++]= READ_FIRMWARE_VER;
	}, [&r,&vers](unsigned char *res, int i ) {
		vers = res[0];
	});
	return vers;
}

std::string PowerUSB::getModelName() {
	const char *name;
	int n = getModel();
	switch( n ) {
		case 0: name = "Not connected (0)"; break;
		case 1: name = "Basic(1)"; break;
		case 2: name = "Digital I/O(2)"; break;
		case 3: name = "Computer Watchdog(3)"; break;
		case 4: name = "Smart Pro(4)"; break;
		default: {
			char buf[100];
			snprintf(buf, sizeof(buf), "unknown(%d)",n );
			return std::string(buf); 
		}
		break;
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
	performUSBTaskWithResult( [](unsigned char *buf, int &n ){
		buf[n++] = 0;
		buf[n++]= READ_MODEL;
	}, [&r](unsigned char *res, int i ) {
		r = res[0];
	});
	return r;
}

// Current Sensing Related Functions
////////////////////////////////////////////////////
int PowerUSB::readCurrentDevice(int *current)
{
	int r;
	r = performUSBTaskWithResult( [](unsigned char *buf, int &n ){
		buf[n++] = 0;
		buf[n++]= READ_CURRENT;
	}, [&r,&current](unsigned char *res, int i ) {
		*current = res[0]<<8 | res[1];
	});
	debug( "readCurrent: current=%p == %d\n", current, *current );
	return r;
}

int PowerUSB::readCurrentCum(int *currentCum)
{
	int r;
	*currentCum = 0;
	r = performUSBTaskWithResult( [](unsigned char *buf, int &n ){
		buf[n++] = 0;
		buf[n++]= READ_CURRENT_CUM;
	}, [&r, &currentCum](unsigned char *res, int i ) {
		*currentCum = res[0]<<24 | res[1]<<16 | res[2]<<8 | res[3];
	});

	debug( "readCurrentCum: currentCum=%p == %d\n", currentCum, *currentCum );
	return r;
}

int PowerUSB::resetCurrentCounter()
{
	int r;
	r = performUSBTask( [](unsigned char *buf, int &n ){
		buf[n++] = 0;
		buf[n++]= RESET_CURRENT_COUNT;
	});
	return r;
}

int PowerUSB::setCurrentSensRatio(int currentRatio)
{
	int ratio = -1;
	int r = performUSBTaskWithResult( [currentRatio](unsigned char *buf, int &n ){
		buf[n++] = 0;
		buf[n++]= SET_CURRENT_RATIO;
		buf[n++]= currentRatio;
	}, [&ratio](unsigned char *res, int i ) {
		ratio = res[0];
	});

	debug( "resetCurrentSensRatio=%d, ratio=%d\n", r, ratio );
	return ratio;
}

int PowerUSB::setOverload(int overload)
{
	int r;
	r = performUSBTask( [overload](unsigned char *buf, int &n ){
		buf[n++] = 0;
		buf[n++]= WRITE_OVERLOAD;
		buf[n++]= overload;
	});
	return r;
}

int PowerUSB::getOverload()
{
	int over = -1;
	int r = performUSBTaskWithResult( [](unsigned char *buf, int &n ){
		buf[n++] = 0;
		buf[n++]= READ_OVERLOAD;
	}, [&over](unsigned char *res, int i ) {
		over = res[0];
	});

	debug( "getOverload=%d, over=%d\n", r, over );
	return over;
}

int PowerUSB::resetBoard()
{
	int r;
	r = performUSBTask( [](unsigned char *buf, int &n ){
		buf[n++] = 0;
		buf[n++]= RESET_BOARD;
	});
	return r;
}

int PowerUSB::setCurrentOffset()
{
	int r;
	r = performUSBTask( [](unsigned char *buf, int &n ){
		buf[n++] = 0;
		buf[n++]= SET_CURRENT_OFFSET;
	});
	return r;
}

///////////////////////////////////////
// Digital IO related functions
/////////////////////////////////////
int PowerUSB::setIODirection(int direction[])
{
	debug( "setIODirection: %02x%02x%02x%02x%02x%02x%02x, attached=%d",
		direction[0], direction[1], direction[2],
		direction[3], direction[4], direction[5],
		direction[6], AttachedState );

	unsigned char outFlag = 0;
	// 7 IO ports. put 7 bytes as bits in one byte
	for ( int i = 0; i < 7; i++) {
		if (direction[i]) {
			outFlag = (outFlag | (0x01 << i));
		}
	}
	int r;
	r = performUSBTask( [outFlag](unsigned char *buf, int &n ){
		buf[n++] = 0;
		buf[n++]= SET_IO_DIRECTION;
		buf[n++]= outFlag;
	});
	return r;
}

int PowerUSB::setOutputState(int outputs[])
{
	unsigned char outFlags = 0;
	// 7 IO ports. put 7 bytes as bits in one byte
	for (int i = 0; i < 7; i++) {
		if (outputs[i]) {
			outFlags = (outFlags | (0x01 << i));
		}
	}
	int r;
	r = performUSBTask( [outFlags](unsigned char *buf, int &n ){
		buf[n++] = 0;
		buf[n++]= SET_IO_OUTPUT;
		buf[n++]= outFlags;
	});
	return r;
}

int PowerUSB::getInputState(int input[])
{
	int r = performUSBTaskWithResult( [](unsigned char *buf, int &n ){
		buf[n++] = 0;
		buf[n++]= GET_IO_INPUT;
	}, [&input](unsigned char *res, int i ) {
		unsigned char ch = res[0];
		for (int j = 0; j < 7; j++)
			input[j] = (ch >> j) & 0x01;

	});
	return r;
}

int PowerUSB::generateClock(int port, int onTime, int offTime)
{
	int r;
	r = performUSBTask( [port, onTime, offTime](unsigned char *buf, int &n ){
		buf[n++] = 0;
		buf[n++]= SET_IO_CLOCK;
		buf[n++]= port;
		buf[n++]= onTime & 0xff;
		buf[n++]= offTime & 0xff;
	});
	return r;
}

int PowerUSB::getOutputState(int outputs[])
{
	int r = performUSBTaskWithResult( [](unsigned char *buf, int &n ){
		buf[n++] = 0;
		buf[n++]= GET_IO_OUTPUT;
	}, [&outputs](unsigned char *res, int i ) {
		unsigned char ch = res[0];
		for (int j = 0; j < 7; j++)
			outputs[j] = (ch >> j) & 0x01;

	});
	return r;
}

int PowerUSB::setInputTrigger(int port, int outlet1, int outlet2, int outlet3, int out1, int out2)
{

	debug( "SetInputTriggerPowerUSB: %d, %d, %d, %d, %d, %d, attached=%d\n",
		port,
		outlet1, outlet2, outlet3,
		out1, out2, AttachedState );

	int r;
	r = performUSBTask( [port, outlet1, outlet2, outlet3, out1, out2](unsigned char *buf, int &n ){
		buf[n++] = 0;
		buf[n++]= SET_IO_TRIGGER;
		buf[n++] = port;
		buf[n++] = (outlet1 >> 8) & 0x00ff;
		buf[n++] = (outlet1 & 0x00ff);
		buf[n++] = (outlet2 >> 8) & 0x00ff;
		buf[n++] = (outlet2 & 0x00ff);
		buf[n++] = (outlet3 >> 8) & 0x00ff;
		buf[n++] = (outlet3 & 0x00ff);
		buf[n++] = (out1 >> 8) & 0x00ff;
		buf[n++] = (out1 & 0x00ff);
		buf[n++] = (out2 >> 8) & 0x00ff;
		buf[n++] = (out2 & 0x00ff);
	});
	return r;
}

// Starts watchdog in the PowerUSB. 
// HbTimeSec: expected time for heartbeat
// numHbMisses: Number of accepted consicutive misses in heartbeat
// resetTimeSec: Amount of time to switch off the computer outlet
/////////////////////////////////////////////////////////////////////////

int PowerUSB::startWatchdogTimer(int HbTimeSec, int numHbMisses, int resetTimeSec)
{
	int r;
	r = performUSBTask( [HbTimeSec, numHbMisses, resetTimeSec](unsigned char *buf, int &n ){
		buf[n++] = 0;
		buf[n++]= START_WDT;
		buf[n++] = 0;
		buf[n++] = HbTimeSec;
		buf[n++] = numHbMisses;
		buf[n++] = resetTimeSec;
	});
	return r;
}

// Stops the watchdog timer in PowerUSB
/////////////////////////////////////////////
int PowerUSB::stopWatchdogTimer()
{
	int r;
	r = performUSBTask( [](unsigned char *buf, int &n ){
		buf[n++] = 0;
		buf[n++]= STOP_WDT;
	});
	return r;
}

// Get the current state of Watchdog in PowerUSB
// Return 0: watchdog is not running, 1: Watchdog is active, 2: In PowerCycle phase
///////////////////////////////////////////////////////////////////////////////////////
int PowerUSB::getWatchdogStatus()
{
	int st = -1;
	int r = performUSBTaskWithResult( [](unsigned char *buf, int &n ){
		buf[n++] = 0;
		buf[n++]= READ_WDT;
	}, [&st](unsigned char *res, int i ) {
		st = res[0];
	});
	return st;
}

// Sends the Heartbeat to PowerUSB
///////////////////////////////////
int PowerUSB::sendHeartBeat()
{
	int r;
	r = performUSBTask( [](unsigned char *buf, int &n ){
		buf[n++] = 0;
		buf[n++]= HEART_BEAT;
	});
	return r;
}

// Switch off the computer outlet and switch it back on after resetTimeSec
///////////////////////////////////////////////////////////////////////////////
int PowerUSB::powerCycle(int resetTimeSec)
{
		int r;
	r = performUSBTask( [resetTimeSec](unsigned char *buf, int &n ){
		buf[n++] = 0;
		buf[n++]= POWER_CYCLE;
		buf[n++]= resetTimeSec;
	});
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
	if(AttachedState == FALSE || CurrentDevice < 0 )
		return -1;

	// Initialize unused bytes to 0xFF for lower EMI and power consumption
	// when driving the USB cable.
	memset( OUTBuffer+len, 0xff, sizeof(OUTBuffer)-len );

	//Now send the packet to the USB firmware on the microcontroller
	//Blocking function, unless an "overlapped" structure is used
	int status = hid_set_nonblocking(AttachedDeviceHandles[CurrentDevice], 1);		
	
	if(status != -1)
	{
		r = hid_write(AttachedDeviceHandles[CurrentDevice], OUTBuffer, sizeof(OUTBuffer));
	}
	return r;
}

// Reads the current open device
/////////////////////////////////////////////////////////////////////////////////////////////////
int PowerUSB::readData()
{
	int r = -1; 

	debug( "readData: connected=%d\n", AttachedState );
	if(AttachedState == FALSE)
		return -1;
	
	//Blocking function, unless an "overlapped" structure is used
	int status = hid_set_nonblocking(AttachedDeviceHandles[CurrentDevice], 1); 	

	if(status != -1)
	{
		r = hid_read(AttachedDeviceHandles[CurrentDevice], INBuffer, sizeof(INBuffer));
	}
	return r;
}

// Searches all the devices in the computer and selects the last device that matches our PID and VID
/////////////////////////////////////////////////////////////////////////////////////////////////
int PowerUSB::checkConnected( )
{
	int FoundIndex = 0;
	hid_device *DeviceHandle = NULL;
	struct hid_device_info *devs = NULL, *cur_dev = NULL;

	// Find our types of devices.
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
					AttachedDeviceCount = FoundIndex+1;
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
	debug( "checkConnected: found dev=%d of %d\n", FoundIndex, AttachedDeviceCount );

	return FoundIndex;
}

bool PowerUSB::haveDeviceAtIndex( int useDev ) {
	return ( useDev >= 0 && useDev < POWER_USB_MAXNUM && AttachedDeviceHandles[useDev] != NULL );
}

bool PowerUSB::selectDevice( int useDev ) {
	if( haveDeviceAtIndex(useDev) ) {
		CurrentDevice = useDev;
		return true;
	}
	return false;
}