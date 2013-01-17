/*
 * A800K.cpp: Drivers for the AVerTV A800 USB2 device
 *
 * Portions of this code come from the DVB drivers in the linux kernel.
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public License
 * as published by the Free Software Foundation; either version 2.1
 * of the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.	 See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place - Suite 330, Boston, MA  02111-1307, USA.
 */

#include <MMInputFamily/I2CNub.hpp>
#include <MMInputFamily/DVBFrontend.hpp>
#include <UDMUtils/UDMResourceFile.hpp>

#include "A800K.h"

#include <IOKit/IOLib.h>
#include <IOKit/usb/IOUSBDevice.h>
#include <IOKit/usb/IOUSBInterface.h>
#include <IOKit/usb/IOUSBPipe.h>
#include <IOKit/IOMemoryDescriptor.h>
#include <IOKit/IOLocks.h>
#include <libkern/OSByteOrder.h>
#include <libkern/c++/OSArray.h>

extern "C"
{
#include <pexpert/pexpert.h>
}

#define printf IOLog



// structs for commands understood by this device

#pragma push
#pragma pack(1)
struct A800USBCommand
{	// olen does not include command, whereas ilen includes whole response
	A800USBCommand( uint8 cid, uint8 olena, uint8 ilena ) :
		olen( olena+1 ), ilen( ilena ), command( cid ) { }
	uint8	olen;
	uint8	ilen;
	uint8	unusedAlign;
	uint8	command;
};

#define A800CommandBoth(NAME,CID,OLEN,ILEN) struct NAME : public A800USBCommand \
	{ NAME() : A800USBCommand( CID, OLEN, ILEN ) { }
#define A800CommandOut(NAME,CID) struct NAME : public A800USBCommand \
	{ NAME() : A800USBCommand( CID, sizeof(NAME)-sizeof(A800USBCommand), 0 ) { }
#define A800CommandIn(NAME,CID) struct NAME : public A800USBCommand \
	{ NAME() : A800USBCommand( CID, 0, sizeof(NAME)-sizeof(A800USBCommand) ) { }

A800CommandBoth( A800I2CWrite, DIBUSB_REQ_I2C_WRITE, 1, 0 )
	uint8		addr;
	uint8		data[10];

	void		set( uint8 addra, const void * dataa, uint sizea )
	{
		uint8 size;
		
		addr = (addra<<1);
		size = min( sizea, sizeof(data) );
		memcpy( data, dataa, size );
		olen = 2+size;
	}
	bool		ok() const	{ return 1; } // I can't see how we're meant to check for success. It doesn't respond with anything.
};

A800CommandBoth( A800I2CWriteRead, DIBUSB_REQ_I2C_READ, 3, 1 )
	uint8		addr;
	uint8		data[10];

	void		set( uint8 addra, const void * dataa, uint osizea, uint isizea )
	{
		uint8 isize, osize;
		
		addr = (addra<<1) | 1; // |1 because it is NOT write-only.
		osize = min( osizea, sizeof(data)-2 ); // we need two extra bytes of this
		memcpy( data, dataa, osize );
		olen = 4+osize;
		
		isize = min( isizea, sizeof(data) );
		data[osize+0] = (isize >> 8) & 0xff;
		data[osize+1] = isize & 0xff;
		ilen = isize;
	}
	bool		ok() const	{ return true; }
	void		get( void * dataa ) const
	{
		memcpy( dataa, &addr, ilen );
	}
};

A800CommandOut( A800SetStreamingControl, DIBUSB_REQ_SET_STREAMING_MODE )
	uint8		value;
	
	void set ()
	{
		value = 0;
		olen = 2;
	}
};

/* modify streaming of the FX2 */
#define DIBUSB_IOCTL_CMD_ENABLE_STREAM	0x01
#define DIBUSB_IOCTL_CMD_DISABLE_STREAM	0x02

A800CommandOut( A800SetIOCtl, DIBUSB_REQ_SET_IOCTL )
	uint8		onoff;
	uint8       wtf;
	
	void set( uint8 onoff_ ) {
		onoff = onoff_ ? DIBUSB_IOCTL_CMD_ENABLE_STREAM : DIBUSB_IOCTL_CMD_DISABLE_STREAM;
		wtf = 0;
		olen = 3;
	}
};

#pragma pop

static const uint32 MAX_COMMAND_SIZE = 12;



#include "A800USBI2CPipe.h"





// --------------------------- start of the A800USB2 class's code ---------------------------


OSDefineMetaClassAndStructors( A800USB2, MMInput );


static const IOPMPowerState powerStates[] =
{
	{ kIOPMPowerStateVersion1, 0, 0, 0,									0, 0, 0, 0, 0, 0, 0, 0 },
	{ kIOPMPowerStateVersion1, IOPMPowerOn, IOPMPowerOn, IOPMPowerOn,	0, 0, 0, 0, 0, 0, 0, 0 }
};


bool A800USB2::init( OSDictionary * pDict )
{
	providerNub_ = NULL;

	cmdDesc_ = NULL;
	cmdAddr_ = NULL;

	powerOn_ = false;
	buffer_ = NULL;
	level_ = NULL;
	going_ = false;
	
	i2cPipe_ = NULL;
	
	bufferDesc_ = NULL;
	levelDesc_ = NULL;

	dataBuffers_ = NULL;
	dataCompletion_ = NULL;

	bool res = this->MMInput::init( pDict );
	IOLog( "A800USB2::init\n" );
	return res;
}

void A800USB2::free()
{
	IOLog( "A800USB2::free\n" );
	
	this->MMInput::free();
}

IOService * A800USB2::probe( IOService * provider, SInt32 * score )
{
	IOLog( "A800USB2::probe: always good for A800\n" );
	return this->MMInput::probe( provider, score );
}


/// temp utility function to print out device strings... just for show
static void printDevString( IOUSBDevice * pDev, const char * format, UInt8 idx )
{
	if (idx != 0)
	{
		char str[256];
		if (pDev->GetStringDescriptor( idx, str, 255 ) == kIOReturnSuccess)
		{
			if (str[0] != 0)
			{
				str[255] = 0;
				IOLog( format, str );
			}
		}
	}
}

bool A800USB2::start( IOService * provider )
{
    IOLog( "A800USB2::start: provider %p\n", provider );
	IOLog( "\tA800USB2.cpp compiled at " __DATE__ " " __TIME__ " AEST\n" );

	// call our super class
    if (!this->MMInput::start( provider )) return false;

	providerNub_ = NULL;

	// make sure we have the right nub
	udevice = OSDynamicCast(IOUSBDevice, provider);
	if (udevice != NULL)
	{
		IOUSBFindInterfaceRequest ir;
		
		IOLog( "\tsuccessfully cast provider nub to IOUSBDevice. Finding the right interface...\n" );
		
		if ( udevice->GetProductID() == 43008 ) {
			ir.bInterfaceClass = 0xFF;
			ir.bInterfaceSubClass = 0xFF;
			ir.bInterfaceProtocol = 0xFF;
			ir.bAlternateSetting = 0;
		} else {
			ir.bInterfaceClass = 0xFF;
			ir.bInterfaceSubClass = 0;
			ir.bInterfaceProtocol = 0;
			ir.bAlternateSetting = 0;
		}
		
		udevice->open( provider );
		
		udevice->SetConfiguration( provider, 1, false );
		
		providerNub_ = udevice->FindNextInterface( NULL, &ir );
		
		if ( providerNub_ == NULL ) {
			IOLog( "\tCouldn't find any interfaces. Oof.\n" );
			return false;
		}
	}	

	// make sure we have the right nub
	if ( providerNub_ == NULL )
		providerNub_ = OSDynamicCast(IOUSBInterface, provider);
	
	if (providerNub_ == NULL)
	{
		IOLog( "\tcouldn't cast provider nub to IOUSBInterface\n" );
		return false;
	}
	IOLog( "\tsuccessfully cast provider nub to IOUSBInterface.\n" );

	IOUSBDevice * pDev = providerNub_->GetDevice();
	IOLog( "\tdevinfo: addr %d speed %d avpower %d maxpacket %d vendor %d product %d devrelease %d #configs %d mfgstridx %d prodstridx %d ser#stridx %d\n",
		pDev->GetAddress(),
		pDev->GetSpeed(),
		int(pDev->GetBusPowerAvailable()),
		pDev->GetMaxPacketSize(),
		pDev->GetVendorID(),
		pDev->GetProductID(),
		pDev->GetDeviceRelease(),
		pDev->GetNumConfigurations(),
		pDev->GetManufacturerStringIndex(),
		pDev->GetProductStringIndex(),
		pDev->GetSerialNumberStringIndex() );
	printDevString( pDev, "\t  manufacturer: '%s'\n", pDev->GetManufacturerStringIndex() );
	printDevString( pDev, "\t  product     : '%s'\n", pDev->GetProductStringIndex() );
	printDevString( pDev, "\t  serial #    : '%s'\n", pDev->GetSerialNumberStringIndex() );

	// ask it for a few things to make sure it is happy
	IOReturn physres;
	UInt8 configNumber = UInt8(-1);
	physres = pDev->GetConfiguration( &configNumber );
	IOLog( "\tdevconf: physical configuration index is %d physres 0x%08X\n",
		int(configNumber), physres );
	USBStatus deviceStatus = USBStatus(-1);
	physres = pDev->GetDeviceStatus( &deviceStatus );
	IOLog( "\tdevconf: physical device status is 0x%04X physres 0x%08X\n",
		int(deviceStatus), physres );

	// open the USBInterface
	if (!providerNub_->open( this ))
	{
		IOLog( "\tcould not open USBInterface\n" );
		IOLog( "\tsneakily trying to close them...\n" );
		IOSleep( 1000 );
		/*
		OSIterator * iter = providerNub_->getOpenClientIterator();
		if (iter != NULL)
		{
			OSObject * oso = iter->getNextObject();
			IOLog( "\toso is at 0x%08X\n", int(oso) );
			IOService * extant = oso ? OSDynamicCast( IOService, oso ) : NULL;
			IOLog( "\tclient is at 0x%08X\n", int(extant) );
			IOSleep( 1000 );
			if (extant != 0)
			{
				providerNub_->close( extant );
				IOLog( "\tclose didn't crash! .. try reloading now\n" );
			}
			iter->release();
		}
		*/
		pDev->ResetDevice();
		if (!providerNub_->open( this ))
		{
			return false;
		}
	}
	IOLog( "\tsuccessfully opened USBInterface\n" );

	// set the 'alternate setting' that we use
	uint8 desiredAltSet = providerNub_->GetAlternateSetting();
	IOLog( "\tcurrent alt setting is %d\n", desiredAltSet );
	switch (pDev->GetProductID())
	{
		case 43008:			// cold
			desiredAltSet = 0;
			break;
		case 43009:			// warm
			desiredAltSet = 1;
			break;
	}
	if (desiredAltSet != providerNub_->GetAlternateSetting())
	{
		if (providerNub_->SetAlternateInterface( this, desiredAltSet ) != kIOReturnSuccess)
		{
			IOLog( "\tcould not set alternate setting value to %d (continuing anyway)\n",
				desiredAltSet );
			//providerNub_->close( this );
			//return false;
		}
		else
		{
			IOLog( "\tsucessfully set alternate setting value to %d\n", desiredAltSet );
		}
	}

	IOLog( "\tifinfo: configval %d ifnum %d numeps %d ifclass %d ifsubclass %d ifprot %d ifstridx %d\n",
		providerNub_->GetConfigValue(),
		providerNub_->GetInterfaceNumber(),
		providerNub_->GetNumEndpoints(),
		providerNub_->GetInterfaceClass(),
		providerNub_->GetInterfaceSubClass(),
		providerNub_->GetInterfaceProtocol(),
		providerNub_->GetInterfaceStringIndex() );
	printDevString( pDev, "\t  interface string: '%s'\n", providerNub_->GetInterfaceStringIndex() );

	// allocate memory for the command buffer
	cmdAddr_ = (uint8*)IOMalloc( MAX_COMMAND_SIZE );
	cmdDesc_ = IOMemoryDescriptor::withAddress( cmdAddr_, MAX_COMMAND_SIZE, kIODirectionOutIn );
	IOLog( "\tallocated command buffer...\n" );

	// now initialise the hardware
	if (!this->initHardware() || dataInPipe_ == NULL)
	{
		if (dataInPipe_ == NULL)
			IOLog( "\thopefully device will re-enumerate and we will re-match\n" );
		else
			IOLog( "\tfailed to init hardware :(\n" );
		cmdDesc_->release();
		IOFree( cmdAddr_, MAX_COMMAND_SIZE );
		providerNub_->close( this );
		return false;
	}
	IOLog( "\tsuccessfully initialised hardware!\n" );

	IOLog( "\tregistering service...\n" );
	this->registerService();
	
	// make the i2c pipe
	IOSleep( 1000 );
	i2cPipe_ = new A800USBI2CPipe( this );
	IOLog( "\tmaking nubs...\n" );
	i2cPipe_->scanBusGlue( this, nubs_, kIOI2CScanArgumentWhole );
	IOLog( "\t%d nubs found\n", int(nubs_.size()) );

	IOLog( "\tinitialising power management...\n" );
	powerOn_ = true;
	this->PMinit();
	this->registerPowerDriver( this, const_cast<IOPMPowerState*>(powerStates),
		sizeof(powerStates)/sizeof(powerStates[0]) );
	providerNub_->joinPMtree( this );

	IOLog( "\tfinished starting.\n" );
	return true;
}


void A800USB2::stop( IOService * provider )
{
	IOLog( "A800USB2::stop\n" );

	// deactivate if we are activated
	if (buffer_ != NULL)
		this->deactivate();

	// deregister from power management
	this->PMstop();

	// stop all our nubs first
	if (i2cPipe_ != NULL)
		i2cPipe_->stopBusGlue( this, nubs_ );

	// delete our i2c pipe
	if (i2cPipe_ != NULL)
	{
		delete i2cPipe_;
		i2cPipe_ = NULL;
	}

	// disable everything
	if (powerOn_)
	{
		this->finiHardware();
	}

	// release allocated memory
	cmdDesc_->release();
	IOFree( cmdAddr_, MAX_COMMAND_SIZE );
	
	// pipes released by providerNub

	// close providerNub
	providerNub_->close( this );

	powerOn_ = false;
	
	// and call our super class
	this->MMInput::stop( provider );

	IOLog( "\tfinished stopping.\n" );
}

/**
 *	We need to close our provider in this function or we won't get stopped
 *	when the device is unplugged.
 */
bool A800USB2::didTerminate( IOService * provider, IOOptionBits options, bool * defer )
{
	IOLog( "A800USB2::didTerminate\n" );
	
	// deactivate if we are activated
	if (buffer_ != NULL)
		this->deactivate();
	
	// stop all our nubs
	if (i2cPipe_ != NULL)
		i2cPipe_->stopBusGlue( this, nubs_ );
	
	// disable everything
	if (powerOn_)
	{
		this->finiHardware();
	}

	// close providerNub
	providerNub_->close( this );

	powerOn_ = false;

	IOLog( "A800USB2::didTerminate: terminated\n" );

	// and call our base class implementation
	return this->MMInput::didTerminate( provider, options, defer );
}



/**
 *	Change whether or not this device is turned on
 */
IOReturn A800USB2::setPowerState( unsigned long powerStateOrdinal, IOService * whatDevice )
{
	IOService * pService;
	return IOPMAckImplied;
	// figure out what we need to do
	bool powerGoingOn = powerStateOrdinal != 0;
	
	if (!powerGoingOn && powerOn_)
	{
		IOLog( "A800USB2::setPowerState: %d\n", int(powerGoingOn) );

		// pass the message on to all our clients (direct and through i2c nubs)
		pService = this->getClient();
		if (pService != NULL) pService->setPowerState( powerStateOrdinal, this );
		for (uint32 i = 0; i < nubs_.size(); i++)
		{
			pService = nubs_[i]->getClient();
			if (pService != NULL) pService->setPowerState( powerStateOrdinal, this );
		}

		// and clean up our own hardware
		//this->finiHardware();
		powerOn_ = false;
	}
	else if (powerGoingOn && !powerOn_)
	{
		IOLog( "A800USB2::setPowerState: %d\n", int(powerGoingOn) );

		// reinitialise our own hardware
		//this->initHardware();
		powerOn_ = true;

		// pass the message on to all our clients (direct and through i2c nubs)
		for (uint32 i = 0; i < nubs_.size(); i++)
		{
			pService = nubs_[i]->getClient();
			if (pService != NULL) pService->setPowerState( powerStateOrdinal, this );
		}
		pService = this->getClient();
		if (pService != NULL) pService->setPowerState( powerStateOrdinal, this );
	}

	// we assume everyone is happy with doing this synchronously
	return IOPMAckImplied;
}

/**
 *	Someone wants to tell us something
 */
IOReturn A800USB2::message( UInt32 type, IOService * provider, void * argument )
{
	if (type != kIOI2CMessageRescanBus)
	{
		return this->MMInput::message( type, provider, argument );
	}
	
	i2cPipe_->scanBusGlue( this, nubs_, argument, provider );
	
	return 0;
}


MMInput::DataKind A800USB2::dataKind() const
{
	return DVB;
}

#define USER_BUFFER_SIZE (188*4096)

MMInput * A800USB2::activate( DataPond & pond )
{
	if (buffer_ != NULL) return NULL;
	
	if (dataInPipe_ == NULL)
	{
		IOLog( "A800USB2::activate: "
			"no data pipe so not activating (maybe you don't have USB2?)\n" );
		return false;
	}

	// make the buffer
	bufferDesc_ = IOBufferMemoryDescriptor::withOptions(
		kIOMemoryKernelUserShared, USER_BUFFER_SIZE + PAGE_SIZE, PAGE_SIZE );
	buffer_ = bufferDesc_ ? (uint8*)bufferDesc_->getBytesNoCopy() : 0;
	//buffer_ = (uint8*)IOMallocAligned( USER_BUFFER_SIZE + PAGE_SIZE, PAGE_SIZE );
	going_ = false;
	
	//if (!buffer_->good())
	if (buffer_ == NULL)
	{
		IOLog( "A800USB2::activate: could not create data buffer\n" );
		return NULL;
	}
	//bufferDesc_ = IOMemoryDescriptor::withAddress( buffer_, USER_BUFFER_SIZE, kIODirectionOutIn );
	bufferDesc_->setLength( USER_BUFFER_SIZE );
	bufferDesc_->setDirection( kIODirectionOutIn );

	//level_ = (UInt32*)IOMallocAligned( PAGE_SIZE, PAGE_SIZE );
	//levelDesc_ = IOMemoryDescriptor::withAddress( level_, PAGE_SIZE, kIODirectionOutIn );
	levelDesc_ = IOBufferMemoryDescriptor::withOptions(
		kIOMemoryKernelUserShared, PAGE_SIZE, PAGE_SIZE );
	levelDesc_->setDirection( kIODirectionOutIn );
	level_ = (UInt32*)levelDesc_->getBytesNoCopy();
	*level_ = 0;
	

	bool ok = true, gotone = false;

	// ask all our i2c devices to activate
	for (uint32 i = 0; i < nubs_.size(); i++)
	{
		DVBFrontend * fe = DVBFrontend::checkClient( nubs_[i] );
		if (fe == NULL) continue;
		ok &= fe->activate();
		gotone = true;
	}
	if ((!ok) || (!gotone))
	{
		this->deactivate();
		IOLog( "A800USB2::activate: could not activate. ok %d gotone %d\n",
			int(ok), int(gotone) );

		return NULL;
	}
	
	IOLog( "A800USB2::activate: activated\n" );

	pond.dataDesc_ = bufferDesc_;
	pond.dataBegin_ = 0;
	pond.dataEnd_ = pond.dataBegin_ + bufferDesc_->getLength();
	pond.dataBlobSize_ = 188;

	pond.levelDesc_ = levelDesc_;
	pond.levelAddress_ = 0;
	pond.levelFormat_ =
#ifndef __LITTLE_ENDIAN__
		0
#else
		1
#endif
		| 16;	// unsynced
	pond.levelBegin_ = 0;

	return this;
}

void A800USB2::deactivate()
{
	this->fill( false );

	// ask all our i2c devices to deactivate
	for (uint32 i = 0; i < nubs_.size(); i++)
	{
		DVBFrontend * fe = DVBFrontend::checkClient( nubs_[i] );
		if (fe == NULL) continue;
		fe->deactivate();
	}

	if (buffer_ != NULL)
	{
		levelDesc_->release();
		levelDesc_ = NULL;
		//IOFreeAligned( level_, PAGE_SIZE );
		level_ = NULL;

		bufferDesc_->release();
		bufferDesc_ = NULL;
		//IOFreeAligned( buffer_, USER_BUFFER_SIZE + PAGE_SIZE );
		buffer_ = NULL;
		
		IOLog( "A800USB2::deactivate: deactivated\n" );
	}
}

MMInput::TuningParams A800USB2::tuningSpace() const
{
	return MMInput::TuningParams( OFDM );
}


bool A800USB2::tune( OSDictionary * params )
{
	if (buffer_ == NULL) return false;

	bool ok = true;

	// ask all our i2c devices to tune
	for (uint32 i = 0; i < nubs_.size(); i++)
	{
		DVBFrontend * fe = DVBFrontend::checkClient( nubs_[i] );
		if (fe == NULL) continue;
		ok |= fe->tune( params );
	}

	return ok;
}


static void dataCompleted( void * target, void * parameter, IOReturn status, UInt32 bufferSizeRemaining );

#define FRAMES_PER_READ 1	// since we are using bulk endpoints :(
#define READS_TO_QUEUE 256

#define A800_INCLUDE 1
#include "DIB3000.h"

bool A800USB2::fill( bool go )
{
	// this is one that we can do!
	if (buffer_ == NULL) return false;
	

	if (go && (!going_))
	{
		// start going
		*level_ = 0;
		
		IOLog( "A800USB2::fill(go): about to queue data transactions\n" );

		int frameSize = dataInPipe_->GetMaxPacketSize();
		if (frameSize <= 0)
		{
			IOLog( "A800USB2::fill(go): frameSize 0, that bad\n" );
			frameSize = 512;
		}
		if (frameSize != 512)
		{
			IOLog( "A800USB2::fill(go): frameSize %d not 512, overriding\n", frameSize );
			frameSize = 512;
		}

		dataMem_ = (uint8*)IOMalloc( frameSize*FRAMES_PER_READ*READS_TO_QUEUE );
		IOLog( "A800USB2::fill(go): allocated %d bytes at 0x%08X since frameSize %d\n",
			frameSize*FRAMES_PER_READ*READS_TO_QUEUE, int(dataMem_), frameSize );

		dataBuffers_ = new IOMemoryDescriptor*[READS_TO_QUEUE];
		for (int i = 0; i < READS_TO_QUEUE; i++)
		{
			dataBuffers_[i] = IOMemoryDescriptor::withAddress(
				dataMem_ + frameSize*FRAMES_PER_READ*i,
				frameSize*FRAMES_PER_READ,
				kIODirectionOutIn );
		}

		dataCompletion_ = new IOUSBCompletion;
		dataCompletion_->target = this;
		dataCompletion_->action = &::dataCompleted;
		dataCompletion_->parameter = 0;
		
		dataUpto_ = 0;

		IOLog( "A800USB2::fill(go): completed setup, about to queue data transactions\n" );
		
		numFramesReceived_ = 0;
		numFramesConsumed_ = 0;
		numFramesPartial_ = 0;

		for (int i = 0; i < READS_TO_QUEUE; i++)
		{
			IOReturn err = dataInPipe_->Read(
				dataBuffers_[i], 1000, 1000, frameSize*FRAMES_PER_READ, dataCompletion_ );
			if (err != kIOReturnSuccess)
			{
				IOLog( "A800USB2::fill(go): got error 0x%08X from Read\n", err );
				break;
			}
		}
		
		//fifoControl
		// ask our devices about it
		for (uint32 i = 0; i < nubs_.size(); i++)
		{
			DVBFrontend * fe = DVBFrontend::checkClient( nubs_[i] );
			if (fe == NULL) continue;
			DIB3000 *d3 = (DIB3000 *)fe;
			d3->fifoControl( 1 );
		}

		IOLog( "A800USB2::fill(go): about give the go signal\n" );
		A800SetStreamingControl go;
		go.set( );
		if (!this->command( go ))
		{
			IOLog( "A800USB2::fill(go): didn't like the go signal :(\n" );
		}
		
		A800SetIOCtl go2;
		go2.set( 1 );
		if (!this->command( go2 ))
		{
			IOLog( "A800USB2::fill(go2): didn't like the go2 signal :(\n" );
		}
		
		IOLog( "A800USB2::fill(go): now going\n" );

		going_ = true;
	}
	else if ((!go) && going_)
	{
		going_ = false;
		
		//fifoControl
		// ask our devices about it
		for (uint32 i = 0; i < nubs_.size(); i++)
		{
			DVBFrontend * fe = DVBFrontend::checkClient( nubs_[i] );
			if (fe == NULL) continue;
			DIB3000 *d3 = (DIB3000 *)fe;
			d3->fifoControl( 0 );
		}
		
		// stop going
		A800SetIOCtl whoa;
		whoa.set( 0 );
		if (!this->command( whoa ))
		{
			IOLog( "A800USB2::fill(stop): didn't like the stop signal :(\n" );
			// continue nevertheless
		}

		int frameSize = dataBuffers_[0]->getLength()/FRAMES_PER_READ;

		IOLog( "A800USB2::fill(stop): about to abort current transactions\n" );

		dataInPipe_->Abort();
		
		// the outstanding calls had better have finished by now!
		IOSleep( 100 );	// just in case for now
		
		IOLog( "A800USB2::fill(stop): about to return resources\n" );
		
		delete dataCompletion_;
		dataCompletion_ = NULL;
		for (int i = 0; i < READS_TO_QUEUE; i++)
			dataBuffers_[i]->release();
		delete [] dataBuffers_;
		dataBuffers_ = NULL;
		IOFree( dataMem_, frameSize*FRAMES_PER_READ*READS_TO_QUEUE );
		dataMem_ = NULL;
		
		IOLog( "A800USB2::fill(stop): stopped.\n" );
		
		IOLog( "A800USB2::fill(stop): frames received %d consumed %d partial %d\n",
			numFramesReceived_, numFramesConsumed_, numFramesPartial_ );
	}

	return true;
}

static void dataCompleted( void * target, void * parameter, IOReturn status, UInt32 bufferSizeRemaining )
{
	A800USB2 * pDevice = (A800USB2*)target;

	// only restart if the previous read succeeded
	if (status != kIOReturnSuccess && status != kIOReturnUnderrun)
	{
		// complain unless we were intentionally stopped
		if (status != kIOReturnAborted)
		{
			IOLog( "A800USB2 dataCompleted: got error status 0x%08X\n", status );
		}
		
		pDevice->dataCompleted( false, 0 );
	}
	else
	{
		// call the device to restart the transfer		
		pDevice->dataCompleted( true, bufferSizeRemaining );
	}
};


void A800USB2::dataCompleted( bool ok, UInt32 remaining )
{
	if (!ok)
	{
		dataUpto_++;
		// but we're pretty much stuffed from here as we'll soon lose track of the buffer
		// (could clear buffer ptr and incr dataUpto_ while ptr bad...)
		return;
	}
	
	// find out which one completed
	int ourIndex = dataUpto_ & (READS_TO_QUEUE-1);
	dataUpto_++;
	if (ourIndex < 0 || ourIndex >= READS_TO_QUEUE)
	{
		IOLog( "A800USB2::dataCompleted: bad index %d\n", ourIndex );
		return;
	}
	
	numFramesReceived_++;
	
	// see how much we got
	int readSize = dataBuffers_[0]->getLength();
	int len = readSize - remaining;
	if (len > 0)
	{
		if (remaining != 0) numFramesPartial_++;
		numFramesConsumed_++;
	
		// copy all the data to the user buffer
		uint8 * source = dataMem_ + ourIndex * readSize;
		uint8 * dest = buffer_ + *level_;
		while (len > 0)
		{
			int more = len;
			if (dest + more > buffer_ + USER_BUFFER_SIZE)
				more = buffer_ + USER_BUFFER_SIZE - dest;
			
			memcpy( dest, source, more );
			
			len -= more;
			source += more;
			dest += more;
			if (dest >= buffer_ + USER_BUFFER_SIZE)
				dest = buffer_;
		}
		*level_ = dest - buffer_;
	}

	// and kick off the read again
	IOReturn err = dataInPipe_->Read(
		dataBuffers_[ourIndex], 1000, 1000, readSize, dataCompletion_ );
	if (err != kIOReturnSuccess)
	{
		IOLog( "A800USB2::dataCompleted: got error 0x%08X from Read\n", err );
	}
}


void A800USB2::situation( UInt32 & progress, UInt32 & strength, UInt32 & quality )
{
	progress = 0;
	strength = 0;
	quality = 0;
	
	if (buffer_ == NULL) return;

	// ask our devices about it
	for (uint32 i = 0; i < nubs_.size(); i++)
	{
		DVBFrontend * fe = DVBFrontend::checkClient( nubs_[i] );
		if (fe == NULL) continue;
		fe->situation( progress, strength, quality );
	}
}

void A800USB2::statistics( UInt32 & blobsTotal, UInt32 & blobsErrors )
{
	blobsTotal = 0;
	blobsErrors = 0;

	if (buffer_ == NULL) return;

	// ask our devices about it
	for (uint32 i = 0; i < nubs_.size(); i++)
	{
		DVBFrontend * fe = DVBFrontend::checkClient( nubs_[i] );
		if (fe == NULL) continue;
		fe->statistics( blobsTotal, blobsErrors );
	}
}




/**
 *	Helper method to find pipes. Needs to be called after alt setting changed.
 */
bool A800USB2::findPipes( uint8 altSet )
{
	uint8 dataPipeEP = 6; //(altSet == 1) ? 4 : 2;

	dataInPipe_ = NULL;
	for (UInt8 pipeIdx = 0; pipeIdx < kUSBMaxPipes; pipeIdx++)
	{
		IOUSBPipe * apipe = providerNub_->GetPipeObj( pipeIdx );
		if (apipe == NULL) break;
		if (apipe->GetEndpointNumber() == 1 && apipe->GetDirection() == kUSBIn)
			bulkInPipe_ = apipe;
		if (apipe->GetEndpointNumber() == 1 && apipe->GetDirection() == kUSBOut)
			bulkOutPipe_ = apipe;
		if (apipe->GetEndpointNumber() == dataPipeEP && apipe->GetDirection() == kUSBIn)
			dataInPipe_ = apipe;
	}
	if (bulkInPipe_ == NULL || bulkOutPipe_ == NULL || dataInPipe_ == NULL)
	{
		IOLog( "\tcould not get all three pipes: bi %p bo %p di %p\n",
			bulkInPipe_, bulkOutPipe_, dataInPipe_ );
		return false;
	}
	else
	{
		IOLog( "\tsuccessfully got all three pipes\n" );
	}
	IOLog( "\tbulk in pipe : ep %d type %d dir %d maxpack %d interval %d status 0x%08X\n",
		bulkInPipe_->GetEndpointNumber(),
		bulkInPipe_->GetType(),				// 0 control 1 isoc 2 bulk 3 interrupt
		bulkInPipe_->GetDirection(),		// 0 out, 1 in, 2 none, 3 any
		bulkInPipe_->GetMaxPacketSize(),
		bulkInPipe_->GetInterval(),
		bulkInPipe_->GetPipeStatus() );		// 0 happy, 0xE000404F stalled
	IOLog( "\tbulk out pipe: ep %d type %d dir %d maxpack %d interval %d status 0x%08X\n",
		bulkOutPipe_->GetEndpointNumber(),
		bulkOutPipe_->GetType(),
		bulkOutPipe_->GetDirection(),
		bulkOutPipe_->GetMaxPacketSize(),
		bulkOutPipe_->GetInterval(),
		bulkOutPipe_->GetPipeStatus() );
	if (dataInPipe_ != NULL)
	{
		IOLog( "\tdata in pipe : ep %d type %d dir %d maxpack %d interval %d status 0x%08X\n",
			dataInPipe_->GetEndpointNumber(),
			dataInPipe_->GetType(),
			dataInPipe_->GetDirection(),
			dataInPipe_->GetMaxPacketSize(),
			dataInPipe_->GetInterval(),
			dataInPipe_->GetPipeStatus() );
	}
	if (bulkInPipe_->GetType() != kUSBBulk ||
		bulkOutPipe_->GetType() != kUSBBulk ||
		(dataInPipe_ != NULL && dataInPipe_->GetType() != kUSBBulk))
	{	// consider checking speed too...
		IOLog( "\t... but pipes are wrong types\n" );
		return false;
	}

	return true;
}


bool A800USB2::initHardware()
{
	// first get it to identify itself
	//A800Identify idCmd;
	IOUSBDevice * pDev = providerNub_->GetDevice();
	if (pDev->GetProductID()==43008)
	{
		// if it can't do that then it mustn't have its firmware yet
		IOLog( "A800USB2::initHardware: uploading firmware...\n" );
		
		// open file
		UDMResourceFile * firmwareFile = UDMResourceFile::withResource( "dvb-usb-avertv-a800-02.fw" );
		UInt64 firmwareFileSize = firmwareFile->size();
		if (firmwareFileSize == ~0ULL)
		{
			firmwareFile->fini();
			IOLog( "A800USB2::initHardware: "
				"Could not find firmware file.\n"
				"\tCopy dvb-usb-avertv-a800-02.fw into Contents/Resources "
				"inside the A800USB2.kext bundle\n" );
			return false;
		}

		// figure out where firmware is based on size of file
		uint firmwareOffset = 0;
		uint firmwareSize = 0;
		if (firmwareFileSize == 10757)
		{
			firmwareOffset = 0x0;
			firmwareSize = 10757;
		}
		else
		{
			IOLog( "A800USB2::initHardware: "
				"Size of dll containing firmware not recognised: %d.\n",
				int(firmwareFileSize) );
			firmwareFile->fini();
			return false;
		}

		// read it in
		uint8 * firmwareData = new uint8[firmwareSize+4];
		if (!firmwareFile->read( firmwareOffset, firmwareData, firmwareSize, false ))
		{
			IOLog( "A800USB2::initHardware: "
				"Error reading firmware data.\n" );
			firmwareFile->fini();
			delete [] firmwareData;
			return false;
		}
		firmwareFile->fini();
		
		memcpy( &firmware.data, firmwareData, firmwareSize );

		// and now get to the uploading

		IOUSBDevRequest req;
		uint8			buf[64];
		IOReturn		err;

		// put the FX2 into reset
		req.bmRequestType = USBmakebmRequestType( kUSBOut, kUSBVendor, kUSBDevice );
		req.bRequest = 0xA0;
		req.wValue = 0xE600;
		req.wIndex = 0;
		req.wLength = 1;
		req.pData = buf;
		req.wLenDone = 0;
		buf[0] = 1;	// reset
		err = providerNub_->DeviceRequest( &req );
		if (err != kIOReturnSuccess)
		{
			IOLog( "A800USB2::initHardware: couldn't put FX2 into reset: 0x%08X\n", err );
			delete [] firmwareData;
			return NULL;
		}
		
		int ret, pos, cum=0;
		struct hexline hx;
		pos = 0;
		
		firmware.length = firmwareSize;
		while ((ret = this->getFirmwareHexLine( &hx, &pos )) > 0) {
			IOLog( "A800USB2::initHardware: Pos: %d\n", pos );
			
			req.bmRequestType = USBmakebmRequestType( kUSBOut, kUSBVendor, kUSBDevice );
			req.bRequest = 0xA0;	// Firmware Load
			req.wValue = hx.addr;
			req.wIndex = 0;
			req.wLength = hx.len;
			req.pData = hx.data;
			req.wLenDone = 0;
			
			cum += hx.len;
	
			// send the request
			err = providerNub_->DeviceRequest( &req );
			if (err != kIOReturnSuccess)
			{
				IOLog( "A800USB2::initHardware: error 0x%08X uploading firmware at byte %d\n", err, pos );
				delete [] firmwareData;
				return NULL;
			}
		}
		
		IOLog( "A800USB2::initHardware: cumulative length sent: %d\n", cum );

		// take the FX2 out of reset
		req.bmRequestType = USBmakebmRequestType( kUSBOut, kUSBVendor, kUSBDevice );
		req.bRequest = 0xA0;
		req.wValue = 0xE600;
		req.wIndex = 0;
		req.wLength = 1;
		req.pData = buf;
		req.wLenDone = 0;
		buf[0] = 0;	// !reset
		err = providerNub_->DeviceRequest( &req );
		if (err != kIOReturnSuccess)
		{
			IOLog( "A800USB2::initHardware: couldn't take FX2 out of reset: 0x%08X\n", err );
			delete [] firmwareData;
			return NULL;
		}
		
		delete [] firmwareData;
		
		IOLog( "A800USB2::initHardware: "
			"successfully uploaded %d bytes of firmware\n", firmwareSize );

		// it will now re-enumerate itself automatically
		dataInPipe_ = NULL;		// make sure we don't continue however
		return true;
	}

	if (!this->findPipes( 0 ))
		return false;

	return true;
}

void A800USB2::finiHardware()
{
	// nothing done here for now
}

int A800USB2::getFirmwareHexLine( struct hexline *hx, int *pos )
{
	uint8 *b = (uint8 *) &firmware.data[*pos];
	int data_offs = 4;
	if (*pos >= firmware.length)
		return 0;

	memset(hx,0,sizeof(struct hexline));

	hx->len  = b[0];

	if ((*pos + hx->len + 4) >= firmware.length)
		return -1;

	hx->addr = (b[1]) | (b[2]<<8);
	hx->type = b[3];

	if (hx->type == 0x04) {
		/* b[4] and b[5] are the Extended linear address record data field */
		hx->addr |= (b[4] << 24) | (b[5] << 16);
	}
	memcpy(hx->data,&b[data_offs],hx->len);
	hx->chk = b[hx->len + data_offs];

	*pos += hx->len + 5;

	return *pos;
}




/**
 *	Send and execute a command on the usb device.
 *	The result is always retrieved unless cmd.ilen is 0.
 */
bool A800USB2::command( A800USBCommand & cmd )
{
	if (cmd.olen > MAX_COMMAND_SIZE || cmd.ilen > MAX_COMMAND_SIZE)
	{
		IOLog( "A800USB2::command(%d): Command olen or ilen exceeded MAX_COMMAND_SIZE\n",
			cmd.command );
		return false;	// too big!
	}

	IOReturn err;
	
	// build the command
	memcpy( cmdAddr_, &cmd.command, cmd.olen );
	memset( cmdAddr_+cmd.olen, 0, MAX_COMMAND_SIZE-cmd.olen );

	// send the request
	err = bulkOutPipe_->Write( cmdDesc_, 1000, 2000, cmd.olen, NULL );
	if (err != kIOReturnSuccess)
	{
		IOLog( "A800USB2::command(%d): Error 0x%08X sending on bulk pipe\n",
			cmd.command, err );
		return false;
	}
	
	//IOLog( "A800USB2::command: successfully wrote %d bytes\n", cmd.olen );
	
	// if no reply is expected then return now
	// (this device does not send 0-byte replies it seems)
	if (cmd.ilen <= 0) return true;

	// get the reply
	IOByteCount got = 0;
	err = bulkInPipe_->Read( cmdDesc_, 1000, 2000, MAX_COMMAND_SIZE, (IOUSBCompletion*)NULL, &got );
	if (err != kIOReturnSuccess)
	{
		IOLog( "A800USB2::command(%d): Error 0x%08X receiving on bulk pipe got %d\n",
			cmd.command, err, int(got) );
		return false;
	}
	
	//IOLog( "A800USB2::command: successfully read %d bytes\n", (int)got );
	
	if (got > MAX_COMMAND_SIZE)
	{
		IOLog( "A800USB2::command: "
			"Reply is bigger than our buffer! (12 bytes)\n" );
		return false;
	}
	if (got < cmd.ilen)
	{
		IOLog( "A800USB2::command: "
			"Reply is smaller (%d) than expected (%d)\n", (int)got, cmd.ilen );
		return false;
	}
	// just discard extra bytes if the reply was bigger than expected
	if (got > cmd.ilen)
	{
		IOLog( "A800USB2::command: "
			"Reply is bigger (%d) than expected (%d) [this is ok]\n", (int)got, cmd.ilen );
	}

	memcpy( 1+&cmd.command, cmdAddr_, cmd.ilen );
	return true;
}












/**
 *	Constructor
 */
A800USBI2CPipe::A800USBI2CPipe( A800USB2 * pOwner ) :
	I2CPipe( 100000 ),
	owner_( *pOwner )
{
}

/**
 *	Destructor
 */
A800USBI2CPipe::~A800USBI2CPipe()
{
}



/**
 *	Attempt to send all the messages in the pipe
 */
int A800USBI2CPipe::flushAttempt( int opTimeoutInMS )
{
	uint i;
	for (i = 0; i < messages_.size(); i++)
	{
		Message & m = messages_[i];

		// we only support reads if they immediately follow a write
		if (m.readNotWrite)
		{
			IOLog( "A800USBI2CPipe::flush: Can only read immediately after a write\n" );
			break;
		}
		// make sure they are not bigger than can be handled (limited by USB pipe size)
		if (m.dataLen > 8)
		{
			IOLog( "A800USBI2CPipe::flush: Maximum write length is 8 bytes (not %d)\n",
				m.dataLen );
			break;
		}

		// see if this is a write followed by a read and pack it into one command
		if (!(i+1 < messages_.size() && messages_[i+1].addr == m.addr &&
			messages_[i+1].readNotWrite))
		{
			// only writing
			A800I2CWrite	cmd;
			cmd.set( m.addr, m.dataPtr, m.dataLen );
			if (!owner_.command( cmd ))
			{
				IOLog( "A800USBI2CPipe::flush: Error delivering i2c write over USB\n" );
				break;
			}
			if (!cmd.ok())
			{
				IOLog( "A800USBI2CPipe::flush: Write returned %d instead of '0'\n", cmd.addr );
				break;
			}

			// otherwise all good then
		}
		else
		{
			i++;
			Message & m2 = messages_[i];

			if (m2.dataLen > 11)
			{
				IOLog( "A800USBI2CPipe::flush: Maximum read length is 11 bytes (not %d)\n",
					m2.dataLen );
				break;
			}

			// writing and reading
			A800I2CWriteRead	cmd;
			cmd.set( m.addr, m.dataPtr, m.dataLen, m2.dataLen );
			if (!owner_.command( cmd ))
			{
				IOLog( "A800USBI2CPipe::flush: Error delivering i2c writeThenRead over USB\n" );
				break;
			}
			if (!cmd.ok())
			{
				//if(!suppressErrors_)
					IOLog( "A800USBI2CPipe::flush: Read returned %d instead of '0'\n", cmd.addr );
				break;
			}
			
			// otherwise the result is there then
			cmd.get( m2.dataPtr );
		}
	}

	return (i == messages_.size()) ? 0 : -1;
}

int A800USBI2CPipe::readRegister( uint8 addr, uint16 reg )
{
	uint8 wb[] = { ((reg >> 8) | 0x80) & 0xff, reg & 0xff };
	uint8 rb[2] = {0xff, 0xff};
	
	this->append( addr, false, &wb, 2 );
	this->append( addr, true, &rb, 2 );
	
	if (this->flush() == 2)
	{
		//IOLog( "A800USBI2CPipe::readRegister: reg %d = 0x%02X\n", int(reg), int((rb[0] << 8) | rb[1]) );
		return (rb[0] << 8) | rb[1];
	}
	
	IOLog( "A800USBI2CPipe::readRegister: flush() didn't return 2. Our data ended up being 0x%02X\n", int((rb[0] << 8) | rb[1]) );
	return 0;
}

int A800USBI2CPipe::writeRegister( uint8 addr, uint16 reg, uint16 val )
{
	uint8 b[] = {
		(reg >> 8) & 0xff, reg & 0xff,
		(val >> 8) & 0xff, val & 0xff,
	};
	
	this->append( addr, false, &b, 4 );
	
	if (this->flush() == 1)
	{
		if ( reg == 0 ) {
		//IOLog( "A800USBI2CPipe::writeRegister: reg %d = 0x%02X\n", int(reg), int(val) );
		IOSleep( 20 );
		}
		return val;
	}
	
	IOLog( "A800USBI2CPipe::writeRegister: flush() didn't return 1.\n" );
	return -1;
}

#define rd(reg) this->readRegister( addr, reg )

/**
 *	Method to scan one addr
 */
bool A800USBI2CPipe::scanOne( uint8 addr )
{
	uint8 readZero = 0;
	int devid;
	
	if ( addr < 0x8 || addr >= 0xd )
		return false;
	
	devid = rd(DIB3000_REG_MANUFACTOR_ID);
	if (devid != DIB3000_I2C_ID_DIBCOM)
	{
		IOLog( "A800USBI2CPipe::scanOne: manu=%d got=%d\n", int(DIB3000_I2C_ID_DIBCOM), devid );
		return false;
	}

	devid = rd(DIB3000_REG_DEVICE_ID);
	if (devid != DIB3000MC_DEVICE_ID && devid != DIB3000P_DEVICE_ID)
	{
		IOLog( "A800USBI2CPipe::scanOne: dev=%d got=%d\n", int(DIB3000MC_DEVICE_ID), devid );
		return false;
	}
	
	IOLog( "A800USBI2CPipe::scanOne: Found DIB3000 at address: %d\n", int(addr) );
	return true;
	
	this->append( addr, /*readNotWrite:*/false, &readZero, 1 );	// hmmm
	this->append( addr, /*readNotWrite:*/true , &readZero, 1 );
	//return this->flush() == 2;
	if (this->flush() == 2)
	{
		IOLog( "A800USBI2CPipe::scanOne: "
			"Got response from addr %d: 0x%02X\n", int(addr), int(readZero) );
		return true;
	}
	return false;
}



