/*
 * A800K.h: Drivers for the AVerTV A800 USB2 device
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

#ifndef A800USB2_H
#define A800USB2_H

#include <mach/mach_types.h>
#include <IOKit/IOService.h>
#include <FTypes.hpp>
#include <make_stl_work_in_kernel.hpp>
#include <MMInputFamily/MMinput.hpp>

class IOUSBDevice;
class IOUSBInterface;
class IOUSBPipe;
class IOMemoryDescriptor;
class IOBufferMemoryDescriptor;
class IOUSBCompletion;

class I2CNub;

struct A800USBCommand;
class A800USBI2CPipe;

typedef struct
{
	uint32	length;
	uint8	data[10757];
} firmware_t;

struct hexline {
	uint8 len;
	uint32 addr;
	uint8 type;
	uint8 data[255];
	uint8 chk;
};

class A800USB2 : public MMInput
{
	OSDeclareDefaultStructors( A800USB2 )

public:
	// IOService stuff

	virtual bool init( OSDictionary * pDict = NULL );
	virtual void free();
	
	virtual IOService * probe( IOService * provider, SInt32 * score );
	virtual bool start( IOService * provider );
	virtual void stop( IOService * provider );
	virtual bool didTerminate( IOService * provider, IOOptionBits options, bool * defer );
	
	virtual IOReturn setPowerState( unsigned long powerStateOrdinal, IOService * whatDevice );
	
	virtual IOReturn message( UInt32 type, IOService * provider, void * argument = 0 );

	bool command( A800USBCommand & cmd );

	// MMInput methods

	virtual DataKind dataKind() const;

	virtual MMInput * activate( DataPond & pond );
	virtual void deactivate();

	virtual TuningParams tuningSpace() const;
	virtual bool tune( OSDictionary * params );
	
	virtual bool fill( bool go );
	
	virtual void situation( UInt32 & progress, UInt32 & strength, UInt32 & quality );
	virtual void statistics( UInt32 & blobsTotal, UInt32 & blobsErrors );

	void dataCompleted( bool ok, UInt32 bufferSizeRemaining );
private:

	bool findPipes( uint8 altSet );
	bool uploadFirmware();
	bool initHardware();
	void finiHardware();

	IOUSBDevice		* udevice;
	IOUSBInterface	* providerNub_;
	IOUSBPipe		* bulkInPipe_, * bulkOutPipe_, * dataInPipe_;

	IOMemoryDescriptor * cmdDesc_;
	uint8 * cmdAddr_;

	bool	powerOn_;
	uint8 *	buffer_;
	UInt32	* level_;
	bool	going_;
	
	A800USBI2CPipe *i2cPipe_;
	std::vector<I2CNub*> nubs_;

	IOBufferMemoryDescriptor * bufferDesc_, * levelDesc_;
	
	uint8				* dataMem_;
	IOMemoryDescriptor	**dataBuffers_;
	IOUSBCompletion		* dataCompletion_;
	uint32				dataUpto_;
	
	int		numFramesReceived_;
	int		numFramesConsumed_;
	int		numFramesPartial_;
	
	firmware_t firmware;
	int getFirmwareHexLine( struct hexline *hx, int *pos );
};






/*
 * bulk msg to/from endpoint 0x01
 *
 * general structure:
 * request_byte parameter_bytes
 */

#define DIBUSB_REQ_START_READ			0x00
#define DIBUSB_REQ_START_DEMOD			0x01

/*
 * i2c read
 * bulk write: 0x02 ((7bit i2c_addr << 1) & 0x01) register_bytes length_word
 * bulk read:  byte_buffer (length_word bytes)
 */
#define DIBUSB_REQ_I2C_READ			0x02

/*
 * i2c write
 * bulk write: 0x03 (7bit i2c_addr << 1) register_bytes value_bytes
 */
#define DIBUSB_REQ_I2C_WRITE			0x03

/*
 * polling the value of the remote control
 * bulk write: 0x04
 * bulk read:  byte_buffer (5 bytes)
 */
#define DIBUSB_REQ_POLL_REMOTE       0x04

/* additional status values for Hauppauge Remote Control Protocol */
#define DIBUSB_RC_HAUPPAUGE_KEY_PRESSED	0x01
#define DIBUSB_RC_HAUPPAUGE_KEY_EMPTY	0x03

/* streaming mode:
 * bulk write: 0x05 mode_byte
 *
 * mode_byte is mostly 0x00
 */
#define DIBUSB_REQ_SET_STREAMING_MODE	0x05

/* interrupt the internal read loop, when blocking */
#define DIBUSB_REQ_INTR_READ			0x06

/* io control
 * 0x07 cmd_byte param_bytes
 *
 * param_bytes can be up to 32 bytes
 *
 * cmd_byte function    parameter name
 * 0x00     power mode
 *                      0x00      sleep
 *                      0x01      wakeup
 *
 * 0x01     enable streaming
 * 0x02     disable streaming
 *
 *
 */
#define DIBUSB_REQ_SET_IOCTL			0x07

/* IOCTL commands */

/* change the power mode in firmware */
#define DIBUSB_IOCTL_CMD_POWER_MODE		0x00
#define DIBUSB_IOCTL_POWER_SLEEP			0x00
#define DIBUSB_IOCTL_POWER_WAKEUP			0x01

/* modify streaming of the FX2 */
#define DIBUSB_IOCTL_CMD_ENABLE_STREAM	0x01
#define DIBUSB_IOCTL_CMD_DISABLE_STREAM	0x02






#define DIB3000_REG_MANUFACTOR_ID		(  1025)
#define DIB3000_I2C_ID_DIBCOM			(0x01b3)

#define DIB3000_REG_DEVICE_ID			(  1026)
#define DIB3000MB_DEVICE_ID				(0x3000)
#define DIB3000MC_DEVICE_ID				(0x3001)
#define DIB3000P_DEVICE_ID				(0x3002)




#endif // A800USB2_H
