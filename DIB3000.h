/*
 * DIB3000.h: Drivers for the DIB3000.
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

#ifndef DIB3000_HPP
#define DIB3000_HPP

#include <IOKit/IOService.h>

#include "MMInputFamily/DVBFrontend.hpp"
#include "MMInputFamily/I2CNub.hpp"
#define fe_code_rate_t int
#define fe_modulation_t int
#ifndef A800_INCLUDE

#define QAM_64 64
#define QAM_16 16
#define QPSK 2
#define QAM_AUTO 0

#define deb_setf IOLog
#define deb_getf IOLog
#define deb_info IOLog
#define deb_srch IOLog
#define msleep IOSleep

#define EINVAL 1
#define EOPNOTSUPP 2

#define BANDWIDTH_8_MHZ 8
#define BANDWIDTH_7_MHZ 7
#define BANDWIDTH_6_MHZ 6
#define BANDWIDTH_AUTO 0
#define TRANSMISSION_MODE_2K 2
#define TRANSMISSION_MODE_8K 8
#define TRANSMISSION_MODE_AUTO 0
#define GUARD_INTERVAL_1_32 32
#define GUARD_INTERVAL_1_16 16
#define GUARD_INTERVAL_1_8 8
#define GUARD_INTERVAL_1_4 4
#define GUARD_INTERVAL_AUTO 0
#define HIERARCHY_NONE 99
#define HIERARCHY_1 1
#define HIERARCHY_2 2
#define HIERARCHY_4 4
#define HIERARCHY_AUTO 0
#define FEC_1_2 1
#define FEC_2_3 2
#define FEC_3_4 3
#define FEC_4_5 4
#define FEC_5_6 5
#define FEC_7_8 6
#define FEC_NONE 99
#define FEC_AUTO 0
#define INVERSION_OFF 0
#define INVERSION_ON 1
#define INVERSION_AUTO 2
#endif
/**
 *	This class manages the DIB3000 DVB-T demodulator chip.
 */
class DIB3000 : public DVBFrontend
{
	OSDeclareDefaultStructors( DIB3000 )

public:
	virtual bool init( OSDictionary * pDict = NULL );
	virtual IOService * probe( IOService * provider, SInt32 * score );

	virtual bool start( IOService * provider );
	virtual void stop( IOService * provider );
	
	virtual IOReturn message( UInt32 type, IOService * provider, void * argument = 0 );
	
	virtual IOReturn setPowerState( unsigned long powerStateOrdinal, IOService * );


	virtual bool activate();
	virtual void deactivate();

	virtual bool tune( OSDictionary * params );
	
	virtual void situation( UInt32 & progress, UInt32 & strength, UInt32 & quality );
	virtual void statistics( UInt32 & blobsTotal, UInt32 & blobsErrors );

private:
	void setFrequency( uint32 freq );
	void setTiming( uint32 fft, uint32 bw );
	void setImpulseNoise( uint32 fft, uint32 bw );
	void saveState();
	void loadState();

	bool					ownPowerOn_;
	bool					activated_;
	UInt32					lastBandwidth_;
	UInt64					savedState_;
	
	I2CNub *pNub;
	int readRegister( uint16 reg );
	int writeRegister( uint16 reg, uint16 val );
	
	int init_done;
	int as_done;
	
	int constellation_;
	int bandwidth_;
	int transmode_;
	int guardi_;
	int hierarchyi_;
	int inversion_;
	int coderate_;
	int frequency_;
	int lastFreq_;
	
	int tunerPassControl( int onoff );
	int pll_addr_;
	
	int setAdaptorConfig(fe_modulation_t con);
	int initAutoScan(int bw, int boost);
	
	int setGeneralConfig( int *auto_val );
	
	int getFrontend( );
public:
	int fifoControl(int onoff);
	
	int pid_parse(int onoff);
	int pid_control(int index, int pid,int onoff);
	
	uint16 demod_address;
};

#ifndef A800_INCLUDE
#define u8 uint8
#define u16 uint16
#define u32 uint32
#define u64 unsigned long long int

#define err IOLog
#define warn IOLog

#define s8 int8
#define s16 int16
#define s32 int


#define rd(reg) this->readRegister( reg )
#define wr(reg,val) this->writeRegister( reg, val )

#define wr_foreach(a,v) { int i; \
	if (sizeof(a) != sizeof(v)) \
		IOLog("sizeof: %zu %zu is different",sizeof(a),sizeof(v));\
	for (i=0; i < sizeof(a)/sizeof(u16); i++) \
		wr(a[i],v[i]); \
	}

#define set_or(reg,val) wr(reg,rd(reg) | val)

#define set_and(reg,val) wr(reg,rd(reg) & val)

#include "dib3000mc_priv.h"

#define DIB3000_REG_MANUFACTOR_ID		(  1025)
#define DIB3000_I2C_ID_DIBCOM			(0x01b3)

#define DIB3000_REG_DEVICE_ID			(  1026)
#define DIB3000MB_DEVICE_ID				(0x3000)
#define DIB3000MC_DEVICE_ID				(0x3001)
#define DIB3000P_DEVICE_ID				(0x3002)
#endif

#endif // DIB3000_HPP
