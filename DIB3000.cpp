/*
 * DIB3000.cpp: Drivers for the DIB3000.
 *
 * ** Currently works only for the AverTV A800. For porting to other devices,
 *  the readRegister and writeRegister functions need to be changed to pass on
 *  the signals to the right place (currently assumes an A800USBI2CPipe) **
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

#include "DIB3000.h"

#include <libkern/c++/OSMetaClass.h>

#include "A800USBI2CPipe.h"

OSDefineMetaClassAndStructors( DIB3000, DVBFrontend )

int DIB3000::pid_parse(int onoff)
{
	u16 tmp = rd(DIB3000MC_REG_SMO_MODE);
	IOLog("%s pid parsing\n",onoff ? "enabling" : "disabling");

	if (onoff) {
		wr(DIB3000MC_REG_SMO_MODE,tmp | DIB3000MC_SMO_MODE_PID_PARSE);
	} else {
		wr(DIB3000MC_REG_SMO_MODE,tmp & DIB3000MC_SMO_MODE_NO_PID_PARSE);
	}
	return 0;
}

int DIB3000::pid_control(int index, int pid,int onoff)
{
	pid = (onoff ? pid | DIB3000_ACTIVATE_PID_FILTERING : 0);
	wr(index+DIB3000MC_REG_FIRST_PID,pid);
	return 0;
}

/*
	.type			= FE_OFDM,
	.frequency_min		= 44250000,
	.frequency_max		= 867250000,
	.frequency_stepsize	= 62500,
	.caps = FE_CAN_INVERSION_AUTO |
			FE_CAN_FEC_1_2 | FE_CAN_FEC_2_3 | FE_CAN_FEC_3_4 |
			FE_CAN_FEC_5_6 | FE_CAN_FEC_7_8 | FE_CAN_FEC_AUTO |
			FE_CAN_QPSK | FE_CAN_QAM_16 | FE_CAN_QAM_64 | FE_CAN_QAM_AUTO |
			FE_CAN_TRANSMISSION_MODE_AUTO |
			FE_CAN_GUARD_INTERVAL_AUTO |
			FE_CAN_RECOVER |
			FE_CAN_HIERARCHY_AUTO,
*/

bool DIB3000::init( OSDictionary * pDict )
{
	ownPowerOn_ = false;
	activated_ = false;
	init_done = 0;
	as_done = 0;
	
	pll_addr_ = 0x60;
	demod_address = 0x0a;
	
	return this->IOService::init( pDict );
}

int DIB3000::readRegister( uint16 reg )
{
	A800USBI2CPipe *ai2cpipe = (A800USBI2CPipe *)pNub->pipe_;
	
	return ai2cpipe->readRegister( demod_address, reg );
}

int DIB3000::writeRegister( uint16 reg, uint16 val )
{
	A800USBI2CPipe *ai2cpipe = (A800USBI2CPipe *)pNub->pipe_;
	
	return ai2cpipe->writeRegister( demod_address, reg, val );
}

IOService * DIB3000::probe( IOService * provider, SInt32 * score )
{
	IOLog( "DIB3000::probe:\n" );
	pNub = OSDynamicCast( I2CNub, provider );
	if (pNub == NULL) return NULL;
	
	demod_address = pNub->address();
	
	uint16 manuId = rd( DIB3000_REG_MANUFACTOR_ID );
	uint16 deviceId = rd( DIB3000_REG_DEVICE_ID );
	if ( manuId == DIB3000_I2C_ID_DIBCOM && (deviceId == DIB3000MC_DEVICE_ID || deviceId == DIB3000P_DEVICE_ID ) )
	{
		IOLog( "DIB3000::probe: successful with chip: 0x%02X\n", int(deviceId) );
	}
	else
	{
		IOLog( "DIB3000::probe: failed since unknown chipID 0x%02X\n", int(deviceId) );
		return false;
	}

	return this->IOService::probe( provider, score );
}

bool DIB3000::start( IOService * provider )
{
	bool ok = this->IOService::start( provider );
	if (!ok) return false;

	pNub = OSDynamicCast( I2CNub, provider );
	if (pNub == NULL) return false;
	
	demod_address = pNub->address();
	
	uint16 manuId = rd( DIB3000_REG_MANUFACTOR_ID );
	uint16 deviceId = rd( DIB3000_REG_DEVICE_ID );
	if ( manuId == DIB3000_I2C_ID_DIBCOM && (deviceId == DIB3000MC_DEVICE_ID || deviceId == DIB3000P_DEVICE_ID ) )
	{
		IOLog( "DIB3000::start: successful with chip: 0x%02X\n", int(deviceId) );
	}
	else
	{
		IOLog( "DIB3000::start: failed since unknown chipID 0x%02X\n", int(deviceId) );
		return false;
	}

	// set up our state
	ownPowerOn_ = true;
	activated_ = false;
	
	constellation_ = QAM_AUTO;
	transmode_ = TRANSMISSION_MODE_AUTO;
	guardi_ = GUARD_INTERVAL_AUTO;
	hierarchyi_ = HIERARCHY_AUTO;
	inversion_ = INVERSION_AUTO;
	
	u16 default_addr = 0x0a;
	
	deb_info("DIB3000::start: initializing the demod the first time. Setting demod addr to 0x%x\n",default_addr);
	wr(DIB3000MC_REG_ELEC_OUT,DIB3000MC_ELEC_OUT_DIV_OUT_ON);
	wr(DIB3000MC_REG_OUTMODE,DIB3000MC_OM_PAR_CONT_CLK);

	wr(DIB3000MC_REG_RST_I2C_ADDR,
		DIB3000MC_DEMOD_ADDR(default_addr) |
		DIB3000MC_DEMOD_ADDR_ON);

	IOLog( "DIB3000::start: demod_address=0x%x, will now be 0x0a\n", demod_address );
	demod_address = 0x0a;
	manuId = rd( DIB3000_REG_MANUFACTOR_ID );
	IOLog( "DIB3000::start: manuId=0x%x, which shouldn\'t really be 0x%x because we\'re not talking to the DIB3000 anymore!\n", manuId, DIB3000_I2C_ID_DIBCOM );

	wr(DIB3000MC_REG_RST_I2C_ADDR,
		DIB3000MC_DEMOD_ADDR(default_addr));

	IOLog( "DIB3000::start: started\n" );
	
	/*
	// FIXME: activate now just for testing :)
	this->activate( );
	
	OSDictionary *d;
	const OSObject *val[2];
	const OSSymbol *key[2];
	
	key[0] = OSSymbol::withCString("BandwidthHz");
	val[0] = OSNumber::withNumber((unsigned long long) 7000000, 32);
	key[1] = OSSymbol::withCString("FrequencyHz");
	val[1] = OSNumber::withNumber((unsigned long long) 219500000, 32);

	d = OSDictionary::withObjects( val, key, 2 );
	
	this->tune( d );
	
	IOLog( "DIB3000::start ran tuning!\n" );
	*/
	return true;
}

void DIB3000::stop( IOService * provider )
{
	IOLog( "DIB3000::stop\n" );

	// disable tuner
	if (ownPowerOn_ && activated_)
	{
		this->deactivate();
	}

	this->IOService::stop( provider );
}


IOReturn DIB3000::message( UInt32 type, IOService * provider, void * argument )
{
	// we don't understand any special messages
	return this->IOService::message( type, provider, argument );
}


IOReturn DIB3000::setPowerState( unsigned long powerStateOrdinal, IOService * )
{
	bool powerGoingOn = powerStateOrdinal != 0;
	if (!powerGoingOn && ownPowerOn_)
	{
		IOLog( "DIB3000::setPowerState to %d\n", int(powerGoingOn) );

		if (activated_)
		{
			this->saveState();
			this->deactivate();
			activated_ = true;
		}
		
		ownPowerOn_ = false;
	}
	else if (powerGoingOn && !ownPowerOn_)
	{
		IOLog( "DIB3000::setPowerState to %d\n", int(powerGoingOn) );

		ownPowerOn_ = true;

		if (activated_)
		{
			activated_ = false;
			this->activate();
			this->loadState();
		}
	}
	return 0;
}


bool DIB3000::activate()
{
	activated_ = true;
	lastBandwidth_ = 0;
	
	IOLog( "DIB3000::activate: Beginning the activation sequence...\n" );

	wr(DIB3000MC_REG_RESTART,DIB3000MC_RESTART_CONFIG);
	wr(DIB3000MC_REG_RESTART,DIB3000MC_RESTART_OFF);
	wr(DIB3000MC_REG_CLK_CFG_1,DIB3000MC_CLK_CFG_1_POWER_UP);
	wr(DIB3000MC_REG_CLK_CFG_2,DIB3000MC_CLK_CFG_2_PUP_MOBILE);
	wr(DIB3000MC_REG_CLK_CFG_3,DIB3000MC_CLK_CFG_3_POWER_UP);
	wr(DIB3000MC_REG_CLK_CFG_7,DIB3000MC_CLK_CFG_7_INIT);

	wr(DIB3000MC_REG_RST_UNC,DIB3000MC_RST_UNC_OFF);
	wr(DIB3000MC_REG_UNK_19,DIB3000MC_UNK_19);

	wr(33,5);
	wr(36,81);
	wr(DIB3000MC_REG_UNK_88,DIB3000MC_UNK_88);

	wr(DIB3000MC_REG_UNK_99,DIB3000MC_UNK_99);
	wr(DIB3000MC_REG_UNK_111,DIB3000MC_UNK_111_PH_N_MODE_0); /* phase noise algo off */
	
	IOLog( "DIB3000::activate: First lot done, now for another bunch...\n" );

	/* mobile mode - portable reception */
	wr_foreach(dib3000mc_reg_mobile_mode,dib3000mc_mobile_mode[1]);

/* TUNER_PANASONIC_ENV57H12D5: */
	wr_foreach(dib3000mc_reg_agc_bandwidth,dib3000mc_agc_bandwidth);
	wr_foreach(dib3000mc_reg_agc_bandwidth_general,dib3000mc_agc_bandwidth_general);
	wr_foreach(dib3000mc_reg_agc,dib3000mc_agc_tuner[1]);

	wr(DIB3000MC_REG_UNK_110,DIB3000MC_UNK_110);
	wr(26,0x6680);
	wr(DIB3000MC_REG_UNK_1,DIB3000MC_UNK_1);
	wr(DIB3000MC_REG_UNK_2,DIB3000MC_UNK_2);
	wr(DIB3000MC_REG_UNK_3,DIB3000MC_UNK_3);
	wr(DIB3000MC_REG_SEQ_TPS,DIB3000MC_SEQ_TPS_DEFAULT);

	wr_foreach(dib3000mc_reg_bandwidth,dib3000mc_bandwidth_8mhz);
	wr_foreach(dib3000mc_reg_bandwidth_general,dib3000mc_bandwidth_general);

	wr(DIB3000MC_REG_UNK_4,DIB3000MC_UNK_4);

	wr(DIB3000MC_REG_SET_DDS_FREQ_MSB,DIB3000MC_DDS_FREQ_MSB_INV_OFF);
	wr(DIB3000MC_REG_SET_DDS_FREQ_LSB,DIB3000MC_DDS_FREQ_LSB);
	
	IOLog( "DIB3000::activate: Great. Now to call setTiming...\n" );

	this->setTiming( TRANSMISSION_MODE_8K,BANDWIDTH_8_MHZ );
//	dib3000mc_set_timing(state,0,TRANSMISSION_MODE_8K,BANDWIDTH_8_MHZ);
//	wr_foreach(dib3000mc_reg_timing_freq,dib3000mc_timing_freq[3]);

	wr(DIB3000MC_REG_UNK_120,DIB3000MC_UNK_120);
	wr(DIB3000MC_REG_UNK_134,DIB3000MC_UNK_134);
	wr(DIB3000MC_REG_FEC_CFG,DIB3000MC_FEC_CFG);

	wr(DIB3000MC_REG_DIVERSITY3,DIB3000MC_DIVERSITY3_IN_OFF);

	this->setImpulseNoise( TRANSMISSION_MODE_8K,BANDWIDTH_8_MHZ );
//	dib3000mc_set_impulse_noise(state,0,TRANSMISSION_MODE_8K,BANDWIDTH_8_MHZ);

/* output mode control, just the MPEG2_SLAVE */
//	set_or(DIB3000MC_REG_OUTMODE,DIB3000MC_OM_SLAVE);
	wr(DIB3000MC_REG_OUTMODE,DIB3000MC_OM_SLAVE);
	wr(DIB3000MC_REG_SMO_MODE,DIB3000MC_SMO_MODE_SLAVE);
	wr(DIB3000MC_REG_FIFO_THRESHOLD,DIB3000MC_FIFO_THRESHOLD_SLAVE);
	wr(DIB3000MC_REG_ELEC_OUT,DIB3000MC_ELEC_OUT_SLAVE);

/* MPEG2_PARALLEL_CONTINUOUS_CLOCK
	wr(DIB3000MC_REG_OUTMODE,
		DIB3000MC_SET_OUTMODE(DIB3000MC_OM_PAR_CONT_CLK,
			rd(DIB3000MC_REG_OUTMODE)));

	wr(DIB3000MC_REG_SMO_MODE,
			DIB3000MC_SMO_MODE_DEFAULT |
			DIB3000MC_SMO_MODE_188);

	wr(DIB3000MC_REG_FIFO_THRESHOLD,DIB3000MC_FIFO_THRESHOLD_DEFAULT);
	wr(DIB3000MC_REG_ELEC_OUT,DIB3000MC_ELEC_OUT_DIV_OUT_ON);
*/

/* diversity */
	wr(DIB3000MC_REG_DIVERSITY1,DIB3000MC_DIVERSITY1_DEFAULT);
	wr(DIB3000MC_REG_DIVERSITY2,DIB3000MC_DIVERSITY2_DEFAULT);

	set_and(DIB3000MC_REG_DIVERSITY3,DIB3000MC_DIVERSITY3_IN_OFF);

	set_or(DIB3000MC_REG_CLK_CFG_7,DIB3000MC_CLK_CFG_7_DIV_IN_OFF);

	// and we're done
	IOLog( "DIB3000::activate: initialised frontend\n" );
	return true;
}

/* for auto search */
u16 dib3000_seq[2][2][2] =     /* fft,gua,   inv   */
	{ /* fft */
		{ /* gua */
			{ 0, 1 },                   /*  0   0   { 0,1 } */
			{ 3, 9 },                   /*  0   1   { 0,1 } */
		},
		{
			{ 2, 5 },                   /*  1   0   { 0,1 } */
			{ 6, 11 },                  /*  1   1   { 0,1 } */
		}
	};

int DIB3000::setGeneralConfig( int *auto_val )
{
	fe_code_rate_t fe_cr = FEC_NONE;
	u8 fft=0, guard=0, qam=0, alpha=0, sel_hp=0, cr=0, hrch=0;
	int seq;
	
	switch (transmode_) {
		case TRANSMISSION_MODE_2K: fft = DIB3000_TRANSMISSION_MODE_2K; break;
		case TRANSMISSION_MODE_8K: fft = DIB3000_TRANSMISSION_MODE_8K; break;
		case TRANSMISSION_MODE_AUTO: break;
		default: return -EINVAL;
	}
	switch (guardi_) {
		case GUARD_INTERVAL_1_32: guard = DIB3000_GUARD_TIME_1_32; break;
		case GUARD_INTERVAL_1_16: guard = DIB3000_GUARD_TIME_1_16; break;
		case GUARD_INTERVAL_1_8:  guard = DIB3000_GUARD_TIME_1_8; break;
		case GUARD_INTERVAL_1_4:  guard = DIB3000_GUARD_TIME_1_4; break;
		case GUARD_INTERVAL_AUTO: break;
		default: return -EINVAL;
	}
	switch (constellation_) {
		case QPSK:   qam = DIB3000_CONSTELLATION_QPSK; break;
		case QAM_16: qam = DIB3000_CONSTELLATION_16QAM; break;
		case QAM_64: qam = DIB3000_CONSTELLATION_64QAM; break;
		case QAM_AUTO: break;
		default: return -EINVAL;
	}
	switch (hierarchyi_) {
		case HIERARCHY_NONE: // fall through
		case HIERARCHY_1: alpha = DIB3000_ALPHA_1; break;
		case HIERARCHY_2: alpha = DIB3000_ALPHA_2; break;
		case HIERARCHY_4: alpha = DIB3000_ALPHA_4; break;
		case HIERARCHY_AUTO: break;
		default: return -EINVAL;
	}
	if (hierarchyi_ == HIERARCHY_NONE) {
		hrch   = DIB3000_HRCH_OFF;
		sel_hp = DIB3000_SELECT_HP;
		fe_cr  = FEC_2_3;
	} else if (hierarchyi_ != HIERARCHY_AUTO) {
		hrch   = DIB3000_HRCH_ON;
		sel_hp = DIB3000_SELECT_LP;
		fe_cr  = FEC_2_3;
	}
	
	switch (fe_cr) {
		case FEC_1_2: cr = DIB3000_FEC_1_2; break;
		case FEC_2_3: cr = DIB3000_FEC_2_3; break;
		case FEC_3_4: cr = DIB3000_FEC_3_4; break;
		case FEC_5_6: cr = DIB3000_FEC_5_6; break;
		case FEC_7_8: cr = DIB3000_FEC_7_8; break;
		case FEC_NONE: break;
		case FEC_AUTO: break;
		default: return -EINVAL;
	}
	
	IOLog( "Writing DIB3000MC_REG_DEMOD_PARM register with: %d,%d,%d,%d\n", alpha,qam,guard,fft );
	
	wr(DIB3000MC_REG_DEMOD_PARM,DIB3000MC_DEMOD_PARM(alpha,qam,guard,fft));
	rd(DIB3000MC_REG_DEMOD_PARM);
	wr(DIB3000MC_REG_HRCH_PARM,DIB3000MC_HRCH_PARM(sel_hp,cr,hrch));

	wr(DIB3000MC_REG_SET_DDS_FREQ_MSB,DIB3000MC_DDS_FREQ_MSB_INV_ON);
	
	switch (inversion_) {
		case INVERSION_OFF:
			wr(DIB3000MC_REG_SET_DDS_FREQ_MSB,DIB3000MC_DDS_FREQ_MSB_INV_OFF);
			break;
		case INVERSION_AUTO: // fall through
		case INVERSION_ON:
			wr(DIB3000MC_REG_SET_DDS_FREQ_MSB,DIB3000MC_DDS_FREQ_MSB_INV_ON);
			break;
		default:
			return -EINVAL;
	}

	seq = dib3000_seq
		[transmode_ == TRANSMISSION_MODE_AUTO]
		[guardi_ == GUARD_INTERVAL_AUTO]
		[inversion_ == INVERSION_AUTO];

	IOLog("seq? %d\n", seq);
	wr(DIB3000MC_REG_SEQ_TPS,DIB3000MC_SEQ_TPS(seq,1));
	*auto_val = constellation_ == QAM_AUTO ||
			hierarchyi_ == HIERARCHY_AUTO ||
		 	guardi_ == GUARD_INTERVAL_AUTO ||
			transmode_ == TRANSMISSION_MODE_AUTO ||
			fe_cr == FEC_AUTO ||
			inversion_ == INVERSION_AUTO;
	return 0;
}

void DIB3000::setTiming( uint32 fft, uint32 bw )
{
	u16 timf_msb,timf_lsb;
	s32 tim_offset,tim_sgn;
	u64 comp1,comp2,comp=0;

	switch (bw) {
		case BANDWIDTH_8_MHZ: comp = DIB3000MC_CLOCK_REF*8; break;
		case BANDWIDTH_7_MHZ: comp = DIB3000MC_CLOCK_REF*7; break;
		case BANDWIDTH_6_MHZ: comp = DIB3000MC_CLOCK_REF*6; break;
		default: err("unknown bandwidth (%d)",bw); break;
	}
	timf_msb = (comp >> 16) & 0xff;
	timf_lsb = (comp & 0xffff);

	// Update the timing offset ;
	/*if (upd_offset > 0) {
		if (!state->timing_offset_comp_done) {
			msleep(200);
			state->timing_offset_comp_done = 1;
		}
		tim_offset = rd(DIB3000MC_REG_TIMING_OFFS_MSB);
		if ((tim_offset & 0x2000) == 0x2000)
			tim_offset |= 0xC000;
		if (fft == TRANSMISSION_MODE_2K)
			tim_offset <<= 2;
		state->timing_offset += tim_offset;
	}*/
	tim_offset = 0;
	tim_sgn = 0;
	/*
	tim_offset = state->timing_offset;
	if (tim_offset < 0) {
		tim_sgn = 1;
		tim_offset = -tim_offset;
	} else
		tim_sgn = 0;
	*/
	comp1 =  (u32)tim_offset * (u32)timf_lsb ;
	comp2 =  (u32)tim_offset * (u32)timf_msb ;
	comp  = ((comp1 >> 16) + comp2) >> 7;

	if (tim_sgn == 0)
		comp = (u32)(timf_msb << 16) + (u32) timf_lsb + comp;
	else
		comp = (u32)(timf_msb << 16) + (u32) timf_lsb - comp ;

	timf_msb = (comp >> 16) & 0xff;
	timf_lsb = comp & 0xffff;

	wr(DIB3000MC_REG_TIMING_FREQ_MSB,timf_msb);
	wr(DIB3000MC_REG_TIMING_FREQ_LSB,timf_lsb);
	
}

void DIB3000::setImpulseNoise( uint32 fft, uint32 bw )
{
	switch (fft) {
		case TRANSMISSION_MODE_2K:
			wr_foreach(dib3000mc_reg_fft,dib3000mc_fft_modes[0]);
			break;
		case TRANSMISSION_MODE_8K:
			wr_foreach(dib3000mc_reg_fft,dib3000mc_fft_modes[1]);
			break;
		default:
			break;
	}

	switch (bw) {
/*		case BANDWIDTH_5_MHZ:
			wr_foreach(dib3000mc_reg_impulse_noise,dib3000mc_impluse_noise[0]);
			break; */
		case BANDWIDTH_6_MHZ:
			wr_foreach(dib3000mc_reg_impulse_noise,dib3000mc_impluse_noise[1]);
			break;
		case BANDWIDTH_7_MHZ:
			wr_foreach(dib3000mc_reg_impulse_noise,dib3000mc_impluse_noise[2]);
			break;
		case BANDWIDTH_8_MHZ:
			wr_foreach(dib3000mc_reg_impulse_noise,dib3000mc_impluse_noise[3]);
			break;
		default:
			break;
	}

	switch ( 0 ) {
		case 0: /* no impulse */ /* fall through */
			wr_foreach(dib3000mc_reg_imp_noise_ctl,dib3000mc_imp_noise_ctl[0]);
			break;
		case 1: /* new algo */
			wr_foreach(dib3000mc_reg_imp_noise_ctl,dib3000mc_imp_noise_ctl[1]);
			set_or(DIB3000MC_REG_IMP_NOISE_55,DIB3000MC_IMP_NEW_ALGO(0)); /* gives 1<<10 */
			break;
		default: /* old algo */
			wr_foreach(dib3000mc_reg_imp_noise_ctl,dib3000mc_imp_noise_ctl[3]);
			break;
	}
}

void DIB3000::deactivate()
{
	activated_ = false;
}

int DIB3000::setAdaptorConfig(fe_modulation_t con)
{
	switch (con) {
		case QAM_64:
			wr_foreach(dib3000mc_reg_adp_cfg,dib3000mc_adp_cfg[2]);
			break;
		case QAM_16:
			wr_foreach(dib3000mc_reg_adp_cfg,dib3000mc_adp_cfg[1]);
			break;
		case QPSK:
			wr_foreach(dib3000mc_reg_adp_cfg,dib3000mc_adp_cfg[0]);
			break;
		case QAM_AUTO:
			break;
		default:
			warn("unkown constellation.");
			break;
	}
	return 0;
}

int dib3000_search_status(u16 irq,u16 lock)
{
	IOLog( "Autosearch status = %d\n", irq );
	if (irq & 0x02) {
		if (lock & 0x01) {
			deb_srch("auto search succeeded\n");
			return 1; // auto search succeeded
		} else {
			deb_srch("auto search not successful\n");
			return 0; // auto search failed
		}
	} else if (irq & 0x01)  {
		deb_srch("auto search failed\n");
		return 0; // auto search failed
	}
	return -1; // try again
}

bool DIB3000::tune( OSDictionary * params )
{
	IOLog( "DIB3000::tune: starting......\n" );
	
	lastFreq_ = frequency_;
	
	if ( as_done == 1 ) {
		int auto_val;
		u16 val;
		auto_val = 0;
		//dib3000mc_set_general_cfg(state,fep,&auto_val);
		this->setGeneralConfig(&auto_val);
		if (auto_val)
			deb_info("auto_val is true, even though an auto search was already performed.\n");

		//dib3000mc_set_impulse_noise(state,0,ofdm->constellation,ofdm->bandwidth);
		this->setImpulseNoise( constellation_,bandwidth_ );

		val = rd(DIB3000MC_REG_DEMOD_PARM);
		wr(DIB3000MC_REG_DEMOD_PARM,val | DIB3000MC_DEMOD_RST_AUTO_SRCH_ON);
		wr(DIB3000MC_REG_DEMOD_PARM,val);

		msleep(30);

		wr(DIB3000MC_REG_ISI,DIB3000MC_ISI_DEFAULT|DIB3000MC_ISI_ACTIVATE);
		
		this->setAdaptorConfig( constellation_ );
		
		wr_foreach(dib3000mc_reg_offset,
				dib3000mc_offset[(transmode_ == TRANSMISSION_MODE_8K)+1]);
		
		as_done = 0;
		
	//	this->fifoControl( 1 );
		
		return true;
	}
	
	/*** HACK ***/
	this->pid_parse( 0 );
	this->pid_control( 0, 0, 0 );
	/*** ***/
	
	constellation_ = QAM_AUTO;
	transmode_ = TRANSMISSION_MODE_AUTO;
	guardi_ = GUARD_INTERVAL_AUTO;
	hierarchyi_ = HIERARCHY_AUTO;
	inversion_ = INVERSION_AUTO;
	
	// set the bandwidth
	OSNumber * bwOb;
	if ((bwOb = OSDynamicCast( OSNumber, params->getObject( "BandwidthHz" ) )) != NULL)
	{
		lastBandwidth_ = bwOb->unsigned32BitValue() / 1000000;
		bandwidth_ = lastBandwidth_;
	}
	
	OSNumber * freqOb;
	if ((freqOb = OSDynamicCast( OSNumber, params->getObject( "FrequencyHz" ) )) != NULL)
	{
		frequency_ = freqOb->unsigned32BitValue();
		if ( lastFreq_ != frequency_ )
			as_done = 0; // if frequency changed, start again.
		//this->setFrequency( frequency_ );
	}
	
	if ( lastFreq_ != frequency_ )
		this->setFrequency( frequency_ );
	
	lastFreq_ = frequency_;
	
	/*
	bandwidth_ = 7;
	frequency_ = lastFreq_ = 226500000;
	this->setFrequency( frequency_ );
	*/
	IOLog( "Bandwidth is: %d\n", bandwidth_ );
	IOLog( "Frequency is: %d\n", frequency_ );
	
	
	
	
	
	this->setTiming( transmode_, bandwidth_ );
	this->initAutoScan( bandwidth_, 0 );
	
	wr_foreach(dib3000mc_reg_agc_bandwidth,dib3000mc_agc_bandwidth);
	wr(DIB3000MC_REG_RESTART,DIB3000MC_RESTART_AGC);
	wr(DIB3000MC_REG_RESTART,DIB3000MC_RESTART_OFF);

	/* Default cfg isi offset adp */
	wr_foreach(dib3000mc_reg_offset,dib3000mc_offset[0]);

	wr(DIB3000MC_REG_ISI,DIB3000MC_ISI_DEFAULT | DIB3000MC_ISI_INHIBIT);
//	dib3000mc_set_adp_cfg(state,ofdm->constellation);
	this->setAdaptorConfig( constellation_ );
	wr(DIB3000MC_REG_UNK_133,DIB3000MC_UNK_133);
	
	wr_foreach(dib3000mc_reg_bandwidth_general,dib3000mc_bandwidth_general);
	/* power smoothing */
	if (bandwidth_ != BANDWIDTH_8_MHZ) {
		wr_foreach(dib3000mc_reg_bw,dib3000mc_bw[0]);
	} else {
		wr_foreach(dib3000mc_reg_bw,dib3000mc_bw[3]);
	}
	int auto_val;
	auto_val = 0;
	this->setGeneralConfig(&auto_val);
	this->setImpulseNoise( constellation_,bandwidth_ );

	uint16 val;
	val = rd(DIB3000MC_REG_DEMOD_PARM);
	wr(DIB3000MC_REG_DEMOD_PARM,val|DIB3000MC_DEMOD_RST_DEMOD_ON);
	wr(DIB3000MC_REG_DEMOD_PARM,val);
	
	msleep(70);
	
	/* something has to be auto searched */
	if (auto_val) {
		int as_count=0;
		int search_state;

		deb_setf("autosearch enabled.\n");

		val = rd(DIB3000MC_REG_DEMOD_PARM);
		wr(DIB3000MC_REG_DEMOD_PARM,val | DIB3000MC_DEMOD_RST_AUTO_SRCH_ON);
		wr(DIB3000MC_REG_DEMOD_PARM,val);

		while ((search_state = dib3000_search_status(
					rd(DIB3000MC_REG_AS_IRQ),1)) < 0 && as_count++ < 100)
			msleep(10);

		deb_info("search_state after autosearch %d after %d checks\n",search_state,as_count);

		if (search_state == 1) {
			as_done = 1;
			this->getFrontend( );
			return this->tune( params );
		}
		
		return false;
	} else {
		this->setImpulseNoise( constellation_,bandwidth_ );
		wr(DIB3000MC_REG_ISI,DIB3000MC_ISI_DEFAULT|DIB3000MC_ISI_ACTIVATE);
//		dib3000mc_set_adp_cfg(state,ofdm->constellation);
		this->setAdaptorConfig( constellation_ );

		/* set_offset_cfg */
		wr_foreach(dib3000mc_reg_offset,
				dib3000mc_offset[(transmode_ == TRANSMISSION_MODE_8K)+1]);
	}
	
//	this->fifoControl( 1 );
	this->getFrontend( );
/*
	OSNumber * freqOb;
	if ((freqOb = OSDynamicCast( OSNumber, params->getObject( "FrequencyHz" ) )) != NULL)
	{
		this->setFrequency( freqOb->unsigned32BitValue() );
	}
*/	
	return true;
}

int DIB3000::getFrontend( )
{
	u16 tps_val,cr_val;
	int inv_test1,inv_test2;
	u32 dds_val, threshold = 0x1000000;
	char tmp[1024] = "";

	if (!(rd(DIB3000MC_REG_LOCK_507) & DIB3000MC_LOCK_507))
		return 0;

	dds_val = (rd(DIB3000MC_REG_DDS_FREQ_MSB) << 16) + rd(DIB3000MC_REG_DDS_FREQ_LSB);
	deb_getf("DDS_FREQ: %6x\n",dds_val);
	if (dds_val < threshold)
		inv_test1 = 0;
	else if (dds_val == threshold)
		inv_test1 = 1;
	else
		inv_test1 = 2;

	dds_val = (rd(DIB3000MC_REG_SET_DDS_FREQ_MSB) << 16) + rd(DIB3000MC_REG_SET_DDS_FREQ_LSB);
	deb_getf("DDS_SET_FREQ: %6x\n",dds_val);
	if (dds_val < threshold)
		inv_test2 = 0;
	else if (dds_val == threshold)
		inv_test2 = 1;
	else
		inv_test2 = 2;

	inversion_ =
		((inv_test2 == 2) && (inv_test1==1 || inv_test1==0)) ||
		((inv_test2 == 0) && (inv_test1==1 || inv_test1==2)) ?
		INVERSION_ON : INVERSION_OFF;

	deb_getf("inversion %d %d, %d\n", inv_test2, inv_test1, inversion_);

	bandwidth_ = lastBandwidth_;

	tps_val = rd(DIB3000MC_REG_TUNING_PARM);

	switch (DIB3000MC_TP_QAM(tps_val)) {
		case DIB3000_CONSTELLATION_QPSK:
			strcat( tmp, "QPSK ");
			constellation_ = QPSK;
			break;
		case DIB3000_CONSTELLATION_16QAM:
			strcat( tmp, "QAM16 ");
			constellation_ = QAM_16;
			break;
		case DIB3000_CONSTELLATION_64QAM:
			strcat( tmp, "QAM64 ");
			constellation_ = QAM_64;
			break;
		default:
			err("Unexpected constellation returned by TPS (%d)", tps_val);
			break;
	}

	if (DIB3000MC_TP_HRCH(tps_val)) {
		strcat( tmp, "HRCH ON ");
		coderate_ = FEC_NONE;
		switch (DIB3000MC_TP_ALPHA(tps_val)) {
			case DIB3000_ALPHA_0:
				strcat( tmp, "HIERARCHY_NONE ");
				hierarchyi_ = HIERARCHY_NONE;
				break;
			case DIB3000_ALPHA_1:
				strcat( tmp, "HIERARCHY_1 ");
				hierarchyi_ = HIERARCHY_1;
				break;
			case DIB3000_ALPHA_2:
				strcat( tmp, "HIERARCHY_2 ");
				hierarchyi_ = HIERARCHY_2;
				break;
			case DIB3000_ALPHA_4:
				strcat( tmp, "HIERARCHY_4 ");
				hierarchyi_ = HIERARCHY_4;
				break;
			default:
				err("Unexpected ALPHA value returned by TPS (%d)", tps_val);
				break;
		}
		cr_val = DIB3000MC_TP_FEC_CR_LP(tps_val);
	} else {
		strcat( tmp, "HRCH OFF ");
		coderate_ = FEC_NONE;
		hierarchyi_ = HIERARCHY_NONE;
		cr_val = DIB3000MC_TP_FEC_CR_HP(tps_val);
	}

	switch (cr_val) {
		case DIB3000_FEC_1_2:
			strcat( tmp, "FEC_1_2 ");
			coderate_ = FEC_1_2;
			break;
		case DIB3000_FEC_2_3:
			strcat( tmp, "FEC_2_3 ");
			coderate_ = FEC_2_3;
			break;
		case DIB3000_FEC_3_4:
			strcat( tmp, "FEC_3_4 ");
			coderate_ = FEC_3_4;
			break;
		case DIB3000_FEC_5_6:
			strcat( tmp, "FEC_5_6 ");
			coderate_ = FEC_4_5;
			break;
		case DIB3000_FEC_7_8:
			strcat( tmp, "FEC_7_8 ");
			coderate_ = FEC_7_8;
			break;
		default:
			err("Unexpected FEC returned by TPS (%d)", tps_val);
			break;
	}

	switch (DIB3000MC_TP_GUARD(tps_val)) {
		case DIB3000_GUARD_TIME_1_32:
			strcat( tmp, "GUARD_INTERVAL_1_32 ");
			guardi_ = GUARD_INTERVAL_1_32;
			break;
		case DIB3000_GUARD_TIME_1_16:
			strcat( tmp, "GUARD_INTERVAL_1_16 ");
			guardi_ = GUARD_INTERVAL_1_16;
			break;
		case DIB3000_GUARD_TIME_1_8:
			strcat( tmp, "GUARD_INTERVAL_1_8 ");
			guardi_ = GUARD_INTERVAL_1_8;
			break;
		case DIB3000_GUARD_TIME_1_4:
			strcat( tmp, "GUARD_INTERVAL_1_4 ");
			guardi_ = GUARD_INTERVAL_1_4;
			break;
		default:
			err("Unexpected Guard Time returned by TPS (%d)", tps_val);
			break;
	}

	switch (DIB3000MC_TP_FFT(tps_val)) {
		case DIB3000_TRANSMISSION_MODE_2K:
			strcat( tmp, "TRANSMISSION_MODE_2K ");
			transmode_ = TRANSMISSION_MODE_2K;
			break;
		case DIB3000_TRANSMISSION_MODE_8K:
			strcat( tmp, "TRANSMISSION_MODE_8K ");
			transmode_ = TRANSMISSION_MODE_8K;
			break;
		default:
			err("unexpected transmission mode return by TPS (%d)", tps_val);
			break;
	}
	strcat( tmp, "\n");
	
	IOLog( "DIB3000::getFrontend: %s", tmp );

	return 0;
}

int DIB3000::initAutoScan(int bw, int boost)
{
	if (boost) {
		wr(DIB3000MC_REG_SCAN_BOOST,DIB3000MC_SCAN_BOOST_ON);
	} else {
		wr(DIB3000MC_REG_SCAN_BOOST,DIB3000MC_SCAN_BOOST_OFF);
	}
	IOLog( "DIB3000::initAutoScan: Bandwidth %dmhz\n", bw );
	switch (bw) {
		case BANDWIDTH_8_MHZ:
			wr_foreach(dib3000mc_reg_bandwidth,dib3000mc_bandwidth_8mhz);
			break;
		case BANDWIDTH_7_MHZ:
			wr_foreach(dib3000mc_reg_bandwidth,dib3000mc_bandwidth_7mhz);
			break;
		case BANDWIDTH_6_MHZ:
			wr_foreach(dib3000mc_reg_bandwidth,dib3000mc_bandwidth_6mhz);
			break;
/*		case BANDWIDTH_5_MHZ:
			wr_foreach(dib3000mc_reg_bandwidth,dib3000mc_bandwidth_5mhz);
			break;*/
		case BANDWIDTH_AUTO:
			return -EOPNOTSUPP;
		default:
			err("unknown bandwidth value (%d).",bw);
			return -EINVAL;
	}
	if (boost) {
		u32 timeout = (rd(DIB3000MC_REG_BW_TIMOUT_MSB) << 16) +
			rd(DIB3000MC_REG_BW_TIMOUT_LSB);
		timeout *= 85; timeout >>= 7;
		wr(DIB3000MC_REG_BW_TIMOUT_MSB,(timeout >> 16) & 0xffff);
		wr(DIB3000MC_REG_BW_TIMOUT_LSB,timeout & 0xffff);
	}
	return 0;
}

struct dvb_pll_desc {
	char *name;
	u32  min;
	u32  max;
	int  count;
	struct {
		u32 limit;
		u32 offset;
		u32 stepsize;
		u8  config;
		u8  cb;
	} entries[12];
};

struct dvb_pll_desc dvb_pll_env57h1xd5 = {
	"Panasonic ENV57H1XD5",
	44250000,
	858000000,
	4,
	{
		{  153000000, 36291666, 166666, 0xc2, 0x41 },
		{  470000000, 36291666, 166666, 0xc2, 0x42 },
		{  526000000, 36291666, 166666, 0xc2, 0x84 },
		{  999999999, 36291666, 166666, 0xc2, 0xa4 },
	},
};

void DIB3000::setFrequency( uint32 freq )
{
	struct dvb_pll_desc *desc = &dvb_pll_env57h1xd5;
	uint32 bandwidth = lastBandwidth_;

	if (freq < 44250000 || freq > 867250000)
	{
		IOLog( "DIB3000::setFrequency: freq %dHz out of range\n", int(freq) );
		return;
	}
	
	/* freq, lastBandwidth_ */
	
	int i;
	u32 div;
	u8 buf[4];
	
	for ( i = 0; i < desc->count; i++ ) {
		if ( freq > desc->entries[i].limit )
			continue;
		break;
	}
	
	IOLog("pll: %s: freq=%d bw=%d | i=%d/%d\n",
			desc->name, freq, bandwidth, i, desc->count);
	
	if ( i == desc->count ) {
		IOLog( "DIB3000::setFrequency: i == count !! oops!\n", int(freq) );
		return;
	}
	
	div = (freq + desc->entries[i].offset) / desc->entries[i].stepsize;
	buf[0] = (div >> 8) & 0xff;
	buf[1] = div & 0xff;
	buf[2] = desc->entries[i].config;
	buf[3] = desc->entries[i].cb;
	
	IOLog("pll: %s: div=%d | buf=0x%02x,0x%02x,0x%02x,0x%02x\n",
		       desc->name, div, buf[0], buf[1], buf[2], buf[3]);
	
	//this->fifoControl( 0 );
	this->tunerPassControl( 1 );
	
	pNub->pipe_->queueWriteAndFlush( pll_addr_, &buf, 4 );
	
	msleep(1);
	
	this->tunerPassControl( 0 );
}

void DIB3000::situation( UInt32 & progress, UInt32 & strength, UInt32 & quality )
{
	u16 val = rd(DIB3000MC_REG_SIGNAL_NOISE_LSB);
	strength = (((val >> 6) & 0xff) << 8) + (val & 0x3f);
	strength = strength << 15;
	quality = strength;
}

void DIB3000::statistics( UInt32 & blobsTotal, UInt32 & blobsErrors )
{
	blobsErrors = rd(DIB3000MC_REG_PACKET_ERRORS);
}

int DIB3000::tunerPassControl( int onoff )
{
	IOLog( "tunerPassControl: %d\n", onoff );
	if (onoff) {
		wr(DIB3000MC_REG_TUNER, DIB3000_TUNER_WRITE_ENABLE(pll_addr_));
	} else {
		wr(DIB3000MC_REG_TUNER, DIB3000_TUNER_WRITE_DISABLE(pll_addr_));
	}
	return 0;
}

int DIB3000::fifoControl(int onoff)
{
	u16 tmp = rd(DIB3000MC_REG_SMO_MODE);

	IOLog("%s fifo\n",onoff ? "enabling" : "disabling");

	if (onoff) {
		IOLog("%d %x\n",tmp & DIB3000MC_SMO_MODE_FIFO_UNFLUSH,tmp & DIB3000MC_SMO_MODE_FIFO_UNFLUSH);
		wr(DIB3000MC_REG_SMO_MODE,tmp & DIB3000MC_SMO_MODE_FIFO_UNFLUSH);
	} else {
		IOLog("%d %x\n",tmp | DIB3000MC_SMO_MODE_FIFO_FLUSH,tmp | DIB3000MC_SMO_MODE_FIFO_FLUSH);
		wr(DIB3000MC_REG_SMO_MODE,tmp | DIB3000MC_SMO_MODE_FIFO_FLUSH);
	}
	
	return 0;
}


/**
 *	Save our current state so it can be loaded when power is restored
 */
void DIB3000::saveState()
{
}

/**
 *	Load our current state since power has just been restored
 */
void DIB3000::loadState()
{

}


