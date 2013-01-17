/**
 *	The I2C pipe for the A800USB2 device.
 */
class A800USB2;

class A800USBI2CPipe : public I2CPipe
{
public:
	A800USBI2CPipe( A800USB2 * pOwner );
	~A800USBI2CPipe();

	virtual bool reset( uint32 frequency = 0 )	{ return true; }
	virtual uint32 status()						{ return 0; }
	virtual int maxDataLength()					{ return 8; }

private:
	virtual int flushAttempt( int opTimeoutInMS );
	virtual bool scanOne( uint8 addr );

	A800USB2 & owner_;
	
public:
	int readRegister( uint8 addr, uint16 reg );
	int writeRegister( uint8 addr, uint16 reg, uint16 val );
};