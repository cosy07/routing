//2017-07-17
// ToInDatagram.h
#ifndef ToInDatagram_h
#define ToInDatagram_h

#include <to_cc1120_STM1.h>
#include <ToDatagram_STM.h>

class  ToInDatagram
{
public:
	ToInDatagram(TO_ELECHOUSE_CC1120& driver, uint8_t masterNumber, uint8_t thisAddress);//실제 16bit 주소
	void	init();
	void 	init(byte ch);
	void 	SetReceive(void);


	void 	setThisAddress(uint8_t thisAddress);
	void	setHeaderFrom(uint8_t address);
	void 	setHeaderTo(uint8_t address);
	void 	setHeaderMaster(uint8_t master);
	void 	setHeaderType(uint8_t type);
	void	setNumOfSlave(uint8_t num);

	uint8_t        headerFrom();
	uint8_t        headerTo();
	uint8_t        headerMaster();
	uint8_t        headerType();

	uint8_t		   getThisAddress();
	uint8_t		   getMasterNumber();

	bool           available();

	byte           recvData(uint8_t* buf);

	void send(uint8_t from, uint8_t to, uint8_t master, uint8_t type, uint8_t* temp_buf, uint8_t size);

	bool sendToWaitAck(uint8_t from, uint8_t to, uint8_t master, uint8_t type, uint8_t* temp_buf, uint8_t size, unsigned long time = 2000);

	void printRecvPacketHeader();

	uint8_t ch;

	byte temp_buf[20];

private:
	TO_ELECHOUSE_CC1120&   _driver;	   /// The Driver we are to use
	uint8_t         	_thisAddress;		/// The address of this node
	uint8_t  			_rxHeaderFrom;
	uint8_t  			_rxHeaderTo;
	uint8_t				_rxHeaderMaster;
	uint8_t				_rxHeaderType;

	uint8_t gatewayNumber = 0;
	uint8_t masterNumber = 0;
	uint8_t num_of_slave = 0;

	unsigned long	sendingTime = -60000;

};

#endif