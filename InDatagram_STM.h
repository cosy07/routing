//2017-07-17
// InDatagram.h
#ifndef InDatagram_h
#define InDatagram_h

#include <INCC1120_STM1.h>

/// the default retry timeout in milliseconds
#define DEFAULT_TIMEOUT 100


/// The default number of retries
#define DEFAULT_RETRIES 3//나중에 5로 수정3
#define MAX_master_num				31		//0 ~ 30, 31 is broadcast

// The default size of the routing table we keep
#define ROUTING_TABLE_SIZE 5  //////////////////////////////////////////////////////////////////////////////////////////////////////////수정
#define ROUTER_MAX_MESSAGE_LEN 48
#define R_MASTER_SEND_NUM 5
#define R_GATEWAY_SEND_NUM 10

//===============================================================
//	2017-04-26	ver.1
/****************************************************************
#define
****************************************************************/
#define REQUEST_BROADCAST				0
#define REQUEST_TYPE					1
#define REQUEST_ACK_TYPE				2
#define R2_REQUEST_TYPE					3
#define R2_REQUEST_REAL_TYPE			4
#define R2_REQUEST_ACK_TYPE				5
#define REQUEST_MULTI_HOP				6
#define REQUEST_MULTI_HOP_ACK			7
#define REQUEST_DIRECT					8
#define REQUEST_DIRECT_ACK				9
#define NEW_NODE_REGISTER				10

#define CHECK_ROUTING					11
#define CHECK_ROUTING_ACK				12
#define CHECK_ROUTING_ACK_REROUTING		13
#define ACK								14
#define NACK							15
#define ROUTING_TABLE_UPDATE			16

#define SCAN_REQUEST_TO_MASTER			17
#define SCAN_REQUEST_ACK_FROM_MASTER	18
#define SCAN_RESPONSE_TO_GATEWAY		19
#define CONTROL_TO_MASTER				20
#define CONTROL_RESPONSE_TO_GATEWAY		21

#define SCAN_REQUEST_TO_RC				22
#define CONTROL_TO_RC					23

#define CONTROL_RESPONSE_BROADCAST		24
#define SCAN_RESPONSE_TO_RC				25

#define SCAN_REQUEST_TO_SLAVE			26
#define SCAN_FINISH_TO_MASTER			27
#define RC_CONTROL_TO_SLAVE				28
#define GW_CONTROL_TO_SLAVE				29

#define INTERNAL_COM_FAIL				30
#define INTERNAL_CONTROL				31
#define INTERNAL_CONTROL_FINISH			32
#define INTERNAL_SCAN					33
#define INTERNAL_SCAN_FINISH			34


#define NONE						0

#define TIME_TERM					1700
#define TIME_HOP					400
#define TIME_CONTROL				5000

//#define NUM_OF_MASTER				17
#define MAX_UN_RECV					2//일단 0으로

class  InDatagram
{
public:
	InDatagram(IN_ELECHOUSE_CC1120& driver, uint8_t masterNumber, uint8_t thisAddress);//실제 16bit 주소
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
	IN_ELECHOUSE_CC1120&   _driver;	   /// The Driver we are to use
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