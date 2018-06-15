//2017-07-17
// Datagram.h
#ifndef Datagram_h
#define Datagram_h

#include <cc1120_STM1.h>

/// the default retry timeout in milliseconds
#define DEFAULT_TIMEOUT 100


/// The default number of retries
#define DEFAULT_RETRIES 3//나중에 5로 수정3

// The default size of the routing table we keep
#define ROUTING_TABLE_SIZE 17  //////////////////////////////////////////////////////////////////////////////////////////////////////////수정
#define ROUTER_MAX_MESSAGE_LEN 48
#define R_MASTER_SEND_NUM 5
#define R_GATEWAY_SEND_NUM 10

//=====================================================================================================
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
#define CHECK_ROUTING					10
#define CHECK_ROUTING_ACK				11
#define ACK								12
#define NACK							13

#define MAX_NUM_OF_MASTER				31		//0 ~ 30, 31 is broadcast

#define SCAN_REQUEST_TO_MASTER			13
#define SCAN_REQUEST_TO_RC_EXTERNAL		14
#define SCAN_REQUEST_TO_SLAVE			15
#define SCAN_RESPONSE_TO_RC				16
#define SCAN_RESPONSE_TO_MASTER			17
#define SCAN_RESPONSE_TO_GATEWAY		18

#define CONTROL_MESSAGE					19		//INSTRUCTION_FROM_GATEWAY
#define SCAN_MESSAGE					20		//SCAN_FROM_GATEWAY
#define SCAN_SLAVE						21
#define SCAN_ACK						22
#define CONTROL_ACK						23		//INSTRUCTION_ACK_FROM_MASTER
#define INSTRUCTION_FROM_RC				24
#define ADD_ROUTE						25
#define MAX_NUM_OF_SLAVE				26
#define SCAN_SLAVE_ACK					27
#define ERROR_MESSAGE					28
#define SCAN_UPDATE						29

#define NONE						0

#define TIME_TERM					1400
#define TIME_HOP					400
#define TIME_CONTROL				5000

#define NUM_OF_MASTER				17
#define MAX_UN_RECV					3//일단 0으로

void RS485_Write_Read(uint8_t *write_buf,uint8_t *read_buf);


class  Datagram
{
public:
	Datagram(ELECHOUSE_CC1120& driver, uint8_t thisAddress = 0);// , uint8_t num_of_master);//실제 16bit 주소
	void	init();
	void 	init(byte ch);
	void 	SetReceive(void);


	void 	setThisAddress(uint8_t thisAddress);
	void 	setHeaderTo(uint8_t to);
	void 	setHeaderFrom(uint8_t from);
	void 	setHeaderSource(uint8_t address);
	void	setHeaderDestination(uint8_t address);
	void 	setHeaderType(uint8_t type);
	void 	setHeaderData(uint8_t data);
	void    setHeaderSeqNum(uint8_t seq);
	void    setHeaderFlags(uint8_t flags);
	void	setHeaderHop(uint8_t hop);

	uint8_t        headerTo();
	uint8_t        headerFrom();
	uint8_t        headerSource();
	uint8_t        headerDestination();
	uint8_t         headerType();
	uint8_t         headerData();
	uint8_t   		headerSeqNum();
	uint8_t   		headerFlags();
	uint8_t			headerHop();
	uint8_t			getThisAddress();



	bool           available();


	void           MainSendControlToSlave();
	void           RCSendControlToMain();


	void           sendto(uint8_t* buf, uint8_t len, uint8_t address);
	byte           recvData(uint8_t* buf);


	/// Values for the possible states for routes
	typedef enum
	{
		Invalid = 0,           ///< No valid route is known
		Discovering,           ///< Discovering a route (not currently used)
		Valid                  ///< Route is valid
	} RouteState;

	/// Defines an entry in the routing table
	typedef struct
	{
		int8_t      dest;      ///< Destination node address
		int8_t      next_hop;  ///< Send via this next hop address
		int8_t      state;     ///< State of this route, one of RouteState
		int8_t		 hop;
	} RoutingTableEntry;

	/// \param[in] thisAddress The address to assign to this node. Defaults to 0


	void addRouteTo(uint8_t  dest, uint8_t  next_hop, uint8_t state = Valid, uint8_t hop = 0);

	RoutingTableEntry* getRouteTo(uint8_t  dest);

	bool deleteRouteTo(uint8_t  dest);

	void clearRoutingTable();

	void printRoutingTable();

	void FromGatewayToMaster();

	int8_t G_find_1stRow_master();

	int8_t G_find_2ndRow_master(uint8_t*);

	uint8_t G_get_i_row_node_list(uint8_t row_number, uint8_t *node_list);

	int8_t G_find_multihop_node(uint8_t, uint8_t*);

	void FromMasterToGateway();

	void M_findCandidateParents();

	void M_find2ndRowMasters();

	void M_masterSendRoutingReply();

	uint8_t convertToAddress(uint8_t gatewayNumber, uint8_t masterNumber, uint8_t slaveNumber);

	uint8_t convertToMasterNumber(uint8_t address);

	byte receive(byte*);

	void send(uint8_t from, uint8_t to, uint8_t src, uint8_t dst, uint8_t type, uint8_t data, uint8_t flags, uint8_t seqNum, uint8_t hop, uint8_t* temp_buf, uint8_t size);

	bool sendToWaitAck(uint8_t from, uint8_t to, uint8_t src, uint8_t dst, uint8_t type, uint8_t data, uint8_t flags, uint8_t seqNum, uint8_t hop, uint8_t* temp_buf, uint8_t size, unsigned long time = 2000);

	void printRecvPacketHeader();

	void printPath();

	void printTree();
	
	bool G_request_path_one_by_one(uint8_t address, byte row_number, uint8_t* node_list, byte number_of_node, uint8_t type);

	bool G_discoverNewPath(uint8_t address, uint8_t row_number);// , bool child = false);

	void G_find_error_node(uint8_t address);

	void getPath(uint8_t address, uint8_t* path);
	
	void changeNextHop(uint8_t address);

protected:

	uint8_t masterBroadcastAddress;

	uint8_t gatewayNumber = 0;

	int	sendingTime = -60000;

	void 	deleteRoute(uint8_t index);

	/// Local routing table
	RoutingTableEntry    _routes[ROUTING_TABLE_SIZE];

	ELECHOUSE_CC1120&   _driver;	   /// The Driver we are to use
	uint8_t         	_thisAddress;		/// The address of this node
	uint8_t  			_rxHeaderTo;
	uint8_t  			_rxHeaderFrom;
	uint8_t  			_rxHeaderSource;
	uint8_t			_rxHeaderDestination;
	uint8_t   			_rxHeaderType;
	uint8_t   			_rxHeaderData;
	uint8_t   			_rxHeaderFlags;
	uint8_t   			_rxHeaderSeqNum;
	uint8_t				_rxHeaderHop;

	uint8_t ch;

	//=================================================================================
	//	2017-04-27 ver.1.1
	bool checkReceive[NUM_OF_MASTER + 1] = { false };
	int8_t parentMaster[NUM_OF_MASTER + 1] = {-1};

	uint8_t candidateAddress = 0;
	signed char candidateRSSI = 0;
	uint8_t candidateHop = 0;

	byte temp_buf[20];
	byte unRecvCnt[NUM_OF_MASTER + 1] = { 0 };
	uint8_t receivedType = 0;//To avoid receiving from Master that has same hop

	unsigned long startTime;
	unsigned long resetTime;
	int sendCnt = 0;
	int recvCnt = 0;
	int reroutingCnt = 0;

	int paperTestCnt = 0;
};

#endif