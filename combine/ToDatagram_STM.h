//2017-07-17
// ToDatagram.h
#ifndef ToDatagram_h
#define ToDatagram_h

#include <to_cc1120_STM1.h>

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

void RS485_Write_Read(uint8_t *write_buf,uint8_t *read_buf);

class  ToDatagram
{
public:
	ToDatagram(TO_ELECHOUSE_CC1120& driver, uint8_t gatewayNumber, uint8_t thisAddress = 0);// , uint8_t master_num);//실제 16bit 주소
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
	uint8_t        headerType();
	uint8_t        headerData();
	uint8_t   	   headerSeqNum();
	uint8_t   	   headerFlags();
	uint8_t		   headerHop();
	uint8_t		   getThisAddress();



	bool           available();


	void           MainSendControlToSlave();
	void           RCSendControlToMain();


	void           sendto(uint8_t* buf, uint8_t len, uint8_t address);
	byte           recvData(uint8_t* buf);


	/// Values for the possible states for routes
	typedef enum
	{
		Invalid = 0,           ///< No valid route is known
		Discovering,           ///< Discovering a route
		Valid                  ///< Route is valid
	} RouteState;

	/// Defines an entry in the routing table
	typedef struct
	{
		int8_t				dest;      ///< Destination node address
		int8_t				next_hop;  ///< Send via this next hop address
		int8_t				state;     ///< State of this route, one of RouteState
		int8_t				hop;
		unsigned long		lastTime;
	} RoutingTableEntry;

	/// \param[in] thisAddress The address to assign to this node. Defaults to 0


	void addRouteTo(uint8_t  dest, uint8_t  next_hop, uint8_t state = Valid, uint8_t hop = 0, unsigned long lastTime = 0);

	RoutingTableEntry* getRouteTo(uint8_t  dest);

	bool deleteRouteTo(uint8_t  dest);

	void clearRoutingTable();

	void printRoutingTable();

	int find_child_node(uint8_t* child_node);

	void FromGatewayToMaster();

	int8_t G_find_1stRow_master();

	int8_t G_find_2ndRow_master(uint8_t*);

	uint8_t G_get_i_row_node_list(uint8_t row_number, uint8_t *node_list);

	int8_t G_find_multihop_node(uint8_t, uint8_t*);

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
	
	void changeNextHop(uint8_t address, uint8_t hopCount);

	void newMaster();

	void checkRoutingTable();

	void deleteRoute(uint8_t index);










	
	RoutingTableEntry    _routes[ROUTING_TABLE_SIZE];

	bool checkReceive[34] = { false };
	uint8_t ch;
	int master_num = 2;

	int8_t parentMaster[34] = { -1 };
	byte unRecvCnt[34] = { 0 };
	byte temp_buf[20];
	byte buffer[20];

private:
	TO_ELECHOUSE_CC1120&   _driver;	   /// The Driver we are to use
	uint8_t         	_thisAddress;		/// The address of this node
	uint8_t  			_rxHeaderTo;
	uint8_t  			_rxHeaderFrom;
	uint8_t  			_rxHeaderSource;
	uint8_t				_rxHeaderDestination;
	uint8_t   			_rxHeaderType;
	uint8_t   			_rxHeaderData;
	uint8_t   			_rxHeaderFlags;
	uint8_t   			_rxHeaderSeqNum;
	uint8_t				_rxHeaderHop;


	uint8_t candidateAddress = 0;
	signed char candidateRSSI = 0;


	//byte temp_buf[20];
	uint8_t receivedType = 0;//To avoid receiving from Master that has same hop

	unsigned long startTime;
	unsigned long resetTime;


	uint8_t masterBroadcastAddress;

	uint8_t gatewayNumber = 0;

	signed long	sendingTime = -60000;

};

#endif