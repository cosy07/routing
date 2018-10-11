//2017-07-17
// Datagram.h
#ifndef Datagram_h
#define Datagram_h

#include "cc1120.h"
#include <string.h>

#define byte uint8_t
/// the default retry timeout in milliseconds
#define DEFAULT_TIMEOUT 100


/// The default number of retries
#define DEFAULT_RETRIES 3//??? 5? ??3
#define MAX_master_num				31		//0 ~ 30, 31 is broadcast

// The default size of the routing table we keep
#define ROUTING_TABLE_SIZE 5  //////////////////////////////////////////////////////////////////////////////////////////////////////////??
#define ROUTER_MAX_MESSAGE_LEN 48
#define R_MASTER_SEND_NUM 5
#define R_GATEWAY_SEND_NUM 10

//===============================================================
//	2017-04-26	ver.1
/****************************************************************
#define
****************************************************************/
#define REQUEST_BROADCAST							0
#define REQUEST_TYPE									1
#define REQUEST_ACK_TYPE							2
#define R2_REQUEST_TYPE								3
#define R2_REQUEST_REAL_TYPE					4
#define R2_REQUEST_ACK_TYPE						5
#define REQUEST_MULTI_HOP							6
#define REQUEST_MULTI_HOP_ACK					7
#define REQUEST_DIRECT								8	
#define REQUEST_DIRECT_ACK						9
#define NEW_NODE_REGISTER							10

#define CHECK_ROUTING									11
#define CHECK_ROUTING_ACK							12
#define CHECK_ROUTING_ACK_REROUTING		13
#define ACK														14
#define NACK													15
#define ROUTING_TABLE_UPDATE					16

#define SCAN_REQUEST_TO_MASTER				17
#define SCAN_REQUEST_ACK_FROM_MASTER	18
#define SCAN_RESPONSE_TO_GATEWAY			19
#define CONTROL_TO_MASTER							20
#define CONTROL_RESPONSE_TO_GATEWAY		21

#define SCAN_REQUEST_TO_RC						22
#define CONTROL_TO_RC									23		

#define CONTROL_RESPONSE_BROADCAST		24
#define SCAN_RESPONSE_TO_RC						25

#define SCAN_REQUEST_TO_SLAVE					26
#define SCAN_FINISH_TO_MASTER					27
#define RC_CONTROL_TO_SLAVE						28
#define GW_CONTROL_TO_SLAVE						29

#define INTERNAL_COM_FAIL							30
#define INTERNAL_CONTROL							31
#define INTERNAL_CONTROL_FINISH				32
#define INTERNAL_SCAN									33
#define INTERNAL_SCAN_FINISH					34


#define NONE						0

#define TIME_TERM					1700
#define TIME_HOP					400
#define TIME_CONTROL				5000

//#define NUM_OF_MASTER				17
#define MAX_UN_RECV					2//?? 0??


//Datagram(ELECHOUSE_CC1120& driver, uint8_t gatewayNumber, uint8_t thisAddress = 0);// , uint8_t master_num);//?? 16bit ??
//void	DatagramInit(uint8_t gatewayNumber, uint8_t thisAddress);
void 	DGInit(uint8_t gatewayNumber, uint8_t thisAddress, byte ch);
void 	DGSetReceive(void);


void 	DGsetThisAddress(uint8_t thisAddress);
void 	DGsetHeaderTo(uint8_t to);
void 	DGsetHeaderFrom(uint8_t from);
void 	DGsetHeaderSource(uint8_t address);
void	DGsetHeaderDestination(uint8_t address);
void 	DGsetHeaderType(uint8_t type);
void 	DGsetHeaderData(uint8_t data);
void	DGsetHeaderSeqNum(uint8_t seq);
void	DGsetHeaderFlags(uint8_t flags);
void	DGsetHeaderHop(uint8_t hop);

uint8_t        DGheaderTo();
uint8_t        DGheaderFrom();
uint8_t        DGheaderSource();
uint8_t        DGheaderDestination();
uint8_t        DGheaderType();
uint8_t        DGheaderData();
uint8_t   	   DGheaderSeqNum();
uint8_t   	   DGheaderFlags();
uint8_t		   DGheaderHop();
uint8_t		   DGgetThisAddress();



bool           DGavailable();


//void           MainSendControlToSlave();
//void           RCSendControlToMain();


//void           sendto(uint8_t* buf, uint8_t len, uint8_t address);
byte           DGrecvData(uint8_t* buf);


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


//void DGaddRouteTo(uint8_t  dest, uint8_t  next_hop, uint8_t state = Valid, uint8_t hop = 0, unsigned long lastTime = 0);
void DGaddRouteTo(uint8_t  dest, uint8_t  next_hop, uint8_t state, uint8_t hop, unsigned long lastTime);

RoutingTableEntry* DGgetRouteTo(uint8_t  dest);

bool DGdeleteRouteTo(uint8_t  dest);

void DGclearRoutingTable();

void DGprintRoutingTable();

int DGfind_child_node(uint8_t* child_node);

void DGFromGatewayToMaster();

int8_t DGG_find_1stRow_master();

int8_t DGG_find_2ndRow_master(uint8_t*);

uint8_t DGG_get_i_row_node_list(uint8_t row_number, uint8_t *node_list);

int8_t DGG_find_multihop_node(uint8_t, uint8_t*);

void DGM_findCandidateParents();

void DGM_find2ndRowMasters();

void DGM_masterSendRoutingReply();

uint8_t DGconvertToAddress(uint8_t gatewayNumber, uint8_t masterNumber, uint8_t slaveNumber);

uint8_t DGconvertToMasterNumber(uint8_t address);

byte DGreceive(byte*);

void DGsend(uint8_t from, uint8_t to, uint8_t src, uint8_t dst, uint8_t type, uint8_t data, uint8_t flags, uint8_t seqNum, uint8_t hop, uint8_t* temp_buf, uint8_t size);

bool DGsendToWaitAck(uint8_t from, uint8_t to, uint8_t src, uint8_t dst, uint8_t type, uint8_t data, uint8_t flags, uint8_t seqNum, uint8_t hop, uint8_t* temp_buf, uint8_t size, unsigned long time);

void DGprintRecvPacketHeader();

void DGprintPath();

void DGprintTree();

bool DGG_request_path_one_by_one(uint8_t address, byte row_number, uint8_t* node_list, byte number_of_node, uint8_t type);

bool DGG_discoverNewPath(uint8_t address, uint8_t row_number);// , bool child = false);

void DGG_find_error_node(uint8_t address);

void DGgetPath(uint8_t address, uint8_t* path);

void DGchangeNextHop(uint8_t address, uint8_t hopCount);

void DGnewMaster();

void DGcheckRoutingTable();

void DGdeleteRoute(uint8_t index);











extern RoutingTableEntry    _routes[ROUTING_TABLE_SIZE];

extern bool DGcheckReceive[34];// = { false };

static volatile uint8_t DGch;
extern int DGmaster_num;// = 2;

extern int8_t DGparentMaster[34];// = { -1 };
extern byte DGunRecvCnt[34];// = { 0 };
extern byte DGtemp_buf[20];
extern byte DGbuffer[20];

//ELECHOUSE_CC1120&   DG_driver;	   /// The Driver we are to use
static volatile uint8_t         	DG_thisAddress;		/// The address of this node
static volatile uint8_t  			DG_rxHeaderTo;
static volatile uint8_t  			DG_rxHeaderFrom;
static volatile uint8_t  			DG_rxHeaderSource;
static volatile uint8_t				DG_rxHeaderDestination;
static volatile uint8_t   			DG_rxHeaderType;
static volatile uint8_t   			DG_rxHeaderData;
static volatile uint8_t   			DG_rxHeaderFlags;
static volatile uint8_t   			DG_rxHeaderSeqNum;
static volatile uint8_t				DG_rxHeaderHop;


static volatile uint8_t DGcandidateAddress;// = 0;
static volatile signed char DGcandidateRSSI;// = 0;


static volatile uint8_t DGreceivedType;// = 0;//To avoid receiving from Master that has same hop

static volatile unsigned long DGstartTime;
static volatile unsigned long DGresetTime;


static volatile uint8_t DGmasterBroadcastAddress;

static volatile uint8_t DGgatewayNumber;// = 0;

static volatile unsigned long	DGsendingTime;// = -60000;


#endif
