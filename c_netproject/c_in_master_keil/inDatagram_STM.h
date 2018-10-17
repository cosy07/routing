//2017-07-17
// Datagram.h
#ifndef Datagram_h
#define Datagram_h

#include "incc1120.h"
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
void 	DGInit(uint8_t masterNumber, uint8_t thisAddress, byte ch);
void 	DGSetReceive(void);


void 	DGsetThisAddress(uint8_t thisAddress);
void 	DGsetHeaderTo(uint8_t to);
void 	DGsetHeaderFrom(uint8_t from);
void 	DGsetHeaderMaster(uint8_t master);
void	DGsetHeaderType(uint8_t type);
void 	DGsetNumOfSlave(uint8_t num);

uint8_t        DGheaderTo();
uint8_t        DGheaderFrom();
uint8_t        DGheaderMaster();
uint8_t        DGheaderType();

uint8_t		   DGgetThisAddress();
uint8_t        DGgetMasterNumber();

bool           DGavailable();


byte           DGrecvData(uint8_t* buf);

void DGsend(uint8_t from, uint8_t to, uint8_t master, uint8_t type, uint8_t* temp_buf, uint8_t size);

bool DGsendToWaitAck(uint8_t from, uint8_t to, uint8_t master, uint8_t type, uint8_t* temp_buf, uint8_t size, unsigned long time);

void DGprintRecvPacketHeader();

static volatile uint8_t DGch;

extern byte DGtemp_buf[20];

static volatile uint8_t       DG_thisAddress;		/// The address of this node
static volatile uint8_t  			DG_rxHeaderTo;
static volatile uint8_t  			DG_rxHeaderFrom;
static volatile uint8_t  			DG_rxHeaderMaster;
static volatile uint8_t				DG_rxHeaderType;

static volatile uint8_t				DGgatewayNumber;
static volatile uint8_t				DGmasterNumber;
static volatile uint8_t				DGnum_of_slave;

static volatile unsigned long		DGsendingTime;


#endif
