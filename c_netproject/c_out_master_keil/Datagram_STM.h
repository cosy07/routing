//2017-07-17
// Datagram.h
#ifndef Datagram_h
#define Datagram_h

#include "cc1120.h"
#include <string.h>

/****************************************************************
#define
****************************************************************/
#define byte uint8_t

#define DEFAULT_RETRIES 							3			// The default number of retries

#define ROUTING_TABLE_SIZE 						31  	
#define ROUTER_MAX_MESSAGE_LEN 				48
#define R_MASTER_SEND_NUM 						5			// G <----- 1hop Master or Master 간의 패킷 수신율 측정을 위해서 보내는 패킷 개수
#define R_GATEWAY_SEND_NUM 						10		// G -----> 1hop Master 의 패킷 수신율 측정을 위해서 보내는 패킷 개수

#define NONE													0

#define TIME_TERM											1700	// hop to hop ACK을 받기 위해서 기다리는 시간(단위 ms)
#define MAX_UN_RECV										3			// 패킷에 대한 응답을 MAX_UN_RECV만큼 받지 못했다면 그 노드는 다시 라우팅해줌



/*****************************패킷 type****************************/

/////////////////////////////// 외부 ///////////////////////////////
// 초기 라우팅시 사용되는 type

// <1hop>
// 1. (G) -----REQUEST_BROADCAST---> (All master)
// 2. (G) -------REQUEST_TYPE------> (1hop master)
// 3. (G) <----REQUEST_ACK_TYPE----- (1hop master)
#define REQUEST_BROADCAST							0			// G가 라우팅 시작 시 사용
#define REQUEST_TYPE									1			// G가 1hop을 찾기 위해 각 M에게 1:1로 물어봄
#define REQUEST_ACK_TYPE							2			// 1hop인 M의 REQUEST_TYPE에 대한 응답

// <2hop>
// 1. (G) -----R2_REQUEST_TYPE-----> (1hop master)
// 2. (G) <-----------ACK----------- (1hop master)
// 3.                                (1hop master) -----R2_REQUEST_REAL_TYPE-----> (2hop master)
// 4.                                (1hop master) <-----R2_REQUEST_ACK_TYPE------ (2hop master)
// 5. (G) <--R2_REQUEST_ACK_TYPE---- (1hop master)
#define R2_REQUEST_TYPE								3
#define R2_REQUEST_REAL_TYPE					4
#define R2_REQUEST_ACK_TYPE						5

// multi hop
#define REQUEST_MULTI_HOP							6
#define REQUEST_MULTI_HOP_ACK					7

// multi hop을 위한 라우팉요청 후에도 응답이 없는 노드들에게 한 번 더 요청
#define REQUEST_DIRECT								8	
#define REQUEST_DIRECT_ACK						9

// 새로운 노드가 자신을 알리고자 보내는 패킷의 type
#define NEW_NODE_REGISTER							10

// Master scan시 사용되는 type
#define CHECK_ROUTING									11 			// G의 scan 요청
#define CHECK_ROUTING_ACK							12 			// M의 scan 응답
#define CHECK_ROUTING_ACK_REROUTING		13 			// M의 scan 응답 + 부모 노드 변경


#define ACK														14			// hop-to-hop ACK
#define NACK													15			// dst까지 패킷이 전달되지 못할 때 src에게 어떤 링크에서 전송이 끊어졌는지를 알려줄 때 사용

#define ROUTING_TABLE_UPDATE					16			// G가 M에게 라우팅 테이블 업데이트 명령을 내림(사용 안함)

// 내부 zone scan시 사용되는 type
#define SCAN_REQUEST_TO_MASTER				17			// G가 내부 zone scan 요청
#define SCAN_REQUEST_ACK_FROM_MASTER	18			// SCAN_REQUEST_TO_MASTER에 대한 src to dst의 ACK
#define SCAN_RESPONSE_TO_GATEWAY			19			// M이 내부 zone scan 완료를 통보

// G가 M에게 제어 명령을 내릴 때 사용되는 type
#define CONTROL_TO_MASTER							20			
#define CONTROL_RESPONSE_TO_GATEWAY		21

/////////////////////////////// 내부 ///////////////////////////////

//내부 마스터가 룸콘에게
#define SCAN_REQUEST_TO_RC						22			
#define CONTROL_TO_RC									23

#define CONTROL_RESPONSE_BROADCAST		24
#define SCAN_RESPONSE_TO_RC						25

#define SCAN_REQUEST_TO_SLAVE					26
#define SCAN_FINISH_TO_MASTER					27
#define RC_CONTROL_TO_SLAVE						28
#define GW_CONTROL_TO_SLAVE						29

//내부 마스터와 외부 마스터끼리의 통신
#define INTERNAL_COM_FAIL							30
#define INTERNAL_CONTROL							31
#define INTERNAL_CONTROL_FINISH				32
#define INTERNAL_SCAN									33
#define INTERNAL_SCAN_FINISH					34


// Values for the possible states for routes
typedef enum
{
	Invalid = 0,          										  // No valid route is known
	Discovering,           										  // Discovering a route
	Valid                  										  // Route is valid
} RouteState;

// Defines an entry in the routing table
typedef struct
{
	int8_t					dest;      								  // Destination node address
	int8_t					next_hop;  								  // Send via this next hop address
	int8_t					state;     								  // State of this route, one of RouteState
	int8_t					hop;			 								  // hop count
	unsigned long		lastTime;									  // last used time
} RoutingTableEntry;

/****************************************************************
function
****************************************************************/

void 	DGInit(uint8_t gatewayNumber, uint8_t thisAddress, byte ch);
void 	DGSetReceive(void);

void DGsetThisAddress(uint8_t thisAddress);
void DGsetHeaderTo(uint8_t to);
void DGsetHeaderFrom(uint8_t from);
void DGsetHeaderSource(uint8_t address);
void DGsetHeaderDestination(uint8_t address);
void DGsetHeaderType(uint8_t type);
void DGsetHeaderData(uint8_t data);
void DGsetHeaderSeqNum(uint8_t seq);
void DGsetHeaderFlags(uint8_t flags);
void DGsetHeaderHop(uint8_t hop);

uint8_t DGheaderTo();
uint8_t DGheaderFrom();
uint8_t DGheaderSource();
uint8_t DGheaderDestination();
uint8_t DGheaderType();
uint8_t DGheaderData();
uint8_t DGheaderSeqNum();
uint8_t DGheaderFlags();
uint8_t	DGheaderHop();
uint8_t	DGgetThisAddress();


bool DGavailable();

byte DGrecvData(uint8_t* buf);

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

bool DGG_discoverNewPath(uint8_t address);

void DGG_find_error_node(uint8_t address);

bool DG_requestDirect(uint8_t address);

void DGgetPath(uint8_t address, uint8_t* path);

void DGchangeNextHop(uint8_t address, uint8_t hopCount);

void DGnewMaster();

void DGcheckRoutingTable();

void DGdeleteRoute(uint8_t index);




/****************************************************************
variable
****************************************************************/


extern RoutingTableEntry _routes[ROUTING_TABLE_SIZE]; // 라우팅 테이블

extern bool DGcheckReceive[34]; // index : 마스터 주소, value : 게이트웨이 입장에서 해당 마스터의 경로 설정이 되었는지 check

extern int DGmaster_num; // master 개수

extern int8_t DGparentMaster[34]; // index : 마스터 주소, value : 마스터의 부모 노드
extern byte DGunRecvCnt[34]; // index : 마스터 주소, value : 해당 마스터에게 받지 못한 응답의 개수
extern byte DGtemp_buf[20]; // 패킷의 payload
extern byte DGbuffer[20]; // 받은 패킷을 임시로 저장하기 위한 배열(DGtemp_buf 복사)

static uint8_t DG_thisAddress;		// The address of this node
static uint8_t DG_rxHeaderTo;
static uint8_t DG_rxHeaderFrom;
static uint8_t DG_rxHeaderSource;
static uint8_t	DG_rxHeaderDestination;
static uint8_t DG_rxHeaderType;
static uint8_t DG_rxHeaderData;
static uint8_t DG_rxHeaderFlags;
static uint8_t DG_rxHeaderSeqNum;
static uint8_t	DG_rxHeaderHop;

static uint8_t DGcandidateAddress; // 마스터의 후보 부모 노드
static signed char DGcandidateRSSI; // 후보 부모 노드의 RSSI

static uint8_t DGreceivedType; //To avoid receiving from Master that has same hop

static unsigned long DGstartTime; // 응답 메시지의 time out 여부를 check하기 위해서 패킷 전송 시간을 기록
static unsigned long DGresetTime; // 마스터가 오랫동안 사용하지 않는 라우팅 테이블 엔트리를 주기적으로 삭제하기 위해서 시간을 기록

static uint8_t DGmasterBroadcastAddress;

static uint8_t DGgatewayNumber; // 층

static unsigned long	DGsendingTime; // 1초에 한 번씩만 전송이 가능하도록 하기위해서 패킷 전송 시간을 기록 (send 함수에서 사용)

#endif
