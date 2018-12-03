//2017-07-17
// Datagram.cpp
//
// Copyright (C) 2011 Mike McCauley
// $Id: RHDatagram.cpp,v 1.6 2014/05/23 02:20:17 mikem Exp $

#include "Datagram_STM.h"


////////////////////////////////////////////////////////////////////
// Public methods


/****************************************************************
*FUNCTION NAME : DGInit(uint8_t gatewayNumber, uint8_t thisAddress, byte ch)
*FUNCTION      : 주소 변수에 주소 넣기, 채널 설정과 같은 초기화
*INPUT         : gatewayNumber(층 번호), thisAddress(층에서 내 주소), ch(사용할 채널)
*OUTPUT        : NONE
****************************************************************/
void DGInit(uint8_t gatewayNumber, uint8_t thisAddress, byte ch)
{
	DG_thisAddress = thisAddress;
	DGgatewayNumber = gatewayNumber;

	DGmaster_num = 1;
	DGcandidateAddress = 0;
	DGcandidateRSSI = 0;
	DGreceivedType = 0;
	DGgatewayNumber = 0;
	DGsendingTime = -60000;
	
	for (int i = 0; i < 34; i++)
	{
		DGcheckReceive[i] = false;
		DGparentMaster[i] = -1;
		DGunRecvCnt[i] = 0;
	}

	DGclearRoutingTable();
	DGmasterBroadcastAddress = 0xFF;

	DGsetThisAddress(DG_thisAddress);
	e_Init(ch);// Setup Channel number
}


/****************************************************************
*FUNCTION NAME : DGSetReceive()
*FUNCTION      : 통신 모듈(cc1120)을 수신가능 상태로
*INPUT         : NONE
*OUTPUT        : NONE
****************************************************************/
void DGSetReceive()
{
	e_SetReceive();
}

void  DGsetThisAddress(uint8_t thisAddress)
{
	e_setThisAddress(thisAddress);
	// Use this address in the transmitted FROM header
	DGsetHeaderFrom(thisAddress);
	DG_thisAddress = thisAddress;
}

void  DGsetHeaderFrom(uint8_t from)
{
	e_setHeaderFrom(from);
}

void DGsetHeaderTo(uint8_t to)
{
	e_setHeaderTo(to);
}


void DGsetHeaderSource(uint8_t source)
{
	e_setHeaderSource(source);
}

void DGsetHeaderDestination(uint8_t destination)
{
	e_setHeaderDestination(destination);
}

void DGsetHeaderType(uint8_t type)
{
	e_setHeaderType(type);
}

void DGsetHeaderData(uint8_t data)
{
	e_setHeaderData(data);
}
void DGsetHeaderSeqNum(uint8_t seq)
{
	e_setHeaderSeqNum(seq);
}
void DGsetHeaderFlags(uint8_t flags)
{
	e_setHeaderFlags(flags);
}
void DGsetHeaderHop(uint8_t hop)
{
	e_setHeaderHop(hop);
}

uint8_t DGheaderTo()
{
	return e_headerTo();
}
uint8_t DGheaderFrom()
{
	return e_headerFrom();
}

uint8_t DGheaderSource()
{
	return(e_headerSource());
}
uint8_t DGheaderDestination()
{
	return(e_headerDestination());
}
uint8_t DGheaderType()
{
	return(e_headerType());
}
uint8_t DGheaderData()
{
	return(e_headerData());
}
uint8_t  DGheaderSeqNum()
{
	return(e_headerSeqNum());
}
uint8_t DGheaderFlags()
{
	return(e_headerFlags());
}
uint8_t DGheaderHop()
{
	return(e_headerHop());
}
uint8_t DGgetThisAddress()
{
	return DG_thisAddress;
}


/****************************************************************
*FUNCTION NAME : DGavailable()
*FUNCTION      : 패킷을 수신중인지 확인
*INPUT         : NONE
*OUTPUT        : 수신 중이라면 true return
****************************************************************/
bool DGavailable()
{
	if (e_CheckReceiveFlag())
		return true;
	else
		return false;
}


/****************************************************************
*FUNCTION NAME : DGrecvData(uint8_t* buf)
*FUNCTION      : It receives data and stores data into buf
*INPUT         : NONE
*OUTPUT        : The size of received data excluding headers
****************************************************************/
byte DGrecvData(uint8_t* buf)
{
	byte size;
	if (size = e_ReceiveData(buf))
	{
		DG_rxHeaderFrom = DGheaderFrom();
		DG_rxHeaderTo = DGheaderTo();
		DG_rxHeaderSource = DGheaderSource();
		DG_rxHeaderDestination = DGheaderDestination();
		DG_rxHeaderType = DGheaderType();
		DG_rxHeaderData = DGheaderData();
		DG_rxHeaderFlags = DGheaderFlags();
		DG_rxHeaderSeqNum = DGheaderSeqNum();
		DG_rxHeaderHop = DGheaderHop();
		return (size - CC1120_HEADER_LEN);
	}
	return 0;
}


/****************************************************************
*FUNCTION NAME : DGaddRouteTo(uint8_t  dest, uint8_t  next_hop, uint8_t state, uint8_t hop, unsigned long lastTime)
*FUNCTION      : dest를 라우팅 테이블에 등록
*INPUT         : dest(목적지), next_hop, state, hop(hop count), lastTime
*OUTPUT        : NONE
****************************************************************/
void DGaddRouteTo(uint8_t  dest, uint8_t  next_hop, uint8_t state, uint8_t hop, unsigned long lastTime)
{
	uint8_t i;

	// First look for an existing entry we can update
	for (i = 0; i < ROUTING_TABLE_SIZE; i++)
	{
		if (_routes[i].dest == dest)
		{
			_routes[i].dest = dest;
			_routes[i].next_hop = next_hop;
			_routes[i].hop = hop;
			_routes[i].state = state;
			_routes[i].lastTime = millis();
			return;
		}
	}

	// Look for an invalid entry we can use
	for (i = 0; i < ROUTING_TABLE_SIZE; i++)
	{
		if (_routes[i].state == Invalid)
		{
			_routes[i].dest = dest;
			_routes[i].next_hop = next_hop;
			_routes[i].hop = hop;
			_routes[i].state = state;
			_routes[i].lastTime = millis();
			return;
		}
	}

	// Should be an invalid slot now
	for (i = 0; i < ROUTING_TABLE_SIZE; i++)
	{
		if (_routes[i].state == Invalid)
		{
			_routes[i].dest = dest;
			_routes[i].next_hop = next_hop;
			_routes[i].hop = hop;
			_routes[i].state = state;
			_routes[i].lastTime = millis();
		}
	}
}


/****************************************************************
*FUNCTION NAME : DGgetRouteTo(uint8_t  dest)
*FUNCTION      : 라우팅 테이블에서 dest에 해당하는 행을 찾아줌
*INPUT         : dest(목적지)
*OUTPUT        : 라우팅 테이블에서 dest에 해당하는 행의 주소
****************************************************************/
RoutingTableEntry* DGgetRouteTo(uint8_t  dest)
{
	uint8_t i;
	for (i = 0; i < ROUTING_TABLE_SIZE; i++)
		if (_routes[i].dest == dest && _routes[i].state != Invalid)
		{
			_routes[i].lastTime = millis();;
			return &_routes[i];
		}
	return NULL;
}


/****************************************************************
*FUNCTION NAME : DGdeleteRoute(uint8_t index)
*FUNCTION      : 라우팅 테이블 엔트리를 지움
*INPUT         : 지우고자 하는 행의 index
*OUTPUT        : NONE
****************************************************************/
void DGdeleteRoute(uint8_t index)
{
	// Delete a route by copying following routes on top of it
	memcpy(&_routes[index], &_routes[index + 1], 
					sizeof(RoutingTableEntry) * (ROUTING_TABLE_SIZE - index - 1));
	_routes[ROUTING_TABLE_SIZE - 1].state = Invalid;
}


/****************************************************************
*FUNCTION NAME : DGprintRoutingTable()
*FUNCTION      : 라우팅 테이블 전체를 출력
*INPUT         : NONE
*OUTPUT        : NONE
****************************************************************/
void DGprintRoutingTable()
{
	uint8_t i;
	for (i = 0; i < ROUTING_TABLE_SIZE; i++)
	{
		printf("%d Dest: %d Next Hop: %d State: %d Hop: %d Last Time: %d\r\n", 
						i, 
						_routes[i].dest, 
						_routes[i].next_hop, 
						_routes[i].state, 
						_routes[i].hop, 
						_routes[i].lastTime);
	}
}


/****************************************************************
*FUNCTION NAME : DGdeleteRouteTo(uint8_t dest)
*FUNCTION      : 라우팅 테이블에서 dest를 지움
*INPUT         : dest (지우고자 하는 주소)
*OUTPUT        : 성공 여부
****************************************************************/
bool DGdeleteRouteTo(uint8_t dest)
{
	uint8_t i;
	for (i = 0; i < ROUTING_TABLE_SIZE; i++)
	{
		if (_routes[i].dest == dest)
		{
			_routes[i].state = Invalid;
			DGdeleteRoute(i);
			return true;
		}
	}
	return false;
}


/****************************************************************
*FUNCTION NAME : DGclearRoutingTable()
*FUNCTION      : 라우팅 테이블 clear
*INPUT         : NONE
*OUTPUT        : NONE
****************************************************************/
void DGclearRoutingTable()
{
	uint8_t i;
	for (i = 0; i < ROUTING_TABLE_SIZE; i++)
	{
		_routes[i].state = Invalid;
		_routes[i].next_hop = -1;
		_routes[i].dest = -1;
		_routes[i].lastTime = 0;
	}
}


/****************************************************************
*FUNCTION NAME : DGcheckRoutingTable()
*FUNCTION      : 오랜기간 동안 사용되지 않는 라우팅 테이블의 엔트리는 지움
*INPUT         : NONE
*OUTPUT        : NONE
****************************************************************/
void DGcheckRoutingTable()
{
	printf("checkRoutingTable()\r\n");
	DGprintRoutingTable();
	for (int8_t i = 0; i < ROUTING_TABLE_SIZE; i++)
	{
		if (_routes[i].state == Valid && _routes[i].lastTime + 1000000 < millis())
		{
			DGdeleteRoute(i);
		}
	}
	printf("after\r\n");
	DGprintRoutingTable();
}


/****************************************************************
*FUNCTION NAME : DGfind_child_node(uint8_t* child_node)
*FUNCTION      : 마스터들이 자식노드를 찾음 
(마스터 입장에서는 라우팅 테이블 엔트리 중 게이트웨이를 제외한 나머지는 모두 자식 노드들)
*INPUT         : 찾은 자식 노드들의 주소를 넣을 포인터
*OUTPUT        : 자식노드 개수
****************************************************************/
int DGfind_child_node(uint8_t* child_node)
{
	int cnt = 0;
	for (int8_t i = 0; i < ROUTING_TABLE_SIZE; i++)
	{
		if (_routes[i].state == Valid && _routes[i].dest != (DG_thisAddress & 0x00))
		{
			child_node[cnt++] = _routes[i].dest;
		}
	}
	return cnt;
}


/****************************************************************
*FUNCTION NAME : DGsend(uint8_t from, 
												uint8_t to, 
												uint8_t src, 
												uint8_t dst, 
												uint8_t type, 
												uint8_t data, 
												uint8_t flags, 
												uint8_t seqNum, 
												uint8_t hop, 
												uint8_t* DGtemp_buf, 
												uint8_t size)

*FUNCTION      : 데이터 전송
*INPUT         : from, to, src, dst, type, data, flags, seqNum, hop, DGtemp_buf, size
*OUTPUT        : NONE
****************************************************************/
void DGsend(uint8_t from, 
						uint8_t to, 
						uint8_t src, 
						uint8_t dst, 
						uint8_t type, 
						uint8_t data, 
						uint8_t flags, 
						uint8_t seqNum, 
						uint8_t hop, 
						uint8_t* DGtemp_buf, 
						uint8_t size)
{
	if (type >= 10 && DGgetRouteTo(dst) == NULL)
		return;
	while (millis() - DGsendingTime < 1000); // 20일에 한 번씩 reset된다고 하면 sendingTime의 type은 int여도 됨
	DGsetHeaderFrom(from);
	DGsetHeaderTo(to);
	DGsetHeaderSource(src);
	DGsetHeaderDestination(dst);
	DGsetHeaderType(type);
	DGsetHeaderData(data);
	DGsetHeaderFlags(flags);
	DGsetHeaderSeqNum(seqNum);
	DGsetHeaderHop(hop);
	e_SendData(DGtemp_buf, size);
	DGsendingTime = millis();
}

//보류
/****************************************************************
*FUNCTION NAME : DGsendToWaitAck(uint8_t from, 
																 uint8_t to, 
																 uint8_t src, 
																 uint8_t dst, 
																 uint8_t type, 
																 uint8_t data, 
																 uint8_t flags, 
																 uint8_t seqNum, 
																 uint8_t hop, 
																 uint8_t* DGtemp_buf, 
																 uint8_t size, 
																 unsigned long time)

*FUNCTION      : 전송 후 hop to hop ACK을 받을 때까지 대기
*INPUT         : from, to, src, dst, type, data, flags, seqNum, hoop, DGtemp_buf, size, time
*OUTPUT        : ACK받았는지 유무
****************************************************************/
bool DGsendToWaitAck(uint8_t from, 
										 uint8_t to, 
										 uint8_t src,
										 uint8_t dst,
										 uint8_t type, 
										 uint8_t data, 
										 uint8_t flags, 
										 uint8_t seqNum, 
										 uint8_t hop, 
										 uint8_t* DGtemp_buf, 
								     uint8_t size, 
										 unsigned long time)
{
	if (type >= 11 && DGgetRouteTo(dst) == NULL)
		return false;
	unsigned long DGstartTime = 0;
	DGsetHeaderFrom(from);
	DGsetHeaderTo(to);
	DGsetHeaderSource(src);
	DGsetHeaderDestination(dst);
	DGsetHeaderType(type);
	DGsetHeaderData(data);
	DGsetHeaderSeqNum(seqNum);
	DGsetHeaderHop(hop);
	byte length = size;
	for (int i = 0; i < size; i++)
		DGbuffer[i] = DGtemp_buf[i];

	for (int i = 0; i < DEFAULT_RETRIES; i++)
	{
		while (millis() - DGsendingTime < 1000);

		printf("retry : %d", i);
		e_SendData(DGbuffer, length);
		DGsendingTime = millis();
		DGstartTime = millis();
		while (millis() - DGstartTime < TIME_TERM)
		{
			DGSetReceive();
			if (DGavailable())
				if (DGrecvData(DGtemp_buf) && DG_rxHeaderFrom == to)
						return true;
		}
		delay(time);
	}
	return false;
}


/****************************************************************
*FUNCTION NAME : DGprintRecvPacketHeader()
*FUNCTION      : 패킷 헤더 출력
*INPUT         : NONE
*OUTPUT        : NONE
****************************************************************/
void DGprintRecvPacketHeader()
{
	printf("-----------------------------------------------------------------------------\r\n");
	printf("DG_rxHeaderFrom : %d\r\n", DG_rxHeaderFrom);
	printf("DG_rxHeaderTo : %d\r\n", DG_rxHeaderTo);
	printf("DG_rxHeaderSource : %d\r\n", DG_rxHeaderSource);
	printf("DG_rxHeaderDestination : %d\r\n", DG_rxHeaderDestination);
	printf("DG_rxHeaderType : %d\r\n", DG_rxHeaderType);
	printf("DG_rxHeaderData : %d\r\n", DG_rxHeaderData);
	printf("DG_rxHeaderFlags : %d\r\n", DG_rxHeaderFlags);
	printf("DG_rxHeaderSeqNum : %d\r\n", DG_rxHeaderSeqNum);
	printf("DG_rxHeaderHop : %d\r\n", DG_rxHeaderHop);
}


/****************************************************************
*FUNCTION NAME : DGprintPath()
*FUNCTION      : 게이트웨이가 각 노드로 가기까지의 경로 출력
*INPUT         : NONE
*OUTPUT        : NONE
****************************************************************/
void DGprintPath()
{
	uint8_t temp_address;
	printf("-----PATH-----\r\n");
	byte index;
	for (uint8_t i = 1; i <= DGmaster_num; i++)
	{
		printf("%d master : ", i);
		temp_address = i;
		while (temp_address != DG_thisAddress)
		{
			printf("%d -> ", temp_address);
			index = temp_address;
			temp_address = DGparentMaster[index];
		}
		printf("%d\r\n", DG_thisAddress);
	}
	printf("--------------\r\n");
}


/****************************************************************
*FUNCTION NAME : DGgetPath(uint8_t address, uint8_t* path)
*FUNCTION      : address로 가기까지의 경로를 path에 저장, G가 사용
								 if G -> M1 -> M2 -> M3 일 때 address가 M3라면
								 path[0] : M3
								 path[1] : M2
								 path[2] : M1
*INPUT         : address, path
*OUTPUT        : NONE
****************************************************************/
void DGgetPath(uint8_t address, uint8_t* path)
{
	uint8_t temp_address = address;
	byte index;
	uint8_t cnt = 0;
	while (temp_address != DG_thisAddress)
	{
		path[cnt++] = temp_address;
		index = temp_address;
		temp_address = DGparentMaster[index];
	}
}


/****************************************************************
*FUNCTION NAME : DGprintTree()
*FUNCTION      : tree형태로 출력시켜주는 tool사용을 위한 함수
*INPUT         : NONE
*OUTPUT        : NONE
****************************************************************/
void DGprintTree()
{
	printf("printTree %d\r\n", 0);
	for (uint8_t i = 1; i <= DGmaster_num; i++)
	{
		if (DGcheckReceive[i])
			printf("treeComponent %d %d\r\n", DGparentMaster[i], i);
	}
}


/****************************************************************
*FUNCTION NAME : DGG_get_i_row_node_list(uint8_t row_number, uint8_t *node_list)
*FUNCTION      : Gatway gets the list of node on the i_th row
*INPUT         : row number
*OUTPUT        : return value: number of node on the i_th row
****************************************************************/
uint8_t DGG_get_i_row_node_list(uint8_t row_number, uint8_t *node_list)
{
	uint8_t j = 0;
	for (int i = 0; i < ROUTING_TABLE_SIZE; i++)
	{
		if (_routes[i].hop == row_number)
		{
			node_list[j] = _routes[i].dest;
			j++;
		}
	}
	return j;
}


/****************************************************************
*FUNCTION NAME : DGFromGatewayToMaster()
*FUNCTION      : 게이트웨이가 초기 라우팅과정을 수행하는 함수
								 DGG_find_1stRow_master()
								 DGG_find_2ndRow_master()
								 DGG_find_multihop_node()
								 DG_requestDirect()
								 를 차례로 호출하며 hop count가 작은 노드부터 찾아 나간다.
*INPUT         : NONE
*OUTPUT        : NONE
****************************************************************/
void DGFromGatewayToMaster() {
	uint8_t address_i;
	uint8_t unknown_node_list[32];
	uint8_t number_of_unknown_node;
	printf("FromGatewayToMaster\r\n");
	printf("[row1]\r\n");

	if (DGG_find_1stRow_master() != DGmaster_num)
	{
		if ((number_of_unknown_node = DGG_find_2ndRow_master(unknown_node_list)))
		{
			if (DGG_find_multihop_node(number_of_unknown_node, unknown_node_list))
			{
				
				// multi hop 라우팅 요청후에도 경로 설정이 되지 않은 노드들에게는 1hop요청 메시지부터 전송하여 다시 경로 설정을 요청함
				printf("REQUEST_DIRECT-------------------------------\r\n");
				
				for (int i = 1; i <= DGmaster_num; i++)
					if (!DGcheckReceive[i]) //경로 설정이 되지 않은 노드
						DG_requestDirect(i);
			}
		}
	}
}


/****************************************************************
*FUNCTION NAME : DGG_find_1stRow_master()
*FUNCTION      : Gateway가 1hop 노드들을 찾음 
								 라우팅 시작 메세지를 R_GATEWAY_SEND_NUM만큼 전송 후
                 각 master들에게 1:1로 라우팅 요청 패킷을 전송
								 master로부터 응답이 도착하면 패킷 수신율을 고려하여 라우팅 테이블에 삽입
*INPUT         : NONE
*OUTPUT        : 라우팅이 완료된 1 hop 노드들의 갯수
****************************************************************/
int8_t DGG_find_1stRow_master()
{
	uint8_t num_of_routed_nodes = 0;
	printf("send broadcast request message\r\n");
	printf("%d\r\n", DGmasterBroadcastAddress);
	for (int i = 0; i < R_GATEWAY_SEND_NUM; i++)
	{// 마스터들의 패킷 수신율 계산을 위해서 R_GATEWAY_SEND_NUM번 라우팅 시작 패킷을 전송
		DGsend(DG_thisAddress, 
					 DGmasterBroadcastAddress, 
					 DG_thisAddress, 
					 DGmasterBroadcastAddress, 
					 REQUEST_BROADCAST, 
					 NONE, 
					 NONE, 
					 NONE, 
					 NONE, 
					 DGtemp_buf, 
					 sizeof(DGtemp_buf));
	}
	for (int i = 1; i <= DGmaster_num; i++)
	{
		uint8_t dst = i;
		uint8_t recvAckNum = 0;
		DGsend(DG_thisAddress, 
					 dst, 
					 DG_thisAddress, 
					 dst, 
					 REQUEST_TYPE, 
					 NONE, 
					 NONE, 
					 NONE, 
					 NONE, 
					 DGtemp_buf, 
					 sizeof(DGtemp_buf));
		// 각 노드들에게 1:1로 경로 설정을 요청
		
		DGstartTime = millis();
		recvAckNum = 0;
		
		//응답받기를 기다림
		while (millis() - DGstartTime < TIME_TERM * R_MASTER_SEND_NUM)
		{
			DGSetReceive();
			if (DGavailable() && DGrecvData(DGtemp_buf) && DG_rxHeaderFrom == dst 
				&& DG_rxHeaderTo == DG_thisAddress && DG_rxHeaderType == REQUEST_ACK_TYPE)
			{
				recvAckNum++; // 수신된 응답 패킷 개수를 카운트함
				if (recvAckNum >= R_MASTER_SEND_NUM * 0.8) // 라우팅 응답 패킷 수신율이 80%이상일 경우에만 라우팅 테이블에 등록시킴
				{
					printf("%d", recvAckNum);
					printf("%f", R_MASTER_SEND_NUM * 0.8);
					DGaddRouteTo(DG_rxHeaderFrom, DG_rxHeaderFrom, Valid, 1, millis());
					DGprintRecvPacketHeader();
					printf("%d master is 1 hop node", i);
					DGcheckReceive[i] = true;
					DGparentMaster[i] = DG_thisAddress;
					DGprintRoutingTable();
					DGprintTree();
					num_of_routed_nodes++;
					break;
				}
			}
		}
		delay(TIME_TERM);
	}
	return num_of_routed_nodes;
}


/****************************************************************
*FUNCTION NAME : DGG_find_2ndRow_master(uint8_t* unknown_node_list)
*FUNCTION      : Gateway가 1hop 노드들에게 라우팅이 안된 노드들의 주소를 payload에 넣어 전송함
*INPUT         : unknown_node_list (라우팅이 되지 낳은 노드의 주소를 담은 패열)
*OUTPUT        : routing안된 노드의 개수
****************************************************************/
// <2hop>
// 1. (G) -----R2_REQUEST_TYPE-----> (1hop master)
// 2. (G) <-----------ACK----------- (1hop master)
// 3.                                (1hop master) -----R2_REQUEST_REAL_TYPE-----> (2hop master)
// 4.                                (1hop master) <-----R2_REQUEST_ACK_TYPE------ (2hop master)
// 5. (G) <--R2_REQUEST_ACK_TYPE---- (1hop master)

int8_t DGG_find_2ndRow_master(uint8_t* unknown_node_list)
{
	//row2 : 1hop인 master들에게 아직 라우팅이 안된 노드들의 주소를 보내서 근처에 노드들이 있는지 찾아달라고 요청
	printf("[row2]\r\n");

	uint8_t node_list[34] = { 0 };
	uint8_t number_of_node;
	uint8_t number_of_unknown_node = 0;

	number_of_node = DGG_get_i_row_node_list(1, node_list);

	printf("1hop's num : %d\r\n", number_of_node);

	for (uint8_t i = 0; i < number_of_node; i++) // 1hop node들에게 2hop 노드들을 찾아달라고 요청
	{
		number_of_unknown_node = 0;
		for (uint8_t j = 1; j <= DGmaster_num; j++) // routing이 안된 노드들의 주소를 라우팅 테이블을 뒤져서 알아냄
		{
			if (!DGcheckReceive[j])
			{
				uint8_t address = j;
				printf("unreceive : %d\r\n", address);
				DGtemp_buf[number_of_unknown_node] = address; // 라우팅이 안된 노드의 주소를 payload에 적음
				number_of_unknown_node++;
			}
		}
		if (number_of_unknown_node == 0)
			return 0;
		
		printf("send to %d\r\n", node_list[i]);

		//from, to, src, dst, type, data, flags, seqnum, hop, payload, size of payload, timeout
		if (!DGsendToWaitAck(DG_thisAddress, 
												 node_list[i], 
												 DG_thisAddress, 
												 node_list[i], 
												 R2_REQUEST_TYPE, 
												 number_of_unknown_node, 
												 NONE, 
												 NONE, 
												 NONE, 
												 DGtemp_buf, 
												 sizeof(DGtemp_buf), 
												 TIME_TERM))
		{ //1hop인 노드에게 요청을 했으나 ACK을 받지 못했을 경우
			DGcheckReceive[node_list[i]] = false;
			DGparentMaster[node_list[i]] = 0;
			continue;
		}

		while (1)
		{
			DGSetReceive();
			if (DGavailable())
			{
				if (DGrecvData(DGtemp_buf)) // 1hop인 노드로부터 요청에 대한 응답을 수신
				{
					if (DG_rxHeaderTo == DG_thisAddress && DG_rxHeaderType == R2_REQUEST_ACK_TYPE && DG_rxHeaderFrom == node_list[i])
					{
						DGsend(DG_thisAddress, 
									 DG_rxHeaderFrom, 
									 DG_thisAddress, 
									 DG_rxHeaderFrom, 
									 ACK, 
									 NONE, 
									 NONE, 
									 NONE, 
									 NONE, 
									 DGtemp_buf, 
									 sizeof(DGtemp_buf));
						DGprintRecvPacketHeader();
						
						// _rxHeaderData에 찾은 노드의 개수가 들어 있음
						for (uint8_t j = 0; j < DG_rxHeaderData; j++)
						{
							uint8_t temp_addr = DGtemp_buf[j];
							DGaddRouteTo(temp_addr, DG_rxHeaderFrom, Valid, 2, millis());
							DGparentMaster[temp_addr] = DG_rxHeaderFrom;
							DGcheckReceive[temp_addr] = true;
							printf("%d \r\n%d \r\n", temp_addr, DGcheckReceive[temp_addr]);
						}
						DGprintRoutingTable();
						DGprintTree();
						break;
					}
				}
			}
		}
	}
	
	number_of_unknown_node = 0;
	for (uint8_t i = 1; i <= DGmaster_num; i++)
	{
		if (!DGcheckReceive[i])
		{
			unknown_node_list[number_of_unknown_node++] = i;
		}
	}
	return number_of_unknown_node;
}


/****************************************************************
*FUNCTION NAME : DGG_find_multihop_node(uint8_t number_of_unknown_node, uint8_t* unknown_node_list)
*FUNCTION      :  Gateway finished the routing discovery upto 2nd row.
Gateway tries to find the 3rd and more row nodes.
- 먼저 2nd row node들에게 아직 라우팅이 안된 노드의 주소를 payload에 포함하여 전송한다.
- 3rd row node를 구한 후에는, 3rd row node에게 요청한다.
(최근에 구한 row의 노드들에게 multi hop 라우팅 요청 메시지를 전송)
*INPUT         : number_of_unknown_node(라우팅이 안된 노드의 개수)
								 unknown_node_list(라우팅이 안 된 노드들의 주소)
*OUTPUT        : 라우팅 안된 노드의 개수
****************************************************************/
int8_t DGG_find_multihop_node(uint8_t number_of_unknown_node, uint8_t* unknown_node_list)
{
	uint8_t row_number = 2;
	uint8_t number_of_node;
	uint8_t node_list[34] = { 0 };
	uint8_t cnt = number_of_unknown_node;

	for (int i = 1; i <= DGmaster_num; i++)
	{
		printf("%d ", DGcheckReceive[i]);
	}
	printf("\r\n");

	number_of_node = DGG_get_i_row_node_list(row_number, node_list);

	printf("[MULTI_HOP]\r\n");
	while (number_of_node != 0)
	{
		for (uint8_t i = 0; i < cnt; i++)
		{
			if (DGG_request_path_one_by_one(unknown_node_list[i], row_number, node_list, number_of_node, REQUEST_MULTI_HOP))
				// 노드 하나하나 일일이 요청함
			{
				if (--number_of_unknown_node == 0)
					return 0;
			}
		}
		number_of_node = DGG_get_i_row_node_list(++row_number, node_list);
	}
	return number_of_unknown_node;
}


/****************************************************************
*FUNCTION NAME : DGG_request_path_one_by_one(uint8_t unknown_address, 
																						 uint8_t row_number, 
																						 uint8_t* node_list, 
																						 byte number_of_node, 
																						 uint8_t type)
*FUNCTION      : node_list에 있는 노드들에게 주소가 unknown_address인 노드가 인접해 있는지 물어봄
*INPUT         : unknown_address (목적지 주소) -> 경로 설정을 해야 할 노드
								 row_number (요청하는 메시지를 보낼 노드들의 hop count)
								 node_list (요청할 노드들의 주소) 
								 -> 경로 설정이 된 노드들 즉, unknown_address 노드가 주변에 있는지 찾아야 할 노드
								 number_of_node (node_list에 들어있는 노드의 개수)
								 type (node_list들이 unknown_address에게 전송할 패킷의 type)
*OUTPUT        : 성공 여부
****************************************************************/
bool DGG_request_path_one_by_one(uint8_t unknown_address, uint8_t row_number, uint8_t* node_list, byte number_of_node, uint8_t type)
{
	printf("G_request_path_one_by_one\r\n");
	uint8_t masterNum = unknown_address;

	for (uint8_t i = 0; i < number_of_node; i++)
	{
		if (node_list[i] == unknown_address)
			continue;
		if (!DGcheckReceive[node_list[i]])
			continue;
		
		DGtemp_buf[0] = unknown_address;

		uint8_t next_hop = DGgetRouteTo(node_list[i])->next_hop;

		if (!DGsendToWaitAck(DG_thisAddress, 
												 next_hop, 
												 DG_thisAddress, 
												 node_list[i], 
												 type, 
												 NONE, 
												 NONE, 
												 NONE, 
												 NONE, 
												 DGtemp_buf, 
												 sizeof(DGtemp_buf), 
												 TIME_TERM))
		{
			DGcheckReceive[next_hop] = false;
			DGparentMaster[next_hop] = -1;
			DGG_discoverNewPath(next_hop);
			continue;
		}

		DGstartTime = millis();
		while ((millis() - DGstartTime) < (row_number + 1) * 4 * DEFAULT_RETRIES * TIME_TERM)
		{
			DGSetReceive();
			if (DGavailable())
			{
				if (DGrecvData(DGtemp_buf) && DG_rxHeaderTo == DG_thisAddress && DG_rxHeaderDestination == DG_thisAddress)
				{
					DGprintRecvPacketHeader();
					DGsend(DG_thisAddress, 
								 DG_rxHeaderFrom, 
								 DG_thisAddress, 
								 DG_rxHeaderFrom, 
								 ACK, 
								 NONE, 
								 NONE, 
								 NONE, 
								 NONE, 
								 DGtemp_buf, 
								 sizeof(DGtemp_buf));
					if (DG_rxHeaderSource == unknown_address 
						&& (DG_rxHeaderType == REQUEST_MULTI_HOP_ACK || DG_rxHeaderType == REQUEST_DIRECT_ACK) 
						&& DG_rxHeaderFrom == next_hop)
					{
						DGaddRouteTo(DG_rxHeaderSource, DG_rxHeaderFrom, Valid, DG_rxHeaderHop + 1, millis());
						DGparentMaster[masterNum] = DGtemp_buf[0];
						DGcheckReceive[masterNum] = true;
						DGprintRoutingTable();
						DGprintTree();
						return true;
					}
					else if (DG_rxHeaderType == NACK)
					{
						printf("receive NACK\r\n");
						uint8_t reroutingAddr = DGtemp_buf[0];
						DGcheckReceive[reroutingAddr] = false;
						DGparentMaster[reroutingAddr] = -1;
						DGG_discoverNewPath(reroutingAddr);
						return false;
					}
					else if (DG_rxHeaderData == 0)
						break;
				}
			}
		}
	}
	return false;
}


/****************************************************************
*FUNCTION NAME : DG_requestDirect(uint8_t address)
*FUNCTION      : hop count를 증가시켜가며 경로 설정 요청
*INPUT         : address (목적지 주소)
*OUTPUT        : 성공 여부
****************************************************************/
bool DG_requestDirect(uint8_t address)
{
	bool oneHop = false;
	// 1hop으로 가정하고 direct로 물어봄
	for (int j = 0; j < DEFAULT_RETRIES; j++)
	{
		DGbuffer[0] = address;
		DGsend(DG_thisAddress, 
					 address, 
					 DG_thisAddress, 
					 address, 
					 REQUEST_DIRECT, 
					 NONE, 
					 NONE, 
					 NONE, 
					 NONE, 
					 DGbuffer, 
					 sizeof(DGbuffer));
		DGstartTime = millis();
		uint8_t recvAckNum = 0;
		while (millis() - DGstartTime < TIME_TERM * 5)
		{
			DGSetReceive();
			if (DGavailable() && DGrecvData(DGtemp_buf) && DG_rxHeaderFrom == address 
				&& DG_rxHeaderTo == DG_thisAddress && DG_rxHeaderType == REQUEST_DIRECT_ACK)
			{
				recvAckNum++;
				if (recvAckNum >= R_MASTER_SEND_NUM * 0.8)
				{
					DGprintRecvPacketHeader();
					uint8_t masterNumber = address;
					printf("%d master is 1 hop node\r\n", masterNumber);
					DGcheckReceive[masterNumber] = true;
					DGparentMaster[masterNumber] = DG_thisAddress;
					DGaddRouteTo(DG_rxHeaderFrom, DG_rxHeaderFrom, Valid, 1, millis());
					DGprintRoutingTable();
					DGprintTree();
					j = DEFAULT_RETRIES;
					oneHop = true;
					return true;
				}
			}
		}
	}
	
	// 1hop요청 후에 응답이 안오면 1hop 노드들에게 해당 노드를 차례대로 요청
	if(!oneHop)
	{
		uint8_t row_number = 1; // 처음엔 1hop노드들을 알아와서 그 노드들에게 물어봄
		uint8_t number_of_node;
		uint8_t node_list[34] = { 0 };

		number_of_node = DGG_get_i_row_node_list(row_number, node_list);
														
		while (number_of_node != 0) 
		{
			if (DGG_request_path_one_by_one(address, row_number, node_list, number_of_node, REQUEST_DIRECT))
				return true;
			row_number++;
			number_of_node = DGG_get_i_row_node_list(row_number, node_list);
		}
	}
	return false;
}
////////////////////////////////////////////운영중 routing 문제 발생 시//////////////////////////////////////////////////////////////



/****************************************************************
*FUNCTION NAME : DGG_discoverNewPath(uint8_t address)
*FUNCTION      : 다시 경로 설정, 
								 다시 경로 설정하는 노드의 자식 노드들에 대한 라우팅 테이블도 수정
*INPUT         : address (목적지 주소)
*OUTPUT        : 성공 여부
****************************************************************/
bool DGG_discoverNewPath(uint8_t address)
{
	if(DG_requestDirect(address))
	{
		for(uint8_t i = 0;i < DG_rxHeaderData;i++)
		{
			DGaddRouteTo(DGtemp_buf[i + 1], DG_rxHeaderFrom, Valid, 0, millis()); 
			// 자식노드의 hop count는 별로 중요하지 않음 일단 0으로 해두고 다음에 해당 자식노드로부터 패킷을 받게되면 hop count에 맞게 setting
		}
		return true;
	}
	return false;
}


/****************************************************************
*FUNCTION NAME : DGchangeNextHop(uint8_t address, uint8_t hopCount)
*FUNCTION      : 라우팅 테이블에서 address의 next_hop을 바꿔줌
*INPUT         : address (주소)
								 hopCount (원래 address의 hop count에서 새로운 hop count를 뺀 값)
*OUTPUT        : NONE
****************************************************************/
void DGchangeNextHop(uint8_t address, uint8_t hopCount)
{
	//Serial.println("changeNextHop");
	//Serial.print("hopCount : ");
	//Serial.println(hopCount);
	//DGprintRoutingTable();
	for (int i = 1; i <= DGmaster_num; i++)
	{
		if (DGparentMaster[i] == address)
		{
			printf("%d\r\n%d\r\n", hopCount, DGgetRouteTo(i)->hop);
			DGaddRouteTo(i, DGgetRouteTo(address)->next_hop, Discovering, DGgetRouteTo(i)->hop - hopCount, millis());
			DGchangeNextHop(i, hopCount);
		}
	}
	DGprintRoutingTable();
}


/****************************************************************
*FUNCTION NAME : DGG_find_error_node(uint8_t address)
*FUNCTION      : error_node를 찾아서 rerouting 함수 호출 
							   (time out이 발생할 경우 어떤 노드에서 전송실패가 발생했는지 찾아서 
									그 노드를 다시 라우팅 해줌)
*INPUT         : address로 가는 도중에 전송이 끊김
*OUTPUT        : NONE
****************************************************************/
void DGG_find_error_node(uint8_t address)
{
	printf("find_error_node\r\n");
	uint8_t path[34] = { 0 };
	DGgetPath(address, path);
	for (int i = 0; i < DGgetRouteTo(address)->hop; i++)
	{
		printf("%d\r\n", path[i]);
	}
	for (int i = DGgetRouteTo(address)->hop - 1; i >= 0; i--)
	{
		if (!DGsendToWaitAck(DG_thisAddress, 
												 DGgetRouteTo(path[i])->next_hop, 
												 DG_thisAddress, 
												 path[i], 
												 CHECK_ROUTING, 
												 NONE, 
												 NONE, 
												 NONE, 
												 NONE, 
												 DGtemp_buf, 
												 sizeof(DGtemp_buf), 
												 TIME_TERM))
		{
			if (DGunRecvCnt[path[i]]++ > MAX_UN_RECV)
			{
				DGcheckReceive[path[i]] = false;
				DGparentMaster[path[i]] = -1;
				DGG_discoverNewPath(path[i]);
				DGunRecvCnt[path[i]] = 0;
			}
			break;
		}
		if (DGgetRouteTo(path[i])->hop == 1)
		{
			DGsend(DG_thisAddress, 
						 DGgetRouteTo(path[i])->next_hop, 
						 DG_thisAddress, 
						 DGgetRouteTo(path[i])->next_hop, 
						 ACK, 
						 NONE, 
						 NONE, 
						 NONE, 
						 NONE, 
						 DGtemp_buf, 
						 sizeof(DGtemp_buf));
			continue;
		}
		DGstartTime = millis();
		while (millis() - DGstartTime < DEFAULT_RETRIES * DGgetRouteTo(path[i])->hop * TIME_TERM * 4)
		{
			DGSetReceive();
			if (DGavailable())
			{
				if (DGrecvData(DGtemp_buf) && DG_rxHeaderTo == DG_thisAddress)
				{
					if (DG_rxHeaderType == NACK)
					{
						DGsend(DG_thisAddress, 
									 DGgetRouteTo(path[i])->next_hop, 
									 DG_thisAddress, 
									 DGgetRouteTo(path[i])->next_hop, 
									 ACK, 
									 NONE, 
									 NONE, 
									 NONE, 
									 NONE, 
									 DGtemp_buf, 
									 sizeof(DGtemp_buf));
						
						printf("NACK\r\n");
						uint8_t temp_address = DGtemp_buf[0];
						if (DGunRecvCnt[temp_address]++ > MAX_UN_RECV)
						{
							DGcheckReceive[temp_address] = false;
							DGparentMaster[temp_address] = -1;
							DGG_discoverNewPath(temp_address);
							DGunRecvCnt[temp_address] = 0;
							return;
						}
					}
				}
			}
		}
	}
}


/****************************************************************
*FUNCTION NAME : DGM_findCandidateParents()
*FUNCTION      : 마스터가 패킷 수신율 or rssi를 고려하여 후보 부모 노드를 선택
*INPUT         : NONE
*OUTPUT        : NONE
****************************************************************/
void DGM_findCandidateParents()
{
	//Serial.println("choose candidate");
	if (DG_rxHeaderType == REQUEST_ACK_TYPE || DG_rxHeaderType == R2_REQUEST_ACK_TYPE || DG_rxHeaderType == REQUEST_MULTI_HOP_ACK)
	{
		//Serial.print("choose candidate2		");
		//Serial.print(DGcandidateRSSI);
		//Serial.print(" : ");
		//Serial.println(e_rssi);
		if (DGreceivedType == 0)
		{
			//Serial.println("receive propagation");
			DGreceivedType = DG_rxHeaderType;
			DGcandidateAddress = DG_rxHeaderFrom;
			DGcandidateRSSI = e_rssi;
			//Serial.print("My choice : ");
			//Serial.println(DGcandidateAddress, HEX);
		}
		else if (DGreceivedType == DG_rxHeaderType) // Type check 안하면, 계속 가장 인접한 노드를 부모로 선택하게 되어서 hop 수가 늘어남
		{
			if (e_rssi >= DGcandidateRSSI)
			{
				//Serial.println("receive propagation");
				DGcandidateAddress = DG_rxHeaderFrom;
				DGcandidateRSSI = e_rssi;
				//Serial.print("My choice : ");
				//Serial.println(DG_rxHeaderFrom, HEX);
			}
		}
	}
}


/****************************************************************
*FUNCTION NAME : DGM_find2ndRowMasters()
*FUNCTION      : 1hop master node가 2hop master node를 찾음
*INPUT         : NONE
*OUTPUT        : NONE
****************************************************************/
void DGM_find2ndRowMasters()
{
	uint8_t childNodeList[32] = { 0 };
	uint8_t unknown_address;
	uint8_t temp[20];
	printf("receive R2 request and I succeed R1\r\n");

	uint8_t count = DG_rxHeaderData;
	uint8_t child_node_num = 0;

	for (int8_t i = 0; i < count; i++) // DGtemp_buf에 수신되는 data가 계속 들어오므로 temp에 복사해두어야함
		temp[i] = DGtemp_buf[i];
	for (int8_t i = 0; i < count; i++)
	{
		printf("send R2 reequest to 2 hop\r\n");
		unknown_address = temp[i];
		uint8_t recvAckNum = 0;
		for (int j = 0; j < 1; j++)
		{
			printf("retry : %d\r\n", j);
			recvAckNum = 0;
			DGsend(DG_thisAddress, 
						 unknown_address, 
						 DG_thisAddress & 0x00, 
						 unknown_address, 
						 R2_REQUEST_REAL_TYPE, 
						 NONE, 
						 NONE, 
						 NONE, 
						 1, 
						 DGtemp_buf, 
						 sizeof(DGtemp_buf));
			
			DGstartTime = millis();
			while (millis() - DGstartTime < TIME_TERM * 5)
			{
				DGSetReceive();
				if (DGavailable() && DGrecvData(DGtemp_buf) && DG_rxHeaderFrom == unknown_address 
					&& DG_rxHeaderTo == DG_thisAddress && DG_rxHeaderType == R2_REQUEST_ACK_TYPE)
				{
					recvAckNum++;
					if (recvAckNum >= R_MASTER_SEND_NUM * 0.8)
					{
						DGaddRouteTo(DG_rxHeaderFrom, DG_rxHeaderFrom, Valid, 1, millis());
						DGprintRecvPacketHeader();
						DGprintRoutingTable();
						childNodeList[child_node_num++] = DG_rxHeaderFrom;;
						j = DEFAULT_RETRIES;
						break;
					}
				}
			}
			delay(TIME_TERM);
		}
	}
	for (int8_t i = 0; i < child_node_num; i++)
		DGtemp_buf[i] = childNodeList[i];
									//from, to, src, dst, type, data, flags, seqnum, hop
	DGsendToWaitAck(DG_thisAddress, 
									DG_thisAddress & 0x00, 
									DG_thisAddress, 
									DG_thisAddress & 0x00, 
									R2_REQUEST_ACK_TYPE, 
									child_node_num, 
									NONE, 
									NONE, 
									1, 
									DGtemp_buf, 
									sizeof(DGtemp_buf), 
									TIME_TERM);
}


/****************************************************************
*FUNCTION NAME : DGM_masterSendRoutingReply()
*FUNCTION      : 2hop 이상인 노드들이 라우팅 요청 패킷을 받았을 경우
								 자신이 정한 후보 부모 노드로부터 라우팅 요청 메시지를
								 받거나 정해놓은 후보 부모 노드가 없을 경우에 해당 노드에게 응답을 보냄 
								 (R_MASTER_SEND_NUM번 보냄)
*INPUT         : NONE
*OUTPUT        : NONE
****************************************************************/
void DGM_masterSendRoutingReply()
{
	//Serial.print("My candidate : ");
	//Serial.println(DGcandidateAddress, HEX);
	//Serial.print("Receved from : ");
	//Serial.println(DG_rxHeaderFrom, HEX);
	if (DGcandidateAddress == DG_rxHeaderFrom || DGcandidateAddress == 0)
	{
		DGaddRouteTo(DG_rxHeaderSource, DG_rxHeaderFrom, Valid, DG_rxHeaderHop + 1, millis());
		DGprintRoutingTable();
		uint8_t type;
		if (DG_rxHeaderType == R2_REQUEST_REAL_TYPE)
			type = R2_REQUEST_ACK_TYPE;
		else if (DG_rxHeaderType == REQUEST_MULTI_HOP)
		{
			type = REQUEST_MULTI_HOP_ACK;
			DGtemp_buf[0] = DG_rxHeaderFrom;
		}
		//from, to, src, dst, type, data, flags, seqnum, hop
		for (uint8_t i = 0; i < R_MASTER_SEND_NUM; i++)
			DGsend(DG_thisAddress, 
						 DG_rxHeaderFrom, 
						 DG_thisAddress, 
						 DG_rxHeaderFrom, 
						 type, 
						 NONE, 
						 NONE, 
						 NONE, 
						 NONE, 
						 DGtemp_buf, 
						 sizeof(DGtemp_buf));
	}
}


/****************************************************************
*FUNCTION NAME: DGnewMaster()
*FUNCTION     : 운영 중 새로운 마스터가 추가될 때 그 마스터가 경로설정을 위해 호출
*INPUT        : NONE
*OUTPUT       : NONE
****************************************************************/
void DGnewMaster()
{
	unsigned long waitTime = millis();
	uint8_t receivedNum[34] = { 0 };
	uint16_t candidate_parent;
	uint8_t maxReceiveNum = 0;
	uint8_t hop = 0;

	while (millis() - waitTime < 10000) //일정 시간동안 패킷을 수신해보고 누구의 패킷을 많이 받았는지 카운트 함
	{
		DGSetReceive();
		if (DGavailable())
			if (DGrecvData(DGtemp_buf))
				receivedNum[DG_rxHeaderFrom]++;
	}

	for (uint8_t i = 1; i <= 34; i++) // 가장 패킷을 많이 수신할 수 있었던 노드를 게이트웨이로 가는 후보 부모 노드로 설정
	{
		if (maxReceiveNum < receivedNum[i])
		{
			candidate_parent = i;
			maxReceiveNum = receivedNum[i];
			printf("i : %d\r\n", i);
		}
	}

	// 후보 부모 노드에게 패킷을 보내봄
	while (!DGsendToWaitAck(DG_thisAddress, 
													candidate_parent, 
													DG_thisAddress, 
													candidate_parent, 
													NEW_NODE_REGISTER, 
													NONE, 
													NONE, 
													NONE, 
													NONE, 
													DGtemp_buf, 
													sizeof(DGtemp_buf), 
													TIME_TERM))
	{
		
		// 선택한 후보 부모 노드로부터 응답을 못받았으면 위의 과정을 반복함
		while (millis() - waitTime < 10000)
		{
			DGSetReceive();
			if (DGavailable())
				if (DGrecvData(DGtemp_buf))
					receivedNum[DG_rxHeaderFrom]++;
		}

		for (uint8_t i = 1; i <= 34; i++)
		{
			if (maxReceiveNum < receivedNum[i])
			{
				candidate_parent = i;
				maxReceiveNum = receivedNum[i];
			}
		}
	}

	// 선택한 후보 부모노드에게 응답을 받았으면 후보 부모 노드를 라우팅 테이블에 등록
	DGaddRouteTo(DG_thisAddress & 0x00, candidate_parent, Valid, DG_rxHeaderHop + 1, millis());
	DGprintRoutingTable();

	
	// 다른 마스터처럼 게이트웨이로부터 외부 스캔 요청 메시지를 받을 수 있는지 여부 확인
	waitTime = millis();
	bool receiveFromG = false;

	while (millis() - waitTime < 600000)
	{
		DGSetReceive();
		if (DGavailable())
		{
			if (DGrecvData(DGtemp_buf))
			{
				if (DG_rxHeaderTo == DG_thisAddress && DG_rxHeaderType == CHECK_ROUTING)
				{
					DGsendToWaitAck(DG_thisAddress, 
													DGgetRouteTo(DG_thisAddress & 0x00)->next_hop, 
													DG_thisAddress, 
													DG_thisAddress & 0x00, 
													CHECK_ROUTING_ACK, 
													NONE, 
													NONE, 
													NONE, 
													NONE, 
													DGtemp_buf, 
													sizeof(DGtemp_buf), 
													TIME_TERM);
					receiveFromG = true;
					break;
				}
			}
		}
	}
	
	if (!receiveFromG) // 10분이 지나도 받지 못할 경우 이 함수를 다시 호출시켜 새로운 후보 부모 노드를 찾음
		DGnewMaster();
}
