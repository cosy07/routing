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
*FUNCTION      : �ּ� ������ �ּ� �ֱ�, ä�� ������ ���� �ʱ�ȭ
*INPUT         : gatewayNumber(�� ��ȣ), thisAddress(������ �� �ּ�), ch(����� ä��)
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
*FUNCTION      : ��� ���(cc1120)�� ���Ű��� ���·�
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
*FUNCTION      : ��Ŷ�� ���������� Ȯ��
*INPUT         : NONE
*OUTPUT        : ���� ���̶�� true return
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
*FUNCTION      : dest�� ����� ���̺� ���
*INPUT         : dest(������), next_hop, state, hop(hop count), lastTime
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
*FUNCTION      : ����� ���̺��� dest�� �ش��ϴ� ���� ã����
*INPUT         : dest(������)
*OUTPUT        : ����� ���̺��� dest�� �ش��ϴ� ���� �ּ�
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
*FUNCTION      : ����� ���̺� ��Ʈ���� ����
*INPUT         : ������� �ϴ� ���� index
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
*FUNCTION      : ����� ���̺� ��ü�� ���
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
*FUNCTION      : ����� ���̺��� dest�� ����
*INPUT         : dest (������� �ϴ� �ּ�)
*OUTPUT        : ���� ����
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
*FUNCTION      : ����� ���̺� clear
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
*FUNCTION      : �����Ⱓ ���� ������ �ʴ� ����� ���̺��� ��Ʈ���� ����
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
*FUNCTION      : �����͵��� �ڽĳ�带 ã�� 
(������ ���忡���� ����� ���̺� ��Ʈ�� �� ����Ʈ���̸� ������ �������� ��� �ڽ� ����)
*INPUT         : ã�� �ڽ� ������ �ּҸ� ���� ������
*OUTPUT        : �ڽĳ�� ����
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

*FUNCTION      : ������ ����
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
	while (millis() - DGsendingTime < 1000); // 20�Ͽ� �� ���� reset�ȴٰ� �ϸ� sendingTime�� type�� int���� ��
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

//����
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

*FUNCTION      : ���� �� hop to hop ACK�� ���� ������ ���
*INPUT         : from, to, src, dst, type, data, flags, seqNum, hoop, DGtemp_buf, size, time
*OUTPUT        : ACK�޾Ҵ��� ����
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
*FUNCTION      : ��Ŷ ��� ���
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
*FUNCTION      : ����Ʈ���̰� �� ���� ��������� ��� ���
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
*FUNCTION      : address�� ��������� ��θ� path�� ����, G�� ���
								 if G -> M1 -> M2 -> M3 �� �� address�� M3���
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
*FUNCTION      : tree���·� ��½����ִ� tool����� ���� �Լ�
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
*FUNCTION      : ����Ʈ���̰� �ʱ� ����ð����� �����ϴ� �Լ�
								 DGG_find_1stRow_master()
								 DGG_find_2ndRow_master()
								 DGG_find_multihop_node()
								 DG_requestDirect()
								 �� ���ʷ� ȣ���ϸ� hop count�� ���� ������ ã�� ������.
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
				
				// multi hop ����� ��û�Ŀ��� ��� ������ ���� ���� ���鿡�Դ� 1hop��û �޽������� �����Ͽ� �ٽ� ��� ������ ��û��
				printf("REQUEST_DIRECT-------------------------------\r\n");
				
				for (int i = 1; i <= DGmaster_num; i++)
					if (!DGcheckReceive[i]) //��� ������ ���� ���� ���
						DG_requestDirect(i);
			}
		}
	}
}


/****************************************************************
*FUNCTION NAME : DGG_find_1stRow_master()
*FUNCTION      : Gateway�� 1hop ������ ã�� 
								 ����� ���� �޼����� R_GATEWAY_SEND_NUM��ŭ ���� ��
                 �� master�鿡�� 1:1�� ����� ��û ��Ŷ�� ����
								 master�κ��� ������ �����ϸ� ��Ŷ �������� ����Ͽ� ����� ���̺� ����
*INPUT         : NONE
*OUTPUT        : ������� �Ϸ�� 1 hop ������ ����
****************************************************************/
int8_t DGG_find_1stRow_master()
{
	uint8_t num_of_routed_nodes = 0;
	printf("send broadcast request message\r\n");
	printf("%d\r\n", DGmasterBroadcastAddress);
	for (int i = 0; i < R_GATEWAY_SEND_NUM; i++)
	{// �����͵��� ��Ŷ ������ ����� ���ؼ� R_GATEWAY_SEND_NUM�� ����� ���� ��Ŷ�� ����
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
		// �� ���鿡�� 1:1�� ��� ������ ��û
		
		DGstartTime = millis();
		recvAckNum = 0;
		
		//����ޱ⸦ ��ٸ�
		while (millis() - DGstartTime < TIME_TERM * R_MASTER_SEND_NUM)
		{
			DGSetReceive();
			if (DGavailable() && DGrecvData(DGtemp_buf) && DG_rxHeaderFrom == dst 
				&& DG_rxHeaderTo == DG_thisAddress && DG_rxHeaderType == REQUEST_ACK_TYPE)
			{
				recvAckNum++; // ���ŵ� ���� ��Ŷ ������ ī��Ʈ��
				if (recvAckNum >= R_MASTER_SEND_NUM * 0.8) // ����� ���� ��Ŷ �������� 80%�̻��� ��쿡�� ����� ���̺� ��Ͻ�Ŵ
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
*FUNCTION      : Gateway�� 1hop ���鿡�� ������� �ȵ� ������ �ּҸ� payload�� �־� ������
*INPUT         : unknown_node_list (������� ���� ���� ����� �ּҸ� ���� �п�)
*OUTPUT        : routing�ȵ� ����� ����
****************************************************************/
// <2hop>
// 1. (G) -----R2_REQUEST_TYPE-----> (1hop master)
// 2. (G) <-----------ACK----------- (1hop master)
// 3.                                (1hop master) -----R2_REQUEST_REAL_TYPE-----> (2hop master)
// 4.                                (1hop master) <-----R2_REQUEST_ACK_TYPE------ (2hop master)
// 5. (G) <--R2_REQUEST_ACK_TYPE---- (1hop master)

int8_t DGG_find_2ndRow_master(uint8_t* unknown_node_list)
{
	//row2 : 1hop�� master�鿡�� ���� ������� �ȵ� ������ �ּҸ� ������ ��ó�� ������ �ִ��� ã�ƴ޶�� ��û
	printf("[row2]\r\n");

	uint8_t node_list[34] = { 0 };
	uint8_t number_of_node;
	uint8_t number_of_unknown_node = 0;

	number_of_node = DGG_get_i_row_node_list(1, node_list);

	printf("1hop's num : %d\r\n", number_of_node);

	for (uint8_t i = 0; i < number_of_node; i++) // 1hop node�鿡�� 2hop ������ ã�ƴ޶�� ��û
	{
		number_of_unknown_node = 0;
		for (uint8_t j = 1; j <= DGmaster_num; j++) // routing�� �ȵ� ������ �ּҸ� ����� ���̺��� ������ �˾Ƴ�
		{
			if (!DGcheckReceive[j])
			{
				uint8_t address = j;
				printf("unreceive : %d\r\n", address);
				DGtemp_buf[number_of_unknown_node] = address; // ������� �ȵ� ����� �ּҸ� payload�� ����
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
		{ //1hop�� ��忡�� ��û�� ������ ACK�� ���� ������ ���
			DGcheckReceive[node_list[i]] = false;
			DGparentMaster[node_list[i]] = 0;
			continue;
		}

		while (1)
		{
			DGSetReceive();
			if (DGavailable())
			{
				if (DGrecvData(DGtemp_buf)) // 1hop�� ���κ��� ��û�� ���� ������ ����
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
						
						// _rxHeaderData�� ã�� ����� ������ ��� ����
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
- ���� 2nd row node�鿡�� ���� ������� �ȵ� ����� �ּҸ� payload�� �����Ͽ� �����Ѵ�.
- 3rd row node�� ���� �Ŀ���, 3rd row node���� ��û�Ѵ�.
(�ֱٿ� ���� row�� ���鿡�� multi hop ����� ��û �޽����� ����)
*INPUT         : number_of_unknown_node(������� �ȵ� ����� ����)
								 unknown_node_list(������� �� �� ������ �ּ�)
*OUTPUT        : ����� �ȵ� ����� ����
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
				// ��� �ϳ��ϳ� ������ ��û��
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
*FUNCTION      : node_list�� �ִ� ���鿡�� �ּҰ� unknown_address�� ��尡 ������ �ִ��� ���
*INPUT         : unknown_address (������ �ּ�) -> ��� ������ �ؾ� �� ���
								 row_number (��û�ϴ� �޽����� ���� ������ hop count)
								 node_list (��û�� ������ �ּ�) 
								 -> ��� ������ �� ���� ��, unknown_address ��尡 �ֺ��� �ִ��� ã�ƾ� �� ���
								 number_of_node (node_list�� ����ִ� ����� ����)
								 type (node_list���� unknown_address���� ������ ��Ŷ�� type)
*OUTPUT        : ���� ����
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
*FUNCTION      : hop count�� �������Ѱ��� ��� ���� ��û
*INPUT         : address (������ �ּ�)
*OUTPUT        : ���� ����
****************************************************************/
bool DG_requestDirect(uint8_t address)
{
	bool oneHop = false;
	// 1hop���� �����ϰ� direct�� ���
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
	
	// 1hop��û �Ŀ� ������ �ȿ��� 1hop ���鿡�� �ش� ��带 ���ʴ�� ��û
	if(!oneHop)
	{
		uint8_t row_number = 1; // ó���� 1hop������ �˾ƿͼ� �� ���鿡�� ���
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
////////////////////////////////////////////��� routing ���� �߻� ��//////////////////////////////////////////////////////////////



/****************************************************************
*FUNCTION NAME : DGG_discoverNewPath(uint8_t address)
*FUNCTION      : �ٽ� ��� ����, 
								 �ٽ� ��� �����ϴ� ����� �ڽ� ���鿡 ���� ����� ���̺� ����
*INPUT         : address (������ �ּ�)
*OUTPUT        : ���� ����
****************************************************************/
bool DGG_discoverNewPath(uint8_t address)
{
	if(DG_requestDirect(address))
	{
		for(uint8_t i = 0;i < DG_rxHeaderData;i++)
		{
			DGaddRouteTo(DGtemp_buf[i + 1], DG_rxHeaderFrom, Valid, 0, millis()); 
			// �ڽĳ���� hop count�� ���� �߿����� ���� �ϴ� 0���� �صΰ� ������ �ش� �ڽĳ��κ��� ��Ŷ�� �ްԵǸ� hop count�� �°� setting
		}
		return true;
	}
	return false;
}


/****************************************************************
*FUNCTION NAME : DGchangeNextHop(uint8_t address, uint8_t hopCount)
*FUNCTION      : ����� ���̺��� address�� next_hop�� �ٲ���
*INPUT         : address (�ּ�)
								 hopCount (���� address�� hop count���� ���ο� hop count�� �� ��)
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
*FUNCTION      : error_node�� ã�Ƽ� rerouting �Լ� ȣ�� 
							   (time out�� �߻��� ��� � ��忡�� ���۽��а� �߻��ߴ��� ã�Ƽ� 
									�� ��带 �ٽ� ����� ����)
*INPUT         : address�� ���� ���߿� ������ ����
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
*FUNCTION      : �����Ͱ� ��Ŷ ������ or rssi�� ����Ͽ� �ĺ� �θ� ��带 ����
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
		else if (DGreceivedType == DG_rxHeaderType) // Type check ���ϸ�, ��� ���� ������ ��带 �θ�� �����ϰ� �Ǿ hop ���� �þ
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
*FUNCTION      : 1hop master node�� 2hop master node�� ã��
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

	for (int8_t i = 0; i < count; i++) // DGtemp_buf�� ���ŵǴ� data�� ��� �����Ƿ� temp�� �����صξ����
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
*FUNCTION      : 2hop �̻��� ������ ����� ��û ��Ŷ�� �޾��� ���
								 �ڽ��� ���� �ĺ� �θ� ���κ��� ����� ��û �޽�����
								 �ްų� ���س��� �ĺ� �θ� ��尡 ���� ��쿡 �ش� ��忡�� ������ ���� 
								 (R_MASTER_SEND_NUM�� ����)
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
*FUNCTION     : � �� ���ο� �����Ͱ� �߰��� �� �� �����Ͱ� ��μ����� ���� ȣ��
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

	while (millis() - waitTime < 10000) //���� �ð����� ��Ŷ�� �����غ��� ������ ��Ŷ�� ���� �޾Ҵ��� ī��Ʈ ��
	{
		DGSetReceive();
		if (DGavailable())
			if (DGrecvData(DGtemp_buf))
				receivedNum[DG_rxHeaderFrom]++;
	}

	for (uint8_t i = 1; i <= 34; i++) // ���� ��Ŷ�� ���� ������ �� �־��� ��带 ����Ʈ���̷� ���� �ĺ� �θ� ���� ����
	{
		if (maxReceiveNum < receivedNum[i])
		{
			candidate_parent = i;
			maxReceiveNum = receivedNum[i];
			printf("i : %d\r\n", i);
		}
	}

	// �ĺ� �θ� ��忡�� ��Ŷ�� ������
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
		
		// ������ �ĺ� �θ� ���κ��� ������ ���޾����� ���� ������ �ݺ���
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

	// ������ �ĺ� �θ��忡�� ������ �޾����� �ĺ� �θ� ��带 ����� ���̺� ���
	DGaddRouteTo(DG_thisAddress & 0x00, candidate_parent, Valid, DG_rxHeaderHop + 1, millis());
	DGprintRoutingTable();

	
	// �ٸ� ������ó�� ����Ʈ���̷κ��� �ܺ� ��ĵ ��û �޽����� ���� �� �ִ��� ���� Ȯ��
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
	
	if (!receiveFromG) // 10���� ������ ���� ���� ��� �� �Լ��� �ٽ� ȣ����� ���ο� �ĺ� �θ� ��带 ã��
		DGnewMaster();
}
