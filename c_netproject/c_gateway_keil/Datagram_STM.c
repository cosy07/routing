//2017-07-17
// Datagram.cpp
//
// Copyright (C) 2011 Mike McCauley
// $Id: RHDatagram.cpp,v 1.6 2014/05/23 02:20:17 mikem Exp $

#include "Datagram_STM.h"


////////////////////////////////////////////////////////////////////
// Public methods
/*void DatagramInit(uint8_t gatewayNumber, uint8_t thisAddress)
{
	//DG_driver = driver;
	DG_thisAddress = thisAddress;
	DGgatewayNumber = gatewayNumber;


	DGmaster_num = 2;
	DGcandidateAddress = 0;
	DGcandidateRSSI = 0;

	DGreceivedType = 0;//To avoid receiving from Master that has same hop

	DGgatewayNumber = 0;

	DGsendingTime = -60000;
	for (int i = 0; i < 34; i++)
	{
		DGcheckReceive[i] = false;
		DGparentMaster[i] = -1;
		DGunRecvCnt[i] = 0;
	}




	//_retransmissions = 0;
	DGclearRoutingTable();
	//DGgatewayNumber = thisAddress >> 10;
	DGmasterBroadcastAddress = 0xFF;//= convertToAddress(DGgatewayNumber, 31, 0);


	for (int i = 0; i < 34; i++)
	{
		DGcheckReceive[i] = false;
		DGparentMaster[i] = -1;
		DGunRecvCnt[i] = 0;
	}

	DGsetThisAddress(DG_thisAddress);
	e_Init();

}*/
////////////////////////////////////////////////////////////////////
// Public methods
void DGInit(uint8_t gatewayNumber, uint8_t thisAddress, byte ch)  // Setup Channel number
{
	//DG_driver = driver;
	DG_thisAddress = thisAddress;
	DGgatewayNumber = gatewayNumber;


	DGmaster_num = 1;
	DGcandidateAddress = 0;
	DGcandidateRSSI = 0;

	DGreceivedType = 0;//To avoid receiving from Master that has same hop

	DGgatewayNumber = 0;

	DGsendingTime = -60000;
	for (int i = 0; i < 34; i++)
	{
		DGcheckReceive[i] = false;
		DGparentMaster[i] = -1;
		DGunRecvCnt[i] = 0;
	}




	//_retransmissions = 0;
	DGclearRoutingTable();
	//DGgatewayNumber = thisAddress >> 10;
	DGmasterBroadcastAddress = 0xFF;//= convertToAddress(DGgatewayNumber, 31, 0);

	DGsetThisAddress(DG_thisAddress);
	e_Init(ch);
	ch = ch;
}

////////////////////////////////////////////////////////////////////
// Public methods
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
bool DGavailable()
{
	/////test?/////
	//return true;
	/////test?/////

	if (e_CheckReceiveFlag()) {
		return true;
	}
	else {
		return false;
	}

}
////////////////////////////////////////////////////////////////////
/****************************************************************
*FUNCTION NAME:  recvData (OLD recvfrom(buf, len, &_from, &_to, &_id, &_flags)) )
*FUNCTION     : It receives data and stores data into buf,
From, To, Source, Type, Data, Flags, SeqNum are stored.
*INPUT        :none
*OUTPUT       : The size of received data excluding headers
****************************************************************/

byte  DGrecvData(uint8_t* buf)
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
	//routing ????? ??? ?? ack????(type ?? routing type? ??? ?? operation message)
	//?? node?? ??? node? ??? ??..
	return 0;
}

////////////////////////////////////////////////////////////////////
// Public methods

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

////////////////////////////////////////////////////////////////////
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

////////////////////////////////////////////////////////////////////
void DGdeleteRoute(uint8_t index)
{
	// Delete a route by copying following routes on top of it
	//memcpy(&_routes[index], &_routes[index + 1], sizeof(RoutingTableEntry) * (ROUTING_TABLE_SIZE - index - 1));
	uint8_t i;
	for(i = index;i < ROUTING_TABLE_SIZE;i++)
	{
		_routes[i].dest = _routes[i+1].dest;
		_routes[i].next_hop = _routes[i+1].next_hop;
		_routes[i].hop = _routes[i+1].hop;
		_routes[i].state = _routes[i+1].state;
		_routes[i].lastTime = _routes[i+1].lastTime;
	}
	_routes[ROUTING_TABLE_SIZE - 1].state = Invalid;
}

////////////////////////////////////////////////////////////////////
void DGprintRoutingTable()
{
	//#ifdef HAVE_SERIAL
	uint8_t i;
	for (i = 0; i < ROUTING_TABLE_SIZE; i++)
	{
		printf("%d Dest: %d Next Hop: %d State: %d Hop: %d Last Time: %d\r\n", i, _routes[i].dest, _routes[i].next_hop, _routes[i].state, _routes[i].hop, _routes[i].lastTime);
	}
	//#endif
}

////////////////////////////////////////////////////////////////////
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

////////////////////////////////////////////////////////////////////
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

////////////////////////////////////////////////////////////////////
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
//////////////////////////////////???? ??? ?? ??//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//=====================================================================================================
// 2017-04-26 ver.1
void DGsend(uint8_t from, uint8_t to, uint8_t src, uint8_t dst, uint8_t type, uint8_t data, uint8_t flags, uint8_t seqNum, uint8_t hop, uint8_t* DGtemp_buf, uint8_t size)
{
	if (type >= 10 && DGgetRouteTo(dst) == NULL)
		return;
	while (millis() - DGsendingTime < 1000);//??? ?? 20?? ??? reset???? DGsendingTime ?? int?? ???
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
bool DGsendToWaitAck(uint8_t from, uint8_t to, uint8_t src, uint8_t dst, uint8_t type, uint8_t data, uint8_t flags, uint8_t seqNum, uint8_t hop, uint8_t* DGtemp_buf, uint8_t size, unsigned long time)
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
		//DGsendingTime = millis();//??? ?? 20?? ??? reset???? DGsendingTime ?? int?? ???, TIME_TERM? 1000 ???? ?? ??? ??? ACK ?? ? send???? ??? ?? ? ?? ?? ??? ???? ??

		printf("retry : %d", i);
		e_SendData(DGbuffer, length);
		DGsendingTime = millis();
		DGstartTime = millis();
		while (millis() - DGstartTime < TIME_TERM * 2)
		{
			DGSetReceive();
			if (DGavailable())
			{
				if (DGrecvData(DGtemp_buf) && DG_rxHeaderFrom == to)
				{
					if (DG_rxHeaderTo == DG_thisAddress && (DG_rxHeaderType == ACK || DG_rxHeaderType == REQUEST_ACK_TYPE || DG_rxHeaderType == R2_REQUEST_ACK_TYPE || DG_rxHeaderType == REQUEST_MULTI_HOP_ACK || DG_rxHeaderType == REQUEST_DIRECT_ACK))
					{
						if (DG_rxHeaderType != ACK)
							DGaddRouteTo(DG_rxHeaderFrom, DG_rxHeaderFrom, Valid, 1, millis());
						return true;
					}
					if (type == R2_REQUEST_TYPE && DG_rxHeaderType == R2_REQUEST_REAL_TYPE)//gateway?? round2??? 1hop master?? ?? master? gateway?? ack? ????, ??? ?? ??
						return true;
					if (type == REQUEST_MULTI_HOP && DG_rxHeaderType == REQUEST_MULTI_HOP)
						return true;
					if ((DG_rxHeaderType == CHECK_ROUTING_ACK || DG_rxHeaderType == SCAN_RESPONSE_TO_GATEWAY) && type == CHECK_ROUTING)
					{
						if (DG_thisAddress != (DG_thisAddress & 0x00))//master?
						{
							DGsend(DG_thisAddress, DG_rxHeaderFrom, DG_thisAddress, DG_rxHeaderFrom, ACK, NONE, NONE, NONE, NONE, DGtemp_buf, sizeof(DGtemp_buf));
							if (DGgetRouteTo(DG_rxHeaderDestination) != NULL)
								DGsendToWaitAck(DG_thisAddress, DGgetRouteTo(DG_rxHeaderDestination)->next_hop, DG_rxHeaderSource, DG_rxHeaderDestination, DG_rxHeaderType, NONE, NONE, NONE, DG_rxHeaderHop + 1, DGtemp_buf, sizeof(DGtemp_buf), 2000);
						}
						return true;
					}
					if (type == CHECK_ROUTING && (DG_rxHeaderType == CHECK_ROUTING || DG_rxHeaderType == CHECK_ROUTING_ACK_REROUTING || SCAN_RESPONSE_TO_GATEWAY))
						return true;
					if (type == SCAN_REQUEST_TO_MASTER && DG_rxHeaderType == SCAN_REQUEST_ACK_FROM_MASTER)
						return true;
				}
			}
		}
		delay(time);
	}
	return false;
}
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
void DGprintPath()
{
	uint8_t temp_address;
	printf("-----PATH-----\r\n");
	byte index;
	for (uint8_t i = 1; i <= DGmaster_num; i++)
	{
		printf("%d master : ", i);
		temp_address = i;// convertToAddress(DGgatewayNumber, i, 0);
		while (temp_address != DG_thisAddress)
		{
			printf("%d -> ", temp_address);
			index = temp_address;// convertToMasterNumber(temp_address);
			temp_address = DGparentMaster[index];
		}
		printf("%d\r\n", DG_thisAddress);
	}
	printf("--------------\r\n");
}
void DGgetPath(uint8_t address, uint8_t* path)
{
	uint8_t temp_address = address;
	byte index;
	uint8_t cnt = 0;
	while (temp_address != DG_thisAddress)
	{
		path[cnt++] = temp_address;
		index = temp_address;// convertToMasterNumber(temp_address);
		temp_address = DGparentMaster[index];
	}
}
void DGprintTree()
{
	printf("abcde %d\r\n", 0);// convertToAddress(DGgatewayNumber, 0, 0));
	for (uint8_t i = 1; i <= DGmaster_num; i++)
	{
		if (DGcheckReceive[i])
		{
			printf("edcba %d %d\r\n", DGparentMaster[i], i);// convertToAddress(DGgatewayNumber, i, 0));
		}
	}
}
/****************************************************************
*FUNCTION NAME:  convertToAddress(uint8_t DGgatewayNumber, uint8_t masterNumber,uint8_t slaveNumber)
*FUNCTION     :  2 ??? ??? ??? ??
*INPUT        : ??? ??, ??? ?? ??, ???? ??
*OUTPUT       : 2???  ?? ??
****************************************************************/
/*uint16_t convertToAddress(uint8_t DGgatewayNumber, uint8_t masterNumber, uint8_t slaveNumber)
{
if (DGgatewayNumber > 64 || masterNumber > 31 || slaveNumber > 16)
return -1;

uint16_t address;
address = DGgatewayNumber << 5;
address = address | masterNumber;
address = address << 5;
address = address | slaveNumber;
return address;

}*/
/****************************************************************
*FUNCTION NAME:  convertToMasterNumber(uint16_t address)
*FUNCTION     :  ???? ??? ??? ??? ??
*INPUT        : 2??? ?? ??
*OUTPUT       : ??? ??
****************************************************************/
/*uint16_t convertToMasterNumber(uint16_t address)
{
uint16_t masterNum;
address = address & 0x03FF;
masterNum = (uint16_t)address >> 5;
return masterNum;
}*/
/****************************************************************
*FUNCTION NAME: G_get_i_row_node_list(uint8_t row_number, uint16_t *node_list )
*FUNCTION     : Gatway gets the list of node on the i_th row

*INPUT        : row number
*OUTPUT       : return value: number of node on the i_th row
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
*FUNCTION NAME:FromGatewatyToMaster
*FUNCTION     :
*INPUT        :
*OUTPUT       :
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
				printf("REQUEST_DIRECT-------------------------------\r\n");
				for (int i = 1; i <= DGmaster_num; i++)
				{
					if (!DGcheckReceive[i])
					{
						address_i = i;
						DGtemp_buf[0] = address_i;
						if (DGsendToWaitAck(DG_thisAddress, address_i, DG_thisAddress, address_i, REQUEST_DIRECT, NONE, NONE, NONE, NONE, DGtemp_buf, sizeof(DGtemp_buf), 2000))
						{
							DGprintRecvPacketHeader();
							printf("%d", i); //Serial.println("master is 1 hop node");
							DGcheckReceive[i] = true;
							DGparentMaster[i] = DG_thisAddress;
							DGaddRouteTo(DG_rxHeaderFrom, DG_rxHeaderFrom, Valid, 1, millis());
							DGprintRoutingTable();
						}
						else
						{
							uint8_t row_number = 1;
							uint8_t number_of_node;
							uint8_t node_list[34] = { 0 };

							number_of_node = DGG_get_i_row_node_list(row_number, node_list);  // the gateway needs the list of 2nd row masters which will send Multi_HOP_REQUEST
																							// to unknown nodes.
							while (number_of_node != 0)
							{
								if (DGG_request_path_one_by_one(address_i, row_number, node_list, number_of_node, REQUEST_DIRECT))
									break;
								number_of_node = DGG_get_i_row_node_list(row_number, node_list);
								row_number++;
							}
						}
					}
				}
			}
		}
	}
}
/****************************************************************
*FUNCTION NAME:  G_find_1stRow_master( )
*FUNCTION     : Gateway? broadcast? master??? ??? ?? ???? ??
??? ????.
- ??? ????, ??? ???? ??
*INPUT        : none
*OUTPUT       : 1 hop ???? ??
****************************************************************/
int8_t DGG_find_1stRow_master()
{
	uint8_t num_of_routed_nodes = 0;
	printf("send broadcast request message\r\n");
	printf("%d\r\n", DGmasterBroadcastAddress);
	for (int i = 0; i < R_GATEWAY_SEND_NUM; i++)
	{
		DGsend(DG_thisAddress, DGmasterBroadcastAddress, DG_thisAddress, DGmasterBroadcastAddress, REQUEST_BROADCAST, NONE, NONE, NONE, NONE, DGtemp_buf, sizeof(DGtemp_buf));
	}
	for (int i = 1; i <= DGmaster_num; i++)
	{
		/////////??? ???? ?? ?????? ack 5? ?? ? ?? ??? ???..?? ???, master 2 hop???? ??? ????? 2hop?? 5? ??? ??,,
		uint8_t dst = i;// convertToAddress(DGgatewayNumber, i, 0);
		uint8_t recvAckNum = 0;
		for (int j = 0; j < 1; j++)//DEFAULT_RETRIES; j++)
		{
			printf("retry : %d", j);
			DGsend(DG_thisAddress, dst, DG_thisAddress, dst, REQUEST_TYPE, NONE, NONE, NONE, NONE, DGtemp_buf, sizeof(DGtemp_buf));
			DGstartTime = millis();
			recvAckNum = 0;
			while (millis() - DGstartTime < TIME_TERM * 5)//?? TIME_TERM??? ???? ???? ???? ?????? ??? ???? ??????? ??? ?? ?? ???? ??? ??? ????? ?????? ??..
			{
				DGSetReceive();
				if (DGavailable() && DGrecvData(DGtemp_buf) && DG_rxHeaderFrom == dst && DG_rxHeaderTo == DG_thisAddress && DG_rxHeaderType == REQUEST_ACK_TYPE)
				{
					recvAckNum++;
					if (recvAckNum >= R_MASTER_SEND_NUM * 0.8)
					{
						printf("%d", recvAckNum);
						printf("%d", R_MASTER_SEND_NUM * 0.8);
						DGaddRouteTo(DG_rxHeaderFrom, DG_rxHeaderFrom, Valid, 1, millis());
						DGprintRecvPacketHeader();
						printf("%d master is 1 hop node", i);
						DGcheckReceive[i] = true;
						DGparentMaster[i] = DG_thisAddress;
						DGprintRoutingTable();
						DGprintTree();
						num_of_routed_nodes++;
						j = DEFAULT_RETRIES;
						break;
					}
				}
			}
			delay(TIME_TERM);
		}
	}
	return num_of_routed_nodes;
}
/****************************************************************
*FUNCTION NAME:  G_find_2ndRow_master( )
*FUNCTION     :  Gateway?  1 hop ?????, ???? ?? ???? ??? ????? ???? ???.

*INPUT        : unknown_node for next hop
*OUTPUT       : routing ?? ??? ??
****************************************************************/
int8_t DGG_find_2ndRow_master(uint8_t* unknown_node_list)
{
	printf("[row2]\r\n");
	//row2 : 1hop? master??? ?? ???? ?? ???? ??? ??? ??? ???? ??? ???

	uint8_t node_list[34] = { 0 };
	uint8_t number_of_node;
	uint8_t number_of_unknown_node = 0;

	number_of_node = DGG_get_i_row_node_list(1, node_list);

	printf("1hop's num : %d\r\n", number_of_node);

	for (uint8_t i = 0; i < number_of_node; i++)//1hop node??? ??
	{
		number_of_unknown_node = 0;
		for (int i = 1; i <= DGmaster_num; i++)
		{
			printf("%d ", DGcheckReceive[i]);
		}
		printf("\r\n");
		for (uint8_t j = 1; j <= DGmaster_num; j++)//routing? ?? ???? ??? table? ??? ???
		{
			if (!DGcheckReceive[j])
			{
				uint8_t address = j;// convertToAddress(DGgatewayNumber, j, 0);
									//routing? ?? ??? ??? ???? ??? ???.
				printf("unreceive : %d\r\n", address);
				DGtemp_buf[number_of_unknown_node] = address;
				//DGtemp_buf[number_of_unknown_node * 2 + 1] = (uint8_t)(0x00FF & address);
				number_of_unknown_node++;
			}
		}
		if (number_of_unknown_node == 0)
			return 0;
		printf("send to %d\r\n", node_list[i]);


		//1hop? ?? ???..test ??? ?? ??? ???? ???? ?? ??.
		if (!DGsendToWaitAck(DG_thisAddress, node_list[i], DG_thisAddress, node_list[i], R2_REQUEST_TYPE, number_of_unknown_node, NONE, NONE, NONE, DGtemp_buf, sizeof(DGtemp_buf), 2000))
		{//from, to, src, dst, type, data, flags, seqnum, hop, payload, size of payload, timeout
		 //DGdeleteRouteTo(node_list[i]);
			DGcheckReceive[node_list[i]] = false;
			DGparentMaster[node_list[i]] = 0;
			/*for (uint8_t j = 0; j < i; j++)
			{
			DGtemp_buf[0] = (uint8_t)node_list[i] >> 8;
			DGtemp_buf[1] = (uint8_t)(0x00FF & node_list[i]);
			if (DGcheckReceive[convertToMasterNumber(node_list[j])] && sendToWaitAck(DG_thisAddress, node_list[j], DG_thisAddress, node_list[j], R2_REQUEST_TYPE, 1, NONE, NONE, NONE, DGtemp_buf, sizeof(DGtemp_buf)))
			{
			//DGstartTime = millis();
			//while (millis() - DGstartTime < 10 * TIME_TERM)
			while(1)//???? ???? ?? ????? ??? ???? ??? ??..but ??? 1hop??? ??? ??? while? ?? ?.
			{
			DGSetReceive();
			if (DGavailable())
			{
			if (DGrecvData(DGtemp_buf) && DG_rxHeaderTo == DG_thisAddress && DG_rxHeaderType == R2_REQUEST_ACK_TYPE && DG_rxHeaderFrom == node_list[j])
			{
			send(DG_thisAddress, DG_rxHeaderFrom, DG_thisAddress, DG_rxHeaderFrom, ACK, NONE, NONE, NONE, NONE, DGtemp_buf, sizeof(DGtemp_buf));
			if (DG_rxHeaderData != 0)
			{
			printRecvPacketHeader();
			DGaddRouteTo(node_list[i], node_list[j], Valid, 2);
			uint16_t masterNum = convertToMasterNumber(node_list[i]);
			DGparentMaster[masterNum] = DG_rxHeaderFrom;
			delay(100);
			DGcheckReceive[masterNum] = true;
			}
			break;
			}
			}
			}
			if (DGcheckReceive[convertToMasterNumber(node_list[i])])
			break;
			}
			}*/
			continue;
		}

		//DGstartTime = millis();
		//while ((millis() - DGstartTime) < number_of_unknown_node * 10 * TIME_TERM)

		while (1)//???? ???? ?? ????? ??? ???? ??? ??..but ??? 1hop??? ??? ??? while? ?? ?.
		{
			DGSetReceive();
			if (DGavailable())
			{
				if (DGrecvData(DGtemp_buf))
				{
					if (DG_rxHeaderTo == DG_thisAddress && DG_rxHeaderType == R2_REQUEST_ACK_TYPE && DG_rxHeaderFrom == node_list[i])
					{
						DGsend(DG_thisAddress, DG_rxHeaderFrom, DG_thisAddress, DG_rxHeaderFrom, ACK, NONE, NONE, NONE, NONE, DGtemp_buf, sizeof(DGtemp_buf));
						DGprintRecvPacketHeader();
						for (uint8_t j = 0; j < DG_rxHeaderData; j++)
						{
							uint8_t temp_addr = DGtemp_buf[j];// word(DGtemp_buf[j * 2], DGtemp_buf[j * 2 + 1]);
							DGaddRouteTo(temp_addr, DG_rxHeaderFrom, Valid, 2, millis());
							uint8_t masterNum = temp_addr;// convertToMasterNumber(temp_addr);
							DGparentMaster[masterNum] = DG_rxHeaderFrom;
							DGcheckReceive[masterNum] = true;
							printf("%d \r\n%d \r\n", masterNum, DGcheckReceive[masterNum]);
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
			unknown_node_list[number_of_unknown_node++] = i;// convertToAddress(DGgatewayNumber, i, 0);
		}
	}
	return number_of_unknown_node;
}
/****************************************************************
*FUNCTION NAME: G_find_multihop_node ()
*FUNCTION     :  Gateway finished the routing discovery upto 2nd row.
Gateway tries to find the 3rd and more row nodes.
- ?? 2nd row node?? ???, ?? routing? ?? ??? ???  payload? ???? ????.
- 3rd row ???? ?????, routing? ?? ??? ???  payload? ????
??? ?? row? ?????  multihop routing ?? ???? ????.
*INPUT        :
*OUTPUT       :
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

	number_of_node = DGG_get_i_row_node_list(row_number, node_list);  // the gateway needs the list of 2nd row masters which will send Multi_HOP_REQUEST
																	// to unknown nodes.

	printf("[MULTI_HOP]\r\n");
	while (number_of_node != 0)
	{
		for (uint8_t i = 0; i < cnt; i++)
		{
			if (DGG_request_path_one_by_one(unknown_node_list[i], row_number, node_list, number_of_node, REQUEST_MULTI_HOP))
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
*FUNCTION NAME:  G_request_path_one_by_one(uint16_t unknown_address, byte row_number, uint16_t* node_list, byte number_of_node, uint8_t type)
*FUNCTION     :	 node_list? ?? ????? ??? unknown_address? ??? ??? ??? ???

*INPUT        :  ??? ??
*OUTPUT       :  ?? ??
****************************************************************/
bool DGG_request_path_one_by_one(uint8_t unknown_address, uint8_t row_number, uint8_t* node_list, byte number_of_node, uint8_t type)
{
	//e_Init(ch);
	printf("G_request_path_one_by_one\r\n");
	uint8_t masterNum = unknown_address;// convertToMasterNumber(unknown_address);

	for (uint8_t i = 0; i < number_of_node; i++)
	{
		if (node_list[i] == unknown_address)
			continue;
		if (!DGcheckReceive[node_list[i]])
			continue;
		DGtemp_buf[0] = unknown_address;
		// (uint8_t)(unknown_address >> 8);
		//DGtemp_buf[1] = (uint8_t)(0x00FF & unknown_address);

		uint8_t next_hop = DGgetRouteTo(node_list[i])->next_hop;

		if (!DGsendToWaitAck(DG_thisAddress, next_hop, DG_thisAddress, node_list[i], type, NONE, NONE, NONE, NONE, DGtemp_buf, sizeof(DGtemp_buf), 2000))
		{
			DGcheckReceive[next_hop] = false;
			DGparentMaster[next_hop] = -1;
			DGG_discoverNewPath(next_hop, DGgetRouteTo(next_hop)->hop);
			continue;
		}
		delay(100);
		DGstartTime = millis();
		while ((millis() - DGstartTime) < (row_number + 1) * 4 * DEFAULT_RETRIES * TIME_TERM)
		{
			DGSetReceive();
			if (DGavailable())
			{
				if (DGrecvData(DGtemp_buf) && DG_rxHeaderTo == DG_thisAddress && DG_rxHeaderDestination == DG_thisAddress)
				{
					DGprintRecvPacketHeader();
					DGsend(DG_thisAddress, DG_rxHeaderFrom, DG_thisAddress, DG_rxHeaderFrom, ACK, NONE, NONE, NONE, NONE, DGtemp_buf, sizeof(DGtemp_buf));
					if (DG_rxHeaderSource == unknown_address && (DG_rxHeaderType == REQUEST_MULTI_HOP_ACK || DG_rxHeaderType == REQUEST_DIRECT_ACK) && DG_rxHeaderFrom == next_hop)
					{
						DGaddRouteTo(DG_rxHeaderSource, DG_rxHeaderFrom, Valid, DG_rxHeaderHop + 1, millis());
						DGparentMaster[masterNum] = DGtemp_buf[0];// word(DGtemp_buf[0], DGtemp_buf[1]);
						DGcheckReceive[masterNum] = true;
						DGprintRoutingTable();
						DGprintTree();
						return true;
					}
					else if (DG_rxHeaderType == NACK)
					{
						printf("receive NACK\r\n");
						uint8_t reroutingAddr = DGtemp_buf[0]; //word(DGtemp_buf[0], DGtemp_buf[1]);
															 //DGdeleteRouteTo(reroutingAddr);
						DGcheckReceive[reroutingAddr] = false;
						DGparentMaster[reroutingAddr] = -1;
						DGG_discoverNewPath(reroutingAddr, DGgetRouteTo(reroutingAddr)->hop);
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
/////??? routing ?? ???///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/****************************************************************
*FUNCTION NAME:  G_discoverNewPath(uint16_t address, uint8_t row_number, bool child)
*FUNCTION     :	 ?? routing ??

*INPUT        :
*OUTPUT       :
****************************************************************/
bool DGG_discoverNewPath(uint8_t address, uint8_t row_number)//, bool child)
{
	printf("G_discoverNewPath : %d\r\n", address);
	uint8_t cpy_row_number = row_number;
	uint8_t number_of_node;
	uint8_t node_list[34] = { 0 };

	//row_number = getRouteTo(address)->hop;
	if (row_number != DGgetRouteTo(address)->hop)
	{
		if (number_of_node = DGG_get_i_row_node_list(row_number, node_list))
		{
			if (DGG_request_path_one_by_one(address, row_number, node_list, number_of_node, REQUEST_DIRECT))
				return true;
			else
			{
				DGcheckReceive[address] = false;
				DGparentMaster[address] = -1;
				DGG_discoverNewPath(address, row_number + 1);
			}
		}
		else
		{
			printf("#######################################routing fail###########################################\r\n");
			return false;//?? ??? ?? ???..
		}
	}

	while (row_number > -1)
	{
		number_of_node = DGG_get_i_row_node_list(row_number, node_list);
		/*if (number_of_node == 1 && node_list[0] == address)//address? ?? hop count? ?? ??? ??, ??? G_request_path_one_by_one?? ??
		row_number--;
		else */if (row_number == 0)
		{
			DGtemp_buf[0] = address;
			//DGtemp_buf[1] = (uint8_t)(0x00FF & address);
			uint8_t recvAckNum = 0;
			for (byte i; i < sizeof(DGtemp_buf); i++)
				DGbuffer[i] = DGtemp_buf[i];
			for (int j = 0; j < DEFAULT_RETRIES; j++)
			{
				DGsend(DG_thisAddress, address, DG_thisAddress, address, REQUEST_DIRECT, NONE, NONE, NONE, NONE, DGbuffer, sizeof(DGbuffer));
				DGstartTime = millis();
				recvAckNum = 0;
				while (millis() - DGstartTime < TIME_TERM * 5)//?? TIME_TERM??? ???? ???? ???? ?????? ??? ???? ??????? ??? ?? ?? ???? ??? ??? ????? ?????? ??..
				{
					DGSetReceive();
					if (DGavailable() && DGrecvData(DGtemp_buf) && DG_rxHeaderFrom == address && DG_rxHeaderTo == DG_thisAddress && DG_rxHeaderType == REQUEST_DIRECT_ACK)
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
							break;
						}
					}
				}
			}
			if (recvAckNum < R_MASTER_SEND_NUM * 0.8)
				row_number--;
			break;
		}
		else
		{
			if (DGG_request_path_one_by_one(address, row_number, node_list, number_of_node, REQUEST_DIRECT))
				break;
			row_number--;
		}
	}
	if (row_number == -1)//hop count? ????? ?? ? ?? ??
	{
		DGcheckReceive[address] = false;
		DGparentMaster[address] = -1;
		DGG_discoverNewPath(address, cpy_row_number + 1);
	}

	if (!DGcheckReceive[address])
	{
		printf("%d-th node is error\r\n", address);
	}

	//if (!child)
	//{
	number_of_node = DGG_get_i_row_node_list(cpy_row_number, node_list);
	for (int i = 1; i <= DGmaster_num; i++)//address? ??? path ?? ??
	{
		//Serial.println("child node rerouting!!");
		if (DGparentMaster[i] == address)
		{
			uint8_t temp_address = i;
			DGcheckReceive[temp_address] = false;
			DGparentMaster[temp_address] = -1;
			DGG_discoverNewPath(temp_address, DGgetRouteTo(temp_address)->hop);// , true);
			DGunRecvCnt[temp_address] = 0;
			//changeNextHop(temp_address);
			//G_request_path_one_by_one(convertToAddress(DGgatewayNumber, i, 0), cpy_row_number, node_list, number_of_node, REQUEST_DIRECT);
		}
	}
	//}
	printf("%d\r\nwhere?\r\n", address);
	DGprintTree();
	return true;
}
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
	//DGprintRoutingTable();
}
/****************************************************************
*FUNCTION NAME:  G_find_error_node(uint16_t address)
*FUNCTION     :	 error_node ??? rerouting?? ??

*INPUT        :
*OUTPUT       :
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
		if (!DGsendToWaitAck(DG_thisAddress, DGgetRouteTo(path[i])->next_hop, DG_thisAddress, path[i], CHECK_ROUTING, NONE, NONE, NONE, NONE, DGtemp_buf, sizeof(DGtemp_buf), 2000))
		{
			//DGdeleteRouteTo(path[i]);
			if (DGunRecvCnt[path[i]]++ > MAX_UN_RECV)
			{
				DGcheckReceive[path[i]] = false;
				DGparentMaster[path[i]] = -1;
				DGG_discoverNewPath(path[i], DGgetRouteTo(path[i])->hop);
				DGunRecvCnt[path[i]] = 0;
			}
			break;
		}
		if (DGgetRouteTo(path[i])->hop == 1)
		{
			DGsend(DG_thisAddress, DGgetRouteTo(path[i])->next_hop, DG_thisAddress, DGgetRouteTo(path[i])->next_hop, ACK, NONE, NONE, NONE, NONE, DGtemp_buf, sizeof(DGtemp_buf));
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
						DGsend(DG_thisAddress, DGgetRouteTo(path[i])->next_hop, DG_thisAddress, DGgetRouteTo(path[i])->next_hop, ACK, NONE, NONE, NONE, NONE, DGtemp_buf, sizeof(DGtemp_buf));
						printf("NACK\r\n");
						//DGdeleteRouteTo(path[i]);
						uint8_t temp_address = DGtemp_buf[0];// word(DGtemp_buf[0], DGtemp_buf[1]);
						if (DGunRecvCnt[temp_address]++ > MAX_UN_RECV)
						{
							DGcheckReceive[temp_address] = false;
							DGparentMaster[temp_address] = -1;
							DGG_discoverNewPath(temp_address, DGgetRouteTo(temp_address)->hop);
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
*FUNCTION NAME:  M_findCandidateParents( )   20180308?? ?
*FUNCTION     :

*INPUT        :
*OUTPUT       :
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
		else if (DGreceivedType == DG_rxHeaderType)//Type check ???, ?? ?? ??? ??? ??? ???? ??? hop?? ?? ????.
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
*FUNCTION NAME:  M_find2ndRowMasters( )
*FUNCTION     :

*INPUT        :
*OUTPUT       :
****************************************************************/
void DGM_find2ndRowMasters()
{
	uint8_t childNodeList[32] = { 0 };
	uint8_t unknown_address;
	uint8_t temp[20];
	printf("receive R2 request and I succeed R1\r\n");

	uint8_t count = DG_rxHeaderData;
	uint8_t child_node_num = 0;

	for (int8_t i = 0; i < count; i++)//DGtemp_buf? ???? data? ?? ????? temp? ???????
	{
		temp[i] = DGtemp_buf[i];
		//temp[i * 2 + 1] = DGtemp_buf[i * 2 + 1];
	}
	for (int8_t i = 0; i < count; i++)
	{
		printf("send R2 reequest to 2 hop\r\n");
		unknown_address = temp[i];// * 2], temp[i * 2 + 1]);
		uint8_t recvAckNum = 0;
		for (int j = 0; j < 1; j++)//DEFAULT_RETRIES; j++)
		{
			printf("retry : %d\r\n", j);
			recvAckNum = 0;
			DGsend(DG_thisAddress, unknown_address, DG_thisAddress & 0x00, unknown_address, R2_REQUEST_REAL_TYPE, NONE, NONE, NONE, 1, DGtemp_buf, sizeof(DGtemp_buf));
			DGstartTime = millis();
			while (millis() - DGstartTime < TIME_TERM * 5)
			{
				DGSetReceive();
				if (DGavailable() && DGrecvData(DGtemp_buf) && DG_rxHeaderFrom == unknown_address && DG_rxHeaderTo == DG_thisAddress && DG_rxHeaderType == R2_REQUEST_ACK_TYPE)
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
	{
		DGtemp_buf[i] = childNodeList[i];
		//DGtemp_buf[i * 2 + 1] = lowByte(childNodeList[i]);
	}
	//from, to, src, dst, type, data, flags, seqnum, hop
	DGsendToWaitAck(DG_thisAddress, DG_thisAddress & 0x00, DG_thisAddress, DG_thisAddress & 0x00, R2_REQUEST_ACK_TYPE, child_node_num, NONE, NONE, 1, DGtemp_buf, sizeof(DGtemp_buf), 2000);
}
/****************************************************************
*FUNCTION NAME:M_masterSendRoutingReply( )
*FUNCTION     :

*INPUT        :
*OUTPUT       :
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
			//DGtemp_buf[1] = (uint8_t)(0x00FF & DG_rxHeaderFrom);
		}
		//from, to, src, dst, type, data, flags, seqnum, hop
		for (uint8_t i = 0; i < R_MASTER_SEND_NUM; i++)
		{
			DGsend(DG_thisAddress, DG_rxHeaderFrom, DG_thisAddress, DG_rxHeaderFrom, type, NONE, NONE, NONE, NONE, DGtemp_buf, sizeof(DGtemp_buf));
		}
	}
}

void DGnewMaster()
{
	unsigned long waitTime = millis();
	uint8_t receivedNum[34] = { 0 };
	uint16_t candidate_parent;
	uint8_t maxReceiveNum = 0;
	uint8_t hop = 0;

	while (millis() - waitTime < 10000)
	{
		DGSetReceive();
		if (DGavailable())
		{

			if (DGrecvData(DGtemp_buf))
			{
				receivedNum[DG_rxHeaderFrom]++;
			}
		}
	}

	for (uint8_t i = 1; i <= 34; i++)
	{
		if (maxReceiveNum < receivedNum[i])
		{
			candidate_parent = i;
			maxReceiveNum = receivedNum[i];
			printf("i : %d\r\n", i);
		}
	}

	while (!DGsendToWaitAck(DG_thisAddress, candidate_parent, DG_thisAddress, candidate_parent, NEW_NODE_REGISTER, NONE, NONE, NONE, NONE, DGtemp_buf, sizeof(DGtemp_buf), 1000))
	{
		while (millis() - waitTime < 10000)
		{
			DGSetReceive();
			if (DGavailable())
			{
				if (DGrecvData(DGtemp_buf))
				{
					receivedNum[DG_rxHeaderFrom]++;
				}
			}
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

	DGaddRouteTo(DG_thisAddress & 0x00, candidate_parent, Valid, DG_rxHeaderHop + 1, millis());//master? hop count ??? ??..????...
	DGprintRoutingTable();

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
					DGsendToWaitAck(DG_thisAddress, DGgetRouteTo(DG_thisAddress & 0x00)->next_hop, DG_thisAddress, DG_thisAddress & 0x00, CHECK_ROUTING_ACK, NONE, NONE, NONE, NONE, DGtemp_buf, sizeof(DGtemp_buf), 2000);
					receiveFromG = true;
					break;
				}
			}
		}
	}
	if (!receiveFromG)
		DGnewMaster();
}
//////////////////////