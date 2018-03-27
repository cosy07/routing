//2017-07-17
// Datagram.cpp
//
// Copyright (C) 2011 Mike McCauley
// $Id: RHDatagram.cpp,v 1.6 2014/05/23 02:20:17 mikem Exp $

#include <Datagram_STM.h>

Datagram::Datagram(ELECHOUSE_CC1120& driver, uint16_t thisAddress)//, uint8_t num_of_master)
	:
	_driver(driver),
	_thisAddress(thisAddress)//,
	//_numOfMaster(num_of_master)
{
	//_retransmissions = 0;
	clearRoutingTable();
	gatewayNumber = thisAddress >> 10;
	masterBroadcastAddress = convertToAddress(gatewayNumber, 31, 0);
}

////////////////////////////////////////////////////////////////////
// Public methods
void Datagram::init()
{
	setThisAddress(_thisAddress);
	_driver.Init();

}
////////////////////////////////////////////////////////////////////
// Public methods
void Datagram::init(byte ch)  // Setup Channel number
{
	setThisAddress(_thisAddress);
	_driver.Init(ch);
}

////////////////////////////////////////////////////////////////////
// Public methods
void Datagram::SetReceive()
{
	_driver.SetReceive();
}

void  Datagram::setThisAddress(uint16_t thisAddress)
{
	_driver.setThisAddress(thisAddress);
	// Use this address in the transmitted FROM header
	setHeaderFrom(thisAddress);
	_thisAddress = thisAddress;
}

void  Datagram::setHeaderFrom(uint16_t from)
{
	_driver.setHeaderFrom(from);
}

void Datagram::setHeaderTo(uint16_t to)
{
	_driver.setHeaderTo(to);
}


void Datagram::setHeaderSource(uint16_t source)
{
	_driver.setHeaderSource(source);
}

void Datagram::setHeaderDestination(uint16_t destination)
{
	_driver.setHeaderDestination(destination);
}

void Datagram::setHeaderType(uint8_t type)
{
	_driver.setHeaderType(type);
}

void Datagram::setHeaderData(uint8_t data)
{
	_driver.setHeaderData(data);
}
void Datagram::setHeaderSeqNum(uint8_t seq)
{
	_driver.setHeaderSeqNum(seq);
}
void Datagram::setHeaderFlags(uint8_t flags)
{
	_driver.setHeaderFlags(flags);
}
void Datagram::setHeaderHop(uint8_t hop)
{
	_driver.setHeaderHop(hop);
}

uint16_t Datagram::headerTo()
{
	uint16_t temp = _driver.headerTo();
	return temp;
}
uint16_t Datagram::headerFrom()
{
	uint16_t temp = _driver.headerFrom();
	return temp;
}

uint16_t Datagram::headerSource()
{
	return(_driver.headerSource());
}
uint16_t Datagram::headerDestination()
{
	return(_driver.headerDestination());
}
uint8_t Datagram::headerType()
{
	return(_driver.headerType());
}
uint8_t Datagram::headerData()
{
	return(_driver.headerData());
}
uint8_t  Datagram::headerSeqNum()
{
	return(_driver.headerSeqNum());
}
uint8_t Datagram::headerFlags()
{
	return(_driver.headerFlags());
}
uint8_t Datagram::headerHop()
{
	return(_driver.headerHop());
}
uint16_t Datagram::getThisAddress()
{
	return _thisAddress;
}
bool Datagram::available()
{   //  Serial.println("111  CHECK"); 
	if (_driver.CheckReceiveFlag()) {
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

byte  Datagram::recvData(uint8_t* buf)
{
	byte size;
	if (size = _driver.ReceiveData(buf))
	{
		_rxHeaderFrom = headerFrom();
		_rxHeaderTo = headerTo();
		_rxHeaderSource = headerSource();
		_rxHeaderDestination = headerDestination();
		_rxHeaderType = headerType();
		_rxHeaderData = headerData();
		_rxHeaderFlags = headerFlags();
		_rxHeaderSeqNum = headerSeqNum();
		_rxHeaderHop = headerHop();
		return (size - CC1120_HEADER_LEN);
	}
	//routing 아닐시에는 여기서 바로 ack보내도록(type 값이 routing type의 값보다 크면 operation message)
	//중간 node에서 목적지 node로 보내는 것도..
	return 0;
}

////////////////////////////////////////////////////////////////////
// Public methods

void Datagram::addRouteTo(uint16_t  dest, uint16_t  next_hop, uint8_t state, uint8_t hop)
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
		}
	}
}

////////////////////////////////////////////////////////////////////
Datagram::RoutingTableEntry* Datagram::getRouteTo(uint16_t  dest)
{
	uint8_t i;
	for (i = 0; i < ROUTING_TABLE_SIZE; i++)
		if (_routes[i].dest == dest && _routes[i].state != Invalid)
			return &_routes[i];
	return NULL;
}

////////////////////////////////////////////////////////////////////
void Datagram::deleteRoute(uint8_t index)
{
	// Delete a route by copying following routes on top of it
	memcpy(&_routes[index], &_routes[index + 1],
		sizeof(RoutingTableEntry) * (ROUTING_TABLE_SIZE - index - 1));
	_routes[ROUTING_TABLE_SIZE - 1].state = Invalid;
}

////////////////////////////////////////////////////////////////////
void Datagram::printRoutingTable()
{
	//#ifdef HAVE_SERIAL
	uint8_t i;
	for (i = 0; i < ROUTING_TABLE_SIZE; i++)
	{
		Serial.print(i, DEC);
		Serial.print(" Dest: ");
		Serial.print(_routes[i].dest, HEX);
		Serial.print(" Next Hop: ");
		Serial.print(_routes[i].next_hop, HEX);
		Serial.print(" State: ");
		Serial.print(_routes[i].state, HEX);
		Serial.print(" Hop: ");
		Serial.println(_routes[i].hop, HEX);
	}
	//#endif
}

////////////////////////////////////////////////////////////////////
bool Datagram::deleteRouteTo(uint16_t dest)
{
	uint8_t i;
	for (i = 0; i < ROUTING_TABLE_SIZE; i++)
	{
		if (_routes[i].dest == dest)
		{
			deleteRoute(i);
			return true;
		}
	}
	return false;
}

////////////////////////////////////////////////////////////////////
void Datagram::clearRoutingTable()
{
	uint8_t i;
	for (i = 0; i < ROUTING_TABLE_SIZE; i++)
	{
		_routes[i].state = Invalid;
		_routes[i].next_hop = 0;
		_routes[i].hop = 0;
	}
}
//////////////////////////////////여기부터 라우팅 하는 코드//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//=====================================================================================================
// 2017-04-26 ver.1
void Datagram::send(uint16_t from, uint16_t to, uint16_t src, uint16_t dst, uint8_t type, uint8_t data, uint8_t flags, uint8_t seqNum, uint8_t hop, uint8_t* temp_buf, uint8_t size)
{
	while (millis() - sendingTime < 1000);
	sendingTime = millis();//어차피 대충 20일에 한번은 reset된다하면 sendingTime 타입 int여도 무방함

	setHeaderFrom(from);
	setHeaderTo(to);
	setHeaderSource(src);
	setHeaderDestination(dst);
	setHeaderType(type);
	setHeaderData(data);
	setHeaderFlags(flags);
	setHeaderSeqNum(seqNum);
	setHeaderHop(hop);
	_driver.SendData(temp_buf, size);
}
bool Datagram::sendToWaitAck(uint16_t from, uint16_t to, uint16_t src, uint16_t dst, uint8_t type, uint8_t data, uint8_t flags, uint8_t seqNum, uint8_t hop, uint8_t* temp_buf, uint8_t size, unsigned long time)
{
	unsigned long startTime = 0;
	setHeaderFrom(from);
	setHeaderTo(to);
	setHeaderSource(src);
	setHeaderDestination(dst);
	setHeaderType(type);
	setHeaderData(data);
	setHeaderSeqNum(seqNum);
	setHeaderHop(hop);

	for (int i = 0; i < DEFAULT_RETRIES; i++)
	{
		while (millis() - sendingTime < 1000);
		sendingTime = millis();//어차피 대충 20일에 한번은 reset된다하면 sendingTime 타입 int여도 무방함, TIME_TERM이 1000 이상이면 이거 없어도 되지만 ACK 보낼 때 send함수쓰고 다음에 바로 이 함수 쓰는 경우가 있으므로 필요


		_driver.SendData(temp_buf, size);
		startTime = millis();
		while (millis() - startTime < TIME_TERM)
		{
			SetReceive();
			if (available())
			{
				if (recvData(temp_buf) &&  _rxHeaderFrom == to)
				{
					if (_rxHeaderTo == _thisAddress && (_rxHeaderType == ACK || _rxHeaderType == REQUEST_ACK_TYPE || _rxHeaderType == R2_REQUEST_ACK_TYPE || _rxHeaderType == REQUEST_MULTI_HOP_ACK || _rxHeaderType == REQUEST_DIRECT_ACK))
					{
						addRouteTo(_rxHeaderFrom, _rxHeaderFrom, Valid, 1);
						return true;
					}
					if (type == R2_REQUEST_TYPE && _rxHeaderType == R2_REQUEST_REAL_TYPE)//gateway에서 round2요청을 1hop master에게 하고 master가 gateway에게 ack을 보냈는데, 못받을 경우 대비
						return true;
					if (type == REQUEST_MULTI_HOP && _rxHeaderType == REQUEST_MULTI_HOP)
						return true;
				}
			}
		}
		delay(time);
	}
	return false;
}
void Datagram::printRecvPacketHeader()
{
	Serial.println("-----------------------------------------------------------------------------");
	Serial.print("_rxHeaderFrom : ");
	Serial.println(_rxHeaderFrom, HEX);
	Serial.print("_rxHeaderTo : ");
	Serial.println(_rxHeaderTo, HEX);
	Serial.print("_rxHeaderSource : ");
	Serial.println(_rxHeaderSource, HEX);
	Serial.print("_rxHeaderDestination : ");
	Serial.println(_rxHeaderDestination, HEX);
	Serial.print("_rxHeaderType : ");
	Serial.println(_rxHeaderType);
	Serial.print("_rxHeaderData : ");
	Serial.println(_rxHeaderData);
	Serial.print("_rxHeaderFlags : ");
	Serial.println(_rxHeaderFlags);
	Serial.print("_rxHeaderSeqNum : ");
	Serial.println(_rxHeaderSeqNum);
	Serial.print("_rxHeaderHop : ");
	Serial.println(_rxHeaderHop);
}
void Datagram::printPath()
{
	uint16_t temp_address;
	Serial.println("-----PATH-----");
	byte index;
	for (uint8_t i = 1; i <= NUM_OF_MASTER;i++)
	{
		Serial.print(i);
		Serial.print(" master : ");
		temp_address = convertToAddress(gatewayNumber, i, 0);
		while (temp_address != _thisAddress)
		{
			Serial.print(temp_address, HEX);
			Serial.print(" -> ");
			index = convertToMasterNumber(temp_address);
			temp_address = parentMaster[index];
		}
		Serial.print(" -> ");
		Serial.println(_thisAddress, HEX);
	}
	Serial.println("--------------");
}
/****************************************************************
*FUNCTION NAME:  convertToAddress(uint8_t gatewayNumber, uint8_t masterNumber,uint8_t slaveNumber)
*FUNCTION     :  2 바이트 형식의 주소로 변환
*INPUT        : 게이트 번호, 매스터 노드 번호, 슬레이브 번호
*OUTPUT       : 2바이트  노드 주소
****************************************************************/
uint16_t Datagram::convertToAddress(uint8_t gatewayNumber, uint8_t masterNumber, uint8_t slaveNumber)
{
	if (gatewayNumber > 64 || masterNumber > 31 || slaveNumber > 16)
		return -1;

	uint16_t address;
	address = gatewayNumber << 5;
	address = address | masterNumber;
	address = address << 5;
	address = address | slaveNumber;
	return address;

}
/****************************************************************
*FUNCTION NAME:  convertToMasterNumber(uint16_t address)
*FUNCTION     :  일반적인 주소로 마스터 번호를 구함
*INPUT        : 2바이트 노도 주소
*OUTPUT       : 마스터 번호
****************************************************************/
uint8_t Datagram::convertToMasterNumber(uint16_t address)
{
	uint8_t masterNum;
	address = address & 0x03FF;
	masterNum = (uint8_t)address >> 5;
	return masterNum;
}
/****************************************************************
*FUNCTION NAME:FromGatewatyToMaster
*FUNCTION     :
*INPUT        :
*OUTPUT       :
****************************************************************/
void Datagram::FromGatewayToMaster() {
	uint16_t address_i;
	Serial.println("FromGatewayToMaster");
	Serial.println("[row1]");
	//row1 : 모든 master들에게 한번씩 보내봄(1hop인 노드 이 때 체크)

	if (G_find_1stRow_master() == NUM_OF_MASTER)// 모든 노드들이 1hop 거리 일경우
	{
		for (int i = 1; i <= NUM_OF_MASTER; i++)
		{
			address_i = convertToAddress(gatewayNumber, i, 0);
			sendToWaitAck(_thisAddress, getRouteTo(address_i)->next_hop, _thisAddress, address_i, CHECK_ROUTING, NONE, NONE, NONE, NONE, temp_buf, sizeof(temp_buf));
		}
		return;
	}
	if (G_find_2ndRow_master() == 0)
	{
		for (int i = 1; i <= NUM_OF_MASTER; i++)
		{
			address_i = convertToAddress(gatewayNumber, i, 0);
			sendToWaitAck(_thisAddress, getRouteTo(address_i)->next_hop, _thisAddress, address_i, CHECK_ROUTING, NONE, NONE, NONE, NONE, temp_buf, sizeof(temp_buf));
		}
		return;
	}
	if (G_find_multihop_node())
	{
		for (int i = 1; i <= NUM_OF_MASTER; i++)
		{
			if (!checkReceive[i])
			{
				address_i = convertToAddress(gatewayNumber, i, 0);
				if (sendToWaitAck(_thisAddress, address_i, _thisAddress, address_i, REQUEST_DIRECT, NONE, NONE, NONE, NONE, temp_buf, sizeof(temp_buf)))
				{
					printRecvPacketHeader();
					Serial.print(i); Serial.println("master is 1 hop node");
					checkReceive[i] = true;
					parentMaster[i] = _thisAddress;
					addRouteTo(_rxHeaderFrom, _rxHeaderFrom, Valid, 1);
					printRoutingTable();
				}
				else
				{
					uint8_t row_number = 1;
					uint8_t number_of_node;
					uint16_t node_list[NUM_OF_MASTER] = { 0 };

					number_of_node = G_get_i_row_node_list(row_number, node_list);  // the gateway needs the list of 2nd row masters which will send Multi_HOP_REQUEST
																					// to unknown nodes.
					while (number_of_node != 0)
					{
						if (G_request_path_one_by_one(address_i, row_number, node_list, number_of_node, REQUEST_DIRECT))
							break;
						number_of_node = G_get_i_row_node_list(row_number, node_list);
						row_number++;
					}
				}
			}
		}
	}
	for (int i = 1; i <= NUM_OF_MASTER; i++)
	{
		address_i = convertToAddress(gatewayNumber, i, 0);
		sendToWaitAck(_thisAddress, getRouteTo(address_i)->next_hop, _thisAddress, address_i, CHECK_ROUTING, NONE, NONE, NONE, NONE, temp_buf, sizeof(temp_buf));
	}
}
/****************************************************************
*FUNCTION NAME:  G_find_1stRow_master( )
*FUNCTION     : Gateway가 broadcast로 master들에게 라우팅 요청 메세지를 전송
응답을 기다린다.
- 응답이 도착하면, 라우팅 테이블에 삽입
*INPUT        : none
*OUTPUT       : 1 hop 노드들의 갯수
****************************************************************/
int8_t Datagram::G_find_1stRow_master()
{
	uint8_t num_of_routed_nodes = 0;
	Serial.println("send broadcast request message");
	Serial.println(masterBroadcastAddress, HEX);
	for (int i = 0; i < R_GATEWAY_SEND_NUM; i++)
	{
		send(_thisAddress, masterBroadcastAddress, _thisAddress, masterBroadcastAddress, REQUEST_BROADCAST, NONE, NONE, NONE, NONE, temp_buf, sizeof(temp_buf));
	}
	for (int i = 1; i <= NUM_OF_MASTER; i++)
	{
		uint16_t dst = convertToAddress(gatewayNumber, i, 0);
		uint8_t recvAckNum = 0;
		for (int j = 0; j < DEFAULT_RETRIES; j++)
		{
			send(_thisAddress, dst, _thisAddress, dst, REQUEST_TYPE, NONE, NONE, NONE, NONE, temp_buf, sizeof(temp_buf));
			startTime = millis();
			while (millis() - startTime < TIME_TERM * 5)
			{
				SetReceive();
				if (available())
				{
					if (recvData(temp_buf) && _rxHeaderFrom == dst && _rxHeaderTo == _thisAddress && _rxHeaderType == REQUEST_ACK_TYPE)
					{
						recvAckNum++;
						if (recvAckNum > 4)
						{
							addRouteTo(_rxHeaderFrom, _rxHeaderFrom, Valid, 1);
							printRecvPacketHeader();
							Serial.print(i); Serial.println("master is 1 hop node");
							checkReceive[i] = true;
							parentMaster[i] = _thisAddress;
							printRoutingTable();
							num_of_routed_nodes++;
							j = DEFAULT_RETRIES;
							break;
						}
					}
				}
			}
			delay(1000);
		}
	}
	for (uint8_t i = 1; i <= NUM_OF_MASTER; i++)
	{
		if (!checkReceive[i])
		{
			Serial.print("unreceive : ");
			Serial.println(i);
		}
	}
	return num_of_routed_nodes;
}
/****************************************************************
*FUNCTION NAME:  G_find_2ndRow_master( )
*FUNCTION     :  Gateway가  1 hop 노드들에게, 라우팅이 안된 노드들의 주소를 페이로드에 포함하여 전송함.

*INPUT        : none
*OUTPUT       :
****************************************************************/
int8_t Datagram::G_find_2ndRow_master()
{
	Serial.println("[row2]");
	//row2 : 1hop인 master들에게 아직 라우팅이 안된 노드들의 주소를 보내서 근처에 노드들이 있는지 물어봄

	uint16_t node_list[NUM_OF_MASTER] = { 0 };
	uint8_t number_of_node;
	uint8_t number_of_unknown_node = 0;

	number_of_node = G_get_i_row_node_list(1, node_list);

	Serial.print("1hop's num : ");
	Serial.println(number_of_node);

	for (uint8_t i = 0; i < number_of_node; i++)
	{
		number_of_unknown_node = 0;
		for (uint8_t j = 1; j <= NUM_OF_MASTER; j++)//routing이 안된 노드들의 주소를 table을 뒤져서 알아냄
		{
			if (!checkReceive[j])
			{
				Serial.print("unreceive : ");
				Serial.println(j);
				uint16_t address = convertToAddress(gatewayNumber, j, 0);
				//routing이 안된 애들의 주소를 전송되는 배열에 넣는다.

				temp_buf[number_of_unknown_node * 2] = (uint8_t)(address >> 8);
				temp_buf[number_of_unknown_node * 2 + 1] = (uint8_t)(0x00FF & address);
				number_of_unknown_node++;
			}
		}
		if (number_of_unknown_node == 0)
			return 0;
		Serial.print("send : ");
		Serial.println(node_list[i], HEX);


		//from, to, src, dst, type, data, flags, seqnum, hop, payload, size of payload, timeout
		if (!sendToWaitAck(_thisAddress, node_list[i], _thisAddress, node_list[i], R2_REQUEST_TYPE, number_of_unknown_node, NONE, NONE, NONE, temp_buf, sizeof(temp_buf)))
		{
			deleteRouteTo(node_list[i]);
			checkReceive[convertToMasterNumber(node_list[i])] = false;
			parentMaster[convertToMasterNumber(node_list[i])] = 0;
			for (uint8_t j = 0; j < i; j++)
			{
				temp_buf[0] = (uint8_t)node_list[i] >> 8;
				temp_buf[1] = (uint8_t)(0x00FF & node_list[i]);
				if (sendToWaitAck(_thisAddress, node_list[j], _thisAddress, node_list[j], R2_REQUEST_TYPE, 1, NONE, NONE, NONE, temp_buf, sizeof(temp_buf)))
				{
					//startTime = millis();
					//while (millis() - startTime < 10 * TIME_TERM)
					while(1)//
					{
						SetReceive();
						if (available())
						{
							if (recvData(temp_buf) && _rxHeaderTo == _thisAddress && _rxHeaderType == R2_REQUEST_ACK_TYPE && _rxHeaderFrom == node_list[j])
							{
								if (_rxHeaderData != 0)
								{
									send(_thisAddress, _rxHeaderFrom, _thisAddress, _rxHeaderFrom, ACK, NONE, NONE, NONE, NONE, temp_buf, sizeof(temp_buf));
									printRecvPacketHeader();
									addRouteTo(node_list[i], node_list[j], Valid, 2);
									uint8_t masterNum = convertToMasterNumber(node_list[i]);
									parentMaster[masterNum] = _rxHeaderFrom;
									checkReceive[masterNum] = true;
								}
								break;
							}
						}
					}
					if (checkReceive[node_list[i]])
						break;
				}
			}
			continue;
		}

		Serial.println(word(temp_buf[0], temp_buf[1]), HEX);
		//startTime = millis();
		//while ((millis() - startTime) < number_of_unknown_node * 10 * TIME_TERM)
		while(1)
		{
			SetReceive();
			if (available())
			{
				if (recvData(temp_buf))
				{
					if (_rxHeaderTo == _thisAddress && _rxHeaderType == R2_REQUEST_ACK_TYPE && _rxHeaderFrom == node_list[i])
					{
						send(_thisAddress, _rxHeaderFrom, _thisAddress, _rxHeaderFrom, ACK, NONE, NONE, NONE, NONE, temp_buf, sizeof(temp_buf));
						for (uint8_t j = 0; j < _rxHeaderData; j++)
						{
							printRecvPacketHeader();
							uint16_t temp_addr = word(temp_buf[j * 2], temp_buf[j * 2 + 1]);
							addRouteTo(temp_addr, _rxHeaderFrom, Valid, 2);
							uint8_t masterNum = convertToMasterNumber(temp_addr);
							parentMaster[masterNum] = _rxHeaderFrom;
							checkReceive[masterNum] = true;

						}
						printRoutingTable();
						break;
					}
				}
			}
		}
	}
	number_of_unknown_node = 0;
	for (uint8_t i = 1; i <= NUM_OF_MASTER; i++)
	{
		if (!checkReceive[i])
			number_of_unknown_node++;
	}
	return number_of_unknown_node;
}
/****************************************************************
*FUNCTION NAME:FromMasterToGateway
*FUNCTION     :
*INPUT        :
*OUTPUT       :
****************************************************************/
void Datagram::FromMasterToGateway() {
	uint8_t receivedRequestNum = 0;
	uint8_t receivedOverhearNum = 1;

	uint16_t priorPacketFrom = -1;
	uint8_t priorPacketType = -1;

	Serial.println("FromMasterToGateway");
	Serial.println(masterBroadcastAddress, HEX);
	while (1)
	{
		SetReceive();
		if (available())
		{
			if (recvData(temp_buf))
			{
				if (priorPacketFrom == _rxHeaderFrom && priorPacketType == _rxHeaderType)
				{
					if (receivedOverhearNum++ > R_MASTER_SEND_NUM - 1)
						M_findCandidateParents();
				}
				else
					receivedOverhearNum = 1;
				M_findCandidateParents();
				if (_rxHeaderTo == _thisAddress || _rxHeaderTo == masterBroadcastAddress)//routing 시 broadcast는 gateway에서 처음에 보내는 거 외에는 없어야함
				{
					addRouteTo(_rxHeaderFrom, _rxHeaderFrom, Valid, 1);
					printRecvPacketHeader();
					if (_rxHeaderDestination != _thisAddress && _rxHeaderDestination != masterBroadcastAddress)
					{
						send(_thisAddress, _rxHeaderFrom, _thisAddress, _rxHeaderFrom, ACK, NONE, NONE, NONE, NONE, temp_buf, sizeof(temp_buf));
						if(!sendToWaitAck(_thisAddress, getRouteTo(_rxHeaderDestination)->next_hop, _rxHeaderSource, _rxHeaderDestination, _rxHeaderType, NONE, NONE, NONE, _rxHeaderHop + 1, temp_buf, sizeof(temp_buf)))
							send(_thisAddress, _rxHeaderFrom, _thisAddress, _rxHeaderFrom, NACK, NONE, NONE, NONE, NONE, temp_buf, sizeof(temp_buf));
					}
					else if (_rxHeaderType == REQUEST_BROADCAST)
					{
						Serial.println("receivedBroadcastAddress");
						Serial.println(receivedRequestNum);
						receivedRequestNum++;
					}
					else if (_rxHeaderType == REQUEST_TYPE && receivedRequestNum > R_GATEWAY_SEND_NUM - 2)
					{
						Serial.println("receive row1 request");
						printRoutingTable();
						//from, to, src, dst, type, data, flags, seqnum, hop
						for(uint8_t i = 0;i < R_MASTER_SEND_NUM;i++)
							send(_thisAddress, _rxHeaderFrom, _thisAddress, _rxHeaderFrom, REQUEST_ACK_TYPE, NONE, NONE, NONE, NONE, temp_buf, sizeof(temp_buf));
					}
					else if (_rxHeaderType == R2_REQUEST_TYPE)
					{
						//from, to, src, dst, type, data, flags, seqnum, hop
						send(_thisAddress, _thisAddress & 0xFC00, _thisAddress, _thisAddress & 0xFC00, ACK, NONE, NONE, NONE, NONE, temp_buf, sizeof(temp_buf));
						M_find2ndRowMasters();
					}
					else if (_rxHeaderType == R2_REQUEST_REAL_TYPE)
					{
						Serial.println("receive R2 routing request and I didn't success in R1 request");
						M_masterSendRoutingReply();
					}/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////20180314 다시보기/////
					else if (_rxHeaderType == REQUEST_MULTI_HOP || _rxHeaderType == REQUEST_DIRECT)
					{
						if (word(temp_buf[0], temp_buf[1]) != _thisAddress)
						{
							//아직 routing 안된 애 한테 보내줌
							Serial.println("received multihop request and send to unrouting Master");
							uint16_t realDst = word(temp_buf[0], temp_buf[1]);
							//from, to, src, dst, type, data, flags, seqnum, hop
							send(_thisAddress, _rxHeaderFrom, _thisAddress, _rxHeaderFrom, ACK, NONE, NONE, NONE, NONE, temp_buf, sizeof(temp_buf));
							if (sendToWaitAck(_thisAddress, realDst, _rxHeaderSource, realDst, _rxHeaderType, NONE, NONE, NONE, _rxHeaderHop + 1, temp_buf, sizeof(temp_buf), TIME_TERM))
							{
								Serial.println("receive multihop ack to me");
								Serial.println(getRouteTo(_rxHeaderDestination)->next_hop, HEX);
								send(_thisAddress, getRouteTo(_rxHeaderDestination)->next_hop, _rxHeaderSource, _rxHeaderDestination, _rxHeaderType, NONE, NONE, NONE, _rxHeaderHop + 1, temp_buf, sizeof(temp_buf));
							}
							else
								sendToWaitAck(_thisAddress, getRouteTo(_thisAddress & 0xFC00)->next_hop, _thisAddress, _thisAddress & 0xFC00, NACK, NONE, NONE, NONE, NONE, temp_buf, sizeof(temp_buf));
						}
						else
						{
							//routing 안 되어있던 애가 받음
							Serial.println("received multi hop request");
							if(_rxHeaderType == REQUEST_MULTI_HOP)
								M_masterSendRoutingReply();
							else if(_rxHeaderType == REQUEST_DIRECT)
							{
								addRouteTo(_rxHeaderSource, _rxHeaderFrom, Valid, _rxHeaderHop + 1);
								printRoutingTable();
								//from, to, src, dst, type, data, flags, seqnum, hop
								send(_thisAddress, _rxHeaderFrom, _thisAddress, _thisAddress & 0xFC00, REQUEST_DIRECT_ACK, NONE, NONE, NONE, NONE, temp_buf, sizeof(temp_buf));
							}
						}
					}
					else if (_rxHeaderType == CHECK_ROUTING)
					{
						send(_thisAddress, getRouteTo(_thisAddress & 0xFC00)->next_hop, _thisAddress, _thisAddress & 0xFC00, ACK, NONE, NONE, NONE, NONE, temp_buf, sizeof(temp_buf));
						break;
					}
				}
				priorPacketFrom = _rxHeaderFrom;
				priorPacketType = _rxHeaderType;
			}
		}
	}
}

/****************************************************************
*FUNCTION NAME:  M_findCandidateParents( )   20180308확인 끝
*FUNCTION     :

*INPUT        :
*OUTPUT       :
****************************************************************/
void Datagram::M_findCandidateParents()
{
	if (_rxHeaderType == REQUEST_ACK_TYPE || _rxHeaderType == R2_REQUEST_ACK_TYPE || _rxHeaderType == REQUEST_MULTI_HOP_ACK)
	{
		if (receivedType == 0)
		{
			Serial.println("receive propagation");
			receivedType = _rxHeaderType;
			candidateAddress = _rxHeaderFrom;
			candidateRSSI = _driver.rssi;
			//candidateHop = 
			Serial.print("My choice : ");
			Serial.println(candidateAddress, HEX);
		}
		else if (receivedType == _rxHeaderType)//Type check 안하면, 계속 가장 인접한 노드를 부모로 선택하게 되어서 hop수가 계속 늘어난다.
		{
			if (_driver.rssi >= candidateRSSI)
			{
				Serial.println("receive propagation");
				candidateAddress = _rxHeaderFrom;
				candidateRSSI = _driver.rssi;
				//candidateHop = 
				Serial.print("My choice : ");
				Serial.println(_rxHeaderFrom, HEX);
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
void Datagram::M_find2ndRowMasters()
{
	uint16_t childNodeList[32];
	uint16_t unknown_address;
	uint8_t temp[20];
	Serial.println("receive R2 request and I succeed R1");

	uint8_t count = _rxHeaderData;  // Guin: number of unknown node
	uint8_t idx = 0;

	for (int8_t i = 0; i < count; i++)//temp_buf에 수신되는 data가 계속 들어오므로 temp에 복사해두어야함
	{
		temp[i*2] = temp_buf[i*2];
		temp[i*2 + 1] = temp_buf[i*2 + 1];
	}
	Serial.print("count : ");
	Serial.println(count);
	for (int8_t i = 0; i < count; i++)
	{
		Serial.println("send R2 reequest to 2 hop");
		unknown_address = word(temp[i*2], temp[i*2 + 1]);
		//from, to, src, dst, type, data, flags, seqnum, hop, payload, size of payload, timeout
		if (sendToWaitAck(_thisAddress, unknown_address, _thisAddress & 0xFC00, unknown_address, R2_REQUEST_REAL_TYPE, NONE, NONE, NONE, 1, temp_buf, sizeof(temp_buf), TIME_TERM))
		{
			Serial.println("receiving");
			printRoutingTable();
			childNodeList[idx++] = _rxHeaderFrom;
		}
	}
	for (int8_t i = 0; i < idx; i++)
	{
		temp_buf[i*2] = highByte(childNodeList[i]);
		temp_buf[i*2 + 1] = lowByte(childNodeList[i]);
	}
	//from, to, src, dst, type, data, flags, seqnum, hop
	sendToWaitAck(_thisAddress, _thisAddress & 0xFC00, childNodeList[0], _thisAddress & 0xFC00, R2_REQUEST_ACK_TYPE, idx, NONE, NONE, 1, temp_buf, sizeof(temp_buf));
}
/****************************************************************
*FUNCTION NAME:M_masterSendRoutingReply( )
*FUNCTION     :

*INPUT        :
*OUTPUT       :
****************************************************************/
void Datagram::M_masterSendRoutingReply()
{
	Serial.print("My candidate : ");
	Serial.println(candidateAddress, HEX);
	Serial.print("Receved from : ");
	Serial.println(_rxHeaderFrom, HEX);
	if (candidateAddress == _rxHeaderFrom || candidateAddress == 0)
	{
		addRouteTo(_rxHeaderSource, _rxHeaderFrom, Valid, _rxHeaderHop + 1);
		printRoutingTable();
		//from, to, src, dst, type, data, flags, seqnum, hop
		if (_rxHeaderType == R2_REQUEST_REAL_TYPE)
			send(_thisAddress, _rxHeaderFrom, _thisAddress, _thisAddress & 0xFC00, R2_REQUEST_ACK_TYPE, NONE, NONE, NONE, NONE, temp_buf, sizeof(temp_buf));
		else if (_rxHeaderType == REQUEST_MULTI_HOP)
			send(_thisAddress, _rxHeaderFrom, _thisAddress, _thisAddress & 0xFC00, REQUEST_MULTI_HOP_ACK, NONE, NONE, NONE, NONE, temp_buf, sizeof(temp_buf));
	}
}
/****************************************************************
*FUNCTION NAME: G_get_i_row_node_list(uint8_t row_number, uint16_t *node_list )
*FUNCTION     : Gatway gets the list of node on the i_th row

*INPUT        : row number
*OUTPUT       : return value: number of node on the i_th row
****************************************************************/
uint8_t Datagram::G_get_i_row_node_list(uint8_t row_number, uint16_t *node_list)
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
*FUNCTION NAME: G_find_multihop_node ()
*FUNCTION     :  Gateway finished the routing discovery upto 2nd row.
Gateway tries to find the 3rd and more row nodes.
- 먼저 2nd row node들을 구한후, 아직 routing이 안된 노드의 주소를  payload에 포함하여 전송한다.
- 3rd row 노드들을 구한후에는, routing이 안된 노드의 주소를  payload에 포함하여
최근에 구한 row의 노드들에게  multihop routing 요청 메시지를 전송한다.
*INPUT        :
*OUTPUT       :
****************************************************************/
int8_t Datagram::G_find_multihop_node()
{
	uint8_t row_number = 2;
	uint8_t number_of_node;
	uint16_t node_list[NUM_OF_MASTER] = { 0 };
	uint8_t number_of_unknown_node = 0;

	for (uint8_t i = 1; i <= NUM_OF_MASTER; i++)
	{
		if (!checkReceive[i])
			number_of_unknown_node++;
	}
	if (number_of_unknown_node == 0)
		return 0;

	number_of_node = G_get_i_row_node_list(row_number++, node_list);  // the gateway needs the list of 2nd row masters which will send Multi_HOP_REQUEST
														   // to unknown nodes.

	Serial.println("[MULTI_HOP]");
	while (number_of_node != 0)
	{
		for (uint8_t i = 1; i <= NUM_OF_MASTER; i++)
		{
			if (!checkReceive[i])
			{
				uint16_t unknown_address = convertToAddress(gatewayNumber, i, 0);
				if (G_request_path_one_by_one(unknown_address, row_number, node_list, number_of_node, REQUEST_MULTI_HOP))
					number_of_unknown_node--;
			}
			if (number_of_unknown_node == 0)
				break;
		}
		if (number_of_unknown_node == 0)
			break;

		number_of_node = G_get_i_row_node_list(row_number++, node_list);
	}
	return number_of_unknown_node;
}
/****************************************************************
*FUNCTION NAME:  G_request_path_one_by_one(uint16_t unknown_address, byte row_number, uint16_t* node_list, byte number_of_node, uint8_t type)
*FUNCTION     :	 node_list에 있는 노드들에게 주소가 unknown_address인 노드가 인접해 있는지 물어봄

*INPUT        :  목적지 주소
*OUTPUT       :  성공 여부
****************************************************************/
bool Datagram::G_request_path_one_by_one(uint16_t unknown_address, uint8_t row_number, uint16_t* node_list, byte number_of_node, uint8_t type)
{
	bool result = false;
	uint8_t masterNum;
	masterNum = convertToMasterNumber(unknown_address);

	for (uint8_t i = 0; i < number_of_node; i++)
	{
		temp_buf[0] = (uint8_t)(unknown_address >> 8);
		temp_buf[1] = (uint8_t)(0x00FF & unknown_address);

		uint16_t next_hop = getRouteTo(node_list[i])->next_hop;
		//from, to, src, dst, type, data, flags, seqnum, hop

		if (!sendToWaitAck(_thisAddress, next_hop, _thisAddress, node_list[i], type, NONE, NONE, NONE, NONE, temp_buf, sizeof(temp_buf)))
		{
			deleteRouteTo(next_hop);
			checkReceive[convertToMasterNumber(next_hop)] = false;
			parentMaster[convertToMasterNumber(next_hop)] = 0;
			continue;
		}

		startTime = millis();
		while (millis() - startTime < row_number * 2 * TIME_TERM * 2)
		{
			SetReceive();
			if (available())
			{
				if (recvData(temp_buf) && _rxHeaderTo == _thisAddress && _rxHeaderDestination == _thisAddress)
				{
					if (_rxHeaderSource == unknown_address && (_rxHeaderType == REQUEST_MULTI_HOP_ACK || _rxHeaderType == REQUEST_DIRECT_ACK) && _rxHeaderFrom == next_hop)
					{
						printRecvPacketHeader();
						addRouteTo(_rxHeaderSource, _rxHeaderFrom, Valid, _rxHeaderHop + 1);
						parentMaster[masterNum] = node_list[i];
						checkReceive[masterNum] = true;
						printRoutingTable();
						return true;
					}
					else if(_rxHeaderType == NACK)
						break;
				}
			}
		}
	}
	return false;
}

/////운영중 routing 문제 발생시///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/****************************************************************
*FUNCTION NAME:  G_discoverNewPath(uint16_t address)
*FUNCTION     :	 다시 routing 해줌

*INPUT        :
*OUTPUT       :
****************************************************************/
void Datagram::G_discoverNewPath(uint16_t address)
{
	uint8_t row_number;
	uint8_t number_of_node;
	uint16_t node_list[NUM_OF_MASTER] = { 0 };

	row_number = getRouteTo(address)->hop;
	number_of_node = G_get_i_row_node_list(row_number, node_list);

	G_request_path_one_by_one(address, row_number + 1, node_list, number_of_node, REQUEST_DIRECT);//함수의 매개변수인 address에 대한 path 새로 지정

	for (int i = 1; i <= NUM_OF_MASTER; i++)//함수의 매개변수인 address의 자식들의 path 새로 지정
	{
		if (parentMaster[i] == address)
		{
			G_request_path_one_by_one(convertToAddress(gatewayNumber, i, 0), row_number + 1, node_list, number_of_node, REQUEST_DIRECT);
		}
	}
}
////////////////////////////////////////////////////////////////////////////여기까지 routing///////////////////////////////////////////////////////////////////////////////////////////////////////