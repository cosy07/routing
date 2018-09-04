//2017-07-17
// ToDatagram.cpp
//
// Copyright (C) 2011 Mike McCauley
// $Id: RHToDatagram.cpp,v 1.6 2014/05/23 02:20:17 mikem Exp $

#include <ToDatagram_STM.h>

ToDatagram::ToDatagram(TO_ELECHOUSE_CC1120& driver, uint8_t gatewayNumber, uint8_t thisAddress)//, uint8_t master_num)
	:
	_driver(driver),
	_thisAddress(thisAddress),
	gatewayNumber(gatewayNumber)//,
	//_numOfMaster(master_num)
{
	//_retransmissions = 0;
	clearRoutingTable();
	//gatewayNumber = thisAddress >> 10;
	masterBroadcastAddress = 0xFF;//= convertToAddress(gatewayNumber, 31, 0);
}

////////////////////////////////////////////////////////////////////
// Public methods
void ToDatagram::init()
{
	setThisAddress(_thisAddress);
	_driver.Init();

}
////////////////////////////////////////////////////////////////////
// Public methods
void ToDatagram::init(byte ch)  // Setup Channel number
{
	setThisAddress(_thisAddress);
	_driver.Init(ch);
	ch = ch;
}

////////////////////////////////////////////////////////////////////
// Public methods
void ToDatagram::SetReceive()
{
	_driver.SetReceive();
}

void  ToDatagram::setThisAddress(uint8_t thisAddress)
{
	_driver.setThisAddress(thisAddress);
	// Use this address in the transmitted FROM header
	setHeaderFrom(thisAddress);
	_thisAddress = thisAddress;
}

void  ToDatagram::setHeaderFrom(uint8_t from)
{
	_driver.setHeaderFrom(from);
}

void ToDatagram::setHeaderTo(uint8_t to)
{
	_driver.setHeaderTo(to);
}


void ToDatagram::setHeaderSource(uint8_t source)
{
	_driver.setHeaderSource(source);
}

void ToDatagram::setHeaderDestination(uint8_t destination)
{
	_driver.setHeaderDestination(destination);
}

void ToDatagram::setHeaderType(uint8_t type)
{
	_driver.setHeaderType(type);
}

void ToDatagram::setHeaderData(uint8_t data)
{
	_driver.setHeaderData(data);
}
void ToDatagram::setHeaderSeqNum(uint8_t seq)
{
	_driver.setHeaderSeqNum(seq);
}
void ToDatagram::setHeaderFlags(uint8_t flags)
{
	_driver.setHeaderFlags(flags);
}
void ToDatagram::setHeaderHop(uint8_t hop)
{
	_driver.setHeaderHop(hop);
}

uint8_t ToDatagram::headerTo()
{
	uint8_t temp = _driver.headerTo();
	return temp;
}
uint8_t ToDatagram::headerFrom()
{
	uint8_t temp = _driver.headerFrom();
	return temp;
}

uint8_t ToDatagram::headerSource()
{
	return(_driver.headerSource());
}
uint8_t ToDatagram::headerDestination()
{
	return(_driver.headerDestination());
}
uint8_t ToDatagram::headerType()
{
	return(_driver.headerType());
}
uint8_t ToDatagram::headerData()
{
	return(_driver.headerData());
}
uint8_t  ToDatagram::headerSeqNum()
{
	return(_driver.headerSeqNum());
}
uint8_t ToDatagram::headerFlags()
{
	return(_driver.headerFlags());
}
uint8_t ToDatagram::headerHop()
{
	return(_driver.headerHop());
}
uint8_t ToDatagram::getThisAddress()
{
	return _thisAddress;
}
bool ToDatagram::available()
{
	/////test��/////
	//return true;
	/////test��/////

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

byte  ToDatagram::recvData(uint8_t* buf)
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
	//routing �ƴҽÿ��� ���⼭ �ٷ� ack��������(type ���� routing type�� ������ ũ�� operation message)
	//�߰� node���� ������ node�� ������ �͵�..
	return 0;
}

////////////////////////////////////////////////////////////////////
// Public methods

void ToDatagram::addRouteTo(uint8_t  dest, uint8_t  next_hop, uint8_t state, uint8_t hop, unsigned long lastTime)
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
ToDatagram::RoutingTableEntry* ToDatagram::getRouteTo(uint8_t  dest)
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
void ToDatagram::deleteRoute(uint8_t index)
{
	// Delete a route by copying following routes on top of it
	memcpy(&_routes[index], &_routes[index + 1],
		sizeof(RoutingTableEntry) * (ROUTING_TABLE_SIZE - index - 1));
	_routes[ROUTING_TABLE_SIZE - 1].state = Invalid;
}

////////////////////////////////////////////////////////////////////
void ToDatagram::printRoutingTable()
{
	//#ifdef HAVE_SERIAL
	uint8_t i;
	for (i = 0; i < ROUTING_TABLE_SIZE; i++)
	{
		Serial.print(i, DEC);
		Serial.print(" Dest: ");
		Serial.print(_routes[i].dest);
		Serial.print(" Next Hop: ");
		Serial.print(_routes[i].next_hop);
		Serial.print(" State: ");
		Serial.print(_routes[i].state);
		Serial.print(" Hop: ");
		Serial.print(_routes[i].hop);
		Serial.print(" Last Time: ");
		Serial.println(_routes[i].lastTime);
	}
	//#endif
}

////////////////////////////////////////////////////////////////////
bool ToDatagram::deleteRouteTo(uint8_t dest)
{
	uint8_t i;
	for (i = 0; i < ROUTING_TABLE_SIZE; i++)
	{
		if (_routes[i].dest == dest)
		{
			_routes[i].state = Invalid;
			deleteRoute(i);
			return true;
		}
	}
	return false;
}

////////////////////////////////////////////////////////////////////
void ToDatagram::clearRoutingTable()
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
void ToDatagram::checkRoutingTable()
{
	Serial.println("checkRoutingTable()");
	printRoutingTable();
	for (int8_t i = 0; i < ROUTING_TABLE_SIZE; i++)
	{
		if (_routes[i].state == Valid && _routes[i].lastTime + 300000 < millis())
		{
			deleteRoute(i);
		}
	}
	Serial.println("after");
	printRoutingTable();
}

int ToDatagram::find_child_node(uint8_t* child_node)
{
	int cnt = 0;
	for (int8_t i = 0; i < ROUTING_TABLE_SIZE; i++)
	{
		if (_routes[i].state == Valid && _routes[i].dest != (_thisAddress & 0x00))
		{
			child_node[cnt++] = _routes[i].dest;
		}
	}
	return cnt;
}

//////////////////////////////////������� ����� �ϴ� �ڵ�//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//=====================================================================================================
// 2017-04-26 ver.1
void ToDatagram::send(uint8_t from, uint8_t to, uint8_t src, uint8_t dst, uint8_t type, uint8_t data, uint8_t flags, uint8_t seqNum, uint8_t hop, uint8_t* temp_buf, uint8_t size)
{
	if (type >= 10 && getRouteTo(dst) == NULL)
		return;
	while (millis() - sendingTime < 1000);//������ ���� 20�Ͽ� �ѹ��� reset�ȴ��ϸ� sendingTime Ÿ�� int���� ������
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
	sendingTime = millis();
}
bool ToDatagram::sendToWaitAck(uint8_t from, uint8_t to, uint8_t src, uint8_t dst, uint8_t type, uint8_t data, uint8_t flags, uint8_t seqNum, uint8_t hop, uint8_t* temp_buf, uint8_t size, unsigned long time)
{
	if (type >= 11 && getRouteTo(dst) == NULL)
		return false;
	unsigned long startTime = 0;
	setHeaderFrom(from);
	setHeaderTo(to);
	setHeaderSource(src);
	setHeaderDestination(dst);
	setHeaderType(type);
	setHeaderData(data);
	setHeaderSeqNum(seqNum);
	setHeaderHop(hop);
	byte length = size;
	for (int i = 0; i < size; i++)
		buffer[i] = temp_buf[i];

	for (int i = 0; i < DEFAULT_RETRIES; i++)
	{
		while (millis() - sendingTime < 1000);
		//sendingTime = millis();//������ ���� 20�Ͽ� �ѹ��� reset�ȴ��ϸ� sendingTime Ÿ�� int���� ������, TIME_TERM�� 1000 �̻��̸� �̰� ��� ������ ACK ���� �� send�Լ����� ������ �ٷ� �� �Լ� ���� ��찡 �����Ƿ� �ʿ�

		Serial.print("retry : ");
		Serial.println(i);
		_driver.SendData(buffer, length);
		sendingTime = millis();
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
						if(_rxHeaderType != ACK)
							addRouteTo(_rxHeaderFrom, _rxHeaderFrom, Valid, 1);
						return true;
					}
					if (type == R2_REQUEST_TYPE && _rxHeaderType == R2_REQUEST_REAL_TYPE)//gateway���� round2��û�� 1hop master���� �ϰ� master�� gateway���� ack�� ���´µ�, ������ ��� ���
						return true;
					if (type == REQUEST_MULTI_HOP && _rxHeaderType == REQUEST_MULTI_HOP)
						return true;
					if ((_rxHeaderType == CHECK_ROUTING_ACK || _rxHeaderType == SCAN_RESPONSE_TO_GATEWAY) && type == CHECK_ROUTING)
					{
						if (_thisAddress != (_thisAddress & 0x00))//master��
						{
							send(_thisAddress, _rxHeaderFrom, _thisAddress, _rxHeaderFrom, ACK, NONE, NONE, NONE, NONE, temp_buf, sizeof(temp_buf));
							if(getRouteTo(_rxHeaderDestination) != NULL)
								sendToWaitAck(_thisAddress, getRouteTo(_rxHeaderDestination)->next_hop, _rxHeaderSource, _rxHeaderDestination, _rxHeaderType, NONE, NONE, NONE, _rxHeaderHop + 1, temp_buf, sizeof(temp_buf));
						}
						return true;
					}
					if (type == CHECK_ROUTING && (_rxHeaderType == CHECK_ROUTING || _rxHeaderType == CHECK_ROUTING_ACK_REROUTING || SCAN_RESPONSE_TO_GATEWAY))
						return true;
					if (type == SCAN_REQUEST_TO_MASTER && _rxHeaderType == SCAN_REQUEST_ACK_FROM_MASTER)
						return true;
				}
			}
		}
		delay(time);
	}
	return false;
}
void ToDatagram::printRecvPacketHeader()
{
	Serial.println("-----------------------------------------------------------------------------");
	Serial.print("_rxHeaderFrom : ");
	Serial.println(_rxHeaderFrom);
	Serial.print("_rxHeaderTo : ");
	Serial.println(_rxHeaderTo);
	Serial.print("_rxHeaderSource : ");
	Serial.println(_rxHeaderSource);
	Serial.print("_rxHeaderDestination : ");
	Serial.println(_rxHeaderDestination);
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
void ToDatagram::printPath()
{
	uint8_t temp_address;
	Serial.println("-----PATH-----");
	byte index;
	for (uint8_t i = 1; i <= master_num;i++)
	{
		Serial.print(i);
		Serial.print(" master : ");
		temp_address = i;// convertToAddress(gatewayNumber, i, 0);
		while (temp_address != _thisAddress)
		{
			Serial.print(temp_address);
			Serial.print(" -> ");
			index = temp_address;// convertToMasterNumber(temp_address);
			temp_address = parentMaster[index];
		}
		//Serial.print(" -> ");
		Serial.println(_thisAddress);
	}
	Serial.println("--------------");
}
void ToDatagram::getPath(uint8_t address, uint8_t* path)
{
	uint8_t temp_address = address;
	byte index;
	uint8_t cnt = 0;
	while (temp_address != _thisAddress)
	{
		path[cnt++] = temp_address;
		index = temp_address;// convertToMasterNumber(temp_address);
		temp_address = parentMaster[index];
	}
}
void ToDatagram::printTree()
{
	Serial.print("abcde");
	Serial.print(" ");
	Serial.println(0);// convertToAddress(gatewayNumber, 0, 0));
	for (uint8_t i = 1; i <= master_num;i++)
	{
		if (checkReceive[i])
		{
			Serial.print("edcba");
			Serial.print(" ");
			Serial.print(parentMaster[i]);
			Serial.print(" ");
			Serial.println(i);// convertToAddress(gatewayNumber, i, 0));
		}
	}
}
/****************************************************************
*FUNCTION NAME:  convertToAddress(uint8_t gatewayNumber, uint8_t masterNumber,uint8_t slaveNumber)
*FUNCTION     :  2 ����Ʈ ������ �ּҷ� ��ȯ
*INPUT        : ����Ʈ ��ȣ, �Ž��� ��� ��ȣ, �����̺� ��ȣ
*OUTPUT       : 2����Ʈ  ��� �ּ�
****************************************************************/
/*uint16_t ToDatagram::convertToAddress(uint8_t gatewayNumber, uint8_t masterNumber, uint8_t slaveNumber)
{
	if (gatewayNumber > 64 || masterNumber > 31 || slaveNumber > 16)
		return -1;

	uint16_t address;
	address = gatewayNumber << 5;
	address = address | masterNumber;
	address = address << 5;
	address = address | slaveNumber;
	return address;

}*/
/****************************************************************
*FUNCTION NAME:  convertToMasterNumber(uint16_t address)
*FUNCTION     :  �Ϲ����� �ּҷ� ������ ��ȣ�� ����
*INPUT        : 2����Ʈ �뵵 �ּ�
*OUTPUT       : ������ ��ȣ
****************************************************************/
/*uint16_t ToDatagram::convertToMasterNumber(uint16_t address)
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
uint8_t ToDatagram::G_get_i_row_node_list(uint8_t row_number, uint8_t *node_list)
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
void ToDatagram::FromGatewayToMaster() {
	uint8_t address_i;
	uint8_t unknown_node_list[32];
	uint8_t number_of_unknown_node;
	Serial.println("FromGatewayToMaster");
	Serial.println("[row1]");

	if (G_find_1stRow_master() != master_num)
	{
		if ((number_of_unknown_node = G_find_2ndRow_master(unknown_node_list)))
		{
			if (G_find_multihop_node(number_of_unknown_node, unknown_node_list))
			{
				Serial.println("REQUEST_DIRECT-------------------------------");
				for (int i = 1; i <= master_num; i++)
				{
					if (!checkReceive[i])
					{
						address_i = i;
						temp_buf[0] = address_i;
						if (sendToWaitAck(_thisAddress, address_i, _thisAddress, address_i, REQUEST_DIRECT, NONE, NONE, NONE, NONE, temp_buf, sizeof(temp_buf)))
						{
							printRecvPacketHeader();
							Serial.print(i); //Serial.println("master is 1 hop node");
							checkReceive[i] = true;
							parentMaster[i] = _thisAddress;
							addRouteTo(_rxHeaderFrom, _rxHeaderFrom, Valid, 1);
							printRoutingTable();
						}
						else
						{
							uint8_t row_number = 1;
							uint8_t number_of_node;
							uint8_t node_list[34] = { 0 };

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
		}
	}
}
/****************************************************************
*FUNCTION NAME:  G_find_1stRow_master( )
*FUNCTION     : Gateway�� broadcast�� master�鿡�� ����� ��û �޼����� ����
������ ��ٸ���.
- ������ �����ϸ�, ����� ���̺� ����
*INPUT        : none
*OUTPUT       : 1 hop ������ ����
****************************************************************/
int8_t ToDatagram::G_find_1stRow_master()
{
	uint8_t num_of_routed_nodes = 0;
	Serial.println("send broadcast request message");
	Serial.println(masterBroadcastAddress);
	for (int i = 0; i < R_GATEWAY_SEND_NUM; i++)
	{
		send(_thisAddress, masterBroadcastAddress, _thisAddress, masterBroadcastAddress, REQUEST_BROADCAST, NONE, NONE, NONE, NONE, temp_buf, sizeof(temp_buf));
	}
	for (int i = 1; i <= master_num; i++)
	{
		/////////���ϰ� �־��ĸ� �̰� �������ϸ鼭 ack 5�� ���� �� �ð� ��� �Ұ���..�̰� ��ġ��, master 2 hop�ڵ尡�� �̰Ŷ� ����Ѻκ� 2hop���� 5�� �޵��� �ϰ�,,
		uint8_t dst = i;// convertToAddress(gatewayNumber, i, 0);
		uint8_t recvAckNum = 0;
		for (int j = 0; j < 1;j++)//DEFAULT_RETRIES; j++)
		{
			Serial.print("retry : ");
			Serial.println(j);
			send(_thisAddress, dst, _thisAddress, dst, REQUEST_TYPE, NONE, NONE, NONE, NONE, temp_buf, sizeof(temp_buf));
			startTime = millis();
			recvAckNum = 0;
			while (millis() - startTime < TIME_TERM * 5)//���� TIME_TERM��ŭ�� ��ٸ��� �ƹ��͵� �������� �������Ϸ��� ������ ���´µ� ����Ʈ���̿��� ���޴� ���� ���� �����Ƿ� ���⼭ ������ ��ٷ��ְ� �������ϴ°� ����..
			{
				SetReceive();
				if (available() && recvData(temp_buf) && _rxHeaderFrom == dst && _rxHeaderTo == _thisAddress && _rxHeaderType == REQUEST_ACK_TYPE)
				{
					recvAckNum++;
					if (recvAckNum >= R_MASTER_SEND_NUM * 0.8)
					{
						Serial.println(recvAckNum);
						Serial.println(R_MASTER_SEND_NUM * 0.8);
						addRouteTo(_rxHeaderFrom, _rxHeaderFrom, Valid, 1);
						printRecvPacketHeader();
						Serial.print(i); Serial.println("master is 1 hop node");
						checkReceive[i] = true;
						parentMaster[i] = _thisAddress;
						printRoutingTable();
						printTree();
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
*FUNCTION     :  Gateway��  1 hop ���鿡��, ������� �ȵ� ������ �ּҸ� ���̷ε忡 �����Ͽ� ������.

*INPUT        : unknown_node for next hop
*OUTPUT       : routing �ȵ� ����� ����
****************************************************************/
int8_t ToDatagram::G_find_2ndRow_master(uint8_t* unknown_node_list)
{
	Serial.println("[row2]");
	//row2 : 1hop�� master�鿡�� ���� ������� �ȵ� ������ �ּҸ� ������ ��ó�� ������ �ִ��� ���

	uint8_t node_list[34] = { 0 };
	uint8_t number_of_node;
	uint8_t number_of_unknown_node = 0;

	number_of_node = G_get_i_row_node_list(1, node_list);

	Serial.print("1hop's num : ");
	Serial.println(number_of_node);

	for (uint8_t i = 0; i < number_of_node; i++)//1hop node�鿡�� ��û
	{
		number_of_unknown_node = 0;
		for (int i = 1; i <= master_num; i++)
		{
			Serial.print(checkReceive[i]);
			Serial.print(" ");
		}
		Serial.println();
		for (uint8_t j = 1; j <= master_num; j++)//routing�� �ȵ� ������ �ּҸ� table�� ������ �˾Ƴ�
		{
			if (!checkReceive[j])
			{
				uint8_t address = j;// convertToAddress(gatewayNumber, j, 0);
				//routing�� �ȵ� �ֵ��� �ּҸ� ���۵Ǵ� �迭�� �ִ´�.
				Serial.print("unreceive : ");
				Serial.println(address);
				temp_buf[number_of_unknown_node] = address;
				//temp_buf[number_of_unknown_node * 2 + 1] = (uint8_t)(0x00FF & address);
				number_of_unknown_node++;
			}
		}
		if (number_of_unknown_node == 0)
			return 0;
		Serial.print("send to ");
		Serial.println(node_list[i]);


		//1hop�� �ְ� ������..test �ÿ��� �̷� ��찡 �������� �����δ� ���� ����.
		if (!sendToWaitAck(_thisAddress, node_list[i], _thisAddress, node_list[i], R2_REQUEST_TYPE, number_of_unknown_node, NONE, NONE, NONE, temp_buf, sizeof(temp_buf)))
		{//from, to, src, dst, type, data, flags, seqnum, hop, payload, size of payload, timeout
			//deleteRouteTo(node_list[i]);
			checkReceive[node_list[i]] = false;
			parentMaster[node_list[i]] = 0;
			/*for (uint8_t j = 0; j < i; j++)
			{
				temp_buf[0] = (uint8_t)node_list[i] >> 8;
				temp_buf[1] = (uint8_t)(0x00FF & node_list[i]);
				if (checkReceive[convertToMasterNumber(node_list[j])] && sendToWaitAck(_thisAddress, node_list[j], _thisAddress, node_list[j], R2_REQUEST_TYPE, 1, NONE, NONE, NONE, temp_buf, sizeof(temp_buf)))
				{
					//startTime = millis();
					//while (millis() - startTime < 10 * TIME_TERM)
					while(1)//�ؿ����� �������� ��� �Ͼ�ϱ� �󸶳� ��ٷ��� �ϴ��� ��..but ��·�� 1hop������ ������ ���ϱ� while�� �ص� ��.
					{
						SetReceive();
						if (available())
						{
							if (recvData(temp_buf) && _rxHeaderTo == _thisAddress && _rxHeaderType == R2_REQUEST_ACK_TYPE && _rxHeaderFrom == node_list[j])
							{
								send(_thisAddress, _rxHeaderFrom, _thisAddress, _rxHeaderFrom, ACK, NONE, NONE, NONE, NONE, temp_buf, sizeof(temp_buf));
								if (_rxHeaderData != 0)
								{
									printRecvPacketHeader();
									addRouteTo(node_list[i], node_list[j], Valid, 2);
									uint16_t masterNum = convertToMasterNumber(node_list[i]);
									parentMaster[masterNum] = _rxHeaderFrom;
									delay(100);
									checkReceive[masterNum] = true;
								}
								break;
							}
						}
					}
					if (checkReceive[convertToMasterNumber(node_list[i])])
						break;
				}
			}*/
			continue;
		}

		//startTime = millis();
		//while ((millis() - startTime) < number_of_unknown_node * 10 * TIME_TERM)

		while(1)//�ؿ����� �������� ��� �Ͼ�ϱ� �󸶳� ��ٷ��� �ϴ��� ��..but ��·�� 1hop������ ������ ���ϱ� while�� �ص� ��.
		{
			SetReceive();
			if (available())
			{
				if (recvData(temp_buf))
				{
					if (_rxHeaderTo == _thisAddress && _rxHeaderType == R2_REQUEST_ACK_TYPE && _rxHeaderFrom == node_list[i])
					{
						send(_thisAddress, _rxHeaderFrom, _thisAddress, _rxHeaderFrom, ACK, NONE, NONE, NONE, NONE, temp_buf, sizeof(temp_buf));
						printRecvPacketHeader();
						for (uint8_t j = 0; j < _rxHeaderData; j++)
						{
							uint8_t temp_addr = temp_buf[j];// word(temp_buf[j * 2], temp_buf[j * 2 + 1]);
							addRouteTo(temp_addr, _rxHeaderFrom, Valid, 2);
							uint8_t masterNum = temp_addr;// convertToMasterNumber(temp_addr);
							parentMaster[masterNum] = _rxHeaderFrom;
							checkReceive[masterNum] = true;
							Serial.println(masterNum);
							Serial.println(checkReceive[masterNum]);
						}
						printRoutingTable();
						printTree();
						break;
					}
				}
			}
		}
	}
	number_of_unknown_node = 0;
	for (uint8_t i = 1; i <= master_num; i++)
	{
		if (!checkReceive[i])
		{
			unknown_node_list[number_of_unknown_node++] = i;// convertToAddress(gatewayNumber, i, 0);
		}
	}
	return number_of_unknown_node;
}
/****************************************************************
*FUNCTION NAME: G_find_multihop_node ()
*FUNCTION     :  Gateway finished the routing discovery upto 2nd row.
Gateway tries to find the 3rd and more row nodes.
- ���� 2nd row node���� ������, ���� routing�� �ȵ� ����� �ּҸ�  payload�� �����Ͽ� �����Ѵ�.
- 3rd row ������ �����Ŀ���, routing�� �ȵ� ����� �ּҸ�  payload�� �����Ͽ�
�ֱٿ� ���� row�� ���鿡��  multihop routing ��û �޽����� �����Ѵ�.
*INPUT        :
*OUTPUT       :
****************************************************************/
int8_t ToDatagram::G_find_multihop_node(uint8_t number_of_unknown_node, uint8_t* unknown_node_list)
{
	uint8_t row_number = 2;
	uint8_t number_of_node;
	uint8_t node_list[34] = { 0 };
	uint8_t cnt = number_of_unknown_node;

	for (int i = 1; i <= master_num; i++)
	{
		Serial.print(checkReceive[i]);
		Serial.print(" ");
	}
	Serial.println();

	number_of_node = G_get_i_row_node_list(row_number, node_list);  // the gateway needs the list of 2nd row masters which will send Multi_HOP_REQUEST
																	// to unknown nodes.

	Serial.println("[MULTI_HOP]");
	while (number_of_node != 0)
	{
		for (uint8_t i = 0; i < cnt; i++)
		{
			if (G_request_path_one_by_one(unknown_node_list[i], row_number, node_list, number_of_node, REQUEST_MULTI_HOP))
			{
				if (--number_of_unknown_node == 0)
					return 0;
			}
		}
		number_of_node = G_get_i_row_node_list(++row_number, node_list);
	}
	return number_of_unknown_node;
}
/****************************************************************
*FUNCTION NAME:  G_request_path_one_by_one(uint16_t unknown_address, byte row_number, uint16_t* node_list, byte number_of_node, uint8_t type)
*FUNCTION     :	 node_list�� �ִ� ���鿡�� �ּҰ� unknown_address�� ��尡 ������ �ִ��� ���

*INPUT        :  ������ �ּ�
*OUTPUT       :  ���� ����
****************************************************************/
bool ToDatagram::G_request_path_one_by_one(uint8_t unknown_address, uint8_t row_number, uint8_t* node_list, byte number_of_node, uint8_t type)
{
	//_driver.Init(ch);
	Serial.println("G_request_path_one_by_one");
	uint8_t masterNum = unknown_address;// convertToMasterNumber(unknown_address);

	for (uint8_t i = 0; i < number_of_node; i++)
	{
		if (node_list[i] == unknown_address)
			continue;
		if (!checkReceive[node_list[i]])
			continue;
		temp_buf[0] = unknown_address;
		// (uint8_t)(unknown_address >> 8);
		//temp_buf[1] = (uint8_t)(0x00FF & unknown_address);


		uint8_t next_hop = getRouteTo(node_list[i])->next_hop;

		if (!sendToWaitAck(_thisAddress, next_hop, _thisAddress, node_list[i], type, NONE, NONE, NONE, NONE, temp_buf, sizeof(temp_buf)))
		{
			checkReceive[next_hop] = false;
			parentMaster[next_hop] = -1;
			G_discoverNewPath(next_hop, getRouteTo(next_hop)->hop);
			continue;
		}
		delay(100);
		startTime = millis();
		while((millis() - startTime) < (row_number + 1) * 4 * DEFAULT_RETRIES * TIME_TERM)
		{
			SetReceive();
			if (available())
			{
				if (recvData(temp_buf) && _rxHeaderTo == _thisAddress && _rxHeaderDestination == _thisAddress)
				{
					printRecvPacketHeader();
					send(_thisAddress, _rxHeaderFrom, _thisAddress, _rxHeaderFrom, ACK, NONE, NONE, NONE, NONE, temp_buf, sizeof(temp_buf));
					if (_rxHeaderSource == unknown_address && (_rxHeaderType == REQUEST_MULTI_HOP_ACK || _rxHeaderType == REQUEST_DIRECT_ACK) && _rxHeaderFrom == next_hop)
					{
						addRouteTo(_rxHeaderSource, _rxHeaderFrom, Valid, _rxHeaderHop + 1);
						parentMaster[masterNum] = temp_buf[0];// word(temp_buf[0], temp_buf[1]);
						checkReceive[masterNum] = true;
						printRoutingTable();
						printTree();
						return true;
					}
					else if (_rxHeaderType == NACK)
					{
						Serial.println("receive NACK");
						uint8_t reroutingAddr = temp_buf[0]; //word(temp_buf[0], temp_buf[1]);
						//deleteRouteTo(reroutingAddr);
						checkReceive[reroutingAddr] = false;
						parentMaster[reroutingAddr] = -1;
						G_discoverNewPath(reroutingAddr, getRouteTo(reroutingAddr)->hop);
						return false;
					}
					else if (_rxHeaderData == 0)
						break;
				}
			}
		}
	}
	return false;
}
/////��� routing ���� �߻���///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/****************************************************************
*FUNCTION NAME:  G_discoverNewPath(uint16_t address, uint8_t row_number, bool child)
*FUNCTION     :	 �ٽ� routing ����

*INPUT        :
*OUTPUT       :
****************************************************************/
bool ToDatagram::G_discoverNewPath(uint8_t address, uint8_t row_number)//, bool child)
{
	Serial.print("G_discoverNewPath : ");
	Serial.println(address);
	uint8_t cpy_row_number = row_number;
	uint8_t number_of_node;
	uint8_t node_list[34] = { 0 };

	//row_number = getRouteTo(address)->hop;
	if (row_number != getRouteTo(address)->hop)
	{
		if (number_of_node = G_get_i_row_node_list(row_number, node_list))
		{
			if (G_request_path_one_by_one(address, row_number, node_list, number_of_node, REQUEST_DIRECT))
				return true;
			else
			{
				checkReceive[address] = false;
				parentMaster[address] = -1;
				G_discoverNewPath(address, row_number + 1);
			}
		}
		else
		{
			Serial.println("#######################################routing fail###########################################");
			return false;//�̷� ���� ���� ������..
		}
	}

	while (row_number > -1)
	{
		number_of_node = G_get_i_row_node_list(row_number, node_list);
		/*if (number_of_node == 1 && node_list[0] == address)//address�� ���� hop count�� ���� ��尡 ����, ������ G_request_path_one_by_one���� �ذ�
			row_number--;
		else */if (row_number == 0)
		{
			temp_buf[0] = address;
			//temp_buf[1] = (uint8_t)(0x00FF & address);
			uint8_t recvAckNum = 0;
			for (byte i; i < sizeof(temp_buf); i++)
				buffer[i] = temp_buf[i];
			for (int j = 0; j < DEFAULT_RETRIES; j++)	
			{
				send(_thisAddress, address, _thisAddress, address, REQUEST_DIRECT, NONE, NONE, NONE, NONE, buffer, sizeof(buffer));
				startTime = millis();
				recvAckNum = 0;
				while (millis() - startTime < TIME_TERM * 5)//���� TIME_TERM��ŭ�� ��ٸ��� �ƹ��͵� �������� �������Ϸ��� ������ ���´µ� ����Ʈ���̿��� ���޴� ���� ���� �����Ƿ� ���⼭ ������ ��ٷ��ְ� �������ϴ°� ����..
				{
					SetReceive();
					if (available() && recvData(temp_buf) && _rxHeaderFrom == address && _rxHeaderTo == _thisAddress && _rxHeaderType == REQUEST_DIRECT_ACK)
					{
						recvAckNum++;
						if (recvAckNum >= R_MASTER_SEND_NUM * 0.8)
						{
							printRecvPacketHeader();
							uint8_t masterNumber = address;
							Serial.print(masterNumber); Serial.println("master is 1 hop node");
							checkReceive[masterNumber] = true;
							parentMaster[masterNumber] = _thisAddress;
							addRouteTo(_rxHeaderFrom, _rxHeaderFrom, Valid, 1);
							printRoutingTable();
							printTree();
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
			if (G_request_path_one_by_one(address, row_number, node_list, number_of_node, REQUEST_DIRECT))
				break;
			row_number--;
		}
	}
	if (row_number == -1)//hop count�� �������Ѿ� ã�� �� �ִ� ���
	{
		checkReceive[address] = false;
		parentMaster[address] = -1;
		G_discoverNewPath(address, cpy_row_number + 1);
	}

	if (!checkReceive[address])
	{
		Serial.print(address);
		Serial.println("-th node is error");
	}

	//if (!child)
	//{
		number_of_node = G_get_i_row_node_list(cpy_row_number, node_list);
		for (int i = 1; i <= master_num; i++)//address�� �ڽ��� path ���� ����
		{
			//Serial.println("child node rerouting!!");
			if (parentMaster[i] == address)
			{
				uint8_t temp_address = i;
				checkReceive[temp_address] = false;
				parentMaster[temp_address] = -1;
				G_discoverNewPath(temp_address, getRouteTo(temp_address)->hop);// , true);
				unRecvCnt[temp_address] = 0;
				//changeNextHop(temp_address);
				//G_request_path_one_by_one(convertToAddress(gatewayNumber, i, 0), cpy_row_number, node_list, number_of_node, REQUEST_DIRECT);
			}
		}
	//}
	Serial.println(address);
	Serial.println("where?");
	printTree();
	return true;
}
void ToDatagram::changeNextHop(uint8_t address, uint8_t hopCount)
{
	//Serial.println("changeNextHop");
	//Serial.print("hopCount : ");
	//Serial.println(hopCount);
	//printRoutingTable();
	for (int i = 1; i <= master_num; i++)
	{
		if (parentMaster[i] == address)
		{
			Serial.println(hopCount);
			Serial.println(getRouteTo(i)->hop);
			addRouteTo(i, getRouteTo(address)->next_hop, Valid, getRouteTo(i)->hop - hopCount);
			//addRouteTo(i, getRouteTo(address)->next_hop, Discovering, getRouteTo(i)->hop - hopCount);
			changeNextHop(i, hopCount);
		}
	}
	//printRoutingTable();
}
/****************************************************************
*FUNCTION NAME:  G_find_error_node(uint16_t address)
*FUNCTION     :	 error_node ã�Ƽ� rerouting�Լ� ȣ��

*INPUT        :
*OUTPUT       :
****************************************************************/
void ToDatagram::G_find_error_node(uint8_t address)
{
	Serial.println("find_error_node");
	uint8_t path[34] = { 0 };
	getPath(address, path);
	for (int i = 0; i < getRouteTo(address)->hop; i++)
	{
		Serial.println(path[i]);
	}
	for (int i = getRouteTo(address)->hop - 1; i >= 0; i--)
	{
		if (!sendToWaitAck(_thisAddress, getRouteTo(path[i])->next_hop, _thisAddress, path[i], CHECK_ROUTING, NONE, NONE, NONE, NONE, temp_buf, sizeof(temp_buf)))
		{
			//deleteRouteTo(path[i]);
			if (unRecvCnt[path[i]]++ > MAX_UN_RECV)
			{
				checkReceive[path[i]] = false;
				parentMaster[path[i]] = -1;
				G_discoverNewPath(path[i], getRouteTo(path[i])->hop);
				unRecvCnt[path[i]] = 0;
			}
			break;
		}
		if (getRouteTo(path[i])->hop == 1)
		{
			send(_thisAddress, getRouteTo(path[i])->next_hop, _thisAddress, getRouteTo(path[i])->next_hop, ACK, NONE, NONE, NONE, NONE, temp_buf, sizeof(temp_buf));
			continue;
		}
		startTime = millis();
		while (millis() - startTime < DEFAULT_RETRIES * getRouteTo(path[i])->hop * TIME_TERM * 4)
		{
			SetReceive();
			if (available())
			{
				if (recvData(temp_buf) && _rxHeaderTo == _thisAddress)
				{
					if (_rxHeaderType == NACK)
					{
						send(_thisAddress, getRouteTo(path[i])->next_hop, _thisAddress, getRouteTo(path[i])->next_hop, ACK, NONE, NONE, NONE, NONE, temp_buf, sizeof(temp_buf));
						Serial.println("NACK");
						//deleteRouteTo(path[i]);
						uint8_t temp_address = temp_buf[0];// word(temp_buf[0], temp_buf[1]);
						if (unRecvCnt[temp_address]++ > MAX_UN_RECV)
						{
							checkReceive[temp_address] = false;
							parentMaster[temp_address] = -1;
							G_discoverNewPath(temp_address, getRouteTo(temp_address)->hop);
							unRecvCnt[temp_address] = 0;
							return;
						}
					}
				}
			}
		}
	}
}


/****************************************************************
*FUNCTION NAME:  M_findCandidateParents( )   20180308Ȯ�� ��
*FUNCTION     :

*INPUT        :
*OUTPUT       :
****************************************************************/
void ToDatagram::M_findCandidateParents()
{
	//Serial.println("choose candidate");
	if (_rxHeaderType == REQUEST_ACK_TYPE || _rxHeaderType == R2_REQUEST_ACK_TYPE || _rxHeaderType == REQUEST_MULTI_HOP_ACK)
	{
		//Serial.print("choose candidate2		");
		//Serial.print(candidateRSSI);
		//Serial.print(" : ");
		//Serial.println(_driver.rssi);
		if (receivedType == 0)
		{
			//Serial.println("receive propagation");
			receivedType = _rxHeaderType;
			candidateAddress = _rxHeaderFrom;
			candidateRSSI = _driver.rssi;
			//Serial.print("My choice : ");
			//Serial.println(candidateAddress, HEX);
		}
		else if (receivedType == _rxHeaderType)//Type check ���ϸ�, ��� ���� ������ ��带 �θ�� �����ϰ� �Ǿ hop���� ��� �þ��.
		{
			if (_driver.rssi >= candidateRSSI)
			{
				//Serial.println("receive propagation");
				candidateAddress = _rxHeaderFrom;
				candidateRSSI = _driver.rssi;
				//Serial.print("My choice : ");
				//Serial.println(_rxHeaderFrom, HEX);
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
void ToDatagram::M_find2ndRowMasters()
{
	uint8_t childNodeList[32] = { 0 };
	uint8_t unknown_address;
	uint8_t temp[20];
	Serial.println("receive R2 request and I succeed R1");

	uint8_t count = _rxHeaderData;
	uint8_t child_node_num = 0;

	for (int8_t i = 0; i < count; i++)//temp_buf�� ���ŵǴ� data�� ��� �����Ƿ� temp�� �����صξ����
	{
		temp[i] = temp_buf[i];
		//temp[i * 2 + 1] = temp_buf[i * 2 + 1];
	}
	for (int8_t i = 0; i < count; i++)
	{
		Serial.println("send R2 reequest to 2 hop");
		unknown_address = temp[i];// * 2], temp[i * 2 + 1]);
		uint8_t recvAckNum = 0;
		for (int j = 0; j < 1;j++)//DEFAULT_RETRIES; j++)
		{
			Serial.print("retry : ");
			Serial.println(j);
			recvAckNum = 0;
			send(_thisAddress, unknown_address, _thisAddress & 0x00, unknown_address, R2_REQUEST_REAL_TYPE, NONE, NONE, NONE, 1, temp_buf, sizeof(temp_buf));
			startTime = millis();
			while (millis() - startTime < TIME_TERM * 5)
			{
				SetReceive();
				if (available() && recvData(temp_buf) && _rxHeaderFrom == unknown_address && _rxHeaderTo == _thisAddress && _rxHeaderType == R2_REQUEST_ACK_TYPE)
				{
					recvAckNum++;
					if (recvAckNum >= R_MASTER_SEND_NUM * 0.8)
					{
						addRouteTo(_rxHeaderFrom, _rxHeaderFrom, Valid, 1);
						printRecvPacketHeader();
						printRoutingTable();
						childNodeList[child_node_num++] = _rxHeaderFrom;;
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
		temp_buf[i] = childNodeList[i];
		//temp_buf[i * 2 + 1] = lowByte(childNodeList[i]);
	}
	//from, to, src, dst, type, data, flags, seqnum, hop
	sendToWaitAck(_thisAddress, _thisAddress & 0x00, _thisAddress, _thisAddress & 0x00, R2_REQUEST_ACK_TYPE, child_node_num, NONE, NONE, 1, temp_buf, sizeof(temp_buf));
}
/****************************************************************
*FUNCTION NAME:M_masterSendRoutingReply( )
*FUNCTION     :

*INPUT        :
*OUTPUT       :
****************************************************************/
void ToDatagram::M_masterSendRoutingReply()
{
	//Serial.print("My candidate : ");
	//Serial.println(candidateAddress, HEX);
	//Serial.print("Receved from : ");
	//Serial.println(_rxHeaderFrom, HEX);
	if (candidateAddress == _rxHeaderFrom || candidateAddress == 0)
	{
		addRouteTo(_rxHeaderSource, _rxHeaderFrom, Valid, _rxHeaderHop + 1);
		printRoutingTable();
		uint8_t type;
		if (_rxHeaderType == R2_REQUEST_REAL_TYPE)
			type = R2_REQUEST_ACK_TYPE;
		else if (_rxHeaderType == REQUEST_MULTI_HOP)
		{
			type = REQUEST_MULTI_HOP_ACK;
			temp_buf[0] = _rxHeaderFrom;
			//temp_buf[1] = (uint8_t)(0x00FF & _rxHeaderFrom);
		}
		//from, to, src, dst, type, data, flags, seqnum, hop
		for (uint8_t i = 0; i < R_MASTER_SEND_NUM; i++)
		{
			send(_thisAddress, _rxHeaderFrom, _thisAddress, _rxHeaderFrom, type, NONE, NONE, NONE, NONE, temp_buf, sizeof(temp_buf));
		}
	}
}

void ToDatagram::newMaster()
{
	unsigned long waitTime = millis();
	uint8_t receivedNum[34] = { 0 };
	uint16_t candidate_parent;
	uint8_t maxReceiveNum = 0;
	uint8_t hop = 0;

	while (millis() - waitTime < 10000)
	{
		SetReceive();
		if (available())
		{
			if (recvData(temp_buf))
			{
				receivedNum[_rxHeaderFrom]++;
			}
		}
	}

	for (uint8_t i = 1; i <= 34; i++)
	{
		if (maxReceiveNum < receivedNum[i])
		{
			candidate_parent = i;
			maxReceiveNum = receivedNum[i];
			Serial.print("i : ");
			Serial.println(i);
		}
	}

	while (!sendToWaitAck(_thisAddress, candidate_parent, _thisAddress, candidate_parent, NEW_NODE_REGISTER, NONE, NONE, NONE, NONE, temp_buf, sizeof(temp_buf), 1000))
	{
		while (millis() - waitTime < 10000)
		{
			SetReceive();
			if (available())
			{
				if (recvData(temp_buf))
				{
					receivedNum[_rxHeaderFrom]++;
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

	addRouteTo(_thisAddress & 0x00, candidate_parent, Valid, _rxHeaderHop + 1);//master�� hop count ���� �ǳ�..�˾ƾߵ�...
	printRoutingTable();

	waitTime = millis();
	bool receiveFromG = false;

	while (millis() - waitTime < 600000)
	{
		SetReceive();
		if (available())
		{
			if (recvData(temp_buf))
			{
				if (_rxHeaderTo == _thisAddress && _rxHeaderType == CHECK_ROUTING)
				{
					sendToWaitAck(_thisAddress, getRouteTo(_thisAddress & 0x00)->next_hop, _thisAddress, _thisAddress & 0x00, CHECK_ROUTING_ACK, NONE, NONE, NONE, NONE, temp_buf, sizeof(temp_buf));
					receiveFromG = true;
					break;
				}
			}
		}
	}
	if(!receiveFromG)
		newMaster();
}
//////////////////////