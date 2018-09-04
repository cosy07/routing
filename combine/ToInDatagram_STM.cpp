//2017-07-17
// ToInDatagram.cpp
//
// Copyright (C) 2011 Mike McCauley
// $Id: RHToInDatagram.cpp,v 1.6 2014/05/23 02:20:17 mikem Exp $

#include <to_cc1120_STM1.h>
#include <ToInDatagram_STM.h>

ToInDatagram::ToInDatagram(TO_ELECHOUSE_CC1120& driver, uint8_t masterNumber, uint8_t thisAddress)//, uint8_t master_num)
	:
	_driver(driver),
	_thisAddress(thisAddress),
	masterNumber(masterNumber)
{
}

////////////////////////////////////////////////////////////////////
// Public methods
void ToInDatagram::init()
{
	setThisAddress(_thisAddress);
	_driver.Init();

}
////////////////////////////////////////////////////////////////////
// Public methods
void ToInDatagram::init(byte ch)  // Setup Channel number
{
	setThisAddress(_thisAddress);
	_driver.Init(ch);
	ch = ch;
}

////////////////////////////////////////////////////////////////////
// Public methods
void ToInDatagram::SetReceive()
{
	_driver.SetReceive();
}

void  ToInDatagram::setThisAddress(uint8_t thisAddress)
{
	_driver.setThisAddress(thisAddress);
	// Use this address in the transmitted FROM header
	setHeaderFrom(thisAddress);
	_thisAddress = thisAddress;
}

void ToInDatagram::setHeaderFrom(uint8_t from)
{
	_driver.setHeaderFrom(from);
}

void ToInDatagram::setHeaderTo(uint8_t to)
{
	_driver.setHeaderTo(to);
}
void  ToInDatagram::setHeaderMaster(uint8_t master)
{
	_driver.setHeaderMaster(master);
}
void ToInDatagram::setHeaderType(uint8_t type)
{
	_driver.setHeaderType(type);
}
void ToInDatagram::setNumOfSlave(uint8_t num)
{
	num_of_slave = num;
}

uint8_t ToInDatagram::headerFrom()
{
	uint8_t temp = _driver.headerFrom();
	return temp;
}
uint8_t ToInDatagram::headerTo()
{
	uint8_t temp = _driver.headerTo();
	return temp;
}
uint8_t ToInDatagram::headerMaster()
{
	return(_driver.headerMaster());
}
uint8_t ToInDatagram::headerType()
{
	return(_driver.headerType());
}
uint8_t ToInDatagram::getThisAddress()
{
	return _thisAddress;
}
uint8_t ToInDatagram::getMasterNumber()
{
	return masterNumber;
}
bool ToInDatagram::available()
{
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

byte  ToInDatagram::recvData(uint8_t* buf)
{
	byte size;
	if (size = _driver.inReceiveData(buf))
	{
		_rxHeaderFrom = headerFrom();
		_rxHeaderTo = headerTo();
		_rxHeaderMaster = headerMaster();
		_rxHeaderType = headerType();
		return (size - CC1120_HEADER_LEN);
	}
	return 0;
}

//////////////////////////////////여기부터 라우팅 하는 코드//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//=====================================================================================================
// 2017-04-26 ver.1
void ToInDatagram::send(uint8_t from, uint8_t to, uint8_t master, uint8_t type, uint8_t* temp_buf, uint8_t size)
{
	while (millis() - sendingTime < 1000);//어차피 대충 20일에 한번은 reset된다하면 sendingTime 타입 int여도 무방함
	setHeaderFrom(from);
	setHeaderTo(to);
	setHeaderMaster(master);
	setHeaderType(type);
	_driver.inSendData(temp_buf, size);
	sendingTime = millis();
}
bool ToInDatagram::sendToWaitAck(uint8_t from, uint8_t to, uint8_t master, uint8_t type, uint8_t* temp_buf, uint8_t size, unsigned long time)
{
	unsigned long startTime = 0;
	setHeaderFrom(from);
	setHeaderTo(to);
	setHeaderMaster(master);
	setHeaderType(type);
	time = 5000;

	for (int i = 0; i < DEFAULT_RETRIES; i++)
	{
		while (millis() - sendingTime < 1000);
		//sendingTime = millis();//어차피 대충 20일에 한번은 reset된다하면 sendingTime 타입 int여도 무방함, TIME_TERM이 1000 이상이면 이거 없어도 되지만 ACK 보낼 때 send함수쓰고 다음에 바로 이 함수 쓰는 경우가 있으므로 필요

		Serial.print("retry : ");
		Serial.println(i);
		_driver.inSendData(temp_buf, size);
		sendingTime = millis();
		startTime = millis();
		while (millis() - startTime < TIME_TERM)
		{
			SetReceive();
			if (available())
			{
				if (recvData(temp_buf))// &&  _rxHeaderFrom == to)
				{
					if (_rxHeaderTo == _thisAddress || _rxHeaderTo == 0xFF)// && _rxHeaderType == ACK)
					{
						Serial.println("success");
						return true;
					}
				}
			}
		}
		delay(time);
	}
	return false;
}
void ToInDatagram::printRecvPacketHeader()
{
	Serial.println("-----------------------------------------------------------------------------");
	Serial.print("_rxHeaderFrom : ");
	Serial.println(_rxHeaderFrom);
	Serial.print("_rxHeaderTo : ");
	Serial.println(_rxHeaderTo);
	Serial.print("_rxHeaderMaster : ");
	Serial.println(_rxHeaderMaster);
	Serial.print("_rxHeaderType : ");
	Serial.println(_rxHeaderType);
}