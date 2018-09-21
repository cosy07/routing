//2017-07-17
// inDatagram.cpp
//
// Copyright (C) 2011 Mike McCauley
// $Id: RHinDatagram.cpp,v 1.6 2014/05/23 02:20:17 mikem Exp $

#include "inDatagram_STM.h"


////////////////////////////////////////////////////////////////////
// Public methods
void DGInit(uint8_t masterNumber, uint8_t thisAddress, byte ch)
{
	DGgatewayNumber = 0;
	DGmasterNumber = masterNumber;
	DGnum_of_slave = 0;

	DGsendingTime = -60000;

	DGsetThisAddress(thisAddress);
	e_Init(ch);
	DGch = ch;
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
	DGsetHeaderFrom(thisAddress);
	DG_thisAddress = thisAddress;
}

void DGsetHeaderFrom(uint8_t from)
{
	e_setHeaderFrom(from);
}

void DGsetHeaderTo(uint8_t to)
{
	e_setHeaderTo(to);
}
void  DGsetHeaderMaster(uint8_t master)
{
	e_setHeaderMaster(master);
}
void DGsetHeaderType(uint8_t type)
{
	e_setHeaderType(type);
}
void DGsetNumOfSlave(uint8_t num)
{
	DGnum_of_slave = num;
}
uint8_t DGheaderFrom()
{
	return e_headerFrom();
}
uint8_t DGheaderTo()
{
	return e_headerTo();
}
uint8_t DGheaderMaster()
{
	return(e_headerMaster());
}
uint8_t DGheaderType()
{
	return(e_headerType());
}
uint8_t DGgetThisAddress()
{
	return DG_thisAddress;
}
uint8_t DGgetMasterNumber()
{
	return DGmasterNumber;
}
bool DGavailable()
{
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
		DG_rxHeaderMaster = DGheaderMaster();
		DG_rxHeaderType = DGheaderType();
		return (size - CC1120_HEADER_LEN);
	}
	return 0;
}

//////////////////////////////////???? ??? ?? ??//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//=====================================================================================================
// 2017-04-26 ver.1
void DGsend(uint8_t from, uint8_t to, uint8_t master, uint8_t type, uint8_t* temp_buf, uint8_t size)
{
	while (millis() - DGsendingTime < 1000);//??? ?? 20?? ??? reset???? sendingTime ?? int?? ???
	DGsetHeaderFrom(from);
	DGsetHeaderTo(to);
	DGsetHeaderMaster(master);
	DGsetHeaderType(type);
	e_SendData(temp_buf, size);
	DGsendingTime = millis();
}
bool DGsendToWaitAck(uint8_t from, uint8_t to, uint8_t master, uint8_t type, uint8_t* temp_buf, uint8_t size, unsigned long time)
{
	unsigned long startTime = 0;
	DGsetHeaderFrom(from);
	DGsetHeaderTo(to);
	DGsetHeaderMaster(master);
	DGsetHeaderType(type);
	time = 2000;

	for (int i = 0; i < DEFAULT_RETRIES; i++)
	{
		while (millis() - DGsendingTime < 1000);
		//sendingTime = millis();//??? ?? 20?? ??? reset???? sendingTime ?? int?? ???, TIME_TERM? 1000 ???? ?? ??? ??? ACK ?? ? send???? ??? ?? ? ?? ?? ??? ???? ??

		printf("retry : %d", i);
		e_SendData(temp_buf, size);
		DGsendingTime = millis();
		startTime = millis();
		while (millis() - startTime < TIME_TERM)
		{
			DGSetReceive();
			if (DGavailable())
			{
				if (DGrecvData(temp_buf))// &&  _rxHeaderFrom == to)
				{
					if (DG_rxHeaderTo == DG_thisAddress || DG_rxHeaderTo == 0xFF)// && _rxHeaderType == ACK)
					{
						printf("success\r\n");
						return true;
					}
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
	printf("_rxHeaderFrom : %d\r\n", DG_rxHeaderFrom);
	printf("_rxHeaderTo : %d\r\n", DG_rxHeaderTo);
	printf("_rxHeaderMaster : %d\r\n", DG_rxHeaderMaster);
	printf("_rxHeaderType : %d\r\n", DG_rxHeaderType);
}