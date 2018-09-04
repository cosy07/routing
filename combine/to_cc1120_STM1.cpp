//2017-07-17 
/*                     CC1120.cpp
*/
#include "TO_CC1120_STM1.h"
#include <Arduino.h>

/****************************************************************/
#define 	WRITE_BURST     	0x40						//write burst
#define 	WRITE_SINGLE     	0x00						//write burst
#define 	READ_SINGLE     	0x80						//read single
#define 	READ_BURST      	0xC0						//read burst
#define 	BYTES_IN_RXFIFO     0x7F  						//byte number in RXfifo

/****************************************************************/


/****************************************************************
*FUNCTION NAME:SpiInit
*FUNCTION     :spi communication initialization
*INPUT        :none
*OUTPUT       :none
****************************************************************/
void TO_ELECHOUSE_CC1120::SpiInit(void)
{
	SCK_PIN = PA5;
	MISO_PIN = PA6;
	MOSI_PIN = PA7;
	SS_PIN = PA4;
	RE_PIN = PB6;
	TE_PIN = PB7;

	//RE_PIN = PB9; //에어컨회사
	//TE_PIN = PB8;

	GDO0 = PB0; // GPIO0 or GPIO2
	RST_PIN = PB1;
	GDO3 = PB10;


	SPI.begin();
	SPI.setBitOrder(MSBFIRST); // Set the SPI_1 bit order

							   //SPI.setDataMode(SPI_MODE0); //Set the  SPI_2 data mode 0
	SPI.setClockDivider(SPI_CLOCK_DIV128);

	pinMode(SS_PIN, OUTPUT);

	digitalWrite(SS_PIN, HIGH); // manually take CSN high between spi transmissions

	pinMode(RST_PIN, OUTPUT); //8

	delay(100);
	Serial.print("SS_PIN ");  Serial.print(SS_PIN);
	Serial.print("   GDO0 ");  Serial.println(GDO0);

	// enable SPI Master, MSB, SPI mode 0, FOSC/4
	SpiMode(0);
}
/****************************************************************
*FUNCTION NAME:SpiMode
*FUNCTION     :set spi mode
*INPUT        :        config               mode
(0<<CPOL) | (0 << CPHA)		 0
(0<<CPOL) | (1 << CPHA)		 1
(1<<CPOL) | (0 << CPHA)		 2
(1<<CPOL) | (1 << CPHA)		 3
*OUTPUT       :none
****************************************************************/
void TO_ELECHOUSE_CC1120::SpiMode(byte config)
{
	SPI.setDataMode(config);
}

/****************************************************************
*FUNCTION NAME: GDO_Set()
*FUNCTION     : set GDO0,GDO2 pin
*INPUT        : none
*OUTPUT       : none
****************************************************************/
void TO_ELECHOUSE_CC1120::GDO_Set(void)
{
	pinMode(GDO0, INPUT);
	digitalWrite(GDO0, HIGH);       // turn on pullup resistors
	pinMode(GDO3, INPUT);
	digitalWrite(GDO3, HIGH);
}

/****************************************************************
*FUNCTION NAME:Reset
*FUNCTION     :CC1101 reset //details refer datasheet of CC1101/CC1100//
*INPUT        :none
*OUTPUT       :none
****************************************************************/
void TO_ELECHOUSE_CC1120::Reset(void)
{


	digitalWrite(RST_PIN, LOW);
	// give the sensor time to set up:
	delay(100);
	digitalWrite(RST_PIN, HIGH);
	// give the sensor time to set up:
	delay(100);
	digitalWrite(SS_PIN, LOW);
	while (digitalRead(MISO_PIN)) Serial.println(" MISO 1");
	SpiTransfer(SRES);
	while (digitalRead(MISO_PIN))Serial.println(" 2");
	digitalWrite(SS_PIN, HIGH);
}
/****************************************************************
*FUNCTION NAME: Constructor
*FUNCTION     :
*INPUT        :none
*OUTPUT       :none
****************************************************************/
TO_ELECHOUSE_CC1120::TO_ELECHOUSE_CC1120() :
	e_thisAddress(0),
	e_txHeaderFrom(0xAA),
	e_txHeaderTo(0xBB),
	e_txHeaderSource(0xCC),
	e_txHeaderDestination(0xDD),
	e_txHeaderType(0xEE),
	e_txHeaderData(0xDF),
	e_txHeaderFlags(0xFF),
	e_txHeaderSeqNum(0x00)
{

}

/****************************************************************
*FUNCTION NAME:Init
*FUNCTION     :CC1101 initialization
*INPUT        :none
*OUTPUT       :none
****************************************************************/
void TO_ELECHOUSE_CC1120::Init(void)
{
	SpiInit();					//spi initialization
	GDO_Set();					//GDO set


	digitalWrite(SS_PIN, HIGH);
	digitalWrite(SCK_PIN, HIGH);
	digitalWrite(MOSI_PIN, LOW);
	pinMode(RE_PIN, OUTPUT);
	pinMode(TE_PIN, OUTPUT);
	EnableLNA();

	Reset();
	RegConfigSettings(0);
	//ManualCalibration();

}

/****************************************************************
*FUNCTION NAME:Init
*FUNCTION     :CC1101 initialization
*INPUT        :none
*OUTPUT       :none
****************************************************************/
void TO_ELECHOUSE_CC1120::Init(byte f)
{

	SpiInit();					//spi initialization
	GDO_Set();					//GDO set
	byte i, t;

	digitalWrite(SS_PIN, HIGH);
	digitalWrite(SCK_PIN, HIGH);
	digitalWrite(MOSI_PIN, LOW);
	pinMode(RE_PIN, OUTPUT);
	pinMode(TE_PIN, OUTPUT);
	EnableLNA();

	Reset();
	RegConfigSettings(f);
	//ManualCalibration();

}


/****************************************************************
*FUNCTION NAME:SpiWriteReg
*FUNCTION     :CC1101 write data to register
*INPUT        :addr: register address; value: register value
*OUTPUT       :none
****************************************************************/
void TO_ELECHOUSE_CC1120::SpiWriteReg(uint16_t addr, byte value)
{
	uint8_t tempExt = (uint8_t)(addr >> 8);
	uint8_t tempAddr = (uint8_t)(addr & 0x00FF);

	if (!tempExt)
	{		/* Pull CS_N low and wait for SO to go low before communication starts */
		digitalWrite(SS_PIN, LOW);
		//Sean delay(100);
		while (digitalRead(MISO_PIN));
		SpiTransfer(tempAddr);
		SpiTransfer(value);
		digitalWrite(SS_PIN, HIGH);
	}
	else if (tempExt == 0x2F)
	{
		digitalWrite(SS_PIN, LOW);
		while (digitalRead(MISO_PIN));
		SpiTransfer(tempExt);
		SpiTransfer(tempAddr);
		SpiTransfer(value);
		digitalWrite(SS_PIN, HIGH);
	}
}






/****************************************************************
*FUNCTION NAME:SpiWriteBurstReg
*FUNCTION     :CC1101 write burst data to register
*INPUT        :addr: register address; buffer:register value array; num:number to write
*OUTPUT       :none
****************************************************************/
void TO_ELECHOUSE_CC1120::SpiWriteBurstReg(byte addr, byte *buffer, byte num)
{
	byte i, temp;


	temp = addr | WRITE_BURST;
	digitalWrite(SS_PIN, LOW);
	while (digitalRead(MISO_PIN));
	SpiTransfer(temp);
	for (i = 0; i < num; i++)
	{
		SpiTransfer(buffer[i]);
	}

	digitalWrite(SS_PIN, HIGH);
}

/****************************************************************
*FUNCTION NAME:SpiStrobe
*FUNCTION     :CC1101 Strobe
*INPUT        :strobe: command; //refer define in CC1101.h//
*OUTPUT       :none
****************************************************************/
byte TO_ELECHOUSE_CC1120::SpiStrobe(byte strobe)
{
	byte sts;
	digitalWrite(SS_PIN, LOW);
	while (digitalRead(MISO_PIN));
	sts = SpiTransfer(strobe);
	digitalWrite(SS_PIN, HIGH);
	return sts;
}

/****************************************************************
*FUNCTION NAME:SpiReadReg
*FUNCTION     :CC1101 read data from register
*INPUT        :addr: register address
*OUTPUT       :register value
****************************************************************/
byte TO_ELECHOUSE_CC1120::SpiReadReg(uint16_t addr)
{

	byte value, temp;
	uint8_t tempExt = (uint8_t)(addr >> 8);
	uint8_t tempAddr = (uint8_t)(addr & 0x00FF);

	if (!tempExt)
	{
		digitalWrite(SS_PIN, LOW);
		while (digitalRead(MISO_PIN));
		temp = tempAddr | READ_SINGLE;
		SpiTransfer(temp);
		value = SpiTransfer(1);
		digitalWrite(SS_PIN, HIGH);
		return value;
	}
	else if (tempExt == 0x2F)
	{
		digitalWrite(SS_PIN, LOW);
		while (digitalRead(MISO_PIN));
		SpiTransfer(tempExt | READ_SINGLE);
		SpiTransfer(tempAddr);
		value = SpiTransfer(1);
		digitalWrite(SS_PIN, HIGH);
		return value;
	}
}

/****************************************************************
*FUNCTION NAME:SpiReadBurstReg
*FUNCTION     :CC1101 read burst data from register
*INPUT        :addr: register address; buffer:array to store register value; num: number to read
*OUTPUT       :none
****************************************************************/
void TO_ELECHOUSE_CC1120::SpiReadBurstReg(uint16_t addr, byte *buffer, byte num)
{
	byte temp, i;
	uint8_t tempExt = (uint8_t)(addr >> 8);
	uint8_t tempAddr = (uint8_t)(addr & 0x00FF);

	if (!tempExt)
	{
		digitalWrite(SS_PIN, LOW);
		while (digitalRead(MISO_PIN));
		temp = tempAddr | READ_BURST;
		SpiTransfer(temp);
		for (i = 0; i<num; i++)
		{
			buffer[i] = SpiTransfer(0);
		}
		digitalWrite(SS_PIN, HIGH);

	}
	else if (tempExt == 0x2F)
	{
		digitalWrite(SS_PIN, LOW);
		while (digitalRead(MISO_PIN));
		temp = tempAddr;
		SpiTransfer(tempExt | READ_BURST);
		SpiTransfer(temp);
		for (i = 0; i<num; i++)
		{
			buffer[i] = SpiTransfer(0);
		}
		digitalWrite(SS_PIN, HIGH);

	}
}

/****************************************************************
*FUNCTION NAME:SpiReadStatus
*FUNCTION     :CC1101 read status register
*INPUT        :addr: register address
*OUTPUT       :status value
****************************************************************/

byte TO_ELECHOUSE_CC1120::SpiReadStatus(uint16_t addr)
{
	byte value, temp;
	uint8_t tempExt = (uint8_t)(addr >> 8);
	uint8_t tempAddr = (uint8_t)(addr & 0x00FF);


	if (!tempExt)
	{
		temp = tempAddr | READ_SINGLE;
		digitalWrite(SS_PIN, LOW);
		while (digitalRead(MISO_PIN));
		SpiTransfer(temp);
		value = SpiTransfer(0);
		digitalWrite(SS_PIN, HIGH);

		return value;
	}
	else if (tempExt == 0x2F)
	{
		digitalWrite(SS_PIN, LOW);
		while (digitalRead(MISO_PIN));
		SpiTransfer(tempExt | READ_SINGLE);
		SpiTransfer(tempAddr);
		value = SpiTransfer(0);
		digitalWrite(SS_PIN, HIGH);

		return value;
	}
}

/****************************************************************
*FUNCTION NAME:RegConfigSettings
*FUNCTION     :CC1101 register config //details refer datasheet of CC1101/CC1100//
*INPUT        :Channel number
*OUTPUT       :none
****************************************************************/
void TO_ELECHOUSE_CC1120::RegConfigSettings(byte ch) 
{ 
	//
	// Rf settings for CC1120
	//
	SpiWriteReg(IOCFG3, 0x06);        //GPIO3 IO Pin Configuration
	SpiWriteReg(IOCFG2, 0x06);        //GPIO2 IO Pin Configuration
	SpiWriteReg(IOCFG1, 0xB0);        //GPIO1 IO Pin Configuration
	SpiWriteReg(IOCFG0, 0x40);        //GPIO0 IO Pin Configuration
	SpiWriteReg(SYNC_CFG1, 0x0B);     //Sync Word Detection Configuration Reg. 1
	SpiWriteReg(DEVIATION_M, 0x89);   //Frequency Deviation Configuration
	SpiWriteReg(MODCFG_DEV_E, 0x02);  //Modulation Format and Frequency Deviation Configur..
	SpiWriteReg(DCFILT_CFG, 0x1C);    //Digital DC Removal Configuration
	SpiWriteReg(PREAMBLE_CFG1, 0x18); //Preamble Length Configuration Reg. 1
	SpiWriteReg(IQIC, 0xC6);          //Digital Image Channel Compensation Configuration
	SpiWriteReg(CHAN_BW, 0x18);       //Channel Filter Configuration
	SpiWriteReg(MDMCFG0, 0x05);       //General Modem Parameter Configuration Reg. 0

	SpiWriteReg(SYMBOL_RATE2, 0x53);  //Symbol Rate Configuration Exponent and Mantissa [1..
	SpiWriteReg(AGC_REF, 0x20);       //AGC Reference Level Configuration
	SpiWriteReg(AGC_CS_THR, 0x19);    //Carrier Sense Threshold Configuration
	SpiWriteReg(AGC_CFG1, 0xA9);      //Automatic Gain Control Configuration Reg. 1
	SpiWriteReg(AGC_CFG0, 0xCF);      //Automatic Gain Control Configuration Reg. 0
	SpiWriteReg(FIFO_CFG, 0x00);      //FIFO Configuration
	SpiWriteReg(FS_CFG, 0x14);        //Frequency Synthesizer Configuration
	SpiWriteReg(PKT_CFG0, 0x20);      //Packet Configuration Reg. 0
	SpiWriteReg(PA_CFG2, 0x4F);       //Power Amplifier Configuration Reg. 2	0x74 : 10dBm, 0x6D : 7dBm, 0x69 : 6dBm, 0x5D : 0dBm, 0x56 : -3dBm, 0x4F : -6dBm, 0x43 : -11dBm
	SpiWriteReg(PKT_LEN, 0xFF);       //Packet Length Configuration
	SpiWriteReg(IF_MIX_CFG, 0x00);    //IF Mix Configuration
	SpiWriteReg(FREQOFF_CFG, 0x22);   //Frequency Offset Correction Configuration
	SpiWriteReg(FREQ2, 0x6A);         //Frequency Configuration [23:16]
	SpiWriteReg(FREQ1, 0x2D);         //Frequency Configuration [15:8]
	SpiWriteReg(FREQ0, 0x99);         //Frequency Configuration [7:0]
	SpiWriteReg(FS_DIG1, 0x00);       //Frequency Synthesizer Digital Reg. 1
	SpiWriteReg(FS_DIG0, 0x5F);       //Frequency Synthesizer Digital Reg. 0
	SpiWriteReg(FS_CAL1, 0x40);       //Frequency Synthesizer Calibration Reg. 1
	SpiWriteReg(FS_CAL0, 0x0E);       //Frequency Synthesizer Calibration Reg. 0
	SpiWriteReg(FS_DIVTWO, 0x03);     //Frequency Synthesizer Divide by 2
	SpiWriteReg(FS_DSM0, 0x33);       //FS Digital Synthesizer Module Configuration Reg. 0
	SpiWriteReg(FS_DVC0, 0x17);       //Frequency Synthesizer Divider Chain Configuration ..
	SpiWriteReg(FS_PFD, 0x50);        //Frequency Synthesizer Phase Frequency Detector Con..
	SpiWriteReg(FS_PRE, 0x6E);        //Frequency Synthesizer Prescaler Configuration
	SpiWriteReg(FS_REG_DIV_CML, 0x14);//Frequency Synthesizer Divider Regulator Configurat..
	SpiWriteReg(FS_SPARE, 0xAC);      //Frequency Synthesizer Spare
	SpiWriteReg(FS_VCO0, 0xB4);       //FS Voltage Controlled Oscillator Configuration Reg..
	SpiWriteReg(XOSC5, 0x0E);         //Crystal Oscillator Configuration Reg. 5
	SpiWriteReg(XOSC1, 0x03);         //Crystal Oscillator Configuration Reg. 1

	SpiWriteReg(PKT_CFG2, 0x10);




////SpiWriteReg(RFEND_CFG1,0x3F);

SetChannel(ch);                         // Set the channel
}

/****************************************************************
*FUNCTION NAME:SendData
*FUNCTION     :use CC1101 send data
*INPUT        :txBuffer: data array to send; size: number of data to send, no more than 61
*OUTPUT       :none
****************************************************************/
void TO_ELECHOUSE_CC1120::SendData(byte *txBuffer,byte size)
{
	//byte sts = SpiStrobe(SNOP + 0x80);
	//Serial.print("send status");
	//Serial.println(sts);
	Serial.println();
	if (size > CC1120_MAX_MESSAGE_LEN)
		return;					// Guin
	
	byte temp[40];
	byte i;

	temp[0] = e_txHeaderFrom;
	temp[1] = e_txHeaderTo;
	temp[2] = e_txHeaderSource;
	temp[3] = e_txHeaderDestination;
	temp[4] = e_txHeaderType;
	temp[5] = e_txHeaderData;
  	temp[6] = e_txHeaderFlags ;
 	temp[7] = e_txHeaderSeqNum++;
	temp[8] = e_txHeaderHop;

    for (i=CC1120_HEADER_LEN ; i<size+CC1120_HEADER_LEN ; i++) {
		temp[i] = txBuffer[i-CC1120_HEADER_LEN];
    }  
	 
	temp[size + CC1120_HEADER_LEN] = 0;
	for (i = 0; i < size + CC1120_HEADER_LEN; i++)
	{
		temp[size + CC1120_HEADER_LEN] ^= temp[i];
	}
   
	

   Serial.println("Sending : From To Src Dst Type Data Fg Seq hop Payload....                         ");
   Serial.print("          ");
   DisableLNA();




  for (i=0; i<CC1120_HEADER_LEN+size+1; i++) {
	   Serial.print(temp[i]);
	  //if ((i == 1) || (i == 3) || (i == 5) || (i == 7) || (i > 7)) 
		  Serial.print("   ");
  }        
	
	Serial.println("          ");
	SpiWriteReg(SINGLE_TXFIFO, CC1120_HEADER_LEN + size + 1 ); // Header + Payload + checksum
	SpiWriteBurstReg(BURST_TXFIFO, temp,  CC1120_HEADER_LEN + size + 1 );
	SpiStrobe(STX) ;		//start send	

	while (!digitalRead(GDO3)); //Serial.println("!digitalRead(GDO3)");    Serial.println(" Sync Word transmitted (GPIO3) ");	// Wait for GDO0 to be set -> sync transmitted  
	while (digitalRead(GDO3)); // Serial.println("digitalRead(GDO3)");     Serial.println(" End of packet (GPIO3)");	// Wait for GDO0 to be cleared -> end of packet

  
   
	delay(100);
	SpiStrobe(SFTX );									//flush TXfifo
//	Serial.println(" ******************************************************* ");	
	EnableLNA();
	//Serial.println("send end");
}

/****************************************************************
*FUNCTION NAME:SendData
*FUNCTION     :use CC1101 send data
*INPUT        :txBuffer: data array to send; size: number of data to send, no more than 61
*OUTPUT       :none
****************************************************************/
void TO_ELECHOUSE_CC1120::inSendData(byte *txBuffer, byte size)
{

	//byte sts = SpiStrobe(SNOP + 0x80);
	//Serial.print("send status");
	//Serial.println(sts);
	Serial.println();
	if (size > CC1120_MAX_MESSAGE_LEN)
		return;					// Guin

	byte temp[40];
	byte i;

	temp[0] = e_txHeaderFrom;
	temp[1] = e_txHeaderTo;
	temp[2] = e_txHeaderMaster;
	temp[3] = e_txHeaderType;

	for (i = CC1120_HEADER_LEN; i<size + CC1120_HEADER_LEN; i++) {
		temp[i] = txBuffer[i - CC1120_HEADER_LEN];
	}

	temp[size + CC1120_HEADER_LEN] = 0;
	for (i = 0; i < size + CC1120_HEADER_LEN; i++)
	{
		temp[size + CC1120_HEADER_LEN] ^= temp[i];
	}



	Serial.println("Sending : From To Master Type Payload....                         ");
	Serial.print("          ");
	DisableLNA();


	for (i = 0; i<CC1120_HEADER_LEN + size + 1; i++) {
		Serial.print(temp[i]);
		//if ((i == 1) || (i == 3) || (i == 5) || (i == 7) || (i > 7)) 
		Serial.print("   ");
	}

	Serial.println("          ");
	SpiWriteReg(SINGLE_TXFIFO, CC1120_HEADER_LEN + size + 1); // Header + Payload + checksum
	SpiWriteBurstReg(BURST_TXFIFO, temp, CC1120_HEADER_LEN + size + 1);
	SpiStrobe(STX);		//start send	


	while (!digitalRead(GDO3));//GDO3으로 해야함 Serial.println("!digitalRead(GDO3)");    //Serial.println(" Sync Word transmitted (GPIO3) ");	// Wait for GDO0 to be set -> sync transmitted  
	while (digitalRead(GDO3));//GDO3으로 해야함Serial.println("digitalRead(GDO3)");     //Serial.println(" End of packet (GPIO3)");	// Wait for GDO0 to be cleared -> end of packet



	delay(100);
	SpiStrobe(SFTX);									//flush TXfifo
														//	Serial.println(" ******************************************************* ");	
	EnableLNA();
	//Serial.println("send end");
}
void TO_ELECHOUSE_CC1120::EnableLNA(void)
{
	digitalWrite(RE_PIN, LOW);
	digitalWrite(TE_PIN, HIGH);
}

void TO_ELECHOUSE_CC1120::DisableLNA(void)
{
	digitalWrite(RE_PIN, LOW);
	digitalWrite(TE_PIN, LOW);

}
/****************************************************************
*FUNCTION NAME:SetReceive
*FUNCTION     :set CC1101 to receive state
*INPUT        :none
*OUTPUT       :none
****************************************************************/
void TO_ELECHOUSE_CC1120::SetReceive(void)
{	
	byte sts;
	byte temp;
	sts = SpiStrobe(SNOP + 0x80); // Receive status...
	temp = sts & 0x70;
	Serial.print(sts);
	Serial.print(" ");
	Serial.print(temp);
	Serial.print(" ;");

	if (sts == 0)//IDLE이면 
		SpiStrobe(SRX);
	if ((sts & 0xF) != 0)//reserved != 0000
		SpiStrobe(SFRX);
	if(temp == 0x20)
		SpiStrobe(SRX);
	if (temp == 0x60)//RX FIFO error
		SpiStrobe(SFRX);
	if (temp == 0x70)//TX FIFO error
		SpiStrobe(SFTX);
}

/****************************************************************
*FUNCTION NAME:CheckReceiveFlag
*FUNCTION     :check receive data or not
*INPUT        :none
*OUTPUT       :flag: 0 no data; 1 receive data 
****************************************************************/
byte TO_ELECHOUSE_CC1120::CheckReceiveFlag(void)
{  
    byte sts;
	byte reserved;

	sts = SpiStrobe(SNOP + 0x80) ; // Receive status...
	reserved = sts & 0x0F;
//	Serial.print("2 state: ");
//  Serial.println(sts,HEX);
	
	/*if ( reserved == 0x0F)
	{  
	    return 1;
	}*/
	
	if(digitalRead(GDO0))			//receive data
	{    	
	     while (digitalRead(GDO0)) 	;	 
	        return 1;
	}
	else							// no data
	{	
		return 0;
	}
	 
}

/****************************************************************
*FUNCTION NAME:ReceiveData
*FUNCTION     :read data received from RXfifo
*INPUT        :rxBuffer: buffer to store data
*OUTPUT       :size of data received
****************************************************************/
byte TO_ELECHOUSE_CC1120::ReceiveData(byte *rxBuffer)
{
	Serial.println();
	byte size;
	byte status[2];
	byte temp[120];
	byte i;
    signed char lqi;
    byte crc;
	byte num;
	size =SpiReadReg(NUM_RXBYTES);
	
	if (size == 0)
		return 0;

	while((num =SpiReadReg(NUM_RXBYTES)) >CC1120_HEADER_LEN)
	{
  		size=SpiReadReg(SINGLE_RXFIFO);
 		if ((size < CC1120_HEADER_LEN) || ( size > CC1120_MAX_MESSAGE_LEN) ) { //  CC1120_MAX_MESSAGE_LEN is  50
		Serial.print("************* Size ERROR : Too short or Too large ********* " );
		 	Serial.println(size);
			delay(100);
			SpiStrobe(SFRX );
		 	return 0; // Too short or Too large to be a real message
		}
		if (num != (size + 3))
		{
			Serial.println("size mismatch");
			SpiStrobe(SFRX);
			return 0;
		}
 
		SpiReadBurstReg(BURST_RXFIFO,temp,size);
	   	SpiReadBurstReg(BURST_RXFIFO,status,2);
		
	
		rssi = rssi_dbm(status[0]);
		lqi = (status[1] & 0x7F);
	    crc = (status[1] & 0x80);
	
	   
		byte checkSum = 0;
		for (i = 0; i < size - 1; i++)
		{
			checkSum ^= temp[i];
		}
		if (temp[size - 1] != checkSum)
		{
			Serial.println("************* checksum ERROR *********  ");
			delay(100);
			SpiStrobe(SFRX);
		    return 0;
		}

		rssi = Read8BitRssi();

		if (rssi < -120) {
			Serial.println("************* weak signal(rssi below threshold)*********  ");
			delay(100);
			SpiStrobe(SFRX);
			return 0;
		}

		Serial.print("-----receive-----");
		Serial.println(millis()/1000);
		for (i=0;i<size; i++) {
			Serial.print(temp[i]);
			//if ((i == 1) || (i == 3) || (i == 5) || (i == 7) || (i > 7)) 
			Serial.print("   ");
		}
	
		Serial.println();

		e_rxHeaderFrom			  =     temp[0];
		e_rxHeaderTo			  =		temp[1];
		e_rxHeaderSource		  =		temp[2];
		e_rxHeaderDestination	  =		temp[3];
		e_rxHeaderType			  = 	temp[4];
		e_rxHeaderData			  = 	temp[5];
		e_rxHeaderFlags			  = 	temp[6];
		e_rxHeaderSeqNum		  = 	temp[7];
		e_rxHeaderHop			  =		temp[8];
	
		memcpy(rxBuffer, temp   + CC1120_HEADER_LEN, size-CC1120_HEADER_LEN -1 ); // copy payload. payload size = size - header length - checksum - bytenumber
		for (i=0;i<120; i++) {
		  temp[i] =0;
		}

		delay(100);
		SpiStrobe(SFRX);
		return size;
	}

	delay(100);
	SpiStrobe(SFRX );
	return 0;
}

/****************************************************************
*FUNCTION NAME:ReceiveData
*FUNCTION     :read data received from RXfifo
*INPUT        :rxBuffer: buffer to store data
*OUTPUT       :size of data received
****************************************************************/
byte TO_ELECHOUSE_CC1120::inReceiveData(byte *rxBuffer)
{
	Serial.println();
	byte size;
	byte status[2];
	byte temp[120];
	byte i;
	signed char lqi;
	byte crc;
	byte num;
	size = SpiReadReg(NUM_RXBYTES);
	if (size == 0)
		return 0;
	while ((num = SpiReadReg(NUM_RXBYTES)) >CC1120_HEADER_LEN)
	{
		size = SpiReadReg(SINGLE_RXFIFO);
		if ((size < CC1120_HEADER_LEN) || (size > CC1120_MAX_MESSAGE_LEN)) { //  CC1120_MAX_MESSAGE_LEN is  50
			Serial.print("************* Size ERROR : Too short or Too large ********* ");
			Serial.println(size);
			delay(100);
			SpiStrobe(SFRX);
			return 0; // Too short or Too large to be a real message
		}
		if (num != (size + 3))
		{
			Serial.println("size mismatch");
			SpiStrobe(SFRX);
			return 0;
		}

		SpiReadBurstReg(BURST_RXFIFO, temp, size);
		SpiReadBurstReg(BURST_RXFIFO, status, 2);


		rssi = rssi_dbm(status[0]);
		lqi = (status[1] & 0x7F);
		crc = (status[1] & 0x80);


		byte checkSum = 0;
		for (i = 0; i < size - 1; i++)
		{
			checkSum ^= temp[i];
		}
		if (temp[size - 1] != checkSum)
		{
			Serial.println("************* checksum ERROR *********  ");
			delay(100);
			SpiStrobe(SFRX);
			return 0;
		}

		rssi = Read8BitRssi();

		if (rssi < -120) {
			Serial.println("************* weak signal(rssi below threshold)*********  ");
			delay(100);
			SpiStrobe(SFRX);
			return 0;
		}

		Serial.print("-----receive-----");
		Serial.println(millis() / 1000);
		for (i = 0; i<size; i++) {
			Serial.print(temp[i]);
			//if ((i == 1) || (i == 3) || (i == 5) || (i == 7) || (i > 7)) 
			Serial.print("   ");
		}

		Serial.println();

		e_rxHeaderFrom = temp[0];
		e_rxHeaderTo = temp[1];
		e_rxHeaderMaster = temp[2];
		e_rxHeaderType = temp[3];

		memcpy(rxBuffer, temp + CC1120_HEADER_LEN, size - CC1120_HEADER_LEN - 1); // copy payload. payload size = size - header length - checksum - bytenumber
		for (i = 0; i<120; i++) {
			temp[i] = 0;
		}

		delay(100);
		SpiStrobe(SFRX);
		return size;
	}

	delay(100);
	SpiStrobe(SFRX);
	return 0;
}

void TO_ELECHOUSE_CC1120::ManualCalibration(void) {

    byte original_fs_cal2;
    byte calResults_for_vcdac_start_high[3];
    byte calResults_for_vcdac_start_mid[3];
    byte marcstate;
    byte writeByte;
	
		
    // 1) Set VCO cap-array to 0 (FS_VCO2 = 0x00)
    writeByte = 0x00;
    SpiWriteReg(FS_VCO2, writeByte);
	
	
    // 2) Start with high VCDAC (original VCDAC_START + 2): 
    original_fs_cal2 = SpiReadReg(FS_CAL2);
	
    writeByte = original_fs_cal2 + VCDAC_START_OFFSET; //  VCDAC_START_OFFSET = 2
    SpiWriteReg(FS_CAL2, writeByte);
	
    // 3) Calibrate and wait for calibration to be done
    //   (radio back in IDLE state)
    SpiStrobe(SCAL);

    do {
        marcstate = SpiReadReg(MARCSTATE);
		////Serial.println(marcstate);
    } while (marcstate != 0x41);
	
    // 4) Read FS_VCO2, FS_VCO4 and FS_CHP register obtained with
    //    high VCDAC_START value
	calResults_for_vcdac_start_high[FS_VCO2_INDEX] = SpiReadReg(FS_VCO2);
    calResults_for_vcdac_start_high[FS_VCO4_INDEX] = SpiReadReg(FS_VCO4);
    calResults_for_vcdac_start_high[FS_CHP_INDEX] = SpiReadReg(FS_CHP);

   

    // 5) Set VCO cap-array to 0 (FS_VCO2 = 0x00)
    writeByte = 0x00;
    SpiWriteReg(FS_VCO2, writeByte);

    // 6) Continue with mid VCDAC (original VCDAC_START):
    writeByte = original_fs_cal2;
    SpiWriteReg(FS_CAL2, writeByte);
	
    // 7) Calibrate and wait for calibration to be done
    //   (radio back in IDLE state)
    SpiStrobe(SCAL);

    do {
         marcstate = SpiReadReg(MARCSTATE);
    } while (marcstate != 0x41);
	
    // 8) Read FS_VCO2, FS_VCO4 and FS_CHP register obtained
    //    with mid VCDAC_START value
    calResults_for_vcdac_start_mid[FS_VCO2_INDEX] = SpiReadReg(FS_VCO2);
    calResults_for_vcdac_start_mid[FS_VCO4_INDEX] = SpiReadReg(FS_VCO4);
	
    calResults_for_vcdac_start_mid[FS_CHP_INDEX] = SpiReadReg(FS_CHP);

    // 9) Write back highest FS_VCO2 and corresponding FS_VCO
    //    and FS_CHP result
    if (calResults_for_vcdac_start_high[FS_VCO2_INDEX] >
        calResults_for_vcdac_start_mid[FS_VCO2_INDEX]) {
        writeByte = calResults_for_vcdac_start_high[FS_VCO2_INDEX];
        SpiWriteReg(FS_VCO2, writeByte);
        writeByte = calResults_for_vcdac_start_high[FS_VCO4_INDEX];
        SpiWriteReg(FS_VCO4, writeByte);
        writeByte = calResults_for_vcdac_start_high[FS_CHP_INDEX];
        SpiWriteReg(FS_CHP, writeByte);
    } else {
        writeByte = calResults_for_vcdac_start_mid[FS_VCO2_INDEX];
        SpiWriteReg(FS_VCO2, writeByte);
        writeByte = calResults_for_vcdac_start_mid[FS_VCO4_INDEX];
        SpiWriteReg(FS_VCO4, writeByte);
        writeByte = calResults_for_vcdac_start_mid[FS_CHP_INDEX];
        SpiWriteReg(FS_CHP, writeByte);
    }
}

void TO_ELECHOUSE_CC1120::SetChannel(int ch) {

	int temp, FreQ2, FreQ1, FreQ0;
	float Base_Frequency = 424.7125;
	float frequency = Base_Frequency + (ch * 0.0125);

	float  f_vco = 8 * frequency; // LO divider * RF;
	long   Freq = f_vco * 65536 / 32;

	FreQ2 = Freq / 65536;
	temp = Freq % 65536;
	FreQ1 = temp / 256;
	FreQ0 = temp % 256;
	SpiWriteReg(FREQ2, FreQ2);         //Frequency Configuration [23:16]
	SpiWriteReg(FREQ1, FreQ1);         //Frequency Configuration [15:8]  
	SpiWriteReg(FREQ0, FreQ0);    //Frequency Configuration [7:0] 


}
void TO_ELECHOUSE_CC1120::PrintChannel() {
	int temp, FreQ2, FreQ1, FreQ0;
	float Base_Frequency = 424.7125;
	float frequency;
	for (int i = 0; i < 30; i++)
	{
		frequency = Base_Frequency + (i * 0.0125);

		float  f_vco = 8 * frequency; // LO divider * RF;
		long   Freq = f_vco * 65536 / 32;
		FreQ2 = Freq / 65536;
		temp = Freq % 65536;
		FreQ1 = temp / 256;
		FreQ0 = temp % 256;
		Serial.println(i);
		Serial.print(FreQ2, HEX);
		Serial.print(" ");
		Serial.print(FreQ1, HEX);
		Serial.print(" ");
		Serial.print(FreQ0, HEX);
		Serial.print(" ");
		Serial.println();
	}
}
int TO_ELECHOUSE_CC1120::Read8BitRssi(void){
	uint8_t rssi2compl,rssiValid;
	uint8_t rssiOffset = 102;
	uint8_t rssiConverted;
	// Read RSSI_VALID from RSSI0
	rssiValid= SpiReadReg(RSSI0);
	// Check if the RSSI_VALID flag is set
	if(rssiValid & 0x01){
	// Read RSSI from MSB register
		rssi2compl= SpiReadReg(RSSI1);
		rssiConverted = (uint8_t)rssi2compl - rssiOffset;
		return rssiConverted;
	}
	// return 0 since new value is not valid
	return 0;
}

//
TO_ELECHOUSE_CC1120 TO_ELECHOUSE_CC1120;

/****************************************************************
*FUNCTION NAME:setThisAddress
*FUNCTION     :setThisAddress
*INPUT        : 
*OUTPUT       : 
****************************************************************/

void TO_ELECHOUSE_CC1120::setThisAddress(uint8_t address)
{
   	 e_thisAddress = address;
}
void TO_ELECHOUSE_CC1120::setHeaderFrom(uint8_t from)
{
   	 e_txHeaderFrom = from;
}
void TO_ELECHOUSE_CC1120::setHeaderTo(uint8_t to)
{
   	 e_txHeaderTo = to;
}
void TO_ELECHOUSE_CC1120::setHeaderSource(uint8_t source)
{
   	 e_txHeaderSource = source;
}
void TO_ELECHOUSE_CC1120::setHeaderDestination(uint8_t destination)
{
	e_txHeaderDestination = destination;
}
void TO_ELECHOUSE_CC1120::setHeaderType(uint8_t type)
{
   	 e_txHeaderType = type;
}
void TO_ELECHOUSE_CC1120::setHeaderData(uint8_t data)
{
   	 e_txHeaderData = data;
}
void TO_ELECHOUSE_CC1120::setHeaderFlags(uint8_t flag)
{
   	 e_txHeaderFlags = flag;
}
void TO_ELECHOUSE_CC1120::setHeaderSeqNum(uint8_t seq)
{
   	 e_txHeaderSeqNum = seq;
}
void TO_ELECHOUSE_CC1120::setHeaderHop(uint8_t hop)
{
	e_txHeaderHop = hop;
}
void TO_ELECHOUSE_CC1120::setHeaderMaster(uint8_t master)
{
	e_txHeaderMaster = master;
}
 uint8_t  TO_ELECHOUSE_CC1120::headerTo( )
{
   	return  e_rxHeaderTo;
}
 uint8_t  TO_ELECHOUSE_CC1120::headerFrom( )
{
	return  e_rxHeaderFrom;
 }
 uint8_t  TO_ELECHOUSE_CC1120::headerSource( )
{ 
	return  e_rxHeaderSource;
}
 uint8_t  TO_ELECHOUSE_CC1120::headerDestination()
 {
	 return e_rxHeaderDestination;
 }
 uint8_t  TO_ELECHOUSE_CC1120::headerType( )
{
 	return  e_rxHeaderType;
  }
 uint8_t  TO_ELECHOUSE_CC1120::headerData( )
{
	return  e_rxHeaderData;
}
 uint8_t  TO_ELECHOUSE_CC1120::headerFlags(     )
{
   	return e_rxHeaderFlags ; 
}
 uint8_t  TO_ELECHOUSE_CC1120::headerSeqNum( )
{
   	return e_rxHeaderSeqNum;  
}
 uint8_t  TO_ELECHOUSE_CC1120::headerHop()
{
	return e_rxHeaderHop;
}
 uint8_t TO_ELECHOUSE_CC1120::headerMaster()
 {
	 return e_rxHeaderMaster;
 }


byte TO_ELECHOUSE_CC1120::setSyncWords(const uint8_t* syncWords, uint8_t len)
{
    if (!syncWords || len != 4)
	return 0; // Only 4 byte sync words are supported
	byte tmp;
    SpiWriteReg(SYNC0  , syncWords[0]);
	SpiWriteReg(SYNC1  , syncWords[1]);
	SpiWriteReg(SYNC2  , syncWords[2]);
	SpiWriteReg(SYNC3  , syncWords[3]);
	
	tmp=SpiReadReg(SYNC0);
	//Serial.print("Received Sync0 Word: ");
	//Serial.print (tmp,HEX);
	//Serial.println(", Sync Word0 Should be 0xDE");
}	

void TO_ELECHOUSE_CC1120::setTxPowerTo5mW( )
{
   	SpiWriteReg(PA_CFG2, 0x6B);  // 5mw
}


#define CRC16_POLY 0x8005
#define CRC_INIT 0xFFFF
#define RSSI_OFFSET 102

uint16_t culCalcCRC(uint8_t crcData, uint16_t crcReg) {
 uint8_t i;
 for (i = 0; i < 8; i++) {
 if (((crcReg & 0x8000) >> 8) ^ (crcData & 0x80))
 crcReg = (crcReg << 1) ^ CRC16_POLY;
 else
 crcReg = (crcReg << 1);
 crcData <<= 1;
 }
 return crcReg;
}// culCalcCRC

uint16_t  CRC(uint8_t *txBuffer,uint8_t size ) {
	uint16_t checksum;
	uint8_t i;
	
	checksum = CRC_INIT; // Init value for CRC calculation
	for (i = 0; i < size; i++)
	 checksum = culCalcCRC(txBuffer[i], checksum); 
 
	 return checksum;
}
static signed char rssi_dbm(unsigned char raw_rssi)
{
  int16_t dbm = 0;

  if(raw_rssi >= 128) {
    dbm = (int16_t)((int16_t)(raw_rssi - 256) / 2) - RSSI_OFFSET;
  } else {
    dbm = (raw_rssi / 2) - RSSI_OFFSET;
  }
  return dbm;
}


