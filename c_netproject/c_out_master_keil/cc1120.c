//2017-07-17 
/*                     CC1120.cpp
*/

#include "string.h"
#include "cc1120.h"

/****************************************************************/
#define 	WRITE_BURST     				0x40						//write burst
#define 	WRITE_SINGLE     				0x00						//write burst
#define 	READ_SINGLE     				0x80						//read single
#define 	READ_BURST      				0xC0						//read burst
#define 	uint8_tS_IN_RXFIFO     	0x7F  						//uint8_t number in RXfifo

/****************************************************************/

/****************************************************************
*FUNCTION NAME:SpiInit
*FUNCTION     :spi communication initialization
*INPUT        :none
*OUTPUT       :none
****************************************************************/
void e_SpiInit(void)
{
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB | RCC_APB2Periph_AFIO, ENABLE);

	SPIx_Init();

	GPIO_InitTypeDef GPIO_InitStruct;
	//	GDO0 = PB0; // GPIO0 and GPIO3	//GDO0 : input
	//	RST_PIN = PB1;								//RST : output	
	GPIO_InitStruct.GPIO_Pin = RST_PIN | TE_PIN | RE_PIN;
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOB, &GPIO_InitStruct);

	delay(100);
	printf("SS_PIN ");	printf("%d", SS_PIN);
	printf("	GDO0 ");	printf("%d\r\n", GDO0);

}

/****************************************************************
*FUNCTION NAME: GDO_Set()
*FUNCTION     : set GDO0,GDO2 pin
*INPUT        : none
*OUTPUT       : none
****************************************************************/
void e_GDO_Set(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;

	GPIO_InitStructure.GPIO_Pin = GDO0 | GDO3;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	GPIO_Init(GPIOB, &GPIO_InitStructure);

	GPIO_SetBits(GPIOB, GDO0);
	GPIO_SetBits(GPIOB, GDO3);
}

/****************************************************************
*FUNCTION NAME:Reset
*FUNCTION     :CC1101 reset //details refer datasheet of CC1101/CC1100//
*INPUT        :none
*OUTPUT       :none
****************************************************************/
void e_Reset(void)
{
	GPIO_ResetBits(GPIOB, RST_PIN);  //low
	delay(100);

	GPIO_SetBits(GPIOB, RST_PIN);
	delay(100);

	GPIO_ResetBits(GPIOA, SS_PIN);
	while (GPIO_ReadInputDataBit(GPIOA, MISO_PIN)) printf("MISO 1\r\n");

	SPIx_Transfer(CC112X_SRES);
	while (GPIO_ReadInputDataBit(GPIOA, MISO_PIN)) printf("2\r\n");
	GPIO_SetBits(GPIOA, SS_PIN);
}


/****************************************************************
*FUNCTION NAME:Init
*FUNCTION     :CC1101 initialization
*INPUT        :none
*OUTPUT       :none
****************************************************************/
/*void e_Init(void)
{
	e_thisAddress = 0;
	e_txHeaderFrom = 0xAA;
	e_txHeaderTo = 0xBB;
	e_txHeaderSource = 0xCC;
	e_txHeaderDestination = 0xDD;
	e_txHeaderType = 0xEE;
	e_txHeaderData = 0xDD;
	e_txHeaderFlags = 0xFF;
	e_txHeaderSeqNum = 0x00;

	PrintfInit();
	MyTimerInit();
	e_SpiInit();					//spi initialization
	e_GDO_Set();					//GDO set

	GPIO_SetBits(GPIOA, SS_PIN);
	GPIO_SetBits(GPIOA, SCK_PIN);
	GPIO_ResetBits(GPIOA, MOSI_PIN);
	e_EnableLNA();
	e_Reset();
	e_RegConfigSettings(0);
	//ManualCalibration();

}*/

/****************************************************************
*FUNCTION NAME:Init
*FUNCTION     :CC1101 initialization
*INPUT        :none
*OUTPUT       :none
****************************************************************/
void e_Init(uint8_t f)
{
	e_thisAddress = 0;
	e_txHeaderFrom = 0xAA;
	e_txHeaderTo = 0xBB;
	e_txHeaderSource = 0xCC;
	e_txHeaderDestination = 0xDD;
	e_txHeaderType = 0xEE;
	e_txHeaderData = 0xDD;
	e_txHeaderFlags = 0xFF;
	e_txHeaderSeqNum = 0x00;

	e_SpiInit();					//spi initialization
	e_GDO_Set();					//GDO set

	GPIO_SetBits(GPIOA, SS_PIN);
	GPIO_SetBits(GPIOA, SCK_PIN);
	GPIO_ResetBits(GPIOA, MOSI_PIN);
	e_EnableLNA();
	e_Reset();
	e_RegConfigSettings(f);
	//ManualCalibration();

}


/****************************************************************
*FUNCTION NAME:SpiWriteReg
*FUNCTION     :CC1101 write data to register
*INPUT        :addr: register address; value: register value
*OUTPUT       :none
****************************************************************/
void e_SpiWriteReg(uint16_t addr, uint8_t value)
{
	uint8_t tempExt = (uint8_t)(addr >> 8);
	uint8_t tempAddr = (uint8_t)(addr & 0x00FF);

	if (!tempExt)
	{		/* Pull CS_N low and wait for SO to go low before communication starts */
		GPIO_ResetBits(GPIOA, SS_PIN);
		delay(100);
		while (GPIO_ReadInputDataBit(GPIOA, MISO_PIN));
		SPIx_Transfer(tempAddr);
		SPIx_Transfer(value);
		GPIO_SetBits(GPIOA, SS_PIN);

	}
	else if (tempExt == 0x2F)
	{
		GPIO_ResetBits(GPIOA, SS_PIN);
		while (GPIO_ReadInputDataBit(GPIOA, MISO_PIN));
		SPIx_Transfer(tempExt);
		SPIx_Transfer(tempAddr);
		SPIx_Transfer(value);
		GPIO_SetBits(GPIOA, SS_PIN);
	}
}



/****************************************************************
*FUNCTION NAME:SpiWriteBurstReg
*FUNCTION     :CC1101 write burst data to register
*INPUT        :addr: register address; buffer:register value array; num:number to write
*OUTPUT       :none
****************************************************************/
void e_SpiWriteBurstReg(uint8_t addr, uint8_t *buffer, uint8_t num)
{
	uint8_t i, temp;


	temp = addr | WRITE_BURST;
	GPIO_ResetBits(GPIOA, SS_PIN);
	while (GPIO_ReadInputDataBit(GPIOA, MISO_PIN));
	SPIx_Transfer(temp);
	for (i = 0; i < num; i++)
	{
		SPIx_Transfer(buffer[i]);
	}

	GPIO_SetBits(GPIOA, SS_PIN);
}

/****************************************************************
*FUNCTION NAME:SpiStrobe
*FUNCTION     :CC1101 Strobe
*INPUT        :strobe: command; //refer define in CC1101.h//
*OUTPUT       :none
****************************************************************/
uint8_t e_SpiStrobe(uint8_t strobe)
{
	uint8_t sts;
	GPIO_ResetBits(GPIOA, SS_PIN);
	while (GPIO_ReadInputDataBit(GPIOA, MISO_PIN));
	sts = SPIx_Transfer(strobe);
	GPIO_SetBits(GPIOA, SS_PIN);
	return sts;
}

/****************************************************************
*FUNCTION NAME:SpiReadReg
*FUNCTION     :CC1101 read data from register
*INPUT        :addr: register address
*OUTPUT       :register value
****************************************************************/
uint8_t e_SpiReadReg(uint16_t addr)
{

	uint8_t value, temp;
	uint8_t tempExt = (uint8_t)(addr >> 8);
	uint8_t tempAddr = (uint8_t)(addr & 0x00FF);

	if (!tempExt)
	{
		GPIO_ResetBits(GPIOA, SS_PIN);
		while (GPIO_ReadInputDataBit(GPIOA, MISO_PIN));
		temp = tempAddr | READ_SINGLE;
		SPIx_Transfer(temp);
		value = SPIx_Transfer(1);
		GPIO_SetBits(GPIOA, SS_PIN);
		return value;
	}
	else if (tempExt == 0x2F)
	{
		GPIO_ResetBits(GPIOA, SS_PIN);
		while (GPIO_ReadInputDataBit(GPIOA, MISO_PIN));
		SPIx_Transfer(tempExt | READ_SINGLE);
		SPIx_Transfer(tempAddr);
		value = SPIx_Transfer(1);
		GPIO_SetBits(GPIOA, SS_PIN);
		return value;
	}
}

/****************************************************************
*FUNCTION NAME:SpiReadBurstReg
*FUNCTION     :CC1101 read burst data from register
*INPUT        :addr: register address; buffer:array to store register value; num: number to read
*OUTPUT       :none
****************************************************************/
void e_SpiReadBurstReg(uint16_t addr, uint8_t *buffer, uint8_t num)
{
	uint8_t temp, i;
	uint8_t tempExt = (uint8_t)(addr >> 8);
	uint8_t tempAddr = (uint8_t)(addr & 0x00FF);

	if (!tempExt)
	{
		GPIO_ResetBits(GPIOA, SS_PIN);
		while (GPIO_ReadInputDataBit(GPIOA, MISO_PIN));
		temp = tempAddr | READ_BURST;
		SPIx_Transfer(temp);
		for (i = 0; i<num; i++)
		{
			buffer[i] = SPIx_Transfer(0);
		}
		GPIO_SetBits(GPIOA, SS_PIN);

	}
	else if (tempExt == 0x2F)
	{
		GPIO_ResetBits(GPIOA, SS_PIN);
		while (GPIO_ReadInputDataBit(GPIOA, MISO_PIN));
		temp = tempAddr;
		SPIx_Transfer(tempExt | READ_BURST);
		SPIx_Transfer(temp);
		for (i = 0; i<num; i++)
		{
			buffer[i] = SPIx_Transfer(0);
		}
		GPIO_SetBits(GPIOA, SS_PIN);
	}
}

/****************************************************************
*FUNCTION NAME:SpiReadStatus
*FUNCTION     :CC1101 read status register
*INPUT        :addr: register address
*OUTPUT       :status value
****************************************************************/

uint8_t e_SpiReadStatus(uint16_t addr)
{
	uint8_t value, temp;
	uint8_t tempExt = (uint8_t)(addr >> 8);
	uint8_t tempAddr = (uint8_t)(addr & 0x00FF);


	if (!tempExt)
	{
		temp = tempAddr | READ_SINGLE;
		GPIO_ResetBits(GPIOA, SS_PIN);
		while (GPIO_ReadInputDataBit(GPIOA, MISO_PIN));
		SPIx_Transfer(temp);
		value = SPIx_Transfer(0);
		GPIO_SetBits(GPIOA, SS_PIN);

		return value;
	}
	else if (tempExt == 0x2F)
	{
		GPIO_ResetBits(GPIOA, SS_PIN);
		while (GPIO_ReadInputDataBit(GPIOA, MISO_PIN));
		SPIx_Transfer(tempExt | READ_SINGLE);
		SPIx_Transfer(tempAddr);
		value = SPIx_Transfer(0);
		GPIO_SetBits(GPIOA, SS_PIN);

		return value;
	}
}

/****************************************************************
*FUNCTION NAME:RegConfigSettings
*FUNCTION     :CC1101 register config //details refer datasheet of CC1101/CC1100//
*INPUT        :Channel number
*OUTPUT       :none
****************************************************************/
void e_RegConfigSettings(uint8_t ch)
{

	//
	// Rf settings for CC1120
	//
	e_SpiWriteReg(CC112X_IOCFG3, 0x06);        //GPIO3 IO Pin Configuration
	e_SpiWriteReg(CC112X_IOCFG2, 0x06);        //GPIO2 IO Pin Configuration
	e_SpiWriteReg(CC112X_IOCFG1, 0xB0);        //GPIO1 IO Pin Configuration
	e_SpiWriteReg(CC112X_IOCFG0, 0x40);        //GPIO0 IO Pin Configuration
	e_SpiWriteReg(CC112X_SYNC_CFG1, 0x0B);     //Sync Word Detection Configuration Reg. 1
	e_SpiWriteReg(CC112X_DEVIATION_M, 0x89);   //Frequency Deviation Configuration
	e_SpiWriteReg(CC112X_MODCFG_DEV_E, 0x02);  //Modulation Format and Frequency Deviation Configur..
	e_SpiWriteReg(CC112X_DCFILT_CFG, 0x1C);    //Digital DC Removal Configuration
	e_SpiWriteReg(CC112X_PREAMBLE_CFG1, 0x18); //Preamble Length Configuration Reg. 1
	e_SpiWriteReg(CC112X_IQIC, 0xC6);          //Digital Image Channel Compensation Configuration
	e_SpiWriteReg(CC112X_CHAN_BW, 0x18);       //Channel Filter Configuration
	e_SpiWriteReg(CC112X_MDMCFG0, 0x05);       //General Modem Parameter Configuration Reg. 0

	e_SpiWriteReg(CC112X_SYMBOL_RATE2, 0x53);  //Symbol Rate Configuration Exponent and Mantissa [1..
	e_SpiWriteReg(CC112X_AGC_REF, 0x20);       //AGC Reference Level Configuration
	e_SpiWriteReg(CC112X_AGC_CS_THR, 0x19);    //Carrier Sense Threshold Configuration
	e_SpiWriteReg(CC112X_AGC_CFG1, 0xA9);      //Automatic Gain Control Configuration Reg. 1
	e_SpiWriteReg(CC112X_AGC_CFG0, 0xCF);      //Automatic Gain Control Configuration Reg. 0
	e_SpiWriteReg(CC112X_FIFO_CFG, 0x00);      //FIFO Configuration
	e_SpiWriteReg(CC112X_FS_CFG, 0x14);        //Frequency Synthesizer Configuration
	e_SpiWriteReg(CC112X_PKT_CFG0, 0x20);      //Packet Configuration Reg. 0
	e_SpiWriteReg(CC112X_PA_CFG2, 0x4F);       //Power Amplifier Configuration Reg. 2	0x74 : 10dBm, 0x6D : 7dBm, 0x69 : 6dBm, 0x5D : 0dBm, 0x56 : -3dBm, 0x4F : -6dBm, 0x43 : -11dBm
	e_SpiWriteReg(CC112X_PKT_LEN, 0xFF);       //Packet Length Configuration
	e_SpiWriteReg(CC112X_IF_MIX_CFG, 0x00);    //IF Mix Configuration
	e_SpiWriteReg(CC112X_FREQOFF_CFG, 0x22);   //Frequency Offset Correction Configuration
	e_SpiWriteReg(CC112X_FREQ2, 0x6A);         //Frequency Configuration [23:16]
	e_SpiWriteReg(CC112X_FREQ1, 0x2D);         //Frequency Configuration [15:8]
	e_SpiWriteReg(CC112X_FREQ0, 0x99);         //Frequency Configuration [7:0]
	e_SpiWriteReg(CC112X_FS_DIG1, 0x00);       //Frequency Synthesizer Digital Reg. 1
	e_SpiWriteReg(CC112X_FS_DIG0, 0x5F);       //Frequency Synthesizer Digital Reg. 0
	e_SpiWriteReg(CC112X_FS_CAL1, 0x40);       //Frequency Synthesizer Calibration Reg. 1
	e_SpiWriteReg(CC112X_FS_CAL0, 0x0E);       //Frequency Synthesizer Calibration Reg. 0
	e_SpiWriteReg(CC112X_FS_DIVTWO, 0x03);     //Frequency Synthesizer Divide by 2
	e_SpiWriteReg(CC112X_FS_DSM0, 0x33);       //FS Digital Synthesizer Module Configuration Reg. 0
	e_SpiWriteReg(CC112X_FS_DVC0, 0x17);       //Frequency Synthesizer Divider Chain Configuration ..
	e_SpiWriteReg(CC112X_FS_PFD, 0x50);        //Frequency Synthesizer Phase Frequency Detector Con..
	e_SpiWriteReg(CC112X_FS_PRE, 0x6E);        //Frequency Synthesizer Prescaler Configuration
	e_SpiWriteReg(CC112X_FS_REG_DIV_CML, 0x14);//Frequency Synthesizer Divider Regulator Configurat..
	e_SpiWriteReg(CC112X_FS_SPARE, 0xAC);      //Frequency Synthesizer Spare
	e_SpiWriteReg(CC112X_FS_VCO0, 0xB4);       //FS Voltage Controlled Oscillator Configuration Reg..
	e_SpiWriteReg(CC112X_XOSC5, 0x0E);         //Crystal Oscillator Configuration Reg. 5
	e_SpiWriteReg(CC112X_XOSC1, 0x03);         //Crystal Oscillator Configuration Reg. 1

	e_SpiWriteReg(CC112X_PKT_CFG2, 0x10);

	e_SetChannel(ch);                         // Set the channel
}

/****************************************************************
*FUNCTION NAME:SendData
*FUNCTION     :use CC1101 send data
*INPUT        :txBuffer: data array to send; size: number of data to send, no more than 61
*OUTPUT       :none
****************************************************************/
void e_SendData(uint8_t *txBuffer, uint8_t size)
{
	printf("\r\n");
	if (size > CC1120_MAX_MESSAGE_LEN)
		return;					// Guin

	uint8_t temp[40];
	uint8_t i;

	temp[0] = e_txHeaderFrom;
	temp[1] = e_txHeaderTo;
	temp[2] = e_txHeaderSource;
	temp[3] = e_txHeaderDestination;
	temp[4] = e_txHeaderType;
	temp[5] = e_txHeaderData;
	temp[6] = e_txHeaderFlags;
	temp[7] = e_txHeaderSeqNum++;
	temp[8] = e_txHeaderHop;

	for (i = CC1120_HEADER_LEN; i<size + CC1120_HEADER_LEN; i++)
	{
		temp[i] = txBuffer[i - CC1120_HEADER_LEN];
	}

	temp[size + CC1120_HEADER_LEN] = 0;
	for (i = 0; i < size + CC1120_HEADER_LEN; i++)
	{
		temp[size + CC1120_HEADER_LEN] ^= temp[i];
	}

	printf("Sending : From   To    Source  Dest Type Data Flag Seq#  Payload.... \r\n");
	printf("          ");
	e_DisableLNA();

	for (i = 0; i<CC1120_HEADER_LEN + size + 1; i++) {
		printf("%d", temp[i]);
		//if (( i == 1) || ( i == 3) || ( i== 5) || ( i == 7) || ( i > 7))	
		printf("   ");
	}

	printf("          \r\n");
	e_SpiWriteReg(CC112X_SINGLE_TXFIFO, CC1120_HEADER_LEN + size + 1); // Header + Payload + checksum
	e_SpiWriteBurstReg(CC112X_BURST_TXFIFO, temp, CC1120_HEADER_LEN + size + 1);
	e_SpiStrobe(CC112X_STX);		//start send	

	while (!GPIO_ReadInputDataBit(GPIOB, GDO0));    //printf(" Sync Word transmitted (GPIO2) \r\n");	// Wait for GDO0 to be set -> sync transmitted  
	while (GPIO_ReadInputDataBit(GPIOB, GDO0));     //printf(" End of packet (GPIO2) \r\n");	// Wait for GDO0 to be cleared -> end of packet

	delay(100);

	e_SpiStrobe(CC112X_SFTX);									//flush TXfifo
															//printf(" ******************************************************* \r\n");	
	e_EnableLNA();
}

void e_EnableLNA()
{
	GPIO_ResetBits(GPIOB, RE_PIN);
	GPIO_SetBits(GPIOB, TE_PIN);
}

void e_DisableLNA()
{
	GPIO_ResetBits(GPIOB, RE_PIN);
	GPIO_ResetBits(GPIOB, TE_PIN);
}
/****************************************************************
*FUNCTION NAME:SetReceive
*FUNCTION     :set CC1101 to receive state
*INPUT        :none
*OUTPUT       :none
****************************************************************/
void e_SetReceive(void)
{
	uint8_t sts;
	uint8_t temp;
	sts = e_SpiStrobe(CC112X_SNOP + 0x80);
	temp = sts & 0x70;

	if (sts == 0)//IDLE
		e_SpiStrobe(CC112X_SRX);
	if ((sts & 0xF) != 0)//reserved != 0x00
		e_SpiStrobe(CC112X_SFRX);
	if (temp == 0x20)
		e_SpiStrobe(CC112X_SRX);
	if (temp == 0x60)//RX FIFO error
		e_SpiStrobe(CC112X_SFRX);
	if (temp == 0x70)//TX FIFO error
		e_SpiStrobe(CC112X_SFTX);
}

/****************************************************************
*FUNCTION NAME:CheckReceiveFlag
*FUNCTION     :check receive data or not
*INPUT        :none
*OUTPUT       :flag: 0 no data; 1 receive data
****************************************************************/
uint8_t e_CheckReceiveFlag(void)
{
	uint8_t sts;
	uint8_t reserved;

	sts = e_SpiStrobe(CC112X_SNOP + 0x80); // Receve status...
	reserved = sts & 0x0F;
	//	//Serial.print("2 state: ");
	//   //Serial.println(sts,HEX);

	/*if ( reserved == 0x0F)
	{
	return 1;
	}*/

	if (GPIO_ReadInputDataBit(GPIOB, GDO0))			//receive data
	{
		while (GPIO_ReadInputDataBit(GPIOB, GDO0));
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
uint8_t e_ReceiveData(uint8_t *rxBuffer)
{
	printf("\r\n");
	uint8_t size;
	uint8_t status[2];
	uint8_t temp[120];
	uint8_t i;
	signed char lqi;
	uint8_t crc;
	uint8_t num;
	if ((size = e_SpiReadReg(CC112X_NUM_RXBYTES)) == 0)
		return 0;

	while ((num = e_SpiReadReg(CC112X_NUM_RXBYTES)) > CC1120_HEADER_LEN)
	{
		size = e_SpiReadReg(CC112X_SINGLE_RXFIFO);
		if ((size < CC1120_HEADER_LEN) || (size > CC1120_MAX_MESSAGE_LEN))
		{
			printf("************* Size ERROR : Too short or Too large ********* ");
			printf("%d\r\n", size);
			delay(100);
			e_SpiStrobe(CC112X_SFRX);
			return 0;
		}
		if (num != (size + 3))
		{
			printf("size mismatch\r\n");
			e_SpiStrobe(CC112X_SFRX);
			return 0;
		}

		e_SpiReadBurstReg(CC112X_BURST_RXFIFO, temp, size);
		e_SpiReadBurstReg(CC112X_BURST_RXFIFO, status, 2);

		e_rssi = rssi_dbm(status[0]);
		lqi = (status[1] & 0x7F);
		crc = (status[1] & 0x80);

		uint8_t checkSum = 0;
		for (i = 0; i < size - 1; i++)
		{
			checkSum ^= temp[i];
		}
		if (temp[size - 1] != checkSum)
		{
			printf("*********************checksum ERROR****************************\r\n");
			delay(100);
			e_SpiStrobe(CC112X_SFRX);
			return 0;
		}

		e_rssi = e_Read8BitRssi();
		if (e_rssi < -120)
		{
			printf("************* weak signal(rssi below threshold)*********\r\n");
			delay(100);
			e_SpiStrobe(CC112X_SFRX);
			return 0;
		}


		printf("-----receive-----");
		printf("%d\r\n", millis() / 1000);
		for (i = 0; i<size; i++) {
			printf("%d", temp[i]);
			//if ((i == 1) || (i == 3) || (i == 5) || ( i == 7) ||   (i > 7)) 
			printf("   ");
		}

		printf("\r\n");

		// Extract the 12 uint8_ts headers
		e_rxHeaderFrom = temp[0];//(temp[0]<<8) | temp[1];  //word(temp[0],temp[1]);
		e_rxHeaderTo = temp[1];//(temp[2]<<8) | temp[3];
		e_rxHeaderSource = temp[2];//(temp[4]<<8) | temp[5];
		e_rxHeaderDestination = temp[3];//(temp[6]<<8) | temp[7];
		e_rxHeaderType = temp[4];//temp[8];
		e_rxHeaderData = temp[5];
		e_rxHeaderFlags = temp[6];
		e_rxHeaderSeqNum = temp[7];
		e_rxHeaderHop = temp[8];

		memcpy(rxBuffer, temp + CC1120_HEADER_LEN, size - CC1120_HEADER_LEN - 1); // copy payload. payload size = size - header length - checksum - uint8_tnumber
		for (i = 0; i<120; i++) {
			temp[i] = 0;
		}
		delay(100);
		e_SpiStrobe(CC112X_SFRX);
		return size;
	}
	delay(100);
	e_SpiStrobe(CC112X_SFRX);
	return 0;
}

void e_ManualCalibration(void) {

	uint8_t original_fs_cal2;
	uint8_t calResults_for_vcdac_start_high[3];
	uint8_t calResults_for_vcdac_start_mid[3];
	uint8_t marcstate;
	uint8_t writeuint8_t;


	// 1) Set VCO cap-array to 0 (FS_VCO2 = 0x00)
	writeuint8_t = 0x00;
	e_SpiWriteReg(CC112X_FS_VCO2, writeuint8_t);


	// 2) Start with high VCDAC (original VCDAC_START + 2): 
	original_fs_cal2 = e_SpiReadReg(CC112X_FS_CAL2);

	writeuint8_t = original_fs_cal2 + VCDAC_START_OFFSET; //  VCDAC_START_OFFSET = 2
	e_SpiWriteReg(CC112X_FS_CAL2, writeuint8_t);

	// 3) Calibrate and wait for calibration to be done
	//   (radio back in IDLE state)
	e_SpiStrobe(CC112X_SCAL);

	do {
		marcstate = e_SpiReadReg(CC112X_MARCSTATE);
		//Serial.println(marcstate);
	} while (marcstate != 0x41);

	// 4) Read FS_VCO2, FS_VCO4 and FS_CHP register obtained with
	//    high VCDAC_START value
	calResults_for_vcdac_start_high[FS_VCO2_INDEX] = e_SpiReadReg(CC112X_FS_VCO2);
	calResults_for_vcdac_start_high[FS_VCO4_INDEX] = e_SpiReadReg(CC112X_FS_VCO4);
	calResults_for_vcdac_start_high[FS_CHP_INDEX] = e_SpiReadReg(CC112X_FS_CHP);



	// 5) Set VCO cap-array to 0 (FS_VCO2 = 0x00)
	writeuint8_t = 0x00;
	e_SpiWriteReg(CC112X_FS_VCO2, writeuint8_t);

	// 6) Continue with mid VCDAC (original VCDAC_START):
	writeuint8_t = original_fs_cal2;
	e_SpiWriteReg(CC112X_FS_CAL2, writeuint8_t);

	// 7) Calibrate and wait for calibration to be done
	//   (radio back in IDLE state)
	e_SpiStrobe(CC112X_SCAL);

	do {
		marcstate = e_SpiReadReg(CC112X_MARCSTATE);
	} while (marcstate != 0x41);

	// 8) Read FS_VCO2, FS_VCO4 and FS_CHP register obtained
	//    with mid VCDAC_START value
	calResults_for_vcdac_start_mid[FS_VCO2_INDEX] = e_SpiReadReg(CC112X_FS_VCO2);
	calResults_for_vcdac_start_mid[FS_VCO4_INDEX] = e_SpiReadReg(CC112X_FS_VCO4);

	calResults_for_vcdac_start_mid[FS_CHP_INDEX] = e_SpiReadReg(CC112X_FS_CHP);

	// 9) Write back highest FS_VCO2 and corresponding FS_VCO
	//    and FS_CHP result
	if (calResults_for_vcdac_start_high[FS_VCO2_INDEX] >
		calResults_for_vcdac_start_mid[FS_VCO2_INDEX]) {
		writeuint8_t = calResults_for_vcdac_start_high[FS_VCO2_INDEX];
		e_SpiWriteReg(CC112X_FS_VCO2, writeuint8_t);
		writeuint8_t = calResults_for_vcdac_start_high[FS_VCO4_INDEX];
		e_SpiWriteReg(CC112X_FS_VCO4, writeuint8_t);
		writeuint8_t = calResults_for_vcdac_start_high[FS_CHP_INDEX];
		e_SpiWriteReg(CC112X_FS_CHP, writeuint8_t);
	}
	else {
		writeuint8_t = calResults_for_vcdac_start_mid[FS_VCO2_INDEX];
		e_SpiWriteReg(CC112X_FS_VCO2, writeuint8_t);
		writeuint8_t = calResults_for_vcdac_start_mid[FS_VCO4_INDEX];
		e_SpiWriteReg(CC112X_FS_VCO4, writeuint8_t);
		writeuint8_t = calResults_for_vcdac_start_mid[FS_CHP_INDEX];
		e_SpiWriteReg(CC112X_FS_CHP, writeuint8_t);
	}
}

void e_SetChannel(int ch) {

	int temp, FreQ2, FreQ1, FreQ0;
	float Base_Frequency = 424.7125;
	float frequency = Base_Frequency + (ch * 0.0125);

	float  f_vco = 8 * frequency; // LO divider * RF;
	long   Freq = f_vco * 65536 / 32;

	FreQ2 = Freq / 65536;
	temp = Freq % 65536;
	FreQ1 = temp / 256;
	FreQ0 = temp % 256;
	e_SpiWriteReg(CC112X_FREQ2, FreQ2);         //Frequency Configuration [23:16]
	e_SpiWriteReg(CC112X_FREQ1, FreQ1);         //Frequency Configuration [15:8]  
	e_SpiWriteReg(CC112X_FREQ0, FreQ0);    //Frequency Configuration [7:0] 
}

void e_PrintChannel() {
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
		printf("%d\r\n", i);
		printf("%x %x %x \r\n", FreQ2, FreQ1, FreQ0);
	}
}
int e_Read8BitRssi(void) {
	uint8_t rssi2compl, rssiValid;
	uint8_t rssiOffset = 102;
	uint8_t rssiConverted;
	// Read RSSI_VALID from RSSI0
	rssiValid = e_SpiReadReg(CC112X_RSSI0);
	// Check if the RSSI_VALID flag is set
	if (rssiValid & 0x01) {
		// Read RSSI from MSB register
		rssi2compl = e_SpiReadReg(CC112X_RSSI1);
		rssiConverted = (uint8_t)rssi2compl - rssiOffset;
		return rssiConverted;
	}
	// return 0 since new value is not valid
	return 0;
}

//
//ELECHOUSE_CC1120 ELECHOUSE_cc1120;

/****************************************************************
*FUNCTION NAME:setThisAddress
*FUNCTION     :setThisAddress
*INPUT        :
*OUTPUT       :
****************************************************************/
void e_setThisAddress(uint8_t address)
{
	e_thisAddress = address;
}
void e_setHeaderFrom(uint8_t from)
{
	e_txHeaderFrom = from;
}
void e_setHeaderTo(uint8_t to)
{
	e_txHeaderTo = to;
}
void e_setHeaderSource(uint8_t source)
{
	e_txHeaderSource = source;
}
void e_setHeaderDestination(uint8_t destination)
{
	e_txHeaderDestination = destination;
}
void e_setHeaderType(uint8_t type)
{
	e_txHeaderType = type;
}
void e_setHeaderData(uint8_t data)
{
	e_txHeaderData = data;
}
void e_setHeaderFlags(uint8_t flag)
{
	e_txHeaderFlags = flag;
}
void e_setHeaderSeqNum(uint8_t seq)
{
	e_txHeaderSeqNum = seq;
}
void e_setHeaderHop(uint8_t hop)
{
	e_txHeaderHop = hop;
}
uint8_t  e_headerTo()
{
	return  e_rxHeaderTo;

}
uint8_t  e_headerFrom()
{
	return  e_rxHeaderFrom;
}
uint8_t  e_headerSource()
{
	return  e_rxHeaderSource;
}
uint8_t  e_headerDestination()
{
	return e_rxHeaderDestination;
}
uint8_t  e_headerType()
{
	return  e_rxHeaderType;
}
uint8_t  e_headerData()
{
	return  e_rxHeaderData;
}
uint8_t  e_headerFlags()
{
	return e_rxHeaderFlags;
}
uint8_t  e_headerSeqNum()
{
	return e_rxHeaderSeqNum;
}
uint8_t  e_headerHop()
{
	return e_rxHeaderHop;
}


uint8_t e_setSyncWords(const uint8_t* syncWords, uint8_t len)
{
	if (!syncWords || len != 4)
		return 0; // Only 4 uint8_t sync words are supported
	uint8_t tmp;
	e_SpiWriteReg(CC112X_SYNC0, syncWords[0]);
	e_SpiWriteReg(CC112X_SYNC1, syncWords[1]);
	e_SpiWriteReg(CC112X_SYNC2, syncWords[2]);
	e_SpiWriteReg(CC112X_SYNC3, syncWords[3]);

	tmp = e_SpiReadReg(CC112X_SYNC0);
	printf("Received Sync0 Word: ");
	printf("%x", tmp);
	printf(", Sync Word0 Should be 0xDE\r\n");
}

void e_setTxPowerTo5mW()
{
	e_SpiWriteReg(CC112X_PA_CFG2, 0x6B);  // 5mw
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

uint16_t CRCC(uint8_t *txBuffer, uint8_t size) {
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

	if (raw_rssi >= 128) {
		dbm = (int16_t)((int16_t)(raw_rssi - 256) / 2) - RSSI_OFFSET;
	}
	else {
		dbm = (raw_rssi / 2) - RSSI_OFFSET;
	}
	return dbm;
}
