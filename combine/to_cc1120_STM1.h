//2017-07-17
/*

	 CC1120.h
*/
#ifndef SMARTRF_CC1120_H
#define SMARTRF_CC1120_H

#include <SPI.h>

// Init constants

#define RX 0
      
#define CC1120_MAX_MESSAGE_LEN 50
#define BROADCAST_ADDRESS 0xFFFF
#define CC1120_HEADER_LEN 9
#define IN_CC1120_HEADER_LEN 4

#define TYPE_ACK 				0x01
#define TYPE_COMMAND 			0x02
#define TYPE_ROUTING_REQUEST 		0x03
#define TYPE_ROUTING_ACK			0x04


/***************************************************************
 *  SmartRF Studio(tm) Export
 *
 *  Radio register settings specifed with C-code
 *  compatible #define statements.
 *
 *  RF device: CC1120
 *
 ***************************************************************/

/* configuration registers */
#define IOCFG3                   0x0000
#define IOCFG2                   0x0001
#define IOCFG1                   0x0002
#define IOCFG0                   0x0003
#define SYNC3                    0x0004
#define SYNC2                    0x0005
#define SYNC1                    0x0006
#define SYNC0                    0x0007
#define SYNC_CFG1                0x0008
#define SYNC_CFG0                0x0009
#define DEVIATION_M              0x000A
#define MODCFG_DEV_E             0x000B
#define DCFILT_CFG               0x000C
#define PREAMBLE_CFG1            0x000D
#define PREAMBLE_CFG0            0x000E
#define FREQ_IF_CFG              0x000F
#define IQIC                     0x0010
#define CHAN_BW                  0x0011
#define MDMCFG1                  0x0012
#define MDMCFG0                  0x0013
#define SYMBOL_RATE2             0x0014
#define SYMBOL_RATE1             0x0015
#define SYMBOL_RATE0             0x0016
#define AGC_REF                  0x0017
#define AGC_CS_THR               0x0018
#define AGC_GAIN_ADJUST          0x0019
#define AGC_CFG3                 0x001A
#define AGC_CFG2                 0x001B
#define AGC_CFG1                 0x001C
#define AGC_CFG0                 0x001D
#define FIFO_CFG                 0x001E
#define DEV_ADDR                 0x001F
#define SETTLING_CFG             0x0020
#define FS_CFG                   0x0021
#define WOR_CFG1                 0x0022
#define WOR_CFG0                 0x0023
#define WOR_EVENT0_MSB           0x0024
#define WOR_EVENT0_LSB           0x0025
#define PKT_CFG2                 0x0026
#define PKT_CFG1                 0x0027
#define PKT_CFG0                 0x0028
#define RFEND_CFG1               0x0029
#define RFEND_CFG0               0x002A
#define PA_CFG2                  0x002B
#define PA_CFG1                  0x002C
#define PA_CFG0                  0x002D
#define PKT_LEN                  0x002E

/* Extended Configuration Registers */
#define IF_MIX_CFG               0x2F00
#define FREQOFF_CFG              0x2F01
#define TOC_CFG                  0x2F02
#define MARC_SPARE               0x2F03
#define ECG_CFG                  0x2F04
#define CFM_DATA_CFG             0x2F05
#define EXT_CTRL                 0x2F06
#define RCCAL_FINE               0x2F07
#define RCCAL_COARSE             0x2F08
#define RCCAL_OFFSET             0x2F09
#define FREQOFF1                 0x2F0A
#define FREQOFF0                 0x2F0B
#define FREQ2                    0x2F0C
#define FREQ1                    0x2F0D
#define FREQ0                    0x2F0E
#define IF_ADC2                  0x2F0F
#define IF_ADC1                  0x2F10
#define IF_ADC0                  0x2F11
#define FS_DIG1                  0x2F12
#define FS_DIG0                  0x2F13
#define FS_CAL3                  0x2F14
#define FS_CAL2                  0x2F15
#define FS_CAL1                  0x2F16
#define FS_CAL0                  0x2F17
#define FS_CHP                   0x2F18
#define FS_DIVTWO                0x2F19
#define FS_DSM1                  0x2F1A
#define FS_DSM0                  0x2F1B
#define FS_DVC1                  0x2F1C
#define FS_DVC0                  0x2F1D
#define FS_LBI                   0x2F1E
#define FS_PFD                   0x2F1F
#define FS_PRE                   0x2F20
#define FS_REG_DIV_CML           0x2F21
#define FS_SPARE                 0x2F22
#define FS_VCO4                  0x2F23
#define FS_VCO3                  0x2F24
#define FS_VCO2                  0x2F25
#define FS_VCO1                  0x2F26
#define FS_VCO0                  0x2F27
#define GBIAS6                   0x2F28
#define GBIAS5                   0x2F29
#define GBIAS4                   0x2F2A
#define GBIAS3                   0x2F2B
#define GBIAS2                   0x2F2C
#define GBIAS1                   0x2F2D
#define GBIAS0                   0x2F2E
#define IFAMP                    0x2F2F
#define LNA                      0x2F30
#define RXMIX                    0x2F31
#define XOSC5                    0x2F32
#define XOSC4                    0x2F33
#define XOSC3                    0x2F34
#define XOSC2                    0x2F35
#define XOSC1                    0x2F36
#define XOSC0                    0x2F37
#define ANALOG_SPARE             0x2F38
#define PA_CFG3                  0x2F39
#define IRQ0M                    0x2F3F
#define IRQ0F                    0x2F40 

/* Status Registers */
#define WOR_TIME1                0x2F64
#define WOR_TIME0                0x2F65
#define WOR_CAPTURE1             0x2F66
#define WOR_CAPTURE0             0x2F67
#define BIST                     0x2F68
#define DCFILTOFFSET_I1          0x2F69
#define DCFILTOFFSET_I0          0x2F6A
#define DCFILTOFFSET_Q1          0x2F6B
#define DCFILTOFFSET_Q0          0x2F6C
#define IQIE_I1                  0x2F6D
#define IQIE_I0                  0x2F6E
#define IQIE_Q1                  0x2F6F
#define IQIE_Q0                  0x2F70
#define RSSI1                    0x2F71
#define RSSI0                    0x2F72
#define MARCSTATE                0x2F73
#define LQI_VAL                  0x2F74
#define PQT_SYNC_ERR             0x2F75
#define DEM_STATUS               0x2F76
#define FREQOFF_EST1             0x2F77
#define FREQOFF_EST0             0x2F78
#define AGC_GAIN3                0x2F79
#define AGC_GAIN2                0x2F7A
#define AGC_GAIN1                0x2F7B
#define AGC_GAIN0                0x2F7C
#define CFM_RX_DATA_OUT          0x2F7D
#define CFM_TX_DATA_IN           0x2F7E
#define ASK_SOFT_RX_DATA         0x2F7F
#define RNDGEN                   0x2F80
#define MAGN2                    0x2F81
#define MAGN1                    0x2F82
#define MAGN0                    0x2F83
#define ANG1                     0x2F84
#define ANG0                     0x2F85
#define CHFILT_I2                0x2F86
#define CHFILT_I1                0x2F87
#define CHFILT_I0                0x2F88
#define CHFILT_Q2                0x2F89
#define CHFILT_Q1                0x2F8A
#define CHFILT_Q0                0x2F8B
#define GPIO_STATUS              0x2F8C
#define FSCAL_CTRL               0x2F8D
#define PHASE_ADJUST             0x2F8E
#define PARTNUMBER               0x2F8F
#define PARTVERSION              0x2F90
#define SERIAL_STATUS            0x2F91
#define MODEM_STATUS1            0x2F92
#define MODEM_STATUS0            0x2F93
#define MARC_STATUS1             0x2F94
#define MARC_STATUS0             0x2F95
#define PA_IFAMP_TEST            0x2F96
#define FSRF_TEST                0x2F97
#define PRE_TEST                 0x2F98
#define PRE_OVR                  0x2F99
#define ADC_TEST                 0x2F9A
#define DVC_TEST                 0x2F9B
#define ATEST                    0x2F9C
#define ATEST_LVDS               0x2F9D
#define ATEST_MODE               0x2F9E
#define XOSC_TEST1               0x2F9F
#define XOSC_TEST0               0x2FA0  
                                        
#define RXFIRST                  0x2FD2   
#define TXFIRST                  0x2FD3   
#define RXLAST                   0x2FD4 
#define TXLAST                   0x2FD5 
#define NUM_TXBYTES              0x2FD6  /* Number of bytes in TXFIFO */ 
#define NUM_RXBYTES              0x2FD7  /* Number of bytes in RXFIFO */
#define FIFO_NUM_TXBYTES         0x2FD8  
#define FIFO_NUM_RXBYTES         0x2FD9  

                                                                                                                                                
/* DATA FIFO Access */
#define SINGLE_TXFIFO            0x3F      /*  TXFIFO  - Single accecss to Transmit FIFO */
#define BURST_TXFIFO             0x7F      /*  TXFIFO  - Burst accecss to Transmit FIFO  */
#define SINGLE_RXFIFO            0xBF      /*  RXFIFO  - Single accecss to Receive FIFO  */
#define BURST_RXFIFO             0xFF      /*  RXFIFO  - Busrrst ccecss to Receive FIFO  */


#define LQI_CRC_OK_BM            0x80
#define LQI_EST_BM               0x7F



/* Command strobe registers */
#define SRES                     0x30      /*  SRES    - Reset chip. */
#define SFSTXON                  0x31      /*  SFSTXON - Enable and calibrate frequency synthesizer. */
#define SXOFF                    0x32      /*  SXOFF   - Turn off crystal oscillator. */
#define SCAL                     0x33      /*  SCAL    - Calibrate frequency synthesizer and turn it off. */
#define SRX                      0x34      /*  SRX     - Enable RX. Perform calibration if enabled. */
#define STX                      0x35      /*  STX     - Enable TX. If in RX state, only enable TX if CCA passes. */
#define SIDLE                    0x36      /*  SIDLE   - Exit RX / TX, turn off frequency synthesizer. */
#define SWOR                     0x38      /*  SWOR    - Start automatic RX polling sequence (Wake-on-Radio) */
#define SPWD                     0x39      /*  SPWD    - Enter power down mode when CSn goes high. */
#define SFRX                     0x3A      /*  SFRX    - Flush the RX FIFO buffer. */
#define SFTX                     0x3B      /*  SFTX    - Flush the TX FIFO buffer. */
#define SWORRST                  0x3C      /*  SWORRST - Reset real time clock. */
#define SNOP                     0x3D      /*  SNOP    - No operation. Returns status byte. */
#define AFC                      0x37      /*  AFC     - Automatic Frequency Correction */

/* Chip states returned in status byte */
#define STATE_IDLE               0x00
#define STATE_RX                 0x10
#define STATE_TX                 0x20
#define STATE_FSTXON             0x30
#define STATE_CALIBRATE          0x40
#define STATE_SETTLING           0x50
#define STATE_RXFIFO_ERROR       0x60
#define STATE_TXFIFO_ERROR       0x70

//*************************************** pins **************************************************//
#define VCDAC_START_OFFSET 2
#define FS_VCO2_INDEX 0
#define FS_VCO4_INDEX 1
#define FS_CHP_INDEX 2
//************************************* class **************************************************//
class TO_ELECHOUSE_CC1120
{
	private:
		void SpiInit(void);
		void SpiMode(byte config);
		inline byte SpiTransfer(byte value) { 
			return SPI.transfer(value);
		}
		void GDO_Set (void);
		void Reset (void);
		void SpiWriteReg(uint16_t addr, byte value);
		void SpiWriteBurstReg(byte addr, byte *buffer, byte num);
		byte SpiStrobe(byte strobe);
		byte SpiReadReg(uint16_t addr);
		void SpiReadBurstReg(uint16_t addr, byte *buffer, byte num);
		byte SpiReadStatus(uint16_t addr);
		

// ********************************************************************************
 	protected:
		uint8_t  e_thisAddress; //Added by Guin
  		uint8_t  e_rxHeaderFrom ;
		uint8_t  e_rxHeaderTo ;
  		uint8_t  e_rxHeaderSource ;
		uint8_t  e_rxHeaderDestination ;
 		uint8_t   e_rxHeaderType ;
		uint8_t   e_rxHeaderData ;
  		uint8_t   e_rxHeaderFlags ;
		uint8_t   e_rxHeaderSeqNum ;
		uint8_t   e_rxHeaderHop ;
		uint8_t  e_rxHeaderMaster;

  		uint8_t  e_txHeaderFrom ;
		uint8_t  e_txHeaderTo ;  		
		uint8_t  e_txHeaderSource ;
		uint8_t  e_txHeaderDestination ;
 		uint8_t   e_txHeaderType ;
		uint8_t   e_txHeaderData ;
		uint8_t   e_txHeaderFlags ;
		uint8_t   e_txHeaderSeqNum ;
		uint8_t   e_txHeaderHop;
		uint8_t  e_txHeaderMaster;

		uint8_t   SS_PIN;
		uint8_t   RST_PIN;   // Sean - need a reset PIN
		uint8_t	  SCK_PIN;
		uint8_t   MISO_PIN;
		uint8_t   MOSI_PIN;
		uint8_t	  RE_PIN;
		uint8_t   TE_PIN;

		uint8_t   GDO0;
		uint8_t	  GDO3;
		uint8_t   _mosi,_miso,_sclk,_cs,_gdo;

		
	public:
TO_ELECHOUSE_CC1120(uint8_t mosi,uint8_t miso, uint8_t sclk, uint8_t cs,uint8_t gdo)
		:_mosi(mosi),_miso(_miso),_sclk(sclk),_cs(cs),_gdo(gdo) {
		}	
		signed char rssi;
		TO_ELECHOUSE_CC1120();	
		
		void Init(void);
		void Init(byte f);

		void SendData(byte *txBuffer, byte size);
		void SetReceive(void);
		byte CheckReceiveFlag(void);
		byte ReceiveData(byte *rxBuffer);

		void inSendData(byte *txBuffer, byte size);
		byte inReceiveData(byte *rxBuffer);
// ********************************************************************************
		void setThisAddress(uint8_t address); //Added by Guin
		void setHeaderFrom(uint8_t address);
		void setHeaderTo(uint8_t address);
		void setHeaderSource(uint8_t address);
		void setHeaderDestination(uint8_t address);
		void setHeaderType(uint8_t type);
		void setHeaderData(uint8_t data);
		void setHeaderFlags(uint8_t seq);
		void setHeaderSeqNum(uint8_t seq);
		void setHeaderHop(uint8_t hop);
		void setHeaderMaster(uint8_t master);
        void RegConfigSettings(byte f);
		void setTxPowerTo5mW();
		int  Read8BitRssi(void);

		byte  setSyncWords(const uint8_t* syncWords, uint8_t len);
		uint8_t  headerFrom( ); //  
		uint8_t  headerTo( );  // return _rxHeaderTo  
		uint8_t  headerSource( );
		uint8_t  headerDestination( );
		uint8_t  headerType( );
		uint8_t  headerData( );
		uint8_t  headerFlags( );
		uint8_t  headerSeqNum( );
		uint8_t  headerHop( );
		uint8_t  headerMaster();
		void ManualCalibration();
		void SetChannel(int ch);
		void PrintChannel();
		void DisableLNA();
		void EnableLNA();
		typedef enum
  		  {
			TransmitPowerM30dBm = 0,      ///< -30dBm
			TransmitPowerM20dBm,          ///< -20dBm
			TransmitPowerM15dBm,          ///< -15dBm
			TransmitPowerM10dBm,          ///< -10dBm
			TransmitPower0dBm,            ///< 0dBm
			TransmitPower5dBm,            ///< 5dBm
			TransmitPower7dBm,            ///< 7dBm
			TransmitPower10dBm,           ///< 10dBm
    		} TransmitPower;

};
static signed char rssi_dbm(unsigned char raw_rssi); 
uint16_t culCalcCRC(uint8_t crcData, uint16_t  crcReg);
uint16_t  CRC(uint8_t *txBuffer,uint8_t size );
//extern  TO_ELECHOUSE_CC1120 TO_ELECHOUSE_CC1120;


#endif
