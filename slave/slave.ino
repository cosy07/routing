#include <InDatagram_STM.h>

#define SSerialRX        PA3  //Serial Receive pin DI
#define SSerialTX        PA2  //Serial Transmit pin RO

#define SSerialTxControl PA1   //RS485 Direction control
#define RS485Transmit    HIGH
#define RS485Receive     LOW

IN_ELECHOUSE_CC1120 cc1120;
InDatagram manager(cc1120, 0x01, 0x02);

uint8_t master_number;

uint8_t _thisAddress;
uint8_t _rxHeaderFrom;
uint8_t _rxHeaderTo;
uint8_t _rxHeaderMaster;
uint8_t _rxHeaderType;

byte inputData[10];
byte outputData[10];

bool scanning = false;
  
void RS485_Write_Read(uint8_t *write_buf,uint8_t *read_buf)
{
  uint8_t num, temp;

  for(int i = 0;i < 10;i++)
  {
    Serial.print(write_buf[i], HEX);
    Serial.print(" ");
  }
  Serial.println();

  
  digitalWrite(SSerialTxControl, RS485Transmit);
  Serial1.write(write_buf, sizeof(write_buf));
  Serial1.flush();
  digitalWrite(SSerialTxControl, RS485Receive);

  num = 0;
  /*while(1)
  {
    if(Serial1.available())
    {
      Serial.println("hi");
      read_buf[num++] = Serial1.read();
      Serial.println(read_buf[num - 1]);
      if(num == 10)
        break;
    }
  }*/    
}

void setup() 
{
  Serial.begin(9600);
  pinMode(SSerialTxControl, OUTPUT);  
  digitalWrite(SSerialTxControl, RS485Receive);  // Init Transceiver
  Serial1.begin(9600);   // set the data rate 
  manager.init(20);
  manager.SetReceive();

 _thisAddress = manager.getThisAddress();
 master_number = manager.getMasterNumber();
}


void loop() 
{
  manager.SetReceive();
  if(manager.available())
  {
    if(manager.recvData(manager.temp_buf))
    {
      _rxHeaderFrom = manager.headerFrom();
      _rxHeaderTo = manager.headerTo();
      _rxHeaderMaster = manager.headerMaster();
      _rxHeaderType = manager.headerType();

      if(_rxHeaderMaster == master_number)
      {
        Serial.println("###########################################");
        if(_rxHeaderType == SCAN_RESPONSE_TO_RC)
        {
          if(_rxHeaderFrom == _thisAddress - 1)
          {
            for(int i = 0;i < 10;i++)
              inputData[i] = manager.temp_buf[i];
            inputData[2] = _thisAddress - 1;
            inputData[9] = 0x00;
            for(int i = 0;i < 9;i++)
              inputData[9] ^= inputData[i];
              
            RS485_Write_Read(inputData, outputData);

            
            for(int i = 0;i < 10;i++)
              manager.temp_buf[i] = outputData[i];
            
            manager.send(_thisAddress, 0, master_number, SCAN_RESPONSE_TO_RC, manager.temp_buf, sizeof(manager.temp_buf)); 
          }
        }
        else if(_rxHeaderType == GW_CONTROL_TO_SLAVE || _rxHeaderType == RC_CONTROL_TO_SLAVE || _rxHeaderType == CONTROL_RESPONSE_BROADCAST)
        {
            for(int i = 0;i < 10;i++)
              inputData[i] = manager.temp_buf[i];

            
            inputData[2] = _thisAddress - 1;
            inputData[9] = 0x00;
            for(int i = 0;i < 9;i++)
              inputData[9] ^= inputData[i];

            
            RS485_Write_Read(inputData, outputData);

        }
      }
    }
  }
}
