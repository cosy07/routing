#include <InDatagram_STM.h>

#define SSerialRX        PA3  //Serial Receive pin DI
#define SSerialTX        PA2  //Serial Transmit pin RO

#define SSerialTxControl PA1   //RS485 Direction control
#define RS485Transmit    HIGH
#define RS485Receive     LOW

IN_ELECHOUSE_CC1120 cc1120;
InDatagram manager(cc1120, 0x01, 0x03);

uint8_t master_number;

uint8_t _thisAddress;
uint8_t _rxHeaderFrom;
uint8_t _rxHeaderTo;
uint8_t _rxHeaderMaster;
uint8_t _rxHeaderType;

byte inputData[10];
byte outputData[10];
//byte tempData[10] = {0xD5, 0x00, 0x02, 0x00, 0x03, 0x03, 0x19, 0x1E, 0x00, 0xD0};

bool scanning = false;
unsigned long rs485_loop_time;
unsigned long scanTime;
byte scanningAddress;
  
void RS485_Write_Read()
{
  uint8_t num = 0;
  
  /*for(int i = 0;i < 10;i++)
  {
    Serial.print(inputData[i], HEX);
    Serial.print(" ");
  }
  Serial.println();*/
  
  digitalWrite(SSerialTxControl, RS485Transmit);
  Serial1.write(inputData, sizeof(inputData));
  Serial1.flush();
  digitalWrite(SSerialTxControl, RS485Receive);

  rs485_loop_time = millis();
  while(rs485_loop_time + 3000 > millis())
  {
    if(Serial1.available())
      outputData[num++] = Serial1.read();
    if(num == 10)
      break;
  }
  Serial1.flush();
  for(int i = 0;i < 10;i++)
  {
    Serial.print(outputData[i], HEX);
    Serial.print(" ");
  }
  Serial.println();
  Serial.print("num : ");
  Serial.println(num);
}

void setup() 
{
  Serial.begin(9600);
  pinMode(SSerialTxControl, OUTPUT);  
  digitalWrite(SSerialTxControl, RS485Receive);  // Init Transceiver
  Serial1.begin(9600);   // set the data rate 
  manager.init(15);
  manager.SetReceive();

 _thisAddress = manager.getThisAddress();
 master_number = manager.getMasterNumber();

 //nvic_irq_set_priority(NVIC_USART2, 14);
 //nvic_irq_set_priority(NVIC_USART1, 14);
}


void loop() 
{
  manager.SetReceive();
  if(manager.available())
  {
    if(manager.recvData(manager.temp_buf))
    {
      Serial.println(millis());
      _rxHeaderFrom = manager.headerFrom();
      _rxHeaderTo = manager.headerTo();
      _rxHeaderMaster = manager.headerMaster();
      _rxHeaderType = manager.headerType();

      if(_rxHeaderMaster == master_number)
      {
        if(_rxHeaderType == SCAN_RESPONSE_TO_RC)
        {
          scanningAddress = _rxHeaderFrom;
          scanTime = millis();
          if(_rxHeaderFrom == _thisAddress - 1)
          {
            for(int i = 0;i < 10;i++)
              inputData[i] = manager.temp_buf[i];
   
            inputData[2] = _thisAddress - 1;
            inputData[9] = 0x00;
            for(int i = 0;i < 9;i++)
              inputData[9] ^= inputData[i];
              
            RS485_Write_Read();
            
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
            
            RS485_Write_Read();

        }
      }
    }
  }
}
