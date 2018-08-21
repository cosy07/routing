#include <InDatagram_STM.h>

#define SSerialRX        PA3  //Serial Receive pin DI
#define SSerialTX        PA2  //Serial Transmit pin RO

#define SSerialTxControl PA1   //RS485 Direction control
#define RS485Transmit    HIGH
#define RS485Receive     LOW

IN_ELECHOUSE_CC1120 cc1120;
InDatagram manager(cc1120, 0x01, 0x01);

uint8_t     master_number;
uint8_t     num_of_slave;

uint8_t _thisAddress;
uint8_t _rxHeaderFrom;
uint8_t _rxHeaderTo;
uint8_t _rxHeaderMaster;
uint8_t _rxHeaderType;

byte inputData[10];
byte outputData[10];
byte state_request[10] {0xD5, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};

uint8_t slave_num;
unsigned long slave_receive_time;

bool scanning = false;
unsigned long time = millis();
unsigned long controlTime = millis();
unsigned long scanTime = millis();
bool timeout = true;
  
void RS485_Write_Read()
{
  uint8_t num = 0;
  
  digitalWrite(SSerialTxControl, RS485Transmit);
  Serial1.write(inputData, sizeof(inputData));
  Serial1.flush();
  digitalWrite(SSerialTxControl, RS485Receive);

  time = millis();
  while(time + 3000 > millis())
  {
    if(Serial1.available())
    {
      outputData[num++] = Serial1.read();
      if(num == 10)
        break;
    }
  }
  Serial1.flush();   
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

 state_request[1] = master_number;
 for(int i = 0;i < 9;i++)
  state_request[9] ^= state_request[i];
}


void loop() 
{
  //if()//외부 마스터로부터 시리얼을 받음
  /*{
    //if()//control
    {
      for(int i = 0;i < 10;i++)
        inputData[i] = state_request[i];
      RS485_Write_Read();
      for(int i = 0; i < 10;i++)
        manager.temp_buf[i] = outputData[i];
      bool receiveACK = manager.sendToWaitAck(_thisAddress, 0x00, master_number, CONTROL_TO_RC, manager.temp_buf, sizeof(manager.temp_buf));
      if(!receiveACK)
      {
        //외부 마스터에게 시리얼 보냄(INTERNAL_COM_FAIL)
      }
      controlTime = millis();
      timeout = true;
      while(millis() - controlTime < 3000)
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
             if(_rxHeaderType == GW_CONTROL_TO_SLAVE)
              {
                timeout = false;
                manager.send(_thisAddress, 0xFF, master_number, CONTROL_RESPONSE_BROADCAST, manager.temp_buf, sizeof(manager.temp_buf));
                //control 끝났다고 외부마스터에게 시리얼통신
              }
              else if(_rxHeaderType == RC_CONTROL_TO_SLAVE)
              {
                for(int i = 0;i < 10;i++)
                  inputData[i] = manager.temp_buf[i];
                inputData[2] = 0x00;
                inputData[9] = 0x00;
                for(int i = 0;i < 9;i++)
                  inputData[9] ^= inputData[i];
                  
                manager.send(_thisAddress, 0xFF, master_number, CONTROL_RESPONSE_BROADCAST, manager.temp_buf, sizeof(manager.temp_buf));
                RS485_Write_Read();
              }
            }
          }
        }
      }
      if(timeout)
      {
        //외부 마스터에게 시리얼 보냄(INTERNAL_COM_FAIL)
      }
    }
    //else if()//scan
    {
      bool receiveACK = manager.sendToWaitAck(_thisAddress, 0x00, master_number, SCAN_REQUEST_TO_RC, manager.temp_buf, sizeof(manager.temp_buf));
      scanning = true;
      if(!receiveACK)
      {
        //외부 마스터에게 시리얼 보냄(INTERNAL_COM_FAIL)
      }
      controlTime = millis();
      timeout = true;
      while(millis() - controlTime < 10000)
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
             if(_rxHeaderType == GW_CONTROL_TO_SLAVE)
              {
                timeout = false;
                manager.send(_thisAddress, 0xFF, master_number, CONTROL_RESPONSE_BROADCAST, manager.temp_buf, sizeof(manager.temp_buf));
                //control 끝났다고 외부마스터에게 시리얼통신
              }
              else if(_rxHeaderType == SCAN_REQUEST_TO_SLAVE && _rxHeaderTo == _thisAddress)
              {
                for(int i = 0;i < 10;i++)
                  inputData[i] = manager.temp_buf[i];
      
                num_of_slave = manager.temp_buf[10];
                RS485_Write_Read();
                
                for(int i = 0;i < 10;i++)
                  manager.temp_buf[i] = outputData[i];
                
                manager.send(_thisAddress, 0, master_number, SCAN_RESPONSE_TO_RC, manager.temp_buf, sizeof(manager.temp_buf));
              }
              else if(_rxHeaderType == SCAN_RESPONSE_TO_RC)
              {
                slave_num = _rxHeaderFrom - 1;
                slave_receive_time = millis();
                if(num_of_slave == _rxHeaderFrom)
                {
                  //scan끝났다고 외부마스터에게 시리얼통신
                  scanning = false;
                }
              }
            }
          }
        }
      }
      if(timeout)
      {
        //외부 마스터에게 시리얼 보냄(INTERNAL_COM_FAIL)
      }
    }
  }*/
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
        if(_rxHeaderType == RC_CONTROL_TO_SLAVE)
        {
          for(int i = 0;i < 10;i++)
            inputData[i] = manager.temp_buf[i];
          inputData[2] = 0x00;
          inputData[9] = 0x00;
          for(int i = 0;i < 9;i++)
            inputData[9] ^= inputData[i];
            
          manager.send(_thisAddress, 0xFF, master_number, CONTROL_RESPONSE_BROADCAST, manager.temp_buf, sizeof(manager.temp_buf));
          RS485_Write_Read();
        }
      }
    }
  }
}
