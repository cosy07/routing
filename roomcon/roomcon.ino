#include <InDatagram_STM.h>

#define SSerialRX        PA3  //Serial Receive pin DI
#define SSerialTX        PA2  //Serial Transmit pin RO

#define SSerialTxControl PA1   //RS485 Direction control
#define RS485Transmit    HIGH
#define RS485Receive     LOW

IN_ELECHOUSE_CC1120 cc1120;
InDatagram manager(cc1120, 0x01, 0x00);

byte receiveFromR[10];
byte receiveFromM[10];
byte roomcon_control[10];
bool rc_control = false;

byte slave_answer[16][10];
byte roomcon_request[16][10];
unsigned long rs485_time[16];

uint8_t master_number;
uint8_t num_of_slave;

uint8_t _thisAddress;
uint8_t _rxHeaderFrom;
uint8_t _rxHeaderTo;
uint8_t _rxHeaderMaster;
uint8_t _rxHeaderType;

uint8_t indexFromR = 0;
uint8_t id;

void setup()
{
  Serial.begin(9600);
  pinMode(SSerialTxControl, OUTPUT);  
  digitalWrite(SSerialTxControl, RS485Receive);  // Init Transceiver
  Serial1.begin(9600);   // set the data rate 
  
  manager.init(20);
  manager.SetReceive();
  _thisAddress = manager.getThisAddress();

  for(int i = 0;i < 16;i++)
  {
    slave_answer[i][0] = 0xB5;
    slave_answer[i][1] = 0;
    slave_answer[i][2] = i;
    slave_answer[i][3] = 0;
    slave_answer[i][4] = 3;
    slave_answer[i][5] = 3;
    slave_answer[i][6] = 0x1A;
    slave_answer[i][7] = 0x21;
    slave_answer[i][8] = 0;
    slave_answer[i][9] = 0;
    for(int j = 0;j < 9;j++)
      slave_answer[i][9] ^= slave_answer[i][j];
  }

  
  nvic_irq_set_priority(NVIC_USART2, 14);
  nvic_irq_set_priority(NVIC_USART1, 14);
  
  attachInterrupt(PA3, receiveFromRC, FALLING);

  master_number = manager.getMasterNumber();
}

void receiveFromRC()
{
  detachInterrupt(PA3);
  while(1)
  {
    if (Serial1.available()) 
    {
      if(indexFromR == 0)
      {
        byte temp = Serial1.read();
        if(temp == 0xD5 || temp == 0xC5)
        {
          receiveFromR[indexFromR++] = temp;
        }
      }
      else
        receiveFromR[indexFromR++] = Serial1.read();
    }
    if(indexFromR == 10)
    {
      for(int i = 0;i < 10;i++)
      {
        Serial.print(receiveFromR[i], HEX);
        Serial.print(" ");
      }
      Serial.println();
      
      id = receiveFromR[2];
      if(id > num_of_slave)
        num_of_slave = id;
      
      digitalWrite(SSerialTxControl, RS485Transmit);
      Serial1.write(slave_answer[id], sizeof(slave_answer[id]));
      Serial1.flush();
      digitalWrite(SSerialTxControl, RS485Receive);

      byte temp = 0;
      for(int i = 0;i < 9;i++)
        temp ^= receiveFromR[i];
        
      if(temp == receiveFromR[9])
      {
        if(receiveFromR[0] == 0xD5)
        {
          for(int i = 0;i < 10;i++)
          {
            roomcon_request[id][i] = receiveFromR[i];
          }
        }
        else if(receiveFromR[0] = 0xC5)//제어요청
        {
          for(int i = 0;i < 10;i++)
          {
            roomcon_control[i] = receiveFromR[i];
          }
          rc_control = true;
        }
      } 
      indexFromR = 0;
      rs485_time[id] = millis();
      break; 
    }
  }
  attachInterrupt(PA3, receiveFromRC, FALLING);
  nvic_irq_set_priority(NVIC_USART2, 14);
  nvic_irq_set_priority(NVIC_USART1, 14);
}

void loop()
{
  //manager.send(_thisAddress, 1, master_number, SCAN_REQUEST_TO_SLAVE, manager.temp_buf, sizeof(manager.temp_buf));
  Serial.println(millis());
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
        if(_rxHeaderType == SCAN_REQUEST_TO_RC)
        {
          manager.send(_thisAddress, _rxHeaderFrom, master_number, ACK, manager.temp_buf, sizeof(manager.temp_buf));  
          for(int j = 0;j < 10;j++)
            manager.temp_buf[j] = roomcon_request[0][j];
          manager.temp_buf[10] = num_of_slave;
          manager.send(_thisAddress, 1, master_number, SCAN_REQUEST_TO_SLAVE, manager.temp_buf, sizeof(manager.temp_buf));
        }
        else if(_rxHeaderType == SCAN_RESPONSE_TO_RC)
        {
          byte temp = 0;
          for(int i = 0;i < 9;i++)
            temp ^= manager.temp_buf[i];
          if(manager.temp_buf[9] == temp)
          {
            for(int i = 0;i < 10;i++)
              slave_answer[_rxHeaderFrom - 1][i] = manager.temp_buf[i];
          }
        }
        else if(_rxHeaderType == CONTROL_TO_RC)
        {
          for(int i = 0;i < 10;i++)
            slave_answer[0][i] = manager.temp_buf[i];
          unsigned long temp_time = millis();
          manager.send(_thisAddress, _rxHeaderFrom, master_number, ACK, manager.temp_buf, sizeof(manager.temp_buf));
          while(temp_time > rs485_time[0]);   

          for(int i = 0;i < 10;i++)
            manager.temp_buf[i] = roomcon_control[i]; 
          manager.sendToWaitAck(_thisAddress, 0xFF, master_number, GW_CONTROL_TO_SLAVE, manager.temp_buf, sizeof(manager.temp_buf));
          rc_control = false;
        }
      }
    }
  }
  if(rc_control)
  {
    delay(2000);
    for(int i = 0;i < 10;i++)
      manager.temp_buf[i] = roomcon_control[i]; 
    manager.sendToWaitAck(_thisAddress, 0xFF, master_number, RC_CONTROL_TO_SLAVE, manager.temp_buf, sizeof(manager.temp_buf));
    rc_control = false;
  }
}
