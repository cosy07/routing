#include <InDatagram_STM.h>

#define SSerialRX        PA3  //Serial Receive pin DI
#define SSerialTX        PA2  //Serial Transmit pin RO

#define SSerialTxControl PA1   //RS485 Direction control
#define RS485Transmit    HIGH
#define RS485Receive     LOW

IN_ELECHOUSE_CC1120 cc1120;
InDatagram manager(cc1120, 0x01, 0x00);

byte receiveFromR[10];
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

unsigned long controllingTime;
unsigned long scanningTime;
bool check_slave_scan_response[16];

void receiveFromRC();

// scan요청을 보내고 응답을 받음
void send_and_wait_for_scan();

void setup()
{
  Serial.begin(9600);
  pinMode(SSerialTxControl, OUTPUT);  
  digitalWrite(SSerialTxControl, RS485Receive);  // Init Transceiver
  Serial1.begin(9600);   // set the data rate 
  
  manager.init(15);
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

        //스캔 요청하라는 명령을 수신
        if(_rxHeaderType == SCAN_REQUEST_TO_RC)
        {
          //ACK 전송
          manager.send(_thisAddress, _rxHeaderFrom, master_number, ACK, manager.temp_buf, sizeof(manager.temp_buf));

          //어떤 slave로부터 응답을 받았는지 check하기 위한 배열 초기화
          for(int j = 0;j <= num_of_slave;j++)
            check_slave_scan_response[j] = false;

          //무선통신모듈 버퍼에 보낼 데이터(에어텍 내부통신 프로토콜 상태 요청 메시지) 저장
          for(int j = 0;j < 10;j++)
            manager.temp_buf[j] = roomcon_request[0][j];

          //slave개수도 보내줌
          manager.temp_buf[10] = num_of_slave;

          //스캔 메시지 송수신
          send_and_wait_for_scan()

          //스캔응답을 보내지 않은 slave가 있다면 한 번 더 전체에 대하여 scan 요청
          for(int j = 0;j <= num_of_slave;j++)
          {
            if(!check_slave_scan_response[j])
            {
             for(int j = 0;j < 10;j++)
                manager.temp_buf[j] = roomcon_request[0][j];
                
              manager.temp_buf[10] = num_of_slave;
              send_and_wait_for_scan();
              break;
            }
          }

          //내부 마스터에게 스캔이 끝났음을 알림
          manager.sendToWaitAck(_thisAddress, 0x01, master_number, SCAN_FINISH_TO_MASTER, manager.temp_buf, sizeof(manager.temp_buf));              
        }


        
        // 게이트 웨이가 제어 요청을 보냄
        // (내부 마스터) -----CONTROL_TO_RC-----> (룸콘) -----GW_CONTROL_TO_SLAVE-----> (슬레이브, 내부 마스터)
        //                                                                                        (내부 마스터) -----CONTROL_RESPONSE_BROADCAST-----> (룸콘, 슬레이브)  
        //                                                                                                            룸콘에게는 ACK의 역할
        //
        
        else if(_rxHeaderType == CONTROL_TO_RC)
        {
          for(int i = 0;i < 10;i++)
            slave_answer[0][i] = manager.temp_buf[i];
          unsigned long temp_time = millis();

          //ACK 전송
          manager.send(_thisAddress, _rxHeaderFrom, master_number, ACK, manager.temp_buf, sizeof(manager.temp_buf));

          // 내부 마스터에게 받은 상태대로 제어 요청을 보내기 위해서 실제 룸콘으로부터 제어 요청 데이터를 받기를 대기
          // (내부 마스터의 통신 노드) -----CONTROL_TO_RC(에어텍 통신 프로토콜의 헤더 : 0xA5)-----> (룸콘의 통신 노드) -----에어텍 통신 프로토콜의 헤더 : 0xA5------> (룸콘) -----에어텍 통신 프로토콜의 헤더 : 0xC5-----> (룸콘의 통신 노드)
          
          while(temp_time > rs485_time[0]);  //rs485_time[x] : 실제 룸콘이 x번째 slave에게 보내야할 데이터를 룸콘의 통신 노드에게 전송한 시간을 저장

          for(int i = 0;i < 10;i++)
            manager.temp_buf[i] = roomcon_control[i]; 

          //broadcast로 slave들에게 제어요청 메시지 전송
          manager.sendToWaitAck(_thisAddress, 0xFF, master_number, GW_CONTROL_TO_SLAVE, manager.temp_buf, sizeof(manager.temp_buf));

          rc_control = false;
        }
      }
    }
  }
  //게이트웨이로부터의 상태요청 및 제어요청 동안, 사용자가 직접 룸콘을 조작할 때의 제어요청 메시지는 미뤄짐

  //사용자가 룸콘을 조작
  if(rc_control && millis() - controllingTime > 5000)
  {
    if(!manager.available())
    {
      for(int i = 0;i < 10;i++)
        manager.temp_buf[i] = roomcon_control[i];
      manager.sendToWaitAck(_thisAddress, 0xFF, master_number, RC_CONTROL_TO_SLAVE, manager.temp_buf, sizeof(manager.temp_buf), 70);
      rc_control = false;
    }
  }
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
          controllingTime = millis();
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

void send_and_wait_for_scan()
{
  manager.send(_thisAddress, 1, master_number, SCAN_REQUEST_TO_SLAVE, manager.temp_buf, sizeof(manager.temp_buf));
  scanningTime = millis();

  //7초동안 slave의 응답 메시지를 대기
  while(millis() - scanningTime < 7000)
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
        
        if(_rxHeaderMaster == master_number && _rxHeaderType == SCAN_RESPONSE_TO_RC)
        {
          byte temp = 0;

          //checksum계산
          for(int i = 0;i < 9;i++)
            temp ^= manager.temp_buf[i];

          //checksum이 맞을 경우에 스캔 응답 데이터를 저장
          if(manager.temp_buf[9] == temp)
          {
            for(int i = 0;i < 10;i++)
              slave_answer[_rxHeaderFrom - 1][i] = manager.temp_buf[i];
          }
          check_slave_scan_response[_rxHeaderFrom] = true;
        }
      } // end of if(manager.recvData(manager.temp_buf))
    } // end of if(manager.available())
  } // end of while(millis() - scanningTime < 7000)
}

