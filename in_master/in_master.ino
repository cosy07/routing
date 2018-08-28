//내부 마스터
//1. 외부마스터와의 통신(게이트웨이로부터의 내부 zone 스캔 요청, 게이트웨이로부터의 내부 zone 제어 요청)
//2. 룸콘 및 슬레이브와의 통신

#include <InDatagram_STM.h>

#define SSerialRX        PA3  //Serial Receive pin DI
#define SSerialTX        PA2  //Serial Transmit pin RO

#define SSerialTxControl PA1   //RS485 Direction control
#define RS485Transmit    HIGH
#define RS485Receive     LOW

IN_ELECHOUSE_CC1120 cc1120;
InDatagram manager(cc1120, 0x01, 0x01);

uint8_t     master_number; // 내가 속한 zone의 외부 master 넘버
uint8_t     num_of_slave; // 내가 속한 zone의 slave 개수

uint8_t _thisAddress;
uint8_t _rxHeaderFrom;
uint8_t _rxHeaderTo;
uint8_t _rxHeaderMaster;
uint8_t _rxHeaderType;

byte inputData[10]; // RS485 송신 버퍼
byte outputData[10]; // RS485 수신 버퍼
byte state_request[10] {0xD5, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00}; //에어텍 내부 통신 프로토콜 상태 요청 메시지

unsigned long rs485_time = millis();
unsigned long controlTime;
unsigned long scanTime;
bool timeout = true;

byte receivedFromOutMaster;

// RS485 통신 함수
// inputData에 있는 데이터를 전송하여 outputData에 받은 데이터를 저장한다.
// 에어텍 내부 통신 프로토콜에서 주고받는 데이터는 10 Bytes
void RS485_Write_Read()
{
  uint8_t num = 0;
  
  digitalWrite(SSerialTxControl, RS485Transmit);
  Serial1.write(inputData, sizeof(inputData));
  Serial1.flush();
  digitalWrite(SSerialTxControl, RS485Receive);

  rs485_time = millis();
  while(rs485_time + 3000 > millis())
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
  //외부 마스터로부터 시리얼을 받음
  if(Serial2.available())
  {
    receivedFromOutMaster = Serial2.read();
    
    //  제어요청일 경우
    // **************** 1단계 ******************
    //  (GW) -----> (외부 마스터) -----> (FCU)
    //                            -----> (내부 마스터)
    //  GW의 제어요청에 맞게 FCU 상태 변화 및 내부마스터에게 알려줌


    // **************** 2단계 ******************
    //  (내부 마스터) -----상태 요청-----> (FCU) -----상태 응답-----> (내부 마스터)
    //  내부 마스터가 FCU에게 현재 상태를 물어봄


    // **************** 3단계 ******************
    // 자신의 상태를 룸콘에게 전송하여 그 zone이 GW의 제어요청대로 변경되도록 함
    // (내부 마스터) -----CONTROL_TO_RC-----> (룸콘) -----GW_CONTROL_TO_SLAVE-----> (슬레이브, 내부 마스터)
    //                                                                                        (내부 마스터) -----CONTROL_RESPONSE_BROADCAST-----> (룸콘, 슬레이브)  
    //                                                                                                            룸콘에게는 ACK의 역할
    //                                                                                                            다른 슬레이브에게는 RC_CONTROL_TO_SLAVE를 못받을경우에 대비
    
    if(receivedFromOutMaster == INTERNAL_CONTROL)
    {
      for(int i = 0;i < 10;i++)
        inputData[i] = state_request[i];

      //FCU에게 현재 상태를 요청
      RS485_Write_Read();

      //현재 상태를 룸콘에게 전송하되, 상태응답이 아닌 변경요청(0xA5)로 보냄
      outputData[0] = 0xA5;
      outputData[9] = 0;

      //checksum 계산 및 전송 버퍼에 데이터 저장
      for(int i = 0; i < 9;i++)
      {
        outputData[9] ^= outputData[i];
        manager.temp_buf[i] = outputData[i];
      }
      manager.temp_buf[9] = outputData[9];

      //룸콘에게 전송
      bool receiveACK = manager.sendToWaitAck(_thisAddress, 0x00, master_number, CONTROL_TO_RC, manager.temp_buf, sizeof(manager.temp_buf));

      //룸콘으로부터 ACK을 받지 못함
      if(!receiveACK)
      {
        Serial2.write(INTERNAL_COM_FAIL);
      }
      else
      {
        controlTime = millis();
        timeout = true;
       
        // 룸콘이 slave에게 제어 요청을 보내길 기다림        
        while(millis() - controlTime < 3000)
        {

          //receive 상태로 전환
          manager.SetReceive();

          //패킷 수신
          if(manager.available())
          {

            //받은 데이터를 저장
            if(manager.recvData(manager.temp_buf))
            {

              //헤더 저장
              _rxHeaderFrom = manager.headerFrom();
              _rxHeaderTo = manager.headerTo();
              _rxHeaderMaster = manager.headerMaster();
              _rxHeaderType = manager.headerType();

              //내가 속한 zone으로부터의 패킷인 경우
              if(_rxHeaderMaster == master_number)
              {
                //룸콘의 제어 요청 수신
               if(_rxHeaderType == GW_CONTROL_TO_SLAVE)
                {
                  timeout = false;
                  manager.send(_thisAddress, 0xFF, master_number, CONTROL_RESPONSE_BROADCAST, manager.temp_buf, sizeof(manager.temp_buf));

                  //제어가 끝났음을 외부 마스터에게 알림
                  Serial2.write(INTERNAL_CONTROL_FINISH);
                }
              }
            }
          }
        }
        
        //타임아웃 발생
        if(timeout)
        {
          //외부마스터에게 에러상황을 알림
          Serial2.write(INTERNAL_COM_FAIL);
        } 
      }
    }





    //  스캔요청일 경우
    //                                                                                              실제 스캔 요청                                   스캔 응답
    //  (GW) -----> (외부 마스터) -----> (내부 마스터) -----SCAN_REQUEST_TO_RC-----> (룸콘) -----SCAN_REQUEST_TO_SLAVE-----> (내부 마스터) -----SCAN_RESPONSE_TO_RC-----> (룸콘)
    //                                                                                                                                                                 (슬레이브 1) -----SCAN_RESPONSE_TO_RC-----> (룸콘)
    //                                                                                                                                                                                                           (슬레이브 2) -----SCAN_RESPONSE_TO_RC-----> (룸콘)
    
    else if(receivedFromOutMaster == INTERNAL_SCAN)
    {
      //룸콘에게 전송
      bool receiveACK = manager.sendToWaitAck(_thisAddress, 0x00, master_number, SCAN_REQUEST_TO_RC, manager.temp_buf, sizeof(manager.temp_buf));

      //룸콘으로부터 ACK을 받지 못함
      if(!receiveACK)
      {
        //외부 마스터에게 시리얼 보냄
        Serial2.write(INTERNAL_COM_FAIL);
      }

      //전체 slave가 다 scan될 때까지 or 일정시간까지 반복문
      
      scanTime = millis();
      timeout = true;
      while(millis() - scanTime < 7000)
      {

        //receive상태로 변환
        manager.SetReceive();

        //패킷 수신
        if(manager.available())
        {

          //수신된 데이터를 자신의 버퍼에 저장
          if(manager.recvData(manager.temp_buf))
          {

            //read header
            _rxHeaderFrom = manager.headerFrom();
            _rxHeaderTo = manager.headerTo();
            _rxHeaderMaster = manager.headerMaster();
            _rxHeaderType = manager.headerType();

            //내가 속한 zone으로부터의 패킷인 경우
            if(_rxHeaderMaster == master_number)
            {

              //룸콘으로부터 스캔요청을 수신
              if(_rxHeaderType == SCAN_REQUEST_TO_SLAVE && _rxHeaderTo == _thisAddress)
              {
                for(int i = 0;i < 10;i++)
                  inputData[i] = manager.temp_buf[i];
                
                num_of_slave = manager.temp_buf[10];
                RS485_Write_Read();
                //FCU에게 현태 상태를 물어봄
                
                for(int i = 0;i < 10;i++)
                  manager.temp_buf[i] = outputData[i];

                //룸콘에게 스캔 응답
                manager.send(_thisAddress, 0, master_number, SCAN_RESPONSE_TO_RC, manager.temp_buf, sizeof(manager.temp_buf));
              }

              //룸콘으로부터 SCAN_FINISH를 받음
              else if(_rxHeaderType == SCAN_FINISH_TO_MASTER)
              { 
                //ACK 전송
                manager.send(_thisAddress, _rxHeaderFrom, master_number, ACK, manager.temp_buf, sizeof(manager.temp_buf));
                
                //scan이 끝났음을 외부 마스터에게 알리기 전에 2초간 여유를 두고 모든 룸콘이 제어 메시지를 송신할 수 있도록 함
                //내부 통신의 경우 TX Power를 조절하여 다른 zone의 패킷을 수신하지 못하도록 하는 것도 좋을 것 같음
  
                timeout = false;
                controlTime = millis();
  
                //2초간 대기하며 자기 zone의 룸콘의 제어 메시지가 있는지를 확인함
                while(millis() - controlTime < 2000)
                {
                  manager.SetReceive();
                  if(manager.available() && manager.recvData(manager.temp_buf))
                  {
                    _rxHeaderFrom = manager.headerFrom();
                    _rxHeaderTo = manager.headerTo();
                    _rxHeaderMaster = manager.headerMaster();
                    _rxHeaderType = manager.headerType();
  
                    //내 zone의 룸콘으로부터 제어메시지를 수신했을 경우
                    if(_rxHeaderMaster == master_number && _rxHeaderType == RC_CONTROL_TO_SLAVE)
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
  
                //scan이 완료됐음을 외부 마스터에게 알림
                Serial2.write(INTERNAL_SCAN_FINISH);
              } // end of else if(_rxHeaderType == SCAN_FINISH_TO_MASTER) 
            }
          }
        }
      }
      if(timeout)
      {
        //외부 마스터에게 시리얼 보냄(INTERNAL_COM_FAIL)
        Serial2.write(3);
      }
    }
  }

  // 룸콘으로부터 제어요청
  // (룸콘) -----RC_CONTROL_TO_SLAVE-----> (슬레이브, 내부 마스터)
  //                                                 (내부 마스터) -----CONTROL_RESPONSE_BROADCAST-----> (룸콘, 슬레이브)  

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

          //checksum 계산
          for(int i = 0;i < 9;i++)
            inputData[9] ^= inputData[i];
            
          manager.send(_thisAddress, 0xFF, master_number, CONTROL_RESPONSE_BROADCAST, manager.temp_buf, sizeof(manager.temp_buf));
          RS485_Write_Read();
        }
      }
    }
  }
}
