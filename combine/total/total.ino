//rs485_time 확인
#include <ToDatagram_STM.h>
#include <ToInDatagram_STM.h>

#define SSerialRX        PA3  //Serial Receive pin DI
#define SSerialTX        PA2  //Serial Transmit pin RO

#define SSerialTxControl PA1   //RS485 Direction control
#define RS485Transmit    HIGH
#define RS485Receive     LOW

#define GATEWAY 0
#define IN_MASTER 1
#define OUT_MASTER 2
#define ROOMCON 3
#define SLAVE 4

byte device;

TO_ELECHOUSE_CC1120 cc1120;

ToDatagram manager(cc1120, 0x00, 0x01); //외부 통신
ToInDatagram manager2(cc1120, 0x01, 0x02); //내부 통신

// 헤더를 저장하기 위한 변수
uint8_t     _thisAddress;
uint8_t     _rxHeaderTo;
uint8_t     _rxHeaderFrom;
uint8_t     _rxHeaderSource;
uint8_t     _rxHeaderDestination;
uint8_t     _rxHeaderType;
uint8_t     _rxHeaderData;
uint8_t     _rxHeaderFlags;
uint8_t     _rxHeaderSeqNum;
uint8_t     _rxHeaderHop;

uint8_t _rxHeaderMaster;

uint8_t receivedRequestNum = 0;
uint8_t receivedOverhearNum = 1;

uint8_t priorPacketFrom = -1;
uint8_t priorPacketType = -1;

bool newNode = false;
bool rerouting = false;
bool scanFinish = false;
uint8_t receivedNum[33] = { 0 };
int8_t rerouting_candidate = -1;

uint8_t check_cnt = 0;
unsigned long check_table_time;
unsigned long check_rerouting_time;
unsigned long rs485_time;

uint8_t masterBroadcastAddress = 0xFF;

byte receivedFromInMaster;

// multi-hop일 경우, 혹은 그 외의 상황에서 timeout여부를 확인하기 위한 변수
bool timeout = true;

uint8_t master_number; // 내가 속한 zone의 외부 master 넘버
uint8_t num_of_slave; // 내가 속한 zone의 slave 개수


byte inputData[10]; // RS485 송신 버퍼
byte outputData[10]; // RS485 수신 버퍼
byte state_request[10] {0xD5, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00}; //에어텍 내부 통신 프로토콜 상태 요청 메시지

byte receivedFromOutMaster;

byte receiveFromR[10];
byte roomcon_control[10];
bool rc_control = false;

byte slave_answer[16][10];
byte roomcon_request[16][10];
unsigned long rs485_time_arr[16];

uint8_t indexFromR = 0;
uint8_t id;

unsigned long controllingTime;
unsigned long scanningTime;
bool check_slave_scan_response[16];

byte tempData[10] = {0xD5, 0x00, 0x02, 0x00, 0x03, 0x03, 0x19, 0x1E, 0x00, 0xD0}; //for test

bool scanning = false;
unsigned long rs485_loop_time;
unsigned long scanTime;
byte scanningAddress;

// 현재 scan 중인 master number(외부 scan) -> 내부 scan과 구분을 위해 check라는 용어 사용
uint8_t checkMasterNum = 1;
uint8_t address_i;
unsigned long startTime;

// 현재 scan 중인 master number(내부 scan)
uint8_t scanMasterNum = 1;

// 다음 마스터에게 내부 스캔 명령을 내려도 되는지 여부를 위한 변수
bool nextScan = true;

// 마스터로부터 받은 응답 메시지를 저장하기 위한 배열 (에어텍 외부통신 프로토콜도 10Bytes)
byte master_answer[32][10];

// 실제 게이트웨이에게 받은 메시지(에어텍 외부통신 프로토콜)을 저장하기 위한 배열
byte gateway_request[32][10];

// 제어명령 수신 여부를 체크하는 배열
// 만일 실제 게이트웨이가 1번 마스터에게 제어 명령을 내렸다면 control_message[1]에 해당 메시지를 저장하고
// beControl[1]을 true로 set

bool beControl[32];
byte control_message[32][10];
unsigned long controlRecvTime[32];

// 실제 게이트웨이로부터 받은 메시지를 임시로 저장
byte receiveFromG[10];
uint8_t indexFromG = 0;
uint8_t group_id;

unsigned long controlTime = 0;

uint8_t receiveFromInMaster;

void setup()
{
  
  //device 정함

  
  switch(device)
  {
    case GATEWAY:
    {
      Serial.begin(9600);
      pinMode(SSerialTxControl, OUTPUT);  
      digitalWrite(SSerialTxControl, RS485Receive);  // Init Transceiver
      Serial1.begin(9600);   // set the data rate 
    
      nvic_irq_set_priority(NVIC_USART2, 0xE);
   
      attachInterrupt(PA3, receiveFromGW, FALLING);
      
      manager.init(15);
      manager.SetReceive();
      manager.FromGatewayToMaster();
      _thisAddress = manager.getThisAddress();
      break;
    }
    case IN_MASTER:
    {
      Serial.begin(9600);
      pinMode(SSerialTxControl, OUTPUT);  
      digitalWrite(SSerialTxControl, RS485Receive);  // Init Transceiver
      Serial1.begin(9600);   // set the data rate 
      manager2.init(15);
      manager2.SetReceive();
    
     _thisAddress = manager2.getThisAddress();
     master_number = manager2.getMasterNumber();
    
     state_request[1] = master_number;
     for(int i = 0;i < 9;i++)
      state_request[9] ^= state_request[i];
      break;
    }
    case OUT_MASTER:
    {
       Serial.begin(9600);
       pinMode(SSerialTxControl, OUTPUT);  
       digitalWrite(SSerialTxControl, RS485Receive);  // Init Transceiver
       Serial1.begin(9600);   // set the data rate 
       Serial2.begin(9600);
       manager.init(15);
       manager.SetReceive();
    
      _thisAddress = manager.getThisAddress();
      check_table_time = millis();
      check_rerouting_time = millis();
      break;
    }
    case ROOMCON:
    {
      Serial.begin(9600);
      pinMode(SSerialTxControl, OUTPUT);  
      digitalWrite(SSerialTxControl, RS485Receive);  // Init Transceiver
      Serial1.begin(9600);   // set the data rate 
      
      manager2.init(15);
      manager2.SetReceive();
      _thisAddress = manager2.getThisAddress();
    
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
    
      master_number = manager2.getMasterNumber();
      break;
    }
    case SLAVE:
    {
      Serial.begin(9600);
      pinMode(SSerialTxControl, OUTPUT);  
      digitalWrite(SSerialTxControl, RS485Receive);  // Init Transceiver
      Serial1.begin(9600);   // set the data rate 
      manager2.init(15);
      manager2.SetReceive();
      
      _thisAddress = manager2.getThisAddress();
      master_number = manager2.getMasterNumber();
    }
  }
}
void loop()
{
  switch(device)
  {
    case GATEWAY:
    {
      while(1)
      {
        manager.printTree();

        // 내부 scan 명령을 내린 마스터로부터 10분 안에 응답을 받지 못하면 다음 마스터의 scan으로 넘어가도록
        if(scanTime + 600000 < millis())
        { 
          // scanMasterNum이 속한 zone의 에러
          // 어떤 에러인지는 알아서..
          nextScan = true;
          if(++scanMasterNum > 32) //manager.master_num)
            scanMasterNum = 1;
        }nextScan = false; // for test
      
      
        //*****************************************************************************************************************제어 명령**********************************************************************************************************************************************
        //제어 메시지 전송, beControl이 set되어 있으면 해당 마스터에게 제어 메시지를 보냄
        for(int beControlIndex = 0;beControlIndex <= 32;beControlIndex++) // 마스터 번호(유선 통신) : 0 ~ 0x1F, 무선통신 주소 : 1 ~ 0x20
        {
          if(beControl[beControlIndex])
          {
            // 마스터 번호(유선 통신) : 0 ~ 0x1F, 무선통신 주소 : 1 ~ 0x20
            address_i = beControlIndex + 1;// convertToAddress(gatewayNumber, i, 0);
      
            // 실제 게이트웨이로부터 받은 제어 메시지를 무선통신 버퍼에 옮김
            for(int i = 0;i < 10;i++)
              manager.temp_buf[i] = control_message[beControlIndex][i];
              
            controlTime = millis();
            
            // 라우팅이 정상인 마스터일 경우
            if(manager.checkReceive[address_i])
            {
              //원하는 목적지의 next_hop에게 패킷 전송 (type : CONTROL_TO_MASTER)
              if (!manager.sendToWaitAck(_thisAddress, manager.getRouteTo(address_i)->next_hop, _thisAddress, address_i, CONTROL_TO_MASTER, NONE, NONE, NONE, NONE, manager.temp_buf, sizeof(manager.temp_buf)))
              {
                // next_hop으로부터 ACK을 받지 못했을 경우
                uint8_t reroutingAddr = manager.getRouteTo(address_i)->next_hop;
      
                // next_hop의 unRecvCnt(패킷을 받지 못한 횟수를 카운트하는 변수) 값을 증가
                if (manager.unRecvCnt[reroutingAddr]++ > MAX_UN_RECV)
                {
                  // unRecvCnt가 threshold 이상일 때 다시 라우팅해줌
                  manager.checkReceive[reroutingAddr] = false;
                  manager.parentMaster[reroutingAddr] = -1;
                  manager.G_discoverNewPath(reroutingAddr, manager.getRouteTo(reroutingAddr)->hop);
                  manager.unRecvCnt[reroutingAddr] = 0;
                  manager.printTree();
                }
              }
              else
              {
                timeout = true;
                startTime = millis();
                while (millis() - startTime < DEFAULT_RETRIES * 4 * manager.getRouteTo(address_i)->hop * TIME_TERM + 5000) // 넉넉히 하려고 *4, 5초는 zone안에서 제어 데이터를 주고받는 시간을 기다려기 위함
                {
                  manager.SetReceive();
                  if (manager.available())
                  {
                    if (manager.recvData(manager.temp_buf) && manager.headerTo() == _thisAddress)
                    {
                      // 헤더 저장
                      _rxHeaderTo = manager.headerTo();
                      _rxHeaderFrom = manager.headerFrom();
                      _rxHeaderSource = manager.headerSource();
                      _rxHeaderDestination = manager.headerDestination();
                      _rxHeaderType = manager.headerType();
                      _rxHeaderData = manager.headerData();
                      _rxHeaderFlags = manager.headerFlags();
                      _rxHeaderSeqNum = manager.headerSeqNum();
                      _rxHeaderHop = manager.headerHop();
              
                      //manager.printRecvPacketHeader();
                      timeout = false;
      
                      // ACK 전송
                      manager.send(_thisAddress, _rxHeaderFrom, _thisAddress, _rxHeaderFrom, ACK, NONE, NONE, NONE, NONE, manager.temp_buf, sizeof(manager.temp_buf));
                      
                      if (_rxHeaderType == NACK)
                      {
                        Serial.println("NACK");
                        uint8_t reroutingAddr = manager.temp_buf[0];
                        if (manager.unRecvCnt[reroutingAddr]++ > MAX_UN_RECV)
                        {
                          manager.checkReceive[reroutingAddr] = false;
                          manager.parentMaster[reroutingAddr] = -1;
                          manager.G_discoverNewPath(reroutingAddr, manager.getRouteTo(reroutingAddr)->hop);
                          manager.unRecvCnt[reroutingAddr] = 0;
                          manager.printTree();
                        }
                      }
                      else if (_rxHeaderType == CONTROL_RESPONSE_TO_GATEWAY)
                      {
                        // control메시지를 무선으로 전송 중에 게이트웨이가 또 제어명령을 할 수도 있으므로
                        // 게이트웨이로부터 명령을 받은 시간(A)을 기록해두고 무선으로 이 제어 명령을 보낸 시간이 A이후일 경우만 제어명령이 완료되었음을 표시
                        
                        if(controlTime > controlRecvTime[beControlIndex])
                          beControl[beControlIndex] = false; // 제어명령을 전송하였음을 표시
                        
                        for(uint8_t i = 0;i < 10;i++)
                          master_answer[beControlIndex][i] = manager.temp_buf[i];
                      }
                      break;
                    }
                  } // end of if(manager.available)
                } // end of while
                if (timeout)
                {
                  Serial.println("timeout!");
                  manager.G_find_error_node(address_i);
                }
              } // end of else if (manager.getRouteTo(address_i)->hop != 1)
            } // end of if(manager.checkReceive[address_i])
          } // end of if(beControl[beControlIndex])
          
          /*else
          {
            for(uint8_t i = 0;i < 10;i++)
              master_answer[beControlIndex][i] = 0;
          }*/
        }
      
      
      
      
      
      
      
      
      
      
        //*****************************************************************************************************************내부 zone scan 명령**********************************************************************************************************************************************
        // nextScan default value : true, 마스터로부터 내부 zone의 scan이 끝났다는 패킷을 받거나, 내부 zone으로부터 일정 시간이 지나도 scan 완료 패킷을 받지 못한다면 true로 set됨
        // 각 zone의 scan 요청
        // scanMasterNum : 현재 scan할 zone의 마스터 번호
        
        if(nextScan)
        {
          Serial.println("SCAN");
          Serial.println(scanMasterNum);
          address_i = scanMasterNum;
      
          // 라우팅이 정상인 마스터일 경우
          if(manager.checkReceive[address_i])
          {
            // sendToWaitAck으로 해당 마스터의 next_hop에게 패킷 전송
            if (!manager.sendToWaitAck(_thisAddress, manager.getRouteTo(address_i)->next_hop, _thisAddress, address_i, SCAN_REQUEST_TO_MASTER, NONE, NONE, NONE, NONE, manager.temp_buf, sizeof(manager.temp_buf)))
            {
              // next_hop으로부터 ACK을 받지 못한경우
              uint8_t reroutingAddr = manager.getRouteTo(address_i)->next_hop;
      
              // 해당 주소의 unRecvCnt값을 증가
              if (manager.unRecvCnt[reroutingAddr]++ > MAX_UN_RECV)
              {
                // unRecvCnt가 threshold이상일 경우 그 마스터의 경로 설정을 다시해줌
                manager.checkReceive[reroutingAddr] = false;
                manager.parentMaster[reroutingAddr] = -1;
                manager.G_discoverNewPath(reroutingAddr, manager.getRouteTo(reroutingAddr)->hop);
                manager.unRecvCnt[reroutingAddr] = 0;
                manager.printTree();
              }
            }
            else if(manager.getRouteTo(address_i)->hop == 1)
            {
              // one hop일경우 sendToWaitAck으로 이미 SCAN_REQUEST_ACK_FROM_MASTER를 수신함
      
              // 헤더 저장
              _rxHeaderTo = manager.headerTo();
              _rxHeaderFrom = manager.headerFrom();
              _rxHeaderSource = manager.headerSource();
              _rxHeaderDestination = manager.headerDestination();
              _rxHeaderType = manager.headerType();
              _rxHeaderData = manager.headerData();
              _rxHeaderFlags = manager.headerFlags();
              _rxHeaderSeqNum = manager.headerSeqNum();
              _rxHeaderHop = manager.headerHop();
      
              // ACK 전송
              manager.send(_thisAddress, _rxHeaderFrom, _thisAddress, _rxHeaderFrom, ACK, NONE, NONE, NONE, NONE, manager.temp_buf, sizeof(manager.temp_buf));
              
              nextScan = false;
              scanTime = millis();
            }
            else if (manager.getRouteTo(address_i)->hop != 1)
            {
              //multi hop일 경우 SCAN_REQUEST_ACK_FROM_MASTER를 수신할 때까지 기다려줌
              
              timeout = true;
              startTime = millis();
              while (millis() - startTime < DEFAULT_RETRIES * 4 * manager.getRouteTo(address_i)->hop * TIME_TERM)
              {
                manager.SetReceive();
                if (manager.available())
                {
                  if (manager.recvData(manager.temp_buf) && manager.headerTo() == _thisAddress)
                  {
                    _rxHeaderTo = manager.headerTo();
                    _rxHeaderFrom = manager.headerFrom();
                    _rxHeaderSource = manager.headerSource();
                    _rxHeaderDestination = manager.headerDestination();
                    _rxHeaderType = manager.headerType();
                    _rxHeaderData = manager.headerData();
                    _rxHeaderFlags = manager.headerFlags();
                    _rxHeaderSeqNum = manager.headerSeqNum();
                    _rxHeaderHop = manager.headerHop();
            
                    manager.printRecvPacketHeader();
                    timeout = false;
                    manager.send(_thisAddress, _rxHeaderFrom, _thisAddress, _rxHeaderFrom, ACK, NONE, NONE, NONE, NONE, manager.temp_buf, sizeof(manager.temp_buf));
        
                    
                    if (_rxHeaderType == NACK)
                    {
                      Serial.println("NACK");
                      uint8_t reroutingAddr = manager.temp_buf[0];
                      if (manager.unRecvCnt[reroutingAddr]++ > MAX_UN_RECV)
                      {
                        manager.checkReceive[reroutingAddr] = false;
                        manager.parentMaster[reroutingAddr] = -1;
                        manager.G_discoverNewPath(reroutingAddr, manager.getRouteTo(reroutingAddr)->hop);
                        manager.unRecvCnt[reroutingAddr] = 0;
                        manager.printTree();
                      }
                    }
                    else if (_rxHeaderType == SCAN_REQUEST_ACK_FROM_MASTER)
                    {
                      scanTime = millis();
                      nextScan = false;
                    }
                    break;
                  } // end of if (manager.recvData(manager.temp_buf) && manager.headerTo() == _thisAddress)
                } // end of if (manager.available())
              } // end of while
              if (timeout)
              {
                Serial.println("timeout!");
                manager.G_find_error_node(address_i);
              }
            } 
          }// end of if(manager.checkReceive[address_i])
        }
      
      
      
      
      
      
        //*****************************************************************************************************************외부 master scan (check)**********************************************************************************************************************************************  
        address_i = checkMasterNum;
      
        // 보낼 데이터를 패킷의 payload에 복사
        for(int i = 0;i < 10;i++)
        {
          manager.temp_buf[i] = gateway_request[checkMasterNum - 1][i];
        }
      
        // 라우팅이 정상인 마스터일 경우
        if(manager.checkReceive[address_i])
        {
          if (!manager.sendToWaitAck(_thisAddress, manager.getRouteTo(address_i)->next_hop, _thisAddress, address_i, CHECK_ROUTING, NONE, NONE, NONE, NONE, manager.temp_buf, sizeof(manager.temp_buf)))
          {
            uint8_t reroutingAddr = manager.getRouteTo(address_i)->next_hop;
            if (manager.unRecvCnt[reroutingAddr]++ > MAX_UN_RECV)
            {
              manager.checkReceive[reroutingAddr] = false;
              manager.parentMaster[reroutingAddr] = -1;
              manager.G_discoverNewPath(reroutingAddr, manager.getRouteTo(reroutingAddr)->hop);
              manager.unRecvCnt[reroutingAddr] = 0;
              manager.printTree();
            }
          }
          
          // multi hop노드의 경우
          else if (manager.getRouteTo(address_i)->hop != 1)
          {
            startTime = millis();
            timeout = true;
      
            // multi hop이므로 src<-->dst 전송까지 대기
            while (millis() - startTime < DEFAULT_RETRIES * 4 * manager.getRouteTo(address_i)->hop * TIME_TERM)
            {
      
              // 수신 상태로 전환
              manager.SetReceive();
      
              // 패킷 수신
              if (manager.available())
              {
                if (manager.recvData(manager.temp_buf) && manager.headerTo() == _thisAddress)
                {
                  // 헤더 저장
                  _rxHeaderTo = manager.headerTo();
                  _rxHeaderFrom = manager.headerFrom();
                  _rxHeaderSource = manager.headerSource();
                  _rxHeaderDestination = manager.headerDestination();
                  _rxHeaderType = manager.headerType();
                  _rxHeaderData = manager.headerData();
                  _rxHeaderFlags = manager.headerFlags();
                  _rxHeaderSeqNum = manager.headerSeqNum();
                  _rxHeaderHop = manager.headerHop();
        
                  manager.printRecvPacketHeader();
                  timeout = false;
      
                  // ACK 전송
                  manager.send(_thisAddress, _rxHeaderFrom, _thisAddress, _rxHeaderFrom, ACK, NONE, NONE, NONE, NONE, manager.temp_buf, sizeof(manager.temp_buf));
      
                  // 패킷이 dst까지 relay되지 못 함
                  if (_rxHeaderType == NACK)
                  {
                    Serial.println("NACK");
      
                    // 전송이 끊긴 노드의 unRecvCnt 값을 증가
                    uint8_t reroutingAddr = manager.temp_buf[0];
                    if (manager.unRecvCnt[reroutingAddr]++ > MAX_UN_RECV)
                    {
                      manager.checkReceive[reroutingAddr] = false;
                      manager.parentMaster[reroutingAddr] = -1;
                      manager.G_discoverNewPath(reroutingAddr, manager.getRouteTo(reroutingAddr)->hop);
                      manager.unRecvCnt[reroutingAddr] = 0;
                      manager.printTree();
                    }
                  }
      
                  // 패킷이 dst까지 relay됨
                  else
                  {
                    // 수신한 데이터 저장
                    for(int i = 0;i < 10;i++)
                      master_answer[checkMasterNum - 1][i] = manager.temp_buf[i];
      
                    // scan완료
                    if (_rxHeaderType == CHECK_ROUTING_ACK)
                    {
                      // 새로운 master가 추가되었다면 _rxHeaderData에 새로 추가된 노드의 개수, payload(master응답 데이터 이후에, index 10부터)에 추가된 노드의 주소가 들어있음
                      for (uint8_t i = 0; i < _rxHeaderData; i++)
                      {
                        manager.addRouteTo(manager.temp_buf[10 + i], _rxHeaderFrom, manager.Valid, manager.getRouteTo(_rxHeaderSource)->hop + 1);
                        manager.checkReceive[manager.temp_buf[10 + i]] = true;
                        manager.parentMaster[manager.temp_buf[10 + i]] = _rxHeaderSource;
                        manager.master_num++;
                        manager.printRoutingTable();
                      }
                    }
      
                    // scan 완료 + 마스터의 경로 변경
                    else if (_rxHeaderType == CHECK_ROUTING_ACK_REROUTING)
                    {
                      int8_t temp_hopCount = manager.getRouteTo(_rxHeaderSource)->hop;
                      manager.addRouteTo(_rxHeaderSource, _rxHeaderFrom, manager.Valid, _rxHeaderHop + 1);
                      manager.changeNextHop(_rxHeaderSource, temp_hopCount - (_rxHeaderHop + 1));
                      manager.parentMaster[_rxHeaderSource] = manager.temp_buf[10];
          
                      manager.printRoutingTable();
                    }
      
                    // scan 완료 + 해당 zone의 내부 scan 완료
                    else if(_rxHeaderType == SCAN_RESPONSE_TO_GATEWAY)
                    {
                      if(++scanMasterNum > 32)//manager.master_num)
                        scanMasterNum = 1;
                      nextScan = true;
                    }
                  }
                  break;
                }
              }
            } // end of while
            if (timeout)
            {
              Serial.println("timeout!");
              manager.G_find_error_node(address_i);
            }
          } // end of multihop 노드 처리
      
      
          // 원래 1hop이거나 rerouting 결과 1hop인 노드들, 위에랑 같은 처리
          else 
          {
            // 헤더 저장
            _rxHeaderTo = manager.headerTo();
            _rxHeaderFrom = manager.headerFrom();
            _rxHeaderSource = manager.headerSource();
            _rxHeaderDestination = manager.headerDestination();
            _rxHeaderType = manager.headerType();
            _rxHeaderData = manager.headerData();
            _rxHeaderFlags = manager.headerFlags();
            _rxHeaderSeqNum = manager.headerSeqNum();
            _rxHeaderHop = manager.headerHop();
      
            // ACK 전송
            manager.send(_thisAddress, _rxHeaderFrom, _thisAddress, _rxHeaderFrom, ACK, NONE, NONE, NONE, NONE, manager.temp_buf, sizeof(manager.temp_buf));
            
            for(int i = 0;i < 10;i++)
              master_answer[checkMasterNum - 1][i] = manager.temp_buf[i];
              
            if (_rxHeaderType == CHECK_ROUTING_ACK)
            {
              for (uint8_t i = 0; i < _rxHeaderData; i++)
              {
                manager.addRouteTo(manager.temp_buf[10 + i], _rxHeaderFrom, manager.Valid, manager.getRouteTo(_rxHeaderSource)->hop + 1);
                manager.checkReceive[manager.temp_buf[10 + i]] = true;
                manager.parentMaster[manager.temp_buf[10 + i]] = _rxHeaderSource;
                manager.printRoutingTable();
                manager.master_num++;
              }
            }
            else if (_rxHeaderType == CHECK_ROUTING_ACK_REROUTING)
            {
              int8_t temp_hopCount = manager.getRouteTo(_rxHeaderSource)->hop;
              manager.addRouteTo(_rxHeaderSource, _rxHeaderFrom, manager.Valid, _rxHeaderHop + 1);
              manager.changeNextHop(_rxHeaderSource, temp_hopCount - (_rxHeaderHop + 1));
              manager.parentMaster[_rxHeaderSource] = manager.temp_buf[10];
          
              manager.printRoutingTable();
            }
            else if(_rxHeaderType == SCAN_RESPONSE_TO_GATEWAY)
            {
              if(++scanMasterNum > 32) //manager.master_num)
                scanMasterNum = 1;
              nextScan = true;
            }
          } // end of 1hop 노드 처리
        } // end of if(manager.checkReceive[address_i]) -> 라우팅이 정상인 마스터일 경우
      
        // 라우팅이 제대로 되지않은 마스터일 경우
        else
        {
          //FCU에게 error임을 표시해주기 위해서
          for(uint8_t i = 0;i < 10;i++)
            master_answer[checkMasterNum - 1][i] = 0;
        }
      
      
      
        //다음 마스터 check
        if(++checkMasterNum > 32) //manager.master_num)
          checkMasterNum = 1;
      }
      break;
    }
    case IN_MASTER:
    {
      while(1)
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
              manager2.temp_buf[i] = outputData[i];
            }
            manager2.temp_buf[9] = outputData[9];
      
            //룸콘에게 전송
            bool receiveACK = manager2.sendToWaitAck(_thisAddress, 0x00, master_number, CONTROL_TO_RC, manager2.temp_buf, sizeof(manager2.temp_buf));
      
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
                manager2.SetReceive();
      
                //패킷 수신
                if(manager2.available())
                {
      
                  //받은 데이터를 저장
                  if(manager2.recvData(manager2.temp_buf))
                  {
      
                    //헤더 저장
                    _rxHeaderFrom = manager2.headerFrom();
                    _rxHeaderTo = manager2.headerTo();
                    _rxHeaderMaster = manager2.headerMaster();
                    _rxHeaderType = manager2.headerType();
      
                    //내가 속한 zone으로부터의 패킷인 경우
                    if(_rxHeaderMaster == master_number)
                    {
                      //룸콘의 제어 요청 수신
                     if(_rxHeaderType == GW_CONTROL_TO_SLAVE)
                      {
                        timeout = false;
                        manager2.send(_thisAddress, 0xFF, master_number, CONTROL_RESPONSE_BROADCAST, manager2.temp_buf, sizeof(manager2.temp_buf));
      
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
            bool receiveACK = manager2.sendToWaitAck(_thisAddress, 0x00, master_number, SCAN_REQUEST_TO_RC, manager2.temp_buf, sizeof(manager2.temp_buf));
      
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
              manager2.SetReceive();
      
              //패킷 수신
              if(manager2.available())
              {
      
                //수신된 데이터를 자신의 버퍼에 저장
                if(manager2.recvData(manager2.temp_buf))
                {
      
                  //read header
                  _rxHeaderFrom = manager2.headerFrom();
                  _rxHeaderTo = manager2.headerTo();
                  _rxHeaderMaster = manager2.headerMaster();
                  _rxHeaderType = manager2.headerType();
      
                  //내가 속한 zone으로부터의 패킷인 경우
                  if(_rxHeaderMaster == master_number)
                  {
      
                    //룸콘으로부터 스캔요청을 수신
                    if(_rxHeaderType == SCAN_REQUEST_TO_SLAVE && _rxHeaderTo == _thisAddress)
                    {
                      for(int i = 0;i < 10;i++)
                        inputData[i] = manager2.temp_buf[i];
                      
                      num_of_slave = manager2.temp_buf[10];
                      RS485_Write_Read();
                      //FCU에게 현태 상태를 물어봄
                      
                      for(int i = 0;i < 10;i++)
                        manager2.temp_buf[i] = outputData[i];
      
                      //룸콘에게 스캔 응답
                      manager2.send(_thisAddress, 0, master_number, SCAN_RESPONSE_TO_RC, manager2.temp_buf, sizeof(manager2.temp_buf));
                    }
      
                    //룸콘으로부터 SCAN_FINISH를 받음
                    else if(_rxHeaderType == SCAN_FINISH_TO_MASTER)
                    { 
                      //ACK 전송
                      manager2.send(_thisAddress, _rxHeaderFrom, master_number, ACK, manager2.temp_buf, sizeof(manager2.temp_buf));
                      
                      //scan이 끝났음을 외부 마스터에게 알리기 전에 2초간 여유를 두고 모든 룸콘이 제어 메시지를 송신할 수 있도록 함
                      //내부 통신의 경우 TX Power를 조절하여 다른 zone의 패킷을 수신하지 못하도록 하는 것도 좋을 것 같음
        
                      timeout = false;
                      controlTime = millis();
        
                      //2초간 대기하며 자기 zone의 룸콘의 제어 메시지가 있는지를 확인함
                      while(millis() - controlTime < 2000)
                      {
                        manager2.SetReceive();
                        if(manager2.available() && manager2.recvData(manager2.temp_buf))
                        {
                          _rxHeaderFrom = manager2.headerFrom();
                          _rxHeaderTo = manager2.headerTo();
                          _rxHeaderMaster = manager2.headerMaster();
                          _rxHeaderType = manager2.headerType();
        
                          //내 zone의 룸콘으로부터 제어메시지를 수신했을 경우
                          if(_rxHeaderMaster == master_number && _rxHeaderType == RC_CONTROL_TO_SLAVE)
                          {
                            for(int i = 0;i < 10;i++)
                              inputData[i] = manager2.temp_buf[i];
                              
                            inputData[2] = 0x00;
                            inputData[9] = 0x00;
                            
                            for(int i = 0;i < 9;i++)
                              inputData[9] ^= inputData[i];
                              
                            manager2.send(_thisAddress, 0xFF, master_number, CONTROL_RESPONSE_BROADCAST, manager2.temp_buf, sizeof(manager2.temp_buf));
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
      
        manager2.SetReceive();
        if(manager2.available())
        {
          if(manager2.recvData(manager2.temp_buf))
          {
            _rxHeaderFrom = manager2.headerFrom();
            _rxHeaderTo = manager2.headerTo();
            _rxHeaderMaster = manager2.headerMaster();
            _rxHeaderType = manager2.headerType();
      
            if(_rxHeaderMaster == master_number)
            {
              if(_rxHeaderType == RC_CONTROL_TO_SLAVE)
              {
                for(int i = 0;i < 10;i++)
                  inputData[i] = manager2.temp_buf[i];
                  
                inputData[2] = 0x00;
                inputData[9] = 0x00;
      
                //checksum 계산
                for(int i = 0;i < 9;i++)
                  inputData[9] ^= inputData[i];
                  
                manager2.send(_thisAddress, 0xFF, master_number, CONTROL_RESPONSE_BROADCAST, manager2.temp_buf, sizeof(manager2.temp_buf));
                RS485_Write_Read();
              }
            }
          }
        }
      }
      break;
    }
    case OUT_MASTER:
    {
      while(1)
      {
         // if()//scan finish 명령을 내부 마스터로부터 받음, 인터럽트로 처리해야 할 듯
        {
          scanFinish = true;
        }
      
        // 10분 간격으로 table update (사용되지 않는 routing table의 행은 삭제)
        if (check_table_time + 600000 < millis())
        {
          manager.checkRoutingTable();
          check_table_time = millis();
        }
      
        // 10분 간격으로 내가 선택한 부모 노드보다 gateway에 더 가까이 있는 노드가 있는 경우 다시 라우팅 해줌
        // 다른 노드의 패킷을 보고 hop count가 더 좋다면 receivedNum 값을 증가시켜줌 (line ???)
        // receivedNum 값이 제일 큰 노드가 후보 부모 노드가 됨
        
        if (check_rerouting_time + 60000 < millis())//10분으로..
        {
          Serial.println("check_rerouting_time");
          check_rerouting_time = millis();
          int8_t max_receivedNum = 0;
          for (uint8_t i = 0; i <= manager.master_num; i++)
          {
            Serial.print(i);
            Serial.print(" : ");
            Serial.println(receivedNum[i]);
            if (max_receivedNum < receivedNum[i])
            {
              rerouting_candidate = i;
              max_receivedNum = receivedNum[i];
              receivedNum[i] = 0;
              rerouting = true;
            }
          }
        }
      
        // 수신 가능 상태로
        manager.SetReceive();
      
        // 패킷 수신
        if (manager.available())
        {
      
          // 받은 데이터 버퍼에 저장
          if (manager.recvData(manager.temp_buf))
          {
      
            // 헤더를 따로 저장
            _rxHeaderTo = manager.headerTo();
            _rxHeaderFrom = manager.headerFrom();
            _rxHeaderSource = manager.headerSource();
            _rxHeaderDestination = manager.headerDestination();
            _rxHeaderType = manager.headerType();
            _rxHeaderData = manager.headerData();
            _rxHeaderFlags = manager.headerFlags();
            _rxHeaderSeqNum = manager.headerSeqNum();
            _rxHeaderHop = manager.headerHop();
      
            // 나에게 온 패킷일 경우 or 브로드캐스트일 경우
            if (_rxHeaderTo == _thisAddress || _rxHeaderTo == masterBroadcastAddress)
            {
              manager.printRecvPacketHeader();
      
              // 목적지가 내가 아닐 경우 (relay 상황)
              if (_rxHeaderDestination != _thisAddress && _rxHeaderTo != masterBroadcastAddress)
              {
                uint8_t temp_source = _rxHeaderSource;
                Serial.println("receive relay");
      
                // gateway가 테이블에 등록되어 있지 않음, 아직 라우팅이 되지 않은 노드
                if (manager.getRouteTo(_thisAddress & 0x00) == NULL)
                {
                  manager.printRoutingTable();
                }
      
                // gateway가 테이블에 등록되어 있음, 라우팅이 완료된 노드
                else
                {
                  // ACK전송 (hop-to-hop에도 ACK을 전송해줌)
                  manager.send(_thisAddress, _rxHeaderFrom, _thisAddress, _rxHeaderFrom, ACK, NONE, NONE, NONE, NONE, manager.temp_buf, sizeof(manager.temp_buf));
      
                  // routing관련 응답을 수신 (내 자식노드가 생겼다는 뜻, 따라서 라우팅 테이블에 등록시킴)
                  if (_rxHeaderType == REQUEST_MULTI_HOP_ACK || _rxHeaderType == REQUEST_DIRECT_ACK)
                  {
                    manager.addRouteTo(_rxHeaderSource, _rxHeaderFrom, manager.Valid, _rxHeaderHop + 1);
                    manager.printRoutingTable();
                  }
      
                  // 초기라우팅 완료 후, 운영 중에 _rxHeaderData에 값이 있다는 것은 NEW_NODE가 있음을 의미, 나에게 ACK이 왔으므로 내 자식노드임 따라서 라우팅 테이블에 등록
                  // _rxHeaderData는 추가해야 할 자식노드의 개수를 의미, temp_buf[10]부터 자식노드의 주소가 들어있음
                  else if (_rxHeaderType == CHECK_ROUTING_ACK && _rxHeaderData > 0)
                  {
                    // hop count 재설정
                    manager.addRouteTo(_rxHeaderSource, _rxHeaderFrom, manager.Valid, _rxHeaderHop + 1);
                    for (uint8_t i = 0; i < _rxHeaderData; i++)
                    {
                      manager.addRouteTo(manager.temp_buf[10 + i], _rxHeaderFrom, manager.Valid, _rxHeaderHop + 2);
                    }
                  }
      
                  //원래 내 자식노드가 아니었는데 rerouting 결과 내 자식노드로.. 따라서 라우팅 테이블에 등록해줌
                  else if (_rxHeaderType == CHECK_ROUTING_ACK_REROUTING)
                  {
                    manager.addRouteTo(_rxHeaderSource, _rxHeaderFrom, manager.Valid, _rxHeaderHop + 1);
                    for(uint8_t i = 0;i < _rxHeaderData;i++)
                    {
                      manager.addRouteTo(manager.temp_buf[11 + i], _rxHeaderFrom, manager.Valid, 0); // 자식노드의 hop count는 별로 중요하지 않음 일단 0으로 해두고 다음에 해당 자식노드로부터 패킷을 받게되면 hop count맞게 setting
                    }
                  }
      
                  
                  /*else if(_rxHeaderType == ROUTING_TABLE_UPDATE)
                  {
                    for (uint8_t i = 1; i < _rxHeaderData; i++)
                      manager.addRouteTo(manager.temp_buf[i], manager.temp_buf[0], manager.Valid, _rxHeaderHop + 1);
                  }*/
      
                  // 패킷의 원래 목적지에 대한 데이터가 내 라우팅 테이블에 존재하지 않음 || 원래 목적지로 전송하기 위해서 next_hop으로 패킷을 보냈는데 ACK을 받지 못한 경우
                  if (manager.getRouteTo(_rxHeaderDestination) == NULL
                    || !manager.sendToWaitAck(_thisAddress, manager.getRouteTo(_rxHeaderDestination)->next_hop, _rxHeaderSource, _rxHeaderDestination, _rxHeaderType, NONE, NONE, NONE, _rxHeaderHop + 1, manager.temp_buf, sizeof(manager.temp_buf)))
                      // sendToWaitAck은 전송 후 ACK 받기를 기다림, ACK을 받지 못했다면 false를 return
                  {
                    // relay되어야 하는 패킷이 gateway로부터 시작했을 때 gateway에게 목적지로의 전송이 안되었다고 알려줌
                    if (temp_source == (_thisAddress & 0x00))
                    {
                      Serial.println("send NACK");
                      manager.temp_buf[0] = manager.getRouteTo(_rxHeaderDestination)->next_hop;
                      manager.sendToWaitAck(_thisAddress, manager.getRouteTo(temp_source)->next_hop, _thisAddress, _thisAddress & 0x00, NACK, NONE, NONE, NONE, NONE, manager.temp_buf, sizeof(manager.temp_buf));
                    }
                  }
                } //end of else
              } // end of if (_rxHeaderDestination != _thisAddress && _rxHeaderTo != masterBroadcastAddress)
      
              // 밑의 경우는 실제 목적지가 자신이거나 브로드캐스트일경우
      
              ///////////////////////////////////////////////////////////////////////////////////////////////////////////////라우팅 관련 패킷///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
              else if (_rxHeaderType == REQUEST_BROADCAST)
              {
                manager.clearRoutingTable();
                Serial.println("receivedBroadcastAddress");
                Serial.println(receivedRequestNum);
                receivedRequestNum++;
              }
              else if (_rxHeaderType == REQUEST_TYPE && receivedRequestNum >= R_GATEWAY_SEND_NUM * 0.8)
              {
                receivedRequestNum = 0;
                Serial.println("receive row1 request");
                manager.printRoutingTable();
                manager.addRouteTo(_rxHeaderFrom, _rxHeaderFrom, manager.Valid, 1);
                //from, to, src, dst, type, data, flags, seqnum, hop
                for(uint8_t i = 0;i < R_MASTER_SEND_NUM;i++)
                  manager.send(_thisAddress, _rxHeaderFrom, _thisAddress, _rxHeaderFrom, REQUEST_ACK_TYPE, NONE, NONE, NONE, NONE, manager.temp_buf, sizeof(manager.temp_buf));
              }
              else if (_rxHeaderType == R2_REQUEST_TYPE)
              {
                receivedRequestNum = 0;
                //from, to, src, dst, type, data, flags, seqnum, hop
                manager.send(_thisAddress, _rxHeaderFrom, _thisAddress, _rxHeaderFrom, ACK, NONE, NONE, NONE, NONE, manager.temp_buf, sizeof(manager.temp_buf));
                manager.M_find2ndRowMasters();
              }
              else if (_rxHeaderType == R2_REQUEST_REAL_TYPE)
              {
                receivedRequestNum = 0;
                Serial.println("receive R2 routing request and I didn't success in R1 request");
                manager.M_masterSendRoutingReply();
              }
              else if (_rxHeaderType == REQUEST_MULTI_HOP || _rxHeaderType == REQUEST_DIRECT)
              {
                receivedRequestNum = 0;
      
                //실제 라우팅하고자 하는 노드의 주소는 payload의 첫번째 바이트에 저장되어 있음
                
                if (manager.temp_buf[0] != _thisAddress)
                {
                  //아직 routing 안된 노드에게 라우팅 요청 패킷 전송
                  
                  Serial.println("received multihop request and send to unrouting Master");
                  uint8_t realDst = manager.temp_buf[0];
                  //from, to, src, dst, type, data, flags, seqnum, hop
                  manager.send(_thisAddress, _rxHeaderFrom, _thisAddress, _rxHeaderFrom, ACK, NONE, NONE, NONE, NONE, manager.temp_buf, sizeof(manager.temp_buf));
                  uint8_t cnt = 0;
                  uint8_t type = _rxHeaderType;
      
                  manager.send(_thisAddress, realDst, _thisAddress & 0x00, realDst, _rxHeaderType, NONE, NONE, NONE, _rxHeaderHop + 1, manager.temp_buf, sizeof(manager.temp_buf));
                  unsigned long startTime = millis();
                  while (millis() - startTime < (R_MASTER_SEND_NUM) * TIME_TERM)
                  {
                    manager.SetReceive();
                    if (manager.available() && manager.recvData(manager.temp_buf) && manager.headerFrom() == realDst && manager.headerTo() == _thisAddress && (manager.headerType() == REQUEST_MULTI_HOP_ACK || manager.headerType() == REQUEST_DIRECT_ACK))
                    {
                      _rxHeaderTo = manager.headerTo();
                      _rxHeaderFrom = manager.headerFrom();
                      _rxHeaderSource = manager.headerSource();
                      _rxHeaderDestination = manager.headerDestination();
                      _rxHeaderType = manager.headerType();
                      _rxHeaderData = manager.headerData();
                      _rxHeaderFlags = manager.headerFlags();
                      _rxHeaderSeqNum = manager.headerSeqNum();
                      _rxHeaderHop = manager.headerHop();
                      cnt++;
                      if (cnt >= R_MASTER_SEND_NUM * 0.8)
                      {
                        manager.addRouteTo(_rxHeaderFrom, _rxHeaderFrom, manager.Valid, 1);
                        manager.printRecvPacketHeader();
                        manager.printRoutingTable();
                        manager.sendToWaitAck(_thisAddress, manager.getRouteTo(_thisAddress & 0x00)->next_hop, _rxHeaderSource, _thisAddress & 0x00, _rxHeaderType, 1, NONE, NONE, _rxHeaderHop + 1, manager.temp_buf, sizeof(manager.temp_buf));
                        break;
                      }
                    }
                  }
                  if (cnt < R_MASTER_SEND_NUM * 0.8)
                  {
                    if(type == REQUEST_MULTI_HOP)
                      manager.sendToWaitAck(_thisAddress, manager.getRouteTo(_thisAddress & 0x00)->next_hop, _thisAddress, _thisAddress & 0x00, REQUEST_MULTI_HOP_ACK, NONE, NONE, NONE, NONE, manager.temp_buf, sizeof(manager.temp_buf));
                    else
                      manager.sendToWaitAck(_thisAddress, manager.getRouteTo(_thisAddress & 0x00)->next_hop, _thisAddress, _thisAddress & 0x00, REQUEST_DIRECT_ACK, NONE, NONE, NONE, NONE, manager.temp_buf, sizeof(manager.temp_buf));
                  }
                }
                else
                {
                  //routing 안 되어있던 노드가 수신
                  Serial.println("received multi hop request");
                  if(_rxHeaderType == REQUEST_MULTI_HOP)
                    manager.M_masterSendRoutingReply();
                    
                  else if(_rxHeaderType == REQUEST_DIRECT)
                  {
                    manager.addRouteTo(_rxHeaderSource, _rxHeaderFrom, manager.Valid, _rxHeaderHop + 1);
                    manager.printRoutingTable();
                    manager.temp_buf[0] = _rxHeaderFrom;
                    //from, to, src, dst, type, data, flags, seqnum, hop
                    for (uint8_t i = 0; i < R_MASTER_SEND_NUM; i++)
                      manager.send(_thisAddress, _rxHeaderFrom, _thisAddress, _thisAddress & 0x00, REQUEST_DIRECT_ACK, NONE, NONE, NONE, NONE, manager.temp_buf, sizeof(manager.temp_buf));
                  }
                }
              }
              ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
      
      
      
      
              ////////////////////////////////////////////////////////////////////////////////////////////////////////////operate 관련 패킷///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
      
              // gateway로부터 master scan 요청 수신
              else if (_rxHeaderType == CHECK_ROUTING)
              {
                receivedRequestNum = 0;
      
                // gateway가 라우팅 테이블에 등록되어 있을 경우에만(재부팅된 노드가 데이터 송수신하는 것을 막기위해서)
                if (manager.getRouteTo(_thisAddress & 0x00) != NULL)
                { 
                  for(int i = 0;i < 10;i++)
                    inputData[i] = manager.temp_buf[i];
                    
                  RS485_Write_Read();
                  
                  for(int i = 0;i < 10;i++)
                    manager.temp_buf[i] = outputData[i];
      
                  // 내부의 scan이 끝남 + master scan 완료 패킷을 게이트웨이에게 전송
                  if(scanFinish)
                  {
                    manager.getRouteTo(_thisAddress & 0x00)->hop = _rxHeaderHop + 1;
                    manager.sendToWaitAck(_thisAddress, manager.getRouteTo(_thisAddress & 0x00)->next_hop, _thisAddress, _thisAddress & 0x00, SCAN_RESPONSE_TO_GATEWAY, NONE, NONE, NONE, NONE, manager.temp_buf, sizeof(manager.temp_buf));
                  }
                  
                  // 새로운 노드가 라우팅 테이블에 추가됨 + scan 완료 패킷을 게이트웨이에게 전송
                  else if (newNode)
                  {
                    int newNodeCnt = 0;
                    for (uint8_t i = 0; i < ROUTING_TABLE_SIZE; i++)
                    {
                      if (manager._routes[i].state == manager.Discovering)
                      {
                        manager.temp_buf[10 + newNodeCnt++] = manager._routes[i].dest;
                        manager._routes[i].state = manager.Valid;
                      }
                    }
                    newNode = false;
                    manager.sendToWaitAck(_thisAddress, manager.getRouteTo(_thisAddress & 0x00)->next_hop, _thisAddress, _thisAddress & 0x00, CHECK_ROUTING_ACK, newNodeCnt, NONE, NONE, NONE, manager.temp_buf, sizeof(manager.temp_buf));
                  }
                  
                  // 새로운 부모가 선택되었음 + scan 완료 패킷을 게이트웨이에게 전송
                  else if (rerouting || _rxHeaderFrom != manager.getRouteTo(_thisAddress & 0x00)->next_hop)
                  {
                    // 게이트웨이에게 나의 부모 노드를 알려주기 위해서 내가 선택한 부모노드의 주소를 payload에 추가해줌
                    // 게이트웨이 입장에서는 패킷을 누구로부터 받았는지 hop count가 누구인지 정도만 알 수 있으므로 따로 기입해줘야 함
                    
                    manager.temp_buf[10] = rerouting_candidate;
      
                    // 조상 노드들이 현재 노드의 자식 노드들을 라우팅 테이블에 등록시키게 하기 위해서 자식 노드들도 payload에 추가
                    uint8_t child_node[10];
                    uint8_t child_node_cnt = manager.find_child_node(child_node);
      
                    for(uint8_t i = 0;i < child_node_cnt;i++)
                      manager.temp_buf[11 + i] = child_node[i];
                    
                    if (manager.sendToWaitAck(_thisAddress, rerouting_candidate, _thisAddress, _thisAddress & 0x00, CHECK_ROUTING_ACK_REROUTING, child_node_cnt, NONE, NONE, NONE, manager.temp_buf, sizeof(manager.temp_buf)))
                      manager.addRouteTo(_thisAddress & 0x00, rerouting_candidate, manager.Valid, _rxHeaderHop + 1);
                    else
                      rerouting_candidate = 0;
                    rerouting = false;
                  }
      
                  // 일반 scan 완료 패킷 전송
                  else
                  {
                    // hop count update
                    manager.getRouteTo(_thisAddress & 0x00)->hop = _rxHeaderHop + 1;
                    manager.sendToWaitAck(_thisAddress, manager.getRouteTo(_thisAddress & 0x00)->next_hop, _thisAddress, _thisAddress & 0x00, CHECK_ROUTING_ACK, NONE, NONE, NONE, NONE, manager.temp_buf, sizeof(manager.temp_buf));
                  }
                }
                // gateway가 라우팅 테이블에 등록되어 있지 않음(reset된 상태)
                else
                  manager.printRoutingTable();
                  
              } // end of else if (_rxHeaderType == CHECK_ROUTING)
      
              // 새로운 노드가 자식 노드 설정 요청
              else if (_rxHeaderType == NEW_NODE_REGISTER && manager.getRouteTo(_thisAddress & 0x00) != NULL)
              {
                // 라우팅 테이블에 추가, 상태를 Discovering으로 하여 new node임을 표시
                manager.addRouteTo(_rxHeaderFrom, _rxHeaderFrom, manager.Discovering, 1);
      
                // ACK전송
                manager.send(_thisAddress, _rxHeaderFrom, _thisAddress, _rxHeaderFrom, ACK, NONE, NONE, NONE, manager.getRouteTo(_thisAddress & 0x00)->hop, manager.temp_buf, sizeof(manager.temp_buf));
                newNode = true;
              }
      
              
              /*else if (_rxHeaderType == ROUTING_TABLE_UPDATE && manager.getRouteTo(_thisAddress & 0x00) != NULL)
              {
                for (uint8_t i = 1; i < _rxHeaderData; i++)
                  manager.addRouteTo(manager.temp_buf[i], manager.temp_buf[0], manager.Valid, _rxHeaderHop + 1);
                manager.send(_thisAddress, _rxHeaderFrom, _thisAddress, _rxHeaderFrom, ACK, NONE, NONE, NONE, NONE, manager.temp_buf, sizeof(manager.temp_buf));
              }*/
             
      
      
              // 내가 속한 zone의 scan 요청
              else if(_rxHeaderType == SCAN_REQUEST_TO_MASTER && manager.getRouteTo(_thisAddress & 0x00) != NULL)
              {
                // hop count 업데이트
                manager.getRouteTo(_thisAddress & 0x00)->hop = _rxHeaderHop + 1;
      
                // 내부 zone scan 명령을 잘 받았다고 응답을 보냄
                manager.sendToWaitAck(_thisAddress, manager.getRouteTo(_thisAddress & 0x00)->next_hop, _thisAddress, _thisAddress & 0x00, SCAN_REQUEST_ACK_FROM_MASTER, NONE, NONE, NONE, NONE, manager.temp_buf, sizeof(manager.temp_buf));
      
                //send scan to internal master in serial
                Serial2.write(INTERNAL_SCAN);
                
              }
              
              else if(_rxHeaderType == CONTROL_TO_MASTER && manager.getRouteTo(_thisAddress & 0x00) != NULL)
              {
                
                manager.send(_thisAddress, _rxHeaderFrom, _thisAddress, _rxHeaderFrom, ACK, NONE, NONE, NONE, NONE, NULL, sizeof(NULL));
                
                manager.getRouteTo(_thisAddress & 0x00)->hop = _rxHeaderHop + 1;
                
                for(int i = 0;i < 10;i++)
                  inputData[i] = manager.temp_buf[i];
                
                RS485_Write_Read();
                
                for(int i = 0;i < 10;i++)
                  manager.temp_buf[i] = outputData[i];
                  
                //send control to internal master in serial
                Serial2.write(INTERNAL_CONTROL);
      
                timeout = true;
                //내부 마스터로부터 제어 명령 전송이 완료됐는지 데이터를 수신하기까지 대기
                
                unsigned long send_ctrl_to_in_master_time = millis();
                while(send_ctrl_to_in_master_time - millis() < 5000)
                {
                  if(Serial2.available())
                  {
                    receiveFromInMaster = Serial2.read();
                    manager.sendToWaitAck(_thisAddress, manager.getRouteTo(_thisAddress & 0x00)->next_hop, _thisAddress, _thisAddress & 0x00, CONTROL_RESPONSE_TO_GATEWAY, NONE, NONE, NONE, NONE, manager.temp_buf, sizeof(manager.temp_buf));
                    timeout = false;
                  }
                }
                if(timeout)
                  manager.sendToWaitAck(_thisAddress, manager.getRouteTo(_thisAddress & 0x00)->next_hop, _thisAddress, _thisAddress & 0x00, CONTROL_RESPONSE_TO_GATEWAY, NONE, NONE, NONE, NONE, manager.temp_buf, sizeof(manager.temp_buf));  
              }  
            }
            ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
      
            else
            {
              if (priorPacketFrom == _rxHeaderFrom && priorPacketType == _rxHeaderType && _rxHeaderFrom != (_thisAddress & 0x00))
              {
                if (++receivedOverhearNum >= R_MASTER_SEND_NUM * 0.8)
                  manager.M_findCandidateParents();
              }
              else
                receivedOverhearNum = 1;
              priorPacketFrom = _rxHeaderFrom;
              priorPacketType = _rxHeaderType;
              if (_rxHeaderType == CHECK_ROUTING && _rxHeaderSource == (_thisAddress & 0x00) && _rxHeaderHop + 1 < manager.getRouteTo(_thisAddress & 0x00)->hop)
              {
                Serial.println("hi");
                receivedNum[_rxHeaderFrom]++;
                Serial.print("_rxHeaderFrom : ");
                Serial.println(_rxHeaderFrom);
                Serial.print("receivedNum : ");
                Serial.println(receivedNum[_rxHeaderFrom]);
              }
            } // end of type check
          } // end of if (manager.recvData(manager.temp_buf))
        } // end of if (manager.available()) 데이터 수신
      }
      break;
    }
    case ROOMCON:
    {
      while(1)
      {
        manager2.SetReceive();
        if(manager2.available())
        {
          if(manager2.recvData(manager2.temp_buf))
          {
            _rxHeaderFrom = manager2.headerFrom();
            _rxHeaderTo = manager2.headerTo();
            _rxHeaderMaster = manager2.headerMaster();
            _rxHeaderType = manager2.headerType();
            
            if(_rxHeaderMaster == master_number)
            {
      
              //스캔 요청하라는 명령을 수신
              if(_rxHeaderType == SCAN_REQUEST_TO_RC)
              {
                //ACK 전송
                manager2.send(_thisAddress, _rxHeaderFrom, master_number, ACK, manager2.temp_buf, sizeof(manager2.temp_buf));
      
                //어떤 slave로부터 응답을 받았는지 check하기 위한 배열 초기화
                for(int j = 0;j <= num_of_slave;j++)
                  check_slave_scan_response[j] = false;
      
                //무선통신모듈 버퍼에 보낼 데이터(에어텍 내부통신 프로토콜 상태 요청 메시지) 저장
                for(int j = 0;j < 10;j++)
                  manager2.temp_buf[j] = roomcon_request[0][j];
      
                //slave개수도 보내줌
                manager2.temp_buf[10] = num_of_slave;
      
                //스캔 메시지 송수신
                send_and_wait_for_scan();
      
                //스캔응답을 보내지 않은 slave가 있다면 한 번 더 전체에 대하여 scan 요청
                for(int j = 0;j <= num_of_slave;j++)
                {
                  if(!check_slave_scan_response[j])
                  {
                   for(int j = 0;j < 10;j++)
                      manager2.temp_buf[j] = roomcon_request[0][j];
                      
                    manager2.temp_buf[10] = num_of_slave;
                    send_and_wait_for_scan();
                    break;
                  }
                }
      
                //내부 마스터에게 스캔이 끝났음을 알림
                manager2.sendToWaitAck(_thisAddress, 0x01, master_number, SCAN_FINISH_TO_MASTER, manager2.temp_buf, sizeof(manager2.temp_buf));              
              }
      
      
              
              // 게이트 웨이가 제어 요청을 보냄
              // (내부 마스터) -----CONTROL_TO_RC-----> (룸콘) -----GW_CONTROL_TO_SLAVE-----> (슬레이브, 내부 마스터)
              //                                                                                        (내부 마스터) -----CONTROL_RESPONSE_BROADCAST-----> (룸콘, 슬레이브)  
              //                                                                                                            룸콘에게는 ACK의 역할
              //
              
              else if(_rxHeaderType == CONTROL_TO_RC)
              {
                for(int i = 0;i < 10;i++)
                  slave_answer[0][i] = manager2.temp_buf[i];
                unsigned long temp_time = millis();
      
                //ACK 전송
                manager2.send(_thisAddress, _rxHeaderFrom, master_number, ACK, manager2.temp_buf, sizeof(manager2.temp_buf));
      
                // 내부 마스터에게 받은 상태대로 제어 요청을 보내기 위해서 실제 룸콘으로부터 제어 요청 데이터를 받기를 대기
                // (내부 마스터의 통신 노드) -----CONTROL_TO_RC(에어텍 통신 프로토콜의 헤더 : 0xA5)-----> (룸콘의 통신 노드) -----에어텍 통신 프로토콜의 헤더 : 0xA5------> (룸콘) -----에어텍 통신 프로토콜의 헤더 : 0xC5-----> (룸콘의 통신 노드)
                
                while(temp_time > rs485_time_arr[0]);  //rs485_time[x] : 실제 룸콘이 x번째 slave에게 보내야할 데이터를 룸콘의 통신 노드에게 전송한 시간을 저장
      
                for(int i = 0;i < 10;i++)
                  manager2.temp_buf[i] = roomcon_control[i]; 
      
                //broadcast로 slave들에게 제어요청 메시지 전송
                manager2.sendToWaitAck(_thisAddress, 0xFF, master_number, GW_CONTROL_TO_SLAVE, manager2.temp_buf, sizeof(manager2.temp_buf));
      
                rc_control = false;
              }
            }
          }
        }
        //게이트웨이로부터의 상태요청 및 제어요청 동안, 사용자가 직접 룸콘을 조작할 때의 제어요청 메시지는 미뤄짐
      
        //사용자가 룸콘을 조작
        if(rc_control && millis() - controllingTime > 5000)
        {
          if(!manager2.available())
          {
            for(int i = 0;i < 10;i++)
              manager2.temp_buf[i] = roomcon_control[i];
            manager2.sendToWaitAck(_thisAddress, 0xFF, master_number, RC_CONTROL_TO_SLAVE, manager2.temp_buf, sizeof(manager2.temp_buf), 70);
            rc_control = false;
          }
        }
      }
      break;
    }
    case SLAVE:
    {
      while(1)
      {
        for(int i = 0;i < 10;i++)
          outputData[i] = 0;
      
        // 무선 통신 모듈 수신 상태로 전환
        manager2.SetReceive();
      
        //패킷 수신
        if(manager2.available())
        {
          //수신 데이터 저장
          if(manager2.recvData(manager2.temp_buf))
          {
            Serial.println(millis());
      
            //헤더 저장
            _rxHeaderFrom = manager2.headerFrom();
            _rxHeaderTo = manager2.headerTo();
            _rxHeaderMaster = manager2.headerMaster();
            _rxHeaderType = manager2.headerType();
      
            //내가 속한 zone의 메시지일 경우
            if(_rxHeaderMaster == master_number)
            {
              //다른 FCU의 scan응답을 들음
              //현재 자신이 속한 존이 scan중임을 인식
              if(_rxHeaderType == SCAN_RESPONSE_TO_RC)
              {
                //주소에 따라 순차적으로 응답을 보냄
                /*이전 주소의 응답은 들었지만 자기 바로 앞의 FCU의 응답 신호를 못들었을 경우*/
                //  |--0번 FCU--|--1번 FCU--|--2번 FCU--|--3번 FCU--|
                //  3번 FCU가 1번FCU의 응답은 들었지만 2번 FCU의 응답을 못듣는 경우에 대비해서
                //  1번 FCU의 응답을 들은 시간을 저장해두고 2번 FCU가 보내야할 구간이 지나가면 자신의 응답을 보냄
                
                scanningAddress = _rxHeaderFrom;  //3번 FCU가 0번 FCU의 응답만 듣고, 1,2번의 응답을 듣지 못하는 경우 시간 계산을 위해서 직전에 들은 응답에 대한 주소를 저장
                scanTime = millis();  //다른 FCU의 응답을 들은 시간을 저장
                scanning = true;  //scan 중임을 표시
                
                for(int i = 0;i < 10;i++)
                  inputData[i] = manager2.temp_buf[i];
      
                 /*자기 바로 앞 주소 FCU의 응답을 들음*/
                if(_rxHeaderFrom == _thisAddress - 1)
                {
                  //실제 FCU가 인식하는 프로토콜에 맞게 변경(통신 노드와 FCU의 주소는 차이가 있음)
                  inputData[2] = _thisAddress - 1;
                  inputData[9] = 0x00;
                  
                  for(int i = 0;i < 9;i++)
                    inputData[9] ^= inputData[i];
      
                  //FCU에게 현재 상태를 물어봄
                  RS485_Write_Read();
                  
                  for(int i = 0;i < 10;i++)
                    manager2.temp_buf[i] = outputData[i];
      
                  //룸콘에게 상태응답을 보냄
                  manager2.send(_thisAddress, 0, master_number, SCAN_RESPONSE_TO_RC, manager2.temp_buf, sizeof(manager2.temp_buf));
                  scanning = false;
                  
                  while(millis() - scanTime < 350);
                }
                
                /*이전 주소의 응답은 들었지만 자기 바로 앞의 FCU의 응답 신호를 못들었을 경우*/
                //  |--0번 FCU--|--1번 FCU--|--2번 FCU--|--3번 FCU--|
                //  3번 FCU가 1번FCU의 응답은 들었지만 2번 FCU의 응답을 못듣는 경우에 대비해서
                //  1번 FCU의 응답을 들은 시간을 저장해두고 2번 FCU가 보내야할 구간이 지나가면 자신의 응답을 보냄
                        
                if(scanning && (millis() - scanningTime > (_thisAddress - scanningAddress) * 350))
                {
                  inputData[2] = _thisAddress - 1;
                  inputData[9] = 0x00;
                  
                  for(int i = 0;i < 9;i++)
                    inputData[9] ^= inputData[i];
                    
                  RS485_Write_Read();
                  
                  for(int i = 0;i < 10;i++)
                    manager2.temp_buf[i] = outputData[i];
                  
                  manager2.send(_thisAddress, 0, master_number, SCAN_RESPONSE_TO_RC, manager2.temp_buf, sizeof(manager2.temp_buf));
                  scanning = false;
                }
              }
      
              //상태변경 요청
              //1. GW_CONTROL_TO_SLAVE : (게이트웨이) -----> (외부 마스터) -----> (내부 마스터) -----> (룸콘) ---GW_CONTROL_TO_SLAVE---> (슬레이브, 내부 마스터 - broadcast)
              //2. RC_CONTROL_TO_SLAVE : (룸콘) ---RC_CONTROL_TO_SLAVE---> (슬레이브, 내부 마스터 - broadcast)
              //3. CONTROL_RESPONSE_BROADCAST : (룸콘) -----> (슬레이브, 내부 마스터 - broadcast)
              //                                                                     (내부마스터) ---CONTROL_RESPONSE_BROADCAST---> (룸콘, 슬레이브)  
              //                                                                      룸콘에게는 ACK의 역할
              //                                                                      다른 슬레이브에게는 RC_CONTROL_TO_SLAVE를 못받을경우에 대비
      
                        
              else if(_rxHeaderType == GW_CONTROL_TO_SLAVE || _rxHeaderType == RC_CONTROL_TO_SLAVE || _rxHeaderType == CONTROL_RESPONSE_BROADCAST)
              {
                for(int i = 0;i < 10;i++)
                  inputData[i] = manager2.temp_buf[i];
                
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
      break;
    }
  }  
}


//Interrupt handler
void receiveFromGW()
{  
  detachInterrupt(PA3);
  while(1)
  {
    // 한바이트씩 읽어서 배열에 저장
    if (Serial1.available()) 
    {
      if(indexFromG == 0)
      {
        byte temp = Serial1.read();
        if(temp == 0xD5 || temp == 0xC5)
        {
          receiveFromG[indexFromG++] = temp;
        }
      }
      else
        receiveFromG[indexFromG++] = Serial1.read();
    }

    // 10Bytes를 모두 수신하였다면
    if(indexFromG == 10)
    {
      group_id = receiveFromG[2];

      // 기존에 마스터로부터 받은 메시지를 실제 게이트웨이에게 전송해줌
      digitalWrite(SSerialTxControl, RS485Transmit);
      Serial1.write(master_answer[group_id], sizeof(group_id));
      Serial1.flush();
      digitalWrite(SSerialTxControl, RS485Receive);
  
      byte temp = 0;
      
      // checksum을 계산하여 오류여부를 보고 정상데이터만 해당하는 배열에 저장
      for(int i = 0;i < 9;i++)
          temp ^= receiveFromG[i];
  
      if(temp == receiveFromG[9])
      {
        // 상태요청
        if(receiveFromG[0] == 0xBD)
        {
            for(int i = 0;i < 10;i++)
            {
              gateway_request[group_id][i] = receiveFromG[i];
            }
        }
        
        // 제어 요청
        else if(receiveFromG[0] == 0xBE)
        {
          for(int i = 0;i < 10;i++)
          {
            control_message[group_id][i] = receiveFromG[i];
            controlRecvTime[group_id] = millis();
          }
          beControl[group_id] = true;
        } 
      }
      indexFromG = 0;
      break; 
    }
  }
  attachInterrupt(PA3, receiveFromGW, FALLING);
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
      rs485_time_arr[id] = millis();
      break; 
    }
  }
  attachInterrupt(PA3, receiveFromRC, FALLING);
  nvic_irq_set_priority(NVIC_USART2, 14);
  nvic_irq_set_priority(NVIC_USART1, 14);
}


void send_and_wait_for_scan()
{
  manager2.send(_thisAddress, 1, master_number, SCAN_REQUEST_TO_SLAVE, manager2.temp_buf, sizeof(manager2.temp_buf));
  scanningTime = millis();

  //7초동안 slave의 응답 메시지를 대기
  while(millis() - scanningTime < 7000)
  {
    manager2.SetReceive();
    if(manager2.available())
    {
      if(manager2.recvData(manager.temp_buf))
      {
        _rxHeaderFrom = manager2.headerFrom();
        _rxHeaderTo = manager2.headerTo();
        _rxHeaderMaster = manager2.headerMaster();
        _rxHeaderType = manager2.headerType();
        
        if(_rxHeaderMaster == master_number && _rxHeaderType == SCAN_RESPONSE_TO_RC)
        {
          byte temp = 0;

          //checksum계산
          for(int i = 0;i < 9;i++)
            temp ^= manager2.temp_buf[i];

          //checksum이 맞을 경우에 스캔 응답 데이터를 저장
          if(manager2.temp_buf[9] == temp)
          {
            for(int i = 0;i < 10;i++)
              slave_answer[_rxHeaderFrom - 1][i] = manager2.temp_buf[i];
          }
          check_slave_scan_response[_rxHeaderFrom] = true;
        }
      } // end of if(manager2.recvData(manager2.temp_buf))
    } // end of if(manager2.available())
  } // end of while(millis() - scanningTime < 7000)
}


bool RS485_Write_Read()
{
  uint8_t index = 0;
  byte buffer[10];

  for(int i = 0;i < 3;i++)
  {
    digitalWrite(SSerialTxControl, RS485Transmit);
    Serial1.write(inputData, sizeof(inputData));
    Serial1.flush();
    digitalWrite(SSerialTxControl, RS485Receive);
  
    rs485_loop_time = millis();
    Serial1.flush();
    while(rs485_loop_time + 3000 > millis())
    {
      if(Serial1.available())
        buffer[index++] = Serial1.read();
      if(index == 10)
        break;
    }
    Serial1.flush();
    if(device == IN_MASTER || device == SLAVE)
    {
      if(buffer[0] == 0xB5 || buffer[0] == 0xA5)
      {
        byte temp = 0;
        for(int i = 0;i < 9;i++)
          temp ^= buffer[i];
        if(buffer[9] == temp)
        {
          for(int i = 0;i < 10;i++)
            outputData[i] = buffer[i];
          return true;
        }
      } 
    }
    else
    {
      if(buffer[0] == 0xAD)
      {
        byte temp = 0;
        for(int i = 0;i < 9;i++)
          temp ^= buffer[i];
        if(buffer[9] == temp)
        {
          for(int i = 0;i < 10;i++)
            outputData[i] = buffer[i];
          return true;
        }
      }
    }
  }
  for(int i = 0;i < 10;i++)
    outputData[i] = 0;
  return false;


  if(index == 10)
  {
    for(int i = 0;i < 10;i++)
    {
      Serial.print(buffer[i], HEX);
      Serial.print(" ");
    }
  }
  Serial.println();
  Serial.print("num : ");
  Serial.println(index);
}

