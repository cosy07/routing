//내부 zone의 error는 관여하지 않음(알아서..)

#include <Datagram_STM.h>
#define SSerialRX        PA3  //Serial Receive pin DI
#define SSerialTX        PA2  //Serial Transmit pin RO

#define SSerialTxControl PA1   //RS485 Direction control
#define RS485Transmit    HIGH
#define RS485Receive     LOW

ELECHOUSE_CC1120 cc1120;
Datagram manager(cc1120, 0x00);


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

// 현재 scan 중인 master number(외부 scan) -> 내부 scan과 구분을 위해 check라는 용어 사용
uint8_t checkMasterNum = 1;
uint8_t address_i;
unsigned long startTime;

// multi-hop일 경우 timeout여부를 확인하기 위한 변수
bool timeout = true;

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

unsigned long scanTime = 0;
unsigned long controlTime = 0;



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

void setup()
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
}

void loop()
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



        /*CHECK_ROUTING_ACK_REROUTING 수신 후 ROUTING_TABLE_UPDATE*/
        /*uint8_t cnt = 1;
        manager.temp_buf[0] = _rxHeaderSource;
        for (uint8_t i = 1; i <= manager.master_num; i++)
        {
          if (manager.getRouteTo(i)->state == manager.Discovering)
          {
            manager.temp_buf[cnt++] = i;
            manager.getRouteTo(i)->state = manager.Valid;
          }
        }
        if(manager.parentMaster[_rxHeaderSource] != _thisAddress)
          manager.sendToWaitAck(_thisAddress, manager.getRouteTo(manager.parentMaster[_rxHeaderSource])->next_hop, _thisAddress, manager.parentMaster[_rxHeaderSource], ROUTING_TABLE_UPDATE, cnt, NONE, NONE, NONE, manager.temp_buf, sizeof(manager.temp_buf));*/
