//routing table에 등록되는 노드 : gateway, 자식 노드들

#include <Datagram_STM.h>

#define SSerialRX        PA3  //Serial Receive pin DI
#define SSerialTX        PA2  //Serial Transmit pin RO

#define SSerialTxControl PA1   //RS485 Direction control
#define RS485Transmit    HIGH
#define RS485Receive     LOW

ELECHOUSE_CC1120 cc1120;
Datagram manager(cc1120, 0x00, 0x01);

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

uint8_t masterBroadcastAddress = 0xFF;

byte inputData[10];
byte outputData[10];

byte receivedFromInMaster;
bool timeout = true;

bool RS485_Write_Read()
{
  byte buffer[10];
  uint8_t index = 0;
  for(int i = 0;i < 3;i++)
  {
    digitalWrite(SSerialTxControl, RS485Transmit);
    Serial1.write(inputData, sizeof(inputData));
    Serial1.flush();
    digitalWrite(SSerialTxControl, RS485Receive);
  
    rs485_time = millis();
    while(rs485_time + 3000 > millis())
    {
      if(Serial1.available())
      {
        buffer[index++] = Serial1.read();
      }
      if(index == 10)
        break;
    }
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
  for(int i = 0;i < 10;i++)
    outputData[i] = 0;
  return false;
}


void setup() 
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
}


void loop() 
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
          Serial2.send(INTERNAL_SCAN);
          
        }
        
        else if(_rxHeaderType == CONTROL_TO_MASTER && manager.getRouteTo(_thisAddress & 0x00) != NULL)
        {
          byte temp;
          manater.send(_thisAddress, _rxHeaderFrom, _thisAddress, _rxHeaderFrom, ACK, NONE, NONE, NONE, NONE, temp, sizeof(temp));
          
          manager.getRouteTo(_thisAddress & 0x00)->hop = _rxHeaderHop + 1;
          
          for(int i = 0;i < 10;i++)
            inputData[i] = manager.temp_buf[i];
          
          RS485_Write_Read();
          
          for(int i = 0;i < 10;i++)
            manager.temp_buf[i] = outputData[i];
            
          //send control to internal master in serial
          Serial2.send(INTERNAL_CONTROL);

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
