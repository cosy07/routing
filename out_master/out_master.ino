#include <Datagram_STM.h>

#define SSerialRX        PA3  //Serial Receive pin DI
#define SSerialTX        PA2  //Serial Transmit pin RO

#define SSerialTxControl PA1   //RS485 Direction control
#define RS485Transmit    HIGH
#define RS485Receive     LOW

ELECHOUSE_CC1120 cc1120;
Datagram manager(cc1120, 0x01);

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
unsigned long check_table_time = millis();
unsigned long check_rerouting_time = millis();


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
  
bool RS485_Write_Read(uint8_t *write_buf,uint8_t *read_buf)
{
  byte buffer[10];
  uint8_t index = 0;
  digitalWrite(SSerialTxControl, RS485Transmit);
  Serial1.write(write_buf, sizeof(write_buf));
  Serial1.flush();
  digitalWrite(SSerialTxControl, RS485Receive);
  while(1)
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
        read_buf[i] = buffer[i];
       return true;
    }
  }
  return false;
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
}


void loop() 
{
  if()//scan finish 명령을 내부 마스터로부터 받음
  {
    scanFinish = true;
  }
  if (check_table_time + 600000 < millis())//10분으로..
  {
    manager.checkRoutingTable();
    check_table_time = millis();
  }
  if (check_rerouting_time + 60000 < millis())//10분으로..
  {
    Serial.println("check_rerouting_time");
    check_rerouting_time = millis();
    int8_t max_receivedNum = 0;
    for (uint8_t i = 1; i < manager.master_num; i++)
    {
      if (max_receivedNum < receivedNum[i])
      {
        rerouting_candidate = i;
        max_receivedNum = receivedNum[i];
        receivedNum[i] = 0;
        rerouting = true;
      }
    }
  }
  manager.SetReceive();
  if (manager.available())
  {
    if (manager.recvData(manager.temp_buf))
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
      
      if (_rxHeaderTo == _thisAddress || _rxHeaderTo == masterBroadcastAddress)//routing 시 broadcast는 gateway에서 처음에 보내는 거 외에는 없어야함
      {
        manager.printRecvPacketHeader();
        if (_rxHeaderDestination != _thisAddress && _rxHeaderTo != masterBroadcastAddress)//relay상황
        {
          uint8_t temp_source = _rxHeaderSource;
          if (manager.getRouteTo(_thisAddress & 0x00) == NULL)
          {
            manager.printRoutingTable();
          }
          manager.send(_thisAddress, _rxHeaderFrom, _thisAddress, _rxHeaderFrom, ACK, NONE, NONE, NONE, NONE, manager.temp_buf, sizeof(manager.temp_buf));
          if (_rxHeaderType == REQUEST_MULTI_HOP_ACK || _rxHeaderType == REQUEST_DIRECT_ACK)
          {
            manager.addRouteTo(_rxHeaderSource, _rxHeaderFrom, manager.Valid, _rxHeaderHop + 1);
            manager.printRoutingTable();
          }
          else if (_rxHeaderType == CHECK_ROUTING_ACK && _rxHeaderData > 0)
          {
            for (uint8_t i = 0; i < _rxHeaderData; i++)
            {
              manager.addRouteTo(manager.temp_buf[10 + i], _rxHeaderFrom, manager.Valid, _rxHeaderHop + 1);
            }
          }
          else if (_rxHeaderType == CHECK_ROUTING_ACK_REROUTING)
          {
            manager.addRouteTo(_rxHeaderSource, _rxHeaderFrom, manager.Valid, _rxHeaderHop + 1);
          }
          else if(_rxHeaderType == ROUTING_TABLE_UPDATE)
          {
            for (uint8_t i = 1; i < _rxHeaderData; i++)
              manager.addRouteTo(manager.temp_buf[i], manager.temp_buf[0], manager.Valid, _rxHeaderHop + 1);
          }
          if (manager.getRouteTo(_rxHeaderDestination) == NULL
            || !manager.sendToWaitAck(_thisAddress, manager.getRouteTo(_rxHeaderDestination)->next_hop, _rxHeaderSource, _rxHeaderDestination, _rxHeaderType, NONE, NONE, NONE, _rxHeaderHop + 1, manager.temp_buf, sizeof(manager.temp_buf)))
          {
            if (temp_source == (_thisAddress & 0x00))
            {
              Serial.println("send NACK");
              manager.temp_buf[0] = manager.getRouteTo(_rxHeaderDestination)->next_hop;
              //manager.temp_buf[1] = (uint8_t)(0x00FF & getRouteTo(_rxHeaderDestination)->next_hop);
              manager.sendToWaitAck(_thisAddress, manager.getRouteTo(temp_source)->next_hop, _thisAddress, _thisAddress & 0x00, NACK, NONE, NONE, NONE, NONE, manager.temp_buf, sizeof(manager.temp_buf));
            }
          }
        }
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
          if (manager.temp_buf[0] != _thisAddress)
          {
            //아직 routing 안된 애 한테 보내줌
            Serial.println("received multihop request and send to unrouting Master");
            uint8_t realDst = manager.temp_buf[0];// word(manager.temp_buf[0], manager.temp_buf[1]);
            //from, to, src, dst, type, data, flags, seqnum, hop
            manager.send(_thisAddress, _rxHeaderFrom, _thisAddress, _rxHeaderFrom, ACK, NONE, NONE, NONE, NONE, manager.temp_buf, sizeof(manager.temp_buf));
            uint8_t cnt = 0;
            uint8_t type = _rxHeaderType;
            for (int j = 0; j < 1;j++)//DEFAULT_RETRIES; j++)
            {
              cnt = 0;
              Serial.print("retry : ");
              Serial.println(j);
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
                    j = DEFAULT_RETRIES;
                    manager.sendToWaitAck(_thisAddress, manager.getRouteTo(_thisAddress & 0x00)->next_hop, _rxHeaderSource, _thisAddress & 0x00, _rxHeaderType, 1, NONE, NONE, _rxHeaderHop + 1, manager.temp_buf, sizeof(manager.temp_buf));
                    break;
                  }
                }
              }
              delay(TIME_TERM);
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
            //routing 안 되어있던 애가 받음
            Serial.println("received multi hop request");
            if(_rxHeaderType == REQUEST_MULTI_HOP)
              manager.M_masterSendRoutingReply();
            else if(_rxHeaderType == REQUEST_DIRECT)
            {
              manager.addRouteTo(_rxHeaderSource, _rxHeaderFrom, manager.Valid, _rxHeaderHop + 1);
              manager.printRoutingTable();
              manager.temp_buf[0] = _rxHeaderFrom;
              //manager.temp_buf[1] = (uint8_t)(0x00FF & _rxHeaderFrom);
              //from, to, src, dst, type, data, flags, seqnum, hop
              for (uint8_t i = 0; i < R_MASTER_SEND_NUM; i++)
                manager.send(_thisAddress, _rxHeaderFrom, _thisAddress, _thisAddress & 0x00, REQUEST_DIRECT_ACK, NONE, NONE, NONE, NONE, manager.temp_buf, sizeof(manager.temp_buf));
            }
          }
        }
        ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////여기서부터 다시보기///////////////////////////////////////////////////////////
        else if (_rxHeaderType == CHECK_ROUTING)
        {
          receivedRequestNum = 0;
          if (manager.getRouteTo(_thisAddress & 0x00) != NULL)
          {



            
            for(int i = 0;i < 10;i++)
              inputData[i] = manager.temp_buf[i];
            while(!RS485_Write_Read(inputData, outputData));

            for(int i = 0;i < 10;i++)
              manager.temp_buf[i] = outputData[i];


            
            if(scanFinish)
            {
              manager.getRouteTo(_thisAddress & 0x00)->hop = _rxHeaderHop + 1;
              manager.sendToWaitAck(_thisAddress, manager.getRouteTo(_thisAddress & 0x00)->next_hop, _thisAddress, _thisAddress & 0x00, SCAN_RESPONSE_TO_GATEWAY, NONE, NONE, NONE, NONE, manager.temp_buf, sizeof(manager.temp_buf));
            }
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
            else if (rerouting || _rxHeaderFrom != manager.getRouteTo(_thisAddress & 0x00)->next_hop)
            {
              manager.temp_buf[10] = rerouting_candidate;
              if (manager.sendToWaitAck(_thisAddress, rerouting_candidate, _thisAddress, _thisAddress & 0x00, CHECK_ROUTING_ACK_REROUTING, NONE, NONE, NONE, NONE, manager.temp_buf, sizeof(manager.temp_buf)))
                manager.addRouteTo(_thisAddress & 0x00, rerouting_candidate, manager.Valid, _rxHeaderHop + 1);
              else
                rerouting_candidate = 0;
              rerouting = false;
            }
            else
            {
              manager.getRouteTo(_thisAddress & 0x00)->hop = _rxHeaderHop + 1;
              manager.sendToWaitAck(_thisAddress, manager.getRouteTo(_thisAddress & 0x00)->next_hop, _thisAddress, _thisAddress & 0x00, CHECK_ROUTING_ACK, NONE, NONE, NONE, NONE, manager.temp_buf, sizeof(manager.temp_buf));
            }
          }
          else
            manager.printRoutingTable();
        }
        else if (_rxHeaderType == NEW_NODE_REGISTER)
        {
          manager.addRouteTo(_rxHeaderFrom, _rxHeaderFrom, manager.Discovering, 1);
          manager.send(_thisAddress, _rxHeaderFrom, _thisAddress, _rxHeaderFrom, ACK, NONE, NONE, NONE, manager.getRouteTo(_thisAddress & 0x00)->hop, manager.temp_buf, sizeof(manager.temp_buf));
          newNode = true;
        }
        else if (_rxHeaderType == ROUTING_TABLE_UPDATE)
        {
          for (uint8_t i = 1; i < _rxHeaderData; i++)
            manager.addRouteTo(manager.temp_buf[i], manager.temp_buf[0], manager.Valid, _rxHeaderHop + 1);
          manager.send(_thisAddress, _rxHeaderFrom, _thisAddress, _rxHeaderFrom, ACK, NONE, NONE, NONE, NONE, manager.temp_buf, sizeof(manager.temp_buf));
        }
        



        
        else if(_rxHeaderType == SCAN_REQUEST_TO_MASTER)
        {
          manager.getRouteTo(_thisAddress & 0x00)->hop = _rxHeaderHop + 1;
          manager.sendToWaitAck(_thisAddress, manager.getRouteTo(_thisAddress & 0x00)->next_hop, _thisAddress, _thisAddress & 0x00, SCAN_REQUEST_ACK_FROM_MASTER, NONE, NONE, NONE, NONE, manager.temp_buf, sizeof(manager.temp_buf));
          
          //send scan to internal master to serial
          
        }
        
        else if(_rxHeaderType == CONTROL_TO_MASTER)
        {
          for(int i = 0;i < 10;i++)
            inputData[i] = manager.temp_buf[i];
          while(!RS485_Write_Read(inputData, outputData));
          for(int i = 0;i < 10;i++)
            manager.temp_buf[i] = outputData[i];
            
          //send control to internal master to serial
          
          if()//control 끝나면
          {
            manager.getRouteTo(_thisAddress & 0x00)->hop = _rxHeaderHop + 1;
            manager.sendToWaitAck(_thisAddress, manager.getRouteTo(_thisAddress & 0x00)->next_hop, _thisAddress, _thisAddress & 0x00, CONTROL_RESPONSE_TO_GATEWAY, NONE, NONE, NONE, NONE, manager.temp_buf, sizeof(manager.temp_buf));
          }
        }






        
      }














      
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
        }
      }

      
    }
  }
}
