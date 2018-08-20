#include <Datagram_STM.h>
#define SSerialRX        PA3  //Serial Receive pin DI
#define SSerialTX        PA2  //Serial Transmit pin RO

#define SSerialTxControl PA1   //RS485 Direction control
#define RS485Transmit    HIGH
#define RS485Receive     LOW

ELECHOUSE_CC1120 cc1120;
Datagram manager(cc1120, 0x00);

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

uint8_t checkMasterNum = 1;
uint8_t address_i;
unsigned long startTime;
bool timeout = true;

uint8_t scanMasterNum = 1;
bool nextScan = true;

byte master_answer[32][10];
byte gateway_request[32][10];

bool beControl[32];
byte control_message[32][10];
byte receiveFromG[10];
uint8_t indexFromG = 0;
uint8_t group_id;

uint8_t turn = 0;
unsigned long scanTime = 0;

void receiveFromGW()
{  
  detachInterrupt(PA3);
  while(1)
  {
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
    if(indexFromG == 10)
    {
      group_id = receiveFromG[2];
      digitalWrite(SSerialTxControl, RS485Transmit);
      Serial1.write(master_answer[group_id], sizeof(group_id));
      Serial1.flush();
      digitalWrite(SSerialTxControl, RS485Receive);
  
      byte temp = 0;
      for(int i = 0;i < 9;i++)
          temp ^= receiveFromG[i];
  
      if(temp == receiveFromG[9])
      {
        if(receiveFromG[0] == 0xBD)
        {
            for(int i = 0;i < 10;i++)
            {
              gateway_request[group_id][i] = receiveFromG[i];
            }
        }
        else if(receiveFromG[0] == 0xBE)//제어요청
        {
          for(int i = 0;i < 10;i++)
          {
            control_message[group_id][i] = receiveFromG[i];
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

  if(scanTime + 600000 < millis())
  {
    //scanMasterNum의 
    //Slave or Roomcon 에러
    
    nextScan = true;
    if(++scanMasterNum > manager.master_num)
      scanMasterNum = 1;
  }nextScan = false;




  for(int beControlIndex = 0;beControlIndex <= 32;beControlIndex++) //마스터 번호(유선 통신) : 0 ~ 0x1F, 무선통신 주소 : 1 ~ 0x20
  {
    if(beControl[beControlIndex])
    {
      timeout = true;
      address_i = beControlIndex + 1;// convertToAddress(gatewayNumber, i, 0);

      for(int i = 0;i < 10;i++)
        manager.temp_buf[i] = control_message[beControlIndex][i];

      if(manager.checkReceive[address_i])
      {
        if (!manager.sendToWaitAck(_thisAddress, manager.getRouteTo(address_i)->next_hop, _thisAddress, address_i, CONTROL_TO_MASTER, NONE, NONE, NONE, NONE, manager.temp_buf, sizeof(manager.temp_buf)))
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
        else if(manager.getRouteTo(address_i)->hop == 1)
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
          manager.send(_thisAddress, _rxHeaderFrom, _thisAddress, _rxHeaderFrom, ACK, NONE, NONE, NONE, NONE, manager.temp_buf, sizeof(manager.temp_buf));
          beControl[beControlIndex] = false;
          timeout = false;
          for(uint8_t i = 0;i < 10;i++)
            master_answer[beControlIndex][i] = manager.temp_buf[i];
        }
        else if (manager.getRouteTo(address_i)->hop != 1)
        {
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
                else if (_rxHeaderType == CONTROL_RESPONSE_TO_GATEWAY)
                {
                  for(uint8_t i = 0;i < 10;i++)
                    master_answer[beControlIndex][i] = manager.temp_buf[i];
                  beControl[beControlIndex] = false;
                }
                break;
              }
            }
          }
          if (timeout)
          {
            Serial.println("timeout!");
            manager.G_find_error_node(address_i);
          }
        } 
      }
    }
    else
    {
      for(uint8_t i = 0;i < 10;i++)
        master_answer[beControlIndex][i] = 0;
    }
  }












  if(nextScan)//default 값 true
  {
    Serial.println("SCAN");
    Serial.println(scanMasterNum);
    address_i = scanMasterNum;
    if(manager.checkReceive[address_i])
    {
      if (!manager.sendToWaitAck(_thisAddress, manager.getRouteTo(address_i)->next_hop, _thisAddress, address_i, SCAN_REQUEST_TO_MASTER, NONE, NONE, NONE, NONE, manager.temp_buf, sizeof(manager.temp_buf)))
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
      else if(manager.getRouteTo(address_i)->hop == 1)
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
        
        manager.send(_thisAddress, _rxHeaderFrom, _thisAddress, _rxHeaderFrom, ACK, NONE, NONE, NONE, NONE, manager.temp_buf, sizeof(manager.temp_buf));
        timeout = false;
        nextScan = false;
        scanTime = millis();
      }
      else if (manager.getRouteTo(address_i)->hop != 1)
      {
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
            }
          }
        }
        if (timeout)
        {
          Serial.println("timeout!");
          manager.G_find_error_node(address_i);
        }
      } 
    }
  }







  
  timeout = true;
  address_i = checkMasterNum;// convertToAddress(gatewayNumber, i, 0);

  for(int i = 0;i < 10;i++)
  {
    manager.temp_buf[i] = gateway_request[checkMasterNum - 1][i];
  }

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
    else if (manager.getRouteTo(address_i)->hop != 1)
    {
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
              uint8_t reroutingAddr = manager.temp_buf[0];// word(manager.temp_buf[0], manager.temp_buf[1]);
              //deleteRouteTo(rerouting_addr);
              if (manager.unRecvCnt[reroutingAddr]++ > MAX_UN_RECV)
              {
                manager.checkReceive[reroutingAddr] = false;
                manager.parentMaster[reroutingAddr] = -1;
                manager.G_discoverNewPath(reroutingAddr, manager.getRouteTo(reroutingAddr)->hop);
                manager.unRecvCnt[reroutingAddr] = 0;
                manager.printTree();
              }
            }
            else
            {
              for(int i = 0;i < 10;i++)
                master_answer[checkMasterNum - 1][i] = manager.temp_buf[i];
                
              if (_rxHeaderType == CHECK_ROUTING_ACK)
              {
                for (uint8_t i = 0; i < _rxHeaderData; i++)
                {
                  manager.addRouteTo(manager.temp_buf[10 + i], _rxHeaderFrom, manager.Valid, manager.getRouteTo(_rxHeaderSource)->hop + 1);
                  manager.checkReceive[manager.temp_buf[10 + i]] = true;
                  manager.parentMaster[manager.temp_buf[10 + i]] = _rxHeaderSource;
                  manager.master_num++;
                  manager.printRoutingTable();
                }
              }
              else if (_rxHeaderType == CHECK_ROUTING_ACK_REROUTING)
              {
                int8_t temp_hopCount = manager.getRouteTo(_rxHeaderSource)->hop;
                manager.addRouteTo(_rxHeaderSource, _rxHeaderFrom, manager.Valid, _rxHeaderHop + 1);
                manager.changeNextHop(_rxHeaderSource, temp_hopCount - (_rxHeaderHop + 1));
                manager.parentMaster[_rxHeaderSource] = manager.temp_buf[10];
    
                manager.printRoutingTable();
                uint8_t cnt = 1;
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
                {
                  manager.sendToWaitAck(_thisAddress, manager.getRouteTo(manager.parentMaster[_rxHeaderSource])->next_hop, _thisAddress, manager.parentMaster[_rxHeaderSource], ROUTING_TABLE_UPDATE, cnt, NONE, NONE, NONE, manager.temp_buf, sizeof(manager.temp_buf));
                  delay(TIME_TERM * DEFAULT_RETRIES * manager.getRouteTo(_rxHeaderSource)->hop);
                }
              }
              else if(_rxHeaderType == SCAN_RESPONSE_TO_GATEWAY)
              {
                if(++scanMasterNum > manager.master_num)
                  scanMasterNum = 1;
                nextScan = true;
              }
            }
            break;
          }
        }
      }
      if (timeout)
      {
        Serial.println("timeout!");
        manager.G_find_error_node(address_i);
      }
    }
    else//rerouting 결과 혹은 원래 1hop인 애들
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
        uint8_t cnt = 1;
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
          manager.sendToWaitAck(_thisAddress, manager.getRouteTo(manager.parentMaster[_rxHeaderSource])->next_hop, _thisAddress, manager.parentMaster[_rxHeaderSource], ROUTING_TABLE_UPDATE, cnt, NONE, NONE, NONE, manager.temp_buf, sizeof(manager.temp_buf));
      }
      else if(_rxHeaderType == SCAN_RESPONSE_TO_GATEWAY)
      {
        if(++scanMasterNum > manager.master_num)
          scanMasterNum = 1;
        nextScan = true;
      }
    } 
  }
  else
  {
    for(uint8_t i = 0;i < 10;i++)
      master_answer[checkMasterNum - 1][i] = 0;
  }






  
  if(++checkMasterNum > manager.master_num)
    checkMasterNum = 1;
}
