#include <Datagram_STM.h>
#define SSerialRX        PA3  //Serial Receive pin DI
#define SSerialTX        PA2  //Serial Transmit pin RO

#define SSerialTxControl PA1   //RS485 Direction control
#define RS485Transmit    HIGH
#define RS485Receive     LOW

ELECHOUSE_CC1120 cc1120;
Datagram manager(cc1120, 0x00);
byte data[10];
byte scan_request_fromPC[32][10];
byte control_message[10][10];
byte maxOP = 0;
byte curOP = 0;
byte master_id = 1;
byte group_id;
byte master_status[32][10] = {0};
byte temp_buf[40];
byte unreceiveFirstTime[32] = {0};
byte unreceiveNum[32] = {0};
//byte SeqNum[33] = { 0 };//seqNum을 쓰는 이유 : 중복된 데이터 전송 때문에 일어나는 혼란을 막기위해서
                        //SeqNum 변수는 타입에 상관없이 목적지마다 한개씩 갖고있어야 한다.
uint16_t master_address;
uint8_t scaning_slave_group_ID =1;

uint16_t thisAddress = 0x0400;
uint16_t To;
unsigned long master_scan_startTime;
byte slave_scan_startTime;
byte maxNum;
int temp;
byte num = 0;
bool waiting_SCAN_SLAVE_ACK = 0;

void setup()
{
  Serial.begin(9600);
  pinMode(SSerialTxControl, OUTPUT);  
  digitalWrite(SSerialTxControl, RS485Receive);  // Init Transceiver
  Serial1.begin(9600);   // set the data rate 
  manager.init(15);
  manager.SetReceive();
  manager.FromGatewayToMaster();
  attachInterrupt(0, receiveFromPC, FALLING);
}
void receiveFromPC()
{
    if (Serial1.available()) 
    {  
      data[num] = Serial1.read();
      num++;
      if(num == 10)
      {
        if(data[0] == 0xBE)  // Control message
        {
            if(maxOP == curOP)
            {
              for(int i = 0;i < 10;i++)
                control_message[maxOP][i] = data[i];
            }
            else
            {
              maxOP++;
              for(int i = 0;i < 10;i++)
                control_message[maxOP][i] = data[i];
            }
            maxOP++;
            if(maxOP >= 32)
                maxOP = 0;
        }  // end of if(data[0] == 0xBE) 
        group_id = data[2]; //Group ID 
        for(int i = 0;i < 10;i++)
        {
          scan_request_fromPC[group_id][i] = data[i];
        }
        if(master_status[group_id][0] != 0)
        {
          digitalWrite(SSerialTxControl, RS485Transmit);
          for(int i = 0;i < 10;i++)
            Serial1.write(master_status[group_id]);//, sizeof(master_status[group_id]));
          Serial1.flush();
          digitalWrite(SSerialTxControl, RS485Receive);
        }
        else
        {
          data[0] = 0xAD;
          data[9] = 0x00;
          for(int i = 0;i < 9;i++)
          {
            data[9] ^= data[i];
          }
          digitalWrite(SSerialTxControl, RS485Transmit);
          for(int i = 0;i < 10;i++)
            Serial1.write(data[i])
          Serial1.flush();
          digitalWrite(SSerialTxControl, RS485Receive);
        }
        num = 0;
      }  // end of  if(num == 10)
    }
}
void loop()
{
    if (master_id >31)
        master_id =1;
    master_address = thisAddress | (uint16_t)(master_id<<5) ;
    
    manager.G_handle_CONTROL_message(&maxOP, &curOP, control_message, master_status);  // if there is any control message from PC, handle it.
                                                                                       // the gateway should wait until the transmission of CONTROL message succeeds.
                                                                                       
    if ( millis() - master_scan_startTime > 1000 ) // Every one second, the gateway continues to send the SCAN message to the master nodes.  
    {                                                 
        if(!manager.G_handle_SCAN_message(master_address, scan_request_fromPC[master_id], master_status)) // This is a scan message to scan all master nodes.
                                                                                                          //  The gateway send a SCAN message to a master node. If the gateway receives SCAN_ACK from the master, then return true   
                                                                                                          // if there is no SCAN_ACK, then increase the number of unreceived packet. 
                                                                                                          //  It will be used to discover a new path if the number of lost packet for the master is more than 5.
        {
            if(unreceiveNum[master_id] == 0)
            {
              unreceiveFirstTime[master_id] = millis()/1000;
            }
            unreceiveNum[master_id]++;
            if(millis() - unreceiveFirstTime[master_id] < 10)//10s
            {
              if(unreceiveNum[master_id] >= 5)
              {
                unreceiveNum[master_id] = 0;
                unreceiveFirstTime[master_id] = 0;
                manager.G_discoverNewPath(master_address);
              }
            }
            else
            {
              unreceiveNum[master_id] = 1;
              unreceiveFirstTime[master_id] = millis();
            }
        }
        master_id ++;
        master_scan_startTime= millis();
    }

    // gateway는 매스터 노드에게 해당 그룹에 있는 슬레이브의 상태를 스캔하도록 명령한다.
   //  slave 노드의 스캔 정보는 룸콘에 저장되며, 스캔이 종료되면 매스터 노드는 gateway에게 SCAN_SLAVE_ACK 메시지 전송함.
    if ( waiting_SCAN_SLAVE_ACK == 0 )   
    {
          To = manager.getRouteTo(scaning_slave_group_ID)->next_hop;
         manager.send(thisAddress, To, thisAddress, scaning_slave_group_ID, SCAN_SLAVE, 0, 0, 0, 1,&scaning_slave_group_ID, sizeof(scaning_slave_group_ID));
         slave_scan_startTime = millis() ;
         waiting_SCAN_SLAVE_ACK = 1;
    }

   // SCAN_SLAVE_ACK 메시지는 수시로 확인하며, 만약에 20초 동안에 완료가 안되면, 다시 매스터에게 재전송한다.
   manager.SetReceive();
   if(manager.available())
    {   
         if(manager.recvData(temp_buf) && manager.headerTo() == thisAddress && manager.headerType()  == SCAN_SLAVE_ACK )
             {
                scaning_slave_group_ID++;
                if (scaning_slave_group_ID >31)
                      scaning_slave_group_ID =1;
                waiting_SCAN_SLAVE_ACK = 0;
                slave_scan_startTime = millis()/1000 ;
             }
    }
  if ( millis() - slave_scan_startTime > 20 ) // 20 s is the waiting time to get the SCAN_SLAVE_ACK
    {
      waiting_SCAN_SLAVE_ACK = 0;
    }
}
