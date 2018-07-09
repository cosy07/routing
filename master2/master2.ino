#include <Datagram_STM.h>
#include <DatagramForSlave.h>
#define SSerialRX        PA3  //Serial Receive pin DI
#define SSerialTX        PA2  //Serial Transmit pin RO

#define SSerialTxControl PA1   //RS485 Direction control
#define RS485Transmit    HIGH
#define RS485Receive     LOW

ELECHOUSE_CC1120 cc1120a;
ELECHOUSE_CC1120 cc1120b;
Datagram manager(cc1120a, 0x00000110);//외부통신
DatagramForSlave manager2(cc1120b, 0x00000110);//내부통신

uint8_t temp_buf[40];

uint16_t thisAddress;
uint16_t roomConAddress = thisAddress + 1;
uint16_t To;


unsigned long slave_scan_startTime;
bool slave_scaning = false;
int number_of_slave;
int slave_id;

uint16_t broadcast_address = thisAddress | 0x001F;
uint16_t gateway_address =  thisAddress & 0xFC00;
uint16_t rc_address =  thisAddress| 0x0001 ;

void RS485_Write_Read(uint8_t *write_buf,uint8_t *read_buf)
{
  uint8_t num, temp;
  digitalWrite(SSerialTxControl, RS485Transmit);
  for(int i = 0;i < 10;i++)
    Serial1.write(write_buf[i]);
  Serial1.flush();
  digitalWrite(SSerialTxControl, RS485Receive);
  while(1)
    {
        if(Serial1.available())
        {
            num = 0;
            while(1)
            {
              temp = Serial1.read();
              if(temp != -1)
              {
                read_buf[num] = temp;
                num++;
              }
              if(num == 10)
               break;
            }
            break;
        }
    }       
}
void setup() 
{
  Serial.begin(9600);
  pinMode(SSerialTxControl, OUTPUT);  
  digitalWrite(SSerialTxControl, RS485Receive);  // Init Transceiver
  Serial1.begin(9600);   // set the data rate 
  manager.init(11);
  manager.SetReceive();
  manager.FromMasterToGateway();
  manager2.init(22);
  manager2.SetReceive();
}
void loop() 
{
  manager.SetReceive();
  if(manager.available())
  {
    if(manager.recvData(temp_buf) && manager.headerTo() == thisAddress)
    {   
      //from, to, src, dst, type, data, flags, seqnum,hop, payload, size of payload
      manager.send(thisAddress, manager.headerFrom(), thisAddress, manager.headerFrom(), ACK, 0, 0, 0, 1,temp_buf, sizeof(temp_buf));
      if(manager.headerDestination() == thisAddress)
      {
        if (manager.headerType()  == CONTROL_MESSAGE )
          {         
            To = manager.headerFrom();
            uint16_t Relay_slave = thisAddress | (uint8_t)(number_of_slave/2+1);
                        
            RS485_Write_Read(temp_buf,temp_buf);
          
              // the master  brodcasts the control message to its slave node.
            if ( manager2.sendToWaitBroadcast(thisAddress, Relay_slave, thisAddress, broadcast_address, CONTROL_MESSAGE , 0, 0, 0, temp_buf , sizeof(temp_buf),TIME_HOP))
            {               
                manager.sendToWait(thisAddress, To , thisAddress, gateway_address, CONTROL_ACK, 0, 0, 0,1, temp_buf, sizeof(temp_buf),TIME_HOP);     
            } 
          }  // end of CONTROL_MESSAGE 
        if (manager.headerType()  == SCAN_MESSAGE )
          {
             To = manager.getRouteTo(gateway_address)->next_hop;
              RS485_Write_Read( temp_buf, temp_buf);
              manager.sendToWait(thisAddress, To , thisAddress, gateway_address, SCAN_ACK, 0, 0, 0, 1,temp_buf, sizeof(temp_buf),TIME_HOP);
          }
       if (manager.headerType()  == SCAN_SLAVE  )
         {
          slave_scaning = true ;
          slave_scan_startTime =  millis(); 
          slave_id=2;  //  the first slave number is 2.the romm con id is 1.
          manager2.M_handle_SCAN_SLAVE_message(slave_id,temp_buf);   
         }
      }
      else  // the received packet is not for me. the manager relays the packet to the next node.
      {
          To = manager.getRouteTo(manager.headerDestination())->next_hop;
          manager.sendToWaitAck(thisAddress, To, manager.headerSource(), manager.headerDestination(), manager.headerType(), manager.headerData(), manager.headerFlags(), manager.headerSeqNum(),manager.headerHop(), temp_buf, sizeof(temp_buf),TIME_HOP);
      }
    }
  }
  
   if ( ( slave_scaning == true)  && (millis() - slave_scan_startTime > 1000 )) // Every one second, the master continues to send the SCAN message to the slave nodes.  
    {
         manager2.M_handle_SCAN_SLAVE_message(slave_id,temp_buf);   
         slave_id ++;
        if (slave_id >number_of_slave + 1 )  // if the master finishes the SCAN message, then send SCAN_SLAVE_ACK to the gateway.
        { 
          To =   manager.getRouteTo(gateway_address)->next_hop  ;
          slave_id = 2;
          slave_scaning = false ;
          manager.sendToWait(thisAddress, To , thisAddress, gateway_address , SCAN_SLAVE_ACK , 0, 0, 0, 1,temp_buf, sizeof(temp_buf),TIME_HOP);       
        }
   }
   
    manager2.SetReceive();
  if(manager2.available())
  {
    if(manager2.recvData(temp_buf) )
    {
      if(manager2.headerType() == MAX_NUM_OF_SLAVE && manager2.headerFrom() == rc_address && manager2.headerTo() == thisAddress) //roomcon으로 부터 slave 개수를 받음
      {
        manager2.send(thisAddress, rc_address, thisAddress, rc_address, ACK, 0, 0, 0, 1,temp_buf, sizeof(temp_buf));
        number_of_slave = temp_buf[0];
      }
     if (manager2.headerType()  == CONTROL_MESSAGE && manager2.headerDestination() == broadcast_address)   //roomcon으로 부터 제어 메시지를 받음
        {         
            
            uint16_t Relay_slave = thisAddress | (uint8_t)(number_of_slave/2+1);
                        
            RS485_Write_Read(temp_buf,temp_buf);
          
              // the master  brodcasts the control message to its slave node.
            manager2.send(thisAddress, Relay_slave, thisAddress, broadcast_address, CONTROL_MESSAGE , 0, 0, 0, 1,temp_buf , sizeof(temp_buf));
            
       }  // end of CONTROL_MESSAGE 
         
     if (manager2.headerType()  == ERROR_MESSAGE )   //slave로 부터 에러 메시지를 받음
         manager2.M_handle_ERROR_message(manager2.headerFrom() );
    }  
  }   
}
