#include <DatagramForSlave.h>
#define SSerialRX        PA3  //Serial Receive pin DI
#define SSerialTX        PA2  //Serial Transmit pin RO

#define SSerialTxControl PA1   //RS485 Direction control
#define RS485Transmit    HIGH
#define RS485Receive     LOW

unsigned long startTime;
ELECHOUSE_CC1120 cc1120;
DatagramForSlave manager(cc1120,0x0101);  // driver, address
uint8_t temp_buf[10];

uint8_t slave_status[10];
uint16_t thisAddress;
bool state_change = false;

uint16_t   master_address = thisAddress & 0xFFE0;
uint16_t broadcast_address = master_address | 0x001F;
uint16_t rc_address =  master_address| 0x0001 ;

void RS485_Write_Read(uint8_t *write_buf,uint8_t *read_buf)
{
  uint8_t num, temp;
  digitalWrite(SSerialTxControl, RS485Transmit);
  for(int i = 0;i < 10;i++)
    Serial1.write(write_buf[i]);//, sizeof(write_buf));
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
  manager.init(22);
  startTime =  millis(); 
  manager.SetReceive();
  pinMode(SSerialTxControl, OUTPUT);  
  digitalWrite(SSerialTxControl, RS485Receive);  // Init Transceiver
  Serial1.begin(9600);   // set the data rate 
}
void loop()
{

  if ( millis() - startTime > 1000 ) // every one second, the slave node monitors the slave FCU through RS485
   {     
          state_change = false;
          startTime =  millis(); 
          
          uint8_t data[10] = {0};   // set the status request format
          data[0] = 0xAD;
          data[2] = (uint8_t)(thisAddress & 0x001F); // set the id from the address;
          data[9] = 0x00;
          for(int i = 0;i < 9;i++)
          {
            data[9] ^= data[i];
          }

         RS485_Write_Read( data, temp_buf);  // send the status request, and get the new state
         
         for(int i = 0;i < 10;i++)
          {
            if (slave_status[i] != temp_buf[i])
                state_change = true;
            slave_status[i] = temp_buf[i];
          }
          if ( state_change == true)  // if there is a change on the status, then report it to the room con             
                manager.sendToWait(thisAddress, rc_address , thisAddress, rc_address, SCAN_UPDATE, 0, 0, 0, 1,slave_status, sizeof(slave_status),TIME_HOP);   
   }
   
  manager.SetReceive();
  if(manager.available())
  {
    if(manager.recvData(temp_buf))  // 
    {
      if( manager.headerTo() == thisAddress && manager.headerType() == SCAN_SLAVE && manager.headerFrom() == master_address)    // master node로 부터 SCAN 메시지 수신
      {
        RS485_Write_Read( temp_buf, temp_buf);
        manager.send(thisAddress,master_address, thisAddress, master_address, SCAN_SLAVE_ACK, 0, 0, 0, 1, temp_buf, sizeof(temp_buf));  //ACK 안보내도 됨
      }
      if(manager.headerType() == CONTROL_MESSAGE && manager.headerDestination() == broadcast_address )  //   CONTROL_MESSAGE 메시지 수신
       {
          RS485_Write_Read( temp_buf, temp_buf);
          if(manager.headerTo() == thisAddress && manager.headerFrom() != thisAddress)
            { 
              delay(400);
              manager.send(thisAddress, broadcast_address, thisAddress, broadcast_address, CONTROL_MESSAGE , 0, 0, 0, 1, temp_buf , sizeof(temp_buf));  //  만약 노드가 브로드캐스트 relay 노드이면, 브로드캐스트 전송
            }
        }  //   end of CONTROL_MESSAGE 메시지 수신
    }  // end of recvData
  }  // end of available()
}
