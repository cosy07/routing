//<룸콘>
//하는 일 : 스캔, 사용자가 룸콘 조작시 FCU들에게 명령
//          master로 부터 control 메시지 수신시 룸콘 업데이트


#include <DatagramForSlave.h>
#define SSerialRX        PA3  //Serial Receive pin DI
#define SSerialTX        PA2  //Serial Transmit pin RO

#define SSerialTxControl PA1   //RS485 Direction control
#define RS485Transmit    HIGH
#define RS485Receive     LOW

ELECHOUSE_CC1120 cc1120;
DatagramForSlave manager(cc1120, 0x0110);
uint16_t thisAddress;
uint16_t   master_address = thisAddress & 0xFFE0;
uint16_t broadcast_address = thisAddress | 0x001F;
 
byte data[10];
byte array_slave_status[16][10] = {0};
byte temp_buf[10];
unsigned long startTime = 0;
unsigned long last_control_message_time = 0;
byte number_of_slave;

uint8_t sendSeqNum = 0;

bool receiveScanAck = false;
bool received_control_message_flag = false;
 
int temp;
int slave_id;
byte num = 0;
int retry = 3;

uint8_t control_message[10][10];
uint8_t temp_control_message[10];
byte maxOP = 0;
byte curOP = 0;
bool change_number_of_slave = false;
bool receiveBroadcast = false;

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
  pinMode(SSerialTxControl, OUTPUT);  
  digitalWrite(SSerialTxControl, RS485Receive);  // Init Transceiver
  Serial1.begin(9600);   // set the data rate 
  manager.init(22);
  manager.SetReceive(); 
  attachInterrupt(0, receiveFromRC, FALLING);
}
void receiveFromRC()
{
  if (Serial1.available()) 
  {
    data[num] = Serial1.read();
    num++;
    if(num == 10)
    {
      slave_id= data[2];
      if(slave_id > number_of_slave)
      {
        number_of_slave = slave_id;
        change_number_of_slave = true;
      }
     if(data[0] == 0xC5)//명령이 오면  임시 control_message 배열에 저장한다. 
      {         
         for(int i = 0;i < 10;i++)
              temp_control_message[i] = data[i];
         last_control_message_time = millis();     
         received_control_message_flag = true;
      }       
      
      if(array_slave_status[slave_id][0] != 0)
      {
        digitalWrite(SSerialTxControl, RS485Transmit);
        for(int i = 0;i < 10;i++)
          Serial1.write(array_slave_status[slave_id][i]);//, sizeof(array_slave_status[slave_id]));
        Serial1.flush();
        digitalWrite(SSerialTxControl, RS485Receive);
      }
      else
      {
        data[0] = 0xB5;
        data[9] = 0x00;
        for(int i = 0;i < 9;i++)
        {
          data[9] ^= data[i];
        }
        digitalWrite(SSerialTxControl, RS485Transmit);
        for(int i = 0;i < 10;i++)
          Serial1.write(data[i]);//, sizeof(data));
        Serial1.flush();
        digitalWrite(SSerialTxControl, RS485Receive);
      }
      num = 0;
    }
  }
}
void loop()
{
 if (change_number_of_slave== true)  // master에게 slave의 갯수를 전송함.
  {
        manager.sendToWait(thisAddress, master_address, thisAddress, master_address, MAX_NUM_OF_SLAVE, 0, 0, 0, 1, &number_of_slave, sizeof(number_of_slave),TIME_HOP);
        change_number_of_slave = false; 
   }        
 if( received_control_message_flag == true && millis() - last_control_message_time > 1000)
 //  room con의 명령어 메시지는 1초 지난 다음에 명령어 메시지를 저장한다. ( 온도설정시 목표 온도까지 설정하기 위한 버튼을 눌렀을 경우, 메시지 전송을 억제하기위함)
    {
       if(maxOP == curOP)
          {
            for(int i = 0;i < 10;i++)
              control_message[maxOP][i] = temp_control_message[i];
          }
        else
          {
            maxOP++;
            for(int i = 0;i < 10;i++)
              control_message[maxOP][i] = temp_control_message[i];
          }
        maxOP++;
        if(maxOP >= 10)
            maxOP = 0;       
        received_control_message_flag == false;
    }
 
  
  manager.SetReceive();
  if(manager.available())
  {
    if(manager.recvData(temp_buf) )
        if ( manager.headerTo() == thisAddress || manager.headerTo() == master_address ||  manager.headerTo() == broadcast_address ) 
        {
            if ( manager.headerType() == SCAN_SLAVE_ACK || manager.headerType() == ERROR_MESSAGE || manager.headerType() == SCAN_UPDATE)
                for(int k = 0;k < 10;k++)
                     array_slave_status[temp_buf[2]][k] = temp_buf[k];   
    
           if ( manager.headerType() == ERROR_MESSAGE  && manager.headerTo() == thisAddress )                       
               manager.send(thisAddress, manager.headerFrom() , thisAddress, manager.headerFrom(), ACK, 0, 0, 0, 1,&sendSeqNum, sizeof(sendSeqNum));   
              
                
            // slave가 상태의 변화로 인한 업데이트 발생시, 전송함. 이를 수신하여 저장하며, ACK 전송
           if ( manager.headerType() == SCAN_UPDATE  && manager.headerTo() == thisAddress )          
               {
                  RS485_Write_Read( temp_buf, temp_buf);
                  manager.send(thisAddress, manager.headerFrom() , thisAddress, manager.headerFrom(), ACK, 0, 0, 0, 1,&sendSeqNum, sizeof(sendSeqNum));   
               }
               
           // master가 제어 메시지를 룸콘 및 slave 에게  전송함. 이를 수신하여 룸콘 설정값   변경.
           if ( manager.headerType() == CONTROL_MESSAGE  && manager.headerTo() == broadcast_address && manager.headerFrom() == master_address)         
              {              
                  RS485_Write_Read( temp_buf, temp_buf);
              }
      
            
      }
  }    
  if (maxOP != curOP )
  {     // if there is any control message from RC, handle it.     
      startTime = millis();
      for (int i = 0; i < retry; i++)  // the room con broadcasts a control message, and wait a broadcast from any slave node. If not, then retransmit it.
       {
           manager.send(thisAddress, broadcast_address,thisAddress,broadcast_address, CONTROL_MESSAGE, 0, 0, 0, 1, control_message[curOP] , sizeof(control_message[curOP]));   
           while   (millis() - startTime < TIME_HOP*2)                    
          {
              manager.SetReceive();
              if (manager.available() && manager.recvData(temp_buf))
               {
                    if ( manager.headerType() == CONTROL_MESSAGE  && manager.headerTo() == broadcast_address  && manager.headerFrom() != master_address )
                       {
                          receiveBroadcast = true;
                          for(int k = 0;k < 10;k++)
                              array_slave_status[temp_buf[2]][k] = temp_buf[k];    
                          break;
                       }
                    if ( manager.headerType() == CONTROL_MESSAGE  && manager.headerTo() == broadcast_address && manager.headerFrom() == master_address)         
                      {              
                          RS485_Write_Read( temp_buf, temp_buf);
                      }
               }
          }
          if (receiveBroadcast) // if there is acontrol message from any slave node, then  store that status.
                  break;
       }              
      curOP++;
      if (curOP >= 10)
          curOP = 0;
  }     
}
