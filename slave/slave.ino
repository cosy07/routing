//slave가 받는 메시지 : 1. 상태 요청 2. 제어 요청
#include <InDatagram_STM.h>

#define SSerialRX        PA3  //Serial Receive pin DI
#define SSerialTX        PA2  //Serial Transmit pin RO

#define SSerialTxControl PA1   //RS485 Direction control
#define RS485Transmit    HIGH
#define RS485Receive     LOW

IN_ELECHOUSE_CC1120 cc1120;
InDatagram manager(cc1120, 0x01, 0x03);

uint8_t master_number;

uint8_t _thisAddress;
uint8_t _rxHeaderFrom;
uint8_t _rxHeaderTo;
uint8_t _rxHeaderMaster;
uint8_t _rxHeaderType;

byte inputData[10];
byte outputData[10];
byte tempData[10] = {0xD5, 0x00, 0x02, 0x00, 0x03, 0x03, 0x19, 0x1E, 0x00, 0xD0}; //for test

bool scanning = false;
unsigned long rs485_loop_time;
unsigned long scanTime;
byte scanningAddress;
  
void RS485_Write_Read()
{
  uint8_t num = 0;
  
  /*for(int i = 0;i < 10;i++)
  {
    Serial.print(inputData[i], HEX);
    Serial.print(" ");
  }
  Serial.println();*/
  
  digitalWrite(SSerialTxControl, RS485Transmit);
  Serial1.write(tempData, sizeof(tempData));
  Serial1.flush();
  digitalWrite(SSerialTxControl, RS485Receive);

  rs485_loop_time = millis();
  Serial1.flush();
  while(rs485_loop_time + 3000 > millis())
  {
    if(Serial1.available())
      outputData[num++] = Serial1.read();
    if(num == 10)
      break;
  }
  Serial1.flush();
  if(num == 10)
  {
    for(int i = 0;i < 10;i++)
    {
      Serial.print(outputData[i], HEX);
      Serial.print(" ");
    }
  }
  Serial.println();
  Serial.print("num : ");
  Serial.println(num);
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
}


void loop() 
{
  for(int i = 0;i < 10;i++)
    outputData[i] = 0;
    
  manager.SetReceive();
  if(manager.available())
  {
    if(manager.recvData(manager.temp_buf))
    {
      Serial.println(millis());
      _rxHeaderFrom = manager.headerFrom();
      _rxHeaderTo = manager.headerTo();
      _rxHeaderMaster = manager.headerMaster();
      _rxHeaderType = manager.headerType();

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
            inputData[i] = manager.temp_buf[i];

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
              manager.temp_buf[i] = outputData[i];

            //룸콘에게 상태응답을 보냄
            manager.send(_thisAddress, 0, master_number, SCAN_RESPONSE_TO_RC, manager.temp_buf, sizeof(manager.temp_buf));
            scanning = false;
          }
        }

        //상태변경 요청
        //1. GW_CONTROL_TO_SLAVE : (게이트웨이) -----> (외부 마스터) -----> (내부 마스터) -----> (룸콘) ---/////^^/////---> (슬레이브, 내부 마스터 - broadcast)
        //2. RC_CONTROL_TO_SLAVE : (룸콘) ---/////^^/////---> (슬레이브, 내부 마스터 - broadcast)
        //3. CONTROL_RESPONSE_BROADCAST : (룸콘) -----> (슬레이브, 내부 마스터 - broadcast)
        //                                                                     (내부마스터) ---/////^^/////---> (룸콘, 슬레이브)  
        //                                                                      룸콘에게는 ACK의 역할
        //                                                                      다른 슬레이브에게는 RC_CONTROL_TO_SLAVE를 못받을경우에 대비

                  
        else if(_rxHeaderType == GW_CONTROL_TO_SLAVE || _rxHeaderType == RC_CONTROL_TO_SLAVE || _rxHeaderType == CONTROL_RESPONSE_BROADCAST)
        {
            for(int i = 0;i < 10;i++)
              inputData[i] = manager.temp_buf[i];
            
            inputData[2] = _thisAddress - 1;
            inputData[9] = 0x00;
            for(int i = 0;i < 9;i++)
              inputData[9] ^= inputData[i];
            
            RS485_Write_Read();

        }
      }
    }
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
      manager.temp_buf[i] = outputData[i];
    
    manager.send(_thisAddress, 0, master_number, SCAN_RESPONSE_TO_RC, manager.temp_buf, sizeof(manager.temp_buf));
    scanning = false;
  }
}
