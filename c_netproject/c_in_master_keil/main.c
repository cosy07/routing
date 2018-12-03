#include "inDatagram_STM.h"

#define  USARTxR                    USART2
#define  RCC_APB1Periph_USARTxR     RCC_APB1Periph_USART2
#define  GPIOxR                     GPIOA
#define  RCC_APB2Periph_GPIOxR      RCC_APB2Periph_GPIOA
#define  GPIO_TxPin_R               GPIO_Pin_2
#define  GPIO_RxPin_R               GPIO_Pin_3

/* Private function prototypes -----------------------------------------------*/
void RCC_Configuration(void);
void GPIO_Configuration(void);
void USART2_Configuration(void);
void USART3_Configuration(void);
void NVIC_Configuration(void);
void RS485_Write_Read(void);

uint8_t master_number;
uint8_t num_of_slave;

uint8_t _thisAddress;
uint8_t _rxHeaderFrom;
uint8_t _rxHeaderTo;
uint8_t _rxHeaderMaster;
uint8_t _rxHeaderType;

byte inputData[10]; // RS485 송신 버퍼
byte outputData[10]; // RS485 수신 버퍼
byte state_request[10] = {0xD5, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00}; //에어텍 내부 통신 프로토콜 상태 요청 메시지

unsigned long rs485_loop_time;
unsigned long controlTime;
unsigned long scanTime;
bool timeout = true;

byte DGtemp_buf[20];

int main()
{
	SystemInit();
	/* System Clocks Configuration */
  RCC_Configuration();
  /* Configure the GPIO ports */
  GPIO_Configuration();
	
	USART2_Configuration();
	USART3_Configuration();
	
	PrintfInit();
	MyTimerInit();
	
		//주소 설정을 위해 처음에 FCU로부터 type이 CA인 값을 받음
	byte initDataFromFCU[4];
	byte tempIndex = 0;
	
	while(1)
	{
		while(tempIndex < 4) //master는 받아야할 데이터가 총 4바이트(CA(type), 층 주소, 외부주소, 내부주소)
		{
			if(USART_GetFlagStatus(USART2, USART_FLAG_RXNE) == SET) // usart2로부터 데이터 수신
			{
				while(USART_GetFlagStatus(USART2, USART_FLAG_RXNE) == RESET); // 데이터를 모두 받을 때까지 대기
				initDataFromFCU[tempIndex++] = USART_ReceiveData(USART2); //수신한 데이터를 변수에 저장
			}
		}
		if(initDataFromFCU[0] == 0xCA) // 맞는 데이터를 수신했다면
		{
				DGInit(initDataFromFCU[2], initDataFromFCU[3], initDataFromFCU[1] * 2); // 층, 주소, 채널 번호(외부채널은 층 * 2 -1, 내부채널은 층 * 2)
				_thisAddress = DGgetThisAddress(); // main.c의 전역변수 초기화
				master_number = DGgetMasterNumber();
			break; // 제대로 수신했다면 while문 탈출
		}
	}	
	
	
	
	
	DGSetReceive();

	while(1)
	{
		
		// 룸콘으로부터 제어요청
		// (룸콘) -----RC_CONTROL_TO_SLAVE-----> (슬레이브, 내부 마스터)
		//                                            (내부 마스터) -----CONTROL_RESPONSE_BROADCAST-----> (룸콘, 슬레이브)
		//																													 룸콘에게는 ACK의 역할, 
		//                                                           슬레이브에게는 혹시 RC_CONTROL_TO_SLAVE를 못받았을 경우에 대비
		DGSetReceive();

		if(DGavailable())
		{
			if(DGrecvData(DGtemp_buf))
			{
				printf("%d\r\n", millis());

				_rxHeaderFrom = DGheaderFrom();
				_rxHeaderTo = DGheaderTo();
				_rxHeaderMaster = DGheaderMaster();
				_rxHeaderType = DGheaderType();

				if(_rxHeaderMaster == master_number)
				{
					if(_rxHeaderType == RC_CONTROL_TO_SLAVE)
					{
						for(int i = 0;i < 10;i++)
							inputData[i] = DGtemp_buf[i];
						
						inputData[2] = 0x00;
						inputData[9] = 0x00;
						
						for(int i = 0;i < 9;i++)
							inputData[9] ^= inputData[i];
						
						DGsend(_thisAddress, 0xFF, master_number, CONTROL_RESPONSE_BROADCAST, DGtemp_buf, sizeof(DGtemp_buf));
						RS485_Write_Read();
					}
				}
			}
		}
	}
	
}

/**
  * @brief  Configures the different system clocks.
  * @param  None
  * @retval : None
  */

void RCC_Configuration(void)
{
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2 | RCC_APB1Periph_USART3, ENABLE);
}

/**
  * @brief  Configures the different GPIO ports.
  * @param  None
  * @retval : None
  */

void GPIO_Configuration(void)
{
  GPIO_InitTypeDef GPIO_InitStructure;

//#if defined USE_USART2	&& defined USE_STM3210B_EVAL
  /* Enable AFIO clock */
  //RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);

  /* Enable the USART2 Pins Software Remapping */
  //GPIO_PinRemapConfig(GPIO_Remap_USART2, ENABLE);
//#endif

	
	  /* Configure USARTx_Tx as alternate function push-pull */
  GPIO_InitStructure.GPIO_Pin = GPIO_TxPin_R;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
  GPIO_Init(GPIOxR, &GPIO_InitStructure);

  /* Configure USARTx_Rx as input floating */
  GPIO_InitStructure.GPIO_Pin = GPIO_RxPin_R;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
  GPIO_Init(GPIOxR, &GPIO_InitStructure);
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
  GPIO_Init(GPIOA, &GPIO_InitStructure);
	
	
	
	
	//for USART3
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
  GPIO_Init(GPIOB, &GPIO_InitStructure);

  /* Configure USARTx_Rx as input floating */
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
  GPIO_Init(GPIOB, &GPIO_InitStructure);

}

void USART2_Configuration(void)
{
	USART_InitTypeDef USART_InitStructure;
	/* USARTx configuration ------------------------------------------------------*/
  /* USARTx configured as follow:
        - BaudRate = 115200 baud  
        - Word Length = 8 Bits
        - One Stop Bit
        - No parity
        - Hardware flow control disabled (RTS and CTS signals)
        - Receive and transmit enabled
  */
  USART_InitStructure.USART_BaudRate = 9600;
  USART_InitStructure.USART_WordLength = USART_WordLength_8b;
  USART_InitStructure.USART_StopBits = USART_StopBits_1;
  USART_InitStructure.USART_Parity = USART_Parity_No ;
  USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
  USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
  
  /* Configure the USARTx */ 
	USART_Init(USARTxR, &USART_InitStructure);

  /* Enable the USARTx */
	USART_Cmd(USARTxR, ENABLE);
	GPIO_ResetBits(GPIOA, GPIO_Pin_1);
}

void USART3_Configuration(void)
{
	USART_InitTypeDef USART_InitStructure;
	/* USARTx configuration ------------------------------------------------------*/
  /* USARTx configured as follow:
        - BaudRate = 115200 baud  
        - Word Length = 8 Bits
        - One Stop Bit
        - No parity
        - Hardware flow control disabled (RTS and CTS signals)
        - Receive and transmit enabled
  */
  USART_InitStructure.USART_BaudRate = 9600;
  USART_InitStructure.USART_WordLength = USART_WordLength_8b;
  USART_InitStructure.USART_StopBits = USART_StopBits_1;
  USART_InitStructure.USART_Parity = USART_Parity_No ;
  USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
  USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
  
  /* Configure the USARTx */ 
	USART_Init(USART3, &USART_InitStructure);
	/* Enable USART3 Receive interrupts */
	USART_ITConfig(USART3, USART_IT_RXNE, ENABLE);
  /* Enable the USARTx */
	USART_Cmd(USART3, ENABLE);
}

/**
  * @brief  Interrupt handler
  * @param  None
  * @retval : None
  */
void USART3_IRQHandler(void)
{
	if (USART_GetITStatus(USART3, USART_IT_RXNE) != RESET) 
	{
		USART_ClearITPendingBit(USART3, USART_IT_RXNE);
		byte temp = USART_ReceiveData(USART3);
		
		// 제어요청일 경우
		// ************ 1단계 ************
		// (GW) -----> (외부 마스터) -----> (FCU)
		//                       -----> (내부 마스터)
		//  GW의 제어요청에 맞게 FCU 상태 변화 및 내부마스터에게 알려줌
		
		// ************ 2단계 ************
		// (내부 마스터) -----상태 요청-----> (FCU) -----상태 응답-----> (내부 마스터)
		//  내부 마스터가 FCU에게 현재 상태를 물어봄
		
		// ************ 3단계 ************
		// 자신의 상태를 룸콘에게 전송하여 그 zone이 GW의 제어요청대로 변경되도록 함
		// (내부 마스터) -----CONTROL_TO_RC-----> (룸콘) -----GW_CONTROL_TO_SLAVE-----> (슬레이브, 내부 마스터)
		//                                                                              (내부 마스터) -----CONTROL_RESPONSE_BROADCAST-----> (룸콘, 슬레이브)
		//                                                                                             룸콘에게는 ACK의 역할
		//																																							  						 다른 슬레이브에게는 RC_CONTROL_TO_SLAVE를 못받을 경우에 대비
		USART_ITConfig(USART3, USART_IT_RXNE, DISABLE);
		if(temp == INTERNAL_CONTROL)
		{
			for(int i = 0;i < 10;i++)
        inputData[i] = state_request[i];

      //FCU에게 현재 상태를 요청
      RS485_Write_Read();

      //현재 상태를 룸콘에게 전송하되, 상태응답이 아닌 변경요청(0xA5)으로 보냄
      outputData[0] = 0xA5;
      outputData[9] = 0;

      //checksum 계산 및 전송 버퍼에 데이터 저장
      for(int i = 0; i < 9;i++)
      {
        outputData[9] ^= outputData[i];
        DGtemp_buf[i] = outputData[i];
      }
      DGtemp_buf[9] = outputData[9];

      //룸콘에게 전송
      bool receiveACK = DGsendToWaitAck(_thisAddress, 0x00, master_number, CONTROL_TO_RC, DGtemp_buf, sizeof(DGtemp_buf), 2000);

      //룸콘으로부터 ACK을 받지 못함
      if(!receiveACK)
      {
				USART_SendData(USART3, INTERNAL_COM_FAIL);
				while(USART_GetFlagStatus(USART3, USART_FLAG_TC) == RESET);
      }
      else
      {
        controlTime = millis();
        timeout = true;
       
        // 룸콘이 slave에게 제어 요청을 보내길 기다림       
        while(millis() - controlTime < 3000)
        {

          //receive 상태로 전환
          DGSetReceive();

          if(DGavailable())
          {

            if(DGrecvData(DGtemp_buf))
            {

              _rxHeaderFrom = DGheaderFrom();
              _rxHeaderTo = DGheaderTo();
              _rxHeaderMaster = DGheaderMaster();
              _rxHeaderType = DGheaderType();

              if(_rxHeaderMaster == master_number)
              {
               if(_rxHeaderType == GW_CONTROL_TO_SLAVE)
                {
                  timeout = false;
                  DGsend(_thisAddress, 0xFF, master_number, CONTROL_RESPONSE_BROADCAST, DGtemp_buf, sizeof(DGtemp_buf));

                  //제어가 끝났음을 외부 마스터에게 알림
									USART_SendData(USART3, INTERNAL_CONTROL_FINISH);
									while(USART_GetFlagStatus(USART3, USART_FLAG_TC) == RESET);
                }
              }
            }
          }
        }
        
        if(timeout)
        {
          USART_SendData(USART3, INTERNAL_COM_FAIL);
					while(USART_GetFlagStatus(USART3, USART_FLAG_TC) == RESET);
        } 
      }
		}
		
		
		
		// 스캔요청일 경우
		//                                                                                     실제 스캔 요청                                   스캔 응답
		// (GW) -----> (외부 마스터) -----> (내부 마스터) -----SCAN_REQUEST_TO_RC-----> (룸콘) -----SCAN_REQUEST_TO_SLAVE-----> (내부 마스터) -----SCAN_RESPONSE_TO_RC-----> (룸콘)
		//                                                                                                                                                  (슬레이브 1) -----SCAN_RESPONSE_TO_RC----->(룸콘)
		//                                                                                                                                                                                        (슬레이브 2) -----SCAN_RESPONSE_TO_RC-----> (룸콘)
		
		else if(temp == INTERNAL_SCAN)
		{
			//룸콘에게 전송
      bool receiveACK = DGsendToWaitAck(_thisAddress, 0x00, master_number, SCAN_REQUEST_TO_RC, DGtemp_buf, sizeof(DGtemp_buf), 2000);

      //룸콘으로부터 ACK을 받지 못함
      if(!receiveACK)
      {
        //외부 마스터에게 시리얼 보냄
        USART_SendData(USART3, INTERNAL_COM_FAIL);
				while(USART_GetFlagStatus(USART3, USART_FLAG_TC) == RESET);
      }

      //전체 slave가 다 scan 될 때까지 or 일정시간까지 반복문
      
      scanTime = millis();
      timeout = true;
      while(millis() - scanTime < 10000)
      {

        //receive상태로 변환
        DGSetReceive();

        //패킷 수신
        if(DGavailable())
        {

          //수신된 데이터를 자신의 버퍼에 저장
          if(DGrecvData(DGtemp_buf))
          {

            //read header
            _rxHeaderFrom = DGheaderFrom();
            _rxHeaderTo = DGheaderTo();
            _rxHeaderMaster = DGheaderMaster();
            _rxHeaderType = DGheaderType();

            //내가 속한 zone으로부터의 패킷인 경우
            if(_rxHeaderMaster == master_number)
            {

              //룸콘으로부터 스캔요청을 수신
              if(_rxHeaderType == SCAN_REQUEST_TO_SLAVE && _rxHeaderTo == _thisAddress)
              {
                for(int i = 0;i < 10;i++)
                  inputData[i] = DGtemp_buf[i];
                
                num_of_slave = DGtemp_buf[10];
                RS485_Write_Read();
                //FCU에게 현재 상태를 물어봄
                
                for(int i = 0;i < 10;i++)
                  DGtemp_buf[i] = outputData[i];

                //룸콘에게 스캔 응답
                DGsend(_thisAddress, 0, master_number, SCAN_RESPONSE_TO_RC, DGtemp_buf, sizeof(DGtemp_buf));
              }

              //룸콘으로부터 SCAN_FINISH를 받음
              else if(_rxHeaderType == SCAN_FINISH_TO_MASTER)
              { 
                //ACK 전송
                DGsend(_thisAddress, _rxHeaderFrom, master_number, ACK, DGtemp_buf, sizeof(DGtemp_buf));
                
                //scan이 끝났음을 외부 마스터에게 알리기 전에 2초간 여유를 두고 모든 룸콘이 제어 메시지를 송신할 수 있도록 함
                //내부 통신의 경우 TX Power를 조절하여 다른 zone의 패킷을 수신하지 못하도록 하는 것도 좋을 것 같음
  
                timeout = false;
                controlTime = millis();
  
                //scan이 완료됐음을 외부 마스터에게 알림
								USART_SendData(USART3, INTERNAL_SCAN_FINISH);
								while(USART_GetFlagStatus(USART3, USART_FLAG_TC) == RESET);
              } // end of else if(_rxHeaderType == SCAN_FINISH_TO_MASTER) 
            }
          }
        }
      }
      if(timeout)
      {
        //외부 마스터에게 내부 통신 에러임을 알림(INTERNAL_COM_FAIL)
        USART_SendData(USART3, INTERNAL_COM_FAIL);
				while(USART_GetFlagStatus(USART3, USART_FLAG_TC) == RESET);
      }
		}
	}
	USART_ITConfig(USART3, USART_IT_RXNE, ENABLE);
}

void RS485_Write_Read()
{
	uint8_t num = 0, checkSum = 0;
	uint8_t receivedData[10];
	
	GPIO_SetBits(GPIOA, GPIO_Pin_1);

	for(int i = 0;i < 10;i++)
	{
		USART_SendData(USART2, inputData[i]);
		//while(USART_GetFlagStatus(USART2, USART_FLAG_TXE) == RESET);
		while(USART_GetFlagStatus(USART2, USART_FLAG_TC) == RESET);
	}

	GPIO_ResetBits(GPIOA, GPIO_Pin_1);
	
	rs485_loop_time = millis();
	while(rs485_loop_time + 3000 > millis())
	{
		if(USART_GetFlagStatus(USART2, USART_FLAG_RXNE))
		{
			//printf("hi");
			while(USART_GetFlagStatus(USART2, USART_FLAG_RXNE) == RESET);
			receivedData[num++] = USART_ReceiveData(USART2);
		}
		if(num == 10)
			break;
	}
	printf("-----receive-----\r\n");
	for(int i = 0;i < 10;i++)
		printf("%x ", receivedData[i]);
	printf("\r\n");
	for(int i = 0;i < 9;i++)
	{
		checkSum ^= receivedData[i];
	}
	if(checkSum == receivedData[9])
	{
		for(int i = 0;i < 10;i++)
		{
			outputData[i] = receivedData[i];
			printf("%x ", outputData[i]);
		}
	}
	printf("\r\n");
}