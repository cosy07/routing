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
void NVIC_Configuration(void);

byte receiveFromR[10];
byte roomcon_control[10];
bool rc_control = false;

byte slave_answer[16][10];
byte roomcon_request[16][10];
unsigned long rs485_time[16];

uint8_t master_number;
uint8_t num_of_slave;

uint8_t _thisAddress;
uint8_t _rxHeaderFrom;
uint8_t _rxHeaderTo;
uint8_t _rxHeaderMaster;
uint8_t _rxHeaderType;

uint8_t indexFromR = 0;
uint8_t id;

unsigned long controllingTime;
unsigned long scanningTime;
bool check_slave_scan_response[16];

byte DGtemp_buf[20];

void receiveFromRC();

// scan??? ??? ??? ??
void send_and_wait_for_scan();

int main()
{
	DGInit(0x01, 0x00, 15); //zone의 외부 마스터 번호, 내 주소, 채널
	_thisAddress = DGgetThisAddress();
	DGSetReceive();

	for(int i = 0;i < 16;i++)
	{
		slave_answer[i][0] = 0xB5;
		slave_answer[i][1] = 0;
		slave_answer[i][2] = i;
		slave_answer[i][3] = 0;
		slave_answer[i][4] = 3;
		slave_answer[i][5] = 3;
		slave_answer[i][6] = 0x1A;
		slave_answer[i][7] = 0x21;
		slave_answer[i][8] = 0;
		slave_answer[i][9] = 0;
		for(int j = 0;j < 9;j++)
			slave_answer[i][9] ^= slave_answer[i][j];
	}
	
	master_number = DGgetMasterNumber();
	/* System Clocks Configuration */
	 RCC_Configuration();
	/* NVIC configuration */
	 NVIC_Configuration();
  /* Configure the GPIO ports */
	GPIO_Configuration();
	
	DGSetReceive();

	while(1)
	{
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

					//스캔 요청하라는 명령을 수신
					if(_rxHeaderType == SCAN_REQUEST_TO_RC)
					{
						//ACK 전송
						DGsend(_thisAddress, _rxHeaderFrom, master_number, ACK, DGtemp_buf, sizeof(DGtemp_buf));

						//어떤 slave로부터 응답을 받았는지 check하기 위한 배열 초기화
						for(int j = 0;j <= num_of_slave;j++)
							check_slave_scan_response[j] = false;

						//무선통신모듈 버퍼에 보낼 데이터(에어텍 내부통신 프로토콜 상태 요청 메시지) 저장
						for(int j = 0;j < 10;j++)
							DGtemp_buf[j] = roomcon_request[0][j];

						//slave개수도 보내줌
						DGtemp_buf[10] = num_of_slave;

						//스캔 메시지 송수신
						send_and_wait_for_scan();

						//스캔 응답을 보내지 않은 slave가 있다면 한 번 더 전체에 대하여 scan 요청
						for(int j = 0;j <= num_of_slave;j++)
						{
							if(!check_slave_scan_response[j])
							{
							 for(int j = 0;j < 10;j++)
									DGtemp_buf[j] = roomcon_request[0][j];
									
								DGtemp_buf[10] = num_of_slave;
								send_and_wait_for_scan();
								break;
							}
						}

						//내부 마스터에게 스캔이 끝났음을 알림
						DGsendToWaitAck(_thisAddress, 0x01, master_number, SCAN_FINISH_TO_MASTER, DGtemp_buf, sizeof(DGtemp_buf), 2000);              
					}

					
					// 게이트웨이가 제어요청을 보냄
					// (내부 마스터) -----CONTROL_TO_RC-----> (룸콘) -----GW_CONTROL_TO_SLAVE-----> (슬레이브, 내부 마스터)
					//                                                                               (내부 마스터) -----CONTROL_RESPONSE_BROADCAST-----> (룸콘, 슬레이브)  
					//                                                                                                   룸콘에게는 ACK의 역할
					//
					
					else if(_rxHeaderType == CONTROL_TO_RC)
					{
						for(int i = 0;i < 10;i++)
							slave_answer[0][i] = DGtemp_buf[i];
						unsigned long temp_time = millis();

						//ACK 전송
						DGsend(_thisAddress, _rxHeaderFrom, master_number, ACK, DGtemp_buf, sizeof(DGtemp_buf));

						// 내부 마스터에게 받은 상태대로 제어 요청을 보내기 위해서 실제 룸콘으로부터 제어 요청 데이터를 받기를 대기
						// (내부 마스터의 통신 노드) -----CONTROL_TO_RC(에어텍 통신 프로토콜의 헤더 : 0xA5)-----> (룸콘의 통신 노드) -----에어텍 통신 프로토콜의 헤더 : 0xA5------> (룸콘) -----에어텍 통신 프로토콜의 헤더 : 0xC5-----> (룸콘의 통신 노드)
						
						while(temp_time > rs485_time[0]);  //rs485_time[x] : 실제 룸콘이 x번째 slave에게 보내야할 데이터를 룸콘의 통신노드에게 전송한 시간을 저장

						for(int i = 0;i < 10;i++)
							DGtemp_buf[i] = roomcon_control[i]; 

						//broadcast로 slave들에게 제어요청 메시지 전송
						DGsendToWaitAck(_thisAddress, 0xFF, master_number, GW_CONTROL_TO_SLAVE, DGtemp_buf, sizeof(DGtemp_buf), 2000);

						rc_control = false;
					}
				}
			}
		}
		//게이트웨이로부터의 상태요청 및 제어요청 동안, 사용자가 직접 룸콘을 조작할 때의 제어요청 메시지는 미뤄짐

		//사용자가 룸콘을 조작
		if(rc_control && millis() - controllingTime > 5000)
		{
			if(!DGavailable())
			{
				for(int i = 0;i < 10;i++)
					DGtemp_buf[i] = roomcon_control[i];
				DGsendToWaitAck(_thisAddress, 0xFF, master_number, RC_CONTROL_TO_SLAVE, DGtemp_buf, sizeof(DGtemp_buf), 1000);
				rc_control = false;
			}
		}
	}
}

void send_and_wait_for_scan()
{
  DGsend(_thisAddress, 1, master_number, SCAN_REQUEST_TO_SLAVE, DGtemp_buf, sizeof(DGtemp_buf));
  scanningTime = millis();

  //10초동안 slave의 응답 메시지를 대기
  while(millis() - scanningTime < 10000)
  {
    DGSetReceive();
    if(DGavailable())
    {
      if(DGrecvData(DGtemp_buf))
      {
        _rxHeaderFrom = DGheaderFrom();
        _rxHeaderTo = DGheaderTo();
        _rxHeaderMaster = DGheaderMaster();
        _rxHeaderType = DGheaderType();
        
        if(_rxHeaderMaster == master_number && _rxHeaderType == SCAN_RESPONSE_TO_RC)
        {
          byte temp = 0;

          //checksum 계산
          for(int i = 0;i < 9;i++)
            temp ^= DGtemp_buf[i];

          //checksum이 맞을 경우에 스캔 응답 데이터를 저장
          if(DGtemp_buf[9] == temp)
          {
            for(int i = 0;i < 10;i++)
              slave_answer[_rxHeaderFrom - 1][i] = DGtemp_buf[i];
          }
          check_slave_scan_response[_rxHeaderFrom] = true;
        }
      } // end of if(DGrecvData(DGtemp_buf))
    } // end of if(DGavailable())
  } // end of while(millis() - scanningTime < 7000)
}

/**
  * @brief  Configures the different system clocks.
  * @param  None
  * @retval : None
  */

void RCC_Configuration(void)
{
  /* Setup the microcontroller system. Initialize the Embedded Flash Interface,  
     initialize the PLL and update the SystemFrequency variable. */
  //SystemInit();
    
  /* Enable GPIOx clock */
  //RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOx | RCC_APB2Periph_GPIOB | RCC_APB2Periph_AFIO, ENABLE);

/* Enable USARTx clocks */ 
  //RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE);
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE);
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
	/* Enable USART2 Receive interrupts */
  USART_ITConfig(USART2, USART_IT_RXNE, ENABLE);
  /* Enable the USARTx */
	USART_Cmd(USARTxR, ENABLE);
	GPIO_ResetBits(GPIOA, GPIO_Pin_1);
}

/**
  * @brief  Configures the nested vectored interrupt controller.
  * @param  None
  * @retval : None
  */
void NVIC_Configuration(void)
{
  NVIC_InitTypeDef NVIC_InitStructure;

  /* Configure the NVIC Preemption Priority Bits */  
  NVIC_PriorityGroupConfig(NVIC_PriorityGroup_0);

  /* Enable the USART2 Interrupt */
  NVIC_InitStructure.NVIC_IRQChannel = USART2_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);
}

/**
  * @brief  Interrupt handler
  * @param  None
  * @retval : None
  */
void USART2_IRQHandler(void)
{
	if (USART_GetITStatus(USART2, USART_IT_RXNE) != RESET) 
	{
		USART_ClearITPendingBit(USART2, USART_IT_RXNE);
		if(indexFromR == 0)
		{
			byte temp = USART_ReceiveData(USART2);
			if(temp == 0xD5 || temp == 0xC5)
			{
				receiveFromR[indexFromR++] = temp;
			}
		}
		else
			receiveFromR[indexFromR++] = USART_ReceiveData(USART2);
	}
	if(indexFromR == 10)
	{
		for(int i = 0;i < 10;i++)
		{
			printf("%x ", receiveFromR[i]);
		}
		printf("\r\n");
		
		id = receiveFromR[2];
		if(id > num_of_slave)
			num_of_slave = id;
		
		USART_ITConfig(USART2, USART_IT_RXNE, DISABLE);
		
		GPIO_SetBits(GPIOA, GPIO_Pin_1);
		
		for(int i = 0;i < 10;i++)
		{
			USART_SendData(USART2, slave_answer[id][i]);
			//while(USART_GetFlagStatus(USART2, USART_FLAG_TXE) == RESET);
			while(USART_GetFlagStatus(USART2, USART_FLAG_TC) == RESET);
		}
		
		GPIO_ResetBits(GPIOA, GPIO_Pin_1);
		
		USART_ITConfig(USART2, USART_IT_RXNE, ENABLE);
		
		
		byte temp = 0;
		for(int i = 0;i < 9;i++)
			temp ^= receiveFromR[i];
			
		if(temp == receiveFromR[9])
		{
			if(receiveFromR[0] == 0xD5)
			{
				for(int i = 0;i < 10;i++)
				{
					roomcon_request[id][i] = receiveFromR[i];
				}
			}
			else if(receiveFromR[0] == 0xC5)//????
			{
				for(int i = 0;i < 10;i++)
				{
					roomcon_control[i] = receiveFromR[i];
				}
				rc_control = true;
				controllingTime = millis();
			}
		} 
		indexFromR = 0;
		rs485_time[id] = millis();
	}
}