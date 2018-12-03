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

uint8_t master_number;

uint8_t _thisAddress;
uint8_t _rxHeaderFrom;
uint8_t _rxHeaderTo;
uint8_t _rxHeaderMaster;
uint8_t _rxHeaderType;

byte inputData[10];
byte outputData[10];
byte temp_outputData[10];
byte tempData[10] = {0xD5, 0x00, 0x02, 0x00, 0x03, 0x03, 0x19, 0x1E, 0x00, 0xD0};

bool scanning = false;
unsigned long rs485_loop_time;
unsigned long scanTime;
byte scanningAddress;

byte DGtemp_buf[20];

void RS485_Write_Read();

int main()
{

	SystemInit();
	/* System Clocks Configuration */
  RCC_Configuration();
  /* Configure the GPIO ports */
  GPIO_Configuration();
	
	USART2_Configuration();

	PrintfInit();
	MyTimerInit();
	
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
	
	
	while(1)
	{
		for(int i = 0;i < 10;i++)
			outputData[i] = 0;

		// 무선 통신 모듈 수신 상태로 전환
		DGSetReceive();

		//패키 수신
		if(DGavailable())
		{
			//수신 데이터 저장
			if(DGrecvData(DGtemp_buf))
			{
				printf("%d\r\n", millis());

				//헤더 저장
				_rxHeaderFrom = DGheaderFrom();
				_rxHeaderTo = DGheaderTo();
				_rxHeaderMaster = DGheaderMaster();
				_rxHeaderType = DGheaderType();

				//내가 속한 zone의 메시지일 경우
				if(_rxHeaderMaster == master_number)
				{
					//다른 FCU의 scan응답을 들음
					//현재 자신이 속한 존이 scan 중임을 인식
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
							inputData[i] = DGtemp_buf[i];

						 /*자기 바로 앞 주소의 FCU의 응답을 들음*/
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
								DGtemp_buf[i] = outputData[i];

							//룸콘에게 상태응답을 보냄
							DGsend(_thisAddress, 0, master_number, SCAN_RESPONSE_TO_RC, DGtemp_buf, sizeof(DGtemp_buf));
							scanning = false;
							
							while(millis() - scanTime < 350);
						}
						
						/*이전 주소의 응답은 들었지만 자기 바로 앞의 FCU의 응답 신호를 못들었을 경우*/
						//  |--0번 FCU--|--1번 FCU--|--2번 FCU--|--3번 FCU--|
						//  3번 FCU가 1번FCU의 응답은 들었지만 2번 FCU의 응답을 못듣는 경우에 대비해서
						//  1번 FCU의 응답을 들은 시간을 저장해두고 2번 FCU가 보내야할 구간이 지나가면 자신의 응답을 보냄
										
						if(scanning && (millis() - scanTime > (_thisAddress - scanningAddress) * 350))
						{
							inputData[2] = _thisAddress - 1;
							inputData[9] = 0x00;
							
							for(int i = 0;i < 9;i++)
								inputData[9] ^= inputData[i];
								
							RS485_Write_Read();
							
							for(int i = 0;i < 10;i++)
								DGtemp_buf[i] = outputData[i];
							
							DGsend(_thisAddress, 0, master_number, SCAN_RESPONSE_TO_RC, DGtemp_buf, sizeof(DGtemp_buf));
							scanning = false;
						}
					}

					//상태변경 요청
					//1. GW_CONTROL_TO_SLAVE : (게이트웨이) -----> (외부 마스터) -----> (내부 마스터) -----> (룸콘) ---GW_CONTROL_TO_SLAVE---> (슬레이브, 내부 마스터 - broadcast)
					//2. RC_CONTROL_TO_SLAVE : (룸콘) ---RC_CONTROL_TO_SLAVE---> (슬레이브, 내부 마스터 - broadcast)
					//3. CONTROL_RESPONSE_BROADCAST : (룸콘) -----> (슬레이브, 내부 마스터 - broadcast)
					//                                                               (내부마스터) ---CONTROL_RESPONSE_BROADCAST---> (룸콘, 슬레이브)  
					//                                                                룸콘에게는 ACK의 역할
					//                                                                다른 슬레이브에게는 RC_CONTROL_TO_SLAVE를 못받을 경우에 대비

										
					else if(_rxHeaderType == GW_CONTROL_TO_SLAVE || _rxHeaderType == RC_CONTROL_TO_SLAVE || _rxHeaderType == CONTROL_RESPONSE_BROADCAST)
					{
						for(int i = 0;i < 10;i++)
							inputData[i] = DGtemp_buf[i];
						
						inputData[2] = _thisAddress - 1;
						inputData[9] = 0x00;
						for(int i = 0;i < 9;i++)
							inputData[9] ^= inputData[i];
						
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

  /* Enable the USARTx */
	USART_Cmd(USARTxR, ENABLE);
	GPIO_ResetBits(GPIOA, GPIO_Pin_1);
}

void RS485_Write_Read()
{
	uint8_t num = 0, checkSum = 0;
	
	GPIO_SetBits(GPIOA, GPIO_Pin_1);

	for(int i = 0;i < 10;i++)
	{
		USART_SendData(USART2, tempData[i]);
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
			temp_outputData[num++] = USART_ReceiveData(USART2);
		}
		if(num == 10)
			break;
	}
	printf("-----receive-----\r\n");
	for(int i = 0;i < 10;i++)
		printf("%x ", temp_outputData[i]);
	printf("\r\n");
	for(int i = 0;i < 9;i++)
	{
		checkSum ^= temp_outputData[i];
	}
	if(checkSum == temp_outputData[9])
	{
		for(int i = 0;i < 10;i++)
		{
			outputData[i] = temp_outputData[i];
			printf("%x ", outputData[i]);
		}
	}
	printf("\r\n");
}