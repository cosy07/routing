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
		while(tempIndex < 4) //master�� �޾ƾ��� �����Ͱ� �� 4����Ʈ(CA(type), �� �ּ�, �ܺ��ּ�, �����ּ�)
		{
			if(USART_GetFlagStatus(USART2, USART_FLAG_RXNE) == SET) // usart2�κ��� ������ ����
			{
				while(USART_GetFlagStatus(USART2, USART_FLAG_RXNE) == RESET); // �����͸� ��� ���� ������ ���
				initDataFromFCU[tempIndex++] = USART_ReceiveData(USART2); //������ �����͸� ������ ����
			}
		}
		if(initDataFromFCU[0] == 0xCA) // �´� �����͸� �����ߴٸ�
		{
				DGInit(initDataFromFCU[2], initDataFromFCU[3], initDataFromFCU[1] * 2); // ��, �ּ�, ä�� ��ȣ(�ܺ�ä���� �� * 2 -1, ����ä���� �� * 2)
				_thisAddress = DGgetThisAddress(); // main.c�� �������� �ʱ�ȭ
				master_number = DGgetMasterNumber();
			break; // ����� �����ߴٸ� while�� Ż��
		}
	}	
	
	
	while(1)
	{
		for(int i = 0;i < 10;i++)
			outputData[i] = 0;

		// ���� ��� ��� ���� ���·� ��ȯ
		DGSetReceive();

		//��Ű ����
		if(DGavailable())
		{
			//���� ������ ����
			if(DGrecvData(DGtemp_buf))
			{
				printf("%d\r\n", millis());

				//��� ����
				_rxHeaderFrom = DGheaderFrom();
				_rxHeaderTo = DGheaderTo();
				_rxHeaderMaster = DGheaderMaster();
				_rxHeaderType = DGheaderType();

				//���� ���� zone�� �޽����� ���
				if(_rxHeaderMaster == master_number)
				{
					//�ٸ� FCU�� scan������ ����
					//���� �ڽ��� ���� ���� scan ������ �ν�
					if(_rxHeaderType == SCAN_RESPONSE_TO_RC)
					{
						//�ּҿ� ���� ���������� ������ ����
						/*���� �ּ��� ������ ������� �ڱ� �ٷ� ���� FCU�� ���� ��ȣ�� ������� ���*/
						//  |--0�� FCU--|--1�� FCU--|--2�� FCU--|--3�� FCU--|
						//  3�� FCU�� 1��FCU�� ������ ������� 2�� FCU�� ������ ����� ��쿡 ����ؼ�
						//  1�� FCU�� ������ ���� �ð��� �����صΰ� 2�� FCU�� �������� ������ �������� �ڽ��� ������ ����
						
						scanningAddress = _rxHeaderFrom;  //3�� FCU�� 0�� FCU�� ���丸 ���, 1,2���� ������ ���� ���ϴ� ��� �ð� ����� ���ؼ� ������ ���� ���信 ���� �ּҸ� ����
						scanTime = millis();  //�ٸ� FCU�� ������ ���� �ð��� ����
						scanning = true;  //scan ������ ǥ��
						
						for(int i = 0;i < 10;i++)
							inputData[i] = DGtemp_buf[i];

						 /*�ڱ� �ٷ� �� �ּ��� FCU�� ������ ����*/
						if(_rxHeaderFrom == _thisAddress - 1)
						{
							//���� FCU�� �ν��ϴ� �������ݿ� �°� ����(��� ���� FCU�� �ּҴ� ���̰� ����)
							inputData[2] = _thisAddress - 1;
							inputData[9] = 0x00;
							
							for(int i = 0;i < 9;i++)
								inputData[9] ^= inputData[i];

							//FCU���� ���� ���¸� ���
							RS485_Write_Read();
							
							for(int i = 0;i < 10;i++)
								DGtemp_buf[i] = outputData[i];

							//���ܿ��� ���������� ����
							DGsend(_thisAddress, 0, master_number, SCAN_RESPONSE_TO_RC, DGtemp_buf, sizeof(DGtemp_buf));
							scanning = false;
							
							while(millis() - scanTime < 350);
						}
						
						/*���� �ּ��� ������ ������� �ڱ� �ٷ� ���� FCU�� ���� ��ȣ�� ������� ���*/
						//  |--0�� FCU--|--1�� FCU--|--2�� FCU--|--3�� FCU--|
						//  3�� FCU�� 1��FCU�� ������ ������� 2�� FCU�� ������ ����� ��쿡 ����ؼ�
						//  1�� FCU�� ������ ���� �ð��� �����صΰ� 2�� FCU�� �������� ������ �������� �ڽ��� ������ ����
										
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

					//���º��� ��û
					//1. GW_CONTROL_TO_SLAVE : (����Ʈ����) -----> (�ܺ� ������) -----> (���� ������) -----> (����) ---GW_CONTROL_TO_SLAVE---> (�����̺�, ���� ������ - broadcast)
					//2. RC_CONTROL_TO_SLAVE : (����) ---RC_CONTROL_TO_SLAVE---> (�����̺�, ���� ������ - broadcast)
					//3. CONTROL_RESPONSE_BROADCAST : (����) -----> (�����̺�, ���� ������ - broadcast)
					//                                                               (���θ�����) ---CONTROL_RESPONSE_BROADCAST---> (����, �����̺�)  
					//                                                                ���ܿ��Դ� ACK�� ����
					//                                                                �ٸ� �����̺꿡�Դ� RC_CONTROL_TO_SLAVE�� ������ ��쿡 ���

										
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