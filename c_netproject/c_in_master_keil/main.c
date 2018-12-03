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

byte inputData[10]; // RS485 �۽� ����
byte outputData[10]; // RS485 ���� ����
byte state_request[10] = {0xD5, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00}; //������ ���� ��� �������� ���� ��û �޽���

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
	
		//�ּ� ������ ���� ó���� FCU�κ��� type�� CA�� ���� ����
	byte initDataFromFCU[4];
	byte tempIndex = 0;
	
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
	
	
	
	
	DGSetReceive();

	while(1)
	{
		
		// �������κ��� �����û
		// (����) -----RC_CONTROL_TO_SLAVE-----> (�����̺�, ���� ������)
		//                                            (���� ������) -----CONTROL_RESPONSE_BROADCAST-----> (����, �����̺�)
		//																													 ���ܿ��Դ� ACK�� ����, 
		//                                                           �����̺꿡�Դ� Ȥ�� RC_CONTROL_TO_SLAVE�� ���޾��� ��쿡 ���
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
		
		// �����û�� ���
		// ************ 1�ܰ� ************
		// (GW) -----> (�ܺ� ������) -----> (FCU)
		//                       -----> (���� ������)
		//  GW�� �����û�� �°� FCU ���� ��ȭ �� ���θ����Ϳ��� �˷���
		
		// ************ 2�ܰ� ************
		// (���� ������) -----���� ��û-----> (FCU) -----���� ����-----> (���� ������)
		//  ���� �����Ͱ� FCU���� ���� ���¸� ���
		
		// ************ 3�ܰ� ************
		// �ڽ��� ���¸� ���ܿ��� �����Ͽ� �� zone�� GW�� �����û��� ����ǵ��� ��
		// (���� ������) -----CONTROL_TO_RC-----> (����) -----GW_CONTROL_TO_SLAVE-----> (�����̺�, ���� ������)
		//                                                                              (���� ������) -----CONTROL_RESPONSE_BROADCAST-----> (����, �����̺�)
		//                                                                                             ���ܿ��Դ� ACK�� ����
		//																																							  						 �ٸ� �����̺꿡�Դ� RC_CONTROL_TO_SLAVE�� ������ ��쿡 ���
		USART_ITConfig(USART3, USART_IT_RXNE, DISABLE);
		if(temp == INTERNAL_CONTROL)
		{
			for(int i = 0;i < 10;i++)
        inputData[i] = state_request[i];

      //FCU���� ���� ���¸� ��û
      RS485_Write_Read();

      //���� ���¸� ���ܿ��� �����ϵ�, ���������� �ƴ� �����û(0xA5)���� ����
      outputData[0] = 0xA5;
      outputData[9] = 0;

      //checksum ��� �� ���� ���ۿ� ������ ����
      for(int i = 0; i < 9;i++)
      {
        outputData[9] ^= outputData[i];
        DGtemp_buf[i] = outputData[i];
      }
      DGtemp_buf[9] = outputData[9];

      //���ܿ��� ����
      bool receiveACK = DGsendToWaitAck(_thisAddress, 0x00, master_number, CONTROL_TO_RC, DGtemp_buf, sizeof(DGtemp_buf), 2000);

      //�������κ��� ACK�� ���� ����
      if(!receiveACK)
      {
				USART_SendData(USART3, INTERNAL_COM_FAIL);
				while(USART_GetFlagStatus(USART3, USART_FLAG_TC) == RESET);
      }
      else
      {
        controlTime = millis();
        timeout = true;
       
        // ������ slave���� ���� ��û�� ������ ��ٸ�       
        while(millis() - controlTime < 3000)
        {

          //receive ���·� ��ȯ
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

                  //��� �������� �ܺ� �����Ϳ��� �˸�
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
		
		
		
		// ��ĵ��û�� ���
		//                                                                                     ���� ��ĵ ��û                                   ��ĵ ����
		// (GW) -----> (�ܺ� ������) -----> (���� ������) -----SCAN_REQUEST_TO_RC-----> (����) -----SCAN_REQUEST_TO_SLAVE-----> (���� ������) -----SCAN_RESPONSE_TO_RC-----> (����)
		//                                                                                                                                                  (�����̺� 1) -----SCAN_RESPONSE_TO_RC----->(����)
		//                                                                                                                                                                                        (�����̺� 2) -----SCAN_RESPONSE_TO_RC-----> (����)
		
		else if(temp == INTERNAL_SCAN)
		{
			//���ܿ��� ����
      bool receiveACK = DGsendToWaitAck(_thisAddress, 0x00, master_number, SCAN_REQUEST_TO_RC, DGtemp_buf, sizeof(DGtemp_buf), 2000);

      //�������κ��� ACK�� ���� ����
      if(!receiveACK)
      {
        //�ܺ� �����Ϳ��� �ø��� ����
        USART_SendData(USART3, INTERNAL_COM_FAIL);
				while(USART_GetFlagStatus(USART3, USART_FLAG_TC) == RESET);
      }

      //��ü slave�� �� scan �� ������ or �����ð����� �ݺ���
      
      scanTime = millis();
      timeout = true;
      while(millis() - scanTime < 10000)
      {

        //receive���·� ��ȯ
        DGSetReceive();

        //��Ŷ ����
        if(DGavailable())
        {

          //���ŵ� �����͸� �ڽ��� ���ۿ� ����
          if(DGrecvData(DGtemp_buf))
          {

            //read header
            _rxHeaderFrom = DGheaderFrom();
            _rxHeaderTo = DGheaderTo();
            _rxHeaderMaster = DGheaderMaster();
            _rxHeaderType = DGheaderType();

            //���� ���� zone���κ����� ��Ŷ�� ���
            if(_rxHeaderMaster == master_number)
            {

              //�������κ��� ��ĵ��û�� ����
              if(_rxHeaderType == SCAN_REQUEST_TO_SLAVE && _rxHeaderTo == _thisAddress)
              {
                for(int i = 0;i < 10;i++)
                  inputData[i] = DGtemp_buf[i];
                
                num_of_slave = DGtemp_buf[10];
                RS485_Write_Read();
                //FCU���� ���� ���¸� ���
                
                for(int i = 0;i < 10;i++)
                  DGtemp_buf[i] = outputData[i];

                //���ܿ��� ��ĵ ����
                DGsend(_thisAddress, 0, master_number, SCAN_RESPONSE_TO_RC, DGtemp_buf, sizeof(DGtemp_buf));
              }

              //�������κ��� SCAN_FINISH�� ����
              else if(_rxHeaderType == SCAN_FINISH_TO_MASTER)
              { 
                //ACK ����
                DGsend(_thisAddress, _rxHeaderFrom, master_number, ACK, DGtemp_buf, sizeof(DGtemp_buf));
                
                //scan�� �������� �ܺ� �����Ϳ��� �˸��� ���� 2�ʰ� ������ �ΰ� ��� ������ ���� �޽����� �۽��� �� �ֵ��� ��
                //���� ����� ��� TX Power�� �����Ͽ� �ٸ� zone�� ��Ŷ�� �������� ���ϵ��� �ϴ� �͵� ���� �� ����
  
                timeout = false;
                controlTime = millis();
  
                //scan�� �Ϸ������ �ܺ� �����Ϳ��� �˸�
								USART_SendData(USART3, INTERNAL_SCAN_FINISH);
								while(USART_GetFlagStatus(USART3, USART_FLAG_TC) == RESET);
              } // end of else if(_rxHeaderType == SCAN_FINISH_TO_MASTER) 
            }
          }
        }
      }
      if(timeout)
      {
        //�ܺ� �����Ϳ��� ���� ��� �������� �˸�(INTERNAL_COM_FAIL)
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