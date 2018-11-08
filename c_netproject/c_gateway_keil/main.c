//���� zone�� error�� �������� ����(�˾Ƽ�..)

#include "Datagram_STM.h"

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


uint8_t     _thisAddress;
uint8_t     _rxHeaderTo;
uint8_t     _rxHeaderFrom;
uint8_t     _rxHeaderSource;
uint8_t     _rxHeaderDestination;
uint8_t     _rxHeaderType;
uint8_t     _rxHeaderData;
uint8_t     _rxHeaderFlags;
uint8_t     _rxHeaderSeqNum;
uint8_t     _rxHeaderHop;

//���� scan ���� master number(�ܺ� scan) -> ���� scan(slave scan)�� ������ ���� check��� ��� ���
uint8_t checkMasterNum = 1;
uint8_t address_i;
unsigned long startTime;

//multi-hop�� ��� timeout���θ� Ȯ���ϱ� ���� ����
bool timeout = true;

//���� scan ���� master number(���� scan)
uint8_t scanMasterNum = 1;

//���� �����Ϳ��� ���� ��ĵ ����� ������ �Ǵ��� ���θ� ���� ����
bool nextScan = true;

//�����ͷκ��� ���� ���� �޽����� �����ϱ� ���� �迭(������ �ܺ���� �������� 10Bytes)
byte master_answer[32][10];

//���� ����Ʈ���̿��� ���� �޽���(������ �ܺ���� ��������)�� �����ϱ� ���� �迭
byte gateway_request[32][10];

//������ ���� ���θ� üũ�ϴ� �迭
//���� ���� ����Ʈ���̰� 1�� �����Ϳ��� ���� ����� ���ȴٸ� control_message[1]�� �ش� �޽����� �����ϰ�
//becontrol[1]�� true�� set

bool beControl[32];
byte control_message[32][10];
unsigned long controlRecvTime[32];

//���� ����Ʈ���̷κ��� ���� �޽����� �ӽ÷� ����
byte receiveFromG[10];
uint8_t indexFromG = 0;
uint8_t group_id;

unsigned long scanTime = 0;
unsigned long controlTime = 0;
unsigned long waitTime = 0;


bool DGcheckReceive[34]; //1�� index���� ���� ��
RoutingTableEntry    _routes[ROUTING_TABLE_SIZE];
int DGmaster_num;
int8_t DGparentMaster[34]; //1�� index���� ���� ��
byte DGunRecvCnt[34]; //1�� index���� ���� ��
byte DGtemp_buf[20];
byte DGbuffer[20];

signed char e_rssi;

int main()
{
	DGInit(0x00, 0x00, 15);
	_thisAddress = DGgetThisAddress();
	DGSetReceive();

	/* System Clocks Configuration */
  RCC_Configuration();
	/* NVIC configuration */
  NVIC_Configuration();
  /* Configure the GPIO ports */
  GPIO_Configuration();
	
	USART2_Configuration();
	
	while(1)
	{
		printf("hi");
	}
	DGFromGatewayToMaster();

	while(1)
	{
		DGprintTree();
		
		// ���� scan ����� ���� �����ͷκ��� 10�� �ȿ� ������ ���� ���ϸ� ���� �������� scan���� �Ѿ����
		if(!nextScan && scanTime + 600000 < millis())
		{ 
			// scanMasterNum�� ���� zone�� ����
			// � ���������� �˾Ƽ�..
			nextScan = true;
			if(++scanMasterNum >= 33) //DGmaster_num)
				scanMasterNum = 1;
		}//nextScan = false; // for test


		//*****************************************************************************************************************���� ���**********************************************************************************************************************************************
		//slave ��ĵ ���� �ƴ� ���� ���� �޽��� ����, beControl�� set�Ǿ� ������ �ش� �����Ϳ��� ���� �޽����� ����
		if(nextScan)
		{
			for(int beControlIndex = 0;beControlIndex < 32;beControlIndex++) // ������ ��ȣ(���� ���) : 0 ~ 0x1F, ������� �ּ� : 1 ~ 0x20
			{
				if(beControl[beControlIndex])
				{
					// ������ ��ȣ(���� ���) : 0 ~ 0x1F, ������� �ּ� : 1 ~ 0x20
					address_i = beControlIndex + 1;// convertToAddress(gatewayNumber, i, 0);

					// ���� ����Ʈ���̷κ��� ���� ���� �޽����� ������� ���ۿ� �ű�
					for(int i = 0;i < 10;i++)
						DGtemp_buf[i] = control_message[beControlIndex][i];
						
					controlTime = millis();
					
					if(DGgetRouteTo(address_i)->hop == 1)
						waitTime = TIME_TERM * 2 + 2500 + 5000;
					else
						waitTime = TIME_TERM * 2;
					
					// ������� ������ �������� ���
					if(DGcheckReceive[address_i])
					{
						//���ϴ� �������� next_hop���� ��Ŷ ����(type : CONTROL_TO_MASTER)
						if (!DGsendToWaitAck(_thisAddress, DGgetRouteTo(address_i)->next_hop, _thisAddress, address_i, CONTROL_TO_MASTER, NONE, NONE, NONE, NONE, DGtemp_buf, sizeof(DGtemp_buf), waitTime))
						{
							// next_hop���κ��� ACK�� ���� ������ ���
							uint8_t reroutingAddr = DGgetRouteTo(address_i)->next_hop;

							// next_hop�� unRecvCnt(��Ŷ�� ���� ���� Ƚ���� ī��Ʈ�ϴ� ����) ���� ����
							if (DGunRecvCnt[reroutingAddr]++ > MAX_UN_RECV)
							{
								// unRecvCnt�� threshold�̻��� �� �ٽ� ���������
								DGcheckReceive[reroutingAddr] = false;
								DGparentMaster[reroutingAddr] = -1;
								DGG_discoverNewPath(reroutingAddr, DGgetRouteTo(reroutingAddr)->hop);
								DGunRecvCnt[reroutingAddr] = 0;
								DGprintTree();
							}
						}
						else
						{
							timeout = true;
							startTime = millis();
							while (millis() - startTime < DEFAULT_RETRIES * 4 * DGgetRouteTo(address_i)->hop * TIME_TERM + 7500) // ??? ??? *4, 5?? zone??? ?? ???? ???? ??? ???? ??
							{
								DGSetReceive();
								if (DGavailable())
								{
									if (DGrecvData(DGtemp_buf) && DGheaderTo() == _thisAddress)
									{
										// ��� ����
										_rxHeaderTo = DGheaderTo();
										_rxHeaderFrom = DGheaderFrom();
										_rxHeaderSource = DGheaderSource();
										_rxHeaderDestination = DGheaderDestination();
										_rxHeaderType = DGheaderType();
										_rxHeaderData = DGheaderData();
										_rxHeaderFlags = DGheaderFlags();
										_rxHeaderSeqNum = DGheaderSeqNum();
										_rxHeaderHop = DGheaderHop();
						
										//DGprintRecvPacketHeader();
										timeout = false;

										// ACK ����
										DGsend(_thisAddress, _rxHeaderFrom, _thisAddress, _rxHeaderFrom, ACK, NONE, NONE, NONE, NONE, DGtemp_buf, sizeof(DGtemp_buf));
										
										if (_rxHeaderType == NACK)
										{
											printf("NACK\r\n");
											uint8_t reroutingAddr = DGtemp_buf[0];
											if (DGunRecvCnt[reroutingAddr]++ > MAX_UN_RECV)
											{
												DGcheckReceive[reroutingAddr] = false;
												DGparentMaster[reroutingAddr] = -1;
												DGG_discoverNewPath(reroutingAddr, DGgetRouteTo(reroutingAddr)->hop);
												DGunRecvCnt[reroutingAddr] = 0;
												DGprintTree();
											}
										}
										else if (_rxHeaderType == CONTROL_RESPONSE_TO_GATEWAY)
										{
											// control�޽����� �������� ���� �߿� ����Ʈ���̰� �� �������� �� ���� �����Ƿ�
											// ����Ʈ���̷κ��� ����� ���� �ð�(A)�� ����صΰ� �������� �� ���� ����� ���� �ð��� A ������ ��츸 ���� ����� �Ϸ�Ǿ����� ǥ��
											
											if(controlTime > controlRecvTime[beControlIndex])
												beControl[beControlIndex] = false; // �������� �����Ͽ����� ǥ��
											
											for(uint8_t i = 0;i < 10;i++)
												master_answer[beControlIndex][i] = DGtemp_buf[i];
										}
										break;
									}
								} // end of if(DGavailable)
							} // end of while
							if (timeout)
							{
								printf("timeout!\r\n");
								DGG_find_error_node(address_i);
							}
						} // end of else if (DGgetRouteTo(address_i)->hop != 1)
					} // end of if(DGcheckReceive[address_i])
				} // end of if(beControl[beControlIndex])
				
				/*else
				{
					for(uint8_t i = 0;i < 10;i++)
						master_answer[beControlIndex][i] = 0;
				}*/
			}
		}










		//*****************************************************************************************************************���� zone scan ���**********************************************************************************************************************************************
		// nextScan default value : true, �����ͷκ��� ���� zone�� scan�� �����ٴ� ��Ŷ�� �ްų�, ���� zone���κ��� ���� �ð��� ������ scan �Ϸ� ��Ŷ�� ���� ���Ѵٸ� true�� set��
		// �� zone�� scan ��û
		// scanMasterNum : ���� scan�� zone�� ������ ��ȣ(���� ��� �ּ�)
		
		if(nextScan)
		{
			printf("SCAN\r\n");
			printf("%d\r\n", scanMasterNum);
			address_i = scanMasterNum;

			// ������� ������ �������� ���
			if(DGcheckReceive[address_i])
			{
				if(DGgetRouteTo(address_i)->hop == 1)
					waitTime = TIME_TERM * 2 + 2500;
				else
					waitTime = TIME_TERM * 2;
				// sendToWaitAck���� �ش� �������� next_hop���� ��Ŷ ����
				if (!DGsendToWaitAck(_thisAddress, DGgetRouteTo(address_i)->next_hop, _thisAddress, address_i, SCAN_REQUEST_TO_MASTER, NONE, NONE, NONE, NONE, DGtemp_buf, sizeof(DGtemp_buf), waitTime))
				{
					// next_hop���κ��� ACK�� ���� ���� ���
					uint8_t reroutingAddr = DGgetRouteTo(address_i)->next_hop;

					// �ش� �ּ��� unRecvCnt ���� ����
					if (DGunRecvCnt[reroutingAddr]++ > MAX_UN_RECV)
					{
						// unRecvCnt�� threshold�̻��� ��� �� �������� ��� ������ �ٽ� ����
						DGcheckReceive[reroutingAddr] = false;
						DGparentMaster[reroutingAddr] = -1;
						DGG_discoverNewPath(reroutingAddr, DGgetRouteTo(reroutingAddr)->hop);
						DGunRecvCnt[reroutingAddr] = 0;
						DGprintTree();
					}
				}
				else if(DGgetRouteTo(address_i)->hop == 1)
				{
					// one hop�� ��� sendToWaitAck���� �̹� SCAN_REQUEST_ACK_FROM_MASTER�� ������

					// ��� ����
					_rxHeaderTo = DGheaderTo();
					_rxHeaderFrom = DGheaderFrom();
					_rxHeaderSource = DGheaderSource();
					_rxHeaderDestination = DGheaderDestination();
					_rxHeaderType = DGheaderType();
					_rxHeaderData = DGheaderData();
					_rxHeaderFlags = DGheaderFlags();
					_rxHeaderSeqNum = DGheaderSeqNum();
					_rxHeaderHop = DGheaderHop();

					// ACK ����
					DGsend(_thisAddress, _rxHeaderFrom, _thisAddress, _rxHeaderFrom, ACK, NONE, NONE, NONE, NONE, DGtemp_buf, sizeof(DGtemp_buf));
					
					nextScan = false; //���� ���� �� ��ĵ ����
					scanTime = millis(); //���� �� ��ĵ ������ �� �� ��� ���� �� ��ĵ�� ���ؼ� timeout���θ� Ȯ���ϱ� ����
				}
				else if (DGgetRouteTo(address_i)->hop != 1)
				{
					//multi hop�� ��� SCAN_REQUEST_ACK_FROM_MASTER�� ������ ������ ��ٷ���
					
					timeout = true;
					startTime = millis();
					while (millis() - startTime < DEFAULT_RETRIES * 4 * DGgetRouteTo(address_i)->hop * TIME_TERM + 2500)
					{
						DGSetReceive();
						if (DGavailable())
						{
							if (DGrecvData(DGtemp_buf) && DGheaderTo() == _thisAddress)
							{
								_rxHeaderTo = DGheaderTo();
								_rxHeaderFrom = DGheaderFrom();
								_rxHeaderSource = DGheaderSource();
								_rxHeaderDestination = DGheaderDestination();
								_rxHeaderType = DGheaderType();
								_rxHeaderData = DGheaderData();
								_rxHeaderFlags = DGheaderFlags();
								_rxHeaderSeqNum = DGheaderSeqNum();
								_rxHeaderHop = DGheaderHop();
				
								DGprintRecvPacketHeader();
								timeout = false;
								DGsend(_thisAddress, _rxHeaderFrom, _thisAddress, _rxHeaderFrom, ACK, NONE, NONE, NONE, NONE, DGtemp_buf, sizeof(DGtemp_buf));
		
								
								if (_rxHeaderType == NACK)
								{
									printf("NACK\r\n");
									uint8_t reroutingAddr = DGtemp_buf[0];
									if (DGunRecvCnt[reroutingAddr]++ > MAX_UN_RECV)
									{
										DGcheckReceive[reroutingAddr] = false;
										DGparentMaster[reroutingAddr] = -1;
										DGG_discoverNewPath(reroutingAddr, DGgetRouteTo(reroutingAddr)->hop);
										DGunRecvCnt[reroutingAddr] = 0;
										DGprintTree();
									}
								}
								else if (_rxHeaderType == SCAN_REQUEST_ACK_FROM_MASTER)
								{
									scanTime = millis();
									nextScan = false;
								}
								break;
							} // end of if (DGrecvData(DGtemp_buf) && DGheaderTo() == _thisAddress)
						} // end of if (DGavailable())
					} // end of while
					if (timeout)
					{
						printf("timeout!\r\n");
						DGG_find_error_node(address_i);
					}
				} 
			}// end of if(DGcheckReceive[address_i])
		}






		//*****************************************************************************************************************�ܺ� master scan (check)**********************************************************************************************************************************************  
		address_i = checkMasterNum;

		// ���� �����͸� ��Ŷ�� payload�� ����
		for(int i = 0;i < 10;i++)
		{
			DGtemp_buf[i] = gateway_request[checkMasterNum - 1][i];
		}
		if(DGgetRouteTo(address_i)->hop == 1)
			waitTime = TIME_TERM * 2 + 2500;
		else
			waitTime = TIME_TERM * 2;
		// ������� ������ �������� ���
		if(DGcheckReceive[address_i])
		{
			if (!DGsendToWaitAck(_thisAddress, DGgetRouteTo(address_i)->next_hop, _thisAddress, address_i, CHECK_ROUTING, NONE, NONE, NONE, NONE, DGtemp_buf, sizeof(DGtemp_buf), waitTime))
			{
				uint8_t reroutingAddr = DGgetRouteTo(address_i)->next_hop;
				if (DGunRecvCnt[reroutingAddr]++ > MAX_UN_RECV)
				{
					DGcheckReceive[reroutingAddr] = false;
					DGparentMaster[reroutingAddr] = -1;
					DGG_discoverNewPath(reroutingAddr, DGgetRouteTo(reroutingAddr)->hop);
					DGunRecvCnt[reroutingAddr] = 0;
					DGprintTree();
				}
			}
			
			// multi hop�뵵�� ���
			else if (DGgetRouteTo(address_i)->hop != 1)
			{
				startTime = millis();
				timeout = true;

				// multi hop�̹Ƿ� src<-->dst ���۱��� ���
				while (millis() - startTime < DEFAULT_RETRIES * 4 * DGgetRouteTo(address_i)->hop * TIME_TERM + 2500)
				{

					// ���� ���·� ��ȯ
					DGSetReceive();

					// ��Ŷ ����
					if (DGavailable())
					{
						if (DGrecvData(DGtemp_buf) && DGheaderTo() == _thisAddress)
						{
							// ��� ����
							_rxHeaderTo = DGheaderTo();
							_rxHeaderFrom = DGheaderFrom();
							_rxHeaderSource = DGheaderSource();
							_rxHeaderDestination = DGheaderDestination();
							_rxHeaderType = DGheaderType();
							_rxHeaderData = DGheaderData();
							_rxHeaderFlags = DGheaderFlags();
							_rxHeaderSeqNum = DGheaderSeqNum();
							_rxHeaderHop = DGheaderHop();
		
							DGprintRecvPacketHeader();
							timeout = false;

							// ACK ����
							DGsend(_thisAddress, _rxHeaderFrom, _thisAddress, _rxHeaderFrom, ACK, NONE, NONE, NONE, NONE, DGtemp_buf, sizeof(DGtemp_buf));

							// ��Ŷ�� dst���� relay���� �� ��
							if (_rxHeaderType == NACK)
							{
								printf("NACK\r\n");

								// ������ ���� ����� unRecvCnt ���� ����
								uint8_t reroutingAddr = DGtemp_buf[0];
								if (DGunRecvCnt[reroutingAddr]++ > MAX_UN_RECV)
								{
									DGcheckReceive[reroutingAddr] = false;
									DGparentMaster[reroutingAddr] = -1;
									DGG_discoverNewPath(reroutingAddr, DGgetRouteTo(reroutingAddr)->hop);
									DGunRecvCnt[reroutingAddr] = 0;
									DGprintTree();
								}
							}

							// ��Ŷ�� dst���� relay ��
							else
							{
								// ������ ������ ����
								for(int i = 0;i < 10;i++)
									master_answer[checkMasterNum - 1][i] = DGtemp_buf[i];

								// scan�Ϸ�
								if (_rxHeaderType == CHECK_ROUTING_ACK)
								{
									// ���ο� master�� �߰��Ǿ��ٸ� _rxHeaderData�� ���� �߰��� ����� ����, payload(master���� ������ ���Ŀ�, index10����)�� �߰��� ����� �ּҰ� �������
									for (uint8_t i = 0; i < _rxHeaderData; i++)
									{
										DGaddRouteTo(DGtemp_buf[10 + i], _rxHeaderFrom, Valid, DGgetRouteTo(_rxHeaderSource)->hop + 1, millis());
										DGcheckReceive[DGtemp_buf[10 + i]] = true;
										DGparentMaster[DGtemp_buf[10 + i]] = _rxHeaderSource;
										DGmaster_num++;
										DGprintRoutingTable();
									}
								}

								// scan �Ϸ� + �������� ��� ����
								else if (_rxHeaderType == CHECK_ROUTING_ACK_REROUTING)
								{
									int8_t temp_hopCount = DGgetRouteTo(_rxHeaderSource)->hop;
									DGaddRouteTo(_rxHeaderSource, _rxHeaderFrom, Valid, _rxHeaderHop + 1, millis());
									DGchangeNextHop(_rxHeaderSource, temp_hopCount - (_rxHeaderHop + 1));
									DGparentMaster[_rxHeaderSource] = DGtemp_buf[10];
			
									DGprintRoutingTable();
								}

								// scan �Ϸ� + �ش� zone�� ���� scan �Ϸ�
								else if(_rxHeaderType == SCAN_RESPONSE_TO_GATEWAY)
								{
									if(++scanMasterNum >= 33)//DGmaster_num)
										scanMasterNum = 1;
									nextScan = true;
								}
							}
							break;
						}
					}
				} // end of while
				if (timeout)
				{
					printf("timeout!\r\n");
					DGG_find_error_node(address_i);
				}
			} // end of multihop ��� ó��


			// ���� 1hop�̰ų� rerouting ��� 1hop�� ����, ������ ���� ó��
			else 
			{
				// ��� ����
				_rxHeaderTo = DGheaderTo();
				_rxHeaderFrom = DGheaderFrom();
				_rxHeaderSource = DGheaderSource();
				_rxHeaderDestination = DGheaderDestination();
				_rxHeaderType = DGheaderType();
				_rxHeaderData = DGheaderData();
				_rxHeaderFlags = DGheaderFlags();
				_rxHeaderSeqNum = DGheaderSeqNum();
				_rxHeaderHop = DGheaderHop();

				// ACK ����
				DGsend(_thisAddress, _rxHeaderFrom, _thisAddress, _rxHeaderFrom, ACK, NONE, NONE, NONE, NONE, DGtemp_buf, sizeof(DGtemp_buf));
				
				for(int i = 0;i < 10;i++)
					master_answer[checkMasterNum - 1][i] = DGtemp_buf[i];
					
				if (_rxHeaderType == CHECK_ROUTING_ACK)
				{
					for (uint8_t i = 0; i < _rxHeaderData; i++)
					{
						DGaddRouteTo(DGtemp_buf[10 + i], _rxHeaderFrom, Valid, DGgetRouteTo(_rxHeaderSource)->hop + 1, millis());
						DGcheckReceive[DGtemp_buf[10 + i]] = true;
						DGparentMaster[DGtemp_buf[10 + i]] = _rxHeaderSource;
						DGprintRoutingTable();
						DGmaster_num++;
					}
				}
				else if (_rxHeaderType == CHECK_ROUTING_ACK_REROUTING)
				{
					int8_t temp_hopCount = DGgetRouteTo(_rxHeaderSource)->hop;
					DGaddRouteTo(_rxHeaderSource, _rxHeaderFrom, Valid, _rxHeaderHop + 1, millis());
					DGchangeNextHop(_rxHeaderSource, temp_hopCount - (_rxHeaderHop + 1));
					DGparentMaster[_rxHeaderSource] = DGtemp_buf[10];
			
					DGprintRoutingTable();
				}
				else if(_rxHeaderType == SCAN_RESPONSE_TO_GATEWAY)
				{
					if(++scanMasterNum >= 33) //DGmaster_num)
						scanMasterNum = 1;
					nextScan = true;
				}
			} // end of 1hop ��� ó��
		} // end of if(DGcheckReceive[address_i]) -> ������� ������ �������� ���

		// ������� ����� �������� �������� ���
		else
		{
			//FCU���� error���� ǥ�����ֱ� ���ؼ�
			for(uint8_t i = 0;i < 10;i++)
				master_answer[checkMasterNum - 1][i] = 0;
		}



		//���� ������ check
		if(++checkMasterNum >= 33) //DGmaster_num)
			checkMasterNum = 1;
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
	//STM32F1������ 10, 2, 50�� ������ �� ����. ���忡 ���� �ٸ�(���� �����Ҽ��� noise ���� ���ɼ��� ������)
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
		if(indexFromG == 0)
		{
			byte temp = USART_ReceiveData(USART2);
			if(temp == 0xD5 || temp == 0xC5)
			{
				receiveFromG[indexFromG++] = temp;
			}
		}
		else
			receiveFromG[indexFromG++] = USART_ReceiveData(USART2);
	}
	
	if(indexFromG == 10)
	{
		for(int i = 0;i < 10;i++)
		{
			printf("%x ", receiveFromG[i]);
		}
		printf("\r\n");
		
		group_id = receiveFromG[2];
		
		USART_ITConfig(USART2, USART_IT_RXNE, DISABLE);	
		GPIO_SetBits(GPIOA, GPIO_Pin_1);
		
		for(int i = 0;i < 10;i++)
		{
			USART_SendData(USART2, master_answer[group_id][i]);
			//while(USART_GetFlagStatus(USART2, USART_FLAG_TXE) == RESET);
			while(USART_GetFlagStatus(USART2, USART_FLAG_TC) == RESET);
		}
		
		GPIO_ResetBits(GPIOA, GPIO_Pin_1);	
		USART_ITConfig(USART2, USART_IT_RXNE, ENABLE);
		

  
		byte temp = 0;
		for(int i = 0;i < 9;i++)
				temp ^= receiveFromG[i];

		if(temp == receiveFromG[9])
		{
			if(receiveFromG[0] == 0xBD) // ���¿�û
			{
					for(int i = 0;i < 10;i++)
					{
						gateway_request[group_id][i] = receiveFromG[i];
					}
			}
			else if(receiveFromG[0] == 0xBE)//�����û
			{
				for(int i = 0;i < 10;i++)
				{
					control_message[group_id][i] = receiveFromG[i];
				}
				beControl[group_id] = true;
			} 
		}
		indexFromG = 0; 
	}
}
