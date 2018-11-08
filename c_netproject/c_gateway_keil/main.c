//내부 zone의 error는 관여하지 않음(알아서..)

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

//현재 scan 중인 master number(외부 scan) -> 내부 scan(slave scan)과 구분을 위해 check라는 용어 사용
uint8_t checkMasterNum = 1;
uint8_t address_i;
unsigned long startTime;

//multi-hop일 경우 timeout여부를 확인하기 위한 변수
bool timeout = true;

//현재 scan 중인 master number(내부 scan)
uint8_t scanMasterNum = 1;

//다음 마스터에게 내부 스캔 명령을 내려도 되는지 여부를 위한 변수
bool nextScan = true;

//마스터로부터 받은 응답 메시지를 저장하기 위한 배열(에어텍 외부통신 프로토콜 10Bytes)
byte master_answer[32][10];

//실제 게이트웨이에게 받은 메시지(에어텍 외부통신 프로토콜)를 저장하기 위한 배열
byte gateway_request[32][10];

//제어명령 수신 여부를 체크하는 배열
//만일 실제 게이트웨이가 1번 마스터에게 제어 명령을 내렸다면 control_message[1]에 해당 메시지를 저장하고
//becontrol[1]을 true로 set

bool beControl[32];
byte control_message[32][10];
unsigned long controlRecvTime[32];

//실제 게이트웨이로부터 받은 메시지를 임시로 저장
byte receiveFromG[10];
uint8_t indexFromG = 0;
uint8_t group_id;

unsigned long scanTime = 0;
unsigned long controlTime = 0;
unsigned long waitTime = 0;


bool DGcheckReceive[34]; //1번 index부터 값이 들어감
RoutingTableEntry    _routes[ROUTING_TABLE_SIZE];
int DGmaster_num;
int8_t DGparentMaster[34]; //1번 index부터 값이 들어감
byte DGunRecvCnt[34]; //1번 index부터 값이 들어감
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
		
		// 내부 scan 명령을 내린 마스터로부터 10분 안에 응답을 받지 못하면 다음 마스터의 scan으로 넘어가도록
		if(!nextScan && scanTime + 600000 < millis())
		{ 
			// scanMasterNum이 속한 zone의 에러
			// 어떤 에러인지는 알아서..
			nextScan = true;
			if(++scanMasterNum >= 33) //DGmaster_num)
				scanMasterNum = 1;
		}//nextScan = false; // for test


		//*****************************************************************************************************************제어 명령**********************************************************************************************************************************************
		//slave 스캔 중이 아닐 때만 제어 메시지 전송, beControl이 set되어 있으면 해당 마스터에게 제어 메시지를 보냄
		if(nextScan)
		{
			for(int beControlIndex = 0;beControlIndex < 32;beControlIndex++) // 마스터 번호(유선 통신) : 0 ~ 0x1F, 무선통신 주소 : 1 ~ 0x20
			{
				if(beControl[beControlIndex])
				{
					// 마스터 번호(유선 통신) : 0 ~ 0x1F, 무선통신 주소 : 1 ~ 0x20
					address_i = beControlIndex + 1;// convertToAddress(gatewayNumber, i, 0);

					// 실제 게이트웨이로부터 받은 제어 메시지를 무선통신 버퍼에 옮김
					for(int i = 0;i < 10;i++)
						DGtemp_buf[i] = control_message[beControlIndex][i];
						
					controlTime = millis();
					
					if(DGgetRouteTo(address_i)->hop == 1)
						waitTime = TIME_TERM * 2 + 2500 + 5000;
					else
						waitTime = TIME_TERM * 2;
					
					// 라우팅이 정상인 마스터일 경우
					if(DGcheckReceive[address_i])
					{
						//원하는 목적지의 next_hop에게 패킷 전송(type : CONTROL_TO_MASTER)
						if (!DGsendToWaitAck(_thisAddress, DGgetRouteTo(address_i)->next_hop, _thisAddress, address_i, CONTROL_TO_MASTER, NONE, NONE, NONE, NONE, DGtemp_buf, sizeof(DGtemp_buf), waitTime))
						{
							// next_hop으로부터 ACK을 받지 못했을 경우
							uint8_t reroutingAddr = DGgetRouteTo(address_i)->next_hop;

							// next_hop의 unRecvCnt(패킷을 받지 못한 횟수를 카운트하는 변수) 값을 증가
							if (DGunRecvCnt[reroutingAddr]++ > MAX_UN_RECV)
							{
								// unRecvCnt가 threshold이상일 때 다시 라우팅해줌
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
										// 헤더 저장
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

										// ACK 전송
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
											// control메시지를 무선으로 전송 중에 게이트웨이가 또 제어명령을 할 수도 있으므로
											// 게이트웨이로부터 명령을 받은 시간(A)을 기록해두고 무선으로 이 제어 명령을 보낸 시간이 A 이후일 경우만 제어 명령이 완료되었음을 표시
											
											if(controlTime > controlRecvTime[beControlIndex])
												beControl[beControlIndex] = false; // 제어명령을 전송하였음을 표시
											
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










		//*****************************************************************************************************************내부 zone scan 명령**********************************************************************************************************************************************
		// nextScan default value : true, 마스터로부터 내부 zone의 scan이 끝났다는 패킷을 받거나, 내부 zone으로부터 일정 시간이 지나도 scan 완료 패킷을 받지 못한다면 true로 set됨
		// 각 zone의 scan 요청
		// scanMasterNum : 현재 scan할 zone의 마스터 번호(무선 통신 주소)
		
		if(nextScan)
		{
			printf("SCAN\r\n");
			printf("%d\r\n", scanMasterNum);
			address_i = scanMasterNum;

			// 라우팅이 정상인 마스터일 경우
			if(DGcheckReceive[address_i])
			{
				if(DGgetRouteTo(address_i)->hop == 1)
					waitTime = TIME_TERM * 2 + 2500;
				else
					waitTime = TIME_TERM * 2;
				// sendToWaitAck으로 해당 마스터의 next_hop에게 패킷 전송
				if (!DGsendToWaitAck(_thisAddress, DGgetRouteTo(address_i)->next_hop, _thisAddress, address_i, SCAN_REQUEST_TO_MASTER, NONE, NONE, NONE, NONE, DGtemp_buf, sizeof(DGtemp_buf), waitTime))
				{
					// next_hop으로부터 ACK을 받지 못한 경우
					uint8_t reroutingAddr = DGgetRouteTo(address_i)->next_hop;

					// 해당 주소의 unRecvCnt 값을 증가
					if (DGunRecvCnt[reroutingAddr]++ > MAX_UN_RECV)
					{
						// unRecvCnt가 threshold이상일 경우 그 마스터의 경로 설정을 다시 해줌
						DGcheckReceive[reroutingAddr] = false;
						DGparentMaster[reroutingAddr] = -1;
						DGG_discoverNewPath(reroutingAddr, DGgetRouteTo(reroutingAddr)->hop);
						DGunRecvCnt[reroutingAddr] = 0;
						DGprintTree();
					}
				}
				else if(DGgetRouteTo(address_i)->hop == 1)
				{
					// one hop일 경우 sendToWaitAck으로 이미 SCAN_REQUEST_ACK_FROM_MASTER를 수신함

					// 헤더 저장
					_rxHeaderTo = DGheaderTo();
					_rxHeaderFrom = DGheaderFrom();
					_rxHeaderSource = DGheaderSource();
					_rxHeaderDestination = DGheaderDestination();
					_rxHeaderType = DGheaderType();
					_rxHeaderData = DGheaderData();
					_rxHeaderFlags = DGheaderFlags();
					_rxHeaderSeqNum = DGheaderSeqNum();
					_rxHeaderHop = DGheaderHop();

					// ACK 전송
					DGsend(_thisAddress, _rxHeaderFrom, _thisAddress, _rxHeaderFrom, ACK, NONE, NONE, NONE, NONE, DGtemp_buf, sizeof(DGtemp_buf));
					
					nextScan = false; //현재 내부 존 스캔 중임
					scanTime = millis(); //내부 존 스캔 응답이 안 올 경우 다음 존 스캔을 위해서 timeout여부를 확인하기 위함
				}
				else if (DGgetRouteTo(address_i)->hop != 1)
				{
					//multi hop일 경우 SCAN_REQUEST_ACK_FROM_MASTER를 수신할 때까지 기다려줌
					
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






		//*****************************************************************************************************************외부 master scan (check)**********************************************************************************************************************************************  
		address_i = checkMasterNum;

		// 보낼 데이터를 패킷의 payload에 복사
		for(int i = 0;i < 10;i++)
		{
			DGtemp_buf[i] = gateway_request[checkMasterNum - 1][i];
		}
		if(DGgetRouteTo(address_i)->hop == 1)
			waitTime = TIME_TERM * 2 + 2500;
		else
			waitTime = TIME_TERM * 2;
		// 라우팅이 정상인 마스터일 경우
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
			
			// multi hop노도의 경우
			else if (DGgetRouteTo(address_i)->hop != 1)
			{
				startTime = millis();
				timeout = true;

				// multi hop이므로 src<-->dst 전송까지 대기
				while (millis() - startTime < DEFAULT_RETRIES * 4 * DGgetRouteTo(address_i)->hop * TIME_TERM + 2500)
				{

					// 수신 상태로 전환
					DGSetReceive();

					// 패킷 수신
					if (DGavailable())
					{
						if (DGrecvData(DGtemp_buf) && DGheaderTo() == _thisAddress)
						{
							// 헤더 저장
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

							// ACK 전송
							DGsend(_thisAddress, _rxHeaderFrom, _thisAddress, _rxHeaderFrom, ACK, NONE, NONE, NONE, NONE, DGtemp_buf, sizeof(DGtemp_buf));

							// 패킷이 dst까지 relay되지 못 함
							if (_rxHeaderType == NACK)
							{
								printf("NACK\r\n");

								// 전송이 끊긴 노드의 unRecvCnt 값을 증가
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

							// 패킷이 dst까지 relay 됨
							else
							{
								// 수신한 데이터 저장
								for(int i = 0;i < 10;i++)
									master_answer[checkMasterNum - 1][i] = DGtemp_buf[i];

								// scan완료
								if (_rxHeaderType == CHECK_ROUTING_ACK)
								{
									// 새로운 master가 추가되었다면 _rxHeaderData에 새로 추가된 노드의 개수, payload(master응답 데이터 이후에, index10부터)에 추가된 노드의 주소가 들어있음
									for (uint8_t i = 0; i < _rxHeaderData; i++)
									{
										DGaddRouteTo(DGtemp_buf[10 + i], _rxHeaderFrom, Valid, DGgetRouteTo(_rxHeaderSource)->hop + 1, millis());
										DGcheckReceive[DGtemp_buf[10 + i]] = true;
										DGparentMaster[DGtemp_buf[10 + i]] = _rxHeaderSource;
										DGmaster_num++;
										DGprintRoutingTable();
									}
								}

								// scan 완료 + 마스터의 경로 변경
								else if (_rxHeaderType == CHECK_ROUTING_ACK_REROUTING)
								{
									int8_t temp_hopCount = DGgetRouteTo(_rxHeaderSource)->hop;
									DGaddRouteTo(_rxHeaderSource, _rxHeaderFrom, Valid, _rxHeaderHop + 1, millis());
									DGchangeNextHop(_rxHeaderSource, temp_hopCount - (_rxHeaderHop + 1));
									DGparentMaster[_rxHeaderSource] = DGtemp_buf[10];
			
									DGprintRoutingTable();
								}

								// scan 완료 + 해당 zone의 내부 scan 완료
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
			} // end of multihop 노드 처리


			// 원래 1hop이거나 rerouting 결과 1hop인 노드들, 위에랑 같은 처리
			else 
			{
				// 헤더 저장
				_rxHeaderTo = DGheaderTo();
				_rxHeaderFrom = DGheaderFrom();
				_rxHeaderSource = DGheaderSource();
				_rxHeaderDestination = DGheaderDestination();
				_rxHeaderType = DGheaderType();
				_rxHeaderData = DGheaderData();
				_rxHeaderFlags = DGheaderFlags();
				_rxHeaderSeqNum = DGheaderSeqNum();
				_rxHeaderHop = DGheaderHop();

				// ACK 전송
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
			} // end of 1hop 노드 처리
		} // end of if(DGcheckReceive[address_i]) -> 라우팅이 정상인 마스터일 경우

		// 라우팅이 제대로 되지않은 마스터일 경우
		else
		{
			//FCU에게 error임을 표시해주기 위해서
			for(uint8_t i = 0;i < 10;i++)
				master_answer[checkMasterNum - 1][i] = 0;
		}



		//다음 마스터 check
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
	//STM32F1에서는 10, 2, 50중 선택할 수 있음. 보드에 따라 다름(높게 설정할수록 noise 유발 가능성이 높아짐)
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
			if(receiveFromG[0] == 0xBD) // 상태요청
			{
					for(int i = 0;i < 10;i++)
					{
						gateway_request[group_id][i] = receiveFromG[i];
					}
			}
			else if(receiveFromG[0] == 0xBE)//제어요청
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
