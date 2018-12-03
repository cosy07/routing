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
void USART3_Configuration(void);
void NVIC_Configuration(void);
void RS485_Write_Read(void);

// 게이트웨이로부터 받은 라우팅 시작 패킷 개수를 카운트하기 위한 변수
uint8_t receivedRequestNum = 0;

// 내 근처에 있는 마스터로부터 엿들은 패킷의 개수 (더 좋은 경로를 설정하기 위하여)
uint8_t receivedOverhearNum = 1;

uint8_t priorPacketFrom = -1;
uint8_t priorPacketType = -1;

bool newNode = false;
bool rerouting = false;
bool scanFinish = false;
uint8_t receivedNum[34] = { 0 };
int8_t rerouting_candidate = -1;

uint8_t check_cnt = 0;
unsigned long check_table_time;
unsigned long check_rerouting_time;
unsigned long rs485_loop_time;


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

uint8_t masterBroadcastAddress = 0xFF;

byte inputData[10];
byte outputData[10];

byte receivedFromInMaster;
bool timeout = true;
byte receiveFromInMaster;
bool routing_complete = false;

bool DGcheckReceive[34];
RoutingTableEntry    _routes[ROUTING_TABLE_SIZE];
int DGmaster_num;
int8_t DGparentMaster[34];
byte DGunRecvCnt[34];
byte DGtemp_buf[20];
byte DGbuffer[20];

signed char e_rssi;

int main()
{
	/* Setup the microcontroller system. Initialize the Embedded Flash Interface,  
	initialize the PLL and update the SystemFrequency variable. */
	SystemInit();
	PrintfInit();	// printf 사용을 위한 초기화
	MyTimerInit(); // delay 및 millis 사용을 위한 초기화
	
	/* System Clocks Configuration */
  RCC_Configuration();
  /* Configure the GPIO ports */
  GPIO_Configuration();
	/* Configure USART2 */
	USART2_Configuration();
	/* Configure USART3 */
	USART3_Configuration();
	
	
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
				DGInit(initDataFromFCU[1], initDataFromFCU[2], initDataFromFCU[1] * 2 - 1); // 층, 주소, 채널 번호(외부채널은 층 * 2 -1, 내부채널은 층 * 2)
				_thisAddress = DGgetThisAddress(); // main.c의 전역변수 초기화
			break; // 제대로 수신했다면 while문 탈출
		}
	}
	
	DGSetReceive();

	while(1)
	{
		// 10분 간격으로 table update (사용되지 않는 routing table의 행은 삭제)
		if (routing_complete && check_table_time + 600000 < millis())
		{
			DGcheckRoutingTable();
			check_table_time = millis();
		}

		// 10분 간격으로 내가 선택한 부모 노드보다 gateway에 더 가까이 있는 노드가 있는 경우 다시 라우팅 해줌
		// 다른 노드의 패킷을 보고 gateway로 가는 hop count가 기존의 부모 노드보다 더 짧다면 receivedNum 값을 증가시켜줌 (line 785)
		// receivedNum 값이 제일 큰 노드가 후보 부모 노드가 됨
		
		if (check_rerouting_time + 600000 < millis())
		{
			printf("check_rerouting_time");
			check_rerouting_time = millis();
			int8_t max_receivedNum = 0;
			for (uint8_t i = 0; i <= 33; i++)
			{
				printf("%d : %d\r\n: ", i, receivedNum[i]);

				if (max_receivedNum < receivedNum[i])
				{
					rerouting_candidate = i;
					max_receivedNum = receivedNum[i];
					receivedNum[i] = 0;
					rerouting = true;
				}
			}
		}

		// 수신 가능 상태로
		DGSetReceive();

		// 패킷 수신
		if (DGavailable())
		{

			// 받은 데이터 버퍼에 저장
			if (DGrecvData(DGtemp_buf))
			{

				// 헤더를 따로 저장
				_rxHeaderTo = DGheaderTo();
				_rxHeaderFrom = DGheaderFrom();
				_rxHeaderSource = DGheaderSource();
				_rxHeaderDestination = DGheaderDestination();
				_rxHeaderType = DGheaderType();
				_rxHeaderData = DGheaderData();
				_rxHeaderFlags = DGheaderFlags();
				_rxHeaderSeqNum = DGheaderSeqNum();
				_rxHeaderHop = DGheaderHop();

				// 나에게 온 패킷일 경우 or 브로드캐스트일 경우
				if (_rxHeaderTo == _thisAddress || _rxHeaderTo == masterBroadcastAddress)
				{
					DGprintRecvPacketHeader();

					// 목적지가 내가 아닐 경우(relay 상황)
					if (_rxHeaderDestination != _thisAddress && _rxHeaderTo != masterBroadcastAddress)
					{
						uint8_t temp_source = _rxHeaderSource;
						printf("receive relay");

						// gateway가 테이블에 등록되어 있지 않음, 아직 라우팅이 되지 않은 노드 -> 아무것도 안함
						if (DGgetRouteTo(_thisAddress & 0x00) == NULL)
						{
							DGprintRoutingTable();
						}

						// gateway가 테이블에 등록되어 있음, 라우팅이 완료된 노드
						else
						{
							// ACK전송 (hop-to-hop에도 ACK을 전송해줌)
							DGsend(_thisAddress, 
										 _rxHeaderFrom, 
										 _thisAddress, 
										 _rxHeaderFrom, 
										 ACK, 
										 NONE, 
										 NONE, 
										 NONE, 
										 NONE, 
										 DGtemp_buf, 
										 sizeof(DGtemp_buf));

							// routing관련 응답을 수신 (내 자식노드가 생겼다는 뜻, 따라서 라우팅 테이블에 등록시킴) = 상향링크로의 패킷임
							if (_rxHeaderType == REQUEST_MULTI_HOP_ACK || _rxHeaderType == REQUEST_DIRECT_ACK)
							{
								DGaddRouteTo(_rxHeaderSource, _rxHeaderFrom, Valid, _rxHeaderHop + 1, millis());
								for(uint8_t i = 0;i < _rxHeaderData;i++)
								{
									DGaddRouteTo(DGtemp_buf[i + 1], _rxHeaderFrom, Valid, 0, millis()); 
									// 자식노드의 hop count는 별로 중요하지 않음 일단 0으로 해두고 다음에 해당 자식노드로부터 패킷을 받게되면 hop count에 맞게 setting
								}
								DGprintRoutingTable();
							}

							// 초기라우팅 완료 후, 운영 중에 _rxHeaderData에 값이 있다는 것은 NEW_NODE가 있음을의미, 나에게 패킷이 왔으므로 내 자식노드임 따라서 라우팅 테이블에 등록
							// _rxHeaderData는 추가해야 할 자식노드의 개수를 의미, temp_buf[10]부터 자식노드의 주소가 들어있음 (temp_buf[0] ~ [9]는 에어텍 프로토콜)
							else if (_rxHeaderType == CHECK_ROUTING_ACK)
							{
								// hop count 재설정
								DGaddRouteTo(_rxHeaderSource, _rxHeaderFrom, Valid, _rxHeaderHop + 1, millis());
								for (uint8_t i = 0; i < _rxHeaderData; i++)
								{
									DGaddRouteTo(DGtemp_buf[10 + i], _rxHeaderFrom, Valid, _rxHeaderHop + 2, millis());
								}
							}

							//원래 내 자식노드가 아니었는데 rerouting 결과 내 자식노드로.. 따라서 라우팅 테이블에 등록해줌
							else if (_rxHeaderType == CHECK_ROUTING_ACK_REROUTING)
							{
								
								//라우팅 테이블에 추가
								DGaddRouteTo(_rxHeaderSource, _rxHeaderFrom, Valid, _rxHeaderHop + 1, millis());
								
								//새롭게 자식노드가 된 노드의 자식노드들은 temp_buf[11]부터 들어있음. 이 노드들도 테이블에 등록(temp_buf[10]에는 자식노드의 바로 위 부모노드)
								for(uint8_t i = 0;i < _rxHeaderData;i++)
								{
									DGaddRouteTo(DGtemp_buf[11 + i], _rxHeaderFrom, Valid, 0, millis()); 
									// 자식노드의 hop count는 별로 중요하지 않음 일단 0으로 해두고 다음에 해당 자식노드로부터 패킷을 받게되면 hop count에 맞게 setting
								}
							}

							
							/*else if(_rxHeaderType == ROUTING_TABLE_UPDATE)
							{
								for (uint8_t i = 1; i < _rxHeaderData; i++)
									DGaddRouteTo(DGtemp_buf[i], DGtemp_buf[0], DGValid, _rxHeaderHop + 1);
							}*/

							// 패킷의 원래 목적지에 대한 데이터가 내 라우팅 테이블에 존재하지 않음 || 원래 목적지로 전송하기 위해서 next_hop으로 패킷을 보냈는데 ACK을 받지 못한 경우
							if (DGgetRouteTo(_rxHeaderDestination) == NULL
								|| !DGsendToWaitAck(_thisAddress, 
																		DGgetRouteTo(_rxHeaderDestination)->next_hop, 
																		_rxHeaderSource, 
																		_rxHeaderDestination, 
																		_rxHeaderType, 
																		NONE, 
																		NONE, 
																		NONE, 
																		_rxHeaderHop + 1, 
																		DGtemp_buf, 
																		sizeof(DGtemp_buf), 
																		TIME_TERM))
									// sendToWaitAck은 전송 후 ACK 받기를 기다림, ACK을 받지 못했다면 false를 return
							{
								// relay되어야 하는 패킷이 gateway로부터 시작했을 때 gateway에게 목적지로의 전송이 안되었다고 알려줌
								if (temp_source == (_thisAddress & 0x00))
								{
									printf("send NACK");
									DGtemp_buf[0] = DGgetRouteTo(_rxHeaderDestination)->next_hop;
									DGsendToWaitAck(_thisAddress, 
																	DGgetRouteTo(temp_source)->next_hop, 
																	_thisAddress, 
																	_thisAddress & 0x00, 
																	NACK, 
																	NONE, 
																	NONE, 
																	NONE, 
																	NONE, 
																	DGtemp_buf, 
																	sizeof(DGtemp_buf), 
																	TIME_TERM);
								}
							}
						} //end of else
					} // end of if (_rxHeaderDestination != _thisAddress && _rxHeaderTo != masterBroadcastAddress) //relay 상황

					// 아래의 경우는 실제 목적지가 자신이거나 브로드캐스트일 경우

					///////////////////////////////////////////////라우팅 관련 패킷//////////////////////////////////////////////////////////
					
					// <1hop>
					// 1. (G) -----REQUEST_BROADCAST---> (All master)
					// 2. (G) -------REQUEST_TYPE------> (1hop master)
					// 3. (G) <----REQUEST_ACK_TYPE----- (1hop master)
					
					else if (_rxHeaderType == REQUEST_BROADCAST)
					{
						//gateway가 라우팅을 시작한다는 패킷을 수신
						DGclearRoutingTable();
						printf("receivedBroadcastAddress : %d", receivedRequestNum);
						receivedRequestNum++;
						//REQUEST_BROADCAST를 몇 번 받았는지 count
					}
					else if (_rxHeaderType == REQUEST_TYPE && receivedRequestNum >= R_GATEWAY_SEND_NUM * 0.8)
					{
						//gateway로부터 1:1로 1hop 라우팅 요청 메시지 수신, REQUEST_BROADCAST 패킷을 80%이상 수신했을 경우에만 응답
						receivedRequestNum = 0;
						printf("receive row1 request");
						DGprintRoutingTable();
						DGaddRouteTo(_rxHeaderFrom, _rxHeaderFrom, Valid, 1, millis());
						//gateway를 라우팅 테이블에 등록시킴
						
						DGprintRoutingTable();
						//from, to, src, dst, type, data, flags, seqnum, hop
						for(uint8_t i = 0;i < R_MASTER_SEND_NUM;i++)
							DGsend(_thisAddress, 
										 _rxHeaderFrom, 
										 _thisAddress, 
										 _rxHeaderFrom, 
										 REQUEST_ACK_TYPE, 
										 NONE, 
										 NONE, 
										 NONE, 
										 NONE, 
										 DGtemp_buf, 
										 sizeof(DGtemp_buf));
					}
					
					
					// <2hop>
					// 1. (G) -----R2_REQUEST_TYPE-----> (1hop master)
					// 2. (G) <-----------ACK----------- (1hop master)
					// 3.                                (1hop master) -----R2_REQUEST_REAL_TYPE-----> (2hop master)
					// 4.                                (1hop master) <-----R2_REQUEST_ACK_TYPE------ (2hop master)
					// 5. (G) <--R2_REQUEST_ACK_TYPE---- (1hop master)
					else if (_rxHeaderType == R2_REQUEST_TYPE)
					{
						receivedRequestNum = 0;
						//from, to, src, dst, type, data, flags, seqnum, hop
						DGsend(_thisAddress, 
									 _rxHeaderFrom, 
									 _thisAddress, 
									 _rxHeaderFrom, 
									 ACK, 
									 NONE,
									 NONE, 
									 NONE, 
									 NONE, 
									 DGtemp_buf, 
									sizeof(DGtemp_buf));
						
						DGM_find2ndRowMasters(); //2 hop 마스터들을 찾아봄
					}
					else if (_rxHeaderType == R2_REQUEST_REAL_TYPE)
					{
						// 1hop 마스터에게 2hop 라우팅 요청 패킷을 수신
						receivedRequestNum = 0;
						printf("receive R2 routing request and I didn't success in R1 request");
						DGM_masterSendRoutingReply(); // 패킷을 보낸 마스터가 부모 노드로 선택된 노드면 응답을 보냄
					}
					
					
					
					// <multi hop>
					// 1. (G) --REQUEST_MULTI_HOP--> (1hop master)
					// 2. (G) <---------ACK--------- (1hop master)
					// 3.                            (1hop master) --REQUEST_MULTI_HOP--> (2hop master)
					// 4.                            (1hop master) <---------ACK--------- (2hop master)
					// 5.																																	(2hop master) --REQUEST_MULTI_HOP--> (3hop master)
					// 6.                            																			(2hop master) <REQUEST_MULTI_HOP_ACK (3hop master)
					// 7.                            (1hop master) <REQUEST_MULTI_HOP_ACK (2hop master)
					// 8.                            (1hop master) ----------ACK--------> (2hop master)
					// 9. (G) <REQUEST_MULTI_HOP_ACK (1hop master)
					//10. (G) ----------ACK--------> (1hop master)
					
					else if (_rxHeaderType == REQUEST_MULTI_HOP || _rxHeaderType == REQUEST_DIRECT)
					{
						receivedRequestNum = 0;

						//실제 라우팅하고자 하는 노드의 주소는 payload의 첫번째 바이트에 저장되어 있음
						
						if (DGtemp_buf[0] != _thisAddress)
						{
							//아직 routing 안된 노드에게 라우팅 요청 패킷 전송
							
							printf("received multihop request and send to unrouting Master");
							uint8_t realDst = DGtemp_buf[0];
							//from, to, src, dst, type, data, flags, seqnum, hop
							DGsend(_thisAddress, 
										 _rxHeaderFrom, 
										 _thisAddress, 
										 _rxHeaderFrom, 
										 ACK, 
										 NONE, 
										 NONE, 
										 NONE, 
										 NONE, 
										 DGtemp_buf, 
										 sizeof(DGtemp_buf));
							
							uint8_t cnt = 0;
							uint8_t type = _rxHeaderType;

							DGsend(_thisAddress, 
										 realDst, 
										 _thisAddress & 0x00, 
										 realDst, 
										 _rxHeaderType, 
										 NONE, 
										 NONE, 
										 NONE, 
										 _rxHeaderHop + 1, 
										 DGtemp_buf, 
										 sizeof(DGtemp_buf));
										 
							unsigned long startTime = millis();
							while (millis() - startTime < (R_MASTER_SEND_NUM) * TIME_TERM)
							{
								DGSetReceive();
								if (DGavailable() && DGrecvData(DGtemp_buf) && DGheaderFrom() == realDst && DGheaderTo() == _thisAddress 
									&& (DGheaderType() == REQUEST_MULTI_HOP_ACK || DGheaderType() == REQUEST_DIRECT_ACK))
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
									cnt++;
									if (cnt >= R_MASTER_SEND_NUM * 0.8)
									{
										DGaddRouteTo(_rxHeaderFrom, _rxHeaderFrom, Valid, 1, millis());
										for(uint8_t i = 0;i < _rxHeaderData;i++)
										{
											DGaddRouteTo(DGtemp_buf[i + 1], _rxHeaderFrom, Valid, 0, millis()); 
											// 자식노드의 hop count는 별로 중요하지 않음 일단 0으로 해두고 다음에 해당 자식노드로부터 패킷을 받게되면 hop count에 맞게 setting
										}
										DGprintRecvPacketHeader();
										DGprintRoutingTable();
										DGsendToWaitAck(_thisAddress, 
																		DGgetRouteTo(_thisAddress & 0x00)->next_hop, 
																		_rxHeaderSource, 
																		_thisAddress & 0x00, 
																		_rxHeaderType, 
																		1, 
																		NONE, 
																		NONE, 
																		_rxHeaderHop + 1, 
																		DGtemp_buf, 
																		sizeof(DGtemp_buf), 
																		TIME_TERM);
										break;
									}
								}
							}
							if (cnt < R_MASTER_SEND_NUM * 0.8)
							{
								if(type == REQUEST_MULTI_HOP)
									DGsendToWaitAck(_thisAddress, 
																	DGgetRouteTo(_thisAddress & 0x00)->next_hop, 
																	_thisAddress, 
																	_thisAddress & 0x00, 
																	REQUEST_MULTI_HOP_ACK, 
																	NONE, 
																	NONE, 
																	NONE, 
																	NONE, 
																	DGtemp_buf, 
																	sizeof(DGtemp_buf), 
																	TIME_TERM);
								else
									DGsendToWaitAck(_thisAddress, 
																	DGgetRouteTo(_thisAddress & 0x00)->next_hop, 
																	_thisAddress, 
																	_thisAddress & 0x00, 
																	REQUEST_DIRECT_ACK, 
																	NONE, 
																	NONE, 
																	NONE, 
																	NONE, 
																	DGtemp_buf, 
																	sizeof(DGtemp_buf), 
																	TIME_TERM);
							}
						}
						else
						{
							//routing 안되어있던 노드가 수신
							printf("received multi hop request");
							if(_rxHeaderType == REQUEST_MULTI_HOP)
								DGM_masterSendRoutingReply();
								
							else if(_rxHeaderType == REQUEST_DIRECT)
							{
								
								DGaddRouteTo(_rxHeaderSource, _rxHeaderFrom, Valid, _rxHeaderHop + 1, millis());
								DGprintRoutingTable();
								DGtemp_buf[0] = _rxHeaderFrom;
								
								// 조상 노드들이 현재 노드의 자식 노드들을 라우팅 테이블에 등록시키게 하기 위해서 자식 노드들도 payload에 추가
								uint8_t child_node[10];
								uint8_t child_node_cnt = DGfind_child_node(child_node);
								for(uint8_t i = 0;i < child_node_cnt;i++)
									DGtemp_buf[i + 1] = child_node[i];
								
								
								//from, to, src, dst, type, data, flags, seqnum, hop
								for (uint8_t i = 0; i < R_MASTER_SEND_NUM; i++)
									DGsend(_thisAddress, 
												 _rxHeaderFrom, 
												 _thisAddress, 
												 _thisAddress & 0x00, 
												 REQUEST_DIRECT_ACK, 
												 child_node_cnt, 
												 NONE, 
												 NONE, 
												 NONE, 
												 DGtemp_buf, 
												 sizeof(DGtemp_buf));
							}
						}
					}
					//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////




					////////////////////////////////////////////operate 관련 패킷////////////////////////////////////////////////////////////////////////

					// gateway로부터 master scan 요청 수신 (외부 마스터들만 scan)
					else if (_rxHeaderType == CHECK_ROUTING)
					{
						receivedRequestNum = 0;
						DGgetRouteTo(_thisAddress & 0x00)->hop = _rxHeaderHop + 1;
						
						// gateway가 라우팅 테이블에 등록되어 있을 경우에만(재부팅된 노드는 라우팅부터 다시 시작해야함)
						if (DGgetRouteTo(_thisAddress & 0x00) != NULL)
						{ 
							routing_complete = true;

							DGsend(_thisAddress,
										 _rxHeaderFrom,
									   _thisAddress,
										 _rxHeaderFrom,
										 ACK,
										 NONE,
										 NONE,
										 NONE,
										 NONE,
										 DGtemp_buf,
										 sizeof(DGtemp_buf));

							for(int i = 0;i < 10;i++)
								inputData[i] = DGtemp_buf[i];
								
							RS485_Write_Read();
							
							for(int i = 0;i < 10;i++)
								DGtemp_buf[i] = outputData[i];

							// 내부의 scan이 끝남 + master scan 완료 패킷을 게이트웨이에게 전송
							if(scanFinish)
							{
								DGsendToWaitAck(_thisAddress, 
																DGgetRouteTo(_thisAddress & 0x00)->next_hop, 
																_thisAddress, 
																_thisAddress & 0x00, 
																SCAN_RESPONSE_TO_GATEWAY, 
																NONE, 
																NONE, 
																NONE, 
																NONE, 
																DGtemp_buf, 
																sizeof(DGtemp_buf), 
																TIME_TERM);
							}
							
							// 새로운 노드가 라우팅 테이블에 추가됨 + scan 완료 패킷을 게이트웨이에게 전송
							else if (newNode)
							{
								int newNodeCnt = 0;
								for (uint8_t i = 0; i < ROUTING_TABLE_SIZE; i++)
									if (_routes[i].state == Discovering)
										DGtemp_buf[10 + newNodeCnt++] = _routes[i].dest;
								newNode = false;
								if(!DGsendToWaitAck(_thisAddress, 
																		DGgetRouteTo(_thisAddress & 0x00)->next_hop, 
																		_thisAddress, 
																		_thisAddress & 0x00, 
																		CHECK_ROUTING_ACK, 
																		newNodeCnt, 
																		NONE, 
																		NONE, 
																		NONE, 
																		DGtemp_buf, 
																		sizeof(DGtemp_buf), 
																		TIME_TERM))
									newNode = true; //gateway에게 전달이 안됐으면 다음에 다시 보냄
										
																		
								else // 전달이 됐으면 라우팅 테이블 상태를 바꿔줌
									for (uint8_t i = 0; i < ROUTING_TABLE_SIZE; i++)
										if (_routes[i].state == Discovering)
											_routes[i].state = Valid;
							}
							
							// 새로운 부모가 선택되었음 + scan 완료 패킷을 게이트웨이에게 전송
							/*else if (rerouting || _rxHeaderFrom != DGgetRouteTo(_thisAddress & 0x00)->next_hop)
							{
								// 게이트웨이에게 나의 부모 노드를 알려주기 위해서 내가 선택한 부모노드의 주소를 payload에 추가해줌
								
								DGtemp_buf[10] = rerouting_candidate;

								// 조상 노드들이 현재 노드의 자식 노드들을 라우팅 테이블에 등록시키게 하기 위해서 자식 노드들도 payload에 추가
								uint8_t child_node[10];
								uint8_t child_node_cnt = DGfind_child_node(child_node);

								for(uint8_t i = 0;i < child_node_cnt;i++)
									DGtemp_buf[11 + i] = child_node[i];
								
								if (DGsendToWaitAck(_thisAddress, 
																		rerouting_candidate, 
																		_thisAddress, 
																		_thisAddress & 0x00, 
																		CHECK_ROUTING_ACK_REROUTING, 
																		child_node_cnt, 
																		NONE, 
																		NONE, 
																		NONE, 
																		DGtemp_buf, 
																		sizeof(DGtemp_buf), 
																		TIME_TERM))
									DGaddRouteTo(_thisAddress & 0x00, rerouting_candidate, Valid, _rxHeaderHop + 1, millis());
								else
									rerouting_candidate = 0;
								rerouting = false;
							}*/

							// 일반 scan 완료 패킷 전송
							else
							{
								DGsendToWaitAck(_thisAddress, 
																DGgetRouteTo(_thisAddress & 0x00)->next_hop, 
																_thisAddress, 
																_thisAddress & 0x00, 
																CHECK_ROUTING_ACK, 
																NONE, 
																NONE, 
																NONE, 
																NONE, 
																DGtemp_buf, 
																sizeof(DGtemp_buf), 
																TIME_TERM);
							}
						}
						
						
						// gateway가 라우팅 테이블에 등록되어 있지 않음(reset된 상태)
						else
							DGprintRoutingTable();
							
					} // end of else if (_rxHeaderType == CHECK_ROUTING)

					// 새로운 노드가 자식 노드 설정 요청
					else if (_rxHeaderType == NEW_NODE_REGISTER && DGgetRouteTo(_thisAddress & 0x00) != NULL)
					{
						// 라우팅 테이블에 추가, 상태를 Discovering으로 하여 new node임을 표시
						DGaddRouteTo(_rxHeaderFrom, _rxHeaderFrom, Discovering, 1, millis());

						// ACK전송
						DGsend(_thisAddress, 
									 _rxHeaderFrom, 
									 _thisAddress, 
									 _rxHeaderFrom, 
									 ACK, 
									 NONE, 
									 NONE, 
									 NONE, 
									 DGgetRouteTo(_thisAddress & 0x00)->hop, 
									 DGtemp_buf, 
									 sizeof(DGtemp_buf));
						newNode = true;
					}

					// 내가 속한 zone의 scan 요청
					else if(_rxHeaderType == SCAN_REQUEST_TO_MASTER && DGgetRouteTo(_thisAddress & 0x00) != NULL)
					{
						DGsend(_thisAddress,
							_rxHeaderFrom,
							_thisAddress,
							_rxHeaderFrom,
							ACK,
							NONE,
							NONE,
							NONE,
							NONE,
							DGtemp_buf,
							sizeof(DGtemp_buf));

						// hop count 업데이트
						DGgetRouteTo(_thisAddress & 0x00)->hop = _rxHeaderHop + 1;

						// 내부 zone scan 명령을 잘 받았다고 응답을 보냄
						DGsendToWaitAck(_thisAddress, 
														DGgetRouteTo(_thisAddress & 0x00)->next_hop, 
														_thisAddress, 
														_thisAddress & 0x00, 
														SCAN_REQUEST_ACK_FROM_MASTER, 
														NONE, 
														NONE, 
														NONE, 
														NONE, 
														DGtemp_buf, 
														sizeof(DGtemp_buf), 
														TIME_TERM);

						USART_ITConfig(USART3, USART_IT_RXNE, DISABLE);
						//send scan to internal master in serial		
						USART_SendData(USART3, INTERNAL_SCAN);
						while(USART_GetFlagStatus(USART3, USART_FLAG_TC) == RESET);
						
						USART_ITConfig(USART3, USART_IT_RXNE, ENABLE);
						
					}
					
					//제어 요청
					else if(_rxHeaderType == CONTROL_TO_MASTER && DGgetRouteTo(_thisAddress & 0x00) != NULL)
					{
						byte temp[2];
						DGsend(_thisAddress, 
									 _rxHeaderFrom, 
									 _thisAddress, 
									 _rxHeaderFrom, 
									 ACK, 
									 NONE, 
									 NONE, 
									 NONE, 
									 NONE, 
									 temp, 
									 sizeof(temp));
						
						DGgetRouteTo(_thisAddress & 0x00)->hop = _rxHeaderHop + 1;
						
						for(int i = 0;i < 10;i++)
							inputData[i] = DGtemp_buf[i];
						
						RS485_Write_Read();
						
						for(int i = 0;i < 10;i++)
							DGtemp_buf[i] = outputData[i];
						
						USART_ITConfig(USART3, USART_IT_RXNE, DISABLE);
						//send control to internal master in serial
						USART_SendData(USART3, INTERNAL_CONTROL);
						while(USART_GetFlagStatus(USART3, USART_FLAG_TC) == RESET);
						
						
						timeout = true;
						//내부 마스터로부터 제어 명령 전송이 완료됐는지 데이터를 수신하기까지 대기
						
						unsigned long send_ctrl_to_in_master_time = millis();
						while(send_ctrl_to_in_master_time - millis() < 5000)
						{
							
							if(USART_GetFlagStatus(USART3, USART_FLAG_RXNE))
							{
								while(USART_GetFlagStatus(USART3, USART_FLAG_RXNE) == RESET);
								
								receiveFromInMaster = USART_ReceiveData(USART3);
								if(receivedFromInMaster == INTERNAL_CONTROL_FINISH)
								{
									DGsendToWaitAck(_thisAddress, 
																	DGgetRouteTo(_thisAddress & 0x00)->next_hop, 
																	_thisAddress, 
																	_thisAddress & 0x00, 
																	CONTROL_RESPONSE_TO_GATEWAY, 
																	NONE, 
																	NONE, 
																	NONE, 
																	NONE, 
																	DGtemp_buf, 
																	sizeof(DGtemp_buf), 
																	TIME_TERM);
									timeout = false;
								}
							}
						}
						USART_ITConfig(USART3, USART_IT_RXNE, ENABLE);
						if(timeout)
							DGsendToWaitAck(_thisAddress, 
															DGgetRouteTo(_thisAddress & 0x00)->next_hop, 
															_thisAddress, 
															_thisAddress & 0x00, 
															CONTROL_RESPONSE_TO_GATEWAY, 
															NONE, 
															NONE, 
															NONE, 
															NONE, 
															DGtemp_buf, 
															sizeof(DGtemp_buf), 
															TIME_TERM);  
					}  
				}
				////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

				else // overhear other's packet
				{
					if (priorPacketFrom == _rxHeaderFrom && priorPacketType == _rxHeaderType && _rxHeaderFrom != (_thisAddress & 0x00))
					{
						if (++receivedOverhearNum >= R_MASTER_SEND_NUM * 0.8)
							DGM_findCandidateParents();
					}
					else
						receivedOverhearNum = 1;
					priorPacketFrom = _rxHeaderFrom;
					priorPacketType = _rxHeaderType;
					if (_rxHeaderType == CHECK_ROUTING && _rxHeaderSource == (_thisAddress & 0x00) && _rxHeaderHop + 1 < DGgetRouteTo(_thisAddress & 0x00)->hop)
					{
						printf("overhearing other's packet to find better route");
						receivedNum[_rxHeaderFrom]++;
						printf("_rxHeaderFrom : %d\r\nreceivedNum : %d", _rxHeaderFrom, receivedNum[_rxHeaderFrom]);
					}
				} // end of type check
			} // end of if (DGrecvData(DGtemp_buf))
		} // end of if (DGavailable()) 데이터 수신
	}
}

/**
  * @brief  Configures the different system clocks.
  * @param  None
  * @retval : None
  */

void RCC_Configuration(void)
{
  /* Enable GPIOx clock */
  //RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOx | RCC_APB2Periph_GPIOB | RCC_APB2Periph_AFIO, ENABLE);

	/* Enable USARTx clocks */ 
  //RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE);
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
		if(temp == INTERNAL_SCAN_FINISH)
			scanFinish = true;
	}
}

void RS485_Write_Read()
{
	uint8_t num = 0, checkSum = 0;
	
	GPIO_SetBits(GPIOA, GPIO_Pin_1);

	for(int i = 0;i < 10;i++)
	{
		USART_SendData(USART2, inputData[i]);
		//while(USART_GetFlagStatus(USART2, USART_FLAG_TXE) == RESET);
		while(USART_GetFlagStatus(USART2, USART_FLAG_TC) == RESET);
	}

	GPIO_ResetBits(GPIOA, GPIO_Pin_1);
	
	rs485_loop_time = millis();
	while(rs485_loop_time + 2000 > millis())
	{
		if(USART_GetFlagStatus(USART2, USART_FLAG_RXNE))
		{
			//printf("hi");
			while(USART_GetFlagStatus(USART2, USART_FLAG_RXNE) == RESET);
			outputData[num++] = USART_ReceiveData(USART2);
		}
		if(num == 10)
			break;
	}
	printf("-----receive-----\r\n");
	for(int i = 0;i < 10;i++)
		printf("%x ", outputData[i]);
	printf("\r\n");
	for(int i = 0;i < 9;i++)
	{
		checkSum ^= outputData[i];
	}
	if(checkSum == outputData[9])
	{
		for(int i = 0;i < 10;i++)
		{
			outputData[i] = outputData[i];
			printf("%x ", outputData[i]);
		}
	}
	printf("\r\n");
}