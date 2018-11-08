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

uint8_t receivedRequestNum = 0;
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

bool DGcheckReceive[34];// = { false };
RoutingTableEntry    _routes[ROUTING_TABLE_SIZE];
int DGmaster_num;// = 2;
int8_t DGparentMaster[34];// = { -1 };
byte DGunRecvCnt[34];// = { 0 };
byte DGtemp_buf[20];
byte DGbuffer[20];

signed char e_rssi;

int main()
{
	DGInit(0x00, 0x01, 15);
	_thisAddress = DGgetThisAddress();
	//master_number = DGgetMasterNumber();

	
	/* System Clocks Configuration */
  RCC_Configuration();
  /* Configure the GPIO ports */
  GPIO_Configuration();
	
	USART2_Configuration();
	USART3_Configuration();
	
	DGSetReceive();

	while(1)
	{
		// 10�� �������� table update (������ �ʴ� routing table�� ���� ����)
		if (routing_complete && check_table_time + 600000 < millis())
		{
			DGcheckRoutingTable();
			check_table_time = millis();
		}

		// 10�� �������� ���� ������ �θ� ��庸�� gateway�� �� ������ �ִ� ��尡 �ִ� ��� �ٽ� ����� ����
		// �ٸ� ����� ��Ŷ�� ���� hop count�� �� ���ٸ� receivedNum ���� ���������� (line ???)
		// receivedNum ���� ���� ū ��尡 �ĺ� �θ� ��尡 ��
		
		if (check_rerouting_time + 600000 < millis())//10������..
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

		// ���� ���� ���·�
		DGSetReceive();

		// ��Ŷ ����
		if (DGavailable())
		{

			// ���� ������ ���ۿ� ����
			if (DGrecvData(DGtemp_buf))
			{

				// ����� ���� ����
				_rxHeaderTo = DGheaderTo();
				_rxHeaderFrom = DGheaderFrom();
				_rxHeaderSource = DGheaderSource();
				_rxHeaderDestination = DGheaderDestination();
				_rxHeaderType = DGheaderType();
				_rxHeaderData = DGheaderData();
				_rxHeaderFlags = DGheaderFlags();
				_rxHeaderSeqNum = DGheaderSeqNum();
				_rxHeaderHop = DGheaderHop();

				// ������ �� ��Ŷ�� ��� or ��ε�ĳ��Ʈ�� ���
				if (_rxHeaderTo == _thisAddress || _rxHeaderTo == masterBroadcastAddress)
				{
					DGprintRecvPacketHeader();

					// �������� ���� �ƴ� ���(relay ��Ȳ)
					if (_rxHeaderDestination != _thisAddress && _rxHeaderTo != masterBroadcastAddress)
					{
						uint8_t temp_source = _rxHeaderSource;
						printf("receive relay");

						// gateway�� ���̺� ��ϵǾ� ���� ����, ���� ������� ���� ���� ��� -> �ƹ��͵� ����
						if (DGgetRouteTo(_thisAddress & 0x00) == NULL)
						{
							DGprintRoutingTable();
						}

						// gateway�� ���̺� ��ϵǾ� ����, ������� �Ϸ�� ���
						else
						{
							// ACK���� (hop-to-hop���� ACK�� ��������)
							DGsend(_thisAddress, _rxHeaderFrom, _thisAddress, _rxHeaderFrom, ACK, NONE, NONE, NONE, NONE, DGtemp_buf, sizeof(DGtemp_buf));

							// routing���� ������ ���� (�� �ڽĳ�尡 ����ٴ� ��, ���� ����� ���̺� ��Ͻ�Ŵ) = ���⸵ũ���� ��Ŷ��
							if (_rxHeaderType == REQUEST_MULTI_HOP_ACK || _rxHeaderType == REQUEST_DIRECT_ACK)
							{
								DGaddRouteTo(_rxHeaderSource, _rxHeaderFrom, Valid, _rxHeaderHop + 1, millis());
								DGprintRoutingTable();
							}

							// �ʱ����� �Ϸ� ��, � �߿� _rxHeaderData�� ���� �ִٴ� ���� NEW_NODE�� �������ǹ�, ������ ACK�� �����Ƿ� �� �ڽĳ���� ���� ����� ���̺� ���
							// _rxHeaderData�� �߰��ؾ� �� �ڽĳ���� ������ �ǹ�, temp_buf[10]���� �ڽĳ���� �ּҰ� ������� (temp_buf[0] ~ [9]�� ������ ��������)
							else if (_rxHeaderType == CHECK_ROUTING_ACK && _rxHeaderData > 0)
							{
								// hop count �缳��
								DGaddRouteTo(_rxHeaderSource, _rxHeaderFrom, Valid, _rxHeaderHop + 1, millis());
								for (uint8_t i = 0; i < _rxHeaderData; i++)
								{
									DGaddRouteTo(DGtemp_buf[10 + i], _rxHeaderFrom, Valid, _rxHeaderHop + 2, millis());
								}
							}

							//���� �� �ڽĳ�尡 �ƴϾ��µ� rerouting ��� �� �ڽĳ���.. ���� ����� ���̺� �������
							else if (_rxHeaderType == CHECK_ROUTING_ACK_REROUTING)
							{
								DGaddRouteTo(_rxHeaderSource, _rxHeaderFrom, Valid, _rxHeaderHop + 1, millis());
								for(uint8_t i = 0;i < _rxHeaderData;i++)
								{
									DGaddRouteTo(DGtemp_buf[11 + i], _rxHeaderFrom, Valid, 0, millis()); // �ڽĳ���� hop count�� ���� �߿����� ���� �ϴ� 0���� �صΰ� ������ �ش� �ڽĳ��κ��� ��Ŷ�� �ްԵǸ� hop count�� �°� setting
								}
							}

							
							/*else if(_rxHeaderType == ROUTING_TABLE_UPDATE)
							{
								for (uint8_t i = 1; i < _rxHeaderData; i++)
									DGaddRouteTo(DGtemp_buf[i], DGtemp_buf[0], DGValid, _rxHeaderHop + 1);
							}*/

							// ��Ŷ�� ���� �������� ���� �����Ͱ� �� ����� ���̺� �������� ���� || ���� �������� �����ϱ� ���ؼ� next_hop���� ��Ŷ�� ���´µ� ACK�� ���� ���� ���
							if (DGgetRouteTo(_rxHeaderDestination) == NULL
								|| !DGsendToWaitAck(_thisAddress, DGgetRouteTo(_rxHeaderDestination)->next_hop, _rxHeaderSource, _rxHeaderDestination, _rxHeaderType, NONE, NONE, NONE, _rxHeaderHop + 1, DGtemp_buf, sizeof(DGtemp_buf), 2000))
									// sendToWaitAck�� ���� �� ACK �ޱ⸦ ��ٸ�, ACK�� ���� ���ߴٸ� false�� return
							{
								// relay�Ǿ�� �ϴ� ��Ŷ�� gateway�κ��� �������� �� gateway���� ���������� ������ �ȵǾ��ٰ� �˷���
								if (temp_source == (_thisAddress & 0x00))
								{
									printf("send NACK");
									DGtemp_buf[0] = DGgetRouteTo(_rxHeaderDestination)->next_hop;
									DGsendToWaitAck(_thisAddress, DGgetRouteTo(temp_source)->next_hop, _thisAddress, _thisAddress & 0x00, NACK, NONE, NONE, NONE, NONE, DGtemp_buf, sizeof(DGtemp_buf), 2000);
								}
							}
						} //end of else
					} // end of if (_rxHeaderDestination != _thisAddress && _rxHeaderTo != masterBroadcastAddress) //relay ��Ȳ

					// �Ʒ��� ���� ���� �������� �ڽ��̰ų� ��ε�ĳ��Ʈ�� ���

					///////////////////////////////////////////////////////////////////////////////////////////////////////////////����� ���� ��Ŷ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
					else if (_rxHeaderType == REQUEST_BROADCAST)
					{
						DGclearRoutingTable();
						printf("receivedBroadcastAddress : %d", receivedRequestNum);
						receivedRequestNum++;
					}
					else if (_rxHeaderType == REQUEST_TYPE && receivedRequestNum >= R_GATEWAY_SEND_NUM * 0.8)
					{
						receivedRequestNum = 0;
						printf("receive row1 request");
						DGprintRoutingTable();
						DGaddRouteTo(_rxHeaderFrom, _rxHeaderFrom, Valid, 1, millis());
						DGprintRoutingTable();
						//from, to, src, dst, type, data, flags, seqnum, hop
						for(uint8_t i = 0;i < R_MASTER_SEND_NUM;i++)
							DGsend(_thisAddress, _rxHeaderFrom, _thisAddress, _rxHeaderFrom, REQUEST_ACK_TYPE, NONE, NONE, NONE, NONE, DGtemp_buf, sizeof(DGtemp_buf));
					}
					else if (_rxHeaderType == R2_REQUEST_TYPE)
					{
						receivedRequestNum = 0;
						//from, to, src, dst, type, data, flags, seqnum, hop
						DGsend(_thisAddress, _rxHeaderFrom, _thisAddress, _rxHeaderFrom, ACK, NONE, NONE, NONE, NONE, DGtemp_buf, sizeof(DGtemp_buf));
						DGM_find2ndRowMasters();
					}
					else if (_rxHeaderType == R2_REQUEST_REAL_TYPE)
					{
						receivedRequestNum = 0;
						printf("receive R2 routing request and I didn't success in R1 request");
						DGM_masterSendRoutingReply();
					}
					else if (_rxHeaderType == REQUEST_MULTI_HOP || _rxHeaderType == REQUEST_DIRECT)
					{
						receivedRequestNum = 0;

						//���� ������ϰ��� �ϴ� ����� �ּҴ� payload�� ù��° ����Ʈ�� ����Ǿ� ����
						
						if (DGtemp_buf[0] != _thisAddress)
						{
							//���� routing �ȵ� ��忡�� ����� ��û ��Ŷ ����
							
							printf("received multihop request and send to unrouting Master");
							uint8_t realDst = DGtemp_buf[0];
							//from, to, src, dst, type, data, flags, seqnum, hop
							DGsend(_thisAddress, _rxHeaderFrom, _thisAddress, _rxHeaderFrom, ACK, NONE, NONE, NONE, NONE, DGtemp_buf, sizeof(DGtemp_buf));
							uint8_t cnt = 0;
							uint8_t type = _rxHeaderType;

							DGsend(_thisAddress, realDst, _thisAddress & 0x00, realDst, _rxHeaderType, NONE, NONE, NONE, _rxHeaderHop + 1, DGtemp_buf, sizeof(DGtemp_buf));
							unsigned long startTime = millis();
							while (millis() - startTime < (R_MASTER_SEND_NUM) * TIME_TERM)
							{
								DGSetReceive();
								if (DGavailable() && DGrecvData(DGtemp_buf) && DGheaderFrom() == realDst && DGheaderTo() == _thisAddress && (DGheaderType() == REQUEST_MULTI_HOP_ACK || DGheaderType() == REQUEST_DIRECT_ACK))
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
										DGprintRecvPacketHeader();
										DGprintRoutingTable();
										DGsendToWaitAck(_thisAddress, DGgetRouteTo(_thisAddress & 0x00)->next_hop, _rxHeaderSource, _thisAddress & 0x00, _rxHeaderType, 1, NONE, NONE, _rxHeaderHop + 1, DGtemp_buf, sizeof(DGtemp_buf), 2000);
										break;
									}
								}
							}
							if (cnt < R_MASTER_SEND_NUM * 0.8)
							{
								if(type == REQUEST_MULTI_HOP)
									DGsendToWaitAck(_thisAddress, DGgetRouteTo(_thisAddress & 0x00)->next_hop, _thisAddress, _thisAddress & 0x00, REQUEST_MULTI_HOP_ACK, NONE, NONE, NONE, NONE, DGtemp_buf, sizeof(DGtemp_buf), 2000);
								else
									DGsendToWaitAck(_thisAddress, DGgetRouteTo(_thisAddress & 0x00)->next_hop, _thisAddress, _thisAddress & 0x00, REQUEST_DIRECT_ACK, NONE, NONE, NONE, NONE, DGtemp_buf, sizeof(DGtemp_buf), 2000);
							}
						}
						else
						{
							//routing �ȵǾ��ִ� ��尡 ����
							printf("received multi hop request");
							if(_rxHeaderType == REQUEST_MULTI_HOP)
								DGM_masterSendRoutingReply();
								
							else if(_rxHeaderType == REQUEST_DIRECT)
							{
								DGaddRouteTo(_rxHeaderSource, _rxHeaderFrom, Valid, _rxHeaderHop + 1, millis());
								DGprintRoutingTable();
								DGtemp_buf[0] = _rxHeaderFrom;
								//from, to, src, dst, type, data, flags, seqnum, hop
								for (uint8_t i = 0; i < R_MASTER_SEND_NUM; i++)
									DGsend(_thisAddress, _rxHeaderFrom, _thisAddress, _thisAddress & 0x00, REQUEST_DIRECT_ACK, NONE, NONE, NONE, NONE, DGtemp_buf, sizeof(DGtemp_buf));
							}
						}
					}
					///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////




					////////////////////////////////////////////////////////////////////////////////////////////////////////////operate ���� ��Ŷ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

					// gateway�κ��� master scan ��û ���� (�ܺ� �����͵鸸 scan)
					else if (_rxHeaderType == CHECK_ROUTING)
					{
						receivedRequestNum = 0;
						
						// gateway�� ����� ���̺� ��ϵǾ� ���� ��쿡��(����õ� ���� ����ú��� �ٽ� �����ؾ���)
						if (DGgetRouteTo(_thisAddress & 0x00) != NULL)
						{ 
							routing_complete = true;
							for(int i = 0;i < 10;i++)
								inputData[i] = DGtemp_buf[i];
								
							RS485_Write_Read();
							
							for(int i = 0;i < 10;i++)
								DGtemp_buf[i] = outputData[i];

							// ������ scan�� ���� + master scan �Ϸ� ��Ŷ�� ����Ʈ���̿��� ����
							if(scanFinish)
							{
								DGgetRouteTo(_thisAddress & 0x00)->hop = _rxHeaderHop + 1;
								DGsendToWaitAck(_thisAddress, DGgetRouteTo(_thisAddress & 0x00)->next_hop, _thisAddress, _thisAddress & 0x00, SCAN_RESPONSE_TO_GATEWAY, NONE, NONE, NONE, NONE, DGtemp_buf, sizeof(DGtemp_buf), 2000);
							}
							
							// ���ο� ��尡 ����� ���̺� �߰��� + scan �Ϸ� ��Ŷ�� ����Ʈ���̿��� ����
							else if (newNode)
							{
								int newNodeCnt = 0;
								for (uint8_t i = 0; i < ROUTING_TABLE_SIZE; i++)
								{
									if (_routes[i].state == Discovering)
									{
										DGtemp_buf[10 + newNodeCnt++] = _routes[i].dest;
										_routes[i].state = Valid;
									}
								}
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
															 sizeof(DGtemp_buf), 2000))
									
								newNode = true; //gateway���� ������ �ȵ����� ������ �ٽ� ����
							}
							
							// ���ο� �θ� ���õǾ��� + scan �Ϸ� ��Ŷ�� ����Ʈ���̿��� ����
							else if (rerouting || _rxHeaderFrom != DGgetRouteTo(_thisAddress & 0x00)->next_hop)
							{
								// ����Ʈ���̿��� ���� �θ� ��带 �˷��ֱ� ���ؼ� ���� ������ �θ����� �ּҸ� payload�� �߰�����
								
								DGtemp_buf[10] = rerouting_candidate;

								// ���� ������ ���� ����� �ڽ� ������ ����� ���̺� ��Ͻ�Ű�� �ϱ� ���ؼ� �ڽ� ���鵵 payload�� �߰�
								uint8_t child_node[10];
								uint8_t child_node_cnt = DGfind_child_node(child_node);

								for(uint8_t i = 0;i < child_node_cnt;i++)
									DGtemp_buf[11 + i] = child_node[i];
								
								if (DGsendToWaitAck(_thisAddress, rerouting_candidate, _thisAddress, _thisAddress & 0x00, CHECK_ROUTING_ACK_REROUTING, child_node_cnt, NONE, NONE, NONE, DGtemp_buf, sizeof(DGtemp_buf), 2000))
									DGaddRouteTo(_thisAddress & 0x00, rerouting_candidate, Valid, _rxHeaderHop + 1, millis());
								else
									rerouting_candidate = 0;
								rerouting = false;
							}

							// �Ϲ� scan �Ϸ� ��Ŷ ����
							else
							{
								// hop count update
								DGgetRouteTo(_thisAddress & 0x00)->hop = _rxHeaderHop + 1;
								DGsendToWaitAck(_thisAddress, DGgetRouteTo(_thisAddress & 0x00)->next_hop, _thisAddress, _thisAddress & 0x00, CHECK_ROUTING_ACK, NONE, NONE, NONE, NONE, DGtemp_buf, sizeof(DGtemp_buf), 2000);
							}
						}
						// gateway�� ����� ���̺� ��ϵǾ� ���� ����(reset�� ����)
						else
							DGprintRoutingTable();
							
					} // end of else if (_rxHeaderType == CHECK_ROUTING)

					// ���ο� ��尡 �ڽ� ��� ���� ��û
					else if (_rxHeaderType == NEW_NODE_REGISTER && DGgetRouteTo(_thisAddress & 0x00) != NULL)
					{
						// ����� ���̺� �߰�, ���¸� Discovering���� �Ͽ� new node���� ǥ��
						DGaddRouteTo(_rxHeaderFrom, _rxHeaderFrom, Discovering, 1, millis());

						// ACK����
						DGsend(_thisAddress, _rxHeaderFrom, _thisAddress, _rxHeaderFrom, ACK, NONE, NONE, NONE, DGgetRouteTo(_thisAddress & 0x00)->hop, DGtemp_buf, sizeof(DGtemp_buf));
						newNode = true;
					}

					
					/*else if (_rxHeaderType == ROUTING_TABLE_UPDATE && DGgetRouteTo(_thisAddress & 0x00) != NULL)
					{
						for (uint8_t i = 1; i < _rxHeaderData; i++)
							DGaddRouteTo(DGtemp_buf[i], DGtemp_buf[0], DGValid, _rxHeaderHop + 1);
						DGsend(_thisAddress, _rxHeaderFrom, _thisAddress, _rxHeaderFrom, ACK, NONE, NONE, NONE, NONE, DGtemp_buf, sizeof(DGtemp_buf));
					}*/
				 


					// ���� ���� zone�� scan ��û
					else if(_rxHeaderType == SCAN_REQUEST_TO_MASTER && DGgetRouteTo(_thisAddress & 0x00) != NULL)
					{
						// hop count ������Ʈ
						DGgetRouteTo(_thisAddress & 0x00)->hop = _rxHeaderHop + 1;

						// ���� zone scan ����� �� �޾Ҵٰ� ������ ����
						DGsendToWaitAck(_thisAddress, DGgetRouteTo(_thisAddress & 0x00)->next_hop, _thisAddress, _thisAddress & 0x00, SCAN_REQUEST_ACK_FROM_MASTER, NONE, NONE, NONE, NONE, DGtemp_buf, sizeof(DGtemp_buf), 2000);

						USART_ITConfig(USART3, USART_IT_RXNE, DISABLE);
						//send scan to internal master in serial		
						USART_SendData(USART3, INTERNAL_SCAN);
						while(USART_GetFlagStatus(USART3, USART_FLAG_TC) == RESET);
						
						USART_ITConfig(USART3, USART_IT_RXNE, ENABLE);
						
					}
					
					//���� ��û
					else if(_rxHeaderType == CONTROL_TO_MASTER && DGgetRouteTo(_thisAddress & 0x00) != NULL)
					{
						byte temp[2];
						DGsend(_thisAddress, _rxHeaderFrom, _thisAddress, _rxHeaderFrom, ACK, NONE, NONE, NONE, NONE, temp, sizeof(temp));
						
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
						//���� �����ͷκ��� ���� ��� ������ �Ϸ�ƴ��� �����͸� �����ϱ���� ���
						
						unsigned long send_ctrl_to_in_master_time = millis();
						while(send_ctrl_to_in_master_time - millis() < 5000)
						{
							
							if(USART_GetFlagStatus(USART3, USART_FLAG_RXNE))
							{
								while(USART_GetFlagStatus(USART3, USART_FLAG_RXNE) == RESET);
								
								receiveFromInMaster = USART_ReceiveData(USART3);
								if(receivedFromInMaster == INTERNAL_CONTROL_FINISH)
								{
									DGsendToWaitAck(_thisAddress, DGgetRouteTo(_thisAddress & 0x00)->next_hop, _thisAddress, _thisAddress & 0x00, CONTROL_RESPONSE_TO_GATEWAY, NONE, NONE, NONE, NONE, DGtemp_buf, sizeof(DGtemp_buf), 2000);
									timeout = false;
								}
							}
						}
						USART_ITConfig(USART3, USART_IT_RXNE, ENABLE);
						if(timeout)
							DGsendToWaitAck(_thisAddress, DGgetRouteTo(_thisAddress & 0x00)->next_hop, _thisAddress, _thisAddress & 0x00, CONTROL_RESPONSE_TO_GATEWAY, NONE, NONE, NONE, NONE, DGtemp_buf, sizeof(DGtemp_buf), 2000);  
					}  
				}
				///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

				else
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
						printf("hi");
						receivedNum[_rxHeaderFrom]++;
						printf("_rxHeaderFrom : %d\r\nreceivedNum : %d", _rxHeaderFrom, receivedNum[_rxHeaderFrom]);
					}
				} // end of type check
			} // end of if (DGrecvData(DGtemp_buf))
		} // end of if (DGavailable()) ������ ����
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