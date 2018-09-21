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
uint8_t receivedNum[33] = { 0 };
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
		// if()//scan finish ??? ?? ?????? ??, ????? ???? ? ?
		{
			scanFinish = true;
		}

		// 10? ???? table update (???? ?? routing table? ?? ??)
		if (routing_complete && check_table_time + 600000 < millis())
		{
			DGcheckRoutingTable();
			check_table_time = millis();
		}

		// 10? ???? ?? ??? ?? ???? gateway? ? ??? ?? ??? ?? ?? ?? ??? ??
		// ?? ??? ??? ?? hop count? ? ??? receivedNum ?? ????? (line ???)
		// receivedNum ?? ?? ? ??? ?? ?? ??? ?
		
		if (check_rerouting_time + 60000 < millis())//10???..
		{
			printf("check_rerouting_time");
			check_rerouting_time = millis();
			int8_t max_receivedNum = 0;
			for (uint8_t i = 0; i <= DGmaster_num; i++)
			{
				printf("%d : %d\: ", i, receivedNum[i]);

				if (max_receivedNum < receivedNum[i])
				{
					rerouting_candidate = i;
					max_receivedNum = receivedNum[i];
					receivedNum[i] = 0;
					rerouting = true;
				}
			}
		}

		// ?? ?? ???
		DGSetReceive();

		// ?? ??
		if (DGavailable())
		{

			// ?? ??? ??? ??
			if (DGrecvData(DGtemp_buf))
			{

				// ??? ?? ??
				_rxHeaderTo = DGheaderTo();
				_rxHeaderFrom = DGheaderFrom();
				_rxHeaderSource = DGheaderSource();
				_rxHeaderDestination = DGheaderDestination();
				_rxHeaderType = DGheaderType();
				_rxHeaderData = DGheaderData();
				_rxHeaderFlags = DGheaderFlags();
				_rxHeaderSeqNum = DGheaderSeqNum();
				_rxHeaderHop = DGheaderHop();

				// ??? ? ??? ?? or ??????? ??
				if (_rxHeaderTo == _thisAddress || _rxHeaderTo == masterBroadcastAddress)
				{
					DGprintRecvPacketHeader();

					// ???? ?? ?? ?? (relay ??)
					if (_rxHeaderDestination != _thisAddress && _rxHeaderTo != masterBroadcastAddress)
					{
						uint8_t temp_source = _rxHeaderSource;
						printf("receive relay");

						// gateway? ???? ???? ?? ??, ?? ???? ?? ?? ??
						if (DGgetRouteTo(_thisAddress & 0x00) == NULL)
						{
							DGprintRoutingTable();
						}

						// gateway? ???? ???? ??, ???? ??? ??
						else
						{
							// ACK?? (hop-to-hop?? ACK? ????)
							DGsend(_thisAddress, _rxHeaderFrom, _thisAddress, _rxHeaderFrom, ACK, NONE, NONE, NONE, NONE, DGtemp_buf, sizeof(DGtemp_buf));

							// routing?? ??? ?? (? ????? ???? ?, ??? ??? ???? ????)
							if (_rxHeaderType == REQUEST_MULTI_HOP_ACK || _rxHeaderType == REQUEST_DIRECT_ACK)
							{
								DGaddRouteTo(_rxHeaderSource, _rxHeaderFrom, Valid, _rxHeaderHop + 1, millis());
								DGprintRoutingTable();
							}

							// ????? ?? ?, ?? ?? _rxHeaderData? ?? ??? ?? NEW_NODE? ??? ??, ??? ACK? ???? ? ????? ??? ??? ???? ??
							// _rxHeaderData? ???? ? ????? ??? ??, temp_buf[10]?? ????? ??? ????
							else if (_rxHeaderType == CHECK_ROUTING_ACK && _rxHeaderData > 0)
							{
								// hop count ???
								DGaddRouteTo(_rxHeaderSource, _rxHeaderFrom, Valid, _rxHeaderHop + 1, millis());
								for (uint8_t i = 0; i < _rxHeaderData; i++)
								{
									DGaddRouteTo(DGtemp_buf[10 + i], _rxHeaderFrom, Valid, _rxHeaderHop + 2, millis());
								}
							}

							//?? ? ????? ????? rerouting ?? ? ?????.. ??? ??? ???? ????
							else if (_rxHeaderType == CHECK_ROUTING_ACK_REROUTING)
							{
								DGaddRouteTo(_rxHeaderSource, _rxHeaderFrom, Valid, _rxHeaderHop + 1, millis());
								for(uint8_t i = 0;i < _rxHeaderData;i++)
								{
									DGaddRouteTo(DGtemp_buf[11 + i], _rxHeaderFrom, Valid, 0, millis()); // ????? hop count? ?? ???? ?? ?? 0?? ??? ??? ?? ??????? ??? ???? hop count?? setting
								}
							}

							
							/*else if(_rxHeaderType == ROUTING_TABLE_UPDATE)
							{
								for (uint8_t i = 1; i < _rxHeaderData; i++)
									DGaddRouteTo(DGtemp_buf[i], DGtemp_buf[0], DGValid, _rxHeaderHop + 1);
							}*/

							// ??? ?? ???? ?? ???? ? ??? ???? ???? ?? || ?? ???? ???? ??? next_hop?? ??? ???? ACK? ?? ?? ??
							if (DGgetRouteTo(_rxHeaderDestination) == NULL
								|| !DGsendToWaitAck(_thisAddress, DGgetRouteTo(_rxHeaderDestination)->next_hop, _rxHeaderSource, _rxHeaderDestination, _rxHeaderType, NONE, NONE, NONE, _rxHeaderHop + 1, DGtemp_buf, sizeof(DGtemp_buf), 2000))
									// sendToWaitAck? ?? ? ACK ??? ???, ACK? ?? ???? false? return
							{
								// relay??? ?? ??? gateway??? ???? ? gateway?? ????? ??? ????? ???
								if (temp_source == (_thisAddress & 0x00))
								{
									printf("send NACK");
									DGtemp_buf[0] = DGgetRouteTo(_rxHeaderDestination)->next_hop;
									DGsendToWaitAck(_thisAddress, DGgetRouteTo(temp_source)->next_hop, _thisAddress, _thisAddress & 0x00, NACK, NONE, NONE, NONE, NONE, DGtemp_buf, sizeof(DGtemp_buf), 2000);
								}
							}
						} //end of else
					} // end of if (_rxHeaderDestination != _thisAddress && _rxHeaderTo != masterBroadcastAddress)

					// ?? ??? ?? ???? ????? ?????????

					///////////////////////////////////////////////////////////////////////////////////////////////////////////////??? ?? ??///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
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

						//?? ?????? ?? ??? ??? payload? ??? ???? ???? ??
						
						if (DGtemp_buf[0] != _thisAddress)
						{
							//?? routing ?? ???? ??? ?? ?? ??
							
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
							//routing ? ???? ??? ??
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




					////////////////////////////////////////////////////////////////////////////////////////////////////////////operate ?? ??///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

					// gateway??? master scan ?? ??
					else if (_rxHeaderType == CHECK_ROUTING)
					{
						receivedRequestNum = 0;
						routing_complete = true;
						
						// gateway? ??? ???? ???? ?? ????(???? ??? ??? ????? ?? ?????)
						if (DGgetRouteTo(_thisAddress & 0x00) != NULL)
						{ 
							for(int i = 0;i < 10;i++)
								inputData[i] = DGtemp_buf[i];
								
							RS485_Write_Read();
							
							for(int i = 0;i < 10;i++)
								DGtemp_buf[i] = outputData[i];

							// ??? scan? ?? + master scan ?? ??? ??????? ??
							if(scanFinish)
							{
								DGgetRouteTo(_thisAddress & 0x00)->hop = _rxHeaderHop + 1;
								DGsendToWaitAck(_thisAddress, DGgetRouteTo(_thisAddress & 0x00)->next_hop, _thisAddress, _thisAddress & 0x00, SCAN_RESPONSE_TO_GATEWAY, NONE, NONE, NONE, NONE, DGtemp_buf, sizeof(DGtemp_buf), 2000);
							}
							
							// ??? ??? ??? ???? ??? + scan ?? ??? ??????? ??
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
								DGsendToWaitAck(_thisAddress, DGgetRouteTo(_thisAddress & 0x00)->next_hop, _thisAddress, _thisAddress & 0x00, CHECK_ROUTING_ACK, newNodeCnt, NONE, NONE, NONE, DGtemp_buf, sizeof(DGtemp_buf), 2000);
							}
							
							// ??? ??? ????? + scan ?? ??? ??????? ??
							else if (rerouting || _rxHeaderFrom != DGgetRouteTo(_thisAddress & 0x00)->next_hop)
							{
								// ??????? ?? ?? ??? ???? ??? ?? ??? ????? ??? payload? ????
								// ????? ????? ??? ????? ???? hop count? ???? ??? ? ? ???? ?? ????? ?
								
								DGtemp_buf[10] = rerouting_candidate;

								// ?? ???? ?? ??? ?? ???? ??? ???? ????? ?? ??? ?? ???? payload? ??
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

							// ?? scan ?? ?? ??
							else
							{
								// hop count update
								DGgetRouteTo(_thisAddress & 0x00)->hop = _rxHeaderHop + 1;
								DGsendToWaitAck(_thisAddress, DGgetRouteTo(_thisAddress & 0x00)->next_hop, _thisAddress, _thisAddress & 0x00, CHECK_ROUTING_ACK, NONE, NONE, NONE, NONE, DGtemp_buf, sizeof(DGtemp_buf), 2000);
							}
						}
						// gateway? ??? ???? ???? ?? ??(reset? ??)
						else
							DGprintRoutingTable();
							
					} // end of else if (_rxHeaderType == CHECK_ROUTING)

					// ??? ??? ?? ?? ?? ??
					else if (_rxHeaderType == NEW_NODE_REGISTER && DGgetRouteTo(_thisAddress & 0x00) != NULL)
					{
						// ??? ???? ??, ??? Discovering?? ?? new node?? ??
						DGaddRouteTo(_rxHeaderFrom, _rxHeaderFrom, Discovering, 1, millis());

						// ACK??
						DGsend(_thisAddress, _rxHeaderFrom, _thisAddress, _rxHeaderFrom, ACK, NONE, NONE, NONE, DGgetRouteTo(_thisAddress & 0x00)->hop, DGtemp_buf, sizeof(DGtemp_buf));
						newNode = true;
					}

					
					/*else if (_rxHeaderType == ROUTING_TABLE_UPDATE && DGgetRouteTo(_thisAddress & 0x00) != NULL)
					{
						for (uint8_t i = 1; i < _rxHeaderData; i++)
							DGaddRouteTo(DGtemp_buf[i], DGtemp_buf[0], DGValid, _rxHeaderHop + 1);
						DGsend(_thisAddress, _rxHeaderFrom, _thisAddress, _rxHeaderFrom, ACK, NONE, NONE, NONE, NONE, DGtemp_buf, sizeof(DGtemp_buf));
					}*/
				 


					// ?? ?? zone? scan ??
					else if(_rxHeaderType == SCAN_REQUEST_TO_MASTER && DGgetRouteTo(_thisAddress & 0x00) != NULL)
					{
						// hop count ????
						DGgetRouteTo(_thisAddress & 0x00)->hop = _rxHeaderHop + 1;

						// ?? zone scan ??? ? ???? ??? ??
						DGsendToWaitAck(_thisAddress, DGgetRouteTo(_thisAddress & 0x00)->next_hop, _thisAddress, _thisAddress & 0x00, SCAN_REQUEST_ACK_FROM_MASTER, NONE, NONE, NONE, NONE, DGtemp_buf, sizeof(DGtemp_buf), 2000);

						//send scan to internal master in serial		
						USART_SendData(USART3, INTERNAL_SCAN);
						while(USART_GetFlagStatus(USART3, USART_FLAG_TC) == RESET);
						
					}
					
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
							
						//send control to internal master in serial
						USART_SendData(USART3, INTERNAL_CONTROL);
						while(USART_GetFlagStatus(USART3, USART_FLAG_TC) == RESET);
						
						
						timeout = true;
						//?? ?????? ?? ?? ??? ????? ???? ?????? ??
						
						unsigned long send_ctrl_to_in_master_time = millis();
						while(send_ctrl_to_in_master_time - millis() < 5000)
						{
							
							if(USART_GetFlagStatus(USART3, USART_FLAG_RXNE))
							{
								while(USART_GetFlagStatus(USART3, USART_FLAG_RXNE) == RESET);
								
								receiveFromInMaster = USART_ReceiveData(USART3);
								DGsendToWaitAck(_thisAddress, DGgetRouteTo(_thisAddress & 0x00)->next_hop, _thisAddress, _thisAddress & 0x00, CONTROL_RESPONSE_TO_GATEWAY, NONE, NONE, NONE, NONE, DGtemp_buf, sizeof(DGtemp_buf), 2000);
								timeout = false;
							}
						}
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
		} // end of if (DGavailable()) ??? ??
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

  /* Enable the USARTx */
	USART_Cmd(USART3, ENABLE);
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
	while(rs485_loop_time + 3000 > millis())
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