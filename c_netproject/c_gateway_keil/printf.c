#include "printf.h"
/****************************************************************
*FUNCTION NAME:PrintfInit
*FUNCTION     :uart communication initialization for printf
*INPUT        :none
*OUTPUT       :none
****************************************************************/
void PrintfInit(void)
{    
  /* Enable GPIOx clock */
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOx, ENABLE);

  /* Enable USARTx clocks */ 
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE);
	
	
	GPIO_InitTypeDef GPIO_InitStructure;

  /* Configure USARTx_Tx as alternate function push-pull */
  GPIO_InitStructure.GPIO_Pin = GPIO_TxPin;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
  GPIO_Init(GPIOx, &GPIO_InitStructure);

  /* Configure USARTx_Rx as input floating */
  GPIO_InitStructure.GPIO_Pin = GPIO_RxPin;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
  GPIO_Init(GPIOx, &GPIO_InitStructure);


	USART_InitTypeDef USART_InitStructure;
	
	USART_InitStructure.USART_BaudRate = 9600;
  USART_InitStructure.USART_WordLength = USART_WordLength_8b;
  USART_InitStructure.USART_StopBits = USART_StopBits_1;
  USART_InitStructure.USART_Parity = USART_Parity_No ;
  USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
  USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
  
  /* Configure the USARTx */ 
  USART_Init(USARTx, &USART_InitStructure);
  /* Enable the USARTx */
  USART_Cmd(USARTx, ENABLE);
}
/**
  * @brief  Retargets the C library printf function to the USART.
  * @param  None
  * @retval : None
  */
PUTCHAR_PROTOTYPE
{
  /* Write a character to the USART */
  USART_SendData(USARTx, (uint8_t) ch);

  /* Loop until the end of transmission */
  while(USART_GetFlagStatus(USARTx, USART_FLAG_TC) == RESET)
  {
  }

  return ch;
}