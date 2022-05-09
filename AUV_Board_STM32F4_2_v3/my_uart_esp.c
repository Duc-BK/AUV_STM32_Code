#include "my_uart_esp.h"

		#define		USARTx_ESP_TX_CLK_INIT		  								RCC_AHB1PeriphClockCmd
		#define 	USARTx_ESP_TX_GPIO_CLK											RCC_AHB1Periph_GPIOC
		#define 	USARTx_ESP_TX_GPIO_PORT 					 					GPIOC
		#define		USARTx_ESP_TX_PIN														GPIO_Pin_10
		#define 	USARTx_ESP_TX_SOURCE											 	GPIO_PinSource10
		
		#define		USARTx_ESP_RX_CLK_INIT		  	 							RCC_AHB1PeriphClockCmd
		#define 	USARTx_ESP_RX_GPIO_CLK											RCC_AHB1Periph_GPIOC
		#define 	USARTx_ESP_RX_GPIO_PORT 				  					GPIOC
		#define		USARTx_ESP_RX_PIN														GPIO_Pin_11
		#define 	USARTx_ESP_RX_SOURCE					 							GPIO_PinSource11
		
		#define 	USARTx_ESP_DMA_CLK													RCC_AHB1Periph_DMA1
		#define 	USARTx_ESP_DMA_CLK_Cmd											RCC_AHB1PeriphClockCmd
		#define 	USARTx_ESP_DMA_Channel											DMA_Channel_4
		#define 	USARTx_ESP_DMA_Stream												DMA1_Stream2
		#define 	USARTx_ESP_DMA_Stream_IRQn									DMA1_Stream2_IRQn
		#define 	USARTx_ESP_DMA_IRQPreemptionPriority				0x00
		#define 	USARTx_ESP_DMA_IRQSubPriority								0x05
		#define 	USARTx_ESP_DMA_Stream_IRQHandler						DMA1_Stream2_IRQHandler
		
		#define		USARTx_ESP																	UART4
		#define 	USARTx_ESP_CLK															RCC_APB1Periph_UART4
		#define		USARTx_ESP_CLK_INIT		 											RCC_APB1PeriphClockCmd
		#define 	USARTx_ESP_BAUDRATE													115200
		#define		USARTx_ESP_AF																GPIO_AF_UART4
		#define 	USARTx_ESP_IRQn															UART4_IRQn
		#define 	USARTx_ESP_IRQPreemptionPriority						0x00
		#define 	USARTx_ESP_IRQSubPriority										0x03

		#define		USARTx_ESP_IRQHandler												UART4_IRQHandler

#define PACKAGE_SIZE 12
#define USARTx_ESP_DMA_BUFFER_LENGTH 5

uint8_t data_init = 0;
bool enable_sending_package = false;
uint8_t byte_data = 0;
bool esp = true;
static uint8_t package[PACKAGE_SIZE];
static uint8_t UART_ESP_DMABuffer[USARTx_ESP_DMA_BUFFER_LENGTH];

void UART4_ESP_Config(void)
{
	GPIO_InitTypeDef 	GPIO_InitStructure; 
	USART_InitTypeDef USART_InitStructure;  
	DMA_InitTypeDef   DMA_InitStruct;
	NVIC_InitTypeDef 	NVIC_InitStruct;
	
	/* Enable GPIO clock */
	USARTx_ESP_TX_CLK_INIT(USARTx_ESP_TX_GPIO_CLK, ENABLE);
	/* Enable UART clock */
	USARTx_ESP_CLK_INIT(USARTx_ESP_CLK, ENABLE);
	
	/* GPIO Configuration for UART Rx */
	USARTx_ESP_RX_CLK_INIT(USARTx_ESP_RX_GPIO_CLK, ENABLE);
	
	GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_Pin   = USARTx_ESP_RX_PIN;
  GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_NOPULL;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(USARTx_ESP_RX_GPIO_PORT, &GPIO_InitStructure);
	
	/* GPIO Configuration for UART Tx */
	GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_Pin   = USARTx_ESP_TX_PIN;
  GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_NOPULL;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(USARTx_ESP_TX_GPIO_PORT, &GPIO_InitStructure);
	
  GPIO_PinAFConfig(USARTx_ESP_TX_GPIO_PORT, USARTx_ESP_TX_SOURCE, USARTx_ESP_AF);
  GPIO_PinAFConfig(USARTx_ESP_RX_GPIO_PORT, USARTx_ESP_RX_SOURCE, USARTx_ESP_AF); 

  /* USARTx configured as follow:
		- BaudRate = 115200 baud  
    - Word Length = 8 Bits
    - One Stop Bit
    - No parity
    - Hardware flow control disabled (RTS and CTS signals)
    - Receive and transmit enabled
  */
  USART_InitStructure.USART_BaudRate = USARTx_ESP_BAUDRATE;
  USART_InitStructure.USART_WordLength = USART_WordLength_8b;
  USART_InitStructure.USART_StopBits = USART_StopBits_1;
  USART_InitStructure.USART_Parity = USART_Parity_No;
  USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
  USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
  USART_Init(USARTx_ESP, &USART_InitStructure);
  
	USART_DMACmd(USARTx_ESP,USART_DMAReq_Rx, DISABLE);
	
	/*Enable DMA clock*/
	USARTx_ESP_DMA_CLK_Cmd(USARTx_ESP_DMA_CLK, ENABLE);
	DMA_InitStruct.DMA_Channel = USARTx_ESP_DMA_Channel;
	DMA_InitStruct.DMA_Memory0BaseAddr = (uint32_t)(&(UART_ESP_DMABuffer));
	DMA_InitStruct.DMA_PeripheralBaseAddr = (uint32_t)(&(USARTx_ESP->DR));
	DMA_InitStruct.DMA_DIR = DMA_DIR_PeripheralToMemory;
	DMA_InitStruct.DMA_BufferSize = USARTx_ESP_DMA_BUFFER_LENGTH;																	
	DMA_InitStruct.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
	DMA_InitStruct.DMA_MemoryInc = DMA_MemoryInc_Enable;
	DMA_InitStruct.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
	DMA_InitStruct.DMA_MemoryDataSize = DMA_PeripheralDataSize_Byte;
	DMA_InitStruct.DMA_Mode = DMA_Mode_Circular;
	DMA_InitStruct.DMA_Priority = DMA_Priority_High;
	DMA_InitStruct.DMA_FIFOMode = DMA_FIFOMode_Disable;
	DMA_InitStruct.DMA_FIFOThreshold = DMA_FIFOThreshold_HalfFull;
	DMA_InitStruct.DMA_MemoryBurst = DMA_MemoryBurst_Single;
	DMA_InitStruct.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;

	DMA_Init(USARTx_ESP_DMA_Stream, &DMA_InitStruct);

	DMA_Cmd(USARTx_ESP_DMA_Stream, DISABLE);	

	/* DMA Transfer complete interrupt configure */
	DMA_ITConfig(USARTx_ESP_DMA_Stream, DMA_IT_TC, ENABLE);

	NVIC_InitStruct.NVIC_IRQChannel = USARTx_ESP_DMA_Stream_IRQn;
	NVIC_InitStruct.NVIC_IRQChannelPreemptionPriority = USARTx_ESP_DMA_IRQPreemptionPriority;
	NVIC_InitStruct.NVIC_IRQChannelSubPriority = USARTx_ESP_DMA_IRQSubPriority;
	NVIC_InitStruct.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStruct);
	
//	NVIC_InitStruct.NVIC_IRQChannel = USARTx_ESP_IRQn;
//	NVIC_InitStruct.NVIC_IRQChannelCmd = ENABLE;
//	NVIC_InitStruct.NVIC_IRQChannelPreemptionPriority = USARTx_ESP_IRQPreemptionPriority;
//	NVIC_InitStruct.NVIC_IRQChannelSubPriority = USARTx_ESP_IRQSubPriority;
//	NVIC_Init(&NVIC_InitStruct);

//	USART_ITConfig(USARTx_ESP,USART_IT_TXE,ENABLE);
	/* Enable USART */
  USART_Cmd(USARTx_ESP, ENABLE);
}

static uint8_t Checksum(uint8_t *arr, uint8_t length)
{
    uint8_t sum = 0;
    for(uint8_t i=0; i<length-1; i++)
    {
        sum += arr[i];
    }
    sum = ~sum;
    sum +=2;
    return sum;
}

void Update_Fuzzy_Data(void)
{
	SETPOINT = UART_ESP_DMABuffer[0];
	ERROR_MAX = UART_ESP_DMABuffer[1];
	ERRORDOT_MAX = UART_ESP_DMABuffer[2];
	OFFSET_MAX = UART_ESP_DMABuffer[3];
	
	//update gain
	K1 = 1.0/(float)ERROR_MAX;
	K2 = 1.0/(float)ERRORDOT_MAX;
	Ku = (float)OFFSET_MAX;
	
	first_data_pass = true;
}

void Pack(void)
{
	package[0] = (uint8_t)MASS_Position.Value;
	package[1] = (uint8_t)PISTOL_Position.Value;
	UCAN_Convert_Float_to_Bytes(XSEN_Pitch.Value, &package[2]);
	package[6] = ERROR_MAX;
	package[7] = ERRORDOT_MAX;
	package[8] = OFFSET_MAX;
	package[9] = SETPOINT;
	package[10] = Operation_Mode;
	
	package[11] = Checksum(package, PACKAGE_SIZE);
}

void Send_Data_ESP(void)
{
	if(!enable_sending_package)
	{
		switch(data_init)
		{
			case 0:
				USART_SendData(USARTx_ESP, 'A');
				data_init ++;
			break;
			case 1:
				USART_SendData(USARTx_ESP, 'R');
				data_init ++;
			break;
			case 2:
				USART_SendData(USARTx_ESP, 'M');
				Pack();
				data_init = 0;
				enable_sending_package = true;
			break;
		}
	}
	else
	{
		USART_SendData(USARTx_ESP, package[byte_data]);
		byte_data ++;
		if(byte_data == PACKAGE_SIZE)
		{
			Pack();
			byte_data = 0;
			enable_sending_package = false;
		}
	}
}

//void USARTx_ESP_IRQHandler(void)
//{
//	if(USART_GetITStatus(USARTx_ESP,USART_IT_TXE) != RESET)
//	{
//		USART_ClearITPendingBit(USARTx_ESP,USART_IT_TXE);
//	}
//}

void USARTx_ESP_DMA_Stream_IRQHandler(void)
{
	if(DMA_GetITStatus(USARTx_ESP_DMA_Stream, DMA_IT_TCIF2) != RESET)
	{
		if(Checksum(UART_ESP_DMABuffer, USARTx_ESP_DMA_BUFFER_LENGTH) == UART_ESP_DMABuffer[USARTx_ESP_DMA_BUFFER_LENGTH-1])
			Flag.Fuzzy_Update = true;
		DMA_ClearITPendingBit(USARTx_ESP_DMA_Stream,DMA_IT_DMEIF2|DMA_IT_FEIF2|DMA_IT_HTIF2|DMA_IT_TCIF2|DMA_IT_TEIF2);
	}
}
