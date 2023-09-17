/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include"stdio.h"
#include"string.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
CAN_HandleTypeDef hcan;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_CAN_Init(void);
/* USER CODE BEGIN PFP */
void CAN_TX(void);
void CAN_RX(void);
void CAN_TXU(unsigned int CAN_ID, float a);
void Uart( unsigned int a,int b , int c, int d);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
CAN_TxHeaderTypeDef Txheader;
CAN_RxHeaderTypeDef Rxheader;

uint8_t Txdata[8];
uint8_t Rxdata[8];
int datacheck =0;
int datacheck1=0;
int datacheck2=0;
uint32_t txmailbox;
char msg[13];
char msg1[13];
char msg2[13];
char hexChar[3];

//void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
//{
//	if(GPIO_Pin== GPIO_PIN_13){
//		Txdata[0] = 100;  //ms delay
//		Txdata[1] = 10;
//
//		HAL_CAN_AddTxMessage(&hcan, &Txheader, Txdata, &txmailbox);
//	}
//}

//void HAL_CAN_RxFifo1MsgPendingCallback(CAN_HandleTypeDef *hcan)
//{
//if(HAL_CAN_GetRxMessage(&hcan, CAN_RX_FIFO0, &RxHeader , rcvd_msg) != HAL_OK){
//
//}

int _write(int file, char *ptr, int len)
{

 int DataIdx;

 for (DataIdx = 0; DataIdx < len; DataIdx++)
 {
   ITM_SendChar(*ptr++);
 }
 return len;
}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
//	typedef struct{
//
//		uint8_t intValue;
//	//	float FloatValue;
//	//	char StringVAlue[20];
//
//
//
//	}MyData;

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_USART2_UART_Init();
  MX_CAN_Init();
  /* USER CODE BEGIN 2 */
  HAL_CAN_Start(&hcan);
  //NOTIFY
//  HAL_CAN_ActivateNotification(&hcan, CAN_IT_RX_FIFO1_MSG_PENDING);

//  Txheader.DLC =2;//DATA LENGTH
//  Txheader.IDE= CAN_ID_STD;
//  Txheader.RTR=CAN_RTR_DATA;
//  Txheader.StdId=0x103; //id
//
//Txdata[0] =200;
//Txdata[1] = 20;





  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
//	  while(! HAL_CAN_GetRxFifoFillLevel(&hcan, CAN_RX_FIFO0));
      CAN_RX();
	//  CAN_TX();

      // Current Sensor
	  CAN_TXU(0x61, 0.00003);   //current sensor 1 value on Can
	  CAN_TXU(0x62, 0.01);      // current sensor 2 on Can
	  CAN_TXU(0x63, 0.02);      // current sensor 3 on Can
	  CAN_TXU(0x64, 0.03);      // current sensor 4 on Can
	  CAN_TXU(0x65, 0.04);      // current sensor 5 on Can
	  CAN_TXU(0x66, 0.05);      // current sensor 6 on Can
	  CAN_TXU(0x67, 0.06);      // current sensor 7 on Can
	  CAN_TXU(0x68, 0.07);      // current sensor 8 on Can
	  CAN_TXU(0x69, 0.08);      // current sensor 9 on Can
	  CAN_TXU(0x70, 0.09);      // current sensor 10 on Can
	  CAN_TXU(0x71, 0.010);      // current sensor 11 on Can
	  CAN_TXU(0x72, 0.011);      // current sensor 12 on Can
	  CAN_TXU(0x73, 0.012);      // current sensor 13 on Can

	  // Temperature Sensor
	  CAN_TXU(0x91, 20.01);      // Temperature sensor 1 on Can
	  CAN_TXU(0x92, 30.015);      // Temperature sensor 2 on Can
	  CAN_TXU(0x93, 40.016);      // Temperature sensor 3 on Can
	  CAN_TXU(0x94, 50.017);      // Temperature sensor 4 on Can
	  CAN_TXU(0x95, 60.018);      // Temperature sensor 5 on Can
	  CAN_TXU(0x96, 70.019);      // Temperature sensor 6 on Can

	  //Don't Forget to Implement Receiver for temperature




    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
//	  if(datacheck)
//	 	  {
//
//	 		  for(int i=0;i<1;i++){
//	 			  HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_5);
////	 			  HAL_Delay(Rxdata[0]);
////	 			  printf("X %d",Rxdata[2]);
//
//
//
//	 		  HAL_UART_Transmit(&huart2, "Data of NUCLEOF446RE  Unit of DATA is in m/s2 \r\n ",60 , 100);
//	 		                sprintf(msg ,"a_x %d \r\n", Rxdata[2]);
//	 			 			  HAL_UART_Transmit(&huart2, (uint8_t*)msg, sizeof(msg), 100);
//	 			 			 sprintf(msg1 ,"a_y %d \r\n", Rxdata[3]);
//	 			 		  HAL_UART_Transmit(&huart2, (uint8_t*)msg1, sizeof(msg1), 100);
//	 			 				sprintf(msg2 ,"a_z %d \r\n", Rxdata[4]);
//	 			 			 HAL_UART_Transmit(&huart2, (uint8_t*)msg2, sizeof(msg2), 100);
//	 			 			  datacheck=0;
//
////	 			 				 		  HAL_CAN_AddTxMessage(&hcan, &Txheader, &Txdata, &txmailbox);
//	 		  }
//	 	  }


//	 		  if (datacheck1) {
//		 		  HAL_UART_Transmit(&huart2, "Data of NUCLEOF446RE  Unit of DATA is in Gyro \r\n ",60 , 100);
//		 		                sprintf(msg ,"g_x %d \r\n", Rxdata[2]);
//		 			 			  HAL_UART_Transmit(&huart2, (uint8_t*)msg, sizeof(msg), 100);
//		 			 			 sprintf(msg1 ,"g_y %d \r\n", Rxdata[3]);
//		 			 		  HAL_UART_Transmit(&huart2, (uint8_t*)msg1, sizeof(msg1), 100);
//		 			 				sprintf(msg2 ,"g_z %d \r\n", Rxdata[4]);
//		 			 			 HAL_UART_Transmit(&huart2, (uint8_t*)msg2, sizeof(msg2), 100);
//
//		 			 				 		  datacheck1 =0;
////		 			 				 		  HAL_CAN_AddTxMessage(&hcan, &Txheader, &Txdata, &txmailbox);
//			}
//	 		  if (datacheck2) {
//	 			  printf("%d",Rxdata[0]);
//	 			  datacheck2 =0;
//
//			}
//	 		  sprintf(hexChar,"%2X", Rxdata[2]);
//	 		  char charValue = Rxdata[2];
//	 		  uint8_t uartData = {charValue};
//	 		  HAL_UART_Transmit(&huart2, &uartData, 3, 100);
//	 		  HAL_Delay(100);
//		  HAL_UART_Transmit(&huart2, Rxdata, sizeof(Rxdata), 100);

//		  MyData originalRx;
//		  char str[2];
//		  memccpy(originalRx, Rxdata , 8);
//		  sprintf(str,"%d", originalRx.intValue);
//		  HAL_UART_Transmit(&huart2, (uint8_t*)str, sizeof(str), 100);
//		  sprintf(str,"%0.1f", originalRx.FloatValue);
//				  HAL_UART_Transmit(&huart2, (uint8_t*)str, sizeof(str), 100);
//				  sprintf(str,"%d", originalRx.StringVAlue);
//						  HAL_UART_Transmit(&huart2, (uint8_t*)str, sizeof(str), 100);





  }

  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_BYPASS;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief CAN Initialization Function
  * @param None
  * @retval None
  */
static void MX_CAN_Init(void)
{

  /* USER CODE BEGIN CAN_Init 0 */

  /* USER CODE END CAN_Init 0 */

  /* USER CODE BEGIN CAN_Init 1 */

  /* USER CODE END CAN_Init 1 */
  hcan.Instance = CAN1;
  hcan.Init.Prescaler = 18;
  hcan.Init.Mode = CAN_MODE_NORMAL;
  hcan.Init.SyncJumpWidth = CAN_SJW_1TQ;
  hcan.Init.TimeSeg1 = CAN_BS1_2TQ;
  hcan.Init.TimeSeg2 = CAN_BS2_1TQ;
  hcan.Init.TimeTriggeredMode = DISABLE;
  hcan.Init.AutoBusOff = DISABLE;
  hcan.Init.AutoWakeUp = DISABLE;
  hcan.Init.AutoRetransmission = DISABLE;
  hcan.Init.ReceiveFifoLocked = DISABLE;
  hcan.Init.TransmitFifoPriority = DISABLE;
  if (HAL_CAN_Init(&hcan) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CAN_Init 2 */
  CAN_FilterTypeDef can1_filter_init;
  	can1_filter_init.FilterActivation = CAN_FILTER_ENABLE;
  	can1_filter_init.FilterBank = 10;
  	can1_filter_init.FilterFIFOAssignment = CAN_FILTER_FIFO1;
  	can1_filter_init.FilterIdHigh = 0x446<<5;
  	can1_filter_init.FilterIdLow = 0x0000;
  	can1_filter_init.FilterMaskIdHigh = 0x1<<14;
  	can1_filter_init.FilterMaskIdLow = 0x0000;
  	can1_filter_init.FilterMode  = CAN_FILTERMODE_IDMASK;
  	can1_filter_init.FilterScale = CAN_FILTERSCALE_32BIT;
  	can1_filter_init.SlaveStartFilterBank= 0;

  	HAL_CAN_ConfigFilter(&hcan, &can1_filter_init);


  /* USER CODE END CAN_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_RESET);

  /*Configure GPIO pin : PC13 */
  GPIO_InitStruct.Pin = GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : PA5 */
  GPIO_InitStruct.Pin = GPIO_PIN_5;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

void CAN_TX(void){

	 Txheader.DLC =2;//DATA LENGTH
	  Txheader.IDE= CAN_ID_STD;
	  Txheader.RTR=CAN_RTR_DATA;
	  Txheader.StdId=0x103; //id

	Txdata[0] =200;
	Txdata[1] = 20;
	 HAL_CAN_AddTxMessage(&hcan, &Txheader, Txdata, &txmailbox);
}
void CAN_TXU(unsigned int CAN_ID, float a){               // unsigned int CAN_ID = 0x1A;
	 uint32_t intVal;
	 Txheader.DLC =4;//DATA LENGTH
	  Txheader.IDE= CAN_ID_STD;
	  Txheader.RTR=CAN_RTR_DATA;
	  Txheader.StdId=CAN_ID; //id
	    intVal = *((uint32_t*)&a);

	    // Copy the bytes of the integer into TxData
	    for (int i = 0; i < 4; i++) {
	        Txdata[i] = (uint8_t)(intVal >> (i * 8));
	    }

	 HAL_CAN_AddTxMessage(&hcan, &Txheader, Txdata, &txmailbox);
	 HAL_Delay(10);
}
void CAN_RX(void){

	  	if( HAL_CAN_GetRxMessage(&hcan, CAN_RX_FIFO1, &Rxheader, Rxdata) == HAL_OK)
	  	{
		if(Rxheader.StdId==0x100)
		{
			Uart(Rxheader.StdId,Rxdata[0],Rxdata[1],Rxdata[3]);

		}
		else if (Rxheader.StdId == 0x102) {

			Uart(Rxheader.StdId,Rxdata[0],Rxdata[1],Rxdata[3]);
		}
		else if (Rxheader.StdId == 0x104) {

					Uart(Rxheader.StdId,Rxdata[0],Rxdata[1],Rxdata[3]);
				}
		else if (Rxheader.StdId == 0x106) {

							Uart(Rxheader.StdId,Rxdata[0],Rxdata[1],Rxdata[3]);
						}
		else if (Rxheader.StdId == 0x108) {

							Uart(Rxheader.StdId,Rxdata[0],Rxdata[1],Rxdata[3]);
						}
		else if (Rxheader.StdId == 0x110) {

							Uart(Rxheader.StdId,Rxdata[0],Rxdata[1],Rxdata[3]);
						}
		else if (Rxheader.StdId == 0x112) {

							Uart(Rxheader.StdId,Rxdata[0],Rxdata[1],Rxdata[3]);
						}

	  	};



};
void Uart( unsigned int a,int b , int c, int d){
	char msg[30];
	char msg1[30];
	sprintf(msg," Sensor Id %x  %d %d %d ",a,b,c,d);
	strcat(msg, "\r\n");
	HAL_UART_Transmit(&huart2, (uint8_t*)msg, sizeof(msg), 100);
}


/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
