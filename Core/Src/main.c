/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdint.h>
#include "Servo_Interpreter.h"
#include "raspi_Interpreter.h"
#include "JY901_Interpreter.h"
#include "CH100_Interpreter.h"
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
SPI_HandleTypeDef hspi1;
DMA_HandleTypeDef hdma_spi1_rx;
DMA_HandleTypeDef hdma_spi1_tx;

UART_HandleTypeDef huart4;
UART_HandleTypeDef huart5;
UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;
UART_HandleTypeDef huart3;
DMA_HandleTypeDef hdma_uart4_rx;
DMA_HandleTypeDef hdma_uart4_tx;
DMA_HandleTypeDef hdma_uart5_rx;
DMA_HandleTypeDef hdma_uart5_tx;
DMA_HandleTypeDef hdma_usart1_rx;
DMA_HandleTypeDef hdma_usart1_tx;
DMA_HandleTypeDef hdma_usart2_rx;
DMA_HandleTypeDef hdma_usart2_tx;
DMA_HandleTypeDef hdma_usart3_rx;
DMA_HandleTypeDef hdma_usart3_tx;

/* USER CODE BEGIN PV */
HAL_StatusTypeDef readRecFlag[4];
HAL_StatusTypeDef readSendFlag[4];
HAL_StatusTypeDef setSendFlag[12];
HAL_StatusTypeDef spiRTFlag;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_SPI1_Init(void);
static void MX_UART4_Init(void);
static void MX_UART5_Init(void);
/* USER CODE BEGIN PFP */
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
int16_t servoAngleRead[12]; // servo angle read value, note that, according to the com manual, this value should be the real value times 10.
int16_t servoAngleSet[12]; // servo angle set value, note that, according to the com manual, this value should be the real value times 10.
uint8_t lineIdx;
uint8_t lineFlag[4];
uint8_t servoIDarray[12];
uint8_t spiRecData[60], spiSendData[60];
uint8_t spiRecData_CH100[79],spiSendData_CH100[79]; // 79 is used
volatile uint8_t getRaspiCMD=0;
uint8_t raspiReadFlag, raspiSetFlag;
volatile int ledCount=0;

uint8_t servoAngleAskCmdL1[6],servoAngleAskCmdL2[6],servoAngleAskCmdL3[6],servoAngleAskCmdL4[6];
//uint8_t servoAngleAskCmdL1[12],servoAngleAskCmdL2[12],servoAngleAskCmdL3[12],servoAngleAskCmdL4[12];
uint8_t servoAngleRecCmdL1[8],servoAngleRecCmdL2[8],servoAngleRecCmdL3[8],servoAngleRecCmdL4[8];
uint8_t servoAngleSetCmdL1[12],servoAngleSetCmdL2[12],servoAngleSetCmdL3[12],servoAngleSetCmdL4[12];
uint8_t servoNum;
uint8_t imuRecData[33];
uint8_t imuRecData_CH100[164];
uint8_t imuAskData[5];
uint8_t accData[6], omegaData[6],rpyData[6];
uint8_t accData_CH100[12],omegaData_CH100[12],quatData_CH100[16];
uint8_t IMU_data[18], IMU_CH100_data[40];

volatile uint8_t TIMCountFlag=0;
volatile uint8_t uartTxFlag[5]; // send complete flag for 4 uart lines for servos, and 1 uart line for imu
volatile uint8_t uartRxFlag[5];
volatile int raspiRecErr=0;
volatile int setFinish=0;
volatile int readFinish=0;

volatile int LEDcountV2=0;

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
	if (huart->Instance==USART1)
		uartTxFlag[0]=1;
	else if (huart->Instance==USART2)
		uartTxFlag[1]=1;
	else if (huart->Instance==USART3)
		uartTxFlag[2]=1;
	else if (huart->Instance==UART5)
		uartTxFlag[3]=1;
	else if (huart->Instance==UART4)
		uartTxFlag[4]=1;

}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	if (huart->Instance==USART1)
		uartRxFlag[0]=1;
	if (huart->Instance==USART2)
		uartRxFlag[1]=1;
	if (huart->Instance==USART3)
		uartRxFlag[2]=1;
	if (huart->Instance==UART5)
		uartRxFlag[3]=1;
	if (huart->Instance==UART4)
	{
		HAL_UART_Receive_DMA(&huart4, imuRecData,33);uartRxFlag[4]=1;JY901_FBdecoder(imuRecData, accData, omegaData, rpyData, IMU_data);} // for JY901
	/*{//HAL_UART_Receive_DMA(&huart4, imuRecData_CH100,164);
	uartRxFlag[4]=1;
	CH100_FBdecoder(imuRecData_CH100, accData_CH100, omegaData_CH100, quatData_CH100, IMU_CH100_data);} // for CH100*/
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    if(huart->ErrorCode != HAL_UART_ERROR_NONE)
    {
        //ERR_TRACE0("baseaddr = 0x%x, error code = 0x%x", huart->Instance, huart->ErrorCode);

        __HAL_UART_CLEAR_NEFLAG(huart);

        //__HAL_UART_CLEAR_OREFLAG(huart);

        huart->RxState = HAL_UART_STATE_READY;

	if (huart->Instance==USART1)
		uartRxFlag[0]=1;
	if (huart->Instance==USART2)
		uartRxFlag[1]=1;
	if (huart->Instance==USART3)
		uartRxFlag[2]=1;
	if (huart->Instance==UART5)
		uartRxFlag[3]=1;
	if (huart->Instance==UART4)
		uartRxFlag[4]=1;
    }
}

void HAL_SPI_ErrorCallback(SPI_HandleTypeDef *hspi)
{
	__HAL_SPI_CLEAR_OVRFLAG(hspi);
}


void HAL_SPI_TxRxCpltCallback(SPI_HandleTypeDef *hspi)
{
	getRaspiCMD=1;
	LEDcountV2++;
	if (LEDcountV2>10)
		{HAL_GPIO_TogglePin(LED_Onboard_GPIO_Port, LED_Onboard_Pin);LEDcountV2=0;}
}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
  int ii=0;
  uint8_t imu_unlock[5];
  for (ii=0;ii<60;ii++)
  	spiSendData[ii]=0;
  uartTxFlag[0]=1;uartTxFlag[1]=1;uartTxFlag[2]=1;uartTxFlag[3]=1;uartTxFlag[4]=1;
  uartRxFlag[0]=1;uartRxFlag[1]=1;uartRxFlag[2]=1;uartRxFlag[3]=1;uartRxFlag[4]=1;
  imu_unlock[0]=0xff;imu_unlock[1]=0xaa;imu_unlock[2]=0x69;imu_unlock[3]=0x88;imu_unlock[4]=0xb5;
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
  MX_DMA_Init();
  MX_USART1_UART_Init();
  MX_USART2_UART_Init();
  MX_USART3_UART_Init();
  MX_SPI1_Init();
  MX_UART4_Init();
  MX_UART5_Init();
  /* USER CODE BEGIN 2 */
  HAL_GPIO_WritePin(LED_Onboard_GPIO_Port, LED_Onboard_Pin,GPIO_PIN_SET);
  //HAL_GPIO_WritePin(WorkState_GPIO_Port, WorkState_Pin,GPIO_PIN_RESET);
  HAL_SPI_TransmitReceive_DMA(&hspi1, spiSendData, spiRecData, 60);

  HAL_UART_Receive_DMA(&huart1, servoAngleRecCmdL1, 8);
  HAL_UART_Receive_DMA(&huart2, servoAngleRecCmdL2, 8);
  HAL_UART_Receive_DMA(&huart3, servoAngleRecCmdL3, 8);
  HAL_UART_Receive_DMA(&huart5, servoAngleRecCmdL4, 8);

  // IMU initialization for JY901
  uartRxFlag[4]=0;
  HAL_UART_Transmit(&huart4, imu_unlock,5,1000);
  HAL_Delay(20);
  HAL_UART_Receive_DMA(&huart4, imuRecData,33);
  JY901_genAskCmd(imuAskData,1); // for single time output, to clear the uart
  HAL_UART_Transmit(&huart4, imuAskData,5,1000);
  HAL_Delay(20);
  while(uartRxFlag[4]==0);

  uartRxFlag[4]=0;
  HAL_UART_Receive_DMA(&huart4, imuRecData,33);
  JY901_genAskCmd(imuAskData,200); // for 200 Hz continuous output
  HAL_UART_Transmit(&huart4, imuAskData,5,1000);
  HAL_Delay(20);

  //IMU initialization for CH100
 /* uartRxFlag[4]=0;
  uint8_t startTrans[11]="AT+EOUT=1\r\n";
  uint8_t endTrans[11]="AT+EOUT=0\r\n";
  HAL_UART_Transmit(&huart4, endTrans,11,1000);
  HAL_Delay(20);
  HAL_UART_Receive_DMA(&huart4, imuRecData_CH100,164);
  HAL_UART_Transmit(&huart4, startTrans,11,1000);
  HAL_Delay(20);
  HAL_UART_Receive_DMA(&huart4, imuRecData_CH100,164);*/
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  /*spiRTFlag=HAL_SPI_TransmitReceive(&hspi1, spiSendData, spiRecData, 60,1000);
	  if (spiRTFlag==HAL_OK)
		  getRaspiCMD=1;
	  else
		  {getRaspiCMD=0;HAL_Delay(1);}*/


	  if (getRaspiCMD==1)
	  {
		  /*uartRxFlag[4]=0;
		  HAL_UART_Receive_DMA(&huart4, imuRecData,33);
		  JY901_genAskCmd(imuAskData,1);
		  HAL_UART_Transmit(&huart4, imuAskData,5,1000);*/
		  //HAL_GPIO_WritePin(WorkState_GPIO_Port, WorkState_Pin,GPIO_PIN_RESET);
		  setFinish=0;
		  readFinish=0;
		  volatile int i;
		  double angleres;
		  //raspiRecErr=raspiCMDdecoder_CH100(spiRecData_CH100, &raspiReadFlag, &raspiSetFlag, servoIDarray, &servoNum,servoAngleSet,&lineIdx);
		  raspiRecErr=raspiCMDdecoder(spiRecData, &raspiReadFlag, &raspiSetFlag, servoIDarray, &servoNum,servoAngleSet,&lineIdx);
		  lineFlag[0]=0;lineFlag[1]=0;lineFlag[2]=0;lineFlag[3]=0;
		  if (lineIdx==1)
				  lineFlag[0]=1;
		  else if (lineIdx==2)
				  lineFlag[1]=1;
		  else if (lineIdx==3)
				  lineFlag[2]=1;
		  else if (lineIdx==4)
				  lineFlag[3]=1;
		  else if (lineIdx==5)
				  {lineFlag[0]=1;lineFlag[1]=1;lineFlag[2]=1;lineFlag[3]=1;}
		  //{lineFlag[1]=1;lineFlag[2]=1;}
		  if (raspiSetFlag==1)
		  {

		  	  for (i=0;i<servoNum;i++)
		  	  {
		  		  //genSetAngleCmd(servoIDarray[i], servoAngleSet[i], 0, 0, servoAngleSetCmd);
		    	  //uartTxFlag[0]=0;
		  		  //setSendFlag[i]=HAL_UART_Transmit_IT(&huart1, servoAngleSetCmd, 12);
		  		  //while(uartTxFlag[0]==0);
 				  if (lineFlag[0]==1)
 				  {
 					  //while(uartTxFlag[0]==0){};
 					  genSetAngleCmd(servoIDarray[i], servoAngleSet[i], 0, 0, servoAngleSetCmdL1);uartTxFlag[0]=0;
 					  HAL_UART_Transmit_DMA(&huart1, servoAngleSetCmdL1,12);
 					  //while(uartTxFlag[0]==0);
 				  }
 				  else
 					  uartTxFlag[0]=1;

 				  if (lineFlag[1]==1)
 				  {
 					 //while(uartTxFlag[1]==0){};
 					  genSetAngleCmd(servoIDarray[i+3], servoAngleSet[i+3], 0, 0, servoAngleSetCmdL2);uartTxFlag[1]=0;
 				      HAL_UART_Transmit_DMA(&huart2, servoAngleSetCmdL2,12);
 				      //while(uartTxFlag[1]==0);
 				  }
 				  else
 					  uartTxFlag[1]=1;

 				  if (lineFlag[2]==1)
 				  {
 					 //while(uartTxFlag[2]==0){};
 				      genSetAngleCmd(servoIDarray[i+6], servoAngleSet[i+6], 0, 0, servoAngleSetCmdL3);uartTxFlag[2]=0;
 				  	  HAL_UART_Transmit_DMA(&huart3, servoAngleSetCmdL3,12);
 				  	  //while(uartTxFlag[2]==0);
 				  }
 				  else
 					  uartTxFlag[2]=1;

 				  if (lineFlag[3]==1)
				  {
 					// while(uartTxFlag[3]==0){};
 				  	  genSetAngleCmd(servoIDarray[i+9], servoAngleSet[i+9], 0, 0, servoAngleSetCmdL4); uartTxFlag[3]=0;
 				  	  HAL_UART_Transmit_DMA(&huart5, servoAngleSetCmdL4,12);
 				  	  //while(uartTxFlag[3]==0);
 				  }
 				  else
 					  uartTxFlag[3]=1;


 				  while(uartTxFlag[0]+uartTxFlag[1]+uartTxFlag[2]+uartTxFlag[3]<4);

 	 			  }
 		  }
		  setFinish=1;
		  if (raspiReadFlag==1)
		  {

			  for (i=0;i<servoNum;i++)
			  {
				  if (lineFlag[0]==1)
				    {uartRxFlag[0]=0;
				    //HAL_UART_Receive_DMA(&huart1, servoAngleRecCmdL1, 8);
				    genAngleAskCmd(servoIDarray[i],servoAngleAskCmdL1);
				    //while(uartTxFlag[0]==0);
				    }
				  else
				  	uartRxFlag[0]=1;
				  if (lineFlag[1]==1)
				    {uartRxFlag[1]=0;
				    //HAL_UART_Receive_DMA(&huart2, servoAngleRecCmdL2, 8);
				    genAngleAskCmd(servoIDarray[i+3],servoAngleAskCmdL2);
				    //while(uartTxFlag[1]==0);
				    }
				  else
				    uartRxFlag[1]=1;
				  if (lineFlag[2]==1)
				    {uartRxFlag[2]=0;
				    //HAL_UART_Receive_DMA(&huart3, servoAngleRecCmdL3, 8);
				    genAngleAskCmd(servoIDarray[i+6],servoAngleAskCmdL3);

				    //while(uartTxFlag[2]==0);
				    }
				  else
				    uartRxFlag[2]=1;
				  if (lineFlag[3]==1)
				    {uartRxFlag[3]=0;
				    //HAL_UART_Receive_DMA(&huart5, servoAngleRecCmdL4, 8);
				    genAngleAskCmd(servoIDarray[i+9],servoAngleAskCmdL4);

				    //while(uartTxFlag[3]==0);
				    }
				  else
				    uartRxFlag[3]=1;

				  if(lineFlag[0]==1) {HAL_UART_Transmit_DMA(&huart1, servoAngleAskCmdL1, 6);}
				  if(lineFlag[1]==1) {HAL_UART_Transmit_DMA(&huart2, servoAngleAskCmdL2, 6);}
				  if(lineFlag[2]==1) {HAL_UART_Transmit_DMA(&huart3, servoAngleAskCmdL3, 6);}
				  if(lineFlag[3]==1) {HAL_UART_Transmit_DMA(&huart5, servoAngleAskCmdL4, 6);}

				  //while(uartRxFlag[0]==0);
				  //while(uartRxFlag[1]==0);
				  //while(uartRxFlag[2]==0);
				  //while(uartRxFlag[3]==0);
				  while(uartRxFlag[0]+uartRxFlag[1]+uartRxFlag[2]+uartRxFlag[3]<4);

				  if (lineFlag[0]==1)
				  {
					  //while(uartRxFlag[0]==0){};
					  angleres=recAngleCmdDecode(servoIDarray[i], servoAngleRecCmdL1, servoAngleRead[i]/10.0);
					  servoAngleRead[i]=angleres*10.0;
				  }
				  if (lineFlag[1]==1)
				  {
					  //while(uartRxFlag[1]==0){};
					  angleres=recAngleCmdDecode(servoIDarray[i+3], servoAngleRecCmdL2, servoAngleRead[i+3]/10.0);
					  servoAngleRead[i+3]=angleres*10.0;
				  }
				  if (lineFlag[2]==1)
				  {
					  //while(uartRxFlag[2]==0){};
					  angleres=recAngleCmdDecode(servoIDarray[i+6], servoAngleRecCmdL3, servoAngleRead[i+6]/10.0);
					  servoAngleRead[i+6]=angleres*10.0;
				  }
				  if (lineFlag[3]==1)
				  {
					  //while(uartRxFlag[3]==0){};
					  angleres=recAngleCmdDecode(servoIDarray[i+9], servoAngleRecCmdL4, servoAngleRead[i+9]/10.0);
					  servoAngleRead[i+9]=angleres*10.0;
				  }
			  }
		  }
		  readFinish=1;

		  //genAngleBackCmd_CH100(servoIDarray, servoNum,servoAngleRead, spiSendData_CH100, IMU_CH100_data);
		  genAngleBackCmd(servoIDarray, servoNum,servoAngleRead, spiSendData, IMU_data);
		  getRaspiCMD=0;
		  //HAL_SPI_TransmitReceive_DMA(&hspi1, spiSendData_CH100, spiRecData_CH100, 79);


	  }
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
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

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 168;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_SLAVE;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_HARD_INPUT;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

}

/**
  * @brief UART4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_UART4_Init(void)
{

  /* USER CODE BEGIN UART4_Init 0 */

  /* USER CODE END UART4_Init 0 */

  /* USER CODE BEGIN UART4_Init 1 */

  /* USER CODE END UART4_Init 1 */
  huart4.Instance = UART4;
  huart4.Init.BaudRate = 921600;
  huart4.Init.WordLength = UART_WORDLENGTH_8B;
  huart4.Init.StopBits = UART_STOPBITS_1;
  huart4.Init.Parity = UART_PARITY_NONE;
  huart4.Init.Mode = UART_MODE_TX_RX;
  huart4.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart4.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN UART4_Init 2 */

  /* USER CODE END UART4_Init 2 */

}

/**
  * @brief UART5 Initialization Function
  * @param None
  * @retval None
  */
static void MX_UART5_Init(void)
{

  /* USER CODE BEGIN UART5_Init 0 */

  /* USER CODE END UART5_Init 0 */

  /* USER CODE BEGIN UART5_Init 1 */

  /* USER CODE END UART5_Init 1 */
  huart5.Instance = UART5;
  huart5.Init.BaudRate = 500000;
  huart5.Init.WordLength = UART_WORDLENGTH_8B;
  huart5.Init.StopBits = UART_STOPBITS_1;
  huart5.Init.Parity = UART_PARITY_NONE;
  huart5.Init.Mode = UART_MODE_TX_RX;
  huart5.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart5.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart5) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN UART5_Init 2 */

  /* USER CODE END UART5_Init 2 */

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 500000;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

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
  huart2.Init.BaudRate = 500000;
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
  * @brief USART3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART3_UART_Init(void)
{

  /* USER CODE BEGIN USART3_Init 0 */

  /* USER CODE END USART3_Init 0 */

  /* USER CODE BEGIN USART3_Init 1 */

  /* USER CODE END USART3_Init 1 */
  huart3.Instance = USART3;
  huart3.Init.BaudRate = 500000;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART3_Init 2 */

  /* USER CODE END USART3_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA2_CLK_ENABLE();
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Stream0_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream0_IRQn);
  /* DMA1_Stream1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream1_IRQn);
  /* DMA1_Stream2_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream2_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream2_IRQn);
  /* DMA1_Stream3_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream3_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream3_IRQn);
  /* DMA1_Stream4_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream4_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream4_IRQn);
  /* DMA1_Stream5_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream5_IRQn);
  /* DMA1_Stream6_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream6_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream6_IRQn);
  /* DMA1_Stream7_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream7_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream7_IRQn);
  /* DMA2_Stream0_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream0_IRQn);
  /* DMA2_Stream3_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream3_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream3_IRQn);
  /* DMA2_Stream5_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream5_IRQn);
  /* DMA2_Stream7_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream7_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream7_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LED_Onboard_GPIO_Port, LED_Onboard_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : LED_Onboard_Pin */
  GPIO_InitStruct.Pin = LED_Onboard_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LED_Onboard_GPIO_Port, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

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
/*  while (1)
  {
  }*/
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

