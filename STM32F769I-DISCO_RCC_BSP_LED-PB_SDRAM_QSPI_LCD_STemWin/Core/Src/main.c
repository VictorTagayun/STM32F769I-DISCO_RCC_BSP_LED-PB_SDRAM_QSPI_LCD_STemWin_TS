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

#include "stlogo.h"
#include "WM.h"
#include "GUI.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

#define BUFFER_SIZE         ((uint32_t)0x1000)
#define WRITE_READ_ADDR     ((uint32_t)0x0800)

#define QSPI_BUFFER_SIZE         ((uint32_t)0x0200)
#define QSPI_WRITE_READ_ADDR     ((uint32_t)0x0050)

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/* Read/Write Buffers */
uint32_t aTxBuffer[BUFFER_SIZE];
uint32_t aRxBuffer[BUFFER_SIZE];

uint8_t qspi_aTxBuffer[QSPI_BUFFER_SIZE];
uint8_t qspi_aRxBuffer[QSPI_BUFFER_SIZE];

/* Status variables */
__IO uint32_t uwWriteReadStatus = 0;

/* Counter index */
uint32_t uwIndex = 0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
/* USER CODE BEGIN PFP */

static void Fill_Buffer(uint32_t *pBuffer, uint32_t uwBufferLenght, uint32_t uwOffset);
static void QPSI_Fill_Buffer(uint8_t *pBuffer, uint32_t uwBufferLenght, uint32_t uwOffset);
static uint8_t Buffercmp(uint8_t* pBuffer1, uint8_t* pBuffer2, uint32_t BufferLength);
static void Display_DemoDescription(void);

extern void MainTask(void);
extern WM_HWIN CreateWindow(void);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */


  /* USER CODE END 1 */

  /* Enable I-Cache---------------------------------------------------------*/
  SCB_EnableICache();

  /* Enable D-Cache---------------------------------------------------------*/
  SCB_EnableDCache();

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
  /* USER CODE BEGIN 2 */

  BSP_LED_Init(LED_GREEN);
  BSP_LED_Init(LED_RED);

  /* Configure the Tamper push-button in GPIO Mode */
  BSP_PB_Init(BUTTON_WAKEUP, BUTTON_MODE_GPIO);


  // SDRAM BSP Init
  if(BSP_SDRAM_Init() != SDRAM_OK)
  {
	  while(1)
	  {
		  BSP_LED_Toggle(LED_RED);
		  HAL_Delay(200);
	  }
  }

  /* Fill the buffer to write */
  Fill_Buffer(aTxBuffer, BUFFER_SIZE, 0xA244250F);

  /* Write data to the SDRAM memory */
  for (uwIndex = 0; uwIndex < BUFFER_SIZE; uwIndex++)
  {
    *(__IO uint32_t*) (SDRAM_DEVICE_ADDR + WRITE_READ_ADDR + 4*uwIndex) = aTxBuffer[uwIndex];
  }

  /* Read back data from the SDRAM memory */
  for (uwIndex = 0; uwIndex < BUFFER_SIZE; uwIndex++)
  {
    aRxBuffer[uwIndex] = *(__IO uint32_t*) (SDRAM_DEVICE_ADDR + WRITE_READ_ADDR + 4*uwIndex);
  }

  /*##-3- Checking data integrity ############################################*/
  for (uwIndex = 0; (uwIndex < BUFFER_SIZE) && (uwWriteReadStatus == 0); uwIndex++)
  {
    if (aRxBuffer[uwIndex] != aTxBuffer[uwIndex])
    {
      uwWriteReadStatus++;
	  while(1)
	  {
		  BSP_LED_Toggle(LED_RED);
		  HAL_Delay(200);
	  }
    }
  }


  // BSP QSPI Init
  if(BSP_QSPI_Init() != QSPI_OK)
  {
	  while(1)
	  {
		  BSP_LED_Toggle(LED_RED);
		  HAL_Delay(200);
	  }
  }

  /* QSPI info structure */
  static QSPI_Info pQSPI_Info;

  /*##-2- Read & check the QSPI info #######################################*/
  /* Initialize the structure */
  pQSPI_Info.FlashSize        = (uint32_t)0x00;
  pQSPI_Info.EraseSectorSize    = (uint32_t)0x00;
  pQSPI_Info.EraseSectorsNumber = (uint32_t)0x00;
  pQSPI_Info.ProgPageSize       = (uint32_t)0x00;
  pQSPI_Info.ProgPagesNumber    = (uint32_t)0x00;

  /* Read the QSPI memory info */
  if(BSP_QSPI_GetInfo(&pQSPI_Info) != QSPI_OK)
  {
	  while(1)
	  {
		  BSP_LED_Toggle(LED_RED);
		  HAL_Delay(200);
	  }
  }

  /* Test the correctness */
  if((pQSPI_Info.FlashSize != 0x4000000) || (pQSPI_Info.EraseSectorSize != 0x1000)  ||
     (pQSPI_Info.ProgPageSize != 0x100)  || (pQSPI_Info.EraseSectorsNumber != 16384) ||
     (pQSPI_Info.ProgPagesNumber != 262144))
  {
	  while(1)
	  {
		  BSP_LED_Toggle(LED_RED);
		  HAL_Delay(200);
	  }
  }

  /*##-3- Erase QSPI memory ################################################*/
  if(BSP_QSPI_Erase_Block(WRITE_READ_ADDR) != QSPI_OK)
  {
	  while(1)
	  {
		  BSP_LED_Toggle(LED_RED);
		  HAL_Delay(200);
	  }
  }

  /*##-4- QSPI memory read/write access  #################################*/
  /* Fill the buffer to write */
  QPSI_Fill_Buffer(qspi_aTxBuffer, QSPI_BUFFER_SIZE, 0xD20F);

  /* Write data to the QSPI memory */
  if(BSP_QSPI_Write(qspi_aTxBuffer, QSPI_WRITE_READ_ADDR, QSPI_BUFFER_SIZE) != QSPI_OK)
  {
	  while(1)
	  {
		  BSP_LED_Toggle(LED_RED);
		  HAL_Delay(200);
	  }
  }

  /* Read back data from the QSPI memory */
  if(BSP_QSPI_Read(qspi_aRxBuffer, QSPI_WRITE_READ_ADDR, QSPI_BUFFER_SIZE) != QSPI_OK)
  {
	  while(1)
	  {
		  BSP_LED_Toggle(LED_RED);
		  HAL_Delay(200);
	  }
  }

  /*##-5- Checking data integrity ############################################*/
  if(Buffercmp(qspi_aRxBuffer, qspi_aTxBuffer, QSPI_BUFFER_SIZE) > 0)
  {
	  while(1)
	  {
		  BSP_LED_Toggle(LED_RED);
		  HAL_Delay(200);
	  }
  }


  // BSP LCD Init
  if(BSP_LCD_Init() != LCD_OK)
  {
	  while(1)
	  {
		  BSP_LED_Toggle(LED_RED);
		  HAL_Delay(200);
	  }
  }

  BSP_LCD_LayerDefaultInit(0, LCD_FB_START_ADDRESS);

  BSP_LCD_Clear(LCD_COLOR_WHITE);

  Display_DemoDescription();

  HAL_Delay(2000);


  // STemWin
  __HAL_RCC_CRC_CLK_ENABLE(); /* Enable the CRC Module */

  GUI_Init();

  /* Enable Window Manager Multibuffering */
  WM_MULTIBUF_Enable(1);

  GUI_Clear();
  GUI_SetFont(&GUI_Font32_1);
  GUI_DispStringAt("Hello world!", (LCD_GetXSize()-100)/2, (LCD_GetYSize()-20)/2);
//  HAL_Delay(1000); // cannot use delay
  GUI_Delay(2000);

  CreateWindow();
  GUI_Exec();
  GUI_Delay(2000); // cannot use delay

  MainTask();


  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
//	  HAL_GPIO_TogglePin(GPIOJ, GPIO_PIN_5);
//	  BSP_LED_Toggle(LED_RED);
	  BSP_LED_Toggle(LED_GREEN);
	  HAL_Delay(200);

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

  /** Configure LSE Drive Capability
  */
  HAL_PWR_EnableBkUpAccess();
  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_BYPASS;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 25;
  RCC_OscInitStruct.PLL.PLLN = 400;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Activate the Over-Drive mode
  */
  if (HAL_PWREx_EnableOverDrive() != HAL_OK)
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

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_6) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();

}

/* USER CODE BEGIN 4 */

/**
  * @brief  Fills buffer with user predefined data.
  * @param  pBuffer: pointer on the buffer to fill
  * @param  uwBufferLenght: size of the buffer to fill
  * @param  uwOffset: first value to fill on the buffer
  * @retval None
  */
static void Fill_Buffer(uint32_t *pBuffer, uint32_t uwBufferLenght, uint32_t uwOffset)
{
  uint32_t tmpIndex = 0;

  /* Put in global buffer different values */
  for (tmpIndex = 0; tmpIndex < uwBufferLenght; tmpIndex++ )
  {
    pBuffer[tmpIndex] = tmpIndex + uwOffset;
  }
}

/**
* @brief  Fills buffer with user predefined data.
* @param  pBuffer: pointer on the buffer to fill
* @param  uwBufferLenght: size of the buffer to fill
* @param  uwOffset: first value to fill on the buffer
* @retval None
*/
static void QPSI_Fill_Buffer(uint8_t *pBuffer, uint32_t uwBufferLenght, uint32_t uwOffset)
{
  uint32_t tmpIndex = 0;

  /* Put in global buffer different values */
  for (tmpIndex = 0; tmpIndex < uwBufferLenght; tmpIndex++ )
  {
    pBuffer[tmpIndex] = tmpIndex + uwOffset;
  }
}

/**
* @brief  Compares two buffers.
* @param  pBuffer1, pBuffer2: buffers to be compared.
* @param  BufferLength: buffer's length
* @retval 1: pBuffer identical to pBuffer1
*         0: pBuffer differs from pBuffer1
*/
static uint8_t Buffercmp(uint8_t* pBuffer1, uint8_t* pBuffer2, uint32_t BufferLength)
{
  while (BufferLength--)
  {
    if (*pBuffer1 != *pBuffer2)
    {
      return 1;
    }

    pBuffer1++;
    pBuffer2++;
  }

  return 0;
}

/**
  * @brief  Display main demo messages
  * @param  None
  * @retval None
  */
static void Display_DemoDescription(void)
{
  char desc[] = "Victor T";

  /* Set LCD Foreground Layer  */
  BSP_LCD_SelectLayer(0);

  BSP_LCD_SetFont(&LCD_DEFAULT_FONT);

  /* Clear the LCD */
  BSP_LCD_SetBackColor(LCD_COLOR_WHITE);
  BSP_LCD_Clear(LCD_COLOR_WHITE);

  /* Set the LCD Text Color */
  BSP_LCD_SetTextColor(LCD_COLOR_DARKBLUE);

  /* Display LCD messages */
  BSP_LCD_DisplayStringAt(0, 10, (uint8_t *)"STM32F769I BSP", CENTER_MODE);
  BSP_LCD_DisplayStringAt(0, 35, (uint8_t *)"Drivers examples", CENTER_MODE);

  /* Draw Bitmap */
  BSP_LCD_DrawBitmap((BSP_LCD_GetXSize() - 80)/2, 65, (uint8_t *)stlogo);

  BSP_LCD_SetFont(&Font12);
  BSP_LCD_DisplayStringAt(0, BSP_LCD_GetYSize()- 20, (uint8_t *)"Copyright (c) STMicroelectronics 2016", CENTER_MODE);

  BSP_LCD_SetFont(&Font16);
  BSP_LCD_SetTextColor(LCD_COLOR_BLUE);
  BSP_LCD_FillRect(0, BSP_LCD_GetYSize()/2 + 15, BSP_LCD_GetXSize(), 60);
  BSP_LCD_SetTextColor(LCD_COLOR_WHITE);
  BSP_LCD_SetBackColor(LCD_COLOR_BLUE);
  BSP_LCD_DisplayStringAt(0, BSP_LCD_GetYSize() / 2 + 30, (uint8_t *)"Press User button to start :", CENTER_MODE);
  BSP_LCD_DisplayStringAt(0, BSP_LCD_GetYSize()/2 + 45, (uint8_t *)desc, CENTER_MODE);
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

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
