## Study how to init all the peripherals by BSP Drivers. RCC is the only initialized in CubeMX with 200Mhz as this is the optimized freq for "most" peripherals


## CubeMX Initiallization

RCC =  
	RCC_OscInitStruct.PLL.PLLM = 25;  
	RCC_OscInitStruct.PLL.PLLN = 400;  
	RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;  
	RCC_OscInitStruct.PLL.PLLQ = 4;  
  
Cortex M7, All enabled   
	ART Acc  
	CPU ICache  
	CPU DCache  
	
	
### BSP LED & PB Init

Create Drivers\BSP\STM32F769I-Discovery
Copy stm32f769i_discovery.c/h to Drivers\BSP\STM32F769I-Discovery
main.h add
	#include "stm32f769i_discovery.h"
add to path   
	Drivers\BSP\STM32F769I-Discovery

init the LEDs
	BSP_LED_Init(LED_GREEN);
	BSP_LED_Init(LED_RED);
	
toggle the LEDs
	BSP_LED_Toggle(LED_RED);
	BSP_LED_Toggle(LED_GREEN);

add  
	uint8_t CheckForUserInput(void)
	
	
### BSP SDRAM Init

copy stm32f769i_discovery_sdram.c/h to Drivers\BSP\STM32F769I-Discovery

main.h add
	#include "stm32f769i_discovery_sdram.h"
	
in stm32f7xx_hal.h, then goto stm32f7xx_hal_conf.h  enable SDRAM by removing comment
	#define HAL_SDRAM_MODULE_ENABLED

copy into Drivers\STM32F7xx_HAL_Driver\Src and Drivers\STM32F7xx_HAL_Driver\Inc
	stm32f7xx_hal_sdram.c/h
	stm32f7xx_ll_fmc.c/h
	
in main.c add
  // SDRAM BSP Init
  if(BSP_SDRAM_Init() != SDRAM_OK)
  {
	  while(1)
		  {
			  BSP_LED_Toggle(LED_RED);
			  HAL_Delay(200);
		  }
  } // BSP_SDRAM_Initialization_sequence() already inside
	
test SDRAM
	Fill_Buffer(aTxBuffer, BUFFER_SIZE, 0xA244250F);
	*(__IO uint32_t*) (SDRAM_DEVICE_ADDR + WRITE_READ_ADDR + 4*uwIndex) = aTxBuffer[uwIndex]; // write
	aRxBuffer[uwIndex] = *(__IO uint32_t*) (SDRAM_DEVICE_ADDR + WRITE_READ_ADDR + 4*uwIndex); // read


## BSP QSPI Init

copy stm32f769i_discovery_qspi.c/h to Drivers\BSP\STM32F769I-Discovery

main.h add  
	#include "stm32f769i_discovery_qspi.h"
	
Create Drivers\BSP\Components  
	add mx25l512 folder
	
copy into Drivers\STM32F7xx_HAL_Driver\Src and Drivers\STM32F7xx_HAL_Driver\Inc
	stm32f7xx_hal_qspi.c/h

in stm32f7xx_hal.h, then goto stm32f7xx_hal_conf.h  enable SDRAM by removing comment
	#define HAL_QSPI_MODULE_ENABLED   
	
in main.c add
  if(BSP_QSPI_Init() != QSPI_OK)
  {
	  while(1)
		  {
			  BSP_LED_Toggle(LED_RED);
			  HAL_Delay(200);
		  }
  }
	
test SQPI
	BSP_QSPI_GetInfo();
	BSP_QSPI_Erase_Block();
	QPSI_Fill_Buffer();
	BSP_QSPI_Write();
	BSP_QSPI_Read();
	Buffercmp();


### BSP LCD Init 

copy stm32f769i_discovery_lcd.c/h to Drivers\BSP\STM32F769I-Discovery

main.h add
	#include "stm32f769i_discovery_lcd.h"  
	
Create Drivers\BSP\Components
add folders
	adv7533 and
	otm8009a

Create Utilities\Fonts
copy all files inside

Create BSP\Components\Common
copy
	audio.h
	
copy into Drivers\STM32F7xx_HAL_Driver\Src and Drivers\STM32F7xx_HAL_Driver\Inc   
	stm32f7xx_hal_dsi.c/h
	stm32f7xx_hal_ltdc.c/h
	stm32f7xx_hal_ltdc_ex.c/h
	stm32f7xx_hal_dma2d.c/h
	
enable  
	#define HAL_DSI_MODULE_ENABLED  
	#define HAL_LTDC_MODULE_ENABLED   
	#define HAL_DMA2D_MODULE_ENABLED    
	
in main.c add
	if(BSP_LCD_Init() != LCD_OK)
	{
	while(1)
		{
		  BSP_LED_Toggle(LED_RED);
		  HAL_Delay(200);
		}
	}
  
test LCD  
	BSP_LCD_LayerDefaultInit(0, LCD_FB_START_ADDRESS);
	BSP_LCD_Clear(LCD_COLOR_WHITE);
	Display_DemoDescription();

	
### BSP TouchScreen Init

copy stm32f769i_discovery_ts.c/h to Drivers\BSP\STM32F769I-Discovery

main.h add
	#include "stm32f769i_discovery_ts.h"
	
Create Drivers\BSP\Components
	add ft6x06 folder

in Drivers\BSP\Components\Common copy   
	ts.h
	
in main.c add
	/* Touchscreen initialization */
	if(BSP_TS_Init(BSP_LCD_GetXSize(), BSP_LCD_GetYSize()) != TS_OK)
	{
		while(1)
		{
			BSP_LED_Toggle(LED_RED);
			HAL_Delay(200);
		}
	}
  
add global variable   
	TS_StateTypeDef  TS_State = {0};
	

### STemWin

enable CRC in main
	__HAL_RCC_CRC_CLK_ENABLE(); /* Enable the CRC Module */
	
create folder Middlewares\ST\STemWin\inc add all *.h files

add to path includes  
	Middlewares\ST\STemWin\inc
	
Copy all binary files 
	Middlewares\ST\STemWin\Lib

add to "library Paths"
	Middlewares\ST\STemWin\Lib
	
add binary file in "libraries"
	:STemWin_CM7_wc32_ARGB.a
	
copy to Core\Src
	GUI_X.c - from example like "STemWin_helloworld" or repository  
	GUIConf.c - from example like "STemWin_helloworld", repository cause error cannot fit in "RAM"  
	LCDConf.c - from example like "STemWin_helloworld" none in repository  
	
copy to Core\Inc
	GUIConf.h - from example like "STemWin_helloworld", repository missing #define GUI_USE_ARGB (1)    /* The color format to use is ABGR */
	LCDConf.h - from example like "STemWin_helloworld" none in repository

add in main.c  
	#include "WM.h"   
	#include "GUI.h"   
	
in LCDConf.c comment the whole function  
	DSI_IO_WriteCmd(uint32_t NbrParams, uint8_t *pParams)
	
STemWin in main.c
	GUI_Init();
	/* Enable Window Manager Multibuffering */
	WM_MULTIBUF_Enable(1);

	GUI_Clear();
	GUI_SetFont(&GUI_Font32_1);
	GUI_DispStringAt("Hello world!", (LCD_GetXSize()-100)/2, (LCD_GetYSize()-20)/2);
	
	
### GUIBuilder

enable GUI_Delay()  
	
in stm32f7xx_it.c add   
	#include "GUI_ConfDefaults.h"  
	extern volatile GUI_TIMER_TIME OS_TimeMS; 
	
in stm32f7xx_it.c find void SysTick_Handler(void) add   
	OS_TimeMS++;
		
Copy WindowDLG.c


### Example "STemWin_fonts"

copy these files and folders   
	STemWin\App\generated\fonts
	STemWin\App\generated\images
	
copy font_app.c

add in main.c   
	MainTask();   
	

### TouchScreen Interrupt - to do

LDC_INT falling is the interrupt pin (

info in stm32f769i_discovery.h

	/**
	  * @brief Touch screen interrupt signal
	  */
	#define TS_INT_PIN                        ((uint32_t)GPIO_PIN_13)  
	#define TS_INT_GPIO_PORT                  ((GPIO_TypeDef*)GPIOI)  
	#define TS_INT_GPIO_CLK_ENABLE()          __HAL_RCC_GPIOI_CLK_ENABLE()  
	#define TS_INT_GPIO_CLK_DISABLE()         __HAL_RCC_GPIOI_CLK_DISABLE()  
	#define TS_INT_EXTI_IRQn                  EXTI15_10_IRQn  


### Example / Reference

BSP - BSP
STemWin - icons and display seems corrupted, with touch
STemWin_acceleration - just animation of night and moon, no touch
STemWin_animation - just animation of cow, no touch
STemWin_fonts - displays all the fonts and TS at the bottom
STemWin_helloworld - no TS,just display text
STemWin_MemoryDevice - no TS or button, just display

### SUmmary

