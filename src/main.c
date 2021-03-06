/**
  ******************************************************************************
  * @file    src/main.c
  * @author  MCD Application Team
  * @version V1.5.0
  * @date    14-April-2017
  * @brief   Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT(c) 2016 STMicroelectronics</center></h2>
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
/* Scheduler includes. */
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"

/* Library includes. */
#if defined(STM32F407xx)
#include "stm32f4xx_it.h"
#include "stm32f4xx_hal.h"
#elif  defined(STM32F103xB)
#include "stm32f1xx_it.h"
#endif

#include "main.h"
#include "bsp.h"
#include "max7219.h"
#include "controls.h"
#include "ws2812_line.h"

#define mainBLINK_PERIOD			( ( TickType_t ) 1000 / portTICK_PERIOD_MS )


typedef enum {
    START_BUTTON = 0,
    CONTRAST_BUTTON,
    BRITHNESS_BUTTON,
    TIME_BUTTON,
    SAFELIGHT_BUTTON,
    WHITE_BUTTON,
    NO_LIGHT,
    VALUE_ENC_LEFT,
    VALUE_ENC_RIGHT,
    NO_BUTTON,
    MAX_MAIN_MESSAGE
} main_message_t;



/* The 'check' task as described at the top of this file. */
static void prvBlinkTask( void *pvParameters );
static void prvMainTask( void *pvParameters );
static void prvControlTask( void *pvParameters );

xQueueHandle key_queue;
xQueueHandle message_queue;

void key_pressed_ISR(uint16_t key)
{
    BaseType_t xHigherPriorityTaskWoken;

    /* We have not woken a task at the start of the ISR. */
    xHigherPriorityTaskWoken = pdFALSE;
    xQueueSendFromISR( key_queue, &key, &xHigherPriorityTaskWoken );
}

int main(void)
{
    /* STM32F103xB HAL library initialization:
      - Configure the Flash prefetch
      - Systick timer is configured by default as source of time base, but user
      can eventually implement his proper time base source (a general purpose
      timer for example or other time source), keeping in mind that Time base
      duration should be kept 1ms since PPP_TIMEOUT_VALUEs are defined and
      handled in milliseconds basis.
      - Set NVIC Group Priority to 4
      - Low Level Initialization
    */
    HAL_Init();
    SystemInit();
    //SystemCoreClockUpdate();
    /* Configure the system clock to 64 MHz */

   	/* Set up the clocks and memory interface. */
	SystemClock_Config();
    controls_init();

    /* Create the 'check' task, which is also defined within this file. */
	xTaskCreate( prvBlinkTask, "Blink", configMINIMAL_STACK_SIZE, NULL, tskIDLE_PRIORITY, NULL );
    xTaskCreate( prvMainTask, "Main", configMINIMAL_STACK_SIZE, NULL, tskIDLE_PRIORITY, NULL );
    xTaskCreate( prvControlTask, "Controls", configMINIMAL_STACK_SIZE, NULL, tskIDLE_PRIORITY, NULL );

    key_queue = xQueueCreate( 80, sizeof( uint16_t ) );
    if( key_queue == NULL )
    {
        for(;;);
    }
    message_queue = xQueueCreate( 8, sizeof( main_message_t ) );
    if( message_queue == NULL )
    {
        for(;;);
    }
    /* Start the scheduler. */
	vTaskStartScheduler();

    /* Will only get here if there was insufficient memory to create the idle
    task.  The idle task is created within vTaskStartScheduler(). */
	for( ;; ) {

    }

}

#if  defined(STM32F103xB)
/**
  * @brief  System Configuration
  *         The system Clock is configured as follow :
  *            System Clock source            = PLL (HSI)
  *            SYSCLK(Hz)                     = 64000000
  *            HCLK(Hz)                       = 64000000
  *            AHB Prescaler                  = 1
  *            APB1 Prescaler                 = 2
  *            APB2 Prescaler                 = 1
  *            PLLMUL                         = 16
  *            Flash Latency(WS)              = 2
  * @param  None
  * @retval None
  */
static void SystemClock_Config( void )
{
    RCC_ClkInitTypeDef clkinitstruct = {0};
    RCC_OscInitTypeDef oscinitstruct = {0};
    GPIO_InitTypeDef  GPIO_InitStruct;

    /* Configure PLL ------------------------------------------------------*/
    /* PLL configuration: PLLCLK = (HSI / 2) * PLLMUL = (8 / 2) * 16 = 64 MHz */
    /* PREDIV1 configuration: PREDIV1CLK = PLLCLK / HSEPredivValue = 64 / 1 = 64 MHz */
    /* Enable HSI and activate PLL with HSi_DIV2 as source */
    oscinitstruct.OscillatorType  = RCC_OSCILLATORTYPE_HSI;
    oscinitstruct.HSEState        = RCC_HSE_OFF;
    oscinitstruct.LSEState        = RCC_LSE_OFF;
    oscinitstruct.HSIState        = RCC_HSI_ON;
    oscinitstruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
    oscinitstruct.HSEPredivValue    = RCC_HSE_PREDIV_DIV1;
    oscinitstruct.PLL.PLLState    = RCC_PLL_ON;
    oscinitstruct.PLL.PLLSource   = RCC_PLLSOURCE_HSI_DIV2;
    oscinitstruct.PLL.PLLMUL      = RCC_PLL_MUL16;
    if (HAL_RCC_OscConfig(&oscinitstruct)!= HAL_OK)
    {
        /* Initialization Error */
        while(1);
    }

  /* Select PLL as system clock source and configure the HCLK, PCLK1 and PCLK2
     clocks dividers */
    clkinitstruct.ClockType = (RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2);
    clkinitstruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
    clkinitstruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
    clkinitstruct.APB2CLKDivider = RCC_HCLK_DIV1;
    clkinitstruct.APB1CLKDivider = RCC_HCLK_DIV2;
    if (HAL_RCC_ClockConfig(&clkinitstruct, FLASH_LATENCY_2)!= HAL_OK)
    {
        /* Initialization Error */
        while(1);
    }

    /* -1- Enable GPIO Clock (to be able to program the configuration registers) */
    LED1_GPIO_CLK_ENABLE();

    /* -2- Configure IO in output push-pull mode to drive external LEDs */
    GPIO_InitStruct.Mode  = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull  = GPIO_PULLDOWN;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;

    GPIO_InitStruct.Pin = LED1_PIN;
    HAL_GPIO_Init(LED1_GPIO_PORT, &GPIO_InitStruct);
}
#endif

/* Described at the top of this file. */
static void prvBlinkTask( void *pvParameters )
{
    TickType_t xLastExecutionTime;
    unsigned long ulTicksToWait = mainBLINK_PERIOD;

	/* Just to remove the compiler warning about the unused parameter. */
	( void ) pvParameters;

	/* Initialise the variable used to control our iteration rate prior to
	its first use. */
	xLastExecutionTime = xTaskGetTickCount();

	for( ;; )
	{
		/* Wait until it is time to run the tests again. */
		vTaskDelayUntil( &xLastExecutionTime, ulTicksToWait );

        HAL_GPIO_TogglePin(LED1_GPIO_PORT, LED1_PIN);
        //ws2812_update();
    }
}

/* Described at the top of this file. */
static void prvMainTask( void *pvParameters )
{
    main_message_t m;

	/* Just to remove the compiler warning about the unused parameter. */
	( void ) pvParameters;

    bsp_MX_GPIO_Init();
    bsp_MX_DMA_Init();
    bsp_MX_SPI1_Init();
    bsp_MX_TIM2_Init();
    bsp_MX_TIM1_Init();

    screen_init();
    ws2812_init();

    uint8_t bri = 10;
    uint8_t con = 10;
    uint8_t time_sec = 10;
    uint8_t* current_value = &time_sec;

    screen_set_mode(SCREEN_TIME_MODE);
    screen_set_value(*current_value);
	for( ;; )
	{
        xQueueReceive(message_queue, &( m ), portMAX_DELAY );
        switch(m) {
        case TIME_BUTTON:
            screen_set_mode(SCREEN_TIME_MODE);
            current_value = &time_sec;
            break;
        case CONTRAST_BUTTON:
            screen_set_mode(SCREEN_CONTRAST_MODE);
            current_value = &con;
            break;
        case BRITHNESS_BUTTON:
            screen_set_mode(SCREEN_BRIGHNESS_MODE);
            current_value = &bri;
            break;
        case START_BUTTON:
            screen_set_mode(SCREEN_EMPTY_MODE);
            break;
        case SAFELIGHT_BUTTON:
            fillLEDcolor(100, 0, 0);
            break;
        case WHITE_BUTTON:
            fillLEDcolor(200, 200, 200);
            break;
        case NO_LIGHT:
            fillLEDcolor(0, 0, 0);
            break;
        case VALUE_ENC_LEFT:
            if(*current_value > 0)
                (*current_value)--;
            break;
        case VALUE_ENC_RIGHT:
            if(*current_value < 100)
                (*current_value)++;
            break;
        default:
            //Do nothing
            break;
        }
        screen_set_value(*current_value);

    }
}

/* Described at the top of this file. */
static void prvControlTask( void *pvParameters )
{

    uint16_t key;
    main_message_t m;
    //bool enc_left_get = false;
    //bool enc_right_get = false;

	for( ;; )
	{
        m = MAX_MAIN_MESSAGE;
        xQueueReceive(key_queue, &( key ), portMAX_DELAY );

        switch(key) {
        case BUTTON_BRITHNESS_PIN:
            m = BRITHNESS_BUTTON;
            break;
        case BUTTON_CONTRAST_PIN:
            m = CONTRAST_BUTTON;
            break;
        case BUTTON_TIME_PIN:
            m = TIME_BUTTON;
            break;
        case BUTTON_START_PIN:
            m = START_BUTTON;
            break;
        case BUTTON_SAFELIGHT_PIN:
        case BUTTON_WHITE_PIN:
            switch(check_white_safe_led_switch()) {
            case BUTTON_WHITE_PIN:
                m = WHITE_BUTTON;
                break;
            case BUTTON_SAFELIGHT_PIN:
                m = SAFELIGHT_BUTTON;
                break;
            default:
                m = NO_LIGHT;
                break;
            };
            break;
            /*case BUTTON_ENCODER_LEFT_PIN:
            m = VALUE_ENC_RIGHT;
            break;
        case BUTTON_ENCODER_RIGHT_PIN:
            m = VALUE_ENC_LEFT;
            break;*/
        default:
            //Do nothing
            break;
        }
        if(m != MAX_MAIN_MESSAGE) {
            xQueueSend(message_queue, ( void * ) &m, ( TickType_t ) 0 );
        }

    }
}


#ifdef  USE_FULL_ASSERT

/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t* file, uint32_t line)
{
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */

  /* Infinite loop */
  while (1)
  {
  }
}
#endif
