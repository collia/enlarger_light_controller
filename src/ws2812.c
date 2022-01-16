#include "ws2812.h"

/*
 * This code was taken from 
 * http://fabioangeletti.altervista.org/blog/stm32-interface-ws2812b/?doing_wp_cron=1528043483.7364630699157714843750
 */

/* Variables -----------------------------------------------*/
static uint8_t LEDbuffer[LED_BUFFER_SIZE];

TIM_HandleTypeDef TimHandle;
TIM_OC_InitTypeDef sConfig;
GPIO_InitTypeDef GPIO_InitStruct;
DMA_HandleTypeDef hdma_tim;

/* Functions -----------------------------------------------*/

/**
 * @brief TIM MSP Initialization
 *        This function configures the hardware resources used in this example:
 *           - Peripheral's clock enable
 *           - Peripheral's GPIO Configuration
 *           - DMA configuration for transmission request by peripheral
 * @param htim: TIM handle pointer
 * @retval None
 */
void HAL_TIM_PWM_MspInit(TIM_HandleTypeDef *htim) {
    /*##-1- Enable peripherals and GPIO Clocks #################################*/
    /* TIMx clock enable */
    TIMx_CLK_ENABLE();

    /* Enable GPIO Channel Clock */
    TIMx_CHANNEL3_GPIO_CLK_ENABLE();

    /* Enable DMA clock */
	DMAx_CLK_ENABLE();

    TIMx_GPIO_TIM2_3_REMAP();
	/* Configure TIM1_Channel1 in output, push-pull & alternate function mode */
    GPIO_InitStruct.Pin = GPIO_PIN_CHANNEL1;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP; //GPIO_MODE_OUTPUT_PP
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;

    HAL_GPIO_Init(TIMx_GPIO_CHANNEL1_PORT, &GPIO_InitStruct);

    /* Set the parameters to be configured */
    //hdma_tim.Init.Request = TIMx_CC1_DMA_REQUEST;
    hdma_tim.Init.Direction = DMA_MEMORY_TO_PERIPH;
    hdma_tim.Init.PeriphInc = DMA_PINC_DISABLE;
    hdma_tim.Init.MemInc = DMA_MINC_ENABLE;
    hdma_tim.Init.PeriphDataAlignment = DMA_PDATAALIGN_WORD;
    hdma_tim.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
    hdma_tim.Init.Mode = DMA_CIRCULAR;
    hdma_tim.Init.Priority = DMA_PRIORITY_HIGH;

    /* Set hdma_tim instance */
    hdma_tim.Instance = TIMx_CC1_DMA_INST;

    /* Link hdma_tim to hdma[TIM_DMA_ID_CC3] (channel3) */
    __HAL_LINKDMA(htim, hdma[TIMx_DMA_CHANNEL], hdma_tim);

    /* Initialize TIMx DMA handle */
    HAL_DMA_Init(htim->hdma[TIMx_DMA_CHANNEL]);

    /*##-2- Configure the NVIC for DMA #########################################*/
    /* NVIC configuration for DMA transfer complete interrupt */
    HAL_NVIC_SetPriority(TIMx_DMA_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(TIMx_DMA_IRQn);
}

void ws2812_init(void) {
    fillBufferBlack();
    //fillBufferWhite();

    TimHandle.Instance = TIMx;

    TimHandle.Init.Period = TIMER_PERIOD - 1;
    TimHandle.Init.RepetitionCounter = LED_BUFFER_SIZE + 1; //LED_BUFFER_SIZE + 1;//0xFFFF;
    TimHandle.Init.Prescaler = (uint32_t)((SystemCoreClock / TIMER_CLOCK_FREQ) - 1);
    TimHandle.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    TimHandle.Init.CounterMode = TIM_COUNTERMODE_UP;

    //TimHandle.Init.Period            = 1999;//PERIOD_VALUE;
    TimHandle.Init.RepetitionCounter = 0;
    TimHandle.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;

    HAL_TIM_PWM_Init(&TimHandle);

    /*##-2- Configure the PWM channel 3 ########################################*/

    sConfig.Pulse        = 0;//TIMER_PERIOD/2;

    sConfig.OCMode       = TIM_OCMODE_PWM1;
    sConfig.OCPolarity   = TIM_OCPOLARITY_HIGH;
    sConfig.OCFastMode   = TIM_OCFAST_DISABLE;
    sConfig.OCNPolarity  = TIM_OCNPOLARITY_HIGH;
    sConfig.OCNIdleState = TIM_OCNIDLESTATE_RESET;
    
    sConfig.OCIdleState  = TIM_OCIDLESTATE_RESET;


    HAL_TIM_PWM_ConfigChannel(&TimHandle, &sConfig, TIMx_CHANNEL);

    //HAL_TIM_PWM_Start(&TimHandle, TIMx_CHANNEL);
    /*##-3- Start PWM signal generation in DMA mode ############################*/
    HAL_TIM_PWM_Start_DMA(&TimHandle, TIMx_CHANNEL, (uint32_t *) LEDbuffer, LED_BUFFER_SIZE);
}

void ws2812_update(void) {
    static uint16_t i =0;
    //HAL_TIM_PWM_ConfigChannel(&TimHandle, &sConfig, TIMx_CHANNEL);
    //HAL_TIM_PWM_Start_DMA(&TimHandle, TIMx_CHANNEL, (uint32_t *) LEDbuffer, LED_BUFFER_SIZE);
    //HAL_GPIO_TogglePin(TIMx_GPIO_CHANNEL1_PORT, GPIO_PIN_CHANNEL1);
    if( i < 16) {
        setLEDcolor(i %16, 255, 0, 0);
        i++;
    } else if( i< 16*2) {
        setLEDcolor(i %16, 0, 255, 0);
        i++;
    } else if( i < 16*3) {
        setLEDcolor(i %16, 0, 0, 255);
        i++;
    } else {
        i = 0;
    }
}

void setLEDcolor(uint32_t LEDnumber, uint8_t RED, uint8_t GREEN, uint8_t BLUE) {
    uint8_t tempBuffer[24];
    uint32_t i;
    uint32_t LEDindex;
    LEDindex = LEDnumber % LED_NUMBER;

    for (i = 0; i < 8; i++) // GREEN data
        tempBuffer[i] = ((GREEN << i) & 0x80) ? WS2812_1 : WS2812_0;
    for (i = 0; i < 8; i++) // RED
        tempBuffer[8 + i] = ((RED << i) & 0x80) ? WS2812_1 : WS2812_0;
    for (i = 0; i < 8; i++) // BLUE
        tempBuffer[16 + i] = ((BLUE << i) & 0x80) ? WS2812_1 : WS2812_0;

    for (i = 0; i < 24; i++)
        LEDbuffer[RESET_SLOTS_BEGIN + LEDindex * 24 + i] = tempBuffer[i];
}

void setWHOLEcolor(uint8_t RED, uint8_t GREEN, uint8_t BLUE) {
    uint32_t index;

    for (index = 0; index < LED_NUMBER; index++)
        setLEDcolor(index, RED, GREEN, BLUE);
}

void fillBufferBlack(void) {
    /*Fill LED buffer - ALL OFF*/
    uint32_t index, buffIndex;
    buffIndex = 0;

    for (index = 0; index < RESET_SLOTS_BEGIN; index++) {
        LEDbuffer[buffIndex] = WS2812_RESET;
        buffIndex++;
    }
    for (index = 0; index < LED_DATA_SIZE; index++) {
        LEDbuffer[buffIndex] = WS2812_0;
        buffIndex++;
    }
    LEDbuffer[buffIndex] = WS2812_0;
    buffIndex++;
    for (index = 0; index < RESET_SLOTS_END; index++) {
        LEDbuffer[buffIndex] = 0;
        buffIndex++;
    }
}

void fillBufferWhite(void) {
    /*Fill LED buffer - ALL OFF*/
    uint32_t index, buffIndex;
    buffIndex = 0;

    for (index = 0; index < RESET_SLOTS_BEGIN; index++) {
        LEDbuffer[buffIndex] = WS2812_RESET;
        buffIndex++;
    }
    for (index = 0; index < LED_DATA_SIZE; index++) {
        LEDbuffer[buffIndex] = WS2812_1;
        buffIndex++;
    }
    LEDbuffer[buffIndex] = WS2812_0;
    buffIndex++;
    for (index = 0; index < RESET_SLOTS_END; index++) {
        LEDbuffer[buffIndex] = WS2812_1;
        buffIndex++;
    }
}

void TIMx_DMA_IRQHandler(void) {
    HAL_DMA_IRQHandler(TimHandle.hdma[TIM_DMA_ID_CC3]);
}

/* Extension Callback *********************************************************/
void HAL_TIMEx_CommutationCallback(TIM_HandleTypeDef *htim) {
}

void HAL_TIMEx_BreakCallback(TIM_HandleTypeDef *htim) {
}

void TIMEx_DMACommutationCplt(DMA_HandleTypeDef *hdma) {
}
