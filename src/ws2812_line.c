#include "stdbool.h"
#include "ws2812_line.h"


typedef struct {
    uint8_t green[8];
    uint8_t red[8];
    uint8_t blue[8];
} led_pwm_config_t;

typedef enum {
    DMA_TRANSFERING_START_RESET,
    DMA_TRANSFERING_LED,
    DMA_TRANSFERING_FINAL_RESET
} dma_state_t;
/* Variables -----------------------------------------------*/
//static uint8_t LEDbuffer[LED_DATA_SIZE];
static led_pwm_config_t LEDConfigMain = {{WS2812_0}, {WS2812_0}, {WS2812_0}};
static const led_pwm_config_t LEDConfigReset = {{WS2812_RESET}, {WS2812_RESET}, {WS2812_RESET}};
static bool LEDConfigNeedUpdate = false;
static led_pwm_config_t LEDConfigTemp = {{WS2812_0}, {WS2812_0}, {WS2812_0}};
//static led_pwm_config_t LEDConfigTemp = {{WS2812_0, WS2812_0, WS2812_0, WS2812_0, WS2812_1, WS2812_1, WS2812_1, WS2812_1},
//                                         {WS2812_0, WS2812_0, WS2812_0, WS2812_0, WS2812_0, WS2812_0, WS2812_0, WS2812_0},
//                                         {WS2812_0, WS2812_0, WS2812_0, WS2812_0, WS2812_0, WS2812_0, WS2812_0, WS2812_0}};
//static const uint8_t LEDResetBuffer[RESET_SLOTS] = {WS2812_RESET};

TIM_HandleTypeDef TimHandle;
TIM_OC_InitTypeDef sConfig;
GPIO_InitTypeDef GPIO_InitStruct;
DMA_HandleTypeDef hdma_tim;

/* Functions -----------------------------------------------*/

void TIMEx_DMACommutationHalfCplt(DMA_HandleTypeDef *hdma);
void TIMEx_DMAError(DMA_HandleTypeDef *hdma);
static void update_dma_state();

void ws2812_init(void) {


    TimHandle.Instance = TIMx;

    TimHandle.Init.Period = TIMER_PERIOD - 1;
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
    LEDConfigMain = LEDConfigReset;
    /*##-3- Start PWM signal generation in DMA mode ############################*/
    HAL_TIM_PWM_Start_DMA(&TimHandle, TIMx_CHANNEL, (uint32_t *)&LEDConfigMain, sizeof(LEDConfigMain));
}

void ws2812_update(void) {
    //HAL_TIM_PWM_ConfigChannel(&TimHandle, &sConfig, TIMx_CHANNEL);
    //HAL_TIM_PWM_Start_DMA(&TimHandle, TIMx_CHANNEL, (uint32_t *) LEDbuffer, LED_BUFFER_SIZE);
    static uint8_t red = 0;
    static uint8_t green = 0;
    static uint8_t blue = 0;

    if(fillLEDcolor(red, green, blue)) {

        if(blue >= 150) {
            blue = 0;
        } else {
            if(green >= 150) {
                green = 0;
                blue += 10;
            } else {
                if(red >= 150) {
                    red = 0;
                    green += 10;
                } else {
                    red += 10;
                }
            }
        }
    }
}

bool fillLEDcolor(uint8_t RED, uint8_t GREEN, uint8_t BLUE) {
    uint32_t i;
    // wait until DMA ends transfer and updates main config
    if(LEDConfigNeedUpdate) {
        return false;
    }

    for (i = 0; i < 8; i++) // GREEN data
        LEDConfigTemp.green[i] = ((GREEN << i) & 0x80) ? WS2812_1 : WS2812_0;
    for (i = 0; i < 8; i++) // RED
        LEDConfigTemp.red[i] = ((RED << i) & 0x80) ? WS2812_1 : WS2812_0;
    for (i = 0; i < 8; i++) // BLUE
        LEDConfigTemp.blue[i] = ((BLUE << i) & 0x80) ? WS2812_1 : WS2812_0;

    LEDConfigNeedUpdate = true;
    return true;
}

void TIMx_DMA_IRQHandler(void) {
    DMA_HandleTypeDef *hdma = TimHandle.hdma[TIM_DMA_ID_CC3]; 

    /* Transfer Complete flag */
    if(__HAL_DMA_GET_TC_FLAG_INDEX(hdma)) {
        update_dma_state();
    }
    /* Half Transfer Complete flag */
    if (__HAL_DMA_GET_HT_FLAG_INDEX(hdma)) {
      
    }
    HAL_DMA_IRQHandler(hdma);    

}

/* Extension Callback *********************************************************/
void HAL_TIMEx_CommutationCallback(TIM_HandleTypeDef *htim) {
}

void HAL_TIMEx_BreakCallback(TIM_HandleTypeDef *htim) {
}

void TIMEx_DMACommutationCplt(DMA_HandleTypeDef *hdma) {
    update_dma_state();
}

void TIMEx_DMACommutationHalfCplt(DMA_HandleTypeDef *hdma) {
    //update_dma_state();
}

void TIMEx_DMAError(DMA_HandleTypeDef *hdma) {

}


static void update_dma_state() {
    static dma_state_t state = DMA_TRANSFERING_START_RESET;
    static uint32_t counter = 0;


    counter++;
    switch(state) {
    case DMA_TRANSFERING_START_RESET:
        if(counter >= 10) {
            LEDConfigMain = LEDConfigTemp;
            state = DMA_TRANSFERING_LED;
            counter = 0;
        }
        break;
    case DMA_TRANSFERING_LED:
        
        if(LEDConfigNeedUpdate) {
            LEDConfigMain = LEDConfigTemp;
            LEDConfigNeedUpdate = false;
        }
        if(counter >= 2*LED_NUMBER) {
            LEDConfigMain = LEDConfigReset;
            state = DMA_TRANSFERING_FINAL_RESET;
            counter = 0;
        }
        break;
    case DMA_TRANSFERING_FINAL_RESET:
        if(counter >= 10) {
            LEDConfigMain = LEDConfigReset;
            state = DMA_TRANSFERING_START_RESET;
            counter = 0;
        }
        break;
    default:
        state = DMA_TRANSFERING_START_RESET;
        break;
    }
}
