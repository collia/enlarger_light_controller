
/* Includes ------------------------------------------------------------------*/

#include <stdbool.h>
#include <stdint.h>

/* Scheduler includes. */
#include "FreeRTOS.h"
#include "task.h"

#include "main.h"
/* Library includes. */
#include "stm32f1xx_it.h"

#include "stm32f1xx_hal.h"
#include "stm32f1xx_hal_rcc.h"
#include "stm32f1xx_ll_exti.h"
#include "stm32f1xx_ll_spi.h"
#include "stm32f1xx_ll_bus.h"
#include "stm32f1xx_ll_system.h"
#include "stm32f1xx_ll_utils.h"
#include "stm32f1xx_ll_gpio.h"
#include "stm32f1xx_ll_pwr.h"


#include "controls.h"


void controls_init() {
    GPIO_InitTypeDef  GPIO_InitStruct;
    __HAL_RCC_GPIOA_CLK_ENABLE();
    
    GPIO_InitStruct.Mode  = GPIO_MODE_IT_FALLING;
    GPIO_InitStruct.Pull  = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    
    GPIO_InitStruct.Pin = BUTTON_BRITHNESS_PIN;
    HAL_GPIO_Init(BUTTON_BRITHNESS_GPIO, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = BUTTON_CONTRAST_PIN;
    HAL_GPIO_Init(BUTTON_CONTRAST_GPIO, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = BUTTON_START_PIN;
    HAL_GPIO_Init(BUTTON_START_GPIO, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = BUTTON_ENCODER_LEFT_PIN;
    HAL_GPIO_Init(BUTTON_ENCODER_LEFT_GPIO, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = BUTTON_ENCODER_RIGHT_PIN;
    HAL_GPIO_Init(BUTTON_ENCODER_RIGHT_GPIO, &GPIO_InitStruct);
#if 0
    HAL_NVIC_SetPriority(EXTI9_5_IRQn, 3, 0);
    HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);
    HAL_NVIC_SetPriority(EXTI15_10_IRQn, 3, 0);
    HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);
#else
    HAL_NVIC_SetPriority(EXTI0_IRQn, 3, 0);
    HAL_NVIC_EnableIRQ(EXTI0_IRQn);
    HAL_NVIC_SetPriority(EXTI1_IRQn, 3, 0);
    HAL_NVIC_EnableIRQ(EXTI1_IRQn);
    HAL_NVIC_SetPriority(EXTI2_IRQn, 3, 0);
    HAL_NVIC_EnableIRQ(EXTI2_IRQn);
    HAL_NVIC_SetPriority(EXTI3_IRQn, 3, 0);
    HAL_NVIC_EnableIRQ(EXTI3_IRQn);
    HAL_NVIC_SetPriority(EXTI4_IRQn, 3, 0);
    HAL_NVIC_EnableIRQ(EXTI4_IRQn);

    HAL_NVIC_SetPriority(EXTI9_5_IRQn, 3, 0);
    HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);


#endif
}

/**
  * @brief EXTI line detection callbacks
  * @param GPIO_Pin: Specifies the pins connected EXTI line
  * @retval None
  */
void HAL_GPIO_EXTI_Callback (uint16_t GPIO_Pin)
{
    #define GPIO_FILTER_DELAY_TICKS 20

    TickType_t ticks = xTaskGetTickCountFromISR();
    static TickType_t ticks_prev = 0;
    static uint16_t previous_pin = -1;

    if(GPIO_Pin == BUTTON_ENCODER_LEFT_PIN) {
        if(GPIO_PIN_RESET == HAL_GPIO_ReadPin(BUTTON_ENCODER_RIGHT_GPIO, BUTTON_ENCODER_RIGHT_PIN)) {
             key_pressed_ISR(GPIO_Pin);
        }
    } else if (GPIO_Pin == BUTTON_ENCODER_RIGHT_PIN) {
        if(GPIO_PIN_RESET == HAL_GPIO_ReadPin(BUTTON_ENCODER_LEFT_GPIO, BUTTON_ENCODER_LEFT_PIN)) {
            key_pressed_ISR(GPIO_Pin);
        }        
    } else {
        if(previous_pin != GPIO_Pin || (ticks - ticks_prev > GPIO_FILTER_DELAY_TICKS)) {   
            key_pressed_ISR(GPIO_Pin);
            previous_pin = GPIO_Pin ;
            ticks_prev = ticks;
        }
    }
        
}
