
/* Includes ------------------------------------------------------------------*/

#include <stdbool.h>
#include <stdint.h>

/* Scheduler includes. */
#include "FreeRTOS.h"
#include "task.h"

#include "main.h"
/* Library includes. */
#if defined(STM32F407xx)
#include "stm32f4xx_hal.h"
#include "stm32f4xx_hal_rcc.h"
#include "stm32f4xx_ll_exti.h"
#include "stm32f4xx_ll_spi.h"
#include "stm32f4xx_ll_bus.h"
#include "stm32f4xx_ll_system.h"
#include "stm32f4xx_ll_utils.h"
#include "stm32f4xx_ll_gpio.h"
#include "stm32f4xx_ll_pwr.h"
#include "stm32f4xx_it.h"
#elif  defined(STM32F103xB)
#include "stm32f1xx_hal.h"
#include "stm32f1xx_hal_rcc.h"
#include "stm32f1xx_ll_exti.h"
#include "stm32f1xx_ll_spi.h"
#include "stm32f1xx_ll_bus.h"
#include "stm32f1xx_ll_system.h"
#include "stm32f1xx_ll_utils.h"
#include "stm32f1xx_ll_gpio.h"
#include "stm32f1xx_ll_pwr.h"
#endif

#include "controls.h"


void controls_init() {
#if  defined(STM32F103xB)
    GPIO_InitTypeDef  GPIO_InitStruct;
    __HAL_RCC_GPIOA_CLK_ENABLE();
    
    GPIO_InitStruct.Mode  = GPIO_MODE_IT_FALLING;
    GPIO_InitStruct.Pull  = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    
    GPIO_InitStruct.Pin = BUTTON_BRITHNESS_PIN;
    HAL_GPIO_Init(BUTTON_BRITHNESS_GPIO, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = BUTTON_CONTRAST_PIN;
    HAL_GPIO_Init(BUTTON_CONTRAST_GPIO, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = BUTTON_TIME_PIN;
    HAL_GPIO_Init(BUTTON_TIME_GPIO, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = BUTTON_ENCODER_LEFT_PIN;
    HAL_GPIO_Init(BUTTON_ENCODER_LEFT_GPIO, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = BUTTON_ENCODER_RIGHT_PIN;
    HAL_GPIO_Init(BUTTON_ENCODER_RIGHT_GPIO, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = BUTTON_START_PIN;
    HAL_GPIO_Init(BUTTON_START_GPIO, &GPIO_InitStruct);

    GPIO_InitStruct.Mode  = GPIO_MODE_IT_RISING_FALLING;
    GPIO_InitStruct.Pin = BUTTON_SAFELIGHT_PIN;
    HAL_GPIO_Init(BUTTON_SAFELIGHT_GPIO, &GPIO_InitStruct);

    GPIO_InitStruct.Mode  = GPIO_MODE_IT_RISING_FALLING;
    GPIO_InitStruct.Pin = BUTTON_WHITE_PIN;
    HAL_GPIO_Init(BUTTON_WHITE_GPIO, &GPIO_InitStruct);

#endif
    HAL_NVIC_SetPriority(EXTI15_10_IRQn, 3, 0);
    HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);
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
}

uint16_t check_white_safe_led_switch() {
    if(GPIO_PIN_RESET == HAL_GPIO_ReadPin(BUTTON_SAFELIGHT_GPIO, BUTTON_SAFELIGHT_PIN)) {
        return BUTTON_SAFELIGHT_PIN;
    }
    if(GPIO_PIN_RESET == HAL_GPIO_ReadPin(BUTTON_WHITE_GPIO, BUTTON_WHITE_PIN)) {
        return BUTTON_WHITE_PIN;
    } else {
        return (BUTTON_SAFELIGHT_PIN | BUTTON_WHITE_PIN);
    }
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

    if(previous_pin != GPIO_Pin || (ticks - ticks_prev > GPIO_FILTER_DELAY_TICKS)) {   
        key_pressed_ISR(GPIO_Pin);
        previous_pin = GPIO_Pin ;
        ticks_prev = ticks;
    }   
}
