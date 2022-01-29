#ifndef __ws2812
#define __ws2812

#include <stdio.h>
#include <string.h>
#include "stm32f1xx_hal.h"

/*
 * This code was taken from 
 * http://fabioangeletti.altervista.org/blog/stm32-interface-ws2812b/?doing_wp_cron=1528043483.7364630699157714843750
 */

/* Definition of TIM instance */
#define TIMx                                TIM2
#define TIMx_CHANNEL                        TIM_CHANNEL_3
#define TIMx_DMA_CHANNEL                    TIM_DMA_ID_CC3
/* Definition for TIMx clock resources */
#define TIMx_CLK_ENABLE                     __HAL_RCC_TIM2_CLK_ENABLE
#define DMAx_CLK_ENABLE                     __HAL_RCC_DMA1_CLK_ENABLE

/* Definition for TIMx Pins */
#define TIMx_CHANNEL3_GPIO_CLK_ENABLE       __HAL_RCC_GPIOB_CLK_ENABLE
#define TIMx_GPIO_CHANNEL1_PORT             GPIOB
#define GPIO_PIN_CHANNEL1                   GPIO_PIN_10
#define TIMx_GPIO_TIM2_3_REMAP              __HAL_AFIO_REMAP_TIM2_ENABLE

/* Definition for TIMx's DMA */
#define TIMx_CC1_DMA_REQUEST                DMA_REQUEST_1
#define TIMx_CC1_DMA_INST                   DMA1_Channel1

/* Definition for DMAx's NVIC */
#define TIMx_DMA_IRQn                       DMA1_Channel1_IRQn
#define TIMx_DMA_IRQHandler                 DMA1_Channel1_IRQHandler

//WS2812
#define WS2812_FREQ                         (800000)            // it is fixed: WS2812 require 800kHz
#define TIMER_CLOCK_FREQ                    (16000000)      // can be modified - multiples of 0.8MHz are suggested
#define TIMER_PERIOD                        (TIMER_CLOCK_FREQ / WS2812_FREQ)
#define LED_NUMBER                          (10)                    // how many LEDs the MCU should control

#define RESET_SLOTS                         (50)

#define WS2812_0                            (TIMER_PERIOD / 3)              // WS2812's zero high time is long about one third of the period
#define WS2812_1                            (TIMER_PERIOD * 2 / 3)      // WS2812's one high time is long about two thirds of the period
#define WS2812_RESET                        (0)

void ws2812_init(void);
void ws2812_update(void);
bool fillLEDcolor(uint8_t RED, uint8_t GREEN, uint8_t BLUE);
void TIMx_DMA_IRQHandler(void);

#endif
