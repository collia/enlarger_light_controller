/* Includes ------------------------------------------------------------------*/

#include <stdbool.h>
#include <stdint.h>

/* Scheduler includes. */
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"

/* Library includes. */
#if defined(STM32F407xx)
#include "stm32f4xx_it.h"
#include "stm32f4xx_hal.h"
#include "stm32f4xx_hal_rcc.h"
#include "stm32f4xx_ll_exti.h"
#include "stm32f4xx_ll_spi.h"
#include "stm32f4xx_ll_bus.h"
#include "stm32f4xx_ll_system.h"
#include "stm32f4xx_ll_utils.h"
#include "stm32f4xx_ll_gpio.h"
#include "stm32f4xx_ll_pwr.h"
#elif  defined(STM32F103xB)
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
#endif


#include "max7219.h"


/*
  ---A---
  |     |
  F     B
  |     |
  ---G---
  |     |
  E     C
  |     |
  ---D---   DP

 */

#define DISPLAY_RAW(dp, a, b, c, d, e, f, g) (((dp) << 7) | \
                                              ((a) << 6) |  \
                                              ((b) << 5) |  \
                                              ((c) << 4) |  \
                                              ((d) << 3) |  \
                                              ((e) << 2) |  \
                                              ((f) << 1) |  \
                                              ((g) << 0) )

typedef struct  {
    uint8_t reg;
    uint8_t data;
} spi_reg_t;

static spi_reg_t max7219_initial_config[] = {
    {0x0C, 0x00},
    {0x01, 0x0f},
    {0x02, 0x0f},
    {0x03, 0x0f},
    {0x04, 0x0f},
    {0x05, 0x00},
    {0x06, 0x00},
    {0x07, 0x00},
    {0x08, 0x00},
    //{0x09, 0xFF}, /*decode all*/
    {0x09, 0x0F}, /*decode right 4 symbols*/
    {0x0a, 0x01}, /*Max intensity*/
    {0x0b, 0x07}, /*Display all*/
    {0x0C, 0x01},
    {0x0f, 0x01},
};

static spi_reg_t max7219_empty_mode_config[] = {
    {0x0f, 0x00},
    {0x01, 0x0a},
    {0x02, 0x0a},
    {0x03, 0x0a},
    {0x04, 0x0a},
    {0x05, 0x00},
    {0x06, 0x00},
    {0x07, 0x00},
    {0x08, 0x00}
};


static spi_reg_t max7219_time_mode_config[] = {
    {0x05, 0x00},
    {0x08, DISPLAY_RAW(0, 1, 0, 1, 1, 0, 1, 1)},
    {0x07, DISPLAY_RAW(0, 1, 0, 0, 1, 1, 1, 1)},
    {0x06, DISPLAY_RAW(0, 1, 0, 0, 1, 1, 1, 0)}
};

static spi_reg_t max7219_contrast_mode_config[] = {
    {0x05, 0x00},
    {0x08, DISPLAY_RAW(0, 1, 0, 0, 1, 1, 1, 0)},
    {0x07, DISPLAY_RAW(0, 1, 1, 1, 1, 1, 1, 0)},
    {0x06, DISPLAY_RAW(0, 0, 0, 0, 0, 0, 0, 0)}
};


static spi_reg_t max7219_brightness_mode_config[] = {
    {0x05, 0x00},
    {0x08, DISPLAY_RAW(0, 1, 1, 1, 1, 1, 1, 1)},
    {0x07, DISPLAY_RAW(0, 1, 1, 1, 0, 1, 1, 1)},
    {0x06, DISPLAY_RAW(0, 0, 0, 0, 0, 1, 1, 0)}
};

#if 0
static const uint8_t * spi1_tx_buffer;
static uint32_t spi1_tx_buffer_len = 0;
static uint32_t spi1_tx_buffer_transffered = 0;
static volatile bool     spi1_tx_buffer_transffer_finished = true;
#endif

static bool max7219_send_reg(const spi_reg_t *reg);
static void max7219_send_regs_array(const spi_reg_t *reg, uint32_t len);
static void Configure_SPI1(void);
static void Configure_MAX7219_SP(void);
static void Activate_SPI1(void);


/**
 * Module is initalize and use screen driver, connected to SPI1
 */

void max7219_init() {

    Configure_SPI1();
    Activate_SPI1();
    Configure_MAX7219_SP();
    max7219_send_regs_array(max7219_initial_config, sizeof(max7219_initial_config)/sizeof(spi_reg_t));
    //max7219_send_regs_array(max7219_initial_config, sizeof(max7219_initial_config)/sizeof(spi_reg_t));
}
#if 0
static bool max7219_send_reg(const spi_reg_t *reg) {
    if(spi1_tx_buffer_transffer_finished && reg) {
        spi1_tx_buffer = (uint8_t*)reg;
        spi1_tx_buffer_len = 2;
        spi1_tx_buffer_transffered = 0;
        spi1_tx_buffer_transffer_finished = false;

        //SPI1_Tx_Callback();
          /* Enable TXE   Interrupt */
        LL_SPI_EnableIT_TXE(SPI1);

        return true;
    } else {
        return false;
    }
}
#else
static bool max7219_send_reg(const spi_reg_t *reg) {
    if(reg) {
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_RESET);
        LL_SPI_TransmitData8(SPI1, reg->reg);
        while((LL_SPI_IsActiveFlag_TXE(SPI1) != 1));
        //while((LL_SPI_IsActiveFlag_BSY(SPI1) == 1));
        LL_SPI_TransmitData8(SPI1, reg->data);
        while((LL_SPI_IsActiveFlag_BSY(SPI1) == 1));
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_SET);

        return true;
    } else {
        return false;
    }
}

#endif

static void max7219_send_regs_array(const spi_reg_t *reg, uint32_t len) {
    for(int i = 0; i < len; i++) {
        while(!max7219_send_reg(&(reg[i]))) {
            //for(int j = 1000; j; i--);
        }
    }
}

/**
  * @brief  This function configures SPI1.
  * @note  This function is used to :
  *        -1- Enables GPIO clock and configures the SPI1 pins.
  *        -2- Configure NVIC for SPI1.
  *        -3- Configure SPI1 functional parameters.
  * @note   Peripheral configuration is minimal configuration from reset values.
  *         Thus, some useless LL unitary functions calls below are provided as
  *         commented examples - setting is default configuration from reset.
  * @param  None
  * @retval None
  */
#if defined(STM32F407xx)
static void Configure_SPI1(void) {
}
#elif  defined(STM32F103xB)
static void Configure_SPI1(void) {
    /* PA5 - sck */
    /* PA6 - MISO */
    /* PA7 - MOSI */

  /* (1) Enables GPIO clock and configures the SPI1 pins ********************/
  /* Enable the peripheral clock of GPIOA */
  LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_GPIOA);
  /* Remap SPI1 pins and disable JTAG */
//  LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_AFIO);
//  LL_GPIO_AF_Remap_SWJ_NOJTAG();
//  while((AFIO->MAPR & AFIO_MAPR_SWJ_CFG_JTAGDISABLE) != AFIO_MAPR_SWJ_CFG_JTAGDISABLE);
//  LL_GPIO_AF_EnableRemap_SPI1();
  /* Configure SCK Pin connected to pin 31 of CN10 connector */
  LL_GPIO_SetPinMode(GPIOA, LL_GPIO_PIN_5, LL_GPIO_MODE_ALTERNATE);
  LL_GPIO_SetPinSpeed(GPIOA, LL_GPIO_PIN_5, LL_GPIO_SPEED_FREQ_LOW);
  LL_GPIO_SetPinPull(GPIOA, LL_GPIO_PIN_5, LL_GPIO_PULL_DOWN);

  /* Configure MOSI Pin connected to pin 29 of CN10 connector */
  LL_GPIO_SetPinMode(GPIOA, LL_GPIO_PIN_7, LL_GPIO_MODE_ALTERNATE);
  LL_GPIO_SetPinSpeed(GPIOA, LL_GPIO_PIN_7, LL_GPIO_SPEED_FREQ_LOW);
  LL_GPIO_SetPinPull(GPIOA, LL_GPIO_PIN_7, LL_GPIO_PULL_DOWN);

  /* (2) Configure NVIC for SPI1 transfer complete/error interrupts **********/
  /* Set priority for SPI1_IRQn */
  //NVIC_SetPriority(SPI1_IRQn, 0);
  /* Enable SPI1_IRQn           */
  //NVIC_EnableIRQ(SPI1_IRQn);

  /* (3) Configure SPI1 functional parameters ********************************/
  /* Enable the peripheral clock of GPIOB */
  LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_SPI1);

  /* Configure SPI1 communication */
  LL_SPI_SetBaudRatePrescaler(SPI1, LL_SPI_BAUDRATEPRESCALER_DIV256);
  LL_SPI_SetTransferDirection(SPI1,LL_SPI_HALF_DUPLEX_TX);
  LL_SPI_SetClockPhase(SPI1, LL_SPI_PHASE_2EDGE);
  LL_SPI_SetClockPolarity(SPI1, LL_SPI_POLARITY_HIGH);
  /* Reset value is LL_SPI_MSB_FIRST */
  //LL_SPI_SetTransferBitOrder(SPI1, LL_SPI_MSB_FIRST);
  LL_SPI_SetDataWidth(SPI1, LL_SPI_DATAWIDTH_8BIT);
  LL_SPI_SetNSSMode(SPI1, LL_SPI_NSS_SOFT);
  LL_SPI_SetMode(SPI1, LL_SPI_MODE_MASTER);

  /* Configure SPI1 transfer interrupts */

  /* Enable SPI1 Error Interrupt */
  //LL_SPI_EnableIT_ERR(SPI1);
}
#endif

static void Configure_MAX7219_SP(void) {
    GPIO_InitTypeDef  GPIO_InitStruct;
    //CS - B0
    /* -1- Enable GPIO Clock (to be able to program the configuration registers) */
    __HAL_RCC_GPIOB_CLK_ENABLE();
    
    GPIO_InitStruct.Mode  = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull  = GPIO_PULLDOWN;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    
    GPIO_InitStruct.Pin = GPIO_PIN_0;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
}
/**
  * @brief  This function Activate SPI1
  * @param  None
  * @retval None
  */
static void Activate_SPI1(void)
{
  /* Enable SPI1 */
  LL_SPI_Enable(SPI1);
}


/**
  * @brief  Function called from SPI1 IRQ Handler when TXE flag is set
  *         Function is in charge  to transmit byte on SPI lines.
  * @param  None
  * @retval None
  */
void  SPI1_Tx_Callback(void)
{
#if 0
    /* Write character in Data register.
       TXE flag is cleared by reading data in DR register */
    if(spi1_tx_buffer_transffered == 0 && !spi1_tx_buffer_transffer_finished) {
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_RESET);
    }
    if(!spi1_tx_buffer_transffer_finished &&
       spi1_tx_buffer_len > spi1_tx_buffer_transffered) {
        LL_SPI_TransmitData8(SPI1, spi1_tx_buffer[spi1_tx_buffer_transffered++]);
    } else {
        while((LL_SPI_IsActiveFlag_TXE(SPI1) != 1));

                  /* Disable TXE   Interrupt */
        LL_SPI_DisableIT_TXE(SPI1);

        spi1_tx_buffer_transffer_finished = true;
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_SET);
    }
#endif
}




void screen_init(void) {
    max7219_init();
    screen_set_mode(SCREEN_EMPTY_MODE);
}
void screen_set_mode(screen_modes m) {
    switch(m) {
    case SCREEN_EMPTY_MODE:
        max7219_send_regs_array(max7219_empty_mode_config, sizeof(max7219_empty_mode_config)/sizeof(spi_reg_t));
        break;
    case SCREEN_TIME_MODE:
        max7219_send_regs_array(max7219_time_mode_config, sizeof(max7219_time_mode_config)/sizeof(spi_reg_t));
        break;
    case SCREEN_CONTRAST_MODE:
        max7219_send_regs_array(max7219_contrast_mode_config, sizeof(max7219_contrast_mode_config)/sizeof(spi_reg_t));
        break;
    case SCREEN_BRIGHNESS_MODE:
        max7219_send_regs_array(max7219_brightness_mode_config, sizeof(max7219_brightness_mode_config)/sizeof(spi_reg_t));
        break;
    default:
        break;
    }
}
void screen_set_value(uint32_t v) {
    spi_reg_t digits[4] = {
            {0x01, 0x0a},
            {0x02, 0x0a},
            {0x03, 0x0a},
            {0x04, 0x0a}};
    uint32_t pv = v;
    if(v <= 9999) {
        digits[0].data = pv % 10;
        pv /= 10;
        for (int i = 1; i < 4; i++) {
            if(pv == 0) {
                digits[i].data = 0xf;
            } else {
                digits[i].data = pv % 10;
                pv /= 10;
            }
        }
    }
    max7219_send_regs_array(digits, sizeof(digits)/sizeof(spi_reg_t));
}
