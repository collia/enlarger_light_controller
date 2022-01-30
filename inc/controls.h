#include <stdbool.h>

#define BUTTON_BRITHNESS_GPIO GPIOA
#define BUTTON_BRITHNESS_PIN  GPIO_PIN_2

#define BUTTON_CONTRAST_GPIO  GPIOA
#define BUTTON_CONTRAST_PIN   GPIO_PIN_3

#define BUTTON_TIME_GPIO     GPIOA
#define BUTTON_TIME_PIN      GPIO_PIN_4

#define BUTTON_ENCODER_LEFT_GPIO     GPIOA
#define BUTTON_ENCODER_LEFT_PIN      GPIO_PIN_0

#define BUTTON_ENCODER_RIGHT_GPIO     GPIOA
#define BUTTON_ENCODER_RIGHT_PIN      GPIO_PIN_1

#define BUTTON_START_GPIO            GPIOA
#define BUTTON_START_PIN             GPIO_PIN_10

#define BUTTON_SAFELIGHT_GPIO        GPIOA
#define BUTTON_SAFELIGHT_PIN         GPIO_PIN_11

#define BUTTON_WHITE_GPIO            GPIOA
#define BUTTON_WHITE_PIN             GPIO_PIN_12

void controls_init(void);
uint16_t check_white_safe_led_switch();
