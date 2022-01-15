#include <stdint.h>

typedef enum screen_modes_t {
    SCREEN_EMPTY_MODE,
    SCREEN_TIME_MODE,
    SCREEN_CONTRAST_MODE,
    SCREEN_BRIGHNESS_MODE,
    SCREEN_MAX_MODE,
} screen_modes;


void screen_init(void);
void screen_set_mode(screen_modes m);
void screen_set_value(uint32_t v);
