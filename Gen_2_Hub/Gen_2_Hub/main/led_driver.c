#include "led_strip.h"
#include "driver/rmt_tx.h"
#include "esp_err.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#define LED_PIN 8     // GPIO connected to onboard RGB LED
#define LED_COUNT 1   // Only one RGB LED

static led_strip_handle_t strip;

void led_init(void)
{
    led_strip_config_t strip_config = {
        .strip_gpio_num = 8,
        .max_leds = 1,
        .led_model = LED_MODEL_WS2812,
        .flags.invert_out = false,
    };

    led_strip_rmt_config_t rmt_config = {
        .clk_src = RMT_CLK_SRC_DEFAULT,
        .resolution_hz = 10 * 1000 * 1000,
        .flags.with_dma = false,
    };

    esp_err_t ret = led_strip_new_rmt_device(&strip_config, &rmt_config, &strip);
    if (ret != ESP_OK) {
        printf("ERROR: Failed to initialize LED strip: %s\n", esp_err_to_name(ret));
        strip = NULL;
        return;
    }

    led_strip_clear(strip);
    printf("LED strip initialized successfully\n");
}

void led_set_color(uint8_t r, uint8_t g, uint8_t b)
{
    led_strip_set_pixel(strip, 0, r, g, b);
    led_strip_refresh(strip);
}

void led_off(void)
{
    led_set_color(0, 0, 0);
}

void flash_color(uint8_t r, uint8_t g, uint8_t b){
    led_set_color(r, g, b);
    vTaskDelay(pdMS_TO_TICKS(500));
    led_off();
}
