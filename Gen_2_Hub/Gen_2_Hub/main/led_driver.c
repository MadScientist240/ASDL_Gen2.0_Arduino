#include "led_strip.h"
#include "driver/rmt_tx.h"
#include "esp_err.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "main.h"

#define LED_PIN 8     // GPIO connected to onboard RGB LED
#define LED_COUNT 1   // Only one RGB LED

static led_strip_handle_t strip;
bool wave_state;
static TaskHandle_t led_task_handle = NULL;

void led_set_color(uint8_t r, uint8_t g, uint8_t b)
{
    led_strip_set_pixel(strip, 0, r, g, b);
    led_strip_refresh(strip);
}

void led_off(void)
{
    wave_state = false;
    led_set_color(0, 0, 0);
}

void led_flash_color(uint8_t r, uint8_t g, uint8_t b){
    wave_state = false;
    vTaskSuspend(led_task_handle);

    for(int i = 0; i < 4; i++){
        led_set_color(r, g, b);
        vTaskDelay(pdMS_TO_TICKS(300));
        led_off();
        vTaskDelay(pdMS_TO_TICKS(300));
    }
    wave_state = true;
    vTaskResume(led_task_handle);
}

void led_wave(uint8_t r, uint8_t g, uint8_t b, uint32_t duration_ms){
    wave_state = true;
    const int steps = 50;
    const int delay_ms = duration_ms / (steps * 2);
    
    // Fade in
    for (int i = 0; i <= steps; i++) {
        float brightness = (float)i / steps;
        led_set_color(r * brightness, g * brightness, b * brightness);
        vTaskDelay(pdMS_TO_TICKS(delay_ms));
    }
    
    // Fade out
    for (int i = steps; i >= 0; i--) {
        float brightness = (float)i / steps;
        led_set_color(r * brightness, g * brightness, b * brightness);
        vTaskDelay(pdMS_TO_TICKS(delay_ms));
    }
}

static void led_task(void *pvParameters){
    while(1){
        if(wave_state){
            iaq_network_state ? led_wave(0, 255, 255, 4000) : led_wave(255, 0, 0, 6000);
        } else {
            vTaskDelay(pdMS_TO_TICKS(50));
        }
    }
}

void led_init(){
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
        strip = NULL;
    }

    led_strip_clear(strip);
    wave_state = true;

    xTaskCreate(led_task, "LED_MGR", 2048, NULL, 3, &led_task_handle);
}