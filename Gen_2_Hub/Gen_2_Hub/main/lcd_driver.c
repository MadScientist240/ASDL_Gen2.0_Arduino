#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "HD44780.h"
#include "main.h"

#define LCD_ADDR    0x27
#define SCL_PIN     6
#define SDA_PIN     7

static TaskHandle_t lcd_task_handle = NULL;

void lcd_setText(char *lineOne, char *lineTwo){
    LCD_home();
    LCD_writeStr(lineOne);
    LCD_setCursor(0, 1);
    LCD_writeStr(lineTwo);
}

void lcd_clear(){
    LCD_clearScreen();
}

void lcd_default(){
    char lineOneBuffer[16];
    snprintf(lineOneBuffer, sizeof(lineOneBuffer), "Num. Spokes: %d", device_count);

    char lineTwoBuffer[16];
    if(!iaq_network_state){
        snprintf(lineTwoBuffer, sizeof(lineTwoBuffer), "IAQ MGR Error");
    } else {
        snprintf(lineTwoBuffer, sizeof(lineTwoBuffer), "NWK State: Good");
    }

    lcd_setText(lineOneBuffer, lineTwoBuffer);
}

static void lcd_task(void *pvParameters){
    while(1){
        lcd_default();
        vTaskDelay(5000);
    }
}

void lcd_init(){
    LCD_init(LCD_ADDR, SDA_PIN, SCL_PIN, 2, 8);
    xTaskCreate(lcd_task, "LCD_MGR", 4096, NULL, 3, &lcd_task_handle);
}