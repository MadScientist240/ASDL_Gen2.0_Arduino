/**
 * @file lcd1602_example.c
 * @author Aad van Gerwen
 * @brief Example code for LCD 1602 driver
 * @version 0.2
 * @date 2025-03-17
 *
 * @copyright Copyright (c) 2025
 *
 */
#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "1602_driver.h"

esp_err_t return_value = ESP_OK;

void lcd_task(void *pvParameters)
{
    lcd_err_t ret = LCD_OK;

    // Initialize LCD object
    lcdInit();

    // Clear previous data on LCD 
    ret = lcdClear();
    lcdAssert(ret);

    // Custom text
    char buffer[16] = "<<< LCD TEST >>>";
     
    // Set text
    ret = lcdSetText(buffer, 0, 0);
    lcdAssert(ret);
  
    // create count variable
    int count = 0;
    int row = 1;

    while (1)
    {
        ret = lcdSetInt(count, 0, row);
        lcdAssert(ret);
         count++;     
        //1 second delay
        vTaskDelay(1000 / portTICK_PERIOD_MS);
        if (count == 10) {
            ret = lcdClear();
            lcdAssert(ret);
            count = 0;
            // Custom text
            char buffer[16] = "<<< LCD TEST >>>";
            ret = lcdSetText(buffer, 0, row);
            lcdAssert(ret);
            if (row == 1) {
                row = 0;
            } 
            else {
                row = 1;
            }  
        }      
    }
}

void app_main(void)
{
    // Create LCD task
    xTaskCreate(lcd_task, "LCD task", 2048, NULL, 4, NULL);
}
