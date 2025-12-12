# 1602 LCD driver

This repository contains an ESP-IDF driver for LCD 1602 module with 4 bit data interface.This
module contains two rows of 16 characters each. It runs on a ESP32 processor and is build using the ESP-IDF build system

## Connecting the component

The driver uses 6 GPIO output pins of the ESP32.


|      | LCD PIN   | ESP32 Pin                  | Default GPIO |
|:----:|:----------|:--------------------------|:-----------:|
| 1  | GND       | GND                        |             |
| 2  | VCC       | 5V                         |             |
| 3  | VL        | 10k potmeter               |             |
| 4  | RS        | CONFIG_REGISTER_SELECT_PIN | 23          |
| 5  | RW        | GND                        |             |
| 6  | EN        | CONFIG_ENABLE_PIN          | 22          |
| 7  | D0        | N.C.                       |             |
| 8  | D1        | N.C.                       |             |
| 9  | D2        | N.C.                       |             |
| 10 | D3        | N.C.                       |             |
| 11 | D4        | CONFIG_DATA_0_PIN          | 19          |
| 12 | D5        | CONFIG_DATA_1_PIN          | 18          |
| 13 | D6        | CONFIG_DATA_2_PIN          | 17          |
| 14 | D7        | CONFIG_DATA_3_PIN          | 16          |
| 15 | BL+       | VCC                        |             |
| 16|  BL-       | GND                        |             |

These connections of the CONFIG_XX pins are made using **idf.py menuconfig** and choose the settings in the menu under **Component config --->App main**
The 10K potentiometer for contrast adjustment is connected between VCC and GND with the center to pin 3 - VL.
## LCD1602 library

The libary only support the LCD Module 1602 with a 4 bits interface.
It uses the ```esp_driver_gpio component```.

# Usage

## API
The API of the driver is located in the include directory ```include/1602_driver.h``` and has the following functions:

```C
- void lcdInit(void);
- lcd_err_t lcdSetText(char *text, int x, int y);
- lcd_err_t lcdSetInt(int val, int x, int y);
- lcd_err_t lcdClear(void);
- void lcdAssert(lcd_err_t lcd_error);
```
The driver does not use interrupt handlers and API funcions run on the calling task. The driver API functions do not block.

# Example

An example is proved in the directory examples/lcd1602-examples
To run the provided example:

```shell
cd examples/lcd1602-example
idf.py build
```
And flash it to the board:
``` shell
idf.py -p PORT flash monitor
```
The example code writes the string <<< LCD TEST >>> on the first row on the second row a counter from 0 incrementing every second till 10 is displayed
If the counter reaches 10 the counter resets to 0 and both rows are flipped. i.e. The string <<< LCD TEST >>> will be on on row 1 and the counter on row 0. This flipping repeats endless.

# License

This component is provided under MIT license, see [LICENSE](LICENSE.txt) file for details.

# Contributing

Please check for contribution guidelines.tbd
