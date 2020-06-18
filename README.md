This project is a DTMF decoder running on a NUCLEO-F072RB board.
Reads analog input from `PA0` and prints decoded code on UART2 @115200, 8N1.
`OUT1` output signal is driven low when DTMF key `1` is detected and is driven high when key `2` is detected.
Same goes with `OUT2` signal when keys `4` and `5` are detected. This is useful for driving, for example, a relay for remote automation purposes. See for example [isolated relay board](https://www.amazon.it/AZDelivery-Modulo-Channel-Optoisolatore-Arduino/dp/B078Q326KT/ref=sr_1_3?dchild=1&keywords=2+relay+module&qid=1592477442&sr=8-3)
The software is easily portable to any other STM32 device as STM32 Cube libraries are used.
The DTMF decoding is taken from an old [ARM Keil project](http://www.keil.com/download/docs/370.asp) based on STM32F103 and SPL libraries.

Pinout

|Signal        | STM32 IO | Nucleo connector |
|--------------|:--------:|:----------------:| 
| ADC input    |   PA0    |      CN7-28      | 
| OUT1         |   PC2    |      CN7-35      |
| OUT2         |   PC3    |      CN7-37      |
| 8kHz TIM DBG |   PC4    |      CN10-34     |
| DMA CPLT DBG |   PB13   |      CN10-30     |


Note: as ADC only reads positive voltage values, audio input signal should be conditioned so to add DC offset to ADC input.
See, for example:

https://electronics.stackexchange.com/questions/14404/dc-biasing-audio-signal
https://electronics.stackexchange.com/questions/5315/how-to-read-an-audio-signal-using-atmega328

Can also be used without signal conditioning but some keys won't be recongnized.
Keys 1, 2, 4, and 5 have been successfully tested without applying DC offset.
