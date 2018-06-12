This project is a DTMF decoder running on a NUCLEO-F072RB board.
Reads analog input from PA0 and prints decoded code on UART2 @115200, 8N1.
The software is easily portable to any other STM32 device since portable STM32 Cube libraries have been used.
The DTMF decoding is taken from an old [ARM Keil project](http://www.keil.com/download/docs/370.asp) based on STM32F103 and SPL libraries.

Pinout

|Signal        | STM32 IO | Nucleo connector |
|--------------|:--------:|:----------------:| 
| ADC input    |   PA0    |      CN7-28      | 
| 8kHz TIM dbg |   PC4    |      CN10-34     |
| DMA CPLT DBG |   PB13   |      CN10-30     |




Note: audio input sinal must be conditioned so to have DC offset
See, for example,
https://electronics.stackexchange.com/questions/14404/dc-biasing-audio-signal
https://electronics.stackexchange.com/questions/5315/how-to-read-an-audio-signal-using-atmega328
