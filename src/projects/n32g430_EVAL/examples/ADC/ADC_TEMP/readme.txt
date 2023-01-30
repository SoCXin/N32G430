1. Function description
    1. ADC samples and converts the analog voltage of the internal temperature sensor to the temperature value
2. Use environment
    Software development environment: KEIL MDK-ARM V5.34.0.0
    Hardware environment: Developed based on the evaluation board N32G430C8L7-STB V1.0
3. Instructions for use
    System configuration£º
       1. Clock source:
            HSE=8M,PLL=128M,AHB=128M,APB1=32M,APB2=64M,ADC CLK=128M/16,ADC 1M CLK=HSE/8 
        2¡¢DMA£º
            DMA_CH1 channel loopback mode carries a half-word ADC conversion result to the ADCConvertedValue variable
        3¡¢ADC£º
            ADC continuous conversion, software triggered, 12 bit data right aligned, conversion channel 17 is the analog
            voltage data of the internal temperature sensor
        4. Port Configuration:
            PA9 selects the TX pin of USART1
            PA10 selects the RX pin for USART1
        5¡¢USART£º
           USART1 115200 Baud rate, 8 data bits, 1 Stop bit, no parity bit, no hardware flow control, send and receive enabled
        6¡¢Functions:
            TempValue = TempCal(ADCConvertedValue)£¬The temperature ADC function converts the raw format data into degrees
    Usage:
        1¡¢compiled to open the debug mode, variable ADCConvertedValue, TempValue added to the watch window to observe
        2¡¢Connect the serial port tool to the PA9 pin and open the serial port receiver tool
        3¡¢Running at full speed, it can be seen that the value of the temperature variable is close to 25 degrees at room temperature, 
           and the serial port tool displays the real-time temperature value in the chip
4¡¢Precautions
       When the system uses the HSE clock (HSI is also normally turned on), RCC_ConfigAdc1mClk(RCC_ADC1MCLK_SRC_HSE, 
       RCC_ADC1MCLK_DIV8)can be configured as HSE or HSI.
       When the system samples the HSI clock (generally, HSE is disabled), RCC_ConfigAdc1mClk(RCC_ADC1MCLK_SRC_HSI, 
       RCC_ADC1MCLK_DIV8) can only be set to HSI.