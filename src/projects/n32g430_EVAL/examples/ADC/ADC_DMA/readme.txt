1. Function description
    1. ADC samples and converts the analog voltage of PA0 pin
    2. ADC conversion results are read to variable ADCConvertedValue through DMA_CH1 channel
2. Use environment
    Software development environment: KEIL MDK-ARM V5.34.0.0
    Hardware environment: Developed based on the evaluation board N32G430C8L7-STB V1.0
3. Instructions for use
    System configuration£º
       1. Clock source:
              HSE=8M,PLL=128M,AHB=128M,APB1=32M,APB2=64M,ADC CLK=128M/16,ADC 1M CLK=HSE/8
       2. Port Configuration:
              PA0 is selected as the analog function ADC conversion channel 1
       3. the DMA:
              DMA_CH1 channel loopback mode carries a half-word ADC conversion result to the ADCConvertedValue variable
       4. the ADC:
              ADC continuous conversion, scan mode, software trigger, 12 bit data right aligned, conversion channel 1 is PA0 analog voltage data
    Usage:
       1. After compiling, open the debugging mode and add the variable gCntAwdg to the Watch window 
           for observation.
       2. By changing the voltage of the PC2 pin, you can see that the conversion result variable changes synchronously
4. Precautions
       When the system uses the HSE clock (HSI is also normally turned on), RCC_ConfigAdc1mClk(RCC_ADC1MCLK_SRC_HSE, 
       RCC_ADC1MCLK_DIV8)can be configured as HSE or HSI.
       When the system samples the HSI clock (generally, HSE is disabled), RCC_ConfigAdc1mClk(RCC_ADC1MCLK_SRC_HSI, 
       RCC_ADC1MCLK_DIV8) can only be set to HSI.