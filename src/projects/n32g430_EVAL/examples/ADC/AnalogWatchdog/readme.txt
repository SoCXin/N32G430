1. Function description
    1. ADC samples and converts the analog voltage of PA1 pin. If it exceeds the threshold
    range defined by the analog watchdog, it will jump into the interrupt program
2. Use environment
    Software development environment: KEIL MDK-ARM V5.34.0.0
    Hardware environment: Developed based on the evaluation board N32G430C8L7-STB V1.0
3. Instructions for use
    System configuration£º
       1. Clock source:
            HSE=8M,PLL=128M,AHB=128M,APB1=32M,APB2=64M,ADC CLK=128M/16,ADC 1M CLK=HSE/8 
       2. the ADC:
            ADC continuous conversion, software triggered, 12 bit data right aligned, conversion of PA1 pin 
            analog voltage data
       3. Port Configuration:
           PA1 is selected as the analog function ADC conversion channel 2
       4. Interruption:
           ADC simulates watchdog interrupt open, grab priority 0, sub priority 0
    Usage:
       1. After compiling, open the debugging mode and add the variable gCntAwdg to the Watch window 
           for observation.
       2. Change the PA0 pin voltage value. When the voltage value exceeds the range of 0x300 to 0xB00 
          defined by the analog watchdog, an interrupt will be entered and the variables will be accumulated.
4. Precautions
       When the system uses the HSE clock (HSI is also normally turned on), RCC_ConfigAdc1mClk(RCC_ADC1MCLK_SRC_HSE, 
       RCC_ADC1MCLK_DIV8)can be configured as HSE or HSI.
       When the system samples the HSI clock (generally, HSE is disabled), RCC_ConfigAdc1mClk(RCC_ADC1MCLK_SRC_HSI, 
       RCC_ADC1MCLK_DIV8) can only be set to HSI.