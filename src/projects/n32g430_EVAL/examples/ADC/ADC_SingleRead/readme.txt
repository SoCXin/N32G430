1. Function description
    1. ADC1 sampling converts the analog voltage of PA0 PA1 pin
    2. Use the software to trigger once and collect once
2. Use environment
    Software development environment: KEIL MDK-ARM V5.34.0.0
    Hardware environment: Developed based on the evaluation board N32G430C8L7-STB V1.0
3. Instructions for use
    System configuration£º
       1. Clock source:
              HSE=8M,PLL=128M,AHB=128M,APB1=32M,APB2=64M,ADC CLK=128M/16,ADC 1M CLK=HSE/8M 
       2. Port Configuration:
              PA0 is selected as the analog function ADC conversion channel 1
              PA1 is selected as the analog function ADC conversion channel 2
       3. the ADC:
             ADC software triggers conversion, right-aligned 12-bit data, and converts analog voltage data of PA0 PA1
    Usage:
       1. Open the debugging mode after compilation, and add the variable ADCConvertedValue to the Watch window for observation
       2. By changing the voltage of PA0 PA1 pin, it can be seen that the conversion result variable changes synchronously
4. Precautions
       When the system uses the HSE clock (HSI is also normally turned on), RCC_ConfigAdc1mClk(RCC_ADC1MCLK_SRC_HSE, 
       RCC_ADC1MCLK_DIV8)can be configured as HSE or HSI.
       When the system samples the HSI clock (generally, HSE is disabled), RCC_ConfigAdc1mClk(RCC_ADC1MCLK_SRC_HSI, 
       RCC_ADC1MCLK_DIV8) can only be set to HSI.