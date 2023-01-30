1. Function description
    1. ADC regular channel samples the analog voltage of pins PA3 and PA4, and injection channel samples the analog voltage of pins PA0 and PA1
    2. The result of rule conversion is read into variable ADC_RegularConvertedValueTab[10] array through DMA_CH1 channel
        End into the transformation results by transforming the interrupt read into variable ADC_InjectedConvertedValueTab [10] array
2. Use environment
    Software development environment: KEIL MDK-ARM V5.34.0.0
    Hardware environment: Developed based on the evaluation board N32G430C8L7-STB V1.0
3. Instructions for use
    System configuration£º
       1. Clock source:
              HSE=8M,PLL=128M,AHB=128M,APB1=32M,APB2=64M,ADC CLK=128M/16,ADC 1M CLK=HSE/8
       2. Interruption:
              ADC injection conversion results complete interrupt open, steal priority 0, sub priority 0
              Interrupt handling receiving injection conversion results to ADC_InjectedConvertedValueTab [10] array
       3. Port Configuration:
              PA3 is selected as the analog function ADC1 conversion channel
              PA4 is selected as the analog function ADC1 conversion channel
              PA0 is selected as the analog function ADC1 conversion channel
              PA1 is selected as the analog function ADC1 conversion channel
              PA11 is selected as an external EXTI event rising edge trigger
              PA15 is selected as an external EXTI event rising edge trigger
       4. DMA:
              DMA_CH1 channel loopback mode carries the 10 half-word ADC1 regular channel conversion results into the ADC_RegularConvertedValueTab[10] array
       5. ADC:
              ADC regular channel scan intermittent mode, EXTI11 trigger, 12 bit data right alignment, conversion channel PA3 and PA4 analog voltage data
              ADC injection channel scan mode, EXTI15 trigger, 12 bit data right alignment, conversion channel PA0 and PA1 analog voltage data
    Usage:
      1, compiled to open the debug mode, variable ADC_RegularConvertedValueTab [10], ADC_InjectedConvertedValueTab [10] added to the watch window
      2. Regular channel data sampling can be triggered by PA11 rising edge, and injection channel data sampling can be triggered by PA15 rising edge
4. Precautions
       When the system uses the HSE clock (HSI is also normally turned on), RCC_ConfigAdc1mClk(RCC_ADC1MCLK_SRC_HSE, 
       RCC_ADC1MCLK_DIV8)can be configured as HSE or HSI.
       When the system samples the HSI clock (generally, HSE is disabled), RCC_ConfigAdc1mClk(RCC_ADC1MCLK_SRC_HSI, 
       RCC_ADC1MCLK_DIV8) can only be set to HSI.