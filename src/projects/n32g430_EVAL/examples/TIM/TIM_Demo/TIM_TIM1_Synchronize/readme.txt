1. Function description
     1. TIM3 TIM4 counts under TIM1 cycle
2. Use environment
     Software development environment: KEIL MDK-ARM V5.34.0.0
     Hardware environment: Developed based on N32G430C8L7_STB V1.0
3. Instructions for use
     System Configuration;
         1. Clock source:
                     HSE=8M,PLL=128M,AHB=128M,APB1=32M,APB2=32M,TIM1 CLK=64M,TIM3 CLK=64M,TIM4 CLK=64M
         2. Port configuration:
                     PA6 is selected as CH1 output of TIM3
                     PB6 is selected as CH1 output of TIM4
                     PA8 is selected as CH1 output of TIM1
         3. TIM:
                     TIM1 CH1 cycle triggers TIM3 TIM4 gate
     Instructions:
         1. Open the debug mode after compiling, and observe the waveforms of TIM1 CH1, TIM3 CH1, TIM4 CH1 with an oscilloscope or logic analyzer
         2. After the program runs, TIM3 15 times the period TIM1, TIM4 10 times the period TIM1
4. Matters needing attention
     without