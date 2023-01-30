1. Function description
     1. TIM3 CH2 pin CH2 rising edge, CH1 falling edge to calculate the frequency
2. Use environment
     Software development environment: KEIL MDK-ARM V5.34.0.0
     Hardware environment: Developed based on N32G430C8L7_STB V1.0
3. Instructions for use
     System Configuration;
         1. Clock source:
                     HSE=8M,PLL=128M,APB1_CLK=32M, TIM3 CLK=64M
         2. Interrupt:
                     TIM3 CH2 rising edge CH1 falling edge interrupt on, steal priority 0, sub priority 1
         3. Port configuration:
                     PA7 is selected as TIM3 CH2 input
                     PA3 is selected as IO output
         4. TIM:
                     TIM3 CH2 rising edge, CH1 falling edge capture interrupt is turned on, the minimum capture frequency range is ((TIM3 CLK/2)/0xffff)Hz, the maximum frequency is ((TIM3 CLK)/2)Hz
     Instructions:
         1. Open the debug mode after compiling, connect PA3 and PA7, and add the variables TIM3Freq and gOnePulsEn to the watch window
         2. The default gOnePulsEn=0, manually give gOnePulsEn=1 each time, then you can see the frequency value calculated by TIM3Freq
4. Matters needing attention
     without