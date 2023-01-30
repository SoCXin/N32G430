Function Description
     1. After TIM3 CH1 CH2 CH3 CH4 reaches the CC value, it outputs ACTIVE level
2. Use environment
     Software development environment: KEIL MDK-ARM V5.34.0.0
     Hardware Environment: Developed based on N32G430C8L7_STB V1.0 EVB
3. Instructions for use
     System Configuration;
         1. Clock source:
                     HSE=8M,PLL=128M,AHB=128M,APB1=32M,TIM3 CLK=64M
         2. Port configuration:
                     PA6 is selected as TIM3 CH1 output
                     PA7 is selected as TIM3 CH2 output
                     PB0 is selected as TIM3 CH3 output
                     PB1 is selected as TIM3 CH4 output
         3. TIM:
     Instructions:
         1. Open the debug mode after compiling, and observe the waveforms of TIM2 CH1 CH2 CH3 CH4 with an oscilloscope or logic analyzer
         2. After the timer runs to CC1 CC2 CC3 CC4, the output of the corresponding channel becomes Active
4. Matters needing attention
     none