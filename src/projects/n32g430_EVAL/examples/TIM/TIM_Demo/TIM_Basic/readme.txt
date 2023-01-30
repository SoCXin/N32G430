1. Function description
     1. TIM6 uses update interrupt to generate timing flip IO
2. Use environment
     Software development environment: KEIL MDK-ARM V5.34.0.0
     Hardware Environment: Developed based on N32G430C8L7-STB V1.0
3. Instructions for use
     System Configuration;
         1. Clock source:
                    HSE=8M,PLL=128M,AHB=128M,APB1=32M,TIM6 CLK=64M
         2. Port configuration:
                    PB6 is selected as IO output
         3. TIM:
                    TIM6 enables periodic interrupts
         4. Interrupt:
                    TIM6 update interrupt on, steal priority 0, sub priority 1
     Instructions:
         1. Open the debug mode after compiling, and observe the waveform of PB6 with an oscilloscope or logic analyzer
         2. After the program runs, the cycle of TIM6 is interrupted and the level of PB6 is flipped4.
Matters needing attention
     without