1. Function description
     1. The rising edge of TIM3 CH2 triggers CH1 to output a single pulse
2. Use environment
     Software development environment: KEIL MDK-ARM V5.34.0.0
     Hardware Environment: Developed based on N32G430C8L7-STB V1.0
3. Instructions for use
     System Configuration;
         1. Clock source:
                     HSE=8M,PLL=128M,AHB=128M,APB1=32M,TIM3 CLK=64M
         2. Port configuration:
                     PA6 is selected as CH1 output of TIM3
                     PA7 is selected as the CH2 input of TIM3
                     PA3 is selected as IO output
         3. TIM:
                     TIM3 configures CH2 rising edge to trigger CH1 to output a single pulse
     Instructions:
         1. Open the debug mode after compiling, connect PA3 to PA7, and observe the waveform of CH1 of TIM3 with an oscilloscope or logic analyzer
         2. Add the variable gSendTrigEn to the watch window, the default gSendTrigEn=0, every time you manually modify gSendTrigEn=1, you will see a single pulse output on the TIM3 CH1 port
4. Matters needing attention
     without