1. Function description
    1. TIM1 CH2 gates TIM1's CH1, TIM2's CH1 and TIM3's CH1
2. Use environment
    Software development environment: KEIL MDK-ARM V5.34.0.0
    Hardware environment: Developed based on N32G430C8L7_STB V1.0
3. Instructions for use
    System Configuration;
        1. Clock source:
                    HSE=8M,PLL=128,AHB=128M,APB1=32M,APB2=32M,TIM1 CLK=64M,TIM3 CLK=64M
        2. Port configuration:
                    PA8 is selected as TIM1 CH1 output
                    PA9 is selected as TIM1 CH2 input
                    PA0 is selected as TIM2 CH1 output
                    PA6 is selected as TIM3 CH1 output
        3. TIM:
                    TIM1 CH2 gate CH1, gate TIM2, TIM3
    Instructions:
        1. After compiling, open the debug mode, and use an oscilloscope or logic analyzer to observe the waveforms of TIM1 CH1, TIM2, and TIM3 CH1
        2. TIM1 CH2 high level timer starts counting, low level stops
4. Matters needing attention
        By default, the PA9 and PA10 jumper caps of the development board are connected to the virtual serial port of NSLINK. If PA9 and PA10 are not used as serial ports in the project, and are used for other purposes, the serial port jumper caps must be unplugged.