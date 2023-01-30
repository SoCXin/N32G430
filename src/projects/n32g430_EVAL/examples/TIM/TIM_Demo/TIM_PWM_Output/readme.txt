1. Function description
    1. TIM1 outputs 4 complementary waveforms
2. Use environment
    Software development environment: KEIL MDK-ARM V5.34.0.0
    Hardware environment: Developed based on N32G430C8L7_STB V1.0
3. Instructions for use
    System Configuration;
        1. Clock source:
                    HSE=8M,PLL=128M,AHB=128M,APB2_CLK=64M,TIM1 CLK=128M
        2. Port configuration:
                    PA8 is selected as TIM1 CH1 output
                    PA9 is selected as TIM1 CH2 output
                    PA10 is selected as TIM1 CH3 output
                    PB13 is selected as TIM1 CH1N output
                    PB14 is selected as TIM1 CH2N output
                    PB15 is selected as TIM1 CH3N output
                    PA11 is selected as TIM1 CH4 output
                    PB2 is selected as TIM1 CH4N output
                    PB12 is selected as TIM1 Breakin input
        3. TIM:
                    TIM1 8-way complementary with dead zone, IOM brake
    Instructions:
        1. Open the debug mode after compiling, and observe the waveform of TIM1 with an oscilloscope or logic analyzer
        2. Four complementary waveforms can be observed
4. Matters needing attention
        By default, the PA9 and PA10 jumper caps of the development board are connected to the virtual serial port of NSLINK. If PA9 and PA10 are not used as serial ports in the project, and are used for other purposes, the serial port jumper caps must be unplugged.