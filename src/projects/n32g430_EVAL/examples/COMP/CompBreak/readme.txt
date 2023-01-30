1. Function description
    1. COMP1 output brake TIM1 TIM8 complementary signal, 
    after the COMP OUT becomes low restore TIM1 TIM8 waveform
2. Use environment
    Software development environment: KEIL MDK-ARM V5.34.0.0
    Hardware environment: Developed based on the evaluation board N32G430C8L7-STB V1.0
3. Instructions for use
    System configuration£º
       1. Clock source:
            HSE=8M,PLL=128M,AHB=128M,APB1=32M,APB2=64M,COMP CLK=32M,TIM1 CLK=128M,TIM8 CLK=128M
       2. Port Configuration:
            PB10 is selected as the analog function COMP INP
            PA5 is selected as the analog function COMP INM
            PA11 is selected as analog feature COMP OUT
            PB11 Select IO output
            PB12 select IO output
            PA0 select TIM1 CH1 output
            PA8 select TIM1 CH2 output
            PA10 Select TIM1 CH3 output
            PA1 select TIM1 CH1N output
            PB14 selected as TIM1 CH2N output
            PB15 is selected as TIM1 CH3N output
            PB4 select TIM1 breakin input
            PA2 select TIM8 CH1 output
            PA3 select TIM8 CH2 output
            PA4 select TIM8 CH3 output
            PA7 select TIM8 CH1N output
            PB3 is selected as TIM8 CH2N output
            PB1 is selected as TIM8 CH3N output
       3. TIM:
            TIM1 starts CH1 CH2 CH3 CH1N CH2N CH3N output,COMP as brake input
            TIM8 starts CH1 CH2 CH3 CH1N CH2N CH3N output,COMP as brake input
       4. COMP£º
            COMP1 output triggers TIM1 TIM8 brake, and resumes TIM1 TIM8 output when there is no output
    Usage:
            1. Open the debugging mode after compilation, connect PB11 to PB10 and PB12 to PA5, and observe 
            the output waveform of TIM1 and TIM8 with oscilloscope or logic analyzer
            2. When the software output PB11 level is greater than PB12, TIM waveform disappears; 
            otherwise, waveform output is normal
4. Precautions
    No