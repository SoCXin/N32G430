1. Function description
    1¡¢COMP1 output PA11 is affected by INP PB10 and INM PA5
2. Use environment
    Software development environment: KEIL MDK-ARM V5.34.0.0
    Hardware environment: Developed based on the evaluation board N32G430C8L7-STB V1.0
3. Instructions for use
    System configuration£º
       1. Clock source:
            HSE=8M,PLL=128M,AHB=128M,APB1=32M,APB2=64M,COMP CLK=32M 
       2. Port Configuration:
            PB10 is selected as the analog function COMP INP
            PA5 is selected as the simulation function COMP INM
            PA11 is selected as emulation feature COMP OUT
            PB11 Select IO output
            PB12 select IO output
       3. COMP£º
            COMP1 inputs PB10, PA5, and outputs PA11
    Usage:
       1. After compiling, open the debugging mode, connect PB11 to PB10 and PB12 to PA5, 
          and observe the output waveform of PA11 using oscilloscope or logic analyzer.
       2. When the software outputs PB11 level greater than PB12, PA11 outputs high level; otherwise, PA11 outputs low level
4. Precautions
    No