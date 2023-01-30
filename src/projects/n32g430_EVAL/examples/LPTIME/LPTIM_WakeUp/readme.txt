1. Function description
    1. LPTIM generates EXTI20 regularly, wakes up the CPU for IO flipping
2. Use environment
    Software development environment: KEIL MDK-ARM V5.34.0.0
    Hardware environment: Developed based on the evaluation board N32G430C8L7-STB V1.0
3. Instructions for use
    System Configuration;
         1. Clock source:
             HSE=8M,PLL=128M,AHB=128M,APB1=32M,APB2=64M,LPTIM CLK=LSI 40K
         2. Interrupt:
             LPTIM cycle triggers EXTI20 interrupt, wakes up CPU
         3. Port configuration:
             PB7 is selected as IO output
         4. LPTIM:
             LPTIM Continuous Timing Mode
    Instructions:
         1. Turn on the debug mode after compiling, and it can be observed that the PA1 pin has a cycle flip, and the CPU continues to enter the low-power mode
4. Matters needing attention
    none   