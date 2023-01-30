1. Function description
    1. LPTIM output PWM signal
2. Use environment
    Software development environment: KEIL MDK-ARM V5.34.0.0
    Hardware environment: Developed based on the evaluation board N32G430C8L7-STB V1.0
3. Instructions for use
    System Configuration;
        1. Clock source:
           HSE=8M,PLL=128M,AHB=128M,APB1=32M,APB2=64M,LPTIM CLK=LSI 40K
        2. Port configuration:
           PB2 selected as LPTIM output
        3. LPTIM:
           LPTIM 4-frequency LSI, output PWM signal
    Instructions:
        1. Open the debug mode after compiling, and you can observe the PWM signal of the PB2 pin
        2. Connect the PB2 pin to the LED so that you can see the LED blink.
4. Matters needing attention
    none