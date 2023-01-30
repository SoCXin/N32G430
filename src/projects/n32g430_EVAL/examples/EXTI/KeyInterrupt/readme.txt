1. Function description
   This routine shows an external trigger interrupt to control the LED flashing

2. Use environment
    Software development environment: KEIL MDK-ARM V5.34.0.0
    Hardware environment: Developed based on the evaluation board N32G430C8L7-STB V1.0

3. Instructions for use

    /* Describe related module configuration methods; for example: clock, I/O, etc. */
    SystemClock: 128MHz
    GPIO: PA5 is selected as external interrupt entry, PB7 controls LED(D3) to blink

    /* Describe the test steps and phenomena of the Demo */
    1. After compiling, download the program to reset and run;
    2. Press and release the KEY2 button, the LED flashes;


4. Matters needing attention
    none