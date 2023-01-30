1. Function description

     /* Briefly describe the project function */
     This example configures and demonstrates setting different system clocks and outputting them from PA8 with MCO


2. Use environment

     /* Software development environment: the name and version number of the software tool used in the current project */
     IDE tool: KEIL MDK-ARM V5.34.0.0
      
     /* Hardware environment: the development hardware platform corresponding to the project */
     Development board: N32G430C8L7-STB V1.0
     
3. Instructions for use

     /* Describe related module configuration methods; for example: clock, I/O, etc. */
     USART: TX - PA9, baud rate 115200
     GPIO: PA8 - multiplexed as MC0 clock output

     /* Describe the test steps and phenomena of the Demo */
     1. After compiling, download the program to reset and run;
     2. Configure the system clock as HSI, HSE, PLL respectively, use the serial port to print out the current SYSCLK, HCLK, PCLK1, PCLK2 
         and other information, and use the PA8 multiplexing pin to output the clock, and view it with an oscilloscope;


4. Matters needing attention