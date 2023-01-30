1. Function description
    1. I2S uses DMA to send and receive data
2. Use environment
    Software development environment: KEIL MDK-ARM V5.34.0.0
    Hardware environment: Developed based on the evaluation board N32G430C8L7-STB V1.0
3. Instructions for use
    /* Describe related module configuration methods; for example: clock, I/O, etc. */
    1.SystemClock：128MHz
    2.GPIO：I2S1: WS--PA4  <--> I2S2: WS--PB12 
             I2S1: SK--PA5  <--> I2S2: SK--PB13
             I2S1: MCK--PA6 <--> I2S2: MCK--PB14
             I2S1: SD--PA7  <--> I2S2: SD--PB15  
    /* Describe the test steps and phenomena of the Demo */
    1.After compiling, download the program to reset and run;
    2.I2S1 uses DMA to send data, I2S2 uses DMA to receive data, after the transmission is completed, compare the sent and received data,
      I2S1 uses DMA to receive data, I2S2 uses DMA to send data, after the transmission is completed, compare the sent and received data,
      If it passes, use the serial port to print PASSS, and if it fails, print ERR;
4. Matters needing attention
    none