1. Function description
    1. SPI uses interrupt to send and receive data in single mode.
2. Use environment
    Software development environment: KEIL MDK-ARM V5.34.0.0
    Hardware environment: Developed based on the evaluation board N32G430C8L7-STB V1.0
3. Instructions for use
    /* Describe related module configuration methods; for example: clock, I/O, etc. */
    1、SystemClock：128MHz
    2、GPIO：SPI1: NSS--PA4  <--> SPI2: NSS--PB12 
             SPI1: SCK--PA5  <--> SPI2: SCK--PB13
             SPI1: MOSI--PA7 <--> SPI2: MISO--PB14
    /* Describe the test steps and phenomena of the Demo */
    1. After compiling, download the program to reset and run;
    2. SPI1 uses interrupt to send data, SPI2 uses interrupt to receive data, after the transmission is completed, 
       compare the sent and received data, use the serial port to print PASSS if it passes, and print ERR if it fails;
4. Matters needing attention
     The "single wire" data lines are MOSI pins on the master side and MISO pins on the slave side