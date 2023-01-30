1. Function description
    1. SPI uses DMA to send and receive data
2. Use environment
    Software development environment: KEIL MDK-ARM V5.34.0.0
    Hardware environment: Developed based on the evaluation board N32G430C8L7-STB V1.0
3. Instructions for use
    /* Describe related module configuration methods; for example: clock, I/O, etc. */
    1. SystemClock: 128MHz
    2、GPIO：SPI1: NSS--PA4  <--> SPI2: NSS--PB12 
             SPI1: SCK--PA5  <--> SPI2: SCK--PB13
             SPI1: MISO--PA6 <--> SPI2: MISO--PB14
             SPI1: MOSI--PA7 <--> SPI2: MOSI--PB15  
    /* Describe the test steps and phenomena of the Demo */
    1. After compiling, download the program to reset and run;；
    2. SPI1 and SPI2 use DMA to send and receive data at the same time. 
       After the transmission is completed, compare the data sent and received. 
       If it passes, use the serial port to print PASSS, and if it fails, print ERR;
4. Matters needing attention
    If a "single wire" data line is used, it is the MOSI pin on the master side and the MISO pin on the slave side