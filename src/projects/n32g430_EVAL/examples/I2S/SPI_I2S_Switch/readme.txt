1. Function description
    1. SPI sends and receives data for CRC check
2. Use environment
    Software development environment: KEIL MDK-ARM V5.34.0.0
    Hardware environment: Developed based on the evaluation board N32G430C8L7-STB V1.0
3. Instructions for use
    /* Describe related module configuration methods; for example: clock, I/O, etc. */
    1、SystemClock：128MHz
    2、GPIO：SPI1: NSS--PA4  <--> SPI2: NSS--PB12 
             SPI1: SCK--PA5  <--> SPI2: SCK--PB13
             SPI1: MISO--PA6 <--> SPI2: MISO--PB14
             SPI1: MOSI--PA7 <--> SPI2: MOSI--PB15
    /* Describe the test steps and phenomena of the Demo */
    1. After compiling, download the program to reset and run;
    2. SPI1 and SPI2 send and receive data at the same time. After the transmission is completed, send the CRC data, 
       check the data and CRC value, check that the status of TransferStatus1 and TransferStatus2 is PASSED, and then check the CRC value;
4. Matters needing attention
    none