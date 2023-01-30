1. Function description
    1. The reference program for CAN sending and receiving.


2. Use environment

    Software development environment: KEIL MDK-ARM V5.34.0.0
    Hardware environment: Developed based on the evaluation board N32G430C8L7-STB V1.0


3. Instructions for use
    
    System Configuration;
    /* Describe the configuration method of related modules; for example: clock, I/O, etc. */
    SystemClock: 128MHz
    CAN: RX-PB8, TX-PB9, baud rate 500K, normal mode

    /* Describe the test steps and phenomena of Demo */
    1. After compiling, download the program to reset and run;
    2. When CAN receives a frame of message, send it to a frame of message
                     
                 
4. Matters needing attention
     without
