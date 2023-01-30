1. Function description

    /* Briefly describe the engineering function */
    This example configures and demonstrates CAN sending and receiving CAN messages in loopback test mode


2. Use environment

    Software development environment: KEIL MDK-ARM V5.34.0.0
    Hardware environment: Developed based on the evaluation board N32G430C8L7-STB V1.0

3. Instructions for use
    
    /* Describe the configuration method of related modules; for example: clock, I/O, etc. */
    SystemClock: 128MHz
    CAN: RX-PB8, TX-PB9, baud rate 500K, loopback test mode

    /* Describe the test steps and phenomena of Demo */
    1. After compiling, download the program to reset and run;
    2. CAN sends a frame of message, and then sends another frame of message after the verification is passed. Through the CAN device, you can see that the message is sent in a loop.


4. Matters needing attention
