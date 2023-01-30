1. Function description

    This demo shows that the USART module LIN mode as the master node polling to send the master
     request frame (0x3C) and the slave reply frame (0x3D).


2. Use environment

    Software development environment: KEIL MDK-ARM V5.34.0.0
    Hardware environment: Developed based on the evaluation board N32G430C8L7-STB V1.0


3. Instructions for use

    The system clock configuration is as follows:
    -Clock source = HSI + PLL
    -System clock = 128MHz
    
    The USART configuration is as follows:
    -Baud rate = 9600baud
    -Word length = 8 data bits
    -1 stop bit
    -Parity control disabled
    -Hardware flow control disabled (RTS and CTS signals)
    -Receiver and transmitter enable
    -LIN mode enable

    Print pin connection is as follows:
    - USART1_ Tx.PA9
    - USART1_ Rx.PA10	
    
    - LIN mode master         LIN mode slave
    - USART2_Tx.PA6  <-->  USART2_Rx.PA7  
    - USART2_Rx.PA7  <-->  USART2_Tx.PA6  

    Test steps and phenomena:
    a, the jumper connects to the slave pin of the master
    b, one development board compiles and downloads LIN_Master code as the master and 
        the other development board compiles and downloads LIN_Slave code as the slave
    c, Reset the slave and then the master
    d, master polling sends request frame (0x3C) and reply frame (0x3D)
       Sending request frame (0x3C) : The master sends an 8-byte 0x0F to the slave and prints the related information
       Sending reply frame (0x3D) : After receiving a reply frame, the slave reply an 8-byte 0x01 message to the master, 
        and the master prints the related information

4. Attention
    none