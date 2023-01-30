1. Function description

     This test example demonstrates the basic communication between USARTx and PC by querying the detection flag.
     Redirect the printf function to USARTx, and use the printf function to output messages to the terminal.
     USARTx can be USART1 or USART2.


2. Use environment
    Software development environment: KEIL MDK-ARM V5.34.0.0
    Hardware environment: Developed based on the evaluation board N32G430C8L7-STB V1.0



3. Instructions for use
     The system clock configuration is as follows:
     - Clock Source = HSI + PLL
     - System Clock = 128MHz
    
     USART is configured as follows:
     - Baud rate = 115200 baud
     - Word length = 8 data bits
     - 1 stop bit
     - checksum control disabled
     - Hardware flow control disabled (RTS and CTS signals)
     - Receiver and transmitter enable
    
     The USART pins are connected as follows:
     - USART1_Tx.PA9
     or
     - USART2_Tx.PA6
    
     Test steps and phenomena:
     - Demo is compiled in KEIL environment and downloaded to MCU
     - reset operation, view serial port printing information


4. Attention
    none