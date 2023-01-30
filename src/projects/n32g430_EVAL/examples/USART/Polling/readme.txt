1. Function description

    This test example demonstrates the basic communication between USARTy and USARTz by querying the detection flag.
    First, USARTy sends TxBuffer1 data to USARTz, and USARTz receives data and stores it in RxBuffer1.
    Compare the received data with the sent data, and store the comparison result into the TransferStatus1 variable.
    Then, USARTz sends TxBuffer2 data to USARTy, and USARTy receives data and stores it in RxBuffer2.
    Compare the received data with the sent data, and store the comparison result into the TransferStatus2 variable.
    USARTy and USARTz can be USART1 and USART2.


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
    - USART1_Tx.PA9 <-------> UART4_Rx.PB1
    - USART1_Rx.PA10 <-------> UART4_Tx.PB0

    
    Test steps and phenomena:
    - Demo is compiled in KEIL environment and downloaded to MCU
    - Reset operation, check the variable TransferStatus twice, where,
      PASSED is the test passed, FAILED is the test abnormal


4. Attention
    none