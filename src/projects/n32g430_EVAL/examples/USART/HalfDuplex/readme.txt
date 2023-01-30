1. Function description

    This test example demonstrates the identification of USARTy and USARTz through query detection to realize the half-duplex communication.
    First, USARTy sends TxBuffer1 data to USARTz, and USARTz receives data and stores it in RxBuffer2.
    Then, USARTz sends TxBuffer2 data to USARTy, and USARTy receives data and stores it in RxBuffer1.
    Finally, compare the two groups of received data and sent data, respectively, and store the comparison results in the TransferStatus1 variable and the TransferStatus2 variable.
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
    - Half-duplex mode enabled
    
    The USART pins are connected as follows:
    - USART1_Tx.PA9 <-------> USART2_Tx.PA6

    
    Test steps and phenomena:
    - Demo is compiled in KEIL environment and downloaded to MCU
    - After the reset operation, check the variables TransferStatus1 and TransferStatus2 in turn, among which,
      PASSED is the test passed, FAILED is the test abnormal

4. Attention
    none