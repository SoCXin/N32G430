1. Function description

    This test example demonstrates basic communication between USARTy and USARTz using hardware flow control. USARTy and USARTz can be USART1 and USART2.
    First, USARTy uses CTS to send TxBuffer1 data, USARTz uses RTS to receive.
    The data is stored in RxBuffer2; then, USARTz uses CTS to send TxBuffer2 data, USARTy use RTS to receive data and store it in RxBuffer1.
    Then, compare the received data with the sent data, and store the comparison results in the variable TransferStatus1 and TransferStatus2.

2. Use environment
    Software development environment: KEIL MDK-ARM V5.34.0.0
    Hardware environment: Developed based on the evaluation board N32G430C8L7-STB V1.0



3. Instructions for use
    The system clock configuration is as follows:
    - Clock Source = HSI + PLL
    - System Clock = 128MHz
    
    USARTy is configured as follows:
    - Baud rate = 115200 baud
    - Word length = 8 data bits
    - 1 stop bit
    - checksum control disabled
    - CTS hardware flow control enabled
    - Transmitter enable
    
    USARTz is configured as follows:
    - Baud rate = 115200 baud
    - Word length = 8 data bits
    - 1 stop bit
    - checksum control disabled
    - RTS hardware flow control enabled
    - Receiver enable
    
    
    The USART pins are connected as follows:
    - USART1_Tx.PA9 <-------> USART2_Rx.PA3
    - USART1_Rx.PA10 <-------> USART2_Tx.PA2
    - USART1_CTS.PA11 <-------> USART2_RTS.PA1
    - USART1_RTS.PA12 <-------> USART2_CTS.PA0

    
    Test steps and phenomena:
    - Demo is compiled in KEIL environment and downloaded to MCU
    - Reset operation, check the variables TransferStatus1 and TransferStatus2 in turn, where,
      PASSED is the test passed, FAILED is the test abnormal

4. Attention
    none