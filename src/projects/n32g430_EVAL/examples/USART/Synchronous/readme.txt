1. Function description

    This test example demonstrates basic communication between USARTy (synchronous mode) and SPIy by querying the detection flag.
    USARTy and SPIy can be USART1 and SPI1, USART3 and SPI1, or USART2 and SPI2.
    First, through the TXC detection flag, USARTy sends TxBuffer1 data to SPIy, and SPIy receives data,then query the RNE detection flag, 
    and store the received data in RxBuffer1.Then, SPIy sends TxBuffer2 data to USARTy by querying the TE detection flag. USARTy uses
    the RXDNE detection flag receives data and stores it in RxBuffer2.
    Finally, compare the two groups of receiving and sending data respectively, and store the comparison results in the TransferStatus1 variable
    and the TransferStatus2 variable.

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
    - Clock enable
    - Clock polarity: keep low when not sending out
    - Clock Phase: The first data is sampled on the second clock edge
    - Last bit clock pulse: The clock pulse of the last bit of data is output from CK
    
    The SPI configuration is as follows:
    - Direction = "Two-Line Bidirectional" mode
    - mode = slave mode
    - data size = 8-bit data frame
    - CPOL = when idle, the clock remains low
    - CPHA = data sampling starts on second clock edge
    - NSS = Enable Software Slave Device Management
    - 1st bit = 1st bit is LSB
    
    
    The USART pins are connected as follows:
    - USART1_Tx.PA9 <-------> SPI1_MOSI.PA7
    - USART1_Rx.PA10 <-------> SPI1_MISO.PA6
    - USART1_Clk.PA8 <-------> SPI1_SCK.PA5
    
    Test steps and phenomena:
    - Demo is compiled in KEIL environment and downloaded to MCU
    - Reset operation, check the variables TransferStatus1 and TransferStatus2 in turn, where,
      PASSED is the test passed, FAILED is the test abnormal


4. Attention
    none