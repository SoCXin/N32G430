1. Function description
    This test example demonstrates how to use the USART multiprocessor mode. USARTy and USARTz can be USART1 and USART2.
    First, set the addresses of USARTy and USARTz to 0x1 and 0x2, respectively. USARTy continuously gives USARTz send the character
    0x33. USARTz receives 0x33 and flips the pin of LED1.
    Once a rising edge is detected on the KEY1_INT_EXTI_LINE line, an EXTI0 interrupt will be generated.
    In the EXTI0_IRQHandler interrupt handler (the ControlFlag = 0), USARTz goes silent mode, in silent mode, the LED pin stops toggling.
    toggling until the KEY1_INT_EXTI_LINE line detects rising edge (the ControlFlag = 1). In the EXTI0_IRQHandler interrupt handler, 
    USARTysend address 0x102 to wake up USARTz. The LED pin restarts toggling.


2. Use environment
    Software development environment: KEIL MDK-ARM V5.34.0.0
    Hardware environment: Developed based on the evaluation board N32G430C8L7-STB V1.0



3. Instructions for use
    The system clock configuration is as follows:
    - Clock Source = HSI + PLL
    - System Clock = 128MHz
    
    USARTy is configured as follows:
    - Baud rate = 115200 baud
    - Word length = 9 data bits
    - 1 stop bit
    - checksum control disabled
    - Hardware flow control disabled (RTS and CTS signals)
    - Receiver and transmitter enable
    
    
    The USART pins are connected as follows:
    - USART1_Tx.PA9 <-------> UART4_Rx.PB1
    - USART1_Rx.PA10 <-------> UART4_Tx.PB0
    
    KEY1_INT_EXTI_LINE.PA6 <-------> KEY3
    
    LED1 <-------> PA1

    
    Test steps and phenomena:
    - Demo is compiled in KEIL environment and downloaded to MCU
    - Reset operation and observe whether LED1 are blinking
    - Press the button KEY and observe whether LED1 stop flashing
    - Press the button KEY again and observe whether LED1 resume to flash


4. Attention
    none