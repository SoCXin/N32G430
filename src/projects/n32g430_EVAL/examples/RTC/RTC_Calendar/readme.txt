1. Function description
    1. Trigger the calendar print through the EXTI line.

2. Use environment
    Software development environment: KEIL MDK-ARM V5.34.0.0
    Hardware environment: Developed based on the evaluation board N32G430C8L7-STB V1.0​

3. Instructions for use

    System Configuration;
        1. RTC clock source: LSE 32.768kHz
        2. Interrupt line: EXTI_LINE7--PA7
        3. Serial port configuration:
                            - Serial port is USART1 (TX: PA9)
                            - Data bits: 8
                            - Stop bit: 1
                            - Parity: none
                            - Baud rate: 115200


    Instructions:
        After compiling and burning it to the evaluation board, PC13 is connected to the PA7 port. After power-on, the serial port will print the corresponding calendar time every 1 second.


4. Attention
    none