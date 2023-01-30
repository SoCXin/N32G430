1. Function description
     1. Trigger the tamper interrupt by detecting the rising edge of PA0 IO.

2. Use environment
    Software development environment: KEIL MDK-ARM V5.34.0.0
    Hardware environment: Developed based on the evaluation board N32G430C8L7-STB V1.0​

3. Instructions for use

     System Configuration;
         1. RTC clock source: LSE  32.768k
         2. Tamper2 port: PA0
         3. Serial port configuration:
                             - Serial port is USART1 (TX: PA9)
                             - Data bits: 8
                             - Stop bit: 1
                             - Parity: none
                             - Baud rate: 115200


     Instructions:
     1. After compiling and burning it to the evaluation board, after power-on, press WAKEUP key(PA0), and the serial port outputs RTC Tamper Interrupt, indicating that a tamper interrupt has occurred.


4. Attention
     none