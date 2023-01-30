1. Function description
    1. Trigger the alarm interrupt when match the alarm time.
    2. Alarm output

2. Use environment
    Software development environment: KEIL MDK-ARM V5.34.0.0
    Hardware environment: Developed based on the evaluation board N32G430C8L7-STB V1.0​

3. Instructions for use

    System Configuration;
        1. RTC clock source: LSE
        2. Alarm clock IO output: PC13
        3. Serial port configuration:
                - Serial port is USART1 (TX:PA9)
                - Data bits: 8
                - Stop bit: 1
                - Parity: none
                - Baud rate: 115200


    Instructions:
        After compiling, burn it to the evaluation board, power on, when the alarm time matches, serial port will print the time.
        At this time, alarm output high level on PC13 port.


4. Attention
    none