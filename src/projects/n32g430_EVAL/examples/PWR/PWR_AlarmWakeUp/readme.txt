1. Function description
	1. Use the RTC alarm clock to wake up STOP2.
	2. Serial port printing enters the exit state

2. Use environment
    Software development environment: KEIL MDK-ARM V5.34.0.0
    Hardware environment: Developed based on the evaluation board N32G430C8L7-STB V1.0


3. Instructions for use
	System configuration;
	1. RTC clock source: LSI
	2. Low power mode: STOP2
	3. Serial port configuration
		- Serial port: USART1 (TX: PA9 RX: PA10) :
		- Data bit: 8
		- Stop bit: 1
		- Parity check: None
		- Baud rate: 115200

	Usage:
	After compiling in KEIL, burn to the evaluation board, power on, after a while, the serial port prints start low power, indicating that it enters the low power mode.
	If Exit low power is printed through the serial port, the alarm clock wakes up STOP2.

4. Precautions
	None
