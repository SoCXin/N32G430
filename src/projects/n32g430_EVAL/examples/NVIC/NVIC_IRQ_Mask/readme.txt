1. Function description
	/* A brief description of the engineering function */
    This routine configures and demonstrates the use of EXIT external interrupts and TIM timer interrupts

2. Use environment

	Software development environment: KEIL MDK-ARM V5.34.0.0
    Hardware environment: Developed based on the evaluation board N32G430C8L7-STB V1.0
        
3. Instructions for use    
	/* Describe the related module configuration method; For example: clock, I/O, etc. */
    SystemClock: 128 MHZ
    USART: TX-PA9, RX-PA10, baud rate 115200
    EXIT: PA0 is in floating input mode, and external interrupt line -exit_line0 is used to enable external interrupt
    TIM: Pre-dividing frequency coefficient - (SystemClock/1200-1), period - (1200-1), start timer interrupt
	/* Describes the test steps and symptoms of Demo */
    1. Reset and run the downloaded program after compilation;
    2. Check the serial port printing information. The timer interrupt information is printed every 1S. 		
       Press the button to stop printing, and press it again to continue printing, indicating that the program is running normally.

4. Precautions
	None
