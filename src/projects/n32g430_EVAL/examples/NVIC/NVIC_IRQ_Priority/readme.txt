1. Function description
	/* A brief description of the engineering function */
    This routine configures and demonstrates NVIC priority Settings

2. Use environment
	
    Software development environment: KEIL MDK-ARM V5.34.0.0
    Hardware environment: Developed based on the evaluation board N32G430C8L7-STB V1.0
        
3. Instructions for use    
	/* Describe the related module configuration method; For example: clock, I/O, etc. */
    SystemClock: 128 MHZ
    USART: TX-PA9, RX-PA10, baud rate 115200
    EXIT: PA0 is floating input mode, external interrupt line -exit_line0, external interrupt is enabled, 	and the priority is 0
    SysTick: Set the interrupt priority to 0
    
	/* Describes the test steps and symptoms of Demo */
    1. Reset and run the downloaded program after compilation;
    2. In normal cases, SysTick interrupt information is displayed. If both the external interrupt and 	SysTick interrupt are triggered at the same time, change the SysTick interrupt priority to 2 and print related 	information.

4. Precautions
	None