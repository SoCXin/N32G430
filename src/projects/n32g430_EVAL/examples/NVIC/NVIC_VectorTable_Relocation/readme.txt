1. Function description
	/* A brief description of the engineering function */
    This routine configures and demonstrates NVIC interrupt directional table relocation

2. Use environment
	
    Software development environment: KEIL MDK-ARM V5.34.0.0
    Hardware environment: Developed based on the evaluation board N32G430C8L7-STB V1.0
        
3. Instructions for use   
	/* Describe the related module configuration method; For example: clock, I/O, etc. */
    SystemClock: 128 MHZ
    USART: TX-PA9, RX-PA10, baud rate 115200
    EXIT: PA0 is floating input mode, external interrupt line -exit_line0, external interrupt is enabled, and the priority is 0
	/* Describes the test steps and symptoms of Demo */
    1. Reset and run the downloaded program after compilation;
    2. At the beginning, the directional table is located in FLASH. When the button is pressed, the backward table is repositioned to SRAM and relevant information is printed, and the program runs normally;

4. Precautions
	None