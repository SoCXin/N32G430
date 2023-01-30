1. Function description
	/* A brief description of the engineering function */
    This routine configures and demonstrates reading and writing directly to FLASH

2. Use environment
	
    Software development environment: KEIL MDK-ARM V5.34.0.0
    Hardware environment: Developed based on the evaluation board N32G430C8L7-STB V1.0
        
3. Instructions for use  
	/* Describe the related module configuration method; For example: clock, I/O, etc. */
    SystemClock: 128 MHZ
    USART: TX-PA9, RX-PA10, baud rate 115200
	/* Describes the test steps and symptoms of Demo */
    1. Reset and run the downloaded program after compilation;
    2. Check the information printed through the serial port. If the data written to the FLASH is the same as the data read from the FLASH, the test is complete.

4. Precautions
	None