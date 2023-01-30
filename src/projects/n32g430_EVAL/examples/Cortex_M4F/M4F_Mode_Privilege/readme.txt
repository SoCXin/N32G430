1. Function description
	/* A brief description of the engineering function */
    This routine configudes and demonstrates how to modify thread mode privileged access and stack

2. Use environment

	Software development environment: KEIL MDK-ARM V5.34.0.0
    Hardware environment: Developed based on the evaluation board N32G430C8L7-STB V1.0

3. Instructions for use

	/* Describe the related module configuration method; For example: clock, I/O, etc. */
    SystemClock: 128 MHZ
    USART: TX-PA9, RX-PA10, baud rate 115200
	/* Describes the test steps and symptoms of Demo */
    1. Reset and run the downloaded program after compilation;
    2. Use the Compiler's Cortex Register window to view the stack and code access level (privileged/non-privileged) in thread mode.

4. Precautions
	None