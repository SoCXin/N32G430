1. Function description
	/* A brief description of the engineering function */
    This routine configures and demonstrates MCU running tests

2. Use environment
    Software development environment: KEIL MDK-ARM V5.34.0.0
    Hardware environment: Developed based on the evaluation board N32G430C8L7-STB V1.0
        
3. Instructions for use   
	/* Describe the related module configuration method; For example: clock, I/O, etc. */
    USART: TX-PA9, TX-PA10, baud rate 115200
    SysTick: clock tick is set to 1MS
	/* Describes the test steps and symptoms of Demo */
    1. Reset and run the downloaded program after compilation;
    2. Print the running result through the serial port;

4. Precautions
	None

5. Note
	The coremask running time in the demo takes about 20s. You can change the running time by changing the value of the ITERATIONS macro. The running time should not be less than 10s.