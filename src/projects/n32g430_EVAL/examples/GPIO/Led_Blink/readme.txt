1. Function description

	1. This example shows the IO control LED blinking


2. Use environment

	Software development environment: KEIL MDK-ARM V5.34.0.0
    Hardware environment: Developed based on the evaluation board N32G430C8L7-STB V1.0


3. Instructions for use
	
	/* Describe related module configuration methods; for example: clock, I/O, etc. */
        SystemClock£º128MHz
        GPIO: PA1, PA7, PA9 control LED (D1, D2, D3) to blink

	/* Describe the test steps and phenomena of the Demo */
        1. After compiling, download the program to reset and run;£»
        2. LED (D1, D2, D3) flashes;

4. Matters needing attention
	none