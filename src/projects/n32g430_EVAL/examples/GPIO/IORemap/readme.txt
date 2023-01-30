1. Function description

    1. This routine shows the high and low levels of the read port and controls the LED (D1¡¢D2) to flash
    2. Control IO port level flip

2. Use environment

	Software development environment: KEIL MDK-ARM V5.34.0.0
    Hardware environment: Developed based on the evaluation board N32G430C8L7-STB V1.0

3. Instructions for use
	
	/* Describe related module configuration methods; for example: clock, I/O, etc. */
	SystemClock£º128MHz
	GPIO: PA6 is selected as the read level port, PA1, PA7 control LED (D1, D2) to flash


	/* Describe the test steps and phenomena of the Demo */
    1. After compiling, download the program to reset and run;
    2. Check the level of the PA6 port:
       If it is high level, LED (D2) is always on;
       If it is low level, the LED (D1) is always on, and the. PA13/PA15/PB4/PA14/PB3 are connected to LED(D3) in turn, if LED3 port is high, LED(D3) will blink

4. Matters needing attention
	When it is detected that the PA6 port is at a high level, close JTAG, and when the PA6 port is at a low level, open the JTAG