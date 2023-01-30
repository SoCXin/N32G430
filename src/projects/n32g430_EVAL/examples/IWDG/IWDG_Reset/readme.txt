1. Function description
	1. IWDG reset function.


2. Use environment

	Software development environment: KEIL MDK-ARM V5.34.0.0
    Hardware environment: Developed based on the evaluation board N32G430C8L7-STB V1.0


3. Instructions for use
	
	System Configuration£º
		1. IWDG clock source: LSI/128
        2. Timeout value: 13s (3.2ms * 4095)
        3. Indicator light: LED1(PA1) LED2(PA7)

	Instructions:
	1. After compiling under KEIL, program it to the evaluation board. After power-on, the serial port prints and 
	   feeds the dog three times, and the indicator LED2 keeps flashing. It means that IWDG feeds the dog normally and the code runs normally.
	2. If the dog is not fed after three feedings, the IWDG resets.


4. Matters needing attention
      If you want to suspend IWDG during debugging, you need to turn on DBG_Peripheral_ON(DBG_IWDG_STOP);

