1. Function description
	1. Enter and exit the SLEEP mode.

2. Use environment
    Software development environment: KEIL MDK-ARM V5.34.0.0
    Hardware environment: Developed based on the evaluation board N32G430C8L7-STB V1.0

3. Instructions for use
	System configuration:
	1. Clock source: HSE+PLL
	2. Clock frequency: 128MHz
	3. Wake up source: PA0
	4. Indicator light: PA1
	Usage:
	After compiling in KEIL, it is burned to the evaluation board. After powering on, the indicator light controlled by PA1 is off. Press the wake button PA0 every once in a while,
	The PA1 control indicator level will be flipped once.

4. Precautions
	When evaluating power consumption, take care to remove the LED to measure.