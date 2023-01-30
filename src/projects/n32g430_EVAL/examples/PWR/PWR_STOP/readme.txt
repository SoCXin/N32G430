1. Function description
	1. Enter and exit STOP mode.

2. Use environment
    Software development environment: KEIL MDK-ARM V5.34.0.0
    Hardware environment: Developed based on the evaluation board N32G430C8L7-STB V1.0

3. Instructions for use
	System configuration:
	1. Clock source: HSE+PLL
	2. Clock frequency: 128MHz
	3. Wake up source: PA0
               
	Usage:
	After compiling in KEIL, it was burned to the evaluation board, connected to the ammeter, and found that the current dropped from mA to uA level after powering on.
	By pressing the wake button (PA0), the current reverted to mA level and after a while dropped back to uA.

4. Precautions
	When evaluating power consumption, take care to remove the LED to measure