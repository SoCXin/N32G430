1. Function description
	1. PVD configuration voltage produces the corresponding interrupt.

2. Use environment
    Software development environment: KEIL MDK-ARM V5.34.0.0
    Hardware environment: Developed based on the evaluation board N32G430C8L7-STB V1.0

3. Instructions for use
	System configuration:
	1. Clock source: HSE+PLL
	2. Clock frequency: 128MHz
	Usage:
	After compiling under KEIL, burn to the evaluation board, go into debug mode through the emulator, and set 	breakpoint in PVD_IRQHandler().
	Run at full speed. Then turn the voltage close to the PVD setting voltage, and the program will stop at the breakpoint.

4. Precautions
	The MSB bit can be extended. For details, see the instructions for using the MSB bit in the user manual