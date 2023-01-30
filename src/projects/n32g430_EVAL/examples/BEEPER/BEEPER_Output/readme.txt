1. Function description

    This example demonstrates the usage of the beeper module.
    
2. Use environment

    Software development environment: KEIL MDK-ARM V5.34.0.0
    Hardware environment: Developed based on the evaluation board N32G430C8L7-STB V1.0

3. Instructions for use
	
    1. Clock source: APB2(64MHz) + PSC(32) + Div(1024)
    2. Output port:
	   PA6		AF_PP
	   PA7		AF_PP

	3. Test steps and phenomena
	   a: Compile and download the code, enter Debug mode
	   b: View the waveform with an oscilloscope to verify the results
	
4. Matters needing attention
	1. The waveforms of PA6 and PA7 are complementary
    2. The value of the APB2 clock frequency selection register BEEPER_CTRL.PSC bit is determined according to the main frequency, so that the 
	   output waveform will not be distorted.