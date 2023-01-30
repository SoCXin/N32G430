1. Function description
	This routine provides a DMA usage for transferring data between peripherals and RAM.
             
	Initialize CLOCK, GPIO, PERIPH, then enable DMA for I2C, then enable DMA
    
	First, DMA_CH6 transfers I2C1_Tx_Buffer data to the data register of I2C1,
	And then I2C1 sends the data to I2C2
	Finally, DMA_CH5 transfers data from the I2C2 data register to I2C2_Rx_Buffer.
    
	Wait for the DMA transfer to complete,
	Compare the data consistency between I2C1_Tx_Buffer and I2C2_Rx_Buffer and output the comparison result to the serial port

2. Use environment
    Software development environment: KEIL MDK-ARM V5.34.0.0
    Hardware environment: Developed based on the evaluation board N32G430C8L7-STB V1.0

3. Instructions for use
	1.Clock source: HSE+PLL
	2.Master clock: 128MHz
	3.DMA channels: DMA_CH5, DMA_CH6
	4.I2C1 configuration:
		SCL   -->  PB8          50MHz, AF_OD
            	SDA   -->  PB9          50MHz, AF_OD
           	ADDR: 0x30(7bit)
            	CLOCK: 100K
    	5.I2C2 configuration:
            	SCL   -->  PB10          50MHz, AF_OD
            	SDA   -->  PB11          50MHz, AF_OD
            	ADDR: 0x30(7bit)
            	CLOCK: 100K    
	6.USART1 configuration:
            	TX  -->  PA9            50MHz, AF_PP
		Baud rate: 115200
		Data bit: 8 bits
		Stop bit: 1bit
		No check
            
	7.Test steps and phenomena
	A. Compile download code reset run
	B. View the printed information from the serial port and verify the result
4. Precautions
	The I2C bus must be equipped with an external pull-up resistor, which is recommended to be 2.2-4.7K
