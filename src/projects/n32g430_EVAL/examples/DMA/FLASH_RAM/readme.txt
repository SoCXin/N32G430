1. Function description
	This routine provides a DMA MemtoMem mode usage for transferring data between FLASH and RAM.
    
	We define a static array SRC_Const_Buffer. The data is stored in a read-only memory in FLASH.
	Configure the DMA_CH1 channel for transferring data to the RAM area DST_Buffer,
	Enable DMA transfer completion interrupt, used to indicate that the transfer is complete,
    
	Wait for data transfer to complete and compare DST_Buffer with SRC_Const_Buffer.
	Output the comparison result to the serial port.
    
2. Use environment
    Software development environment: KEIL MDK-ARM V5.34.0.0
    Hardware environment: Developed based on the evaluation board N32G430C8L7-STB V1.0

3. Instructions for use
	1.Clock source: HSE+PLL
	2.Master clock: 128MHz
	3.DMA channel: DMA_CH1
    
	4.USART1 configuration:
	TX  -->  PA9            50MHz, AF_PP
	Baud rate: 115200
	Data bit: 8 bits
	Stop bit: 1bit
	No check
	5.Test steps and phenomena
	A. Compile download code reset run
	B. View the printed information from the serial port and verify the result
4. Precautions
	None
