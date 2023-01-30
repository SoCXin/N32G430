1. Function description
    This routine provides a DMA usage for transferring data between peripherals and RAM.
             
    Initialize CLOCK, GPIO, PERIPH, then enable DMA function of SPI, and then DMA

    First DMA_CH5 transfers data from Slave_Tx_Buffer to the TX data register of SPI2 device, and the data stream is sent from SPI2 TX
    To SPI1 RX, DMA_CH4 transfers data from the RX register of SPI1 to master_rx_buffer.

    At the same time, DMA_CH3 transfers data from Master_Tx_Buffer to the TX data register of SPI1 device,and the data stream is sent from SPI1 TX
    To SPI2 RX, DMA_CH2 transfers data from SPI2's RX register to Slave_rx_Buffer.
    Wait for the DMA transfer to complete,
    Compare data consistency between Slave_Rx_Buffer and Master_Tx_Buffer, output the comparison result to USART2(TX:PB4)
    Compare Master_Rx_Buffer and Slave_Tx_Buffer to USART2(TX:PB4)

2. Use environment
    Software development environment: KEIL MDK-ARM V5.34.0.0
    Hardware environment: Developed based on the evaluation board N32G430C8L7-STB V1.0

3. Instructions for use
    1. Clock source: HSE+PLL
    2. Master clock: 128MHz
    3. DMA channels: DMA_CH2, DMA_CH3, DMA_CH4, DMA_CH5
    4. SPI1 configuration:
        NSS   -->  PA4          50MHz, AF_PP
        SCK   -->  PA5          50MHz, AF_PP
        MISO  -->  PA6          50MHz, AF_PP
        MOSI  -->  PA7          50MHz, AF_PP
    	Full duplex
    	Main mode
    	8 bit transmission
    	Polarity: start at low/second edge
    	Piece of software to choose
   	 	Big end in front MSB

    5. SPI2 Configuration:
        NSS   -->  PB12         50MHz, AF_PP
        SCK   -->  PB13         50MHz, AF_PP
        MISO  -->  PB14         50MHz, AF_PP
        MOSI  -->  PB15         50MHz, AF_PP
    	Full duplex
    	From the pattern
    	8 bit transmission
    	Polarity: start at low/second edge
    	Piece of software to choose
    	Big end in front MSB

    6. USART1 configuration:
        TX  -->  PA9            50MHz, AF_PP
    	Baud rate: 115200
    	Data bit: 8 bits
    	Stop bit: 1bit
    	No check

    7. Test steps and phenomena
        A. Compile download code reset run
        B. View the printed information from the serial port and verify the result
        
4. Precautions
    None
