1. Function description

     This routine shows the use of I2C interrupts to send and receive data as a slave.

2. Use environment

    Software development environment:
         IDE tool: KEIL MDK-ARM V5.34.0.0
    
     Hardware environment:
         Development board: N32G430C8L7-STB V1.0


3. Instructions for use

     1. Main clock: 128MHz
     2. I2C1 configuration:
             SCL --> PB6
             SDA --> PB7
             CLOCK: 100KHz
            
     3. USART1 configuration:
             TX --> PA9
             RX --> PA10
             Baud rate: 115200
        

     4. Test steps and phenomena
         a, jumper to connect the host I2C1
         b, compile and download the code to reset and run
         c, see the print information from the serial port, and verify the result

4. Matters needing attention
     SCL and SDA must be pulled up