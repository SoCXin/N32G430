1. Function description

     This routine demonstrates communication with an external EEPROM via the I2C module.

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
             CLOCK: 400KHz
            
     3. USART1 configuration:
             TX --> PA9
             RX --> PA10
             Baud rate: 115200
    
     4. Test using EEPROM model:
             AT24C04 (capacity 4kb)
        

     5. Test steps and phenomena
         a, check the EEPROM connection
         b, compile and download the code to reset and run
         c, see the print information from the serial port, and verify the result

4. Matters needing attention
    none