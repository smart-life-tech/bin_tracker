/*
 * PIC16F15324 VL53L1X Time of Flight Sensor Example
 * 
 * This file demonstrates how to use the VL53L1X (TOF400C) laser distance sensor
 * to control an LED based on proximity detection
 * 
 * Sensor: VL53L1X ToF Sensor
 * Communication: I2C (SCL on RC3, SDA on RC4)
 * Oscillator: 8 MHz Internal
 * I2C Baud Rate: 100 kHz
 */

// Oscillator configuration
#pragma config FEXTOSC = OFF    
#pragma config RSTOSC = LFINTOSCPWR_1MHZIDLED
#pragma config MCLRE = ON       
#pragma config BOREN = ON       
#pragma config WDTE = OFF       
#pragma config IESO = OFF
#pragma config PWRTE = ON

// VL53L1X I2C Address (7-bit: 0x52)
// Per datasheet: 0x52 for write, 0x53 for read (LSB determines R/W)
#define VL53L1X_ADDRESS         0x52
#define VL53L1X_ADDRESS_WR      0x52  // Write address (LSB = 0)
#define VL53L1X_ADDRESS_RD      0x53  // Read address (LSB = 1)

// VL53L1X Register Addresses (16-bit)
#define VL53L1X_WHO_AM_I         0x010F
#define VL53L1X_WHO_AM_I_VALUE   0xEA

#define VL53L1X_RESULT_RANGE     0x0089
#define VL53L1X_RESULT_STATUS    0x0089
#define VL53L1X_GPIO1_MODE       0x0084
#define VL53L1X_SYSRANGE_START   0x0018
#define VL53L1X_RESULT__RANGE_STATUS  0x0089

// Distance threshold (in mm) - turn LED on if object closer than this
#define DISTANCE_THRESHOLD       300     // 30cm

// LED pin definitions
#define LED_PIN RA4
#define LED_TRIS TRISAbits.TRISA4

// Function prototypes
void I2C_Init(void);
void GPIO_Init(void);
void Delay_ms(unsigned int ms);
void I2C_Start(void);
void I2C_Stop(void);
void I2C_Send_Byte(unsigned char data);
unsigned char I2C_Receive_Byte(unsigned char ack);
unsigned char I2C_Write_Register_16bit(unsigned short reg, unsigned char value);
unsigned char I2C_Read_Register_16bit(unsigned short reg, unsigned char *value);
unsigned char I2C_Read_Sequential(unsigned short start_reg, unsigned char *data, unsigned char length);
unsigned int VL53L1X_Read_Distance(void);
unsigned char VL53L1X_Init(void);
unsigned char VL53L1X_Start_Ranging(void);
unsigned char VL53L1X_Check_Sensor(void);

void main(void)
{
    GPIO_Init();
    I2C_Init();
    Delay_ms(100);  // Wait for sensor startup
    
    // Initialize VL53L1X sensor
    if(!VL53L1X_Init())
    {
        // Initialization failed - blink LED 3 times
        unsigned char i;
        for(i = 0; i < 3; i++)
        {
            LED_PIN = 1;
            Delay_ms(200);
            LED_PIN = 0;
            Delay_ms(200);
        }
        while(1);  // Hang
    }
    
    // Start continuous ranging
    VL53L1X_Start_Ranging();
    Delay_ms(100);
    
    while(1)
    {
        unsigned int distance;
        
        // Read distance from sensor
        distance = VL53L1X_Read_Distance();
        
        // Control LED based on distance threshold
        if(distance < DISTANCE_THRESHOLD && distance > 0)
        {
            LED_PIN = 1;  // Turn LED ON - object detected close
        }
        else
        {
            LED_PIN = 0;  // Turn LED OFF - object too far or no detection
        }
        
        // Update interval
        Delay_ms(100);
    }
}

/*
 * GPIO_Init - Configure GPIO pins
 * RA4 = LED output
 * RC3 = I2C SCL (open-drain output)
 * RC4 = I2C SDA (open-drain output)
 */
void GPIO_Init(void)
{
    // Configure Port A
    ANSELA = 0x00;           // All digital I/O
    TRISA = 0b00010111;      // RA4 as output (LED)
    PORTA = 0x00;
    
    // Configure Port C
    ANSELC = 0x00;           // All digital I/O
    TRISC = 0b00011000;      // RC3 and RC4 as inputs (I2C - open drain)
    PORTC = 0x00;
}

/*
 * I2C_Init - Initialize I2C module (Master mode)
 * SCL = RC3, SDA = RC4
 * Baud Rate: 400 kHz max per VL53L1X datasheet
 * Using 100 kHz for reliability
 */
void I2C_Init(void)
{
    // I2C Clock calculation: BAUD = (Fosc / (4 * I2C_Speed)) - 1
    // For 100 kHz @ 8MHz: BAUD = (8MHz / (4 * 100kHz)) - 1 = 19
    // For 400 kHz @ 8MHz: BAUD = (8MHz / (4 * 400kHz)) - 1 = 4
    SSP1ADD = 19;  // 100 kHz (recommended for stability)
    
    // SSP1CON1: I2C Mode Select bits = 1000 (I2C Master mode)
    SSP1CON1 = 0b00101000;   // Enable I2C, Master mode
    
    SSP1CON2 = 0x00;
    SSP1CON3 = 0x00;
    
    SSP1STATbits.SMP = 1;    // Slew rate disabled for standard I2C speeds
}

/*
 * I2C_Start - Generate I2C START condition
 */
void I2C_Start(void)
{
    SSP1CON2bits.SEN = 1;    // Start condition
    while(SSP1CON2bits.SEN);  // Wait for completion
}

/*
 * I2C_Stop - Generate I2C STOP condition
 */
void I2C_Stop(void)
{
    SSP1CON2bits.PEN = 1;    // Stop condition
    while(SSP1CON2bits.PEN);  // Wait for completion
}

/*
 * I2C_Send_Byte - Send a byte over I2C
 * Returns: ACKSTAT (0 = ACK received, 1 = NACK received)
 */
void I2C_Send_Byte(unsigned char data)
{
    SSP1BUF = data;
    while(SSP1STATbits.BF);   // Wait for transmission
    while(SSP1CON2bits.ACKEN); // Wait for ACK bit processing
}

/*
 * I2C_Receive_Byte - Receive a byte over I2C
 * Parameter: ack - 1 to send ACK, 0 to send NACK
 * Returns: Received byte
 */
unsigned char I2C_Receive_Byte(unsigned char ack)
{
    unsigned char data;
    
    SSP1CON2bits.RCEN = 1;   // Enable receive mode
    while(SSP1STATbits.BF == 0);  // Wait for data
    data = SSP1BUF;
    
    if(ack)
    {
        SSP1CON2bits.ACKDT = 0;  // Send ACK
    }
    else
    {
        SSP1CON2bits.ACKDT = 1;  // Send NACK
    }
    SSP1CON2bits.ACKEN = 1;
    while(SSP1CON2bits.ACKEN);
    
    return data;
}

/*
 * VL53L1X_Check_Sensor - Verify VL53L1X sensor presence via WHO_AM_I register
 * Returns: 1 if sensor found, 0 if not found
 * 
 * Per datasheet: WHO_AM_I register at 0x010F should return 0xEA
 */
unsigned char VL53L1X_Check_Sensor(void)
{
    unsigned char who_am_i = 0;
    unsigned char status;
    
    // Read WHO_AM_I register (0x010F)
    status = I2C_Read_Register_16bit(VL53L1X_WHO_AM_I, &who_am_i);
    
    if(status != 1)
    {
        return 0;  // I2C communication error
    }
    
    // Verify sensor ID
    if(who_am_i != VL53L1X_WHO_AM_I_VALUE)
    {
        return 0;  // Invalid sensor ID
    }
    
    return 1;  // Sensor verified
}

/*
 * VL53L1X_Init - Initialize the VL53L1X sensor
 * Returns: 1 if successful, 0 if failed
 * 
 * Note: This is a simplified initialization.
 * For complete setup, refer to VL53L1X application note for full initialization sequence
 */
unsigned char VL53L1X_Init(void)
{
    // Check if sensor is present
    if(!VL53L1X_Check_Sensor())
    {
        return 0;  // Sensor not found
    }
    
    // Additional initialization steps would go here
    // (See VL53L1X datasheet for complete initialization sequence)
    
    return 1;  // Sensor initialized
}

/*
 * VL53L1X_Start_Ranging - Start continuous distance measurement
 */
unsigned char VL53L1X_Start_Ranging(void)
{
    // This is a simplified version
    // For a complete implementation, additional initialization 
    // sequences from the datasheet should be followed
    
    return 1;
}

/*
 * I2C_Write_Register_16bit - Write to a 16-bit indexed register
 * Per datasheet format: S ADDRESS[7:0] As INDEX[15:8] As INDEX[7:0] As DATA[7:0] As P
 * Returns: 1 if success, 0 if error
 */
unsigned char I2C_Write_Register_16bit(unsigned short reg, unsigned char value)
{
    unsigned char reg_high, reg_low;
    
    reg_high = (unsigned char)((reg >> 8) & 0xFF);  // Upper byte of address
    reg_low = (unsigned char)(reg & 0xFF);           // Lower byte of address
    
    // Start condition
    I2C_Start();
    
    // Send write address (0x52)
    I2C_Send_Byte(VL53L1X_ADDRESS_WR);
    if(SSP1CON2bits.ACKSTAT) 
    {
        I2C_Stop();
        return 0;  // No ACK from slave
    }
    
    // Send register address high byte
    I2C_Send_Byte(reg_high);
    if(SSP1CON2bits.ACKSTAT)
    {
        I2C_Stop();
        return 0;
    }
    
    // Send register address low byte (auto-increment will handle subsequent bytes)
    I2C_Send_Byte(reg_low);
    if(SSP1CON2bits.ACKSTAT)
    {
        I2C_Stop();
        return 0;
    }
    
    // Send data byte
    I2C_Send_Byte(value);
    if(SSP1CON2bits.ACKSTAT)
    {
        I2C_Stop();
        return 0;
    }
    
    // Stop condition
    I2C_Stop();
    return 1;  // Success
}

/*
 * I2C_Read_Register_16bit - Read from a 16-bit indexed register
 * Per datasheet format: S ADDRESS[7:0] As INDEX[15:8] As INDEX[7:0] As P S ADDRESS[7:0] As DATA[7:0] Am P
 * Returns: 1 if success, 0 if error
 */
unsigned char I2C_Read_Register_16bit(unsigned short reg, unsigned char *value)
{
    unsigned char reg_high, reg_low;
    
    reg_high = (unsigned char)((reg >> 8) & 0xFF);
    reg_low = (unsigned char)(reg & 0xFF);
    
    // First: Set register address
    I2C_Start();
    I2C_Send_Byte(VL53L1X_ADDRESS_WR);  // Write mode
    if(SSP1CON2bits.ACKSTAT)
    {
        I2C_Stop();
        return 0;
    }
    
    // Send register address high byte
    I2C_Send_Byte(reg_high);
    if(SSP1CON2bits.ACKSTAT)
    {
        I2C_Stop();
        return 0;
    }
    
    // Send register address low byte
    I2C_Send_Byte(reg_low);
    if(SSP1CON2bits.ACKSTAT)
    {
        I2C_Stop();
        return 0;
    }
    
    // Second: Read the data (repeated START, not stop)
    I2C_Start();  // Repeated START
    I2C_Send_Byte(VL53L1X_ADDRESS_RD);  // Read mode
    if(SSP1CON2bits.ACKSTAT)
    {
        I2C_Stop();
        return 0;
    }
    
    // Receive data byte with NACK (this is the last byte)
    *value = I2C_Receive_Byte(0);  // 0 = send NACK
    
    I2C_Stop();
    return 1;  // Success
}

/*
 * I2C_Read_Sequential - Read multiple consecutive bytes using auto-increment
 * Per datasheet: S ADDRESS[7:0] As INDEX[7:0] As P S ADDRESS[7:0] As DATA[7:0] Am DATA[7:0] Am DATA[7:0] As P
 * Returns: 1 if success, 0 if error
 */
unsigned char I2C_Read_Sequential(unsigned short start_reg, unsigned char *data, unsigned char length)
{
    unsigned char i;
    unsigned char reg_high, reg_low;
    
    reg_high = (unsigned char)((start_reg >> 8) & 0xFF);
    reg_low = (unsigned char)(start_reg & 0xFF);
    
    // Set starting register address
    I2C_Start();
    I2C_Send_Byte(VL53L1X_ADDRESS_WR);
    if(SSP1CON2bits.ACKSTAT)
    {
        I2C_Stop();
        return 0;
    }
    
    I2C_Send_Byte(reg_high);
    if(SSP1CON2bits.ACKSTAT)
    {
        I2C_Stop();
        return 0;
    }
    
    I2C_Send_Byte(reg_low);
    if(SSP1CON2bits.ACKSTAT)
    {
        I2C_Stop();
        return 0;
    }
    
    // Read sequential data with auto-increment
    I2C_Start();  // Repeated START
    I2C_Send_Byte(VL53L1X_ADDRESS_RD);
    if(SSP1CON2bits.ACKSTAT)
    {
        I2C_Stop();
        return 0;
    }
    
    // Read all data bytes
    for(i = 0; i < length; i++)
    {
        if(i < (length - 1))
        {
            data[i] = I2C_Receive_Byte(1);  // Send ACK for all but last byte
        }
        else
        {
            data[i] = I2C_Receive_Byte(0);  // Send NACK for last byte
        }
    }
    
    I2C_Stop();
    return 1;  // Success
}

/*
 * VL53L1X_Read_Distance - Read distance measurement from sensor
 * Returns: Distance in millimeters (0 if error)
 * 
 * Reads the distance result from register 0x0089
 */
unsigned int VL53L1X_Read_Distance(void)
{
    unsigned char distance_high, distance_low;
    unsigned int distance = 0;
    
    // Read high byte of distance
    if(!I2C_Read_Register_16bit(VL53L1X_RESULT_RANGE, &distance_high))
    {
        return 0;  // Error reading
    }
    
    // Read low byte of distance
    if(!I2C_Read_Register_16bit(VL53L1X_RESULT_RANGE + 1, &distance_low))
    {
        return 0;  // Error reading
    }
    
    // Combine bytes: distance is 16-bit value
    distance = (distance_high << 8) | distance_low;
    
    return distance;
}

/*
 * Delay_ms - Software delay in milliseconds
 */
void Delay_ms(unsigned int ms)
{
    unsigned int i, j;
    for(i = 0; i < ms; i++)
        for(j = 0; j < 124; j++);  // Calibrated for 8MHz
}

/*
 * Hardware Connections:
 * 
 * VL53L1X -> PIC16F15324
 * VCC     -> Pin 15 (VDD)
 * GND     -> Pin 8/16 (VSS)
 * SCL     -> RC3 (Pin 11)
 * SDA     -> RC4 (Pin 10)
 * GPIO0   -> Not used (optional interrupt)
 * GPIO1   -> Not used (optional interrupt)
 * 
 * LED Circuit:
 * RA4 (Pin 3) -> 220Ω Resistor -> LED Anode
 * LED Cathode -> VSS (Ground)
 * 
 * I2C Connections:
 * - 4.7kΩ pull-up resistors on both SCL and SDA (to VDD)
 * - 0.1µF bypass capacitor on VL53L1X VCC
 * 
 * Notes:
 * - VL53L1X operates at 2.8V to 3.6V (use level shifter if needed)
 * - Ensure proper I2C pull-ups for reliable communication
 * - Distance readings are in millimeters
 * - Adjust DISTANCE_THRESHOLD as needed (0-4000mm typical range)
 */
