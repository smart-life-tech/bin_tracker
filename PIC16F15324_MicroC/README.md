# PIC16F15324 MicroC IDE Projects

This folder contains C code examples for the PIC16F15324 microcontroller using MikroElektronika MicroC IDE.

## Project Files

### 1. **main.c** - Basic GPIO & LED Blinking
- Simple LED blinking example
- GPIO configuration
- Basic delay function
- Good starting point for beginners

**Features:**
- Blinks LED on RA4 pin
- 500ms on/off timing
- Software-based delay

### 2. **timer_example.c** - Timer-Based Interrupts
- Timer0 interrupt-driven LED blinking
- More accurate timing using hardware timer
- Interrupt service routine (ISR) implementation
- 1ms interrupt intervals

**Features:**
- Hardware Timer0 for precise timing
- Global interrupt handling
- LED toggle every 2 seconds

### 3. **uart_example.c** - Serial Communication
- UART communication at 9600 baud
- Serial receive and transmit
- Echo back received characters
- Simple command processing (L=LED on, O=LED off)

**Features:**
- TX on RC0 (Pin 14)
- RX on RC1 (Pin 13)
- 9600 baud rate @ 8MHz
- Interrupt-driven receive

### 4. **vl53l1x_led_control.c** - Time of Flight Sensor LED Control
- VL53L1X (TOF400C) laser distance sensor integration
- I2C communication with sensor
- LED control based on proximity detection
- Distance threshold-based triggering

**Features:**
- I2C Master mode at 100 kHz
- SCL on RC3 (Pin 11), SDA on RC4 (Pin 10)
- Reads distance measurements in millimeters
- LED turns ON when object is closer than threshold (configurable)
- Perfect for proximity-based control applications

## Hardware Configuration

### Pinout - PIC16F15324 (DIP-16)
```
Pin 1  : MCLR  (Reset)
Pin 2  : RA5
Pin 3  : RA4   (LED output)
Pin 4  : RA3
Pin 5  : RA2
Pin 6  : RA1
Pin 7  : RA0
Pin 8  : VSS   (Ground)
Pin 9  : RC5
Pin 10 : RC4   (I2C SDA / UART RX)
Pin 11 : RC3   (I2C SCL)
Pin 12 : RC2
Pin 13 : RC1   (RX)
Pin 14 : RC0   (TX)
Pin 15 : VDD   (Power)
Pin 16 : VSS   (Ground)
```

### Recommended Circuit
- **Power Supply:** 3.3V or 5V
- **Bypass Capacitor:** 0.1µF between VDD and VSS
- **Reset:** 10kΩ pull-up on MCLR pin
- **LED:** 220Ω resistor in series with LED on RA4
- **UART:** USB-to-Serial adapter for serial communication
- **I2C Pull-ups:** 4.7kΩ resistors on SCL (RC3) and SDA (RC4) to VDD
- **VL53L1X:** 0.1µF bypass capacitor, level shifter if needed for 5V operation

## Getting Started

1. **Install MicroC IDE** for PIC microcontrollers (MikroElektronika)
2. **Create New Project:**
   - Project → New Project
   - Select Device: PIC16F15324
   - Select Compiler: mikroC for PIC
3. **Copy Code:** Paste the example code into your project
4. **Configure:**
   - Set Oscillator to internal 8MHz (or adjust as needed)
   - Adjust baud rate if using UART
5. **Build & Program:**
   - Build project (Build → Build)
   - Download to PIC (ICSP programmer)

## Key Configurations

### Oscillator Settings
The examples use:
- **Internal Oscillator:** 8MHz
- **Configuration Bits:** LFINTOSCPWR_1MHZIDLED

### Clock Formula for Baud Rate
```
BRG = (Clock_Speed / (4 * Baud_Rate)) - 1
```
For 8MHz and 9600 baud:
- BRG = (8,000,000 / (4 * 9600)) - 1 = 207

### Delay Timing (main.c)
The software delay is calibrated for 8MHz oscillator. Adjust the inner loop value if using different frequencies:
- Formula: delay_value ≈ (Frequency_MHz / 8) × 124

## Modification Tips

### Change LED Pin
If not using RA4:
1. Update `TRISA` register for the new pin
2. Change LED toggle line: `RA4 = ~RA4` to your pin
3. Update documentation

### Change Oscillator Frequency
If using different oscillator:
1. Update `#pragma config` directives
2. Recalculate delay values: `new_delay = old_delay × (old_freq / new_freq)`
3. Recalculate UART baud rate generator values

### Add More I/O
- Configure unused pins in `GPIO_Init()`
- Set appropriate TRIS and ANSEL register bits
- Remember: TRIS bit = 1 (input), TRIS bit = 0 (output)

## VL53L1X Sensor Setup

The VL53L1X Time of Flight sensor communicates via I2C. To use it:

1. **Hardware Connections:**
   - VL53L1X VCC → PIC VDD (Pin 15)
   - VL53L1X GND → PIC VSS (Pin 8/16)
   - VL53L1X SCL → RC3 (Pin 11)
   - VL53L1X SDA → RC4 (Pin 10)

2. **I2C Pull-up Resistors:**
   - 4.7kΩ from RC3 to VDD
   - 4.7kΩ from RC4 to VDD

3. **Distance Threshold:**
   - Modify `#define DISTANCE_THRESHOLD` in vl53l1x_led_control.c
   - Value in millimeters (default: 300mm = 30cm)
   - LED turns ON when object is closer than threshold

4. **Sensor Range:**
   - Typical range: 0-4000mm
   - Works best with reflective surfaces
   - Can be affected by ambient light and surface properties

## Troubleshooting

| Issue | Solution |
|-------|----------|
| LED doesn't blink | Check pin configuration, verify power supply |
| UART not working | Verify TX/RX connections, check baud rate setting |
| I2C sensor not detected | Check SCL/SDA pull-ups, verify I2C clock speed, check sensor address |
| VL53L1X initialization fails | Verify sensor power supply (2.8-3.6V), check I2C connections |
| Inconsistent distance readings | Move sensor away from reflective surfaces, check ambient light |
| Code not running | Check MCLR pin connection, verify programmer |
| Wrong timing | Calibrate delay based on actual oscillator frequency |

## Additional Resources

- PIC16F15324 Datasheet
- MicroC IDE Help Documentation
- MikroElektronika Reference Manual

## Notes

- These examples use polling-based approach for simplicity
- For production code, consider using interrupts for UART
- Always verify electrical connections before programming
- Use appropriate series resistors for LED protection
