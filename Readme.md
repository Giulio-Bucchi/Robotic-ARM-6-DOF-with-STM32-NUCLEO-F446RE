# ROBOTIC ARM WITH STM32 NUCLEO F446RE 

## HARDWARE

To start this project you need hardware components :

    - Stm32 nucleo F446RE 
    - PWM servo driver shield I2C PCA9685
    - Power supply output 6 A and 5 V or 6 V
    - 6 DOF Robotic Mechanical ARM with servos :
        - you can buy the kit from aliexpress or using 3d print for the chassis
        - if you want to buy servos the one that i m using MG996R can spin only for 180 degrees
        

    
## Optional : 

Breadboard with 3 buttons to control the spin of the servos othwerise you can control it with the code by the main.c but you ll have to write it.






## Video of robotic arm :

<video src="https://github.com/user-attachments/assets/63f48958-d302-423b-9053-b8e0cf829b70" controls width="600"></video>




## Hardware Wiring Table

| Device         | Pin/Port         | Connects To           | Description                          |
|:--------------:|:----------------:|:---------------------:|:-------------------------------------:|
| **STM32**      | GND              | PCA9685 GND           | Common ground                        |
| **STM32**      | 3V3              | PCA9685 VCC           | Power for logic (use 5V if supported)|
| **STM32**      | SDA              | PCA9685 SDA           | I2C Data                             |
| **STM32**      | SCL              | PCA9685 SCL           | I2C Clock                            |
| **PCA9685**    | V+ (Power)       | Power Supply 5V 6A    | Servo motors power                   |
| **PCA9685**    | GND              | Power Supply GND      | Power ground for servos              |
| **PCA9685**    | PWM0–PWM5        | Servo 1–6 (Signal)    | Signal wire for each servo           |
| **Breadboard** | Button 1         | STM32 D2              | Push button 1 (to GND)               |
| **Breadboard** | Button 2         | STM32 D3              | Push button 2 (to GND)               |
| **Breadboard** | Button 3         | STM32 D4              | Push button 3 (to GND)               |
| **Breadboard** | GND rail         | STM32 GND             | All buttons connected to ground      |

---
**Wiring Notes:**
- Each button is connected between the STM32 digital pin (D2, D3, D4) and GND.
- Servo motors are powered by the PCA9685, which is supplied by a 5V 6A power source.
- Communication between STM32 and PCA9685 uses I2C (SDA/SCL).


## How to start the project Software: 

First of all you need to download STM32CubeMX. Once you downloaded it, you can chose file -> new project -> board selector and in this part inside the field "Commercial Part number" you type the F446RE than you select the board and click start project.

![](imgs/STM32.png)

In Pinout & Configuration is very important to select Connectivity , I2C1 , on the field disable put I2C, select GPIO settings ( under the board) select PB8 and PB9 as GPIO Pull UP. Once u done that u type generate code and it will make the folder of the name of the project.

### VS Code Setup

1. Open VS Code and install the **STM32 VS Code Extension** (`stm32-vscode-extension.stm32-vscode-extension`)
2. Accept all notifications that pop up during installation
3. **Important**: Set **ST-Link server** as the debugger (this is required for debugging STM32 boards)
4. The extension provides:
   - Integrated build commands
   - Debug configuration
   - Flash/upload capabilities
   - Serial monitor for UART communication

Once you done all of that we can start building our code in the Core/Src/main.c


### Software Features

This project implements a servo control system with multiple control modes:

#### 1. **Button Control Mode**
- **USER Button (Blue button on STM32)**: Cycles through servos 1-6
- **Button D2**: Moves selected servo forward/backward (auto-oscillation while pressed)
- **Button D3**: Resets selected servo to default position (90° or 120° depending on servo)
- **LED Indicator**: Blinks N times to show which servo is selected (N = servo number)



#### 1. **Key Functions**
- **PCA9685 I2C Communication**: Controls up to 16 servos via PWM
- **Servo Calibration**: Inverted PWM mapping (410 ticks = 0°, 205 ticks = 180°)
- **Safe Initialization**: All servos start at safe positions, PWM disabled when idle
- **Error Handling**: LED blink patterns indicate initialization errors

#### Code Structure
- `PCA9685_Init()`: Initializes PWM driver with 50Hz frequency
- `Servo_SetAngle()`: Moves servo to specified angle
- `Servo_AngleToPWM()`: Converts angle to PWM duty cycle
- `PCA9685_DisablePWM()`: Turns off servo to prevent jitter
- Button debouncing and auto-oscillation for smooth control


## Building and Flashing

### Prerequisites
- STM32CubeMX (for project generation)
- CMake and Ninja (build system)
- ARM GCC toolchain (`arm-none-eabi-gcc`)
- STM32CubeProgrammer (for flashing)
- VS Code with STM32 extension (optional but recommended)


Or use VS Code tasks:
- **Build STM32**: Compiles the project
- **Clean Build**: Removes build files
- **Flash STM32**: Programs the board
- **Clean and Rebuild**: Full rebuild from scratch


## Troubleshooting

### Common Issues

**LED blinks 15 times fast**: PCA9685 not detected
- Check I2C wiring (SDA/SCL)
- Verify PCA9685 power (VCC and V+)
- Check I2C pull-up resistors on PB8/PB9

**LED blinks 5 times**: PCA9685 initialization error
- Check I2C communication
- Verify PCA9685 address (default 0x40)

**Servos jittering**: 
- Check power supply (5V 6A minimum)
- Ensure common ground between STM32, PCA9685, and power supply
- PWM is automatically disabled when idle to prevent jitter

**Servo not responding**:
- Verify servo is connected to correct PWM channel (0-5)
- Check servo power connections
- Use UART commands to test individual servos


## FUTURE PROJECT : USING A GLOVE TO CONTROL THE ARM 

