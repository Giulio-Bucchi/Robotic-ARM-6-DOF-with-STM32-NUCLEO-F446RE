# ROBOTIC ARM WITH STM32 NUCLEO F446RE 

## HARDWARE

To start this project you need hardware components :

    - Stm32 nucleo F446RE  ( 51 euros )
    - PWM servo driver shield I2C PCA9685 ( 10 euros )
    - Power supply output 6 A and 5 V or 6 V ( 16 euros )
    - 6 DOF Robotic Mechanical ARM with servos ( 38 euros ):
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

This project implements a servo control system with multiple control modes and a detailed logic for safe and flexible servo management.


#### Servo Control Logic

- **Initialization**: At startup, all servos are moved to safe default positions (usually 90° or 120°) to avoid mechanical stress. Servo 3 is intentionally disabled for safety.
    ```c
    // In main.c
    for(int i = 0; i < 6; i++) {
            if (i == 2) { servo_angles[i] = 110; /* disabled */ }
            else if (i == 3 || i == 5) { Servo_SetAngle(i, 120); servo_angles[i] = 120; }
            else { Servo_SetAngle(i, 90); servo_angles[i] = 90; }
    }
    ```
- **Button Selection**: The USER button cycles through the six servos. The currently selected servo is indicated by LED blinks (N blinks = servo number).
    ```c
    if (HAL_GPIO_ReadPin(B1_GPIO_Port, B1_Pin) == GPIO_PIN_RESET) {
            selected_servo = (selected_servo + 1) % 6;
            IndicateSelectedServo(selected_servo); // LED blinks
            HAL_Delay(300); // Debounce
    }
    ```
- **Movement**: Pressing Button D2 moves the selected servo incrementally. The direction is managed by an array (`servo_direction[]`), allowing oscillation between min/max angles. Button D3 resets the selected servo to its default position.
    ```c
    // Move selected servo (D2 pressed)
    servo_angles[selected_servo] += (2 * servo_direction[selected_servo]);
    if (servo_angles[selected_servo] >= max_angle) { servo_angles[selected_servo] = max_angle; servo_direction[selected_servo] = -1; }
    else if (servo_angles[selected_servo] <= min_angle) { servo_angles[selected_servo] = min_angle; servo_direction[selected_servo] = 1; }
    Servo_SetAngle(selected_servo, servo_angles[selected_servo]);
    ```
- **Angle Limits**: Each servo has its own allowed range (e.g., 0–180°, 120–180°, 45–120°) to prevent over-rotation and damage. These limits are enforced in the code before sending PWM signals.
- **PWM Generation**: The function `Servo_AngleToPWM(angle, channel)` converts the desired angle to a PWM value suitable for the PCA9685. The mapping is inverted (410 ticks = 0°, 205 ticks = 180°) for standard servos.
    ```c
    uint16_t Servo_AngleToPWM(uint8_t angle, uint8_t channel) {
            if (angle > 180) angle = 180;
            return 410 - ((uint32_t)angle * 205 / 180);
    }
    ```
- **I2C Communication**: All PWM commands are sent to the PCA9685 via I2C. Initialization and error handling routines blink the LED to indicate status.
    ```c
    HAL_I2C_Mem_Write(&hi2c1, PCA9685_ADDRESS, LED0_ON_L + 4 * channel, 1, data, 4, 100);
    ```
- **Debouncing**: Button presses are debounced in software to avoid false triggers and ensure smooth control.
    ```c
    uint8_t IsButtonPressed(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin, uint8_t button_id) {
            if (HAL_GetTick() - last_button_time[button_id] < DEBOUNCE_DELAY) return 0;
            if (HAL_GPIO_ReadPin(GPIOx, GPIO_Pin) == GPIO_PIN_RESET) {
                    last_button_time[button_id] = HAL_GetTick();
                    return 1;
            }
            return 0;
    }
    ```
- **Safety**: When a button is released, the PWM signal for the servo is disabled (`PCA9685_DisablePWM()`), preventing jitter and reducing power consumption.
    ```c
    if (button_was_pressed && selected_servo != 2) {
            PCA9685_DisablePWM(selected_servo);
            button_was_pressed = 0;
    }
    ```
- **UART (if enabled)**: The code supports UART commands for advanced control and debugging, allowing direct angle/channel selection and status queries.

#### Main Code Structure

- `main.c` contains:
    - Initialization routines for GPIO, UART, I2C, and PCA9685
    - Main loop for button polling and servo control
    - Functions for angle-to-PWM conversion, servo movement, and error indication
    - Debounce logic and LED feedback
- `stm32f4xx_it.c` handles interrupts (if used)
- `main.h` defines pin mappings and hardware configuration

#### Example: Moving a Servo

1. Select servo with USER button (LED blinks N times)
2. Press D2 to move servo: code checks limits, updates angle, sends PWM
3. Release D2: PWM is disabled, servo holds position
4. Press D3 to reset servo to default angle

This logic ensures safe, reliable, and user-friendly control of all six servos.

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


# THOUGHTS : 

The project was created to understand how the STM32 works. It was built with a budget of about 110–120 euros. It was fun to build, especially learning from mistakes like burning a servo motor and the PCA9685 board. The downside is that the chassis is not very sturdy due to the weight of the gripper and the servo motors; I wouldn’t recommend the MG996R servos, but rather much more powerful ones. Unfortunately, the weight greatly affected the construction of the arm, so I used two rubber bands to hold some parts together and a 5kg dumbbell to stabilize the platform. Overall, it was a fun experience and I hope you enjoy it!




## FUTURE PROJECT : USING A GLOVE TO CONTROL THE ARM 

