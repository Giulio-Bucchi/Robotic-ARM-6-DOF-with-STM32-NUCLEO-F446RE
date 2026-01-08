/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f4xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <string.h>
#include <stdio.h>
#include <stdlib.h>

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
// Sistema di controllo con pulsanti
uint8_t selected_servo = 0;  // Servo selezionato (0-5 nel codice = Servo 1-6 fisicamente)
// SERVO 3 (tuo) = SERVO 2 (codice/array index 2) - parte da 120° (braccio in basso)
uint8_t servo_angles[6] = {90, 90, 120, 90, 90, 90}; // Angoli dei 6 servo

// Direzione di rotazione per ogni servo (1 = avanti, -1 = indietro)
int8_t servo_direction[6] = {1, 1, 1, 1, 1, 1};

// Definizione angoli per i 4 pulsanti (SOLO GPIOA per test)
// A0=0°, A1=90°, A4=180°, A5=90° (ignoriamo A2/A3 per ora)
const uint8_t button_angles[6] = {0, 90, 90, 135, 180, 90};

// Debounce separato per ogni pulsante (6 pulsanti angoli + USER button = 7)
uint32_t last_button_time[7] = {0, 0, 0, 0, 0, 0, 0};
#define DEBOUNCE_DELAY 250  // ms

// Variabile per memorizzare l'angolo del servo
uint8_t servo_angle = 90; // Angolo iniziale a 90°
uint8_t servo_channel = 0; // Canale del servo (0-15)

// Buffer per ricezione UART
uint8_t rx_buffer[50];    // Buffer per costruire il comando
uint8_t rx_index = 0;     // Indice nel buffer comando
uint8_t rx_data;          // Singolo carattere ricevuto

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_I2C1_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

// Indirizzo I2C del PCA9685
#define PCA9685_ADDRESS (0x40 << 1)

// Registri PCA9685
#define PCA9685_MODE1 0x00
#define PCA9685_PRESCALE 0xFE
#define LED0_ON_L 0x06

// Funzione per convertire angolo in valore PWM per servo
// Per servo standard: 1ms=0°, 1.5ms=90°, 2ms=180°
// Con prescaler per 50Hz: 4096 ticks = 20ms
// 1ms = 205 ticks, 1.5ms = 307 ticks, 2ms = 410 ticks
uint16_t Servo_AngleToPWM(uint8_t angle, uint8_t channel) {
    // Limita l'angolo tra 0 e 180
    if (angle > 180) angle = 180;
    
    // Tutti i servo usano lo stesso range INVERTITO
    // Range INVERTITO: da 410 (2ms) a 205 (1ms)
    uint16_t pwm = 410 - ((uint32_t)angle * 205 / 180);
    
    return pwm;
}

// Funzione di inizializzazione con controllo errori
HAL_StatusTypeDef PCA9685_Init(void) {
    uint8_t data;
    HAL_StatusTypeDef status;
    
    // Test 1: Verifica comunicazione I2C
    status = HAL_I2C_IsDeviceReady(&hi2c1, PCA9685_ADDRESS, 3, 100);
    if (status != HAL_OK) {
        // NON TROVA IL PCA9685: lampeggia 15 volte VELOCE
        for(int i = 0; i < 15; i++) {
            HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_SET);
            HAL_Delay(80);
            HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);
            HAL_Delay(80);
        }
        return status;
    }
    
    // Reset completo via I2C (indirizzo speciale - software reset)
    data = 0x06;
    HAL_I2C_Master_Transmit(&hi2c1, 0x00, &data, 1, 100);
    HAL_Delay(10);
    
    // Sleep mode per cambiare prescaler
    data = 0x10; // Solo bit SLEEP attivo
    status = HAL_I2C_Mem_Write(&hi2c1, PCA9685_ADDRESS, PCA9685_MODE1, 1, &data, 1, 100);
    if (status != HAL_OK) {
        // ERRORE scrittura MODE1: lampeggia LED veloce 5 volte
        for(int i = 0; i < 5; i++) {
            HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_SET);
            HAL_Delay(100);
            HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);
            HAL_Delay(100);
        }
        return status;
    }
    HAL_Delay(10);
    
    // Imposta frequenza a ~50Hz (per servo)
    // Prescaler = round(25MHz / (4096 * 50Hz)) - 1 = 121
    data = 121; // Prescaler per ~50Hz (standard per servo)
    status = HAL_I2C_Mem_Write(&hi2c1, PCA9685_ADDRESS, PCA9685_PRESCALE, 1, &data, 1, 100);
    if (status != HAL_OK) return status;
    HAL_Delay(5);
    
    // Risveglia il chip: scrivi 0x00 per uscire da sleep
    data = 0x00;
    status = HAL_I2C_Mem_Write(&hi2c1, PCA9685_ADDRESS, PCA9685_MODE1, 1, &data, 1, 100);
    if (status != HAL_OK) return status;
    HAL_Delay(1);
    
    // Abilita auto-increment per scritture multiple più veloci
    data = 0x20; // Bit 5 = AI (Auto-Increment)
    status = HAL_I2C_Mem_Write(&hi2c1, PCA9685_ADDRESS, PCA9685_MODE1, 1, &data, 1, 100);
    if (status != HAL_OK) return status;
    HAL_Delay(1);
    
    // IMPORTANTE: Spegni TUTTI i canali PWM prima di usarli
    // Questo previene segnali spazzatura che fanno tremare i servo
    for(int i = 0; i < 16; i++) {
        // Imposta OFF = 4096 (sempre OFF, nessun segnale PWM)
        uint8_t off_data[4] = {0, 0, 0, 0x10}; // 0x1000 = 4096 in little-endian
        HAL_I2C_Mem_Write(&hi2c1, PCA9685_ADDRESS, LED0_ON_L + 4 * i, 1, off_data, 4, 100);
    }
    HAL_Delay(10);
    
    // Leggi MODE1 per debug
    uint8_t mode1_read;
    status = HAL_I2C_Mem_Read(&hi2c1, PCA9685_ADDRESS, PCA9685_MODE1, 1, &mode1_read, 1, 100);
    
    // Debug: lampeggia in base al valore di MODE1
    if (mode1_read == 0x20 || mode1_read == 0x21) {
        // SUCCESS - 2 lampeggi LENTI
        for(int i = 0; i < 2; i++) {
            HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_SET);
            HAL_Delay(300);
            HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);
            HAL_Delay(300);
        }
    } else if (mode1_read & 0x10) {
        // Ancora in SLEEP - 10 lampeggi
        for(int i = 0; i < 10; i++) {
            HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_SET);
            HAL_Delay(100);
            HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);
            HAL_Delay(100);
        }
    } else {
        // Valore MODE1 inaspettato: lampeggia 7 volte
        for(int i = 0; i < 7; i++) {
            HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_SET);
            HAL_Delay(150);
            HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);
            HAL_Delay(150);
        }
    }
    
    return HAL_OK;
}

// Funzione per impostare PWM con controllo errori
HAL_StatusTypeDef PCA9685_SetPWM(uint8_t channel, uint16_t on, uint16_t off) {
    uint8_t data[4];
    data[0] = on & 0xFF;
    data[1] = on >> 8;
    data[2] = off & 0xFF;
    data[3] = off >> 8;
    
    return HAL_I2C_Mem_Write(&hi2c1, PCA9685_ADDRESS, LED0_ON_L + 4 * channel, 1, data, 4, 100);
}

// Funzione per muovere il servo a un angolo specifico
void Servo_SetAngle(uint8_t channel, uint8_t angle) {
    uint16_t pwm = Servo_AngleToPWM(angle, channel);
    PCA9685_SetPWM(channel, 0, pwm);
}

// Funzione per indicare il servo selezionato con lampeggi LED
void IndicateSelectedServo(uint8_t servo_num) {
    // Lampeggia LED N volte (N = servo_num + 1)
    for(int i = 0; i <= servo_num; i++) {
        HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_SET);
        HAL_Delay(150);
        HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);
        HAL_Delay(150);
    }
    HAL_Delay(500); // Pausa finale
}

// Funzione per verificare se un pulsante è premuto (con debounce)
// button_id: 0-5 per i pulsanti angoli, 6 per USER button
uint8_t IsButtonPressed(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin, uint8_t button_id) {
    if (HAL_GetTick() - last_button_time[button_id] < DEBOUNCE_DELAY) {
        return 0; // Ignora se troppo presto
    }
    
    if (HAL_GPIO_ReadPin(GPIOx, GPIO_Pin) == GPIO_PIN_RESET) {
        last_button_time[button_id] = HAL_GetTick();
        return 1;
    }
    return 0;
}

// Funzione per inviare messaggio via UART
void UART_SendString(char* str) {
    HAL_UART_Transmit(&huart2, (uint8_t*)str, strlen(str), 100);
}

// Funzione per processare comandi ricevuti via UART
void ProcessCommand(char* cmd) {
    char response[100];
    
    // Comando: "angle XXX" - imposta angolo del servo (0-180)
    if (strncmp(cmd, "angle ", 6) == 0) {
        int angle = atoi(cmd + 6);
        if (angle >= 0 && angle <= 180) {
            servo_angle = angle;
            Servo_SetAngle(servo_channel, servo_angle);
            sprintf(response, "OK: Servo a %d gradi\r\n", servo_angle);
            UART_SendString(response);
        } else {
            UART_SendString("ERROR: Angolo deve essere 0-180\r\n");
        }
    }
    // Comando: "channel X" - seleziona canale servo (0-15)
    else if (strncmp(cmd, "channel ", 8) == 0) {
        int ch = atoi(cmd + 8);
        if (ch >= 0 && ch <= 15) {
            servo_channel = ch;
            sprintf(response, "OK: Canale %d selezionato\r\n", servo_channel);
            UART_SendString(response);
        } else {
            UART_SendString("ERROR: Canale deve essere 0-15\r\n");
        }
    }
    // Comando: "status" - mostra stato corrente
    else if (strcmp(cmd, "status") == 0) {
        sprintf(response, "Canale: %d, Angolo: %d°\r\n", servo_channel, servo_angle);
        UART_SendString(response);
    }
    // Comando: "help" - mostra comandi disponibili
    else if (strcmp(cmd, "help") == 0) {
        UART_SendString("\r\n=== COMANDI DISPONIBILI ===\r\n");
        UART_SendString("angle XXX    - Imposta angolo (0-180)\r\n");
        UART_SendString("channel X    - Seleziona canale (0-15)\r\n");
        UART_SendString("status       - Mostra stato corrente\r\n");
        UART_SendString("help         - Mostra questo menu\r\n");
        UART_SendString("===========================\r\n\r\n");
    }
    else {
        UART_SendString("ERROR: Comando non riconosciuto. Scrivi 'help'\r\n");
    }
}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_USART2_UART_Init();
  MX_I2C1_Init();
  /* USER CODE BEGIN 2 */

  // LED verde lampeggia 3 volte VELOCE - programma avviato
  for(int i = 0; i < 3; i++) {
      HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_SET);
      HAL_Delay(100);
      HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);
      HAL_Delay(100);
  }
  HAL_Delay(500);

  // Inizializza PCA9685
  HAL_StatusTypeDef init_status = PCA9685_Init();
  
  if (init_status != HAL_OK) {
      // Errore inizializzazione - LED fisso
      HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_SET);
      while(1);
  }
  
  // Posiziona i servo nelle posizioni iniziali sicure
  // TUO SERVO 3 = CODICE SERVO 2 (array index 2) - parte da 45° (braccio in basso - Foto 1)
  for(int i = 0; i < 6; i++) {
      if (i == 2) {  // SERVO 2 nel codice = TUO SERVO 3
          Servo_SetAngle(i, 120);  // Parte da 120° (posizione alta)
          servo_angles[i] = 120;
      } else {
          Servo_SetAngle(i, 90);  // Altri servo a 90°
          servo_angles[i] = 90;
      }
  }
  HAL_Delay(500);
  
  // Messaggio di benvenuto via UART
  UART_SendString("\r\n\r\n");
  UART_SendString("================================\r\n");
  UART_SendString("  CONTROLLO 6 SERVO CON PULSANTI\r\n");
  UART_SendString("  STM32 + PCA9685\r\n");
  UART_SendString("================================\r\n");
  UART_SendString("USER: Cambia servo\r\n");
  UART_SendString("A0-A5: Angoli (30-180)\r\n\r\n");
  
  // Indica servo selezionato inizialmente
  IndicateSelectedServo(selected_servo);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    
    // ============= PULSANTE USER: Cambio Servo =============
    if (HAL_GPIO_ReadPin(B1_GPIO_Port, B1_Pin) == GPIO_PIN_RESET) {
        // Aspetta rilascio pulsante
        while(HAL_GPIO_ReadPin(B1_GPIO_Port, B1_Pin) == GPIO_PIN_RESET) {
            HAL_Delay(10);
        }
        
        // Cambia servo selezionato (0-5)
        selected_servo = (selected_servo + 1) % 6;
        
        // Indica servo selezionato con lampeggi
        IndicateSelectedServo(selected_servo);
        
        HAL_Delay(300); // Debounce
    }
    
    // ============= TEST DIAGNOSTICO - STAMPA STATO PIN =============
    static uint32_t last_debug_time = 0;
    
    // Ogni secondo, stampa lo stato di tutti i pin (per debug con LED)
    if (HAL_GetTick() - last_debug_time > 2000) {
        last_debug_time = HAL_GetTick();
        // Lampeggio veloce = sistema vivo
        for(int i=0; i<2; i++) {
            HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_SET);
            HAL_Delay(50);
            HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);
            HAL_Delay(50);
        }
    }
    
    // ============= 3 PULSANTI: D2, D3, D4 =============
    
    // D2 (PA10): Oscillazione continua avanti-indietro
    // TUO SERVO 3 (codice servo 2) ha limiti speciali (45° ↔ 120°) per evitare blocchi
    // Altri servo: range completo (0° ↔ 180°)
    if (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_10) == GPIO_PIN_RESET) {
        // Incrementa o decrementa in base alla direzione
        servo_angles[selected_servo] += (5 * servo_direction[selected_servo]);
        
        // Limiti speciali per TUO SERVO 3 (codice = 2)
        if (selected_servo == 2) {
            // TUO SERVO 3: limita tra 75° e 110° 
            if (servo_angles[selected_servo] >= 110) {
                servo_angles[selected_servo] = 110;
                servo_direction[selected_servo] = -1; // Inverte: ora va indietro
            } 
            else if (servo_angles[selected_servo] <= 75) {
                servo_angles[selected_servo] = 75;
                servo_direction[selected_servo] = 1;  // Inverte: ora va avanti
            }
        }
        // Limiti normali per altri servo
        else {
            if (servo_angles[selected_servo] >= 180) {
                servo_angles[selected_servo] = 180;
                servo_direction[selected_servo] = -1; // Inverte: ora va indietro
            } 
            else if (servo_angles[selected_servo] <= 0) {
                servo_angles[selected_servo] = 0;
                servo_direction[selected_servo] = 1;  // Inverte: ora va avanti
            }
        }
        
        Servo_SetAngle(selected_servo, servo_angles[selected_servo]);
        HAL_Delay(20); // Velocità fluida
    }
    
    // D3 (PB3): Reset posizione (90° per tutti, 45° per TUO servo 3)
    if (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_3) == GPIO_PIN_RESET) {
        if (selected_servo == 2) {  // TUO SERVO 3 = codice servo 2
            servo_angles[selected_servo] = 120;   // Torna a 45° (posizione bassa)
            Servo_SetAngle(selected_servo, 120);
        } else {
            servo_angles[selected_servo] = 90;   // Altri servo tornano a 90°
            Servo_SetAngle(selected_servo, 90);
        }
        
        // Aspetta rilascio pulsante per evitare reset multipli
        while(HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_3) == GPIO_PIN_RESET) {
            HAL_Delay(10);
        }
        HAL_Delay(200); // Debounce
    }
    
    // D4 (PB5): Non fa nulla (placeholder per future funzioni)
    // if (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_5) == GPIO_PIN_RESET) {
    //     // Nessuna azione
    // }
    
    // Piccolo delay per non sovraccaricare
    HAL_Delay(10);

  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 16;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 100000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : LD2_Pin */
  GPIO_InitStruct.Pin = LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LD2_GPIO_Port, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */
  
  // Configurazione 3 pulsanti: D2, D3, D4
  // D2=PA10 (rotazione continua), D3=PB3 (reset), D4=PB5 (nessuna azione)
  
  // PA10 (D2)
  GPIO_InitStruct.Pin = GPIO_PIN_10;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
  
  // PB3, PB5 (D3, D4)
  GPIO_InitStruct.Pin = GPIO_PIN_3 | GPIO_PIN_5;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

// Callback per ricezione UART
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
    if (huart->Instance == USART2) {
        char received = rx_data;  // Carattere ricevuto
        
        // Echo del carattere ricevuto (eccetto \r e \n che gestiamo dopo)
        if (received != '\r' && received != '\n') {
            HAL_UART_Transmit(&huart2, (uint8_t*)&received, 1, 10);
        }
        
        if (received == '\r' || received == '\n') {
            // Fine comando - elabora
            if (rx_index > 0) {
                rx_buffer[rx_index] = '\0'; // Termina stringa
                UART_SendString("\r\n");
                ProcessCommand((char*)rx_buffer);
                rx_index = 0; // Reset buffer
            } else {
                UART_SendString("\r\n");  // Solo newline se buffer vuoto
            }
        } else if (received == 127 || received == 8) {
            // Backspace
            if (rx_index > 0) {
                rx_index--;
                UART_SendString("\b \b");  // Cancella carattere visivamente
            }
        } else if (rx_index < 49) {
            // Aggiungi carattere al buffer comando
            rx_buffer[rx_index] = received;
            rx_index++;
        } else {
            // Buffer pieno - reset
            rx_index = 0;
            UART_SendString("\r\nERROR: Comando troppo lungo\r\n");
        }
        
        // Riabilita ricezione
        HAL_UART_Receive_IT(&huart2, &rx_data, 1);
    }
}

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
