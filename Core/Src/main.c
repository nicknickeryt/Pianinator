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
#include "gpio.h"
#include "usart.h"
#include "usb_device.h"
#include "usbd_midi.h"


/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
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

/* USER CODE BEGIN PV */
uint8_t midi_rx_byte;    // pojedynczy odebrany bajt z UART2
uint8_t last_status = 0; // zapamiętany ostatni status (running status)
uint8_t midi_data[2];    // bufor danych MIDI (DATA1, DATA2)
uint8_t data_count = 0;  // ile danych już odebrano

char uart_tx_buf[64]; // bufor do wysyłania tekstu na PC

const char *note_names[12] = {"C",  "C#", "D",  "D#", "E",  "F",
                              "F#", "G",  "G#", "A",  "A#", "B"};
uint8_t usb_midi_packet[4];
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
static void USB_MIDI_SendNote(uint8_t note, uint8_t velocity, uint8_t channel,
                              uint8_t note_on) {
  // sprawdzamy czy endpoint wolny
  if (USBD_MIDI_GetState(&hUsbDeviceFS) != MIDI_IDLE)
    return;

  if (note_on) {
    usb_midi_packet[0] = 0x09; // CIN: Note On
    usb_midi_packet[1] = 0x90 | (channel & 0x0F);
  } else {
    usb_midi_packet[0] = 0x08; // CIN: Note Off
    usb_midi_packet[1] = 0x80 | (channel & 0x0F);
  }

  usb_midi_packet[2] = note;
  usb_midi_packet[3] = velocity;

  USBD_MIDI_SendPackets(&hUsbDeviceFS, usb_midi_packet, MIDI_EPIN_SIZE);
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
  if (huart->Instance == USART2) {
    uint8_t byte = midi_rx_byte;

    if (byte & 0x80) // status byte
    {
      last_status = byte;
      data_count = 0; // reset licznika danych
    } else            // data byte
    {
      if (last_status == 0) {
        // brak statusu → ignorujemy
        goto next_byte;
      }

      midi_data[data_count++] = byte;

      // NOTE ON / OFF potrzebuje 2 bajtów danych
      uint8_t cmd = last_status & 0xF0;
      if((cmd == 0x90 || cmd == 0x80) && data_count == 2) {
        uint8_t note = midi_data[0];
        uint8_t velocity = midi_data[1];
        uint8_t channel = (last_status & 0x0F) + 1;

        uint8_t note_on = (cmd == 0x90 && velocity > 0);

        int octave = (note / 12);
        const char *name = note_names[note % 12];

        int len = snprintf(
            uart_tx_buf, sizeof(uart_tx_buf), "%s %s%d ch=%d vel=%d\r\n",
            note_on ? "NOTE ON " : "NOTE OFF", name, octave, channel, velocity);

        HAL_UART_Transmit(&huart1, (uint8_t *)uart_tx_buf, len, HAL_MAX_DELAY);

        // >>> USB MIDI <<<
        USB_MIDI_SendNote(note, velocity, channel - 1, note_on);

        data_count = 0;
      }
    }

  next_byte:
    // ponownie uruchamiamy odbiór kolejnego bajtu MIDI
    HAL_UART_Receive_IT(&huart2, &midi_rx_byte, 1);
  }
}

/* USER CODE END 0 */

/**
 * @brief  The application entry point.
 * @retval int
 */
int main(void) {

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick.
   */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_USART1_UART_Init();
  MX_USART2_UART_Init();
  MX_USB_DEVICE_Init();
  /* USER CODE BEGIN 2 */
  HAL_UART_Receive_IT(&huart2, &midi_rx_byte, 1);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1) {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
 * @brief System Clock Configuration
 * @retval None
 */
void SystemClock_Config(void) {
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
   * in the RCC_OscInitTypeDef structure.
   */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
   */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK |
                                RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK) {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USB;
  PeriphClkInit.UsbClockSelection = RCC_USBCLKSOURCE_PLL_DIV1_5;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK) {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
 * @brief  This function is executed in case of error occurrence.
 * @retval None
 */
void Error_Handler(void) {
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1) {
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
void assert_failed(uint8_t *file, uint32_t line) {
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line
     number, ex: printf("Wrong parameters value: file %s on line %d\r\n", file,
     line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
