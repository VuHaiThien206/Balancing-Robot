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
#include "i2c.h"
#include "tim.h"
#include "usb_device.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stdio.h"
#include "string.h"
#include "mpu6050.h"
#include "usbd_cdc_if.h"
#include "motor.h"
#include "balance.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define LOG_EVERY   20      /* print one log line every 20 control cycles (20 x 5 ms = 100 ms) */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
MPU6050_sensor mpu;
extern USBD_HandleTypeDef hUsbDeviceFS;   /* defined in usb_device.c */
static char log_buf[160];   /* must stay valid until USB finishes sending -> static */
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
/* Blocking print: only for start-up messages (before the control loop runs). */
static void usb_print(const char *s)
{
    uint32_t t = HAL_GetTick();
    while (CDC_Transmit_FS((uint8_t *)s, strlen(s)) == USBD_BUSY &&
           (HAL_GetTick() - t) < 50) { }
}

/* Non-blocking log: if USB is still busy, skip this line instead of waiting,
 * so the 200 Hz control loop is never delayed. */
static void log_try_send(void)
{
    USBD_CDC_HandleTypeDef *hcdc = (USBD_CDC_HandleTypeDef *)hUsbDeviceFS.pClassData;
    if (hcdc == NULL || hcdc->TxState != 0) return;

    snprintf(log_buf, sizeof(log_buf),
             "%.3f\t,%.3f\t,%.3f\t,%.2f\t,%.2f\t,%.2f\t,%.1f\t,%.2f\t,%.2f\t,%.1f\t,%d\r\n",
             mpu.ax, mpu.ay, mpu.az, mpu.gx, mpu.gy, mpu.gz,
             mpu.temp, mpu.roll, mpu.pitch,
             Balance_GetOutput(), Balance_IsArmed());
    CDC_Transmit_FS((uint8_t *)log_buf, strlen(log_buf));
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
  MX_USB_DEVICE_Init();
  MX_I2C1_Init();
  MX_TIM3_Init();
  /* USER CODE BEGIN 2 */
  Motor_Init(&htim3);
  HAL_Delay(1000);   /* wait for USB CDC to be ready */
  while (MPU6050_Init(&hi2c1) != HAL_OK) {
      usb_print("MPU6050 not responding!\r\n");
      HAL_Delay(1000);
  }
  usb_print("Calibrating gyroscope, keep board still...\r\n");
  HAL_Delay(1000);   /* let the sensor settle before calibrating */
  while (MPU6050_CalibrateGyro(&hi2c1) != HAL_OK) {
      usb_print("Gyro calibration failed (moving?), retrying...\r\n");
      HAL_Delay(500);
  }
  Balance_Init();
  usb_print("Ready. Hold the robot upright to start balancing.\r\n");

  uint32_t next = HAL_GetTick();   /* time of the next control cycle */
  uint16_t log_div = 0;
  uint8_t  i2c_fail = 0;
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    uint32_t now = HAL_GetTick();
    if ((int32_t)(now - next) >= 0) {
        next += BAL_LOOP_MS;
        if ((int32_t)(now - next) > 50) next = now;   /* fell far behind: resync, no catch-up burst */

        uint8_t ok = (MPU6050_ReadAll(&hi2c1, &mpu) == HAL_OK);
        if (ok) {
            i2c_fail = 0;
        } else if (++i2c_fail >= 3) {      /* I2C bus probably locked up by motor noise: restart it */
            i2c_fail = 0;
            HAL_I2C_DeInit(&hi2c1);
            __HAL_RCC_I2C1_FORCE_RESET();    /* hardware-reset the I2C peripheral (clears a stuck BUSY flag) */
            __HAL_RCC_I2C1_RELEASE_RESET();
            MX_I2C1_Init();
            MPU6050_Init(&hi2c1);          /* re-configure the sensor (gyro offsets are kept) */
        }
        Balance_Update(&mpu, ok);

        if (++log_div >= LOG_EVERY) {
            log_div = 0;
            log_try_send();
        }
    }
    /* USER CODE END 3 */
  }
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
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 168;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 7;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

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