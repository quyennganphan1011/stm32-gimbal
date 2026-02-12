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
#include "i2s.h"
#include "spi.h"
#include "usb_host.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "MahonyAHRS.h"
#include <cmath>
#include <cstdio>
#include <stdint.h>
#include <stdio.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define MPU6050_ADDR        (0x68 << 1) 
#define MPU6050_WHO_AM_I    0x75
#define MPU6050_PWR_MGMT_1  0x6B
#define MPU6050_GYRO_CONFIG 0x1B
#define MPU6050_ACCEL_CONFIG 0x1C
#define MPU6050_ACCEL_XOUT_H 0x3B

#define ACCEL_SCALE (1.0f / 16384.0f)
#define GYRO_SCALE  (1.0f / 65.5f)
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

static float ax = 0, ay = 0, az = 0;
static float gx = 0, gy = 0, gz = 0;
static float gyro_offset[3] = {0};
static float accel_offset[3] = {0};
static uint32_t last_micros = 0;
static bool mpu_initialized = false;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void MX_USB_HOST_Process(void);

/* USER CODE BEGIN PFP */
static bool MPU6050_WriteReg(uint8_t reg, uint8_t data);
static bool MPU6050_ReadRegs(uint8_t reg, uint8_t *buf, uint8_t len);
static bool MPU6050_Init(void);

static void MPU6050_CalibrateAccel(void);
static void MPU6050_CalibrateGyro(void);
static void MPU6050_ReadSensors(void);
static uint32_t micros(void);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE BEGIN 0 */

#ifdef __GNUC__
int _write(int file, char *ptr, int len) {
  for(int i = 0; i < len; i++) {
    ITM_SendChar(*ptr++);
  }
  return len;
}
#endif
// MPU6050 Driver Functions
static bool MPU6050_WriteReg(uint8_t reg, uint8_t data) {
  return HAL_I2C_Mem_Write(&hi2c1, MPU6050_ADDR, reg, 1, &data, 1, 100) == HAL_OK;
}
static bool MPU6050_ReadRegs(uint8_t reg, uint8_t *buf, uint8_t len) {
    return HAL_I2C_Mem_Read(&hi2c1, MPU6050_ADDR, reg, 1, buf, len, 100) == HAL_OK;
}
static bool MPU6050_Init(void) {
    uint8_t whoami = 0;
    
    // Wake up MPU6050 (thoát sleep mode)
    if (!MPU6050_WriteReg(MPU6050_PWR_MGMT_1, 0x00)) return false;
    HAL_Delay(10);
    
    // Kiểm tra ID: MPU6050 = 0x68
    if (!MPU6050_ReadRegs(MPU6050_WHO_AM_I, &whoami, 1)) return false;
    if (whoami != 0x68) return false;  // ⚠️ QUAN TRỌNG: 0x68 cho MPU6050 (khác 0x71 của MPU9250)
    
    // Cấu hình gyro ±500dps (0x08 = 500dps)
    if (!MPU6050_WriteReg(MPU6050_GYRO_CONFIG, 0x08)) return false;
    
    // Cấu hình accel ±2g (default - 0x00)
    // Nếu muốn ±4g: dùng 0x08; ±8g: 0x10; ±16g: 0x18
    if (!MPU6050_WriteReg(MPU6050_ACCEL_CONFIG, 0x00)) return false;
    
    return true;
}
static void MPU6050_CalibrateGyro(void) {
    int16_t gx_raw, gy_raw, gz_raw;
    float sum_gx = 0, sum_gy = 0, sum_gz = 0;
    uint8_t buf[6];
    
    HAL_Delay(2000); // Chờ cảm biến ổn định (UAV đứng yên)
    
    for (int i = 0; i < 200; i++) {
        if (MPU6050_ReadRegs(0x43, buf, 6)) { // Gyro registers (0x43-0x48)
            gx_raw = (int16_t)((buf[0] << 8) | buf[1]);
            gy_raw = (int16_t)((buf[2] << 8) | buf[3]);
            gz_raw = (int16_t)((buf[4] << 8) | buf[5]);
            
            sum_gx += gx_raw;
            sum_gy += gy_raw;
            sum_gz += gz_raw;
        }
        HAL_Delay(5);
    }
    
    // Lưu offset (deg/s)
    gyro_offset[0] = (sum_gx / 200.0f) * GYRO_SCALE;
    gyro_offset[1] = (sum_gy / 200.0f) * GYRO_SCALE;
    gyro_offset[2] = (sum_gz / 200.0f) * GYRO_SCALE;
}
static void MPU6050_CalibrateAccel(void){
  float sum_ax = 0, sum_ay = 0, sum_az = 0;
    uint8_t buf[6];
    
    for (int i = 0; i < 200; i++) {
        if (MPU6050_ReadRegs(MPU6050_ACCEL_XOUT_H, buf, 6)) {
            int16_t ax_raw = (int16_t)((buf[0] << 8) | buf[1]);
            int16_t ay_raw = (int16_t)((buf[2] << 8) | buf[3]);
            int16_t az_raw = (int16_t)((buf[4] << 8) | buf[5]);
            
            sum_ax += ax_raw * ACCEL_SCALE;
            sum_ay += ay_raw * ACCEL_SCALE;
            sum_az += az_raw * ACCEL_SCALE;
        }
        HAL_Delay(5);
    }
    
    accel_offset[0] = sum_ax / 200.0f;
    accel_offset[1] = sum_ay / 200.0f;
    accel_offset[2] = (sum_az / 200.0f) - 1.0f;
}
static void MPU6050_ReadSensors(void) {
    uint8_t buf[14];
    
    if (!MPU6050_ReadRegs(MPU6050_ACCEL_XOUT_H, buf, 14)) {
        return; // Lỗi I2C
    }
    
    // Accel data (6 bytes: 0x3B-0x40)
    int16_t accel_x = (int16_t)((buf[0] << 8) | buf[1]);
    int16_t accel_y = (int16_t)((buf[2] << 8) | buf[3]);
    int16_t accel_z = (int16_t)((buf[4] << 8) | buf[5]);
    
    // Gyro data (6 bytes: 0x43-0x48) - bỏ qua 2 byte temp (0x41-0x42)
    int16_t gyro_x = (int16_t)((buf[8] << 8) | buf[9]);
    int16_t gyro_y = (int16_t)((buf[10] << 8) | buf[11]);
    int16_t gyro_z = (int16_t)((buf[12] << 8) | buf[13]);
    
    // Chuyển sang đơn vị vật lý + bù offset
    // ax = accel_x * ACCEL_SCALE;
    // ay = accel_y * ACCEL_SCALE;
    // az = accel_z * ACCEL_SCALE;

    ax = accel_x * ACCEL_SCALE - accel_offset[0];
    ay = accel_y * ACCEL_SCALE - accel_offset[1];
    az = accel_z * ACCEL_SCALE - accel_offset[2];
    
    gx = gyro_x * GYRO_SCALE - gyro_offset[0];
    gy = gyro_y * GYRO_SCALE - gyro_offset[1];
    gz = gyro_z * GYRO_SCALE - gyro_offset[2];
}
// Timing chính xác (microsecond) dùng DWT counter
static uint32_t micros(void) {
    static bool dwt_initialized = false;
    if (!dwt_initialized) {
        CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
        DWT->CYCCNT = 0;
        DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
        dwt_initialized = true;
    }
    return DWT->CYCCNT / (HAL_RCC_GetHCLKFreq() / 1000000);
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
  MX_I2C1_Init();
  MX_I2S3_Init();
  MX_SPI1_Init();
  MX_USB_HOST_Init();
  /* USER CODE BEGIN 2 */

  CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
  ITM->LAR = 0xC5ACCE55;
  ITM->TER = 0x1;
  ITM->TCR = 0x0001000D;

  if (MPU6050_Init()){
    MPU6050_CalibrateGyro();
    MPU6050_CalibrateAccel();
    // Mahony.Kp = 1.5f;
    // Mahony.Ki = 0.0f;
    last_micros = micros();
    mpu_initialized = true;
  } else {
    Error_Handler();
  }
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */
    MX_USB_HOST_Process();

    /* USER CODE BEGIN 3 */
    if (mpu_initialized) {
      MPU6050_ReadSensors();
      uint32_t current_micros = micros();
      float dt = (current_micros - last_micros) / 1000000.0f;
      last_micros = current_micros;
      
      if (dt < 0.0005f) dt = 0.0005f;
      if (dt > 0.02f)   dt = 0.02f;

      // MahonyFilter.updateIMU(gx, gy, gz, ax, ay, az);
      MahonyFilter.updateIMU(gy, gx, gz, ay, ax, az);  
      printf("ax=%.2f ay=%.2f az=%.2f\n", ax, ay, az);
      printf("Roll: %.2f | Pitch: %.2f | Yaw: %.2f\n",
            MahonyFilter.getRoll(),
            MahonyFilter.getPitch(),
            MahonyFilter.getYaw());
    }
    HAL_Delay(2);
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
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 336;
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
      HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_14);
      HAL_Delay(200);
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
