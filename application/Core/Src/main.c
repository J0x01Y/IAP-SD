/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
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
#include "spi.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <string.h>
#include "mbedtls/pk.h"
#include "mbedtls/ecdsa.h"
#include "mbedtls/ecp.h"
#include "mbedtls/sha256.h"
#include "mbedtls/md.h"
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

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

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
  MX_I2C3_Init();
  MX_USART1_UART_Init();
  MX_SPI3_Init();
  /* USER CODE BEGIN 2 */
  printf("application %s Start, Build@%s [%s]\r\n", __func__, __DATE__, __TIME__);
  uint32_t func_addr = *((uint32_t *)0x2000F004);
  uint32_t opcode = *((uint32_t *)func_addr);
  
  printf("mbedtls_free() = 0x%08lX, Opcode = 0x%08lX\r\n", func_addr, opcode);
  Test_ECDSA_Verify();

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  LL_FLASH_SetLatency(LL_FLASH_LATENCY_2);
  while(LL_FLASH_GetLatency()!= LL_FLASH_LATENCY_2)
  {
  }
  LL_PWR_SetRegulVoltageScaling(LL_PWR_REGU_VOLTAGE_SCALE2);
  LL_RCC_HSE_EnableBypass();
  LL_RCC_HSE_Enable();

   /* Wait till HSE is ready */
  while(LL_RCC_HSE_IsReady() != 1)
  {

  }
  LL_RCC_PLL_ConfigDomain_SYS(LL_RCC_PLLSOURCE_HSE, LL_RCC_PLLM_DIV_8, 336, LL_RCC_PLLP_DIV_4);
  LL_RCC_PLL_Enable();

   /* Wait till PLL is ready */
  while(LL_RCC_PLL_IsReady() != 1)
  {

  }
  while (LL_PWR_IsActiveFlag_VOS() == 0)
  {
  }
  LL_RCC_SetAHBPrescaler(LL_RCC_SYSCLK_DIV_1);
  LL_RCC_SetAPB1Prescaler(LL_RCC_APB1_DIV_2);
  LL_RCC_SetAPB2Prescaler(LL_RCC_APB2_DIV_1);
  LL_RCC_SetSysClkSource(LL_RCC_SYS_CLKSOURCE_PLL);

   /* Wait till System clock is ready */
  while(LL_RCC_GetSysClkSource() != LL_RCC_SYS_CLKSOURCE_STATUS_PLL)
  {

  }
  LL_SetSystemCoreClock(84000000);

   /* Update the time base */
  if (HAL_InitTick (TICK_INT_PRIORITY) != HAL_OK)
  {
    Error_Handler();
  }
  LL_RCC_SetTIMPrescaler(LL_RCC_TIM_PRESCALER_TWICE);
}

/* USER CODE BEGIN 4 */
// Private key
// -----BEGIN EC PRIVATE KEY-----
// MHcCAQEEIP170IwcWFyyUvvG0FqVeoU/EVcsGk6/c+vPesou5E2GoAoGCCqGSM49
// AwEHoUQDQgAElfYCLiS8pRyChZ+PS+zUtJD0qllo7d/xcYeWgYJez9z5JTsowo3C
// UDuTIwJCCRHkqtqaYKuUYCPEt6KO1YyJ+w==
// -----END EC PRIVATE KEY-----

void Test_ECDSA_Verify(void) {
  const uint8_t message[] = "Hello STM32 + mbedTLS@app!";
  const uint8_t publicKey[] ={
    "-----BEGIN PUBLIC KEY-----\r\n"
    "MFkwEwYHKoZIzj0CAQYIKoZIzj0DAQcDQgAElfYCLiS8pRyChZ+PS+zUtJD0qllo\r\n"
    "7d/xcYeWgYJez9z5JTsowo3CUDuTIwJCCRHkqtqaYKuUYCPEt6KO1YyJ+w==\r\n"
    "-----END PUBLIC KEY-----\r\n"
  };
  //ASN.1 DER format
  const uint8_t signature[] = {
    0x30, 0x44, 0x02, 0x20, 0x53, 0x35, 0x18, 0x82, 0x69, 0xc2, 0x09, 0x8e,
    0xa1, 0x3d, 0x32, 0xcf, 0x5a, 0x0d, 0xb1, 0x39, 0x1b, 0x22, 0x41, 0x6d,
    0x17, 0x9e, 0xf8, 0x27, 0xe0, 0x2e, 0x13, 0x1e, 0xe5, 0x0b, 0x1f, 0x9f,
    0x02, 0x20, 0x7e, 0x54, 0x82, 0x28, 0xb8, 0xd5, 0x24, 0x12, 0x29, 0x87,
    0xa7, 0x0b, 0x0b, 0xa0, 0x90, 0x0a, 0x59, 0x34, 0xc4, 0x4e, 0x05, 0x92,
    0x65, 0xe4, 0xac, 0xcd, 0x95, 0x3b, 0x50, 0x2f, 0xe1, 0x00
  };
  const uint8_t signature_rs[64] = {
    // r (32 bytes)
    0x53, 0x35, 0x18, 0x82, 0x69, 0xC2, 0x09, 0x8E, 0xA1, 0x3D, 0x32, 0xCF, 0x5A, 0x0D, 0xB1, 0x39,
    0x1B, 0x22, 0x41, 0x6D, 0x17, 0x9E, 0xF8, 0x27, 0xE0, 0x2E, 0x13, 0x1E, 0xE5, 0x0B, 0x1F, 0x9F,
    // s (32 bytes)
    0x7E, 0x54, 0x82, 0x28, 0xB8, 0xD5, 0x24, 0x12, 0x29, 0x87, 0xA7, 0x0B, 0x0B, 0xA0, 0x90, 0x0A,
    0x59, 0x34, 0xC4, 0x4E, 0x05, 0x92, 0x65, 0xE4, 0xAC, 0xCD, 0x95, 0x3B, 0x50, 0x2F, 0xE1, 0x00
  };

  int ret;
  uint8_t hash[32];
  mbedtls_pk_context ctx;

  // Initialize the public key context
  mbedtls_pk_init(&ctx);
  ret = mbedtls_pk_parse_public_key(&ctx, publicKey, sizeof(publicKey));
  if (ret != 0) {
    printf("Failed to parse public key! mbedtls error: -0x%04X\r\n", -ret);
    return;
  }

  // Calculate hash of the message
  const mbedtls_md_info_t *md_info;

  md_info = mbedtls_md_info_from_string("SHA256");

  mbedtls_md(md_info, message, strlen((const char*)message), hash);
  // mbedtls_sha256(message, strlen((const char*)message), hash, 0);
  printf("Hash = ");
  for (int i = 0; i < 32; i++) printf("%02X", hash[i]);
  printf("\r\n");

  // Verify signature
  ret = mbedtls_pk_verify(
    &ctx,
    MBEDTLS_MD_SHA256,
    hash, sizeof(hash),
    signature, sizeof(signature));
  if (ret == 0) {
    printf("ECDSA signature is VALID!\r\n");
  } else {
    printf("ECDSA signature is INVALID! mbedtls error: -0x%04X\r\n", -ret);
  }
  mbedtls_ecdsa_context ecdsa;

  mbedtls_ecdsa_init(&ecdsa);
  mbedtls_ecdsa_from_keypair(&ecdsa, ctx.pk_ctx);
  mbedtls_mpi r, s;
  mbedtls_mpi_init(&r);
  mbedtls_mpi_init(&s);

  mbedtls_mpi_read_binary(&r, signature_rs, 32);
  mbedtls_mpi_read_binary(&s, signature_rs + 32, 32);
  ret = mbedtls_ecdsa_verify(&ecdsa.grp, hash, sizeof(hash), &ecdsa.Q, &r, &s);
  if (ret == 0) {
    printf("ECDSA signature (RS) is VALID!\r\n");
  } else {
    printf("ECDSA signature (RS) is INVALID! -0x%04X\r\n", -ret);
  }

  mbedtls_pk_free(&ctx);
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

#ifdef  USE_FULL_ASSERT
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
  UNUSED(file);
  UNUSED(line);
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
