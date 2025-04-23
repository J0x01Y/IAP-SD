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
#include "fatfs.h"
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
typedef void (*pFunction)(void);
typedef struct {
  uint32_t SectorAddress;
  uint32_t SectorSize;
} FlashSectorInfo;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define USER_APP_ADDRESS    0x08020000
#define READ_BUFFER_SIZE    0x1000  // 4KB
#define FW_UPDATE_FILE_NAME "app.bin"

#define SECTOR_COUNT (uint32_t)(sizeof(stm32F4SectorMap) / sizeof(FlashSectorInfo))
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
extern FATFS USERFatFS;
extern char USERPath[4];
const FlashSectorInfo stm32F4SectorMap[] = {
  {0x08000000, 0x04000},
  {0x08004000, 0x04000},
  {0x08008000, 0x04000},
  {0x0800C000, 0x04000},
  {0x08010000, 0x10000},
  {0x08020000, 0x20000},
  {0x08040000, 0x20000},
  {0x08060000, 0x20000},
  {0x08080000, 0x20000},
  {0x080A0000, 0x20000},
  {0x080C0000, 0x20000},
  {0x080E0000, 0x20000},
};
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
HAL_StatusTypeDef UpdateFW(void);
void Test_ECDSA_Verify(void);
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
  MX_FATFS_Init();
  /* USER CODE BEGIN 2 */
  printf("bootloader %s Start, Build@%s [%s]\r\n", __func__, __DATE__, __TIME__);
  Test_ECDSA_Verify();
  if(LL_GPIO_IsInputPinSet(SD_DET__GPIO_Port, SD_DET__Pin) == 0){
    UpdateFW();
  }else{
    printf("SD card not detected\r\n");
  }

  __disable_irq();
  if (((*(__IO uint32_t*)USER_APP_ADDRESS) & 0x2FFE0000 ) == 0x20000000)
  {
    pFunction application;
    printf("MSP: 0x%08lX\r\n", *(uint32_t*)USER_APP_ADDRESS);
    printf("ResetHandler: 0x%08lX\r\n", *(uint32_t*)(USER_APP_ADDRESS + 4));
    printf("Execute user Program\r\n");
    HAL_SPI_MspDeInit(&hspi3);
    LL_GPIO_DeInit(GPIOC);
    LL_GPIO_DeInit(GPIOH);
    LL_GPIO_DeInit(GPIOA);
    LL_GPIO_DeInit(GPIOB);
    LL_GPIO_DeInit(GPIOD);
    HAL_UART_MspDeInit(&huart1);
    LL_RCC_DeInit();

    // __set_BASEPRI (0x20);
    // __set_PRIMASK (1);
    // __set_FAULTMASK (1);
    // for(uint8_t i=0; i<8; i++){
    //   NVIC->ICER [i] =0xFFFFFFFF;
    //   NVIC->ICPR [i] =0xFFFFFFFF;
    // }
    // SysTick->CTRL = 0;
    // SysTick->LOAD = 0;
    // SysTick->VAL = 0;
    // __set_BASEPRI (0);
    // __set_PRIMASK (0);
    // __set_FAULTMASK (0);

    __set_CONTROL(0);
    __set_MSP(*(__IO uint32_t*) USER_APP_ADDRESS);
    __ISB();
    application = (pFunction) * (__IO uint32_t*) (USER_APP_ADDRESS + 4);
    application();
  }
  else
  {
    printf("No user Program\r\n");
  }
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
HAL_StatusTypeDef UpdateFW(void) {
  FIL fwUpdateFile;
  FRESULT res;
  UINT br;
  HAL_StatusTypeDef status = HAL_OK;
  uint8_t buffer[READ_BUFFER_SIZE];  // 4KB buffer
  FILINFO fno;
  uint32_t fwFileSize;

  printf("Updating firmware...\r\n");

  if (f_mount(&USERFatFS, (TCHAR const*)USERPath, 0) != FR_OK) {
    printf("FatFs Initialization Error\r\n");
    Error_Handler();
  }

  if (f_open(&fwUpdateFile, FW_UPDATE_FILE_NAME, FA_READ) != FR_OK) {
    printf("Failed to open %s\r\n", FW_UPDATE_FILE_NAME);
    Error_Handler();
  }

  f_stat(FW_UPDATE_FILE_NAME, &fno);
  fwFileSize = fno.fsize;
  uint32_t appEnd = USER_APP_ADDRESS + fwFileSize;
  printf("Firmware[%s] size: 0x%08lX Byte\r\n", FW_UPDATE_FILE_NAME, fwFileSize);
  for (uint32_t sectorIndex = 0; sectorIndex < SECTOR_COUNT; ++sectorIndex) {
    uint32_t sectorAddr = stm32F4SectorMap[sectorIndex].SectorAddress;
    uint32_t sectorSize = stm32F4SectorMap[sectorIndex].SectorSize;

    if (sectorAddr + sectorSize <= USER_APP_ADDRESS) {
      continue;
    }

    if (sectorAddr >= appEnd) {
      printf("Firmware update complete\r\n");
      break;
    }

    uint32_t fileOffset = sectorAddr - USER_APP_ADDRESS;
    res = f_lseek(&fwUpdateFile, fileOffset);
    if (res != FR_OK) break;

    uint8_t needUpdate = 0;

    for (uint32_t offset = 0; offset < sectorSize; offset += READ_BUFFER_SIZE) {
      uint32_t readSize = (sectorSize - offset > READ_BUFFER_SIZE) ? READ_BUFFER_SIZE : (sectorSize - offset);
      if ((sectorAddr + offset) >= appEnd) break;
      res = f_read(&fwUpdateFile, buffer, readSize, &br);
      if (res != FR_OK || br == 0) break;

      for (uint32_t i = 0; i < br; i += sizeof(uint32_t)) {
        uint32_t fileData = *((uint32_t*)(buffer + i));
        uint32_t flashData = *((uint32_t*)(sectorAddr + offset + i));
        if (fileData != flashData) {
          needUpdate = 1;
          break;
        }
      }

      if (needUpdate) break;
    }

    if (needUpdate) {
      uint32_t sectorError = 0;
      FLASH_EraseInitTypeDef eraseInit = {
        .TypeErase = FLASH_TYPEERASE_SECTORS,
        .VoltageRange = FLASH_VOLTAGE_RANGE_3,
        .Sector = FLASH_SECTOR_0 + sectorIndex,
        .NbSectors = 1
      };

      printf("Need Update 0x%08lX ~ 0x%08lX\r\n",
        sectorAddr,
        sectorAddr + sectorSize);
      HAL_FLASH_Unlock();
      status = HAL_FLASHEx_Erase(&eraseInit, &sectorError);
      HAL_FLASH_Lock();

      if (status != HAL_OK) {
        printf("Erase failed at sector %lu (error sector: %lu)\r\n", sectorIndex, sectorError);
        break;
      }

      printf("Erased sector at 0x%08lX\r\n", sectorAddr);

      f_lseek(&fwUpdateFile, fileOffset);
      HAL_FLASH_Unlock();

      for (uint32_t offset = 0; offset < sectorSize; offset += READ_BUFFER_SIZE) {
        uint32_t writeSize = (sectorSize - offset > READ_BUFFER_SIZE) ? READ_BUFFER_SIZE : (sectorSize - offset);
        if ((sectorAddr + offset) >= appEnd) break;

        res = f_read(&fwUpdateFile, buffer, writeSize, &br);
        if (res != FR_OK || br == 0) break;
        printf("Written 0x%08lX ~ 0x%08lX",
          sectorAddr + offset,
          sectorAddr + offset + br);
        for (uint32_t i = 0; i < br; i += sizeof(uint32_t)) {
          uint32_t data = *((uint32_t*)(buffer + i));
          status = HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, sectorAddr + offset + i, data);
          if (status != HAL_OK) break;
        }
        if(status != HAL_OK){
          printf("[FAILED]\r\n");
          break;
        }
        printf("[OK]\r\n");
      }

      HAL_FLASH_Lock();

      if (status == HAL_OK)
        printf("Sector 0x%08lX [%lu bytes] complete\r\n", sectorAddr, sectorSize);
      else
        printf("Sector 0x%08lX write failed\r\n", sectorAddr);
    } else {
      printf("Skip sector at 0x%08lX (No change)\r\n", sectorAddr);
    }
  }

  f_close(&fwUpdateFile);
  return status;
}

// Private key
// -----BEGIN EC PRIVATE KEY-----
// MHcCAQEEIHUdofRp8ZW1ypw8enFmC3yE0mFgHqcRIqDdan+2LrPooAoGCCqGSM49
// AwEHoUQDQgAE13leKg2pPD1VyWlHaMAwoaNuVq23WvkW+hFNfiA/ljV31mZWZNph
// xzkrOO4mx+KRYd/xcep6Bnapiv+8P0zLWg==
// -----END EC PRIVATE KEY-----

void Test_ECDSA_Verify(void) {
  const uint8_t message[] = "Hello STM32 + mbedTLS!";
  const uint8_t publicKey[] ={
    "-----BEGIN PUBLIC KEY-----\r\n"
    "MFkwEwYHKoZIzj0CAQYIKoZIzj0DAQcDQgAE13leKg2pPD1VyWlHaMAwoaNuVq23\r\n"
    "WvkW+hFNfiA/ljV31mZWZNphxzkrOO4mx+KRYd/xcep6Bnapiv+8P0zLWg==\r\n"
    "-----END PUBLIC KEY-----\r\n"
  };
  //ASN.1 DER format
  const uint8_t signature[] = {
    0x30, 0x45, 0x02, 0x21, 0x00, 0xf5, 0x31, 0x32, 0x4f, 0xd3, 0xec, 0xb4,
    0x55, 0xf0, 0x9d, 0xa6, 0x4c, 0x63, 0x76, 0xd1, 0xd7, 0xa9, 0x54, 0x68,
    0x27, 0x67, 0xe5, 0xc9, 0x5a, 0x2b, 0x09, 0x57, 0xce, 0xb4, 0xc3, 0x93,
    0x7a, 0x02, 0x20, 0x5e, 0xc4, 0xcb, 0xe4, 0x3d, 0x7a, 0x94, 0x3d, 0xd5,
    0xd2, 0x8b, 0x80, 0xbf, 0xb4, 0xac, 0x3b, 0x3c, 0xa0, 0x2a, 0x7d, 0x02,
    0xde, 0x76, 0x11, 0xf5, 0x99, 0xa6, 0x20, 0xaf, 0x8d, 0x95, 0x94
  };
  const uint8_t signature_rs[64] = {
    // r (32 bytes)
    0xf5, 0x31, 0x32, 0x4f, 0xd3, 0xec, 0xb4, 0x55, 0xf0, 0x9d, 0xa6, 0x4c, 0x63, 0x76, 0xd1, 0xd7,
    0xa9, 0x54, 0x68, 0x27, 0x67, 0xe5, 0xc9, 0x5a, 0x2b, 0x09, 0x57, 0xce, 0xb4, 0xc3, 0x93, 0x7a,
    // s (32 bytes)
    0x5e, 0xc4, 0xcb, 0xe4, 0x3d, 0x7a, 0x94, 0x3d, 0xd5, 0xd2, 0x8b, 0x80, 0xbf, 0xb4, 0xac, 0x3b,
    0x3c, 0xa0, 0x2a, 0x7d, 0x02, 0xde, 0x76, 0x11, 0xf5, 0x99, 0xa6, 0x20, 0xaf, 0x8d, 0x95, 0x94
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
  printf("Wrong parameters value: file %s on line %ld\r\n", file, line);
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
