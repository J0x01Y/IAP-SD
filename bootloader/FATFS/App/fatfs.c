/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file   fatfs.c
  * @brief  Code for fatfs applications
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
#include "fatfs.h"

uint8_t retUSER;    /* Return value for USER */
char USERPath[4];   /* USER logical drive path */
FATFS USERFatFS;    /* File system object for USER logical drive */
FIL USERFile;       /* File object for USER */

/* USER CODE BEGIN Variables */

/* USER CODE END Variables */

void MX_FATFS_Init(void)
{
  /*## FatFS: Link the USER driver ###########################*/
  retUSER = FATFS_LinkDriver(&USER_Driver, USERPath);

  /* USER CODE BEGIN Init */
  /* additional user code for init */
  /* USER CODE END Init */
}

/**
  * @brief  Gets Time from RTC
  * @param  None
  * @retval Time in DWORD
  */
DWORD get_fattime(void)
{
  /* USER CODE BEGIN get_fattime */
  return 0;
  /* USER CODE END get_fattime */
}

/* USER CODE BEGIN Application */
uint8_t exf_getfree(uint8_t *drv, uint32_t *total, uint32_t *free)
{
  FATFS *fs1;
  uint8_t res;
  uint32_t fre_clust=0, fre_sect=0, tot_sect=0;
  res =(uint32_t)f_getfree((const TCHAR*)drv, (DWORD*)&fre_clust, &fs1);
  if(res==0){
    tot_sect=(fs1->n_fatent-2)*fs1->csize;
    fre_sect=fre_clust*fs1->csize;
#if _MAX_SS!=512
    tot_sect*=fs1->ssize/512;
    fre_sect*=fs1->ssize/512;
#endif
    *total=tot_sect>>1;	//單位為KB
    *free=fre_sect>>1;	//單位為KB
  }
  return res;
}
/* USER CODE END Application */
