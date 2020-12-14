/**
  ******************************************************************************
  * @file    FLASH/FLASH_EraseProgram/Src/main.c
  * @author  MCD Application Team
  * @brief   This example provides a description of how to erase and program the
  *			 STM32F4xx FLASH.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT(c) 2017 STMicroelectronics</center></h2>>
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/** @addtogroup STM32F4xx_HAL_Examples
  * @{
  */

/** @addtogroup FLASH_Program
  * @{
  */

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
#define FLASH_USER_START_ADDR   ADDR_FLASH_SECTOR_5   /* Start @ of user Flash area */
#define FLASH_USER_END_ADDR     ADDR_FLASH_SECTOR_11  +  GetSectorSize(ADDR_FLASH_SECTOR_11) -1 /* End @ of user Flash area : sector start address + sector size -1 */

#define DATA_32                 ((uint32_t)0x12345678)

/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
uint32_t FirstSector = 0, NbOfSectors = 0, Address = 0;
uint32_t SectorError = 0;
__IO uint64_t data32 = 0 , MemoryProgramStatus = 0;

/*Variable used for Erase procedure*/
static FLASH_EraseInitTypeDef EraseInitStruct;

/* Private function prototypes -----------------------------------------------*/
static void SystemClock_Config(void);
static uint32_t GetSector(uint32_t Address);
static uint32_t GetSectorSize(uint32_t Sector);

bb BB_ROOK_6[64];

const bb MAGIC_ROOK[64] = {
    0x0080004000608010L, 0x2240100040012002L, 0x008008a000841000L,
    0x0100204900500004L, 0x020008200200100cL, 0x40800c0080020003L,
    0x0080018002000100L, 0x4200042040820d04L, 0x10208008a8400480L,
    0x4064402010024000L, 0x2181002000c10212L, 0x5101000850002100L,
    0x0010800400080081L, 0x0012000200300815L, 0x060200080e002401L,
    0x4282000420944201L, 0x1040208000400091L, 0x0010004040002008L,
    0x0082020020804011L, 0x0005420010220208L, 0x8010510018010004L,
    0x05050100088a1400L, 0x0009008080020001L, 0x2001060000408c01L,
    0x0060400280008024L, 0x9810401180200382L, 0x0200201200420080L,
    0x0280300100210048L, 0x0000080080800400L, 0x0002010200081004L,
    0x8089000900040200L, 0x0040008200340047L, 0x0400884010800061L,
    0xc202401000402000L, 0x0800401301002004L, 0x4c43502042000a00L,
    0x0004a80082800400L, 0xd804040080800200L, 0x060200080e002401L,
    0x0203216082000104L, 0x0000804000308000L, 0x004008100020a000L,
    0x1001208042020012L, 0x0400220088420010L, 0x8010510018010004L,
    0x8009000214010048L, 0x6445006200130004L, 0x000a008402460003L,
    0x0080044014200240L, 0x0040012182411500L, 0x0003102001430100L,
    0x4c43502042000a00L, 0x1008000400288080L, 0x0806003008040200L,
    0x4200020801304400L, 0x8100640912804a00L, 0x300300a043168001L,
    0x0106610218400081L, 0x008200c008108022L, 0x0201041861017001L,
    0x00020010200884e2L, 0x0205000e18440001L, 0x202008104a08810cL,
    0x800a208440230402L
};

const int SHIFT_ROOK[64] = {
    52, 53, 53, 53, 53, 53, 53, 52,
    53, 54, 54, 54, 54, 54, 54, 53,
    53, 54, 54, 54, 54, 54, 54, 53,
    53, 54, 54, 54, 54, 54, 54, 53,
    53, 54, 54, 54, 54, 54, 54, 53,
    53, 54, 54, 54, 54, 54, 54, 53,
    53, 54, 54, 54, 54, 54, 54, 53,
    52, 53, 53, 53, 53, 53, 53, 52,
};

/* Private functions ---------------------------------------------------------*/

/**
  * @brief  Main program
  * @param  None
  * @retval None
  */

int bb_squares(bb value, int squares[64]) {
    int i = 0;
    int sq;
    while (value) {
        POP_LSB(sq, value);
        squares[i++] = sq;
    }
    return i;
}

bb bb_slide(int sq, int truncate, bb obstacles, int directions[4][2]) {
    bb value = 0;
    int rank = sq / 8;
    int file = sq % 8;
    for (int i = 0; i < 4; i++) {
        bb previous = 0;
        for (int n = 1; n < 9; n++) {
            int r = rank + directions[i][0] * n;
            int f = file + directions[i][1] * n;
            if (r < 0 || f < 0 || r > 7 || f > 7) {
                if (truncate) {
                    value &= ~previous;
                }
                break;
            }
            bb bit = BIT(RF(r, f));
            value |= bit;
            if (bit & obstacles) {
                break;
            }
            previous = bit;
        }
    }
    return value;
}

bb bb_slide_rook(int sq, int truncate, bb obstacles) {
    int directions[4][2] = {
        {-1, 0}, {1, 0}, {0, -1}, {0, 1}
    };
    return bb_slide(sq, truncate, obstacles, directions);
}

int main(void)
{
  /* STM32F4xx HAL library initialization:
       - Configure the Flash prefetch, instruction and Data caches
       - Configure the Systick to generate an interrupt each 1 msec
       - Set NVIC Group Priority to 4
       - Global MSP (MCU Support Package) initialization
     */
  HAL_Init();

  /* Configure the system clock to 168 MHz */
  SystemClock_Config();

  /* Unlock the Flash to enable the flash control register access *************/
  HAL_FLASH_Unlock();

//  /* Erase the user Flash area
//    (area defined by FLASH_USER_START_ADDR and FLASH_USER_END_ADDR) ***********/
//
//  /* Get the 1st sector to erase */
//  FirstSector = GetSector(FLASH_USER_START_ADDR);
//  /* Get the number of sector to erase from 1st sector*/
//  NbOfSectors = GetSector(FLASH_USER_END_ADDR) - FirstSector + 1;
//
//  /* Fill EraseInit structure*/
//  EraseInitStruct.TypeErase = FLASH_TYPEERASE_SECTORS;
//  EraseInitStruct.VoltageRange = FLASH_VOLTAGE_RANGE_3;
//  EraseInitStruct.Sector = FirstSector;
//  EraseInitStruct.NbSectors = NbOfSectors;
//  if(HAL_FLASHEx_Erase(&EraseInitStruct, &SectorError) != HAL_OK)
//  {
//    /*
//      Error occurred while sector erase.
//      User can add here some code to deal with this error.
//      SectorError will contain the faulty sector and then to know the code error on this sector,
//      user can call function 'HAL_FLASH_GetError()'
//    */
//    while (1)
//    {
//    }
//  }
//
//  /* Note: If an erase operation in Flash memory also concerns data in the data or instruction cache,
//     you have to make sure that these data are rewritten before they are accessed during code
//     execution. If this cannot be done safely, it is recommended to flush the caches by setting the
//     DCRST and ICRST bits in the FLASH_CR register. */
//  __HAL_FLASH_DATA_CACHE_DISABLE();
//  __HAL_FLASH_INSTRUCTION_CACHE_DISABLE();
//
//  __HAL_FLASH_DATA_CACHE_RESET();
//  __HAL_FLASH_INSTRUCTION_CACHE_RESET();
//
//  __HAL_FLASH_INSTRUCTION_CACHE_ENABLE();
//  __HAL_FLASH_DATA_CACHE_ENABLE();
//
//  /* Program the user Flash area word by word
//    (area defined by FLASH_USER_START_ADDR and FLASH_USER_END_ADDR) ***********/
//
//  // ATTACK_ROOK
//  for (int sq = 0; sq < 64; sq++) {
//      BB_ROOK_6[sq] = bb_slide_rook(sq, 1, 0L);
//  }
//  int offset = 0;
//  int squares[64];
//  for (int sq = 0; sq < 64; sq++) {
//      int count = bb_squares(BB_ROOK_6[sq], squares);
//      int n = 1 << count;
//      for (int i = 0; i < n; i++) {
//          bb obstacles = 0;
//          for (int j = 0; j < count; j++) {
//              if (i & (1 << j)) {
//                  obstacles |= BIT(squares[j]);
//              }
//          }
//          bb value = bb_slide_rook(sq, 0, obstacles);
//          int index = (obstacles * MAGIC_ROOK[sq]) >> SHIFT_ROOK[sq];
//          bb previous = *(__IO uint64_t*)(FLASH_USER_START_ADDR + ((offset + index) * 8));
////            bb previous = ATTACK_ROOK[offset + index];
//          if (previous != 0xffffffffffffffff && previous != value) {
//              while(1);
//          }
//          HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, FLASH_USER_START_ADDR + ((offset + index) * 8), (uint32_t)value);
//          HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, FLASH_USER_START_ADDR + ((offset + index) * 8) + 4, (uint32_t)(value >> 32));
////            ATTACK_ROOK[offset + index] = value;
//      }
//      offset += 1 << (64 - SHIFT_ROOK[sq]);
//  }

  /* Lock the Flash to disable the flash control register access (recommended
     to protect the FLASH memory against possible unwanted operation) *********/
  HAL_FLASH_Lock();

  /* Check if the programmed data is OK
      MemoryProgramStatus = 0: data programmed correctly
      MemoryProgramStatus != 0: number of words not programmed correctly ******/
  Address = FLASH_USER_START_ADDR;
  MemoryProgramStatus = 0;

  while (Address < FLASH_USER_END_ADDR)
  {
    data32 = *(__IO uint64_t*)Address;

    Address = Address + 8;
  }

  /* Infinite loop */
  while (1)
  {
  }
}

/**
  * @brief  Gets the sector of a given address
  * @param  None
  * @retval The sector of a given address
  */
static uint32_t GetSector(uint32_t Address)
{
  uint32_t sector = 0;

  if((Address < ADDR_FLASH_SECTOR_1) && (Address >= ADDR_FLASH_SECTOR_0))
  {
    sector = FLASH_SECTOR_0;
  }
  else if((Address < ADDR_FLASH_SECTOR_2) && (Address >= ADDR_FLASH_SECTOR_1))
  {
    sector = FLASH_SECTOR_1;
  }
  else if((Address < ADDR_FLASH_SECTOR_3) && (Address >= ADDR_FLASH_SECTOR_2))
  {
    sector = FLASH_SECTOR_2;
  }
  else if((Address < ADDR_FLASH_SECTOR_4) && (Address >= ADDR_FLASH_SECTOR_3))
  {
    sector = FLASH_SECTOR_3;
  }
  else if((Address < ADDR_FLASH_SECTOR_5) && (Address >= ADDR_FLASH_SECTOR_4))
  {
    sector = FLASH_SECTOR_4;
  }
  else if((Address < ADDR_FLASH_SECTOR_6) && (Address >= ADDR_FLASH_SECTOR_5))
  {
    sector = FLASH_SECTOR_5;
  }
  else if((Address < ADDR_FLASH_SECTOR_7) && (Address >= ADDR_FLASH_SECTOR_6))
  {
    sector = FLASH_SECTOR_6;
  }
  else if((Address < ADDR_FLASH_SECTOR_8) && (Address >= ADDR_FLASH_SECTOR_7))
  {
    sector = FLASH_SECTOR_7;
  }
  else if((Address < ADDR_FLASH_SECTOR_9) && (Address >= ADDR_FLASH_SECTOR_8))
  {
    sector = FLASH_SECTOR_8;
  }
  else if((Address < ADDR_FLASH_SECTOR_10) && (Address >= ADDR_FLASH_SECTOR_9))
  {
    sector = FLASH_SECTOR_9;
  }
  else if((Address < ADDR_FLASH_SECTOR_11) && (Address >= ADDR_FLASH_SECTOR_10))
  {
    sector = FLASH_SECTOR_10;
  }
  else /* (Address < FLASH_END_ADDR) && (Address >= ADDR_FLASH_SECTOR_11) */
  {
    sector = FLASH_SECTOR_11;
  }

  return sector;
}

/**
  * @brief  Gets sector Size
  * @param  None
  * @retval The size of a given sector
  */
static uint32_t GetSectorSize(uint32_t Sector)
{
  uint32_t sectorsize = 0x00;

  if((Sector == FLASH_SECTOR_0) || (Sector == FLASH_SECTOR_1) || (Sector == FLASH_SECTOR_2) || (Sector == FLASH_SECTOR_3))
  {
    sectorsize = 16 * 1024;
  }
  else if(Sector == FLASH_SECTOR_4)
  {
    sectorsize = 64 * 1024;
  }
  else
  {
    sectorsize = 128 * 1024;
  }
  return sectorsize;
}

/**
  * @brief  System Clock Configuration
  *         The system Clock is configured as follow :
  *            System Clock source            = PLL (HSE)
  *            SYSCLK(Hz)                     = 168000000
  *            HCLK(Hz)                       = 168000000
  *            AHB Prescaler                  = 1
  *            APB1 Prescaler                 = 4
  *            APB2 Prescaler                 = 2
  *            HSE Frequency(Hz)              = 25000000
  *            PLL_M                          = 25
  *            PLL_N                          = 336
  *            PLL_P                          = 2
  *            PLL_Q                          = 7
  *            VDD(V)                         = 3.3
  *            Main regulator output voltage  = Scale1 mode
  *            Flash Latency(WS)              = 5
  * @param  None
  * @retval None
  */
static void SystemClock_Config(void)
{
  RCC_ClkInitTypeDef RCC_ClkInitStruct;
  RCC_OscInitTypeDef RCC_OscInitStruct;

  /* Enable Power Control clock */
  __HAL_RCC_PWR_CLK_ENABLE();

  /* The voltage scaling allows optimizing the power consumption when the device is
     clocked below the maximum system frequency, to update the voltage scaling value
     regarding system frequency refer to product datasheet.  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /* Enable HSE Oscillator and activate PLL with HSE as source */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 25;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 7;
  HAL_RCC_OscConfig(&RCC_OscInitStruct);

  /* Select PLL as system clock source and configure the HCLK, PCLK1 and PCLK2
     clocks dividers */
  RCC_ClkInitStruct.ClockType = (RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2);
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;
  HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5);

  /* STM32F405x/407x/415x/417x Revision Z devices: prefetch is supported  */
  if (HAL_GetREVID() == 0x1001)
  {
    /* Enable the Flash prefetch */
    __HAL_FLASH_PREFETCH_BUFFER_ENABLE();
  }
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t* file, uint32_t line)
{
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */

  /* Infinite loop */
  while (1)
  {
  }
}
#endif

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
