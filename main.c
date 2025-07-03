/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include<stdio.h>
#include<string.h>
#include<stdlib.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */




/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
#define MAJOR 0   // BL Major version Number
#define MINOR 1   // BL Minor version Number

#define BOOTLOADER_SIZE        0x80000000  // 64 KB
#define APPLICATION_SLOT_START 0x08010000  // Start of Application Slot
#define FIRMWARE_SLOT_START    0x08040000  // Start of Firmware Slot
#define FIRMWARE_SLOT_SIZE     0x00040000  // 224 KB
#define CHUNK_SIZE             256     // Size of data chunk to be received
#define FLASH_SECTOR_SIZE      0x80000000 // 64 KB size for the sector
#define APPLICATION_SLOT_SIZE  0x08010000 //
#define STORED_CRC_OFFSET    (FIRMWARE_SLOT_START + FLASH_SECTOR_SIZE - 4)
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;
extern CRC_HandleTypeDef hcrc;
/* USER CODE BEGIN PV */
const uint8_t BL_Version[2] = {MAJOR, MINOR};
uint8_t RxBuffer[1024];
int flag=0;
uint8_t ResponseHeader[8];
int chunk_check;
uint8_t SizeHeader[32];
int res;
int myflag = 0;
int size = 0;
char* occurrence;
uint8_t buffer[CHUNK_SIZE];
uint8_t last_buffer[CHUNK_SIZE];/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_USART2_UART_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
static void goto_application(void);

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	flag = 1;
}

uint32_t get_sector_size(uint32_t sector)
{
    if (sector < 4)
    { // Sectors 0-3 are 16KB each
        return 0x4000;
    }

    else if (sector == 4)
    { // Sector 4 is 64KB
        return 0x10000;
    }

    else
    { // Sectors 5 and above are 128KB each
        return 0x20000;
    }
}

uint32_t get_sector_number(uint32_t address)
{
    if (address < 0x08004000)
    { // Sectors 0-3 are 16KB each
        return (address - 0x08000000) / 0x4000;
    }
    else if (address < 0x08008000)
    { // Sector 4 is 64KB
        return 4;
    }
    else
    { // Sectors 5 and above are 128KB each
        return 5 + (address - 0x08008000) / 0x20000;
    }
}

void erase_flash(uint32_t start_address, uint32_t end_address)
{
    /*HAL_FLASH_Unlock();
    FLASH_EraseInitTypeDef eraseInit;
    uint32_t pageError = 0;
eraseInit.TypeErase = FLASH_TYPEERASE_SECTORS;
    eraseInit.Sector = FLASH_SECTOR_6; // Assuming sector 4 onward is used
    eraseInit.NbSectors = 6; // Erase sectors based on your flash organization
    eraseInit.VoltageRange = FLASH_VOLTAGE_RANGE_3;

    HAL_FLASHEx_Erase(&eraseInit, &pageError);
    HAL_FLASH_Lock();*/

    HAL_FLASH_Unlock();
    FLASH_EraseInitTypeDef eraseInit;
    uint32_t sectorError = 0;

    eraseInit.TypeErase = FLASH_TYPEERASE_SECTORS;
    eraseInit.VoltageRange = FLASH_VOLTAGE_RANGE_3;
    eraseInit.NbSectors = 1; // Erase one sector at a time

    uint32_t current_address = start_address;

    while (current_address < end_address) {
        // Calculate the current sector number based on the address
        uint32_t sector = get_sector_number(current_address);
        // Set the sector to erase
                eraseInit.Sector = sector;

                // Perform the erase operation
                if (HAL_FLASHEx_Erase(&eraseInit, &sectorError) != HAL_OK) {
                    // Handle the error
                    printf("Error erasing sector %lu\n", sector);
                    break;
                }

                // Move to the next sector based on the size of the current sector
                current_address += get_sector_size(sector);
            }

            HAL_FLASH_Lock();


        }
uint8_t receive_chunk(uint8_t* buffer , uint32_t length)
{
	static int i=0;
	printf("receiving chunk : %d\r\n", i++);
	chunk_check = HAL_UART_Receive(&huart1, buffer, length , HAL_MAX_DELAY);
	return chunk_check;
}


void write_chunk_to_flash(uint32_t address, uint8_t* data, uint32_t length) {

	HAL_FLASH_Unlock();
    for (uint32_t i = 0; i < length; i++)
    {
        //uint32_t word = *((uint32_t*)(data + i));
        HAL_FLASH_Program(FLASH_TYPEPROGRAM_BYTE, address + i , data[i]);
        printf("Writing chunk to flash[%d/%d]\r\n", i, length);
        //address = address+4;
    }
    HAL_FLASH_Lock();
}

int8_t verify_firmware(void)
{
    // Simple verification for demonstration (e.g., checksum or CRC)
    uint32_t calculated_crc = 0;
    uint32_t firmware_length = FLASH_SECTOR_SIZE - 4;  // exclude stored CRC

    // Reset CRC data register
    HAL_CRC_DeInit(&hcrc);
    HAL_CRC_Init(&hcrc);

    // Compute CRC word by word
    calculated_crc = HAL_CRC_Calculate(&hcrc, (uint32_t *)FIRMWARE_SLOT_START, firmware_length / 4);

    // Read expected CRC from last 4 bytes of firmware
    uint32_t expected_crc = *(uint32_t *)(STORED_CRC_OFFSET);

    
    
    
    

    return (calculated_crc == expected_crc) ? 1 : 0;  // Assuming verification is successful
}

uint8_t receive_firmware(void)
{
    uint32_t flash_address = FIRMWARE_SLOT_START;
    erase_flash(FIRMWARE_SLOT_START, FIRMWARE_SLOT_START + FIRMWARE_SLOT_SIZE); // Example size
    HAL_UART_Transmit(&huart1, (uint8_t *)"START\n", strlen("START\n"), HAL_MAX_DELAY); // sending to the esp
    HAL_UART_Transmit(&huart2, (uint8_t *)"START\r\n", strlen("START\r\n"), HAL_MAX_DELAY); // sending to the terminal for
    //int count = 0;


    uint32_t total_firmware_size = 0;
    uint32_t received_bytes = 0;
    // Step 1: Receive the total firmware size
    if (receive_chunk((uint8_t *)&total_firmware_size, sizeof(total_firmware_size)) != HAL_OK)
    	{
         	 printf("Error receiving firmware size\r\n");
             return 0;
        }
    printf("Total firmware size: %lu bytes\r\n", total_firmware_size);

    uint32_t remaining_bytes = total_firmware_size;
    while (remaining_bytes > 0)
    	{
        	uint32_t bytes_to_receive = (remaining_bytes < CHUNK_SIZE) ? remaining_bytes : CHUNK_SIZE;

            if (receive_chunk(buffer, bytes_to_receive) != HAL_OK)
            {
                printf("Receive chunk error\r\n");
                return 0;
            }

            // Write the current chunk to flash memory
            write_chunk_to_flash(flash_address, buffer, bytes_to_receive);
            flash_address += bytes_to_receive;

            // Update counters
            received_bytes += bytes_to_receive;
            remaining_bytes -= bytes_to_receive;

            memset(buffer, 0, sizeof(buffer));

            if (remaining_bytes == 0)
            	{
                	printf("All firmware data received.\r\n");
                    break;
                }

          }
    if (verify_firmware(FIRMWARE_SLOT_START))
    {
        // Assuming that the application slot needs to be erased before writing
        //erase_flash(APPLICATION_SLOT_START, APPLICATION_SLOT_START + APPLICATION_SLOT_SIZE);

        flash_address = FIRMWARE_SLOT_START;
        uint32_t app_address = APPLICATION_SLOT_START;
        remaining_bytes = total_firmware_size;

        while(remaining_bytes > 0)
        {
            uint32_t bytes_to_copy = (remaining_bytes < CHUNK_SIZE) ? remaining_bytes : CHUNK_SIZE;

            // Copy the data from firmware slot to the application slot
            memset((void*)buffer, 0, sizeof(buffer));
            memcpy(buffer, (uint8_t*)flash_address, bytes_to_copy);
            write_chunk_to_flash(app_address, buffer, bytes_to_copy);

            // Update the addresses and remaining bytes
            flash_address += bytes_to_copy;
            app_address += bytes_to_copy;
            remaining_bytes -= bytes_to_copy;

            //            memset(buffer, 0, sizeof(buffer));
         }

           // Once the copy is complete, jump to the application
             goto_application();
      }

      else
      {
         return 0;
      }

      	  return 1;

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
  MX_USART1_UART_Init();
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */
  // Turn ON the Green Led to tell the user that Bootloader is running
    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12, GPIO_PIN_SET );    //Green LED ON
    printf("Starting Bootloader(%d.%d)\n", BL_Version[0], BL_Version[1] );
    HAL_Delay(2000);   //2sec delay for nothing

    // Jump to application
   // goto_application();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	  if(flag == 1)
	  	{
	  		  //DOWNLOAD THE CODE AND WRITE INTO THE FLASH TO APPLICATION
	  		  HAL_UART_Transmit(&huart1, (uint8_t*)"AT+CHECK_UPDATE\n",strlen("AT+CHECK_UPDATE\n"), HAL_MAX_DELAY);
	  		  HAL_UART_Transmit(&huart2, (uint8_t*)"AT+CHECK_UPDATE\r\n",strlen("AT+CHECK_UPDATE\r\n"), HAL_MAX_DELAY);
	  		  //HAL_Delay(1000);
	  		  memset((void*)ResponseHeader,0, sizeof(ResponseHeader));
	  		  memset((void*)SizeHeader,0, sizeof(SizeHeader));
	  		  HAL_UART_Receive(&huart1, ResponseHeader, sizeof(ResponseHeader), 5000);
	  		  snprintf((char *)SizeHeader, sizeof(SizeHeader), "Response : %s\r\n", ResponseHeader);
	  		  HAL_UART_Transmit(&huart2, SizeHeader,sizeof(SizeHeader), HAL_MAX_DELAY);

	  		 // HAL_UART_Transmit(&huart2, ResponseHeader,sizeof(ResponseHeader), HAL_MAX_DELAY);
	  		  HAL_Delay(1000);
	  		  res = strcmp((char *)ResponseHeader,"YES");
	  		  if(strncmp((char *)ResponseHeader, "YES", 3) == 0)
	  		  {
	  			  receive_firmware();
	  			  flag = 0;
	  		  }
			  else if(strncmp((char *)ResponseHeader, "NO", 2) == 0)
			  {
				  flag = 0;
				  goto_application();
			  }

			  flag = 0;
			  goto_application();
		}
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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 50;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
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

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

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
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12, GPIO_PIN_RESET);

  /*Configure GPIO pin : PE3 */
  GPIO_InitStruct.Pin = GPIO_PIN_3;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pin : PD12 */
  GPIO_InitStruct.Pin = GPIO_PIN_12;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
#ifdef __GNUC__
  /* With GCC, small printf (option LD Linker->Libraries->Small printf
     set to 'Yes') calls __io_putchar() */
int __io_putchar(int ch)
#else
int fputc(int ch, FILE *f)
#endif /* __GNUC__ */
{
  /* Place your implementation of fputc here */
  /* e.g. write a character to the UART3 and Loop until the end of transmission */
  HAL_UART_Transmit(&huart2, (uint8_t *)&ch, 1, HAL_MAX_DELAY);

  return ch;
}

/**
  * @brief Jump to application from the Bootloader
  * @retval None
  */
static void goto_application(void)
{
	printf("Jump to application\r\n");

	// 1. Set the Vector Table Offset Register (VTOR) to the base of the slot
	    SCB->VTOR = APPLICATION_SLOT_START;
	    void (*app_reset_handler)(void) = (void*)(*((volatile uint32_t*) (APPLICATION_SLOT_START + 4U)));

	    // 3. Set the main stack pointer (MSP) to the start value of the slot's vector table
	    __set_MSP(*(__IO uint32_t *)APPLICATION_SLOT_START);

	    app_reset_handler();


//
//	void (*app_reset_handler)(void) = (void*)(*((volatile uint32_t*) (0x08040000 + 4U)));
//
//  // Turn OFF the Green Led to tell the user that Bootloader is not running
//  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_RESET );    //Green LED OFF
//
//  /* Reset the Clock */
//  HAL_RCC_DeInit();
//  HAL_DeInit();
//  __set_MSP(*(volatile uint32_t*) 0x08040000);
//  SysTick->CTRL = 0;
//  SysTick->LOAD = 0;
//  SysTick->VAL = 0;

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
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
