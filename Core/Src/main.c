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
#include "fatfs.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "RFM95.h"
#include "fatfs_sd.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
#ifdef __GNUC__
  #define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
#else
  #define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)
#endif /* __GNUC__ */
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
RTC_HandleTypeDef hrtc;

SPI_HandleTypeDef hspi1;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */
PUTCHAR_PROTOTYPE
{
	HAL_UART_Transmit(&huart1, (uint8_t *)&ch, 1, 0xFFFF);
	return ch;
}

//-----LoRa-----
RFM95_t rfm95;
bool txFlag;
bool rxFlag;
uint8_t rxBuff[16];
int id = 0;

//-----SD CARD-----
FATFS fs; // file system
FIL file; // file
FRESULT fresult; // to store the result
char buffer[1024]; // to store data
UINT br, bw;
FATFS *pfs;
DWORD fre_clust;
uint32_t total, free_space;
DIR dir; // Directory
FILINFO fno; // File info

//-----RTC-----
RTC_DateTypeDef gDate;
RTC_TimeTypeDef gTime;

//-----Timer-----
uint8_t timerStartFlag = 0;
uint8_t timer2Flag = 0;
uint8_t timer3Flag = 0;
//-----Command Line-----
typedef struct
{
	uint8_t temp[30];
	uint8_t cmd[10];
	uint8_t param[30];
	uint8_t cmdLen;
	uint8_t paramLen;
}rxCmd;
rxCmd myRxCmd;
uint8_t pos = 0;
uint8_t rxData = 0;
volatile uint8_t uartFlag = 0, cmdFlag = 0;
/* sf_10_cr45: 2500 ms
 * sf_9_cr45: 2247 ms
 * sf_8_cr45: 2134 ms
 * sf_7_cr45: 2070 ms
 * sf_10_cr48: 2592 ms
 * sf_9_cr48: 2298 ms
 * sf_8_cr48: 2163 ms
 * sf_7_cr48: 2090 ms */
uint32_t cr_sf_array[4][4] = {{2070, 2134, 2247, 2500},
							{2077, 2143, 2264, 2530},
							{2083, 2153, 2281, 2561},
							{2090, 2163, 2298, 2592}};
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_SPI1_Init(void);
static void MX_TIM1_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_RTC_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef* htim);
void init_Date_Time(void);
void get_Date_Time(void);
void SD_init(void);
void Cmd_display(void);
bool STM32_DIO0(void);
void STM32_NRST(bool val);
void STM32_NSEL(bool val);
void STM32_DelayUs(uint32_t delay);
uint8_t STM32_SPI_Write(uint8_t* pTxData, uint8_t dataLen, uint32_t timeout);
uint8_t STM32_SPI_Read(uint8_t* pRxData, uint8_t dataLen, uint32_t timeout);
uint8_t STM32_SPI_WriteRead(uint8_t* pTxData, uint8_t* pRxData, uint8_t dataLen, uint32_t timeout);
uint8_t STM32_SPI_CheckState(void);

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	// The user push button on your black pill board is connected between PA0 and GND.
	if(GPIO_Pin == BTN_Pin)
	{
		txFlag = true;
	}
	if(GPIO_Pin == DIO0_Pin)
	{
		rxFlag = true;
	}
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	if (huart->Instance == USART1)
	{
		/* Enabling interrupt receive again */
		HAL_UART_Receive_IT(&huart1,(uint8_t*)&rxData,1);
		switch(rxData)
		{
		case ' ':
			// clear array
			if(myRxCmd.cmdLen)
				memset(myRxCmd.cmd, '\0', myRxCmd.cmdLen);
			memcpy(myRxCmd.cmd, myRxCmd.temp, pos + 1);
			memset(myRxCmd.temp, '\0', pos + 1);
			myRxCmd.cmdLen = pos + 1;
			pos = 0;
			uartFlag = 1;
			printf(" ");
			break;
		case '\r':
			if(uartFlag)
			{
				// clear array
				if(myRxCmd.paramLen)
					memset(myRxCmd.param, '\0', myRxCmd.paramLen);
				memcpy(myRxCmd.param, myRxCmd.temp, pos + 1);
				myRxCmd.paramLen = pos + 1;
			}
			else
			{
				// clear array
				if(myRxCmd.cmdLen)
					memset(myRxCmd.cmd, '\0', myRxCmd.cmdLen);
				memcpy(myRxCmd.cmd, myRxCmd.temp, pos + 1);
				myRxCmd.cmdLen = pos + 1;
			}
			memset(myRxCmd.temp, '\0', pos + 1);
			pos = 0;
			uartFlag = 0;
			/* Start execute command */
			cmdFlag = 1;
			printf("\r\n");
			break;
		default:
			myRxCmd.temp[pos] = rxData;
			pos++;
			printf("%c", rxData);
		}
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
  MX_SPI1_Init();
  MX_TIM1_Init();
  MX_USART1_UART_Init();
  MX_FATFS_Init();
  MX_RTC_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  /* USER CODE BEGIN 2 */
	HAL_TIM_Base_Start(&htim1);
	/* Enabling interrupt receive again */
	HAL_UART_Receive_IT(&huart1,(uint8_t*)&rxData,1);
//	__HAL_UART_ENABLE_IT(&huart1,UART_IT_RXNE);
	/* LoRa initialization */
	rfm95.DIO.DIO0 = STM32_DIO0;
	rfm95.NRST = STM32_NRST;
	rfm95.NSEL = STM32_NSEL;
	rfm95.DelayUs = STM32_DelayUs;
	rfm95.SPI_Write = STM32_SPI_Write;
	rfm95.SPI_Read = STM32_SPI_Read;
	rfm95.SPI_WriteRead = STM32_SPI_WriteRead;
	rfm95.SPI_CheckState = STM32_SPI_CheckState;
	rfm95.spi_ok = (uint8_t)HAL_SPI_STATE_READY;

	uint8_t ret = RFM95_LoRa_Init(&rfm95);
	if(ret != RFM95_OK)
	{
	  printf("Init... fail!\r\n");
	  printf("Error code: %d\r\n", ret);
	}
	else
	{
	  printf("Init... success!\r\n");
	}
	RFM95_LoRa_setSyncWord(&rfm95, 0x34);

	HAL_Delay(500);
	/* Initialize the SD card, verify file creation, updating and deletion of file. */
	SD_init();
	HAL_Delay(500);
	/* Display operating commands */
	Cmd_display();

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  if(cmdFlag)
	  {
		  if(strncmp("mk", (char*)myRxCmd.cmd, myRxCmd.cmdLen) == 0)
		  {
			  if(myRxCmd.paramLen == 0)
			  {
				  printf("File name is invalid !\r\n");
			  }
			  else
			  {
				  uint8_t filename[20];
				  memcpy(filename, myRxCmd.param, myRxCmd.paramLen);
				  /* Creating/Reading a file */
				  fresult = f_open(&file, (char*)filename, FA_OPEN_ALWAYS | FA_READ | FA_WRITE);
				  if(fresult == FR_OK) printf("Create file %s successfully !\r\n", filename);
				  /* Close file */
				  fresult = f_close(&file);
				  if(fresult == FR_OK) printf("Close file !\r\n");
			  }
			  memset(myRxCmd.cmd, '\0', myRxCmd.cmdLen);
			  myRxCmd.cmdLen = 0;
			  memset(myRxCmd.param, '\0', myRxCmd.paramLen);
			  myRxCmd.paramLen = 0;
			  cmdFlag = 0;
		  }
		  else if(strncmp("del", (char*)myRxCmd.cmd, myRxCmd.cmdLen) == 0)
		  {
			  if(myRxCmd.paramLen == 0)
			  {
				  printf("File name is invalid !\r\n");
			  }
			  else
			  {
				  /* Remove files */
				  fresult = f_unlink((char*)myRxCmd.param);
				  if(fresult == FR_OK)
					  printf("%s removed successfully !\r\n", myRxCmd.param);
				  else if(fresult == FR_NO_FILE)
					  printf("%s is not found !\r\n", myRxCmd.param);
			  }
			  memset(myRxCmd.cmd, '\0', myRxCmd.cmdLen);
			  myRxCmd.cmdLen = 0;
			  memset(myRxCmd.param, '\0', myRxCmd.paramLen);
			  myRxCmd.paramLen = 0;
			  cmdFlag = 0;
		  }
		  else if(strncmp("read", (char*)myRxCmd.cmd, myRxCmd.cmdLen) == 0)
		  {
			  if(myRxCmd.paramLen == 0)
			  {
				  printf("File name is invalid !\r\n");
			  }
			  else
			  {
				  uint8_t readBuff[10240] = {0};
				  /* Open file to read */
				  fresult = f_open(&file, (char*)myRxCmd.param, FA_READ);
				  /* Read string from the file */
				  f_read(&file, readBuff, f_size(&file), &br);
				  printf("Read %s :\r\n", myRxCmd.param);
				  printf("%s\r\n", readBuff);
				  /* Close file */
				  fresult = f_close(&file);
				  if(fresult == FR_OK) printf("Close file !\r\n");
			  }
			  memset(myRxCmd.cmd, '\0', myRxCmd.cmdLen);
			  myRxCmd.cmdLen = 0;
			  memset(myRxCmd.param, '\0', myRxCmd.paramLen);
			  myRxCmd.paramLen = 0;
			  cmdFlag = 0;
		  }
		  else if(strncmp("ls", (char*)myRxCmd.cmd, myRxCmd.cmdLen) == 0)
		  {
			  f_opendir(&dir, "/");
			  do{
				  f_readdir(&dir, &fno);
				  if(fno.fname[0] != 0)
					  printf("File found: %s \r\n", fno.fname);
			  }while(fno.fname[0]);
			  f_closedir(&dir);
			  memset(myRxCmd.cmd, '\0', myRxCmd.cmdLen);
			  myRxCmd.cmdLen = 0;
			  cmdFlag = 0;
		  }
		  else if(strncmp("rtc", (char*)myRxCmd.cmd, myRxCmd.cmdLen) == 0)
		  {
			  if(strncmp("on", (char*)myRxCmd.param, myRxCmd.paramLen) == 0)
			  {
				  get_Date_Time();
				  printf("%02d-%02d-%04d\r\n", gDate.Date, gDate.Month, 2000 + gDate.Year);
				  printf("%02d:%02d:%02d\r\n", gTime.Hours, gTime.Minutes, gTime.Seconds);
				  HAL_Delay(5000);
			  }
			  else if(strncmp("off", (char*)myRxCmd.param, myRxCmd.paramLen) == 0)
			  {
				  memset(myRxCmd.cmd, '\0', myRxCmd.cmdLen);
				  myRxCmd.cmdLen = 0;
				  memset(myRxCmd.param, '\0', myRxCmd.paramLen);
				  myRxCmd.paramLen = 0;
				  cmdFlag = 0;
			  }
			  else
			  {
				  printf("Invalid parameter !\r\n");
			  }
		  }
		  else if(strncmp("ready", (char*)myRxCmd.cmd, myRxCmd.cmdLen) == 0)
		  {
			  ret = RFM95_LoRa_prepareReceive(&rfm95, true);
			  if(ret != RFM95_OK)
			  {
				  printf("Rx init.. fail !\r\n");
				  printf("Error code: %d\r\n", ret);
			  }
			  else
			  {
				  printf("Rx init.. success !\r\n");
			  }
			  memset(myRxCmd.cmd, '\0', myRxCmd.cmdLen);
			  myRxCmd.cmdLen = 0;
			  cmdFlag = 0;
			  /* able to initialize timer interrupt once. */
			  timerStartFlag = 1;
		  }
		  else if(strncmp("stop", (char*)myRxCmd.cmd, myRxCmd.cmdLen) == 0)
		  {
			  ret = RFM95_LoRa_setOpMode(&rfm95, SLEEP_MODE);
			  if(ret == RFM95_OK)
				  printf("Exit rx mode !\r\n");
			  memset(myRxCmd.cmd, '\0', myRxCmd.cmdLen);
			  myRxCmd.cmdLen = 0;
			  cmdFlag = 0;
			  id = 0;
			  /* restart the timer */
			  htim2.Instance->CNT &= 0x0;
			  HAL_TIM_Base_Stop_IT(&htim2);
			  htim3.Instance->CNT &= 0x0;
			  HAL_TIM_Base_Stop_IT(&htim3);
		  }
		  else if(strncmp("rx", (char*)myRxCmd.cmd, myRxCmd.cmdLen) == 0)
		  {
			  if(myRxCmd.paramLen == 0)
			  {
				  printf("Invalid parameter !\r\n");
			  }
			  else
			  {
				  /* Timer interrupt and RTC time recorder startup to alignment time */
				  if(timerStartFlag)
				  {
					  /* restart the timer */
					  htim2.Instance->CNT &= 0x0;
					  HAL_TIM_Base_Start_IT(&htim2);
					  /* initialize RTC to 00:00 */
					  init_Date_Time();
					  /* According to the parameter, define counter period value (AutoReload Register). */
					  uint8_t row,col; // row: cr, column: sf
					  row = (rfm95.Settings.LoRa.codingRate >> 1) - 1;
					  col = rfm95.Settings.LoRa.spreadingFactor - 7;
					  printf("Matrix[%d][%d]\r\n", row, col);
					  htim3.Instance->ARR = cr_sf_array[row][col] * 10 - 1;
					  timerStartFlag = 0;
				  }
				  /* If it exceeded threshold, triggering Timer interrupt */
				  if(timer2Flag | timer3Flag)
				  {
					  uint8_t buff[50] = {0};
					  /* Receive data fail */
					  /* Update time */
					  get_Date_Time();
					  printf("Rx didn't receive the predicted signal !\r\n");
					  /* id, status, mm:ss.ooo */
					  int milisec = (1.0f - (float)gTime.SubSeconds / (float)gTime.SecondFraction) * 1000;
					  if(milisec >= 1000) milisec = 999;
					  sprintf((char*)buff, "%d,%d,%02d:%02d.%03d\n", id++, -1, gTime.Minutes, gTime.Seconds, milisec);
					  printf("Current time: %02d:%02d.%03d \r\n", gTime.Minutes, gTime.Seconds, milisec);
					  /* Updating an existing file */
					  fresult = f_open(&file, (char*)myRxCmd.param, FA_OPEN_ALWAYS | FA_WRITE);
					  if(fresult == FR_OK) printf("%s opened successfully !\r\n", myRxCmd.param);
					  /* Move to offset to the end to the file */
					  fresult = f_lseek(&file, f_size(&file));
					  /* Writing text */
					  fresult = f_puts((char*)buff, &file);
					  /* Close file */
					  fresult = f_close(&file);
					  if(fresult == FR_OK) printf("Close file !\r\n");

					  timer2Flag = 0;
					  HAL_TIM_Base_Stop_IT(&htim2);
					  timer3Flag = 0;

					  htim3.Instance->CNT &= 0x0;
					  HAL_TIM_Base_Start_IT(&htim3);
				  }
				  // DIO0 interrupt
				  if(rxFlag)
				  {
					  /* Refresh Timer2 Interrupt */
					  htim2.Instance->CNT &= 0x0;
					  HAL_TIM_Base_Start_IT(&htim2);
					  /* Stop Timer3 Interrupt */
					  htim3.Instance->CNT &= 0x0;
					  HAL_TIM_Base_Stop_IT(&htim3);

					  ret = RFM95_LoRa_receive(&rfm95, rxBuff, 16);
					  if(ret != RFM95_ERR_RX_FAIL)
					  {
						  int8_t crc = 0;
						  uint8_t buff[50] = {0};
						  if(ret == RFM95_ERR_RX_PAYLOAD_CRC)
						  {
							  crc = 0;
							  printf("Rx.. crc error !\r\n");
						  }
						  else if(ret == RFM95_OK)
						  {
							  crc = 1;
							  printf("Rx.. success!\r\n");
						  }
						  int16_t rssi = RFM95_getPcktRSSI(&rfm95);
						  float snr = RFM95_getPcktSNR(&rfm95);
						  printf("RSSI: %d, SNR: %f\r\n", rssi, snr);
						  for(int i = 0; i<16;i++)
						  {
							  printf("%02x ", rxBuff[i]);
						  }
						  printf("\r\n");

						  /* Update time */
						  get_Date_Time();
						  /* id, status, mm:ss.ooo */
						  int milisec = (1.0f - (float)gTime.SubSeconds / (float)gTime.SecondFraction) * 1000;
						  if(milisec >= 1000) milisec = 999;
						  sprintf((char*)buff, "%d,%d,%02d:%02d.%03d,%d,%.2f\n", id++, crc, gTime.Minutes, gTime.Seconds, milisec, rssi, snr);
						  printf("Current time: %02d:%02d.%03d \r\n", gTime.Minutes, gTime.Seconds, milisec);
						  /* Updating an existing file */
						  fresult = f_open(&file, (char*)myRxCmd.param, FA_OPEN_ALWAYS | FA_WRITE);
						  if(fresult == FR_OK) printf("%s opened successfully !\r\n", myRxCmd.param);
						  /* Move to offset to the end to the file */
						  fresult = f_lseek(&file, f_size(&file));
						  /* Writing text */
						  fresult = f_puts((char*)buff, &file);
						  /* Close file */
						  f_close(&file);
						  if(fresult == FR_OK) printf("Close file !\r\n");
					  }
					  else
					  {
						  printf("Rx.. fail!\r\n");
						  printf("ERROR CODE: %d\r\n", ret);
					  }
					  rxFlag = false;
				  }
			  }
		  }
		  else if(strncmp("sf", (char*)myRxCmd.cmd, myRxCmd.cmdLen) == 0)
		  {
			  int sf = atoi((char*)myRxCmd.param);
			  if(sf >= 7 && sf <= 12 )
			  {
				  uint8_t ret;
				  ret = RFM95_LoRa_setSpreadingFactor(&rfm95, sf);
				  if(ret) printf("ERROR CODE: %d\r\n", ret);
				  else printf("SF %d is set.\r\n", sf);
			  }
			  else
			  {
				  printf("Invalid parameter !\r\n");
			  }
			  memset(myRxCmd.cmd, '\0', myRxCmd.cmdLen);
			  myRxCmd.cmdLen = 0;
			  memset(myRxCmd.param, '\0', myRxCmd.paramLen);
			  myRxCmd.paramLen = 0;
			  cmdFlag = 0;
		  }
		  else if(strncmp("bw", (char*)myRxCmd.cmd, myRxCmd.cmdLen) == 0)
		  {
			  int bw = atoi((char*)myRxCmd.param);
			  if(bw >= 7 && bw <=500)
			  {
				  lora_bw lbw;
				  if(bw <= 8)
					  lbw = BW_7_8_kHz;
				  else if(bw <= 11)
					  lbw = BW_10_4_kHz;
				  else if(bw <= 16)
					  lbw = BW_15_6_kHz;
				  else if(bw <= 21)
					  lbw = BW_20_8_kHz;
				  else if(bw <= 32)
					  lbw = BW_31_25_kHz;
				  else if(bw <= 42)
					  lbw = BW_41_7_kHz;
				  else if(bw <= 63)
					  lbw = BW_62_5_kHz;
				  else if(bw <= 125)
					  lbw = BW_125_kHz;
				  else if(bw <= 250)
					  lbw = BW_250_kHz;
				  else if(bw <= 500)
					  lbw = BW_500_kHz;

				  uint8_t ret = RFM95_LoRa_setBandwidth(&rfm95, lbw);
				  if(ret) printf("ERROR CODE: %d\r\n", ret);
				  else printf("BW %d is set.\r\n", bw);
			  }
			  else
			  {
				  printf("Invalid parameter !\r\n");
			  }
			  memset(myRxCmd.cmd, '\0', myRxCmd.cmdLen);
			  myRxCmd.cmdLen = 0;
			  memset(myRxCmd.param, '\0', myRxCmd.paramLen);
			  myRxCmd.paramLen = 0;
			  cmdFlag = 0;
		  }
		  else if(strncmp("cr", (char*)myRxCmd.cmd, myRxCmd.cmdLen) == 0)
		  {
			  int cr = atoi((char*)myRxCmd.param);
			  if(cr >= 1 && cr <=4)
			  {
				  uint8_t ret = RFM95_LoRa_setCodingRate(&rfm95, cr*2);
				  if(ret) printf("ERROR CODE: %d\r\n", ret);
				  else printf("CR 4/%d is set.\r\n", cr + 4);
			  }
			  else
			  {
				  printf("Invalid parameter !\r\n");
			  }
			  memset(myRxCmd.cmd, '\0', myRxCmd.cmdLen);
			  myRxCmd.cmdLen = 0;
			  memset(myRxCmd.param, '\0', myRxCmd.paramLen);
			  myRxCmd.paramLen = 0;
			  cmdFlag = 0;
		  }
		  else if(strncmp("freq", (char*)myRxCmd.cmd, myRxCmd.cmdLen) == 0)
		  {
			  int kHz = atoi((char*)myRxCmd.param);
			  if(kHz >= 902000 && kHz <= 928000)
			  {
				  uint8_t ret = RFM95_setFrequency(&rfm95, kHz*1000);
				  if(ret) printf("ERROR CODE: %d\r\n", ret);
				  else printf("Frequency %d is set.\r\n", kHz*1000);
			  }
			  else
			  {
				  printf("Invalid parameter !\r\n");
			  }
			  memset(myRxCmd.cmd, '\0', myRxCmd.cmdLen);
			  myRxCmd.cmdLen = 0;
			  memset(myRxCmd.param, '\0', myRxCmd.paramLen);
			  myRxCmd.paramLen = 0;
			  cmdFlag = 0;
		  }
		  else if(strncmp("mount", (char*)myRxCmd.cmd, myRxCmd.cmdLen) == 0)
		  {
			  fresult = f_mount(&fs, "", 0);
			  if(fresult != FR_OK)
				  printf("error in mounting SD CARD... \r\n");
			  else
				  printf("SD CARD mounted successfully... \r\n");
			  /* Check free space */
			  f_getfree("", &fre_clust, &pfs);

			  total = (uint32_t)((pfs->n_fatent - 2) * pfs->csize * 0.5);
			  printf("SD CARD total size: \t%lu\r\n", total);
			  free_space = (uint32_t)(fre_clust * pfs->csize * 0.5);
			  printf("SD CARD free space: \t%lu\r\n", free_space);

			  memset(myRxCmd.cmd, '\0', myRxCmd.cmdLen);
			  myRxCmd.cmdLen = 0;
			  memset(myRxCmd.param, '\0', myRxCmd.paramLen);
			  myRxCmd.paramLen = 0;
			  cmdFlag = 0;
		  }
		  else
		  {
			  printf("Invalid command !\r\n");
			  cmdFlag = 0;
		  }
	  }
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
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_LSE;
  RCC_OscInitStruct.LSEState = RCC_LSE_ON;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 100;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
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

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief RTC Initialization Function
  * @param None
  * @retval None
  */
static void MX_RTC_Init(void)
{

  /* USER CODE BEGIN RTC_Init 0 */

  /* USER CODE END RTC_Init 0 */

  RTC_TimeTypeDef sTime = {0};
  RTC_DateTypeDef sDate = {0};

  /* USER CODE BEGIN RTC_Init 1 */

  /* USER CODE END RTC_Init 1 */

  /** Initialize RTC Only
  */
  hrtc.Instance = RTC;
  hrtc.Init.HourFormat = RTC_HOURFORMAT_24;
  hrtc.Init.AsynchPrediv = 127;
  hrtc.Init.SynchPrediv = 255;
  hrtc.Init.OutPut = RTC_OUTPUT_DISABLE;
  hrtc.Init.OutPutPolarity = RTC_OUTPUT_POLARITY_HIGH;
  hrtc.Init.OutPutType = RTC_OUTPUT_TYPE_OPENDRAIN;
  if (HAL_RTC_Init(&hrtc) != HAL_OK)
  {
    Error_Handler();
  }

  /* USER CODE BEGIN Check_RTC_BKUP */

  /* USER CODE END Check_RTC_BKUP */

  /** Initialize RTC and set the Time and Date
  */
  sTime.Hours = 0x21;
  sTime.Minutes = 0x50;
  sTime.Seconds = 0x0;
  sTime.DayLightSaving = RTC_DAYLIGHTSAVING_NONE;
  sTime.StoreOperation = RTC_STOREOPERATION_RESET;
  if (HAL_RTC_SetTime(&hrtc, &sTime, RTC_FORMAT_BCD) != HAL_OK)
  {
    Error_Handler();
  }
  sDate.WeekDay = RTC_WEEKDAY_SATURDAY;
  sDate.Month = RTC_MONTH_FEBRUARY;
  sDate.Date = 0x24;
  sDate.Year = 0x24;

  if (HAL_RTC_SetDate(&hrtc, &sDate, RTC_FORMAT_BCD) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN RTC_Init 2 */

  /* USER CODE END RTC_Init 2 */

}

/**
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_64;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

}

/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 100-1;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 65535;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 100-1;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 3000000-1;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 10000-1;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 25000-1;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */

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
  HAL_GPIO_WritePin(BLUE_GPIO_Port, BLUE_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, NSS_Pin|RST_Pin|SD_CS_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin : BLUE_Pin */
  GPIO_InitStruct.Pin = BLUE_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(BLUE_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : BTN_Pin */
  GPIO_InitStruct.Pin = BTN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(BTN_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : DIO0_Pin */
  GPIO_InitStruct.Pin = DIO0_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(DIO0_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : NSS_Pin RST_Pin SD_CS_Pin */
  GPIO_InitStruct.Pin = NSS_Pin|RST_Pin|SD_CS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI0_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(EXTI0_IRQn);

  HAL_NVIC_SetPriority(EXTI4_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(EXTI4_IRQn);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
/* Trigger Timer Interrupt per 3 seconds */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef* htim)
{
	if(htim == &htim2)
	{
		timer2Flag = 1;
	}
	if(htim == &htim3)
	{
		timer3Flag = 1;
	}
}

void Cmd_display(void)
{
	printf("Create file -----------------> mk    [filename] \r\n");
	printf("Delete file -----------------> del   [filename] \r\n");
	printf("Read file -------------------> read  [filename] \r\n");
	printf("Listing files ---------------> ls \r\n");
	printf("Rx mode turn on -------------> ready \r\n");
	printf("Rx mode turn off ------------> stop \r\n");
	printf("Persist data to a file ------> rx    [filename] \r\n");
	printf("Get current time ------------> rtc   [on/off]\r\n");
	printf("Set Spreading Factor --------> sf    [number] \r\n");
	printf("Set Bandwidth ---------------> bw    [kHz] \r\n");
	printf("Set Coding Rate -------------> cr    [number] \r\n");
	printf("Set Frequency ---------------> freq  [kHz] \r\n");
	printf("Reset SD Card ---------------> mount");
}

void SD_init(void)
{
	/* Mount SD Card */
	fresult = f_mount(&fs, "", 0);
	if(fresult != FR_OK)
		printf("error in mounting SD CARD... \r\n");
	else
		printf("SD CARD mounted successfully... \r\n");
	/* Check free space */
	f_getfree("", &fre_clust, &pfs);

	total = (uint32_t)((pfs->n_fatent - 2) * pfs->csize * 0.5);
	printf("SD CARD total size: \t%lu\r\n", total);
	free_space = (uint32_t)(fre_clust * pfs->csize * 0.5);
	printf("SD CARD free space: \t%lu\r\n", free_space);
	/* Creating/Reading a file */
	fresult = f_open(&file, "test.txt", FA_OPEN_ALWAYS | FA_READ | FA_WRITE);
	/* Writing text */
	fresult = f_puts("First line\n", &file);
	/* Close file */
	fresult = f_close(&file);
	/* Open file to read */
	fresult = f_open(&file, "test.txt", FA_READ);
	/* Read string from the file */
	f_read(&file, buffer, f_size(&file), &br);
	printf("%s\r\n", buffer);
	/* Close file */
	fresult = f_close(&file);

	/* Updating an existing file */
	fresult = f_open(&file, "test.txt", FA_OPEN_ALWAYS | FA_WRITE);
	/* Move to offset to the end to the file */
	fresult = f_lseek(&file, f_size(&file));
	/* Writing text */
	fresult = f_puts("This is updated data and it should be in the end\n", &file);
	f_close(&file);
	/* Open file to read */
	fresult = f_open(&file, "test.txt", FA_READ);
	/* Read string from the file */
	f_read(&file, buffer, f_size(&file), &br);
	printf("%s\r\n", buffer);
	f_close(&file);
	/* Remove files */
	fresult = f_unlink("test.txt");
	if(fresult == FR_OK) printf("test.txt removed successfully\r\n");
}

void init_Date_Time()
{
	gTime.Hours = 0x14;
	gTime.Minutes = 0x00;
	gTime.Seconds = 0x00;
	gDate.Date = 0x26;
	/* Get the RTC current Time */
	HAL_RTC_SetTime(&hrtc, &gTime, RTC_FORMAT_BIN);
	/* Get the RTC current Date */
	HAL_RTC_SetDate(&hrtc, &gDate, RTC_FORMAT_BIN);
}

void get_Date_Time(void)
{
	/* Get the RTC current Time */
	HAL_RTC_GetTime(&hrtc, &gTime, RTC_FORMAT_BIN);
	/* Get the RTC current Date */
	HAL_RTC_GetDate(&hrtc, &gDate, RTC_FORMAT_BIN);
}

bool STM32_DIO0(void)
{
	return (bool)HAL_GPIO_ReadPin(DIO0_GPIO_Port, DIO0_Pin);
}

void STM32_NRST(bool val)
{
	HAL_GPIO_WritePin(RST_GPIO_Port, RST_Pin, (GPIO_PinState)val);
}

void STM32_NSEL(bool val)
{
	HAL_GPIO_WritePin(NSS_GPIO_Port, NSS_Pin, (GPIO_PinState)val);
}

void STM32_DelayUs(uint32_t delay)
{
	__HAL_TIM_SET_COUNTER(&htim1, 0);
	while((__HAL_TIM_GET_COUNTER(&htim1)) < delay);
}

uint8_t STM32_SPI_Write(uint8_t* pTxData, uint8_t dataLen, uint32_t timeout)
{
	return (uint8_t)HAL_SPI_Transmit(&hspi1, pTxData, dataLen, timeout);
}

uint8_t STM32_SPI_Read(uint8_t* pRxData, uint8_t dataLen, uint32_t timeout)
{
	return (uint8_t)HAL_SPI_Receive(&hspi1, pRxData, dataLen, timeout);
}

uint8_t STM32_SPI_WriteRead(uint8_t* pTxData, uint8_t* pRxData, uint8_t dataLen, uint32_t timeout)
{
	uint8_t* rxbuff = (uint8_t*)malloc((dataLen) * sizeof(uint8_t));
	uint8_t ret;
	ret = (uint8_t)HAL_SPI_TransmitReceive(&hspi1, pTxData, rxbuff, dataLen, timeout);
	for(int i = 0; i < dataLen - 1; i++)
	{
		pRxData[i] = rxbuff[i + 1];
	}
	free(rxbuff);
	return ret;
}

uint8_t STM32_SPI_CheckState(void)
{
	return (uint8_t)HAL_SPI_GetState(&hspi1);
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
