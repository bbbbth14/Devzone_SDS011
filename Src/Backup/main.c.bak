/**
  ******************************************************************************
  * File Name          : main.c
  * Description        : Main program body
  ******************************************************************************
  *
  * COPYRIGHT(c) 2020 STMicroelectronics
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
#include "stm32f1xx_hal.h"

/* USER CODE BEGIN Includes */
#include "bme280.h"
#include "bme280_defs.h"
#include "namepage.h"
#include "SDS011.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

//Define Marcos
#define ESP_CMD_INDEX UINT8_C(4)
#define LCD_CMD_INDEX UINT8_C(2)
#define RTC_DATA_NUM_FORM UINT8_C(2)
#define RTC_DATA_FIELD UINT8_C(7)
#define RTC_DATA_STATUS_BYTE UINT8_C(0)
#define SDS_FRAME (9)
#define LCD_LEVEL_DATA_INDEX (6)
#define DATA_AQI_INDEX (0)
#define ESP_DATA_FIELD (6)
#define ESP_DATA_NUM_FORM (4)
#define MAX_COUNTER_VARIABLES (10)
#define FLAG_RAISING (1)
#define FLAG_FALLING (0)
#define TIME_FORMAT_SECOND (0)
#define TIME_FORMAT_MINUTE (1)
#define TIME_FORMAT_HOUR (2)
#define TIME_FORMAT_DAY (3)
#define FLAG_RAISE (1)
#define FLAG_DEFAULT (0)


/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

RTC_HandleTypeDef hrtc;

SPI_HandleTypeDef hspi1;

TIM_HandleTypeDef htim3;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;
UART_HandleTypeDef huart3;

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/

//Begin variables
UART_HandleTypeDef hua;
struct bme280_dev dev;
struct bme280_data cdata;
uint32_t countloop = 0;
float pm25 = 0, pm10 = 0;
uint16_t SDS_OFFSET = 100;
uint32_t TIME_UNIT[] = {1, 60, 3600, 86400};
uint16_t PM_MAX1 = 0, PM_MIN1 = 500; 
uint16_t PM_MAX2 = 0, PM_MIN2 = 500; 
uint8_t PROCESS = 0;

//Interrupt UART Variables 
uint8_t Rx_DataReg1;
uint8_t Rx_DataReg2;
uint8_t Rx_DataReg3;
volatile uint8_t index1 = 0;
volatile uint8_t index2 = 0;
volatile uint8_t index3 = 0;

//Buffers 
uint8_t buffer1[100];
char buffer2[100];
char buffer3[100];
char message1[100]; 
char message2[100];
char rtc_data[100];
char esp_data[100];
char wifi_ssid[100];
char wifi_pass[100];
char cmd_manual[100];
char cmd_auto[100];
uint8_t sds_data[10];
uint16_t data_outdoor[10];
uint16_t Air_data[10];

//Flags
uint8_t ESP_BUSY = 1;
uint8_t ESP_ISCONNECTED = 0;
uint8_t FLAG_SDS = 1;
uint8_t FLAG_RTC = 1;
uint8_t FLAG_UPDATE_DATA = 1;
uint8_t FLAG_REQUEST_DATA = 1;
uint8_t FLAG_SEND_DATA = 1;
uint8_t FLAG_UPDATE_CONFIG_1 = 1;
uint8_t FLAG_UPDATE_CONFIG_2 = 1;
uint8_t FLAG_UPDATE_LCD_DATA = 1;
uint8_t FLAG_UPDATE_LCD_DATE_TIME = 1;
uint8_t FLAG_SENSORS_READ = 1;
uint8_t FLAG_REQUEST_TIME_NTP = 1;
uint8_t FLAG_UPDATE_ESP_STATUS = 1;
uint8_t ENABLE_READ_RTC = 1;
uint8_t FLAG_REQUEST_ESP_STATUS = 1;
uint8_t FLAG_RECV_STATUS = 1;
uint8_t FLAG_MODE_AUTO = 0;
uint8_t FLAG_CMD_MANUAL = 1;
uint8_t FLAG_CMD_AUTO = 1;
uint16_t checksum = 0;
uint8_t checksumFlag = 0;

//Couter variables
uint32_t counter[MAX_COUNTER_VARIABLES] = {0};
uint8_t hcnt[MAX_COUNTER_VARIABLES] = {0}; // Handle for counter registration
uint8_t dcnt[MAX_COUNTER_VARIABLES] = {0}; // Freze counter 
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_RTC_Init(void);
static void MX_I2C1_Init(void);
static void MX_SPI1_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_TIM3_Init(void);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/

/* USER CODE END PFP */

/* USER CODE BEGIN 0 */

// I2C delay/read/write for driver BME

int fputc(int c, FILE *f)
{
	HAL_UART_Transmit(&hua, (uint8_t*)&c, 1, 0x00FF);
	return 0;
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	// ------------------------------ UART ESP ------------------------------
	if (huart->Instance == huart2.Instance) 
	{
		
		if (index2 == 0) memset(&buffer2, 0, 100);
		//HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_RESET);
		switch (index2) 
		{
			case 0: 
				if (Rx_DataReg2 == '#') {buffer2[index2++] = Rx_DataReg2; }
				break;
			case 1:
				if (Rx_DataReg2 == 'E') {buffer2[index2++] = Rx_DataReg2; }
				else index2 = 0;
				break;
			case 2:
				if (Rx_DataReg2 == 'S') {buffer2[index2++] = Rx_DataReg2; }
				else index2 = 0;
				break;
			case 3:
				if (Rx_DataReg2 == 'P') {buffer2[index2++] = Rx_DataReg2; }
				else index2 = 0;
				break;
			default:
				if (Rx_DataReg2 != 0x0D) buffer2[index2++] = Rx_DataReg2;
				else 
				{
					// check buffer1[ESP_CMD_INDEX] for commander code:
					// 0 is State ESP in progress (Busy) 
					// 1 is State IsConnected Wifi
					// 2 is RTC data from NTP server
					// 3 is DataMessage from Local Network ESP, then copy to Message1[100]
					switch (buffer2[ESP_CMD_INDEX])
					{
						case '0': ESP_BUSY = 0; PROCESS = 0;
							break;
						case '1': 
							if (buffer2[ESP_CMD_INDEX + 1] == '1') 
								ESP_ISCONNECTED = 1;
							else 
								ESP_ISCONNECTED = 0; 
							dcnt[CNT6] = 0;
							break;
						case '2':
							FLAG_RTC = 0;
							ENABLE_READ_RTC = 0;
							HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15, GPIO_PIN_RESET) ;
							for (int i = 0; i < index2 - ESP_CMD_INDEX - 1; i++) 
								rtc_data[i] = buffer2[i + ESP_CMD_INDEX + 1];
							break;
						case '3': 
							FLAG_UPDATE_DATA = 0; HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_RESET);
							for (int i = 0; i < index2 - ESP_CMD_INDEX - 1; i++) 
								esp_data[i] = buffer2[i + ESP_CMD_INDEX + 1];
							dcnt[CNT4] = 0;
							break;
					}
					index2 = 0;
				}
		}
		HAL_UART_Receive_IT(&huart2, &Rx_DataReg2, 1);
	}
	// ------------------------------ UART SDS ------------------------------
	if (huart->Instance == huart1.Instance)
	{
		if (index1 == 0) memset(&buffer1, 0 , 100);
		switch (index1)
		{
			case 0:
				if (Rx_DataReg1 == 0xAA) {buffer1[index1++] = Rx_DataReg1;}
				break;
			case 1: 
				if (Rx_DataReg1 == 0xC0) {buffer1[index1++] = Rx_DataReg1;}
				else index1 = 0;
				break;
			default:
				if (index1  < SDS_FRAME) 
				{
					buffer1[index1] = Rx_DataReg1;
					if (index1 != 8) {checksum += Rx_DataReg1;}
					else if (Rx_DataReg1 == (checksum % 256)) checksumFlag = 1;
					index1++;
				}
				if (index1 == SDS_FRAME && Rx_DataReg1 == 0xAB)
				{
					if (checksumFlag)
					{
						for (int i = 0; i < 4; i++)  // Take 4 byte data of PM2.5 & PM10 
						{
							sds_data[i] = buffer1[i+2];
						}
						FLAG_SDS = 0;
					}
					index1 = 0;
				}
		}			
		HAL_UART_Receive_IT(&huart1, &Rx_DataReg1, 1);
	}
	// ------------------------------ UART LCD ------------------------------
	if (huart->Instance == huart3.Instance)
	{
		if (index3 == 0) memset(&buffer3, 0 , 100);
		{
			switch (index3)
			{
				case 0: 
					if (Rx_DataReg3 == '#') buffer3[index3++] = Rx_DataReg3;
					break;
				case 1: // N stands for "Nextion LCD"
					if (Rx_DataReg3 == 'N') buffer3[index3++] = Rx_DataReg3;
					else index3 = 0;
					break;
				default:
					if (Rx_DataReg3 != 0x0D) buffer3[index3++] = Rx_DataReg3;
					else
					{
						switch (buffer3[LCD_CMD_INDEX])
						{
							case '0': // reserved 
								break;
							case '1': // SSID WiFi Input
								FLAG_UPDATE_CONFIG_1 = 0;
								memset(&wifi_ssid, 0, 100);
								for (int i = 0; i < index3 - LCD_CMD_INDEX - 1; i++)
								{
									wifi_ssid[i] = buffer3[i + LCD_CMD_INDEX + 1];
								}
								break;
							case '2': // Password WiFi Input
								FLAG_UPDATE_CONFIG_2 = 0;
								memset(&wifi_pass, 0, 100);
								for (int i = 0; i < index3 - LCD_CMD_INDEX - 1; i++)
								{
									wifi_pass[i] = buffer3[i + LCD_CMD_INDEX + 1];
								}
								break;
							case '3':
								FLAG_CMD_MANUAL = 0;
								FLAG_MODE_AUTO = 1;
								memset(&cmd_manual, 0, 100);
								for (int i = 0; i < index3 - LCD_CMD_INDEX - 1; i++)
								{
									cmd_manual[i] = buffer3[i + LCD_CMD_INDEX + 1];
								}
								break;
							case '4':
								FLAG_REQUEST_TIME_NTP = 0;
								break;
							case '5':
								FLAG_REQUEST_DATA = 0;
								break;
						}
						index3 = 0;
					}
			}
		}
		HAL_UART_Receive_IT(&huart3, &Rx_DataReg3, 1);
	}
}
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	if(htim -> Instance == htim3.Instance)
	{
		for(int i = 0; i < MAX_COUNTER_VARIABLES; i++)
		{
			if(hcnt[i] == 1 && dcnt[i] == 0) counter[i]++;
		}
	}
}
uint8_t USER_ATOI(char c)
{
	if ((c > 0x2F) && (c < 0x3A)) 
	{
		return (c - 48);
	}
	else 
	{
		return 0;
	}
}
void PARSE_TIME(char time[RTC_DATA_FIELD][RTC_DATA_NUM_FORM], char *data)
{
	uint8_t n = 0;
	for(int i = 1; i < RTC_DATA_FIELD * RTC_DATA_NUM_FORM + 1;)  // Byte 1 of RTC_data for Status
	{
		for(int j = 0; j < RTC_DATA_NUM_FORM; j++)
		{
			time[n][j] = data[i++];
		}
		n++;
	}
}
void PARSE_DATA(char *data_in, uint16_t *data_out, uint8_t offset)
{
	uint8_t n = 0;
	char intermediate[ESP_DATA_FIELD][ESP_DATA_NUM_FORM];
	for(int i = 0; i < ESP_DATA_FIELD * ESP_DATA_NUM_FORM;)
	{
		for(int j = 0; j < ESP_DATA_NUM_FORM; j++)
		{
			intermediate[n][j] = data_in[i++];
		}
		n++;
	}
	for(int i = offset; i < ESP_DATA_FIELD + offset; i++)
	{
		data_out[i] = 0;
		for(int j = 0; j < ESP_DATA_NUM_FORM; j++)
		{
			data_out[i] += USER_ATOI(intermediate[i - offset][j]) * pow(10, ESP_DATA_NUM_FORM - 1 - j);
		}
	}
}
uint8_t BcdToDec(uint8_t Num) 
{
	return ((Num / 16) * 10 + (Num % 16)); 
}

uint8_t DecToBcd(uint8_t Num) 
{
	return ((Num / 10) * 16 + (Num % 10)); 
}
uint16_t USER_ROUND(float Num)
{
	int a = (int)Num;
	if (Num <= 0) return 0;
	if (Num - a >= 0.5){
		return (a + 1);
	}
	else{
		return a;
	}
}

void SET_DATE_TIME_FROM_NTP( uint8_t *flag, char *data)
{
	if(*flag == 0)
	{
		RTC_TimeTypeDef UTime;
		RTC_DateTypeDef UDate;
		char time[RTC_DATA_FIELD][2] = {0};
		uint8_t field[RTC_DATA_FIELD] = {0};
		
		PARSE_TIME(time, data);
		
		for(int i = 0; i < RTC_DATA_FIELD; i++)
		{
			for(int j = 0; j < RTC_DATA_NUM_FORM; j++)
			{
				field[i] += USER_ATOI(time[i][j]) * pow(10, RTC_DATA_NUM_FORM - 1 - j);
			}
		}
		UDate.Year = DecToBcd(field[0]);
		UDate.Month = DecToBcd(field[1]);
		UDate.Date = DecToBcd(field[2]);
		UDate.WeekDay = DecToBcd(field[6]);
		
		UTime.Hours = DecToBcd(field[3]);
		UTime.Minutes = DecToBcd(field[4]);
		UTime.Seconds = DecToBcd(field[5]);
//		hua = huart3;
//		printf("\nSET: BCD(%d:%d:%d)\n", UTime.Hours, UTime.Minutes, UTime.Seconds);
		HAL_RTC_SetTime(&hrtc, &UTime, RTC_FORMAT_BCD);
		HAL_RTC_SetDate(&hrtc, &UDate, RTC_FORMAT_BCD);
		*flag = 1;
	}
}

void SEND_DATA_THINGSPEAK(volatile uint8_t *esp_status, uint8_t *process, uint8_t *wifi, volatile uint8_t *flag, uint16_t *data, uint8_t field)
{
	if (*esp_status == 0 && *flag == 0 && *wifi == 1 && data[2] != 0)
	{
		char str[100] = {0};
		char buf[10] = {0};
		for(int i = 0; i < field; i++)
		{
			sprintf(buf, "%d%c", data[i], ';');
			strcat(str, buf);
		}
		hua  = huart2;
		printf("#3DATA;%s\r", str);
		hua  = huart3;
		printf("#3DATA;%s\r", str);
		
		*process = PROC_SEND_DATA_TO_THINGSPEAK;
		*esp_status = 1;
		*flag = 1;
	}
}	

void ESP_RESET()
{
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_RESET);
	HAL_Delay(50);
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_SET);
}

// I2C delay/read/write for driver BME
void USER_DELAY(uint32_t period)
{
 HAL_Delay(period);
}
int8_t USER_READ(uint8_t dev_id, uint8_t reg_addr, uint8_t *reg_data, uint16_t len)
{
	HAL_StatusTypeDef status = HAL_OK;
	int8_t rslt = 0;
	status = HAL_I2C_Mem_Read(&hi2c1, (uint8_t)(dev_id<<1), (uint8_t)reg_addr, I2C_MEMADD_SIZE_8BIT, reg_data, len, 100);
	if (status != HAL_OK) rslt = 1;
	return rslt;
}
int8_t USER_WRITE(uint8_t dev_id, uint8_t reg_addr, uint8_t *reg_data, uint16_t len)
{
	HAL_StatusTypeDef status = HAL_OK;
	int8_t rslt = 0;
	status = HAL_I2C_Mem_Write(&hi2c1, (uint8_t)(dev_id<<1), (uint8_t)reg_addr, I2C_MEMADD_SIZE_8BIT, reg_data, len, 100);
	if (status != HAL_OK) rslt = 1;
	return rslt;
}

int8_t BME280_UINIT(struct bme280_dev *pdev)
{
	int8_t rslt = BME280_OK;
	pdev -> dev_id = BME280_I2C_ADDR_PRIM;
	pdev -> intf = BME280_I2C_INTF;
	pdev -> read = USER_READ;
	pdev -> write = USER_WRITE;
	pdev -> delay_ms = USER_DELAY;

	rslt = bme280_init(pdev);
	return rslt;
}
int8_t BME280_SETTING(struct bme280_dev *pdev)
{
	int8_t rslt;
	uint8_t settings_sel;

	/* Recommended mode of operation: Indoor navigation */
	pdev->settings.osr_h = BME280_OVERSAMPLING_1X;
	pdev->settings.osr_p = BME280_OVERSAMPLING_16X;
	pdev->settings.osr_t = BME280_OVERSAMPLING_2X;
	pdev->settings.filter = BME280_FILTER_COEFF_16;
	pdev->settings.standby_time = BME280_STANDBY_TIME_62_5_MS;

	settings_sel = BME280_OSR_PRESS_SEL;
	settings_sel |= BME280_OSR_TEMP_SEL;
	settings_sel |= BME280_OSR_HUM_SEL;
	settings_sel |= BME280_STANDBY_SEL;
	settings_sel |= BME280_FILTER_SEL;
	rslt = bme280_set_sensor_settings(settings_sel, pdev);
	rslt = bme280_set_sensor_mode(BME280_NORMAL_MODE, pdev);
	
	return rslt;
}
int8_t BME280_READ_DATA(struct bme280_data *pcomp_data, struct bme280_dev *pdev)
{
	int8_t rslt;
	rslt = bme280_get_sensor_data(BME280_ALL, pcomp_data, pdev);
	return rslt;
}

uint16_t getAqi(float C, uint8_t type) 
{
  float I, I_hi, I_low;
  float  C_hi, C_low;
  if (type == 25) {
    if (C > 0 && C < 15.4) {
      C_hi  = 15.4;
      C_low = 0.0;
      I_hi  = 50;
      I_low = 0;
    }
    else if (C < 40.4) {
      C_hi  = 40.4;
      C_low = 15.5;
      I_hi  = 100;
      I_low = 51;
    }
    else if (C < 65.4) {
      C_hi  = 65.4;
      C_low = 40.5;
      I_hi  = 150;
      I_low = 101;
    }
    else if (C < 150.4) {
      C_hi  = 150.4;
      C_low = 65.5;
      I_hi  = 200;
      I_low = 151;
    }
    else if (C < 250.4) {
      C_hi  = 250.4;
      C_low = 150.5;
      I_hi  = 300;
      I_low = 201;
    }
    else if (C < 350.4) {
      C_hi  = 350.4;
      C_low = 250.5;
      I_hi  = 400;
      I_low = 301;
    }
    else if (C < 500.4) {
      C_hi  = 500.4;
      C_low = 350.5;
      I_hi  = 500;
      I_low = 401;
    }
  }
  if (type == 10) {
    if (C > 0 && C < 54) {
      C_hi  = 54;
      C_low = 0.0;
      I_hi  = 50;
      I_low = 0;
    }
    else if (C < 154) {
      C_hi  = 154;
      C_low = 55;
      I_hi  = 100;
      I_low = 51;
    }
    else if (C < 254) {
      C_hi  = 254;
      C_low = 155;
      I_hi  = 150;
      I_low = 101;
    }
    else if (C < 354) {
      C_hi  = 354;
      C_low = 255;
      I_hi  = 200;
      I_low = 151;
    }
    else if (C < 424) {
      C_hi  = 424;
      C_low = 355;
      I_hi  = 300;
      I_low = 201;
    }
    else if (C < 504) {
      C_hi  = 504;
      C_low = 425;
      I_hi  = 400;
      I_low = 301;
    }
    else if (C < 604) {
      C_hi  = 604;
      C_low = 505;
      I_hi  = 500;
      I_low = 401;
    }
    else return 500;
  }

  I = ((I_hi - I_low) / (C_hi - C_low) * (C - C_low)) + I_low;
  return (uint16_t)I;
}

uint16_t USER_MAX(uint16_t num1, uint16_t num2)
{
	if (num1 > num2)
	{
		return num1;
	}
	else 
	{
		return num2;
	}
}
uint16_t USER_MIN(uint16_t num1, uint16_t num2)
{
	if (num1 < num2)
	{
		return num1;
	}
	else 
	{
		return num2;
	}
}
void SDS011_USER_READ(uint8_t *flag, float *pm25, float *pm10)
{
	if (*flag == 0)
	{
		*pm25 = (float)(sds_data[1] << 8);
		*pm25 += (float)sds_data[0];
		*pm25 /= 10.0;
		*pm10 = (float)(sds_data[3] << 8);
		*pm10 += (float)sds_data[2];
		*pm10 /= 10.0;
		*flag = 1;
	}
}
void LCD_CMD(char *cmd)
{
	hua = huart3;
	printf("%s", cmd);
	printf("%c%c%c", 0xFF, 0xFF, 0xFF);
}
void LCD_UPDATE_DATA_PAGE(uint8_t *flag, uint8_t pg_number, uint16_t *data, uint8_t flag_option)
{
	if (*flag == 0)
	{
		if (pg_number == LCD_PAGE_INDOOR)
		{
			//Normal Data
			for (int i = 0; i < LCD_FIELD_OF_PAGE; i++)
			{
				char cmd[50];
				sprintf(cmd, "%s.n%d.val=%d", LCD_PAGE_NAME[pg_number], i, data[i]);
				LCD_CMD(cmd);
			}
			
			// Max, Min AQI
			char max_aqi[50], min_aqi[50];
			PM_MAX1 = USER_MAX(PM_MAX1, data[DATA_AQI_INDEX]);
			PM_MIN1 = USER_MIN(PM_MIN1, data[DATA_AQI_INDEX]);
			sprintf(max_aqi, "%s.%s.val=%d", LCD_PAGE_NAME[pg_number], LCD_VAR_NAME[LCD_VAR_MAX_NAME], PM_MAX1);
			sprintf(min_aqi, "%s.%s.val=%d", LCD_PAGE_NAME[pg_number], LCD_VAR_NAME[LCD_VAR_MIN_NAME], PM_MIN1);
			if (data[DATA_AQI_INDEX] != 0)
			{
				LCD_CMD(max_aqi);
				LCD_CMD(min_aqi);
			}
		}
		else
		{
			if (pg_number == LCD_PAGE_OUTDOOR)
			{
				//Normal Data
				for (int i = 0; i < LCD_FIELD_OF_PAGE; i++)
				{
					char cmd[50];
					sprintf(cmd, "%s.n%d.val=%d", LCD_PAGE_NAME[pg_number], i, data[i]);
					LCD_CMD(cmd);
				}	
				
				//Level Water
				char level[50];
				sprintf(level, "%s.%s.val=%d",
											LCD_PAGE_NAME[LCD_PAGE_OUTDOOR],
											LCD_VAR_NAME[LCD_VAR_LEVEL_NAME],
											data[LCD_LEVEL_DATA_INDEX]);
				LCD_CMD(level);
				
				//Max, Min AQI
				char max_aqi[50], min_aqi[50];
				PM_MAX2 = USER_MAX(PM_MAX2, data[DATA_AQI_INDEX]);
				PM_MIN2 = USER_MIN(PM_MIN2, data[DATA_AQI_INDEX]);
				sprintf(max_aqi, "%s.%s.val=%d", LCD_PAGE_NAME[pg_number], LCD_VAR_NAME[LCD_VAR_MAX_NAME], PM_MAX2);
				sprintf(min_aqi, "%s.%s.val=%d", LCD_PAGE_NAME[pg_number], LCD_VAR_NAME[LCD_VAR_MIN_NAME], PM_MIN2);
				if (data[DATA_AQI_INDEX] != 0)
				{
					LCD_CMD(max_aqi);
					LCD_CMD(min_aqi);
				}
			}		
		}
		if (flag_option == 1) *flag = 1;
	}
}
uint8_t REQUEST_UPDATE_DATA(uint8_t *flag_free, uint8_t *process, uint8_t *flag_request, uint8_t slave_id, uint8_t id_cnt)
{
	if(*flag_free == 0 && *flag_request == 0)
	{
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, GPIO_PIN_RESET);
		hua = huart2;
		printf("#5SLAVE%d\r", slave_id);	
		*process = PROC_RECV_DATA_OUTDOOR_DEVICE;
		*flag_request = 1;
		*flag_free = 1;
		dcnt[id_cnt] = 1;
		return 1; 
	}
	else return 0;
}
void GET_UPDATE_DATA(uint8_t *flag_update, char *data_in, uint16_t *data_out)
{
	if(*flag_update == 0)
	{
		PARSE_DATA(data_in, data_out, 1);
		data_out[0] = USER_MAX(getAqi(data_out[1], 25), getAqi(data_out[2], 10));
		*flag_update = 1;
	}
}
void FLAG_UPDATE_STATUS(uint8_t *flag, uint8_t counter_id, uint8_t type_action, uint8_t time_format, uint16_t period)
{
	/*Type Action: 	0 for FALLING (1 -> 0)
									1 for RASING (0 -> 1)
									
		Time Format:	0 - Second
									1 - Minute
									2 - Hour
									3 - Day
		TIME_UNIT[] = {1, 60, 3600, 86400}
	*/
	if(counter_id < MAX_COUNTER_VARIABLES)
	{
		if(hcnt[counter_id] == 0) hcnt[counter_id] = 1;  // Flag Registration 
		if(counter[counter_id] >= period * TIME_UNIT[time_format])
		{
			if(type_action == FLAG_RAISING) *flag = 1;
			else *flag = 0;
			counter[counter_id] = 0; // Reset counter
		}
	}

}
void READ_SENSORS(uint8_t *flag, struct bme280_dev *pdev, struct bme280_data *pdata, float *pm25, float *pm10, uint8_t *flagsds)
{
	if(*flag == 0)
	{
		BME280_READ_DATA(pdata, pdev);
		SDS011_USER_READ(flagsds, pm25, pm10);	
		*flag = 1;
	}
}
void REQUEST_TIME_NTP(uint8_t *flagfree, uint8_t *process, uint8_t *wifi, uint8_t *flag)
{
	if(*flagfree == 0 && *flag == 0 && *wifi == 1)
	{
		hua = huart2;
		printf("#1122\r");
		*process = PROC_SYNC_TIME;
		*flag = 1;
		*flagfree = 1;
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_RESET);
	}
}
void DISPLAY_TIME(uint8_t *en_flag, uint8_t *update_flag, RTC_TimeTypeDef *ptime, RTC_DateTypeDef *pdate)
{
	if (*en_flag == 0 && *update_flag == 0)
	{
		
		HAL_RTC_GetTime(&hrtc, ptime, RTC_FORMAT_BCD);
		HAL_RTC_GetDate(&hrtc, pdate, RTC_FORMAT_BCD);
		char up_date[50];
		char up_time[50];
		sprintf(up_date, "%s.%s.txt=\"%s, %d %s\"", LCD_PAGE_NAME[LCD_PAGE_INDOOR],
								LCD_VAR_NAME[LCD_VAR_DATE_NAME], WEEKDAY[BcdToDec(pdate->WeekDay)], BcdToDec(pdate->Date), MONTH[BcdToDec(pdate->Month) - 1]);
		sprintf(up_time, "%s.%s.txt=\"%d:%d:%d\"", LCD_PAGE_NAME[LCD_PAGE_INDOOR],
								LCD_VAR_NAME[LCD_VAR_TIME_NAME], BcdToDec(ptime->Hours), BcdToDec(ptime->Minutes), BcdToDec(ptime->Seconds));		
		LCD_CMD(up_date);
		LCD_CMD(up_time);
		*update_flag = 1;
	}
}
void DISPLAY_STATUS(uint8_t *flag, uint8_t state, uint8_t process)
{
	if (*flag == 0)
	{
		if (state == 1 || state == 0)
		{
			char cmd1[50], cmd2[50];
			sprintf(cmd1, "%s.%s.val=%d", LCD_PAGE_NAME[LCD_PAGE_INDOOR], LCD_VAR_NAME[LCD_VAR_STATE_NAME], state);
			sprintf(cmd2, "%s.%s.val=%d", LCD_PAGE_NAME[LCD_PAGE_INDOOR], LCD_VAR_NAME[LCD_VAR_PROCESS_NAME], process);
			LCD_CMD(cmd1);
			LCD_CMD(cmd2);
		}
		*flag = 1;		
	}
}
void REQUEST_ESP_STATUS(uint8_t *flag_free, uint8_t *process, uint8_t *flag_request, uint8_t id_cnt)
{
	if (*flag_free == 0 && *flag_request == 0)
	{
		hua = huart2;
		printf("#2\r");
		FLAG_RECV_STATUS = 0;
		*process = PROC_QUERY_STATUS;
		*flag_free = 1;
		*flag_request = 1;	
		dcnt[id_cnt] = 1;
		
	}
		
}
void WRITE_CONFIG(uint8_t *flag_free, uint8_t *flag1, uint8_t *flag2, uint8_t *process)
{
	if (*flag_free == 0 && *flag1 == 0 && *flag2 == 0)
	{
		hua = huart2;
		printf("#6;%s;%s;\r", wifi_ssid, wifi_pass);
		*process = PROC_WRITE_CONFIG;
		*flag1 = 1;
		*flag2 = 1;
		*flag_free = 1;
	}
}
void SEND_COMMAND_MANUAL(uint8_t *flag_free, uint8_t *process, uint8_t *flag, char *cmd)
{
	if (*flag_free == 0 && *flag == 0)
	{
		hua = huart2;
		printf("#4;%s;\r", cmd);
		*process = PROC_SEND_COMMAND_MODE_MANUAL;
		*flag_free = 1;
		*flag = 1;
		
	}
}
void SEND_COMMAND_AUTO(uint8_t *flag_free, uint8_t *process, uint8_t *flag, uint8_t *manual, char *cmd)
{
	if (*flag_free == 0 && *flag == 0 && *manual == 0)
	{
		hua = huart2;
		printf("#4;%s;\r", cmd);
		*process = PROC_SEND_COMMAND_MODE_AUTO;
		*flag_free = 1;
		*flag = 1;
		
	}
}
/* USER CODE END 0 */

int main(void)
{

  /* USER CODE BEGIN 1 */
	
  /* USER CODE END 1 */

  /* MCU Configuration----------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* Configure the system clock */
  SystemClock_Config();

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_RTC_Init();
  MX_I2C1_Init();
  MX_SPI1_Init();
  MX_USART1_UART_Init();
  MX_USART2_UART_Init();
  MX_USART3_UART_Init();
  MX_TIM3_Init();

  /* USER CODE BEGIN 2 */
	//---------------------------------------------------------
	countloop++;
	RTC_TimeTypeDef RTC_Time;
	RTC_DateTypeDef RTC_Date;
	ESP_RESET();
	HAL_Delay(2000);
	HAL_UART_Receive_IT(&huart1, &Rx_DataReg1, 1);
	HAL_UART_Receive_IT(&huart2, &Rx_DataReg2, 1);
	HAL_UART_Receive_IT(&huart3, &Rx_DataReg3, 1);
	
	// INIT & SETUP ESP 
	hua = huart2;
	printf("import myos\r");
	//HAL_Delay(5000);
	
	//INIT BME280
	hua = huart3;
	printf("INIT BME\n");
	BME280_UINIT(&dev);
	hua = huart3;
	printf("INIT BME OK\n");
	BME280_SETTING(&dev);
	FLAG_REQUEST_DATA = 1;
	FLAG_SEND_DATA = 0;
	FLAG_REQUEST_TIME_NTP = 0;

	//Start Timer for OS
	HAL_TIM_Base_Start_IT(&htim3);
	//---------------------------------------------------------
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
  /* USER CODE END WHILE */

  /* USER CODE BEGIN 3 */
		
		// Read data from Sensors
		FLAG_UPDATE_STATUS(&FLAG_SENSORS_READ, CNT0, FLAG_FALLING, TIME_FORMAT_SECOND, 1);
		READ_SENSORS(&FLAG_SENSORS_READ, &dev, &cdata, &pm25, &pm10, &FLAG_SDS);
		
		// Update LCD Display
		FLAG_UPDATE_STATUS(&FLAG_UPDATE_LCD_DATA, CNT1, FLAG_FALLING, TIME_FORMAT_SECOND, 1);
		if (FLAG_UPDATE_LCD_DATA == 0)
		{
			Air_data[0] = USER_MAX(getAqi(pm25, 25), getAqi(pm10, 10));
			Air_data[1] = USER_ROUND(pm25);
			Air_data[2] = USER_ROUND(pm10);
			Air_data[3] = USER_ROUND(cdata.temperature);
			Air_data[4] = USER_ROUND(cdata.humidity);
			Air_data[5] = USER_ROUND(cdata.pressure / 100);		
		}
		LCD_UPDATE_DATA_PAGE(&FLAG_UPDATE_LCD_DATA, 0, Air_data, FLAG_DEFAULT);
		LCD_UPDATE_DATA_PAGE(&FLAG_UPDATE_LCD_DATA, 1, data_outdoor, FLAG_RAISE);
		
		// Set date time periodic
		FLAG_UPDATE_STATUS(&FLAG_REQUEST_TIME_NTP, CNT2, FLAG_FALLING, TIME_FORMAT_DAY, 1);
		REQUEST_TIME_NTP(&ESP_BUSY, &PROCESS, &ESP_ISCONNECTED, &FLAG_REQUEST_TIME_NTP);
		SET_DATE_TIME_FROM_NTP(&FLAG_RTC, rtc_data);
		
		// Send data to Thingspeak
		FLAG_UPDATE_STATUS(&FLAG_SEND_DATA, CNT3, FLAG_FALLING, TIME_FORMAT_MINUTE, 1);
		SEND_DATA_THINGSPEAK(&ESP_BUSY, &PROCESS, &ESP_ISCONNECTED, &FLAG_SEND_DATA, Air_data, 6);
		
		// Get data of outdoor device
		FLAG_UPDATE_STATUS(&FLAG_REQUEST_DATA, CNT4, FLAG_FALLING, TIME_FORMAT_DAY, 2);
		REQUEST_UPDATE_DATA(&ESP_BUSY, &PROCESS, &FLAG_REQUEST_DATA, 1, CNT4);
		GET_UPDATE_DATA(&FLAG_UPDATE_DATA, esp_data, data_outdoor);
		
		// Update time to LCD
		FLAG_UPDATE_STATUS(&FLAG_UPDATE_LCD_DATE_TIME, CNT5, FLAG_FALLING, TIME_FORMAT_SECOND, 1);
		DISPLAY_TIME(&ENABLE_READ_RTC, &FLAG_UPDATE_LCD_DATE_TIME, &RTC_Time, &RTC_Date);
		
		// Update Status of WiFi connection and State of ESP (Free or Busy)
		FLAG_UPDATE_STATUS(&FLAG_REQUEST_ESP_STATUS, CNT6, FLAG_FALLING, TIME_FORMAT_SECOND, 30);
		REQUEST_ESP_STATUS(&ESP_BUSY, &PROCESS, &FLAG_REQUEST_ESP_STATUS, CNT6);
		
		FLAG_UPDATE_STATUS(&FLAG_UPDATE_ESP_STATUS, CNT7, FLAG_FALLING, TIME_FORMAT_SECOND, 1);
		DISPLAY_STATUS(&FLAG_UPDATE_ESP_STATUS, ESP_ISCONNECTED, PROCESS);
		
		// Write Config to ESP
		WRITE_CONFIG(&ESP_BUSY, &FLAG_UPDATE_CONFIG_1, &FLAG_UPDATE_CONFIG_2, &PROCESS);
		
		// Send Command Auto-Mode
		SEND_COMMAND_AUTO(&ESP_BUSY, &PROCESS, &FLAG_CMD_AUTO, &FLAG_MODE_AUTO, cmd_auto);
		
		// Send Command Manual-Mode
		SEND_COMMAND_MANUAL(&ESP_BUSY, &PROCESS, &FLAG_CMD_MANUAL, cmd_manual);
		
		//HAL_Delay(10);
		
  }
  /* USER CODE END 3 */

}

/** System Clock Configuration
*/
void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;
  RCC_PeriphCLKInitTypeDef PeriphClkInit;

  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL6;
  HAL_RCC_OscConfig(&RCC_OscInitStruct);

  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;
  HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1);

  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_RTC;
  PeriphClkInit.RTCClockSelection = RCC_RTCCLKSOURCE_HSE_DIV128;
  HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit);

  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}

/* I2C1 init function */
void MX_I2C1_Init(void)
{

  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 100000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  HAL_I2C_Init(&hi2c1);

}

/* RTC init function */
void MX_RTC_Init(void)
{

  RTC_TimeTypeDef sTime;
  RTC_DateTypeDef DateToUpdate;

    /**Initialize RTC and set the Time and Date 
    */
  hrtc.Instance = RTC;
  hrtc.Init.AsynchPrediv = RTC_AUTO_1_SECOND;
  hrtc.Init.OutPut = RTC_OUTPUTSOURCE_NONE;
  HAL_RTC_Init(&hrtc);

  sTime.Hours = 0x1;
  sTime.Minutes = 0x0;
  sTime.Seconds = 0x0;

  HAL_RTC_SetTime(&hrtc, &sTime, RTC_FORMAT_BCD);

  DateToUpdate.WeekDay = RTC_WEEKDAY_MONDAY;
  DateToUpdate.Month = RTC_MONTH_JANUARY;
  DateToUpdate.Date = 0x1;
  DateToUpdate.Year = 0x0;

  HAL_RTC_SetDate(&hrtc, &DateToUpdate, RTC_FORMAT_BCD);

}

/* SPI1 init function */
void MX_SPI1_Init(void)
{

  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 10;
  HAL_SPI_Init(&hspi1);

}

/* TIM3 init function */
void MX_TIM3_Init(void)
{

  TIM_ClockConfigTypeDef sClockSourceConfig;
  TIM_MasterConfigTypeDef sMasterConfig;

  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 48000;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 999;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  HAL_TIM_Base_Init(&htim3);

  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig);

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig);

}

/* USART1 init function */
void MX_USART1_UART_Init(void)
{

  huart1.Instance = USART1;
  huart1.Init.BaudRate = 9600;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  HAL_UART_Init(&huart1);

}

/* USART2 init function */
void MX_USART2_UART_Init(void)
{

  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  HAL_UART_Init(&huart2);

}

/* USART3 init function */
void MX_USART3_UART_Init(void)
{

  huart3.Instance = USART3;
  huart3.Init.BaudRate = 115200;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  HAL_UART_Init(&huart3);

}

/** Configure pins as 
        * Analog 
        * Input 
        * Output
        * EVENT_OUT
        * EXTI
*/
void MX_GPIO_Init(void)
{

  GPIO_InitTypeDef GPIO_InitStruct;

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4|GPIO_PIN_8, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_9, GPIO_PIN_RESET);

  /*Configure GPIO pin : PA4 */
  GPIO_InitStruct.Pin = GPIO_PIN_4;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PB12 PB13 PB14 PB15 
                           PB9 */
  GPIO_InitStruct.Pin = GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15 
                          |GPIO_PIN_9;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : PA8 */
  GPIO_InitStruct.Pin = GPIO_PIN_8;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

#ifdef USE_FULL_ASSERT

/**
   * @brief Reports the name of the source file and the source line number
   * where the assert_param error has occurred.
   * @param file: pointer to the source file name
   * @param line: assert_param error line source number
   * @retval None
   */
void assert_failed(uint8_t* file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
    ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */

}

#endif

/**
  * @}
  */ 

/**
  * @}
*/ 

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
