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
#include "adc.h"
#include "dac.h"
#include "dma.h"
#include "i2c.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "oled.h"
#include "font.h"
#include "arm_const_structs.h"
#include "arm_math.h"
#include "AD9833.h"
#include "filter.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

#define ADC_BUFFER_SIZE   ((uint32_t)  4096)

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

uint16_t adc_data_buffer[ADC_BUFFER_SIZE * 2 * 2] __attribute__((section(".adc")));
uint16_t adc_data_out[ADC_BUFFER_SIZE * 2 * 2] __attribute__((section(".dac")));

float32_t fft_input_1_1[ADC_BUFFER_SIZE * 2];
// float32_t fft_input_1_2[ADC_BUFFER_SIZE * 2];
// float32_t fft_input_2_1[ADC_BUFFER_SIZE * 2] __attribute__((section(".dac")));
// float32_t fft_input_2_2[ADC_BUFFER_SIZE * 2] __attribute__((section(".dac")));
float32_t magnitude_output[ADC_BUFFER_SIZE];

float32_t hanning_win_table[ADC_BUFFER_SIZE];


KalmanFilter phase_filter;



int converted = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
char *Float2String(float value);
void calculate_hamming_window();
void data_process();
int detect_wareform_type(uint32_t base_idx);
float calculate_phase(int offset, int fft_size, const arm_cfft_instance_f32 * S);
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
  calculate_hamming_window();
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
  MX_DMA_Init();
  MX_ADC1_Init();
  MX_I2C1_Init();
  MX_USART1_UART_Init();
  MX_DAC1_Init();
  MX_TIM6_Init();
  MX_TIM1_Init();
  /* USER CODE BEGIN 2 */
  if (HAL_ADCEx_Calibration_Start(&hadc1, ADC_CALIB_OFFSET, ADC_SINGLE_ENDED) != HAL_OK){Error_Handler();}
  if (HAL_ADC_Start_DMA(&hadc1,(uint32_t *)adc_data_buffer,ADC_BUFFER_SIZE * 2 * 2) != HAL_OK){Error_Handler();}
  OLED_Init();

  kalman_filter_init(&phase_filter, 0.5, 0.1);

  AD9833_WaveSeting(110900.1,0,TRI_WAVE,0);
  for (int i=0;i<=10;i++)AD9833_AmpSet(200); //设置幅值，幅值最大 255

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    // HAL_GPIO_TogglePin(GPIOG, GPIO_PIN_7);
    OLED_NewFrame();
    if (converted) {
      converted = 0;
      HAL_GPIO_WritePin(GPIOG, GPIO_PIN_7, GPIO_PIN_RESET);

      int dot = 512;
      for (int i=0; i<dot; i++) {
        char msg[40];
        sprintf(msg, "%d,%d,%d,%d\n" , adc_data_out[i*2+1], adc_data_out[i*2], adc_data_out[i*2+dot*2+1], adc_data_out[i*2+dot*2]);
        // HAL_UART_Transmit(&huart1, (uint8_t *)msg, strlen(msg), HAL_MAX_DELAY);
      }

      data_process();

      if (HAL_ADC_Start_DMA(&hadc1,(uint32_t *)adc_data_buffer,ADC_BUFFER_SIZE * 2 * 2) != HAL_OK){Error_Handler();}
    } else {
      HAL_GPIO_WritePin(GPIOG, GPIO_PIN_7, GPIO_PIN_SET);
    }
    OLED_ShowFrame();
    HAL_Delay(50);
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

  /** Supply configuration update enable
  */
  HAL_PWREx_ConfigSupply(PWR_LDO_SUPPLY);

  /** Configure the main internal regulator output voltage
  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE0);

  while(!__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY)) {}

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_DIV1;
  RCC_OscInitStruct.HSICalibrationValue = 64;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 34;
  RCC_OscInitStruct.PLL.PLLP = 1;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
  RCC_OscInitStruct.PLL.PLLRGE = RCC_PLL1VCIRANGE_3;
  RCC_OscInitStruct.PLL.PLLVCOSEL = RCC_PLL1VCOWIDE;
  RCC_OscInitStruct.PLL.PLLFRACN = 3072;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2
                              |RCC_CLOCKTYPE_D3PCLK1|RCC_CLOCKTYPE_D1PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.SYSCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB3CLKDivider = RCC_APB3_DIV2;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_APB1_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_APB2_DIV2;
  RCC_ClkInitStruct.APB4CLKDivider = RCC_APB4_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

#define FLOAT_STR_COUNT 4   // 支持最多4个并发调用
#define FLOAT_STR_SIZE 16
char *Float2String(float value)
{
  // 每个线程有自己的缓冲区，避免竞争
  _Thread_local static char buf[FLOAT_STR_COUNT][FLOAT_STR_SIZE];
  _Thread_local static int idx = 0;

  char* str = buf[idx];
  idx = (idx + 1) % FLOAT_STR_COUNT;

  int Head = abs((int)value);
  char c = value >= 0 ? '+' : '-';
  float p = fabsf(value - (int)value);
  char zero[5] = "";
  for (int i = 0; i < 4; i++) {
    p *= 10;
    if (p < 1) strcat(zero, "0");
    else break;
  }
  int Point = abs((int)((value - Head) * 100));
  snprintf(str, FLOAT_STR_SIZE, "%c%d.%s%d", c, Head, zero, Point);
  return str;
}

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc)
{
  if(hadc->Instance == ADC1)
  {
    HAL_ADC_Stop_DMA(&hadc1);
    memset(adc_data_out,0,sizeof(adc_data_out));
    memcpy(adc_data_out,adc_data_buffer,sizeof(adc_data_buffer));
    converted = 1;
  }
}

void calculate_hamming_window() {
  int n = ADC_BUFFER_SIZE;
  for (int i = 0; i < n; ++i) {
    hanning_win_table[i] = 0.54 - 0.46 * arm_cos_f32(2 * M_PI * i / (n - 1));
    // hanning_win_table[i] = 1;
  }
}

#define HISTORY_SIZE 8           /* 必须是 2 的幂次，方便移位除法 */
static float phase_history[HISTORY_SIZE] = {0};
float calc_slope_scan_f32(void)
{
  float sum_x  = 0.0f;
  float sum_y  = 0.0f;
  float sum_xy = 0.0f;
  float sum_x2 = 0.0f;

  /* 现遍历，现累加 */
  for (int i = 0; i < HISTORY_SIZE; ++i) {
    float x = (float)i;                 // 时间轴 0,1,2…
    float y = phase_history[i];
    sum_x  += x;
    sum_y  += y;
    sum_xy += x * y;
    sum_x2 += x * x;
  }

  float den = HISTORY_SIZE * sum_x2 - sum_x * sum_x;
  if (den == 0.0f) return 0.0f;           // 防除 0

  float k = (HISTORY_SIZE * sum_xy - sum_x * sum_y) / den;
  return k;                               // 单位：相位/ms
}

float global_DIV = 0;
float freq_correction = 0;
float last_freq_correction = 0;
float last_freq = 0;
int last_mode = 0;
#define BASE_FREQ_LOW 0.0961538 // khz <=15k
#define BASE_FREQ_HIGH 0.095849 // khz
void data_process() {
  // calculate the average value
  int sig_avg_1 = 0;
  int sig_avg_2 = 0;
  for (int i = 0; i < ADC_BUFFER_SIZE; ++i) {
    sig_avg_1 += adc_data_out[i * 2];
    sig_avg_2 += adc_data_out[i * 2 + 1];
  }
  sig_avg_1 /= ADC_BUFFER_SIZE;
  sig_avg_2 /= ADC_BUFFER_SIZE;

  // apply hanning window and convert to float
  // prepare for fft
  for (int i = 0; i < ADC_BUFFER_SIZE; ++i) {
    fft_input_1_1[i*2] = hanning_win_table[i] * ((float32_t)adc_data_out[i*2] - sig_avg_1) / 65535.0f; // real part
    fft_input_1_1[i*2 + 1] = 0.0f; // imaginary part

    // fft_input_1_2[i*2] = hanning_win_table[i] * ((float32_t)adc_data_out[i*2 + 1] - sig_avg_2) / 65535.0f; // real part
    // fft_input_1_2[i*2 + 1] = 0.0f; // imaginary part

    // fft_input_2_1[i*2] = hanning_win_table[i] * ((float32_t)adc_data_out[i*2 + ADC_BUFFER_SIZE] - sig_avg_1) / 65535.0f; // real part
    // fft_input_2_1[i*2 + 1] = 0.0f; // imaginary part

    // fft_input_2_2[i*2] = hanning_win_table[i] * ((float32_t)adc_data_out[i*2 + 1 + ADC_BUFFER_SIZE] - sig_avg_2) / 65535.0f; // real part
    // fft_input_2_2[i*2 + 1] = 0.0f; // imaginary part
  }

  // perform fft
  arm_cfft_f32(&arm_cfft_sR_f32_len4096, fft_input_1_1, 0, 1);
  // arm_cfft_f32(&arm_cfft_sR_f32_len4096, fft_input_2_2, 0, 1);
  // arm_cfft_f32(&arm_cfft_sR_f32_len4096, fft_input_2_1, 0, 1);
  // arm_cfft_f32(&arm_cfft_sR_f32_len4096, fft_input_2_2, 0, 1);
  arm_cmplx_mag_f32(fft_input_1_1 , magnitude_output , ADC_BUFFER_SIZE);

  // show magnitude
  float tmp_magnitude = 0;
  for (int i = 1; i < 128; ++i) {
    tmp_magnitude = 0;
    for (int j = 0; j < ADC_BUFFER_SIZE / 2 / 128; ++j) {
      tmp_magnitude += pow(magnitude_output[i * ADC_BUFFER_SIZE / 2 / 128 + j],2);
    }
    tmp_magnitude = sqrt(tmp_magnitude);
    OLED_DrawLine(i, 64, i, 64 - tmp_magnitude*0.1, OLED_COLOR_NORMAL);
  }

  // find the peak frequency
  uint32_t peak_index;
  float32_t peak_value;
  arm_max_f32(magnitude_output + 1, ADC_BUFFER_SIZE / 2 - 1, &peak_value, &peak_index);
  peak_index += 1; // because we skipped the DC component
  char msg[30];
  float freq;
  if (peak_index < 160) {
    freq = round(peak_index * BASE_FREQ_LOW * 10) / 10.0f;
    // freq = peak_index * BASE_FREQ_LOW;
    sniprintf(msg, sizeof(msg), "F1:%s kHz", Float2String(freq));
  }else {
    freq = round(peak_index * BASE_FREQ_HIGH * 10) / 10.0f;
    // freq = peak_index * BASE_FREQ_HIGH;
    sniprintf(msg, sizeof(msg), "F1:%s kHz", Float2String(freq));
  }
  OLED_PrintString(0, 0, msg, &font16x16, OLED_COLOR_NORMAL);

  // detect waveform type
  int signal1_waveform_type = detect_wareform_type(peak_index);
  if (signal1_waveform_type == 0) {
    sniprintf(msg, sizeof(msg), "Sine");
  } else if (signal1_waveform_type == 1) {
    sniprintf(msg, sizeof(msg), "Ramp");
  } else {
    sniprintf(msg, sizeof(msg), "Square");
  }
  OLED_PrintString(80, 0, msg, &font16x16, OLED_COLOR_NORMAL);

  // sniprintf(msg, sizeof(msg), "div:%s", Float2String(global_DIV));
  // OLED_PrintString(0,32, msg, &font16x16, OLED_COLOR_NORMAL);


  float phase_1_1 = calculate_phase(0, 256, &arm_cfft_sR_f32_len256);
  float phase_1_2 = calculate_phase(1, 256, &arm_cfft_sR_f32_len256);

  if (phase_1_1 - phase_1_2 > 180) phase_1_2 += 360;
  if (phase_1_2 - phase_1_1 > 180) phase_1_1 += 360;
  float phase_diff = (phase_1_1 - phase_1_2);

  for (int i = 1; i < HISTORY_SIZE; ++i) {
    phase_history[i] = phase_history[i - 1];
  }
  phase_history[0] = phase_diff;
  if (phase_diff > 170 && phase_history[1] < -165) {
    for (int i = 1; i < HISTORY_SIZE; ++i) {
      phase_history[i] += 360;
    }
  }
  else if (phase_diff < -170 && phase_history[1] > 165) {
    for (int i = 1; i < HISTORY_SIZE; ++i) {
      phase_history[i] -= 360;
    }
  }
  float k = calc_slope_scan_f32(); // 相位变化率

  sniprintf(msg, sizeof(msg), "%d,%d\n", (int)(phase_diff*1000), (int)(k*1000));
  HAL_UART_Transmit(&huart1, (uint8_t *)msg, strlen(msg), HAL_MAX_DELAY);

  if (k > 0.2) {
    freq_correction -= 0.0003 * freq / 10.f;
  }
  else if (k < -0.2) {
    freq_correction += 0.0003 * freq / 10.f;
    // test git
  }


  int mode;
  switch (signal1_waveform_type) {
    case 0: mode = SIN_WAVE; break;
    case 1: mode = TRI_WAVE; break;
    case 2: mode = SQU_WAVE; break;
  }

  const float p = 1000.026f;
  static int correction_wait = 0;
  if (freq * p != last_freq || mode != last_mode || (fabs(freq_correction - last_freq_correction)>0.0005 && correction_wait++ > 10)) {
    AD9833_WaveSeting(freq * p + freq_correction,0,mode,0);
    last_freq = freq * p;
    last_mode = mode;
    last_freq_correction = freq_correction;
    correction_wait = 0;
  }


  // calculate the amplitude of the first signal
  const float fft_scaling_factor = 2.0f / ADC_BUFFER_SIZE * 2.0f / 1.5f;
  const float amplitude_ratio[] = {1.0f, 1.2237f, 0.7754f}; // sine, ramp, square
  float32_t amplitude1 = 0;
  const int band = 5;
  for (int i = -band; i <= band; ++i) {
    amplitude1 += pow(magnitude_output[peak_index + i] * fft_scaling_factor, 2);
  }
  amplitude1 = sqrt(amplitude1) * amplitude_ratio[signal1_waveform_type] * 8.0f;
  // sniprintf(msg, sizeof(msg), "A1:%s V", Float2String(amplitude1)); // 3.3V ref
  // OLED_PrintString(0, 32, msg, &font16x16, OLED_COLOR_NORMAL);


  // eliminate the first signal
  const float first_harmonic_ratio[] = {1000000.0f, 9.0f, 3.0f}; // sine, ramp, square
  const float second_harmonic_ratio[] = {1000000.0f, 25.0f, 5.0f}; // sine, ramp, square
  for (int i = -band; i <= band; ++i) {
    // eliminate the first harmonic
    magnitude_output[peak_index * 3 + i] -= magnitude_output[peak_index] / first_harmonic_ratio[signal1_waveform_type];
    if (magnitude_output[peak_index * 3 + i] < 0) magnitude_output[peak_index * 3 + i] = 0;
    // eliminate the second harmonic
    magnitude_output[peak_index * 5 + i] -= magnitude_output[peak_index] / second_harmonic_ratio[signal1_waveform_type];
    if (magnitude_output[peak_index * 5 + i] < 0) magnitude_output[peak_index * 5 + i] = 0;
  }
  // eliminate the fundamental
  for (int i = -band; i <= band; ++i) {
    magnitude_output[peak_index + i] = 0;
  }


  // find the second signal
  // find the peak frequency
  uint32_t peak_index2;
  float32_t peak_value2;
  arm_max_f32(magnitude_output + 1, ADC_BUFFER_SIZE / 2 - 1, &peak_value2, &peak_index2);
  peak_index2 += 1; // because we skipped the DC component
  if (peak_index2 < 160) {
    sniprintf(msg, sizeof(msg), "F2:%s kHz", Float2String(peak_index2 * BASE_FREQ_LOW));
  }else {
    sniprintf(msg, sizeof(msg), "F2:%s kHz", Float2String(peak_index2 * BASE_FREQ_HIGH));
  }
  OLED_PrintString(0, 16, msg, &font16x16, OLED_COLOR_NORMAL);

  // detect waveform type
  int signal2_waveform_type = detect_wareform_type(peak_index2);
  if (signal2_waveform_type == 0) {
    sniprintf(msg, sizeof(msg), "Sine");
  } else if (signal2_waveform_type == 1) {
    sniprintf(msg, sizeof(msg), "Ramp");
  } else {
    sniprintf(msg, sizeof(msg), "Square");
  }
  OLED_PrintString(80, 16, msg, &font16x16, OLED_COLOR_NORMAL);

  // calculate the amplitude of the second signal
  float32_t amplitude2 = 0;
  for (int i = -band; i <= band; ++i) {
    amplitude2 += pow(magnitude_output[peak_index2 + i], 2);
  }
  amplitude2 = sqrt(amplitude2) * amplitude_ratio[signal2_waveform_type] / 65536.0f * 3.3f;
  sniprintf(msg, sizeof(msg), "A2:%s V", Float2String(amplitude2)); // 3.3V ref
  // OLED_PrintString(64, 32, msg, &font16x16, OLED_COLOR_NORMAL);

}


int detect_wareform_type(uint32_t base_idx) {
  // calculate fundamental energy
  float32_t fund_energy = 0;
  const int band = 6;
  for (int i = -band; i <= band; ++i) {
    fund_energy += pow(magnitude_output[base_idx + i], 2);
  }
  fund_energy = sqrt(fund_energy);
  // find the first harmonic
  uint32_t first_harmonic_index = base_idx * 3;
  float first_harmonic_value = magnitude_output[first_harmonic_index];
  for (int i = first_harmonic_index - 10; i <= first_harmonic_index + 10; ++i) {
    if (magnitude_output[i] > first_harmonic_value) {
      first_harmonic_value = magnitude_output[i];
      first_harmonic_index = i;
    }
  }
  // calculate first harmonic energy
  float32_t first_harmonic_energy = 0;
  for (int i = -band; i <= band; ++i) {
    first_harmonic_energy += pow(magnitude_output[first_harmonic_index + i], 2);
  }
  first_harmonic_energy = sqrt(first_harmonic_energy);
  // predict waveform type
  int waveform_type = 0; // 0: sine, 1: ramp, 2: square
  float DIV = fund_energy / first_harmonic_energy;
  global_DIV = DIV;
  if (DIV < 5 && DIV > 1) {
    waveform_type = 2;
  } else if (DIV < 16 && DIV > 6) {
    waveform_type = 1;
  } else {
    waveform_type = 0;
  }
  return waveform_type;
}


float32_t phase_fft[4096 * 2];
float calculate_phase(int offset, int fft_size, const arm_cfft_instance_f32 * S) {
  float sig_avg = 0;
  for (int i = 0; i < fft_size; ++i) {
    sig_avg += (float)adc_data_out[i*2 + offset];
  }
  sig_avg /= (float)fft_size;
  for (int i = 0; i < fft_size; ++i) {
    phase_fft[i*2] = (0.54 - 0.46 * arm_cos_f32(2 * M_PI * i / (fft_size - 1))) * ((float32_t)adc_data_out[i*2 + offset] - sig_avg) / 65535.0f;
    phase_fft[i*2 + 1] = 0; // imaginary part
  }
  arm_cfft_f32(S, phase_fft, 0, 1);
  uint32_t peak_index = 0;
  float32_t peak_value = 0;
  // for (int i = 0; i < fft_size; ++i) {
    // char msg[20];
    // sprintf(msg, "%d\n" , (int)(sqrt(phase_fft[i*2] * phase_fft[i*2] + phase_fft[i*2 + 1] * phase_fft[i*2 + 1])*1000));
    // HAL_UART_Transmit(&huart1, (uint8_t *)msg, strlen(msg), HAL_MAX_DELAY);
  // }
  for (int i = 1; i < fft_size / 2; ++i) {
    static float32_t mag;
    mag = sqrt(phase_fft[i*2] * phase_fft[i*2] + phase_fft[i*2 + 1] * phase_fft[i*2 + 1]);
    if (mag > peak_value) {
      peak_value = mag;
      peak_index = i;
    }
  }
  float phase = atan2(phase_fft[peak_index*2 + 1], phase_fft[peak_index*2]) * 180.0f / M_PI;
  return phase;
}



















// void generate_dac_waveform(double freq, int type, float amplitude) {
//   if (type == 0) { // sine
//     for (int i = 0; i < DAC_BUFFER_SIZE; ++i) {
//       dac_output[i] = (uint16_t)((sin(2 * M_PI * i / ((double)DAC_BUFFER_SIZE / freq)) + 1.0f) / 2.0f * 4095);
//     }
//   } else if (type == 1) { // ramp
//     int period = ((double)DAC_BUFFER_SIZE) / freq;
//     for (int i = 0; i < (double)DAC_BUFFER_SIZE; ++i) {
//       int half_period = period / 2;
//       int phase = i % period;
//
//       if (phase < half_period) {
//         // 上升段
//         dac_output[i] = (uint16_t)(phase * (4095.0f / half_period));
//       } else {
//         // 下降段
//         dac_output[i] = (uint16_t)((period - phase) * (4095.0f / half_period));
//       }
//     }
//   } else { // square
//     for (int i = 0; i < DAC_BUFFER_SIZE; ++i) {
//       if (i % (int)((double)DAC_BUFFER_SIZE / freq) < ((double)DAC_BUFFER_SIZE / freq) / 2) {
//         dac_output[i] = 4095;
//       } else {
//         dac_output[i] = 0;
//       }
//     }
//   }
//   for (int i = 0; i < DAC_BUFFER_SIZE; ++i) {
//     dac_output[i] = (uint16_t)((double)dac_output[i] * (amplitude / 3.3));
//   }
// }

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
