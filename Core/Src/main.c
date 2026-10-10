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
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <math.h>
#include "ICM42688.h"
#include "serialcmd.h"
#include "camera.h"
#include "tracking.h"
#include "chassis.h"
#include "encoder.h"
#include "balance.h"
#include "oled.h"
#include "speed_ctrl.h"

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
enum {
    STATE_IDLE = 0,
    STATE_RUNNING,
    STATE_BRAKING,
    STATE_DONE,
} sys_state = STATE_IDLE;

static uint32_t state_start_ms;
static uint32_t last_key_ms;
static uint32_t last_lf_ms;
static uint32_t last_dbg_ms;
static uint8_t  uart4_rx_byte;
static uint8_t  usart3_rx_byte;
volatile uint8_t bal_20ms_flag;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MPU_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
int _write(int fd,char *ptr,int len)
{
  HAL_UART_Transmit(&huart4,(uint8_t *)ptr,len,HAL_MAX_DELAY);
  return len;
}

void calculate(float *gx_offset, float *gy_offset, float *gz_offset)
{
 float sum[3] = {0};
  ICM42688_Data rawdata;

  for (int i = 0; i < 1000; i++)
  {
      ICM42688_ReadAct(&rawdata);
      if (i < 50) continue;
      sum[0] += rawdata.gyro_x;
      sum[1] += rawdata.gyro_y;
      sum[2] += rawdata.gyro_z;
      HAL_Delay(5);
  }
  *gx_offset = sum[0] / 950.0f;
  *gy_offset = sum[1] / 950.0f;
  *gz_offset = sum[2] / 950.0f;
  printf("Gyro offset: %.4f, %.4f, %.4f dps\r\n",
         *gx_offset, *gy_offset, *gz_offset);
}

/* 题4 B点距离 A→B (mm)，根据实际赛道标定 */
#define B_POINT_DISTANCE_MM  1500.0f

/* 过线检测: 4路传感器全黑 */
static inline uint8_t line_crossed(void)
{
    return (TRACK_X1_READ() == 0 && TRACK_X2_READ() == 0 &&
            TRACK_X3_READ() == 0 && TRACK_X4_READ() == 0);
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

  /* MPU Configuration--------------------------------------------------------*/
MPU_Config();

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
  MX_UART4_Init();
  MX_USART1_UART_Init();
  MX_USART3_UART_Init();
  MX_I2C2_Init();
  MX_TIM1_Init();
  MX_TIM3_Init();
  MX_TIM2_Init();
  MX_TIM6_Init();
  HAL_TIM_Base_Start_IT(&htim6);
  /* USER CODE BEGIN 2 */
  TRACK_IR4_Init();
  Chassis_Init();
  Enc_Init();
  Spd_Init();
  Balance_Init(&huart1);
  OLED_Init();
  OLED_Clear();
  OLED_Printf(0, 0, "Task: %d  [^v]", g_task_mode);
  OLED_ShowString(0, 4, "[SEL] Start");
  printf("OK — Task mode: %d\r\n", g_task_mode);
  SerialCmd_Init();
  HAL_UART_Receive_IT(&huart4, &uart4_rx_byte, 1);
  Camera_Init();
  HAL_UART_Receive_IT(&huart3, &usart3_rx_byte, 1);

  /* IMU 初始化 + 陀螺仪零偏校准 */
  if (ICM42688_Init() == 0) {
      float gx_off, gy_off, gz_off;
      printf("ICM42688 OK\r\n");
      calculate(&gx_off, &gy_off, &gz_off);
  } else {
      printf("ICM42688 FAIL\r\n");
  }

  last_key_ms = HAL_GetTick();
  last_lf_ms  = HAL_GetTick();
  last_dbg_ms = HAL_GetTick();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
      uint32_t now = HAL_GetTick();

      // 串口指令处理 (每次循环) 
      SerialCmd_Update(NULL, NULL);

      // 四键扫描 (20ms + 边沿检测 + 非阻塞消抖)
      if (now - last_key_ms >= 20) {
          static uint8_t  k_prev[4]  = {0};     
          static uint8_t  k_edge[4]  = {0};     
          static uint32_t k_lockout = 0;        

          uint8_t k_raw[4];
          k_raw[0] = (HAL_GPIO_ReadPin(up_GPIO_Port,   up_Pin)   == GPIO_PIN_RESET);
          k_raw[1] = (HAL_GPIO_ReadPin(down_GPIO_Port, down_Pin) == GPIO_PIN_RESET);
          k_raw[2] = (HAL_GPIO_ReadPin(mid_GPIO_Port,  mid_Pin)  == GPIO_PIN_RESET);
          k_raw[3] = (HAL_GPIO_ReadPin(set_GPIO_Port,  set_Pin)  == GPIO_PIN_RESET);
          last_key_ms = now;

          //边沿检测
          for (int i = 0; i < 4; i++) {
              if (k_raw[i] && !k_prev[i]) k_edge[i] = 1;
              k_prev[i] = k_raw[i];
          }

          // (防止连发) 
          if (now >= k_lockout) {
              if (sys_state == STATE_IDLE) {
                  if (k_edge[0] && g_task_mode < 6) {  // UP
                      g_task_mode++;
                      OLED_Printf(0, 0, "Task: %d  [^v]", g_task_mode);
                      printf("Task mode: %d\r\n", g_task_mode);
                      k_lockout = now + 300;
                  }
                  if (k_edge[1] && g_task_mode > 2) {  // DOWN 
                      g_task_mode--;
                      OLED_Printf(0, 0, "Task: %d  [^v]", g_task_mode);
                      printf("Task mode: %d\r\n", g_task_mode);
                      k_lockout = now + 300;
                  }
                  if (k_edge[2]) {  // SEL 
                      g_key_pressed = 1;
                      k_lockout = now + 300;
                  }
              }
              if (k_edge[3] && (sys_state == STATE_RUNNING || sys_state == STATE_BRAKING)) {
                  // STOP 紧急停车
                  sys_state = STATE_DONE;
                  g_lap_time_ms = now - state_start_ms;
                  Chassis_Stop();
                  Balance_Enable(0);
                  printf("→ STOP (manual)\r\n");
                  OLED_Printf(0, 6, "STOPPED");
                  k_lockout = now + 500;
              }
          }

       
          for (int i = 0; i < 4; i++) k_edge[i] = 0;
      }

      uint8_t need_balance = (g_task_mode >= 3);  // 题3-6 需要控球 
      uint8_t need_racing  = (g_task_mode != 3);  // 题2,4,5,6 需要循迹

      /* ---- 摄像头 + 平衡控制 (TIM6 20ms) ---- */
      if (need_balance && bal_20ms_flag) {
          bal_20ms_flag = 0;

          static int16_t ball_pos = 0;
          static int16_t ball_prev = 0;
          static float   ball_vel = 0.0f;
          static uint32_t last_cam_ms = 0;
          static uint8_t  detected = 0;

          CameraData cam = Camera_GetData();
          if (cam.fresh) {
              detected = cam.detected;
              if (cam.detected) {
                  ball_pos = cam.dx;
                  if (last_cam_ms > 0) {
                      float dt_cam = (now - last_cam_ms) * 0.001f;
                      if (dt_cam > 0.0f && dt_cam < 0.5f) {
                          ball_vel = (float)(ball_pos - ball_prev) * 0.1f / dt_cam;
                      }
                  }
                  ball_prev = ball_pos;
                  last_cam_ms = now;
              }
          }

          if (detected) {
              Balance_SetBallVel(ball_vel);
              if (g_task_mode == 3) {
                  Balance_RunTrajectory(ball_pos, 0.02f);
              } else {
                  Balance_SetTarget(g_ball_target);
                  Balance_Update(ball_pos, 0.02f);
              }
          }
      }

      // 循迹控制 (10ms)
      if (need_racing && (sys_state == STATE_RUNNING || sys_state == STATE_BRAKING) &&
          now - last_lf_ms >= 10) {

          if (g_task_mode == 4) {
              float rL, rR;
              Chassis_LineRatio(&rL, &rR);
              Spd_SetRatio(rL, rR);
              Spd_Update();

              if (Spd_Arrived()) {
                  sys_state = STATE_DONE;
                  g_lap_time_ms = now - state_start_ms;
                  float final_dist = Enc_Dist();
                  OLED_Clear();
                  OLED_Printf(0, 0, "Task 4 DONE!");
                  OLED_Printf(0, 2, "T:%.1fs D:%.0fmm",
                             (double)g_lap_time_ms * 0.001, (double)final_dist);
                  printf("→ DONE! Time:%lums Dist:%.1fmm\r\n",
                         (unsigned long)g_lap_time_ms, (double)final_dist);
              }
          } else {
              Chassis_LineFollow();
              if (sys_state == STATE_BRAKING) {
                  if (Chassis_BrakeUpdate()) {
                      sys_state = STATE_DONE;
                      Balance_Enable(0);
                      uint32_t elapsed = now - state_start_ms;
                      g_lap_time_ms = elapsed;
                      OLED_Clear();
                      OLED_Printf(0, 0, "Task %d DONE!", g_task_mode);
                      OLED_Printf(0, 2, "Time: %.1fs", (double)elapsed * 0.001);
                      printf("→ DONE! Time: %lu ms\r\n", (unsigned long)elapsed);
                  }
              }
          }
          last_lf_ms = now;
      }

      //状态机跳转
      switch (sys_state) {

      case STATE_IDLE:
          Chassis_Stop();
          if (g_key_pressed) {
              g_key_pressed = 0;
              sys_state = STATE_RUNNING;
              state_start_ms = now;
              g_lap_time_ms = 0;

              if (need_balance) {
                  Balance_Enable(1);
                  Balance_SetTarget(g_ball_target);
              }
              if (g_task_mode == 4) {
                  Spd_Reset();
                  Spd_SetDuty(0.39f);   
                  Spd_AddDist(50.0f);  
                  Spd_SetStop(B_POINT_DISTANCE_MM);
              }
              OLED_Clear();
              OLED_Printf(0, 0, "Task %d RUNNING", g_task_mode);
              printf("→ RUN (mode %d)\r\n", g_task_mode);
          }
          break;

      case STATE_RUNNING:
          {
              uint32_t elapsed = now - state_start_ms;

              // 过线检测
              uint8_t crossed = (elapsed > 3000) ? line_crossed() : 0;

              uint8_t stop_flag = 0;

              //右两路(X3+X4)同时见黑 
              if (g_task_mode == 2 && elapsed > 7000 &&
                  TRACK_X3_READ() == 0 && TRACK_X4_READ() == 0) {
                  Chassis_BrakeHard();
                  HAL_Delay(200);   // 保持硬刹
                  Chassis_Stop();
                  sys_state = STATE_DONE;
                  Balance_Enable(0);
                  g_lap_time_ms = elapsed;
                  OLED_Clear();
                  OLED_Printf(0, 0, "Task 2 DONE!");
                  OLED_Printf(0, 2, "Time: %.1fs", (double)elapsed * 0.001);
                  printf("→ DONE! Time: %lu ms\r\n", (unsigned long)elapsed);
                  break;
              } else if (g_task_mode == 3) {
                  /* 题3: 轨迹完成后保持住 */
                  // 轨迹完成由 Balance_RunTrajectory 内部处理
              } else if ((g_task_mode == 5 || g_task_mode == 6) && crossed) {
                  /* 题5/6: 完整一圈后过A线停止 */
                  stop_flag = 1;
              }

              if (stop_flag) {
                  sys_state = STATE_BRAKING;
                  Chassis_SoftBrake();
                  OLED_Printf(0, 6, "Braking...");
                  printf("→ BRAKING\r\n");
              }
          }
          break;

      case STATE_BRAKING:
          /* 刹车由循迹控制区的 Chassis_BrakeUpdate 驱动，完成后自动切到 DONE */
          break;

      case STATE_DONE:
          /* 停车保持 */
          Chassis_Stop();
          /* 题3/4/6 保持球在目标位置 */
          break;
      }

      //调试打印 + OLED 刷新 (200ms)
      if (now - last_dbg_ms >= 200) {
          last_dbg_ms = now;
          CameraData cam = Camera_PeekData();
          uint32_t disp_t;
          if (sys_state == STATE_RUNNING)
              disp_t = now - state_start_ms;
          else if (sys_state == STATE_DONE)
              disp_t = g_lap_time_ms;
          else
              disp_t = 0;
          // OLED 显示
          OLED_Printf(0, 2, "Mode:%d", g_task_mode);
          const char *state_str[] = {"IDLE","RUN ","BRAK","DONE"};
          OLED_Printf(48, 2, "%s", state_str[sys_state]);
          OLED_Printf(0, 4, "T:%.1fs", (double)disp_t * 0.001);
          if (g_task_mode == 4 && sys_state == STATE_RUNNING) {
              OLED_Printf(0, 6, "R:%.0fmm", Spd_Remain());
          } else {
              OLED_Printf(0, 6, "%s X:%d", cam.detected ? "Y" : "N", cam.dx);
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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 5;
  RCC_OscInitStruct.PLL.PLLN = 192;
  RCC_OscInitStruct.PLL.PLLP = 2;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
  RCC_OscInitStruct.PLL.PLLRGE = RCC_PLL1VCIRANGE_2;
  RCC_OscInitStruct.PLL.PLLVCOSEL = RCC_PLL1VCOWIDE;
  RCC_OscInitStruct.PLL.PLLFRACN = 0;
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

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart == &huart4) {
        SerialCmd_RxByte(uart4_rx_byte);
        HAL_UART_Receive_IT(&huart4, &uart4_rx_byte, 1);
    } else if (huart == &huart3) {
        Camera_RxByte(usart3_rx_byte);
        HAL_UART_Receive_IT(&huart3, &usart3_rx_byte, 1);
    }
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    if (huart == &huart3) {
        uint32_t err = HAL_UART_GetError(huart);
        if (err & (HAL_UART_ERROR_ORE | HAL_UART_ERROR_NE | HAL_UART_ERROR_FE)) {
            __HAL_UART_CLEAR_FLAG(huart,
                UART_CLEAR_OREF | UART_CLEAR_NEF | UART_CLEAR_FEF);
            HAL_UART_Receive_IT(&huart3, &usart3_rx_byte, 1);
        }
    }
}

/* USER CODE END 4 */

 /* MPU Configuration */

void MPU_Config(void)
{
  MPU_Region_InitTypeDef MPU_InitStruct = {0};

  /* Disables the MPU */
  HAL_MPU_Disable();

  /** Initializes and configures the Region and the memory to be protected
  */
  MPU_InitStruct.Enable = MPU_REGION_ENABLE;
  MPU_InitStruct.Number = MPU_REGION_NUMBER0;
  MPU_InitStruct.BaseAddress = 0x0;
  MPU_InitStruct.Size = MPU_REGION_SIZE_4GB;
  MPU_InitStruct.SubRegionDisable = 0x87;
  MPU_InitStruct.TypeExtField = MPU_TEX_LEVEL0;
  MPU_InitStruct.AccessPermission = MPU_REGION_NO_ACCESS;
  MPU_InitStruct.DisableExec = MPU_INSTRUCTION_ACCESS_DISABLE;
  MPU_InitStruct.IsShareable = MPU_ACCESS_SHAREABLE;
  MPU_InitStruct.IsCacheable = MPU_ACCESS_NOT_CACHEABLE;
  MPU_InitStruct.IsBufferable = MPU_ACCESS_NOT_BUFFERABLE;

  HAL_MPU_ConfigRegion(&MPU_InitStruct);
  /* Enables the MPU */
  HAL_MPU_Enable(MPU_PRIVILEGED_DEFAULT);

}

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
