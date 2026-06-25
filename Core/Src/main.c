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
#include "cmsis_os.h"
#include "can.h"
#include "dma.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include "board_LED.h"
#include "uart_printf.h"
#include "uart_sent.h"
#include "bsp_rc.h"
#include "remote_control.h"
#include "get_rc.h"
#include "bsp_can.h"
#include "CAN_receive.h"
#include "jy61p.h"
#include "pid.h"
#include "chassis_motor_control.h"
#include "gimbal_motor_control.h"
#include "shoot_control.h"
#include <math.h>
#include <stdbool.h>
#include "BMI088driver.h"
#include "IMU_DATA_GET.h"
#include "MahonyAHRS.h"
#include "auto_aim.h"


/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

const RC_ctrl_t *local_rc_ctrl ;//ң�������ݴ洢�ռ�
int16_t rc_ch0 ;
int16_t rc_ch1 ;
int16_t rc_ch2 ;
int16_t rc_ch3 ;
int16_t rc_ch4 ;
int16_t rc_s0 ;
int16_t rc_s1 ;
int16_t key_W ;
int16_t key_S ;
int16_t key_A ;
int16_t key_D ;
int16_t key_SHIFT ;
int16_t key_CTRL ;
int16_t key_Q ;
int16_t key_E ;
int16_t key_R ;
int16_t key_F ;
int16_t key_G ;
int16_t key_Z ;
int16_t key_X ;
int16_t key_C ;
int16_t key_V ;
int16_t key_B ;


int16_t mouse_vx ;
int16_t mouse_vy ;
int16_t mouse_press_l ;
int16_t mouse_press_r ;


int16_t rc_receive_state ;//ң����״̬ 0Ϊ���ߣ�1Ϊ����
uint32_t rc_receive_time ;//ң�������յ����ݵ�ʱ���



int16_t yaw_6020_state ;//6020���״̬ 0Ϊ����1Ϊ����

float gimbal_vx ;
float gimbal_vy ;

float chassis_vx ;
float chassis_vy ;
float chassis_vround ;

float yaw_angle_difference ;
float yaw_radian_difference ;

uint32_t gyro_time ;
int16_t gyro_state ;






//chassis
int16_t CHASSIS_3508_ID1_GIVEN_SPEED ;
int16_t CHASSIS_3508_ID1_GIVEN_CURRENT ;
int16_t CHASSIS_3508_ID1_VXY_COMPUTE_SPEED ;

int16_t CHASSIS_3508_ID2_GIVEN_SPEED ;
int16_t CHASSIS_3508_ID2_GIVEN_CURRENT ;
int16_t CHASSIS_3508_ID2_VXY_COMPUTE_SPEED ;

int16_t CHASSIS_3508_ID3_GIVEN_SPEED ;
int16_t CHASSIS_3508_ID3_GIVEN_CURRENT ;
int16_t CHASSIS_3508_ID3_VXY_COMPUTE_SPEED ;

int16_t CHASSIS_3508_ID4_GIVEN_SPEED ;
int16_t CHASSIS_3508_ID4_GIVEN_CURRENT ;
int16_t CHASSIS_3508_ID4_VXY_COMPUTE_SPEED ;

float CHASSIS_3508_ALL_COMPUTE_SPEED ;

float CHASSIS_FOLLOW_GIMBAL_GIVEN_SPEED ;

float beyond_power ;

int16_t chassis_power_state ;//0Ϊ������1Ϊ����������״̬


//gimbal_vx
float YAW_6020_ID2_GIVEN_SPEED ;
int16_t YAW_6020_ID2_GIVEN_CURRENT ;
float YAW_6020_ID2_GIVEN_ANGLE ;

float PITCH_3510_ID5_GIVEN_ANGLE ;
float PITCH_3510_ID5_GIVEN_SPEED ;
int16_t PITCH_3510_ID5_GIVEN_CURRENT ;
float pitch_motor_mean_speed ;
int16_t pitch_motor_speed_last_data [3] ;


//friction wheel
int16_t FRICTION_WHEEL_3510_ID1_GIVEN_SPEED ;
int16_t FRICTION_WHEEL_3510_ID1_GIVEN_CURRENT ;

int16_t FRICTION_WHEEL_3510_ID2_GIVEN_SPEED ;
int16_t FRICTION_WHEEL_3510_ID2_GIVEN_CURRENT ;

int16_t FRICTION_WHEEL_3510_ID3_GIVEN_SPEED ;
int16_t FRICTION_WHEEL_3510_ID3_GIVEN_CURRENT ;

int16_t FRICTION_WHEEL_3510_ID4_GIVEN_SPEED ;
int16_t FRICTION_WHEEL_3510_ID4_GIVEN_CURRENT ;

int16_t FRICTION_WHEEL_3510_ID5_GIVEN_SPEED ;
int16_t FRICTION_WHEEL_3510_ID5_GIVEN_CURRENT ;

int16_t FRICTION_WHEEL_3510_ID6_GIVEN_SPEED ;
int16_t FRICTION_WHEEL_3510_ID6_GIVEN_CURRENT ;



//shoot
float SHOOT_3510_ID7_GIVEN_SPEED ;
int16_t SHOOT_3510_ID7_GIVEN_CURRENT ;


//robot_information
int16_t robot_level = 1;
uint32_t robot_level_time ;

float robot_max_power ;

float send_out_all_speed ;

uint32_t servo_time ;
uint32_t servo_rc_time ;
uint32_t servo_state ;

float gyro[3];
float acce[3];
float temp;
float INS_quat[4] = {1.0f, 0.0f, 0.0f, 0.0f};
float INS_angle[3] = {0.0f, 0.0f, 0.0f};
float INS_degree[3] = {0.0f, 0.0f, 0.0f};


float pitch_speed_from_bmi088 ;
float yaw_speed_from_bmi088 ;
float roll_speed_from_bmi088 ;


float pitch_angle_from_bmi088 ;
float yaw_angle_from_bmi088 ;
float roll_angle_from_bmi088 ;

float pitch_radian_from_bmi088 ;
float yaw_radian_from_bmi088 ;
float roll_radian_from_bmi088 ;

bool reset_tracker ;


uint8_t tx_buffer[sizeof(auto_aim_tx_packet)];

float YAW_IMU_LAST_ECD ;
float YAW_IMU_LAPS ;
float YAW_IMU_ABSCISSA ;


float yaw_imu_preprocess ;

uint8_t uart1_receive_data ;//���ڵ�ǰ�����ֽ�


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
void MX_FREERTOS_Init(void);
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
  MX_DMA_Init();
  MX_USART6_UART_Init();
  MX_USART3_UART_Init();
  MX_CAN1_Init();
  MX_CAN2_Init();
  MX_USART1_UART_Init();
  MX_TIM1_Init();
  MX_TIM8_Init();
  MX_SPI1_Init();
  /* USER CODE BEGIN 2 */

    HAL_TIM_Base_Start(&htim1);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
    HAL_TIM_Base_Start(&htim8);
    HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_2);
    HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_3);

    HAL_UART_Receive_DMA(&huart1, &uart1_receive_data, 1);  //����2���������ж�

    remote_control_init();//ң������ʼ��

    local_rc_ctrl = get_remote_control_point();//ң������ʼ��

    can_filter_init();//canͨѶ��ʼ��
    BMI088_init();

    //���̵����ʼ��
    chassis_3508_id1_speed_pid_init();
    chassis_3508_id2_speed_pid_init();
    chassis_3508_id3_speed_pid_init();
    chassis_3508_id4_speed_pid_init();

    chassis_follow_gimbal_angle_pid_init();//���̸����ʼ��

    //��̨�����ʼ��
    yaw_speed_pid_init();//yaw�ٶȻ�pid��ʼ��
    yaw_angle_pid_init();//yawλ�û�pid��ʼ��
    pitch_speed_from_bmi88_pid_init();//pitch�ٶȻ�pid��ʼ��
    pitch_angle_pid_init();//pitch�ǶȻ�pid��ʼ��

    //Ħ���ֵ����ʼ��
    friction_wheel_3510_id1_speed_pid_init();//Ħ����id1�ٶȻ���ʼ��
    friction_wheel_3510_id2_speed_pid_init();//Ħ����id2�ٶȻ���ʼ��
    friction_wheel_3510_id3_speed_pid_init();//Ħ����id3�ٶȻ���ʼ��
    friction_wheel_3510_id4_speed_pid_init();//Ħ����id4�ٶȻ���ʼ��
    friction_wheel_3510_id5_speed_pid_init();//Ħ����id5�ٶȻ���ʼ��
    friction_wheel_3510_id6_speed_pid_init();//Ħ����id6�ٶȻ���ʼ��

    //�����̵����ʼ��
    shoot_2006_id3_speed_pid_init();//������id3�ٶȻ���ʼ��


  /* USER CODE END 2 */

  /* Call init function for freertos objects (in cmsis_os2.c) */
  MX_FREERTOS_Init();

  /* Start scheduler */
  osKernelStart();

  /* We should never get here as control is now taken by the scheduler */

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
  RCC_OscInitStruct.PLL.PLLM = 6;
  RCC_OscInitStruct.PLL.PLLN = 168;
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
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM7 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM7) {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */

  /* USER CODE END Callback 1 */
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
