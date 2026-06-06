/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : 智能小车电控主程序 (包含模块化测试)
 ******************************************************************************
 */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "gpio.h"
#include "tim.h"
#include "usart.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "alert.h"
#include "app.h"
#include "chassis.h"
#include "debug.h"
#include "motor.h"
#include "openmv.h"
#include "pid.h"
#include "sensor.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/**
 * 🛠️ 测试模式选择宏定义 (极其重要)
 * 0: 【比赛模式】运行 app.c 的完整状态机
 * 1: 【底盘测试】闭环定速直行测试 (验证速度 PID)
 * 2: 【循迹测试】开启 LUT 查表循迹 (验证巡线平滑度)
 * 3: 【声光测试】循环播放声光提示
 * 4: 【VOFA测试】通过串口发送超声波和 OpenMV 数据到电脑查看
 */
#define TEST_MODE 0
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
    MX_TIM4_Init();
    MX_USART1_UART_Init();
    MX_TIM1_Init();
    MX_TIM2_Init();
    MX_TIM3_Init();
    /* USER CODE BEGIN 2 */

    // 1. 启动所有 PWM 输出 (电机)
    HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_2);
    HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_3);
    HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_4);

    // 2. 启动硬件编码器计数
    HAL_TIM_Encoder_Start(&htim2, TIM_CHANNEL_ALL);
    HAL_TIM_Encoder_Start(&htim3, TIM_CHANNEL_ALL);

    // 3. 各模块软件初始化
    Chassis_Init();
    OpenMV_Init();
    App_Init();

    // 开机短鸣提示初始化完成
    HAL_GPIO_WritePin(BEEP_GPIO_Port, BEEP_Pin, GPIO_PIN_SET);
    HAL_Delay(200);
    HAL_GPIO_WritePin(BEEP_GPIO_Port, BEEP_Pin, GPIO_PIN_RESET);
    HAL_Delay(1000); // 留出一点放置小车的时间

    // 4. 开启 TIM1 10ms 节拍器中断 (系统心跳)
    HAL_TIM_Base_Start_IT(&htim1);

    /* USER CODE END 2 */

    /* Infinite loop */
    /* USER CODE BEGIN WHILE */
    while (1)
    {
#if (TEST_MODE == 0)
        // 【正常比赛模式】
        App_Run();

#elif (TEST_MODE == 1)
        // 【闭环定速测试】: 屏蔽循迹，强制 0.35m/s 直行
        extern uint8_t enable_line_tracking;
        enable_line_tracking = 0;
        Chassis_SetPhysicalSpeed(0.35f, 0.35f);
        HAL_Delay(100);

#elif (TEST_MODE == 2)
        // 【循迹测试】: 屏蔽 APP 逻辑，单纯让底盘按黑线跑
        // 实际上什么都不用写，因为 10ms 中断里已经在跑 Chassis_Update() 了
        HAL_Delay(100);

#elif (TEST_MODE == 3)
        // 【声光测试】: 每隔5秒播放一次寻找目标声光
        Alert_Start(ALERT_TARGET_FOUND);
        while (!Alert_Process())
            ; // 阻塞等待播放完成
        HAL_Delay(5000);

#elif (TEST_MODE == 4)
        // 【VOFA+ 传感器数据检查测试】
        float vofa_debug[4];
        vofa_debug[0] = Hcsr04GetLength(); // 通道0: 超声波距离 (m)
        vofa_debug[1] = (float)Get_XunJi_State(); // 通道1: 灰度状态 (0~31)
        vofa_debug[2] = (float)openmv_found_target; // 通道2: MV 是否看到目标 (0/1)
        vofa_debug[3] = (float)openmv_x_error; // 通道3: MV 视觉偏差
        JustFloat_Send(vofa_debug, 4);
        HAL_Delay(50); // 50ms 传一次给上位机
#endif
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
    RCC_OscInitTypeDef RCC_OscInitStruct = { 0 };
    RCC_ClkInitTypeDef RCC_ClkInitStruct = { 0 };

    /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
    RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
    RCC_OscInitStruct.HSEState = RCC_HSE_ON;
    RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
    RCC_OscInitStruct.HSIState = RCC_HSI_ON;
    RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
    RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
    RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
    if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
    {
        Error_Handler();
    }

    /** Initializes the CPU, AHB and APB buses clocks
  */
    RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
        | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
    RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
    RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
    RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

    if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
    {
        Error_Handler();
    }
}

/* USER CODE BEGIN 4 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef* htim)
{
    if (htim->Instance == TIM1) // 确保是 TIM1 溢出 (10ms)
    {
        // 底盘运动学核心更新 (包括循迹 LUT 和 PID 计算)
        Chassis_Update();
    }
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
#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t* file, uint32_t line)
{
    /* USER CODE BEGIN 6 */
    /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
    /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
