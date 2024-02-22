/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
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
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "key.h"
#include "LIN.h"
#include "12864.h"
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
uint8_t RevByte = 0;
uint8_t pRevByte = 0;
uint8_t RxFlag = 0;
uint8_t RxLength = 0;
uint8_t selectMode = 1;
uint8_t timer = 0;
uint8_t powerOn = DISABLE;
uint8_t patternCycleNumber = 0;
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
    //解决断电后重新上电，程序运行异常的问题
    //原因：在程序刚开始时加上一个延时，因为外设上电时间不够，所以加个延时，等待外设上电，再进行初始化
    HAL_Delay(700);
  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_USART2_UART_Init();
  MX_TIM6_Init();
  /* USER CODE BEGIN 2 */
    //一定要先清除串口空闲中断，然后在打开串口空闲中断，因为串口初始化完成后会自动将IDLE置位，
    // 导致还没有接受数据就进入到中断里面去了，所以打开IDLE之前，先把它清楚掉
    //清除串口空闲中断
    __HAL_UART_CLEAR_IDLEFLAG(&huart2);
    //打开串口空闲中断
    __HAL_UART_ENABLE_IT(&huart2,UART_IT_IDLE);
    //开启中断接收
    Util_Receive_IT(&huart2);
    //使能更新中断
    HAL_TIM_Base_Start_IT(&htim6);
    //启动定时器
    HAL_TIM_Base_Start(&htim6);
    //使能系统运行指示灯
    HAL_GPIO_WritePin(LED_System_GPIO_Port,LED_System_Pin,GPIO_PIN_SET);
    //使能TJA1028LIN芯片的EN
    HAL_GPIO_WritePin(TJA1028_EN_GPIO_Port,TJA1028_EN_Pin,GPIO_PIN_SET);
    //使能TJA1028LIN芯片的RSTN
    HAL_GPIO_WritePin(TJA1028_RSTN_GPIO_Port,TJA1028_RSTN_Pin,GPIO_PIN_SET);
    LCDInit();
    //初始化数据
    timer = 0;
    InfiniteLoop = 1;
    currentStepSize = 480;
    currentCycleCount = 61000;
    patternCycleNumber = 0;
    Data_To_LIN(currentStepSize,currentCycleCount,0);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

      //显示屏刷新
      if (powerOn && patternCycleNumber < MODE_CYCLE_TIMES_SET_VALUE)
      {
          DisplayChineseCharacter(FIRST_LINE + 3,operation,sizeof(operation) / sizeof(uint8_t));
      }
      else
      {
          DisplayChineseCharacter(FIRST_LINE + 3,stop,sizeof(stop) / sizeof(uint8_t));
      }
      DisplayCharacter(SECOND_LINE + 3,timer,3);
      DisplayCharacter(FOURTH_LINE + 5,selectMode,1);
      DisplayCharacter(FIRST_LINE + 7,patternCycleNumber,2);
      //检测开始按钮（模式1）
      if (General_Key_Scan(Start_Key_GPIO_Port,Start_Key_Pin))
      {
          selectMode = 1;
          powerOn = ENABLE;
          timer = 0;
          patternCycleNumber = 0;
          //定时器计数清零
          __HAL_TIM_SET_COUNTER(&htim6,0);
          DisplayChineseCharacter(THIRD_LINE,"                ", strlen("                "));
      }
      //检测结束按钮（模式2）
      if (General_Key_Scan(Finished_Key_GPIO_Port,Finished_Key_Pin))
      {
          selectMode = 2;
          powerOn = ENABLE;
          timer = 0;
          patternCycleNumber = 0;
          //定时器计数清零
          __HAL_TIM_SET_COUNTER(&htim6,0);
          DisplayChineseCharacter(THIRD_LINE,"                ", strlen("                "));
      }
      //检测步数加按钮（模式3）
      if (General_Key_Scan(Step_Add_GPIO_Port,Step_Add_Pin))
      {
          selectMode = 3;
          powerOn = ENABLE;
          timer = 0;
          patternCycleNumber = 0;
          //定时器计数清零
          __HAL_TIM_SET_COUNTER(&htim6,0);
          DisplayChineseCharacter(THIRD_LINE,"                ", strlen("                "));
      }
      //循环发送数据
      if(powerOn && patternCycleNumber < MODE_CYCLE_TIMES_SET_VALUE)
      {
          Send_LIN_Data();
      }
      if (RxFlag)
      {
          LIN_Data_Process(RxLength);
          RxFlag = 0;
      }
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
  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSIDiv = RCC_HSI_DIV1;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
/**
* 重写接收中断函数
*/
void Util_Receive_IT(UART_HandleTypeDef *huart)
{
    if(huart == &huart2)
    {
        if(HAL_UART_Receive_IT(huart, &RevByte, 1) != HAL_OK)
        {
            Error_Handler();
        }
    }
}

/**
 * 接收中断回调函数，每次接收一个字节
 * huart2每次只接受1个字节的数据，通过串口空闲中断来判断数据是否传输结束
 *
 * @brief Rx Transfer completed callback.
 * @param huart UART handle.
 * @retval None
 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    //LIN数据
    if(huart == &huart2)
    {
        pLINRxBuff[pRevByte] = RevByte;
        pRevByte++;
    }
    Util_Receive_IT(huart);
}

//串口空闲中断
void UART_IDLECallBack(UART_HandleTypeDef *huart)
{
    if(huart == &huart2)
    {
        if((__HAL_UART_GET_FLAG(huart,UART_FLAG_IDLE) != RESET))
        {
            __HAL_UART_CLEAR_IDLEFLAG(&huart2);//清除标志位
            RxFlag = 1;
            RxLength = pRevByte;
            pRevByte = 0;
        }
    }
}

/**
 * 重写UART错误中断处理程序，重新开启USART中断
 *
 * @brief UART error callback.
 * @param huart UART handle.
 * @retval None
 */
void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    /* Prevent unused argument(s) compilation warning */
    //解决串口溢出，导致不断进入串口中断函数，使MCU过载的问题
    if(HAL_UART_GetError(huart) & HAL_UART_ERROR_ORE)
    {
        //清除ORE标志位
        __HAL_UART_FLUSH_DRREGISTER(huart);
        Util_Receive_IT(huart);
        huart->ErrorCode = HAL_UART_ERROR_NONE;
    }
}

/**
 * 定时器中断回调函数-1分钟中断一次
 * @param htim
 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if(htim->Instance == htim6.Instance && patternCycleNumber < MODE_CYCLE_TIMES_SET_VALUE)
    {
        timer++;
        if(selectMode == 1 && timer == 60)
        {
            stateReversal(&powerOn);
            timer = 0;
        }
        if (selectMode == 2 && powerOn == ENABLE && timer == 1){
            powerOn = DISABLE;
            timer = 0;
        }
        if (selectMode == 2 && powerOn == DISABLE && timer == 240){
            powerOn = ENABLE;
            timer = 0;
        }
        if(selectMode == 3 && powerOn == ENABLE && timer == 350){
            powerOn = DISABLE;
            timer = 0;
        }
        if(selectMode == 3 && powerOn == DISABLE && timer == 130){
            powerOn = ENABLE;
            timer = 0;
            patternCycleNumber++;
        }
    }
}

/**
 * 不推荐在中断里使用延时函数
 * 在实际应用中发现，在STM32的中断里使用延时函数HAL_Delay(Delay)容易出现问题（与SysTick中断的优先级），故采用while(t--)代替延时函数
 * 12864显示屏的写操作中使用了HAL_Delay(Delay)函数，导致程序卡在延时函数无法跳出来
 * @param t_ms
 */
void ms_Delay(uint16_t t_ms)
{
    uint32_t t = t_ms * 3127;
    while (t--);
}

/**
 * 状态反转
 * @param state
 */
void stateReversal(uint8_t *state)
{
    if (*state == DISABLE)
    {
        *state = ENABLE;
    }
    else
    {
        *state = DISABLE;
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
